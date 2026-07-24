"""DAQ IOC: keysight counter family behind gated-acquire PVs."""
from __future__ import annotations

import argparse
import asyncio
import functools

from pystxmcontrol.iocs import configure_ioc_logging, require_caproto

require_caproto()

from caproto import ChannelType  # noqa: E402
from caproto.server import PVGroup, pvproperty, run  # noqa: E402

from pystxmcontrol.iocs.config import read_slice  # noqa: E402

MAX_LINE = 16384

LINE_STATES = ["IDLE", "ARMED", "ACQUIRING", "ERROR"]


def _enum_str(prop) -> str:
    """Enum pvproperty value normalized to its string label (caproto stores
    either the index or the label depending on version/path)."""
    v = prop.value
    return v if isinstance(v, str) else prop.enum_strings[int(v)]


class DaqGroup(PVGroup):
    dwell = pvproperty(value=1.0, name=":DWELL", precision=3, doc="dwell per point, ms")
    mode = pvproperty(value=0, enum_strings=("point", "line"),
                      dtype=ChannelType.ENUM, name=":MODE")
    acquire = pvproperty(value=0, name=":ACQUIRE",
                         doc="write 1: acquire one point; put completes when done")
    counts = pvproperty(value=0.0, name=":COUNTS", read_only=True)
    counts_wf = pvproperty(value=[0.0], name=":COUNTS:WF", read_only=True,
                           max_length=MAX_LINE)
    rate = pvproperty(value=0.0, name=":RATE", read_only=True, doc="counts/s")
    line_npoints = pvproperty(value=10, name=":LINE:NPOINTS",
                              doc="points in the next fly line")
    line_trigger = pvproperty(value="EXT", enum_strings=["EXT", "IMM", "BUS"],
                              dtype=ChannelType.ENUM, name=":LINE:TRIGGER")
    line_arm = pvproperty(value=0, name=":LINE:ARM",
                          doc="write 1: arm a line; put completes when armed")
    line_abort = pvproperty(value=0, name=":LINE:ABORT")
    line_index = pvproperty(value=0, name=":LINE:INDEX", read_only=True,
                            doc="increments AFTER :COUNTS:WF holds the line")
    line_status = pvproperty(value="IDLE", enum_strings=list(LINE_STATES),
                             dtype=ChannelType.ENUM, name=":LINE:STATUS",
                             read_only=True)
    line_error = pvproperty(value="", name=":LINE:ERROR", read_only=True,
                            dtype=ChannelType.CHAR, max_length=256,
                            report_as_string=True)

    def __init__(self, prefix, *, daq, daq_entry, **kwargs):
        super().__init__(prefix, **kwargs)
        self._daq = daq
        self._entry = daq_entry
        self._started = False
        self._line_task = None
        self._line_starting = False

    async def _ensure_started(self):
        if not self._started:
            loop = asyncio.get_running_loop()
            await loop.run_in_executor(None, self._daq.start)
            self._started = True

    @mode.putter
    async def mode(self, instance, value):
        # caproto delivers the enum STRING to the putter (ChannelEnum.verify_value
        # maps a valid index write to its enum string first). Anything else --
        # an unknown string from a STRING-dtype write, or an out-of-range index
        # that verify_value passed through unmapped -- is rejected here.
        if value not in instance.enum_strings:
            raise ValueError(f"invalid MODE {value!r}; expected one of "
                             f"{instance.enum_strings}")
        return value

    def _line_busy(self) -> bool:
        return self._line_starting or (self._line_task is not None and not self._line_task.done())

    @line_arm.putter
    async def line_arm(self, instance, value):
        if not value:
            return 0
        if self._line_busy():
            # No-op + error PV. NEVER raise here: the caproto threading
            # client does not resolve put futures on ErrorResponse, so a
            # raised rejection would hang the caller until its timeout.
            await self.line_error.write("ARM rejected: line in progress")
            return 0
        n = int(self.line_npoints.value)
        dwell = float(self.dwell.value)
        problems = []
        if not (1 <= n <= MAX_LINE):
            problems.append(f"NPOINTS {n} outside 1..{MAX_LINE}")
        if dwell <= 0:
            problems.append(f"DWELL {dwell} must be > 0")
        if problems:
            await self.line_status.write("ERROR")
            await self.line_error.write("; ".join(problems)[:255])
            return 0
        # Set synchronous busy flag BEFORE any await to prevent race where two
        # concurrent ARM puts both pass _line_busy() check but both spawn tasks.
        self._line_starting = True
        try:
            await self._ensure_started()
            loop = asyncio.get_running_loop()
            trigger = _enum_str(self.line_trigger)
            if self._daq.simulation:
                # sim contract (matches the old in-process fly path): the sim
                # keysight generates count*samples poisson points after sleeping
                # dwell*count*samples.
                cfg = functools.partial(self._daq.config, dwell, count=n, samples=1)
            else:
                # hardware line-trigger contract: one trigger event, n samples.
                cfg = functools.partial(self._daq.config, dwell, count=1,
                                        samples=n, trigger=trigger)
            await loop.run_in_executor(None, cfg)
            if not self._daq.simulation:
                await loop.run_in_executor(None, self._daq.initLine)
            await self.line_error.write("")
            await self.line_status.write("ARMED")
            # Spawned AFTER arming succeeds; the ARM put's completion is the
            # client's cue that it may command its motor move.
            self._line_task = asyncio.create_task(self._acquire_line(n, dwell))
        finally:
            # Clear the starting flag now that the task is assigned or setup failed.
            self._line_starting = False
        return 0

    async def _acquire_line(self, n: int, dwell: float):
        # Watchdog: a crashed/absent client must not wedge the detector.
        watchdog = max(5.0, 4.0 * n * dwell / 1000.0 + 5.0)
        try:
            await self.line_status.write("ACQUIRING")
            line = await asyncio.wait_for(self._daq.getLine(), timeout=watchdog)
        except asyncio.TimeoutError:
            await self.line_status.write("ERROR")
            await self.line_error.write(
                f"line watchdog: no data within {watchdog:.1f}s; auto-disarmed")
            return
        except asyncio.CancelledError:
            await self.line_status.write("IDLE")
            await self.line_error.write("line aborted")
            raise
        except Exception as exc:
            # Catch any other exception (e.g., from getLine or write_line) and
            # surface it so the client is not left waiting on a wedged detector.
            await self.line_status.write("ERROR")
            await self.line_error.write(str(exc)[:255])
            return
        # ---- ordering contract: waveform FIRST, then INDEX ----
        try:
            await self.write_line(line)
            await self.line_index.write(self.line_index.value + 1)
        except Exception as exc:
            # Catch exceptions from write_line or index increment.
            await self.line_status.write("ERROR")
            await self.line_error.write(str(exc)[:255])
            return
        await self.line_status.write("IDLE")

    @line_abort.putter
    async def line_abort(self, instance, value):
        if value and self._line_busy():
            self._line_task.cancel()
            try:
                await self._line_task
            except asyncio.CancelledError:
                pass
            except Exception:  # noqa: BLE001 - abort is best-effort teardown
                pass
            await self.line_status.write("IDLE")
        return 0

    @acquire.putter
    async def acquire(self, instance, value):
        if not value:
            return 0
        if self._line_busy():
            await self.line_error.write("ACQUIRE rejected: line in progress")
            return 0
        await self._ensure_started()
        loop = asyncio.get_running_loop()
        dwell_ms = self.dwell.value
        await loop.run_in_executor(
            None, functools.partial(self._daq.config, dwell_ms, count=1, samples=1))
        data = await self._daq.getPoint()  # coroutine on OUR loop
        c = float(data[0])
        await self.counts.write(c)
        await self.rate.write(c / (dwell_ms / 1000.0) if dwell_ms else 0.0)
        return 0

    async def write_line(self, line) -> None:
        """Publish a fly-line waveform (called in-process by the fly loop)."""
        await self.counts_wf.write([float(x) for x in line])
        if len(line):
            await self.counts.write(float(line[-1]))


def build_pvdb_for_entry(entry: dict, prefix: str):
    import pystxmcontrol.drivers as drv
    cls = getattr(drv, entry["driver"])
    daq = cls(address=entry["address"], port=entry.get("port", 0),
              simulation=entry["simulation"])
    daq.meta.update(entry)  # UPDATE not replace: keeps default 'gate' key
    group = DaqGroup(prefix, daq=daq, daq_entry=entry)
    return dict(group.pvdb), group


def build_pvdb_from_slice(s: dict) -> dict:
    if s["kind"] != "daq":
        raise ValueError(f"daq_ioc requires kind=daq, got {s['kind']!r}")
    pvdb, _ = build_pvdb_for_entry(s["entry"], s["prefix"])
    return pvdb


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--slice", required=True)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)
    configure_ioc_logging()
    run(build_pvdb_from_slice(read_slice(args.slice)), log_pv_names=not args.quiet)


if __name__ == "__main__":
    main()
