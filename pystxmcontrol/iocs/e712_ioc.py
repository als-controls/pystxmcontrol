"""E712 fly IOC: motor records + FLY PVGroup with an IOC-side line loop.

Consistency contract: on each completed line, ALL data/pos waveforms are
written BEFORE :INDEX increments. Clients monitor :INDEX, then read waveforms.
"""
from __future__ import annotations

import argparse
import asyncio
import functools

import numpy as np

from pystxmcontrol.iocs import require_caproto

require_caproto()

from caproto import ChannelType  # noqa: E402
from caproto.server import PVGroup, pvproperty, run  # noqa: E402

from pystxmcontrol.iocs.config import read_slice  # noqa: E402
from pystxmcontrol.iocs.daq_ioc import MAX_LINE, build_pvdb_for_entry  # noqa: E402

STATES = ["IDLE", "ARMED", "FLYING", "ERROR"]


def _enum_index(prop) -> int:
    """Normalize a caproto enum pvproperty's current value to its index.

    Different caproto versions store the ChannelData value as either the
    int index or the string label -- accept both.
    """
    v = prop.value
    if isinstance(v, str):
        return prop.enum_strings.index(v)
    return int(v)


def _fly_group_class(axis_labels: list, daq_keys: list):
    """Build the FLY PVGroup class with per-DAQ DATA waveforms baked into the
    class namespace up front (not via setattr afterwards) -- caproto's
    PVGroupMeta collects pvproperty attributes at class-creation time, so any
    dynamic per-fleet PVs must already be present when the class object is
    created via ``type()``.
    """

    start = pvproperty(value=0.0, name=":START")
    stop = pvproperty(value=0.0, name=":STOP")
    npoints = pvproperty(value=10, name=":NPOINTS")
    dwell = pvproperty(value=1.0, name=":DWELL", doc="ms per point")
    axis = pvproperty(value=axis_labels[0], enum_strings=list(axis_labels),
                       dtype=ChannelType.ENUM, name=":AXIS")
    fly_mode = pvproperty(value="raster", enum_strings=["raster", "continuous"],
                          dtype=ChannelType.ENUM, name=":MODE")
    arm = pvproperty(value=0, name=":ARM")
    go = pvproperty(value=0, name=":GO")
    abort = pvproperty(value=0, name=":ABORT")
    state = pvproperty(value="IDLE", enum_strings=list(STATES),
                       dtype=ChannelType.ENUM, name=":STATE", read_only=True)
    error = pvproperty(value="", name=":ERROR", read_only=True,
                       dtype=ChannelType.CHAR, max_length=256,
                       report_as_string=True)
    pos = pvproperty(value=[0.0], name=":POS", read_only=True, max_length=MAX_LINE)
    index = pvproperty(value=0, name=":INDEX", read_only=True)

    data_props = {
        key: pvproperty(value=[0.0], name=f":DATA:{key}", read_only=True,
                        max_length=MAX_LINE)
        for key in daq_keys
    }

    @arm.putter
    async def arm(self, instance, value):
        if not value:
            return 0
        if STATES[_enum_index(self.state)] == "FLYING":
            # Do NOT touch STATE or the abort event: a racing ABORT must
            # still be able to interrupt the in-flight line.
            await self.error.write("ARM while FLYING rejected")
            return 0
        n = int(self.npoints.value)
        motor = self._current_motor()
        cfg = motor.config
        lo, hi = cfg["minValue"], cfg["maxValue"]
        problems = []
        if not (1 <= n <= MAX_LINE):
            problems.append(f"NPOINTS {n} outside 1..{MAX_LINE}")
        for label, v in (("START", self.start.value), ("STOP", self.stop.value)):
            if not (lo <= v <= hi):
                problems.append(f"{label} {v} outside limits [{lo}, {hi}]")
        if self.dwell.value <= 0:
            problems.append(f"DWELL {self.dwell.value} must be > 0")
        if problems:
            await self._set_state("ERROR", "; ".join(problems)[:255])
        else:
            self._abort_event.clear()
            await self._set_state("ARMED")
        return 0

    @go.putter
    async def go(self, instance, value):
        if not value:
            return 0
        current = STATES[_enum_index(self.state)]
        if current != "ARMED":
            if current == "FLYING":
                # Do NOT touch STATE or the abort event: a racing GO must not
                # clobber a healthy in-flight line's STATE with ERROR (mirrors
                # the ARM-while-FLYING rejection above).
                await self.error.write("GO rejected: line in progress")
            else:
                await self._set_state("ERROR", f"GO rejected: state is {current}")
            return 0
        # Synchronous guard closing the check-then-act race: two concurrent
        # GO puts could both observe ARMED before either flips STATE, and
        # both proceed to _fly_one_line(). This flag is set before the first
        # await so the second putter's check below is atomic with respect to
        # the first (no await happens between the state check and this set).
        if self._flying:
            await self.error.write("GO rejected: line in progress")
            return 0
        self._flying = True
        try:
            await self._set_state("FLYING")
            try:
                aborted = await self._fly_one_line()
            except Exception as exc:  # noqa: BLE001 - surfaced on :ERROR
                await self._set_state("ERROR", str(exc)[:255])
                return 0
            if aborted:
                await self._set_state("IDLE")
            else:
                await self._set_state("ARMED")
        finally:
            self._flying = False
        return 0

    @abort.putter
    async def abort(self, instance, value):
        if value:
            self._abort_event.set()
        return 0

    def __init__(self, prefix, *, motors, daq_groups, simulation=True, **kwargs):
        PVGroup.__init__(self, prefix, **kwargs)
        self._motors = motors
        self._daq_groups = daq_groups
        self._simulation = simulation
        self._abort_event = asyncio.Event()
        self._flying = False
        self._data_props = {k: getattr(self, f"data_{k}") for k in daq_keys}

    async def _set_state(self, name, error_msg=""):
        await self.state.write(name)
        await self.error.write(error_msg)

    def _current_motor(self):
        label = self.axis.enum_strings[_enum_index(self.axis)]
        return self._motors[label]

    async def _fly_one_line(self) -> bool:
        loop = asyncio.get_running_loop()
        n = int(self.npoints.value)
        dwell = float(self.dwell.value)
        x0, x1 = float(self.start.value), float(self.stop.value)

        lines = {}
        for key, group in self._daq_groups.items():
            if self._abort_event.is_set():
                return True
            daq = group._daq
            await group._ensure_started()
            await loop.run_in_executor(
                None, functools.partial(daq.config, dwell, count=n, samples=1))
            if self._simulation:
                get_task = asyncio.ensure_future(daq.getLine())
                while not get_task.done():
                    if self._abort_event.is_set():
                        get_task.cancel()
                        try:
                            await get_task
                        except asyncio.CancelledError:
                            pass
                        return True
                    await asyncio.sleep(0.02)
                lines[key] = get_task.result()
            else:  # pragma: no cover - hardware path, benchmark-only (Task 12)
                motor = self._current_motor()
                motor.trajectory_pixel_count = n
                motor.trajectory_pixel_dwell = dwell
                motor.lineMode = "continuous"
                motor.trajectory_start = (x0, 0.0)
                motor.trajectory_stop = (x1, 0.0)
                motor.update_trajectory()
                line, _ = await asyncio.gather(
                    daq.getLine(),
                    loop.run_in_executor(None, motor.moveLine))
                lines[key] = line

        positions = np.linspace(x0, x1, n)
        if not self._simulation:  # pragma: no cover - hardware path
            motor = self._current_motor()
            if hasattr(motor, "positions"):
                positions = np.asarray(motor.positions)[:n]

        # ---- ordering contract: waveforms FIRST, then INDEX ----
        for key, line in lines.items():
            await self._data_props[key].write([float(v) for v in line])
            await self._daq_groups[key].write_line(line)
        await self.pos.write([float(v) for v in positions])
        await self.index.write(self.index.value + 1)
        return False

    namespace = {
        "start": start, "stop": stop, "npoints": npoints, "dwell": dwell,
        "axis": axis, "fly_mode": fly_mode, "arm": arm, "go": go,
        "abort": abort, "state": state, "error": error, "pos": pos,
        "index": index, "__init__": __init__, "_set_state": _set_state,
        "_current_motor": _current_motor, "_fly_one_line": _fly_one_line,
    }
    for key, prop in data_props.items():
        namespace[f"data_{key}"] = prop

    return type("FlyGroup", (PVGroup,), namespace)


def FlyGroup(prefix, *, motors, daq_groups, simulation=True, **kwargs):
    """Instantiate a FLY PVGroup wired to the given motors/DAQ groups.

    ``motors`` maps enum axis label -> driver; ``daq_groups`` maps a DAQ key
    -> an already-built ``daq_ioc.DaqGroup`` instance (co-hosted in the same
    IOC process, so the fly loop can call ``write_line`` on it directly).
    """
    cls = _fly_group_class(list(motors), list(daq_groups))
    return cls(prefix, motors=motors, daq_groups=daq_groups,
               simulation=simulation, **kwargs)


def build_pvdb_from_slice(s: dict) -> dict:
    if not (s["kind"] == "controller" and s["controller_cls"] == "E712Controller"):
        raise ValueError(
            f"e712_ioc requires kind=controller/controller_cls=E712Controller, "
            f"got kind={s['kind']!r} controller_cls={s.get('controller_cls')!r}")
    from pystxmcontrol.iocs.base import MotorRecordGroup, build_controller, build_motor

    controller_dict = {
        "controller": s["controller_cls"],
        "address": s["controller_id"],
        "port": s["port"],
        "simulation": s["simulation"],
    }
    controller = build_controller(controller_dict)
    pvdb: dict = {}
    motors = {}
    for m in s["motors"]:
        drv = build_motor(m["entry"]["driver"], controller, m["entry"],
                          m["entry"]["axis"])
        motors[m["entry"]["axis"]] = drv
        pvdb.update(MotorRecordGroup(m["pv"], driver=drv,
                                     motor_config=m["entry"]).pvdb)

    daq_groups = {}
    for d in s.get("daqs", []):
        dq_pvdb, group = build_pvdb_for_entry(d["entry"], d["prefix"])
        pvdb.update(dq_pvdb)
        daq_groups[d["key"]] = group

    if daq_groups:
        fly_prefix = f"STXM{s['station']}:{s['label']}:FLY"
        fly = FlyGroup(fly_prefix, motors=motors, daq_groups=daq_groups,
                       simulation=bool(s["simulation"]))
        pvdb.update(fly.pvdb)
    return pvdb


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--slice", required=True)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)
    run(build_pvdb_from_slice(read_slice(args.slice)), log_pv_names=not args.quiet)


if __name__ == "__main__":
    main()
