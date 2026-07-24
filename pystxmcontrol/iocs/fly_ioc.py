"""Controller-agnostic fly IOC: motor records + FLY PVGroup with an IOC-side
line loop.

Serves any fly-capable controller group (see config.FLY_CAPABLE_CONTROLLERS,
e.g. E712Controller, nptController) -- the line loop is controller-agnostic and
drives the selected axis motor through the duck-typed fly interface
(trajectory_* attrs + update_trajectory/moveLine + continuous lineMode).

Historically this lived in a module named ``e712_ioc``; the logic is not
specific to the E712, so it now lives here under a controller-neutral name.
``pystxmcontrol.iocs.e712_ioc`` remains as a thin back-compat shim that
re-exports everything defined here.

Consistency contract: on each completed line, ALL data/pos waveforms are
written BEFORE :INDEX increments. Clients monitor :INDEX, then read waveforms.

DAQs are consumed as standalone services (see daq_ioc.DaqGroup's :LINE:*
surface) over Channel Access -- the fly IOC holds no in-process DAQ objects,
only CA client handles (DaqClient) keyed by DAQ prefix.
"""
from __future__ import annotations

import argparse
import asyncio
import functools
import threading
import time

import numpy as np

from pystxmcontrol.iocs import configure_ioc_logging, require_caproto

require_caproto()

from caproto import ChannelType  # noqa: E402
from caproto.server import PVGroup, pvproperty, run  # noqa: E402

from pystxmcontrol.iocs.config import read_slice  # noqa: E402
from pystxmcontrol.iocs.daq_ioc import MAX_LINE  # noqa: E402

STATES = ["IDLE", "ARMED", "FLYING", "ERROR"]


class DaqClient:
    """Blocking CA client handle for one DAQ service's :LINE: surface.

    All methods are BLOCKING and must be called from an executor thread,
    never from the IOC's asyncio loop. One persistent caproto threading
    Context is shared across clients (and with _command_shutters); PVs
    connect lazily on first use and stay connected; the :LINE:INDEX monitor
    is subscribed exactly once.
    """

    def __init__(self, prefix: str, ctx):
        self._prefix = prefix
        self._ctx = ctx
        self._connected = False
        self._last = {}          # last-written config values
        self._index = None       # latest :LINE:INDEX seen by the monitor
        self._index_event = threading.Event()
        self._armed_from = None  # :LINE:INDEX value captured at arm()

    def _connect(self):
        if self._connected:
            return
        p = self._prefix
        (self._npoints, self._dwell, self._trigger, self._arm, self._abort,
         self._index_pv, self._status, self._error, self._wf) = \
            self._ctx.get_pvs(
                f"{p}:LINE:NPOINTS", f"{p}:DWELL", f"{p}:LINE:TRIGGER",
                f"{p}:LINE:ARM", f"{p}:LINE:ABORT", f"{p}:LINE:INDEX",
                f"{p}:LINE:STATUS", f"{p}:LINE:ERROR", f"{p}:COUNTS:WF")
        for pv in (self._npoints, self._dwell, self._trigger, self._arm,
                   self._abort, self._index_pv, self._status, self._error,
                   self._wf):
            pv.wait_for_connection(timeout=5.0)
        sub = self._index_pv.subscribe(data_type="native")
        sub.add_callback(self._on_index)
        self._connected = True

    def _on_index(self, sub, response):
        self._index = int(response.data[0])
        self._index_event.set()

    def _write_if_changed(self, key, pv, value, **kw):
        if self._last.get(key) != value:
            pv.write(value, wait=True, timeout=5.0, **kw)
            self._last[key] = value

    def configure(self, dwell: float, npoints: int, trigger: str):
        self._connect()
        self._write_if_changed("npoints", self._npoints, int(npoints))
        self._write_if_changed("dwell", self._dwell, float(dwell))
        # enum by STRING dtype (threading-client enum gotcha)
        self._write_if_changed("trigger", self._trigger, trigger,
                               data_type=ChannelType.STRING)

    def arm(self):
        self._connect()
        if self._index is None:
            self._index = int(self._index_pv.read().data[0])
        self._armed_from = self._index
        self._index_event.clear()
        self._arm.write(1, wait=True, timeout=10.0)
        # Fast-fail on contention: a rejected ARM leaves STATUS != ARMED /
        # ACQUIRING (and an explanation on :LINE:ERROR). Without this check
        # the caller would only discover the rejection via its line timeout.
        status = int(self._status.read().data[0])
        if status not in (1, 2):  # ARMED, ACQUIRING
            err = self._error.read(data_type="native").data
            msg = (err.decode() if isinstance(err, bytes)
                   else bytes(int(x) for x in err if int(x)).decode())
            raise RuntimeError(
                f"DAQ {self._prefix} did not arm (status index {status}): {msg}")

    def wait_line(self, deadline_monotonic: float) -> bool:
        """True when :LINE:INDEX advances past its at-arm value; False on
        deadline."""
        while True:
            if (self._index is not None and self._armed_from is not None
                    and self._index > self._armed_from):
                return True
            remaining = deadline_monotonic - time.monotonic()
            if remaining <= 0:
                return False
            self._index_event.wait(min(remaining, 0.05))
            self._index_event.clear()

    def read_line(self, n: int):
        self._connect()
        data = self._wf.read().data
        return [float(v) for v in data[:n]]

    def abort(self):
        """Best-effort; never raises (teardown path)."""
        try:
            self._connect()
            self._abort.write(1, wait=True, timeout=2.0)
        except Exception:  # noqa: BLE001
            pass


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

    def __init__(self, prefix, *, motors, daq_pvs, simulation=True,
                 io_lock=None, shutter_pvs=None, **kwargs):
        PVGroup.__init__(self, prefix, **kwargs)
        self._motors = motors
        self._daq_pvs = dict(daq_pvs)
        self._simulation = simulation
        self._abort_event = asyncio.Event()
        self._flying = False
        self._data_props = {k: getattr(self, f"data_{k}") for k in daq_keys}
        # Shared with the motor records on the same controller so a fly line's
        # motor I/O never interleaves with the axes' RBV pollers on the link.
        import threading as _threading
        self._io_lock = io_lock if io_lock is not None else _threading.Lock()
        # Beam-shutter MODE PVs (served by the dedicated shutter IOC) to open
        # for the beam during a line and close after. Commanded over Channel
        # Access -- the fly IOC must NOT open the Arduino directly (that
        # contends with the shutter IOC). Empty -> no beam gating.
        self._shutter_pvs = list(shutter_pvs or [])
        self._ca_ctx = None
        self._daq_clients = {}

    def _daq_client(self, key):
        """Lazily build the shared Context + per-key DaqClient. Called from
        executor threads only (Context and connection setup block)."""
        if self._ca_ctx is None:
            from caproto.threading.client import Context
            self._ca_ctx = Context()
        if key not in self._daq_clients:
            self._daq_clients[key] = DaqClient(self._daq_pvs[key], self._ca_ctx)
        return self._daq_clients[key]

    def _command_shutters(self, mode):
        """Best-effort CA write of ``<shutter>:MODE`` for every beam shutter
        tied to this controller's DAQs. Runs in an executor thread and NEVER
        raises into the fly loop -- a shutter that won't respond must not fail
        the line (worst case: dark counts). ``mode`` is "OPEN"/"CLOSED"/"AUTO".
        """
        if not self._shutter_pvs:
            return
        try:
            from caproto.threading.client import Context
        except Exception:  # noqa: BLE001 - CA client unavailable
            return
        if self._ca_ctx is None:
            self._ca_ctx = Context()
        for prefix in self._shutter_pvs:
            try:
                (pv,) = self._ca_ctx.get_pvs(f"{prefix}:MODE")
                pv.wait_for_connection(timeout=1.0)
                pv.write(mode, wait=True, timeout=1.0)
            except Exception:  # noqa: BLE001 - best-effort, per-shutter
                pass

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
        motor = self._current_motor()
        axis_label = self.axis.enum_strings[_enum_index(self.axis)]
        other_label = "x" if axis_label == "y" else "y"
        other_motor = self._motors.get(other_label)
        line_trigger = getattr(motor, "line_trigger", "EXT")
        deadline_wall = max(5.0, (dwell * n) / 1000.0 * 4.0 + 5.0)

        def _setup_trajectory():
            # Caller must hold self._io_lock.
            perp = (other_motor.getPos()
                    if other_motor is not None
                    and hasattr(other_motor, "getPos") else 0.0)
            motor.trajectory_pixel_count = n
            motor.trajectory_pixel_dwell = dwell
            motor.lineMode = "continuous"
            if axis_label == "y":
                motor.trajectory_start = (perp, x0)
                motor.trajectory_stop = (perp, x1)
            else:
                motor.trajectory_start = (x0, perp)
                motor.trajectory_stop = (x1, perp)
            motor.update_trajectory()

        def _run_line():
            with self._io_lock:
                _setup_trajectory()
                motor.moveLine()

        def _prepare_line():
            # Free-run (non-EXT) trigger: pre-position BEFORE the DAQs are
            # armed (arming starts acquisition immediately for IMM).
            with self._io_lock:
                _setup_trajectory()
                motor.prepareLine()

        def _run_prepared_line():
            with self._io_lock:
                motor.moveLine()

        async def _close_beam():
            await loop.run_in_executor(None, self._command_shutters, "CLOSED")

        stage = {"name": "queued"}
        lines: dict = {}

        async def _line():
            stage["name"] = "daq_config"
            for key in self._daq_pvs:
                client = await loop.run_in_executor(None, self._daq_client, key)
                await loop.run_in_executor(
                    None, functools.partial(client.configure, dwell, n,
                                            line_trigger))
            if not self._simulation and line_trigger != "EXT":
                stage["name"] = "prepare"
                await loop.run_in_executor(None, _prepare_line)
            stage["name"] = "daq_arm"
            for key in self._daq_pvs:
                await loop.run_in_executor(None, self._daq_clients[key].arm)
            if not self._simulation:
                stage["name"] = "beam_open"
                await loop.run_in_executor(None, self._command_shutters, "OPEN")
                try:
                    stage["name"] = "move"
                    await loop.run_in_executor(
                        None,
                        _run_prepared_line if line_trigger != "EXT"
                        else _run_line)
                finally:
                    await _close_beam()
            stage["name"] = "daq_wait"
            wait_deadline = time.monotonic() + deadline_wall
            for key in self._daq_pvs:
                ok = await loop.run_in_executor(
                    None, self._daq_clients[key].wait_line, wait_deadline)
                if not ok:
                    raise TimeoutError(
                        f"DAQ {self._daq_pvs[key]} produced no line within "
                        f"{deadline_wall:.1f}s")
            stage["name"] = "daq_read"
            for key in self._daq_pvs:
                lines[key] = await loop.run_in_executor(
                    None, functools.partial(self._daq_clients[key].read_line, n))
            stage["name"] = "done"

        fly_task = asyncio.ensure_future(_line())
        deadline = loop.time() + deadline_wall + 2.0  # inner deadline fires first
        while not fly_task.done():
            if self._abort_event.is_set() or loop.time() > deadline:
                fly_task.cancel()
                try:
                    await fly_task
                except BaseException:  # noqa: BLE001 - best-effort teardown
                    pass
                for key in list(self._daq_clients):
                    await loop.run_in_executor(None, self._daq_clients[key].abort)
                if not self._simulation:
                    await _close_beam()
                if self._abort_event.is_set():
                    return True
                raise TimeoutError(
                    f"fly line stalled at step '{stage['name']}' (no completion "
                    f"within {deadline_wall:.1f}s, dwell={dwell} ms, n={n})")
            await asyncio.sleep(0.02)
        exc = fly_task.exception()
        if exc is not None:
            raise exc

        positions = np.linspace(x0, x1, n)
        if not self._simulation and hasattr(motor, "positions"):
            positions = np.asarray(motor.positions)[:n]

        # ---- ordering contract: waveforms FIRST, then INDEX ----
        for key, line in lines.items():
            await self._data_props[key].write([float(v) for v in line])
        await self.pos.write([float(v) for v in positions])
        await self.index.write(self.index.value + 1)
        return False

    namespace = {
        "start": start, "stop": stop, "npoints": npoints, "dwell": dwell,
        "axis": axis, "fly_mode": fly_mode, "arm": arm, "go": go,
        "abort": abort, "state": state, "error": error, "pos": pos,
        "index": index, "__init__": __init__, "_set_state": _set_state,
        "_current_motor": _current_motor, "_fly_one_line": _fly_one_line,
        "_command_shutters": _command_shutters, "_daq_client": _daq_client,
    }
    for key, prop in data_props.items():
        namespace[f"data_{key}"] = prop

    return type("FlyGroup", (PVGroup,), namespace)


def FlyGroup(prefix, *, motors, daq_pvs, simulation=True, io_lock=None,
             **kwargs):
    """Instantiate a FLY PVGroup wired to the given motors/DAQ services.

    ``motors`` maps enum axis label -> driver; ``daq_pvs`` maps a DAQ key ->
    the CA prefix of its standalone daq_ioc service (see DaqClient) -- the
    fly loop reaches the DAQ over Channel Access, never in-process.
    ``io_lock`` is the controller's shared transaction lock (see
    MotorRecordGroup); pass the same lock used for this controller's motor
    records so a fly line serializes with their RBV pollers.
    """
    cls = _fly_group_class(list(motors), list(daq_pvs))
    return cls(prefix, motors=motors, daq_pvs=daq_pvs,
               simulation=simulation, io_lock=io_lock, **kwargs)


def build_pvdb_from_slice(s: dict) -> dict:
    from pystxmcontrol.iocs.config import FLY_CAPABLE_CONTROLLERS

    if not (s["kind"] == "controller"
            and s["controller_cls"] in FLY_CAPABLE_CONTROLLERS):
        raise ValueError(
            f"fly IOC requires kind=controller with a fly-capable controller_cls "
            f"({sorted(FLY_CAPABLE_CONTROLLERS)}), got kind={s['kind']!r} "
            f"controller_cls={s.get('controller_cls')!r}")
    from pystxmcontrol.iocs.base import MotorRecordGroup, build_controller, build_motor

    controller_dict = {
        "controller": s["controller_cls"],
        "address": s["controller_id"],
        "port": s["port"],
        "simulation": s["simulation"],
    }
    controller = build_controller(controller_dict)
    # One lock per controller, shared by every motor record AND the fly loop,
    # so no two blocking transactions overlap on the controller's link.
    import threading
    io_lock = threading.Lock()
    pvdb: dict = {}
    motors = {}
    for m in s["motors"]:
        drv = build_motor(m["entry"]["driver"], controller, m["entry"],
                          m["entry"]["axis"])
        motors[m["entry"]["axis"]] = drv
        pvdb.update(MotorRecordGroup(m["pv"], driver=drv,
                                     motor_config=m["entry"],
                                     io_lock=io_lock).pvdb)

    daq_pvs = s.get("daq_pvs", {})
    shutter_pvs = sorted(set(s.get("shutters", {}).values()))
    if daq_pvs:
        fly_prefix = f"STXM{s['station']}:{s['label']}:FLY"
        fly = FlyGroup(fly_prefix, motors=motors, daq_pvs=daq_pvs,
                       simulation=bool(s["simulation"]), io_lock=io_lock,
                       shutter_pvs=shutter_pvs)
        pvdb.update(fly.pvdb)
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
