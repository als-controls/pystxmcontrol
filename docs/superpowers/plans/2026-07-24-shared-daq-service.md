# Shared DAQ CA Service Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Every DAQ becomes a standalone CA service; fly IOCs become CA clients of it, so any number of fly-capable controllers (E712, nPoint, MMC) share detectors.

**Architecture:** `daq_ioc.DaqGroup` grows a line-mode CA surface (`:LINE:ARM` with put-completion = armed, write-then-increment `:LINE:INDEX`, busy guard, watchdog). `fly_ioc` replaces in-process `daq_groups` with a `DaqClient` (persistent caproto threading Context, subscribe-once INDEX monitor) and both sim and hardware paths go through CA. `supervisor.plan_fleet` drops absorb-all and the >1-fly-group raise. Spec: `docs/superpowers/specs/2026-07-24-shared-daq-service-design.md`.

**Tech Stack:** Python, caproto server + threading client, pytest.

## Global Constraints

- Repo/worktree: `C:\Users\rp\PycharmProjects\ncs\lightfall-pystxmcontrol\_pystxmcontrol_iocs_wt`, branch `feature/caproto-iocs`. CWD for all commands.
- Test command: `C:\Users\rp\PycharmProjects\ncs\.venv\Scripts\python.exe -m pytest <paths> -q` — NEVER bare `pytest`.
- **Do NOT modify anything under `pystxmcontrol/drivers/`** — Ron is hardware-testing the MMC drivers from this branch right now. No task below needs driver changes.
- Pre-existing failures to IGNORE (env deps missing in this venv): `tests/iocs/test_config.py::test_motor_pv_naming`, `::test_colocated_derived`, `tests/iocs/test_supervisor.py::test_plan_fleet_modules` (asserts ≥4 motor_ioc groups but MCL is unavailable here — if your supervisor changes touch this test, fix its expectations but know the 4th-group shortfall is environmental), `tests/iocs/test_e712_fly.py` errors (no pipython), `tests/iocs/test_npt_read_framing.py` collection (no pylibftdi).
- Ordering contract (both fly IOC and DAQ service): waveform PVs are written BEFORE the index PV increments.
- caproto gotchas (learned on this branch): enum putters receive the enum STRING; threading-client enum writes go by index or `data_type=ChannelType.STRING`; test CA needs the `ioc_harness` fixture from `tests/iocs/conftest.py` (it handles EPICS_CA(S)_SERVER_PORT/ADDR_LIST); the threading client does NOT resolve put futures on ErrorResponse — so rejections must be no-op + error-PV, never a raised putter exception.
- `git add` with explicit paths only. Every commit message ends with:
  `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>` and
  `Claude-Session: https://claude.ai/code/session_01AHAAJ8YhaHTjQRLC9aAfso`

---

### Task 1: DAQ line-mode CA surface

**Files:**
- Modify: `pystxmcontrol/iocs/daq_ioc.py`
- Test: `tests/iocs/test_daq_line_service.py` (new)

**Interfaces:**
- Consumes: existing `DaqGroup` (`:DWELL`, `:ACQUIRE`, `:COUNTS:WF`, `write_line`, `_ensure_started`, `self._daq` keysight driver with `.simulation`, `.config(dwell, count, samples, trigger)`, `.initLine()`, coroutine `.getLine()`).
- Produces: new PVs on every DAQ service — `:LINE:NPOINTS` (int), `:LINE:TRIGGER` (enum EXT/IMM/BUS), `:LINE:ARM` (put-completion = armed), `:LINE:INDEX` (read-only, increments AFTER `:COUNTS:WF` is written), `:LINE:ABORT`, `:LINE:STATUS` (enum IDLE/ARMED/ACQUIRING/ERROR), `:LINE:ERROR` (char string). Busy guard + watchdog semantics per spec. Task 2's `DaqClient` drives exactly these.

- [ ] **Step 1: Write the failing tests**

Create `tests/iocs/test_daq_line_service.py`:

```python
"""Line-mode CA surface of the standalone DAQ service (simulation)."""
import time

import pytest

PREFIX = "STXMSIM:DAQSVC"


def _read_str(pv):
    data = pv.read(data_type="native").data
    if isinstance(data, bytes):
        return data.decode().rstrip("\x00")
    return bytes(int(x) for x in data if int(x) != 0).decode()


@pytest.fixture
def daq_service(ioc_harness):
    from pystxmcontrol.iocs.daq_ioc import build_pvdb_for_entry
    entry = {"name": "Counter1", "driver": "keysight53230A", "address": "sim",
             "port": 5025, "channel": 1, "ndim": 0, "gate": False,
             "record": True, "simulation": True}
    pvdb, _ = build_pvdb_for_entry(entry, PREFIX)
    ioc_harness.start(pvdb)
    return ioc_harness, ioc_harness.client()


def _pvs(ctx, *suffixes):
    pvs = ctx.get_pvs(*[f"{PREFIX}:{s}" for s in suffixes], timeout=15)
    for pv in pvs:
        pv.wait_for_connection(timeout=15)
    return pvs


def _wait_index(index_pv, target, timeout=15.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if index_pv.read().data[0] >= target:
            return True
        time.sleep(0.02)
    return False


def test_line_arm_completes_and_publishes(daq_service):
    _, ctx = daq_service
    npts, dwell, arm, index, wf, status = _pvs(
        ctx, "LINE:NPOINTS", "DWELL", "LINE:ARM", "LINE:INDEX",
        "COUNTS:WF", "LINE:STATUS")
    npts.write(25, wait=True)
    dwell.write(1.0, wait=True)
    assert index.read().data[0] == 0
    arm.write(1, wait=True, timeout=15)          # put-completion = armed
    assert _wait_index(index, 1)                  # sim line: ~25 ms later
    data = wf.read().data
    assert len(data) == 25 and all(v > 0 for v in data)
    assert status.read().data[0] == 0             # back to IDLE


def test_arm_while_busy_rejected(daq_service):
    _, ctx = daq_service
    npts, dwell, arm, err, abort, status = _pvs(
        ctx, "LINE:NPOINTS", "DWELL", "LINE:ARM", "LINE:ERROR",
        "LINE:ABORT", "LINE:STATUS")
    npts.write(200, wait=True)
    dwell.write(20.0, wait=True)                  # 4 s sim line
    arm.write(1, wait=True, timeout=15)
    arm.write(1, wait=True, timeout=15)           # second arm: no-op + error
    assert "rejected" in _read_str(err)
    abort.write(1, wait=True, timeout=15)
    t0 = time.monotonic()
    while status.read().data[0] not in (0, 3) and time.monotonic() - t0 < 10:
        time.sleep(0.05)
    assert status.read().data[0] == 0             # IDLE after abort


def test_abort_leaves_index_unchanged(daq_service):
    _, ctx = daq_service
    npts, dwell, arm, abort, index = _pvs(
        ctx, "LINE:NPOINTS", "DWELL", "LINE:ARM", "LINE:ABORT", "LINE:INDEX")
    before = index.read().data[0]
    npts.write(200, wait=True)
    dwell.write(20.0, wait=True)
    arm.write(1, wait=True, timeout=15)
    abort.write(1, wait=True, timeout=15)
    time.sleep(0.2)
    assert index.read().data[0] == before


def test_point_acquire_rejected_during_line(daq_service):
    _, ctx = daq_service
    npts, dwell, arm, acquire, err, abort = _pvs(
        ctx, "LINE:NPOINTS", "DWELL", "LINE:ARM", "ACQUIRE", "LINE:ERROR",
        "LINE:ABORT")
    npts.write(200, wait=True)
    dwell.write(20.0, wait=True)
    arm.write(1, wait=True, timeout=15)
    acquire.write(1, wait=True, timeout=15)       # no-op + error, NOT hang
    assert "ACQUIRE rejected" in _read_str(err)
    abort.write(1, wait=True, timeout=15)


def test_arm_validation_errors(daq_service):
    _, ctx = daq_service
    npts, arm, status, err = _pvs(
        ctx, "LINE:NPOINTS", "LINE:ARM", "LINE:STATUS", "LINE:ERROR")
    npts.write(0, wait=True)
    arm.write(1, wait=True, timeout=15)
    assert status.read().data[0] == 3             # ERROR
    assert "NPOINTS" in _read_str(err)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `C:\Users\rp\PycharmProjects\ncs\.venv\Scripts\python.exe -m pytest tests\iocs\test_daq_line_service.py -q`
Expected: FAIL — `:LINE:*` PVs don't exist (`get_pvs` timeout / connection failure).

- [ ] **Step 3: Implement the line-mode surface**

In `pystxmcontrol/iocs/daq_ioc.py`, add after the imports:

```python
LINE_STATES = ["IDLE", "ARMED", "ACQUIRING", "ERROR"]


def _enum_str(prop) -> str:
    """Enum pvproperty value normalized to its string label (caproto stores
    either the index or the label depending on version/path)."""
    v = prop.value
    return v if isinstance(v, str) else prop.enum_strings[int(v)]
```

Add these pvproperties to `DaqGroup` (alongside the existing ones):

```python
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
```

In `DaqGroup.__init__`, add `self._line_task = None`.

Add methods + putters to `DaqGroup`:

```python
    def _line_busy(self) -> bool:
        return self._line_task is not None and not self._line_task.done()

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
        # ---- ordering contract: waveform FIRST, then INDEX ----
        await self.write_line(line)
        await self.line_index.write(self.line_index.value + 1)
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
```

Modify the existing `acquire` putter — insert immediately after `if not value: return 0`:

```python
        if self._line_busy():
            await self.line_error.write("ACQUIRE rejected: line in progress")
            return 0
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `C:\Users\rp\PycharmProjects\ncs\.venv\Scripts\python.exe -m pytest tests\iocs\test_daq_line_service.py tests\iocs\test_daq_ioc.py -q`
Expected: all pass (line-service tests plus no regression in the existing DAQ IOC tests).

- [ ] **Step 5: Commit**

```bash
git add pystxmcontrol/iocs/daq_ioc.py tests/iocs/test_daq_line_service.py
git commit -m "feat(daq): line-mode CA surface (LINE:ARM/INDEX/ABORT, busy guard, watchdog)"
```

---

### Task 2: fly_ioc becomes a CA client of DAQ services

**Files:**
- Modify: `pystxmcontrol/iocs/fly_ioc.py` (FlyGroup ctor/`_fly_one_line`/`build_pvdb_from_slice`, module docstring)
- Modify: `tests/iocs/test_mmc_fly.py`, `tests/iocs/test_npt_fly.py`, `tests/iocs/test_e712_fly.py` (fixtures: DAQ service pvdb + `daq_pvs` instead of `daq_groups`)
- Test: additions in `tests/iocs/test_mmc_fly.py`

**Interfaces:**
- Consumes: Task 1's `:LINE:*` PV surface.
- Produces: `DaqClient(prefix, ctx)` in fly_ioc.py with blocking methods `configure(dwell, npoints, trigger)`, `arm()` (raises `RuntimeError` if the service reports busy/error after the put), `wait_line(deadline_monotonic) -> bool`, `read_line(n) -> list[float]`, `abort()`; `FlyGroup(prefix, *, motors, daq_pvs, simulation=True, io_lock=None, shutter_pvs=None)` (**`daq_groups` parameter is GONE**); `build_pvdb_from_slice` consumes slice key `daq_pvs: {key: prefix}` and no longer builds DAQ pvdbs. Fly IOC's own client-facing PVs (`:DATA:<key>`, `:POS`, `:INDEX`, states) unchanged.

- [ ] **Step 1: Update the fly-test fixtures and add the CA-path test (failing first)**

In `tests/iocs/test_mmc_fly.py`, replace the `mmc_fly_ioc` fixture body's DAQ wiring: instead of

```python
    daq_pvdb, daq_group = build_pvdb_for_entry(daq_entry, "STXMSIM:DEFAULT")
    ...
    fly = FlyGroup(FLY_PREFIX, motors={"x": mx, "y": my},
                   daq_groups={"default": daq_group}, simulation=True)
```

use

```python
    daq_pvdb, _ = build_pvdb_for_entry(daq_entry, "STXMSIM:DEFAULT")
    ...
    fly = FlyGroup(FLY_PREFIX, motors={"x": mx, "y": my},
                   daq_pvs={"default": "STXMSIM:DEFAULT"}, simulation=True)
```

(both the DAQ service pvdb and the fly pvdb go into the same `ioc_harness.start(pvdb)` — the fly group reaches the DAQ PVs over real CA loopback). Make the same mechanical change in `test_npt_fly.py` and `test_e712_fly.py` fixtures. Then add to `test_mmc_fly.py`:

```python
def test_fly_line_data_travels_over_ca(mmc_fly_ioc):
    """The DAQ service's own :LINE:INDEX advances when the fly IOC runs a
    line -- proof the fly loop consumed the CA service, not an in-process
    object."""
    import numpy as np
    _, ctx = mmc_fly_ioc
    start, stop, npts, dwell, arm, go, fly_index = _fly_pvs(
        ctx, *[f"{FLY_PREFIX}:{s}" for s in (
            "START", "STOP", "NPOINTS", "DWELL", "ARM", "GO", "INDEX")])
    (daq_index,) = _fly_pvs(ctx, "STXMSIM:DEFAULT:LINE:INDEX")
    d0 = daq_index.read().data[0]
    start.write(-5.0, wait=True); stop.write(5.0, wait=True)
    npts.write(25, wait=True); dwell.write(1.0, wait=True)
    arm.write(1, wait=True, timeout=15)
    go.write(1, wait=True, timeout=60)
    assert fly_index.read().data[0] >= 1
    assert daq_index.read().data[0] == d0 + 1
```

(Adjust `_fly_pvs` usage to however that helper is defined in the file — it takes full PV names.)

- [ ] **Step 2: Run to verify failure**

Run: `C:\Users\rp\PycharmProjects\ncs\.venv\Scripts\python.exe -m pytest tests\iocs\test_mmc_fly.py -q`
Expected: FAIL — `FlyGroup() got an unexpected keyword argument 'daq_pvs'`.

- [ ] **Step 3: Implement DaqClient and rewrite the fly loop**

In `pystxmcontrol/iocs/fly_ioc.py`:

3a. Add near the top (after the existing imports): `import threading`, `import time`, and the client class:

```python
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
        from caproto import ChannelType
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
```

3b. `FlyGroup` construction changes inside `_fly_group_class` — the `__init__` in the namespace becomes:

```python
    def __init__(self, prefix, *, motors, daq_pvs, simulation=True,
                 io_lock=None, shutter_pvs=None, **kwargs):
        PVGroup.__init__(self, prefix, **kwargs)
        self._motors = motors
        self._daq_pvs = dict(daq_pvs)
        self._simulation = simulation
        self._abort_event = asyncio.Event()
        self._flying = False
        self._data_props = {k: getattr(self, f"data_{k}") for k in daq_keys}
        import threading as _threading
        self._io_lock = io_lock if io_lock is not None else _threading.Lock()
        self._shutter_pvs = list(shutter_pvs or [])
        self._ca_ctx = None
        self._daq_clients = {}
```

and add a helper method (into the namespace dict, like `_command_shutters`):

```python
    def _daq_client(self, key):
        """Lazily build the shared Context + per-key DaqClient. Called from
        executor threads only (Context and connection setup block)."""
        if self._ca_ctx is None:
            from caproto.threading.client import Context
            self._ca_ctx = Context()
        if key not in self._daq_clients:
            self._daq_clients[key] = DaqClient(self._daq_pvs[key], self._ca_ctx)
        return self._daq_clients[key]
```

(`_command_shutters` keeps its own lazy `self._ca_ctx is None` check — both now share the same Context.)

3c. Rewrite `_fly_one_line` — the per-DAQ loop with its sim/hardware fork is replaced by one arm-all → move-once → collect-all sequence; only motor handling stays sim-gated:

```python
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
```

Notes baked into the code above (keep as comments where marked): the fly IOC no longer calls `write_line` (the DAQ service publishes its own `:COUNTS:WF`); the inner `_line` coroutine owns the deadline via `wait_line`, the outer loop is the abort/backstop with +2 s grace; a cancelled `run_in_executor` leaves its worker thread to finish `wait_line` at deadline — bounded and harmless.

3d. `FlyGroup` factory + `build_pvdb_from_slice`: change the factory signature to `FlyGroup(prefix, *, motors, daq_pvs, simulation=True, io_lock=None, **kwargs)` (docstring: `daq_pvs` maps DAQ key → CA prefix of its standalone service). `_fly_group_class(list(motors), list(daq_pvs))`. In `build_pvdb_from_slice`, DELETE the whole `for d in s.get("daqs", [])` block (including the gate-override comment — that concern moves to the supervisor's DAQ slices in Task 3) and replace with:

```python
    daq_pvs = s.get("daq_pvs", {})
    shutter_pvs = sorted(set(s.get("shutters", {}).values()))
    if daq_pvs:
        fly_prefix = f"STXM{s['station']}:{s['label']}:FLY"
        fly = FlyGroup(fly_prefix, motors=motors, daq_pvs=daq_pvs,
                       simulation=bool(s["simulation"]), io_lock=io_lock,
                       shutter_pvs=shutter_pvs)
        pvdb.update(fly.pvdb)
    return pvdb
```

Also remove the now-unused `build_pvdb_for_entry` import at the top (keep `MAX_LINE`), and update the module docstring's daq sentence to say the fly IOC consumes standalone DAQ services over CA.

- [ ] **Step 4: Run the fly suites**

Run: `C:\Users\rp\PycharmProjects\ncs\.venv\Scripts\python.exe -m pytest tests\iocs\test_mmc_fly.py tests\iocs\test_npt_fly.py tests\iocs\test_daq_line_service.py tests\iocs\test_mmc_driver.py -q`
Expected: all pass (npt tests may skip on missing pylibftdi — fine). CA fly tests take longer than before (~extra seconds); GO timeouts in tests are already 60 s.

- [ ] **Step 5: Commit**

```bash
git add pystxmcontrol/iocs/fly_ioc.py tests/iocs/test_mmc_fly.py tests/iocs/test_npt_fly.py tests/iocs/test_e712_fly.py
git commit -m "feat(fly): consume DAQs as CA services (DaqClient); arm-all/fly-once line loop"
```

---

### Task 3: supervisor + slice plumbing

**Files:**
- Modify: `pystxmcontrol/iocs/supervisor.py` (`plan_fleet`)
- Modify: `pystxmcontrol/iocs/config.py` (`write_slice`)
- Test: `tests/iocs/test_supervisor.py` (update plan_fleet tests), `tests/iocs/test_controller_serialization.py` if it builds fly slices (check and update mechanically)

**Interfaces:**
- Consumes: fly slices now need `daq_pvs` (Task 2); DAQ slices unchanged in shape (`kind=daq`).
- Produces: `plan_fleet` yields: one `daq_ioc` plan per DAQ (FIRST, before controller IOCs), controller plans (fly-capable → `fly_ioc` with `daq_pvs` in slice, else `motor_ioc`), shutters, derived last. NO absorb logic, NO >1-fly-group ValueError. `write_slice(group, fleet, path, daq_pvs=None)` — `daqs` parameter is GONE; controller payload carries `daq_pvs` when given; DaqEntry payload gets `gate=False` override when a shutter IOC owns its gate address.

- [ ] **Step 1: Update the failing tests**

In `tests/iocs/test_supervisor.py`, rewrite the absorb-oriented plan_fleet tests. Read the existing tests first (they use a `fleet` fixture from shipped configs and synthetic fleets for E712/npt absorb cases) and apply:

```python
def test_plan_fleet_daqs_always_standalone(fleet, tmp_path):
    from pystxmcontrol.iocs.supervisor import plan_fleet
    plans = plan_fleet(fleet, str(tmp_path))
    daq_plans = [p for p in plans if p.module == "pystxmcontrol.iocs.daq_ioc"]
    assert len(daq_plans) == len(fleet.daqs)
    # DAQ services start before any controller IOC (fly IOCs are their clients)
    modules = [p.module for p in plans]
    first_ctrl = min(i for i, m in enumerate(modules)
                     if m in ("pystxmcontrol.iocs.motor_ioc",
                              "pystxmcontrol.iocs.fly_ioc"))
    last_daq = max(i for i, m in enumerate(modules)
                   if m == "pystxmcontrol.iocs.daq_ioc")
    assert last_daq < first_ctrl


def test_plan_fleet_fly_slice_carries_daq_pvs(fleet, tmp_path):
    import json
    from pystxmcontrol.iocs.supervisor import plan_fleet
    plans = plan_fleet(fleet, str(tmp_path))
    fly = [p for p in plans if p.module == "pystxmcontrol.iocs.fly_ioc"]
    for p in fly:
        with open(p.slice_path) as f:
            s = json.load(f)
        assert "daqs" not in s
        assert set(s["daq_pvs"]) == {d.key for d in fleet.daqs}


def test_plan_fleet_multiple_fly_groups_allowed(fleet, tmp_path):
    """Two fly-capable groups in one fleet must plan without raising and
    each get daq_pvs for ALL daqs."""
    import copy, json
    from pystxmcontrol.iocs.supervisor import plan_fleet
    f2 = copy.deepcopy(fleet)
    fly_capable = [g for g in f2.controller_groups
                   if g.controller_cls in ("E712Controller", "nptController",
                                           "mmcController")]
    if len(fly_capable) < 2:
        # synthesize a second fly group from the first
        src = copy.deepcopy(fly_capable[0] if fly_capable
                            else f2.controller_groups[0])
        src.controller_cls = "mmcController"
        src.label = src.label + "_B"
        src.controller_id = src.controller_id + "_B"
        f2.controller_groups.append(src)
        if not fly_capable:
            f2.controller_groups[-1].controller_cls = "mmcController"
            fly_capable = [f2.controller_groups[-1]]
            src2 = copy.deepcopy(src); src2.label += "2"; src2.controller_id += "2"
            f2.controller_groups.append(src2)
    plans = plan_fleet(f2, str(tmp_path))
    fly_plans = [p for p in plans if p.module == "pystxmcontrol.iocs.fly_ioc"]
    assert len(fly_plans) >= 2
    for p in fly_plans:
        with open(p.slice_path) as f:
            s = json.load(f)
        assert set(s["daq_pvs"]) == {d.key for d in f2.daqs}


def test_daq_slice_gate_disabled_when_shutter_owns_it(fleet, tmp_path):
    """A DAQ whose gate address is owned by a shutter IOC must be sliced
    with gate=False so the standalone daq_ioc never opens the Arduino."""
    import json
    from pystxmcontrol.iocs.supervisor import plan_fleet
    plans = plan_fleet(fleet, str(tmp_path))
    shutter_addrs = {sh.address for sh in fleet.shutters}
    for p in plans:
        if p.module != "pystxmcontrol.iocs.daq_ioc":
            continue
        with open(p.slice_path) as f:
            s = json.load(f)
        if s["entry"].get("gate address") in shutter_addrs:
            assert not s["entry"].get("gate")
```

Delete/replace the old absorb tests (`test_plan_fleet_e712_absorbs_daqs`, `test_plan_fleet_npt_absorbs_daqs`, `test_plan_fleet_rejects_multiple_e712_groups` — exact names may differ; anything asserting absorption or the multi-fly ValueError goes). Update `test_plan_fleet_modules`'s expectations mechanically if your changes alter which modules appear (its ≥4 motor_ioc assertion fails pre-existing in this env — leave that assertion as-is unless your change alters the true count).

- [ ] **Step 2: Run to verify failures**

Run: `C:\Users\rp\PycharmProjects\ncs\.venv\Scripts\python.exe -m pytest tests\iocs\test_supervisor.py -q`
Expected: new tests FAIL (absorb logic still present, `daq_pvs` missing).

- [ ] **Step 3: Implement**

`pystxmcontrol/iocs/config.py` — `write_slice`: change the signature to `def write_slice(group, fleet, path, daq_pvs=None):`. In the `ControllerGroup` branch replace the `if daqs:` block with:

```python
        if daq_pvs:
            payload["daq_pvs"] = dict(daq_pvs)
```

In the `DaqEntry` branch, apply the gate override:

```python
    elif isinstance(group, DaqEntry):
        entry = dict(group.entry)
        if entry.get("gate") and any(sh.address == entry.get("gate address")
                                     for sh in fleet.shutters):
            # The Arduino gate/shutter is owned by the dedicated shutter IOC;
            # the standalone DAQ service must not open that serial port
            # (keysight.start would contend and hang). Beam gating happens
            # via the shutter IOC's CA PV, commanded by fly IOCs per line.
            entry = dict(entry, gate=False)
        payload = {"kind": "daq", "station": fleet.station, "key": group.key,
                   "entry": entry, "prefix": group.prefix,
                   "motor_pv": fleet.motor_pv}
```

`pystxmcontrol/iocs/supervisor.py` — `plan_fleet` becomes:

```python
def plan_fleet(fleet: FleetConfig, slice_dir: str,
               shutter_iocs: bool = True, startup_delay: float = 3.0) -> list[IocPlan]:
    Path(slice_dir).mkdir(parents=True, exist_ok=True)
    plans: list[IocPlan] = []
    # DAQ services FIRST: every fly IOC is a CA client of them.
    for d in fleet.daqs:
        p = str(Path(slice_dir) / f"daq_{d.key}.json")
        write_slice(d, fleet, p)
        plans.append(IocPlan(name=f"daq_{d.key}",
                             module="pystxmcontrol.iocs.daq_ioc", slice_path=p))
    daq_pvs = {d.key: d.prefix for d in fleet.daqs}
    for g in fleet.controller_groups:
        p = str(Path(slice_dir) / f"{g.label}.json")
        if g.controller_cls in FLY_CAPABLE_CONTROLLERS:
            write_slice(g, fleet, p, daq_pvs=daq_pvs)
            plans.append(IocPlan(name=g.label, module="pystxmcontrol.iocs.fly_ioc",
                                 slice_path=p))
        else:
            write_slice(g, fleet, p)
            plans.append(IocPlan(name=g.label, module="pystxmcontrol.iocs.motor_ioc",
                                 slice_path=p))
    if shutter_iocs:
        for sh in fleet.shutters:
            p = str(Path(slice_dir) / f"{sh.key}.json")
            write_slice(sh, fleet, p)
            plans.append(IocPlan(name=sh.key,
                                 module="pystxmcontrol.iocs.shutter_ioc", slice_path=p))
    for d in fleet.derived_remote:  # LAST: they are CA clients of the above
        p = str(Path(slice_dir) / f"derived_{d.key}.json")
        write_slice(d, fleet, p)
        plans.append(IocPlan(name=f"derived_{d.key}",
                             module="pystxmcontrol.iocs.derived_ioc",
                             slice_path=p, delay=startup_delay))
    return plans
```

(Keep the derived-IOC `delay=startup_delay` if that's what the current code does — read the existing tail of `plan_fleet` and preserve the delay/naming behavior for shutters/derived exactly; only the daq/fly/raise logic changes.)

- [ ] **Step 4: Run supervisor + serialization + fly tests**

Run: `C:\Users\rp\PycharmProjects\ncs\.venv\Scripts\python.exe -m pytest tests\iocs\test_supervisor.py tests\iocs\test_controller_serialization.py tests\iocs\test_mmc_fly.py -q`
Expected: all pass except the known env-dependent `test_plan_fleet_modules` shortfall if it still applies.

- [ ] **Step 5: Commit**

```bash
git add pystxmcontrol/iocs/supervisor.py pystxmcontrol/iocs/config.py tests/iocs/test_supervisor.py tests/iocs/test_controller_serialization.py
git commit -m "feat(supervisor): standalone DAQ services for all fleets; drop absorb + multi-fly raise"
```

---

### Task 4: multi-fly shared-DAQ integration test

**Files:**
- Test: `tests/iocs/test_multi_fly_shared_daq.py` (new)

**Interfaces:**
- Consumes: Tasks 1-2 (DAQ service + `FlyGroup(daq_pvs=...)`).

- [ ] **Step 1: Write the test**

```python
"""Two fly IOG groups sharing ONE DAQ service: serialized use works; a
line on B while A is mid-line fails fast with the busy rejection."""
import time

import pytest

DAQ_PREFIX = "STXMSIM:SHARED"
FLY_A = "STXMSIM:MMCA:FLY"
FLY_B = "STXMSIM:MMCB:FLY"


@pytest.fixture
def two_fly_ioc(ioc_harness):
    from pystxmcontrol.iocs.base import build_controller, build_motor
    from pystxmcontrol.iocs.daq_ioc import build_pvdb_for_entry
    from pystxmcontrol.iocs.fly_ioc import FlyGroup

    daq_entry = {"name": "Counter1", "driver": "keysight53230A",
                 "address": "sim", "port": 5025, "channel": 1, "ndim": 0,
                 "gate": False, "record": True, "simulation": True}
    pvdb, _ = build_pvdb_for_entry(daq_entry, DAQ_PREFIX)

    entry = {"axis": "x", "minValue": -50.0, "maxValue": 50.0, "offset": 0.0,
             "units": 1.0, "max velocity": 1000.0, "simulation": 1}
    for label, prefix in (("A", FLY_A), ("B", FLY_B)):
        ctrl = build_controller({"controller": "mmcController",
                                 "address": f"COM9{label}", "port": 0,
                                 "simulation": True})
        mx = build_motor("mmcMotor", ctrl, dict(entry), "x")
        my = build_motor("mmcMotor", ctrl, dict(entry, axis="y"), "y")
        fly = FlyGroup(prefix, motors={"x": mx, "y": my},
                       daq_pvs={"shared": DAQ_PREFIX}, simulation=True)
        pvdb.update(fly.pvdb)
    ioc_harness.start(pvdb)
    return ioc_harness, ioc_harness.client()


def _pvs(ctx, *names):
    pvs = ctx.get_pvs(*names, timeout=15)
    for pv in pvs:
        pv.wait_for_connection(timeout=15)
    return pvs


def _setup(ctx, prefix, npts, dwell_ms):
    start, stop, np_, dw, arm = _pvs(
        ctx, f"{prefix}:START", f"{prefix}:STOP", f"{prefix}:NPOINTS",
        f"{prefix}:DWELL", f"{prefix}:ARM")
    start.write(-5.0, wait=True); stop.write(5.0, wait=True)
    np_.write(npts, wait=True); dw.write(dwell_ms, wait=True)
    arm.write(1, wait=True, timeout=15)


def test_two_fly_groups_take_turns_on_one_daq(two_fly_ioc):
    _, ctx = two_fly_ioc
    (go_a,) = _pvs(ctx, f"{FLY_A}:GO")
    (go_b,) = _pvs(ctx, f"{FLY_B}:GO")
    (idx_a,) = _pvs(ctx, f"{FLY_A}:INDEX")
    (idx_b,) = _pvs(ctx, f"{FLY_B}:INDEX")
    _setup(ctx, FLY_A, 25, 1.0)
    go_a.write(1, wait=True, timeout=60)
    assert idx_a.read().data[0] == 1
    _setup(ctx, FLY_B, 25, 1.0)
    go_b.write(1, wait=True, timeout=60)
    assert idx_b.read().data[0] == 1


def test_contention_fails_fast_with_busy_error(two_fly_ioc):
    _, ctx = two_fly_ioc
    (go_a,) = _pvs(ctx, f"{FLY_A}:GO")
    (go_b,) = _pvs(ctx, f"{FLY_B}:GO")
    (state_b,) = _pvs(ctx, f"{FLY_B}:STATE")
    _setup(ctx, FLY_A, 200, 20.0)          # ~4 s sim line on A
    _setup(ctx, FLY_B, 25, 1.0)
    go_a.write(1, wait=False)               # fire-and-forget; A is now flying
    time.sleep(0.5)
    t0 = time.monotonic()
    go_b.write(1, wait=True, timeout=60)    # B must fail FAST (arm rejection)
    elapsed = time.monotonic() - t0
    assert state_b.read().data[0] == 3      # ERROR
    assert elapsed < 4.0                    # fast-fail, not a line-timeout
    # A's line finishes untouched
    (idx_a,) = _pvs(ctx, f"{FLY_A}:INDEX")
    t0 = time.monotonic()
    while idx_a.read().data[0] < 1 and time.monotonic() - t0 < 30:
        time.sleep(0.1)
    assert idx_a.read().data[0] == 1
```

- [ ] **Step 2: Run it**

Run: `C:\Users\rp\PycharmProjects\ncs\.venv\Scripts\python.exe -m pytest tests\iocs\test_multi_fly_shared_daq.py -q`
Expected: both pass. If `go_a.write(1, wait=False)` returns an unresolved future warning, that's fine — the test synchronizes via INDEX.

- [ ] **Step 3: Commit**

```bash
git add tests/iocs/test_multi_fly_shared_daq.py
git commit -m "test(fly): two fly groups share one DAQ service; contention fails fast"
```

---

### Task 5: performance benchmark (gate)

**Files:**
- Test: `tests/iocs/test_daq_service_perf.py` (new)

**Interfaces:**
- Consumes: Tasks 1-2. Gate from the spec: CA-hop component overhead ≤ 5 ms median on localhost.

- [ ] **Step 1: Write the benchmark test**

```python
"""Performance gate for the shared-DAQ CA hop (spec 2026-07-24, section 4).

Component gates (median over 20 reps, localhost):
  - DaqClient.arm() round-trip           <= 5 ms
  - DaqClient.read_line() waveform fetch <= 5 ms
End-to-end GO overhead is REPORTED (printed) with a generous pathology
ceiling only -- Windows timer granularity (~15.6 ms) sits inside both the
sim DAQ's asyncio.sleep and the fly loop's 20 ms poll, so a tight
end-to-end gate would measure the OS clock, not the CA hop.
"""
import statistics
import time

import pytest

DAQ_PREFIX = "STXMSIM:PERF"
FLY_PREFIX = "STXMSIM:PERFFLY:FLY"
N, DWELL_MS, REPS = 50, 1.0, 20


@pytest.fixture
def perf_ioc(ioc_harness):
    from pystxmcontrol.iocs.base import build_controller, build_motor
    from pystxmcontrol.iocs.daq_ioc import build_pvdb_for_entry
    from pystxmcontrol.iocs.fly_ioc import FlyGroup

    daq_entry = {"name": "Counter1", "driver": "keysight53230A",
                 "address": "sim", "port": 5025, "channel": 1, "ndim": 0,
                 "gate": False, "record": True, "simulation": True}
    pvdb, _ = build_pvdb_for_entry(daq_entry, DAQ_PREFIX)
    ctrl = build_controller({"controller": "mmcController", "address": "COM97",
                             "port": 0, "simulation": True})
    entry = {"axis": "x", "minValue": -50.0, "maxValue": 50.0, "offset": 0.0,
             "units": 1.0, "max velocity": 1000.0, "simulation": 1}
    mx = build_motor("mmcMotor", ctrl, dict(entry), "x")
    fly = FlyGroup(FLY_PREFIX, motors={"x": mx}, daq_pvs={"d": DAQ_PREFIX},
                   simulation=True)
    pvdb.update(fly.pvdb)
    ioc_harness.start(pvdb)
    return ioc_harness, ioc_harness.client()


def test_ca_hop_component_overhead(perf_ioc):
    from caproto.threading.client import Context
    from pystxmcontrol.iocs.fly_ioc import DaqClient
    _, _ctx_unused = perf_ioc
    ctx = Context()
    client = DaqClient(DAQ_PREFIX, ctx)
    client.configure(DWELL_MS, N, "EXT")

    arm_times, read_times = [], []
    for _ in range(REPS):
        t0 = time.perf_counter()
        client.arm()
        arm_times.append(time.perf_counter() - t0)
        deadline = time.monotonic() + 10.0
        assert client.wait_line(deadline)
        t0 = time.perf_counter()
        line = client.read_line(N)
        read_times.append(time.perf_counter() - t0)
        assert len(line) == N

    arm_med = statistics.median(arm_times) * 1000
    read_med = statistics.median(read_times) * 1000
    print(f"\n[perf] arm median {arm_med:.2f} ms  "
          f"(min {min(arm_times)*1000:.2f}, max {max(arm_times)*1000:.2f})")
    print(f"[perf] read median {read_med:.2f} ms  "
          f"(min {min(read_times)*1000:.2f}, max {max(read_times)*1000:.2f})")
    assert arm_med <= 5.0, f"ARM CA round-trip median {arm_med:.2f} ms > 5 ms gate"
    assert read_med <= 5.0, f"WF read median {read_med:.2f} ms > 5 ms gate"


def test_end_to_end_line_overhead_reported(perf_ioc):
    _, ctx = perf_ioc
    names = [f"{FLY_PREFIX}:{s}" for s in
             ("START", "STOP", "NPOINTS", "DWELL", "ARM", "GO", "INDEX")]
    pvs = ctx.get_pvs(*names, timeout=15)
    for pv in pvs:
        pv.wait_for_connection(timeout=15)
    start, stop, npts, dwell, arm, go, index = pvs
    start.write(-5.0, wait=True); stop.write(5.0, wait=True)
    npts.write(N, wait=True); dwell.write(DWELL_MS, wait=True)
    arm.write(1, wait=True, timeout=15)
    nominal = N * DWELL_MS / 1000.0
    overheads = []
    for _ in range(REPS):
        t0 = time.perf_counter()
        go.write(1, wait=True, timeout=60)
        overheads.append(time.perf_counter() - t0 - nominal)
    med = statistics.median(overheads) * 1000
    print(f"\n[perf] end-to-end line overhead median {med:.1f} ms over "
          f"{REPS} lines (n={N}, dwell={DWELL_MS} ms; includes fly-loop "
          f"20 ms poll + OS timer granularity, NOT just the CA hop)")
    assert med < 200.0, f"pathological per-line overhead: {med:.1f} ms"
```

- [ ] **Step 2: Run it and read the numbers**

Run: `C:\Users\rp\PycharmProjects\ncs\.venv\Scripts\python.exe -m pytest tests\iocs\test_daq_service_perf.py -q -s`
Expected: both pass; the printed medians are the deliverable — copy them into your report. If `test_ca_hop_component_overhead` misses the 5 ms gate, do NOT loosen the gate: report BLOCKED with the measured distribution (the design's mitigations — persistent context, subscribe-once, write-if-changed — are already in; a miss means something structural).

- [ ] **Step 3: Commit**

```bash
git add tests/iocs/test_daq_service_perf.py
git commit -m "test(perf): CA-hop component gate (<=5ms) + end-to-end line overhead report"
```

---

### Task 6: docs

**Files:**
- Modify: `pystxmcontrol/iocs/README.md` (DAQ ownership / fly section)
- Modify: `docs/superpowers/2026-07-12-caproto-iocs-followups.md`

**Interfaces:** none (docs only).

- [ ] **Step 1: README**

In `pystxmcontrol/iocs/README.md`: find the section describing the fly IOC absorbing DAQ entries (search "absorb") and rewrite it: every DAQ runs as a standalone `daq_ioc` CA service exposing point mode (`:ACQUIRE`) and line mode (`:LINE:NPOINTS/:LINE:TRIGGER/:LINE:ARM/:LINE:INDEX/:LINE:ABORT/:LINE:STATUS/:LINE:ERROR`, contract: `:COUNTS:WF` before `:LINE:INDEX`); fly IOCs are CA clients (any number of fly-capable controllers share all detectors, one line at a time per detector — contention is rejected with `:LINE:ERROR`); the DAQ service auto-disarms via watchdog (`max(5 s, 4·n·dwell + 5 s)`); supervisor starts DAQ services before controller IOCs.

- [ ] **Step 2: Follow-ups doc**

In `docs/superpowers/2026-07-12-caproto-iocs-followups.md`, mark the two retired items (strike-through with a trailing note, keep the lines for history):

- "Hardware fly branch restructure..." → append: `-- RESOLVED 2026-07-24 (shared DAQ service: fly loop arms all DAQs, flies once; see specs/2026-07-24-shared-daq-service-design.md)`
- "Multi-E712 DAQ mapping..." → append: `-- RESOLVED 2026-07-24 (DAQs are standalone CA services; plan_fleet no longer absorbs or raises)`

- [ ] **Step 3: Commit**

```bash
git add pystxmcontrol/iocs/README.md docs/superpowers/2026-07-12-caproto-iocs-followups.md
git commit -m "docs(iocs): shared DAQ service — README + follow-ups resolved"
```

---

## Final verification (whole suite)

Run: `C:\Users\rp\PycharmProjects\ncs\.venv\Scripts\python.exe -m pytest tests\iocs -q`
Expected green except the documented pre-existing env failures. The e2e ophyd fly test (`test_e2e_ophyd.py`) exercises the supervisor path — if it builds fly slices with `daqs`, it needed updating in Task 3; verify it passes or skips consistently.
