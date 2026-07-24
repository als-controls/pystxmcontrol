# XPS Modernization + Fly Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rewrite `xpsController`/`xpsMotor` as a clean, lock-agnostic driver pair preserving David's device-interaction semantics verbatim, add the mmcMotor-contract fly interface (constant-velocity, `line_trigger="IMM"`), and register the XPS as fly-capable.

**Architecture:** Structure-only rewrite: one `_transact` framing site with typed `XPSError` (no `eval()`, no `[-2,'']` sentinels), David's control+monitor dual sockets, relative-move + position-tolerance-poll completion, disable/enable abort. Fly = `prepareLine`/`moveLine` split calling `moveTo` at line velocity. Spec: `docs/superpowers/specs/2026-07-24-xps-fly-integration-design.md` — its "Itemized device-interaction weaknesses" are PRESERVED behaviors, not bugs to fix.

**Tech Stack:** Python, sockets, caproto (IOC layer, unchanged), pytest.

## Global Constraints

- Repo/worktree: `C:\Users\rp\PycharmProjects\ncs\lightfall-pystxmcontrol\_pystxmcontrol_iocs_wt`, branch `feature/caproto-iocs`. CWD for all commands.
- Test command: `C:\Users\rp\PycharmProjects\ncs\.venv\Scripts\python.exe -m pytest <paths> -q` — NEVER bare `pytest`.
- Under `pystxmcontrol/drivers/` touch ONLY `xpsController.py` and `xpsMotor.py`. No other driver files.
- **Preserve David's device-interaction semantics** (spec's guiding rule): relative moves composed from current position; completion = position within tolerance (default 5.0 controller units, config `"position_tolerance"`); move timeout default `config.get("timeout", 1)` (legacy default 1 s — small, but faithful); abort = `GroupMotionDisable` + 1 s sleep + `GroupMotionEnable` + 1 s sleep; `setAxisParams` keeps the legacy `velocity * 1000` scale. Do NOT "fix" these.
- Drivers are lock-agnostic: no `threading.Lock` in the two driver files.
- caproto gotchas: putter rejections must be no-op + error PV (threading client hangs on ErrorResponse); enum writes by index or `ChannelType.STRING`; `ioc_harness` fixture from `tests/iocs/conftest.py` handles CA env.
- Pre-existing env failures to IGNORE: `tests/iocs/test_config.py::test_motor_pv_naming`, `::test_colocated_derived`, `tests/iocs/test_e712_fly.py` errors (no pipython), `tests/iocs/test_npt_read_framing.py` collection (no pylibftdi). `tests/iocs/test_supervisor.py::test_plan_fleet_modules` is env-shortfall-prone AND its expectations legitimately change in Task 3 (XPS moves from motor_ioc to fly_ioc) — Task 3 updates it.
- `git add` explicit paths only. Every commit message ends with:
  `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>` and
  `Claude-Session: https://claude.ai/code/session_01BywLtkc8dZK3VAPQ4EBeg7`

---

### Task 1: xpsController transaction layer

**Files:**
- Modify (full rewrite): `pystxmcontrol/drivers/xpsController.py`
- Test: `tests/iocs/test_xps_driver.py` (new)

**Interfaces:**
- Produces: `XPSError(IOError)`; `xpsController(address="192.168.168.253", port=5001, simulation=False)` with `initialize(simulation=False)`; sockets `._control` (moves, SGamma set, disable/enable) and `._monitor` (queries) — tests inject fakes there; `_transact(sock, command, timeout=None) -> str payload` (raises `XPSError` on socket error/timeout/malformed/nonzero code); `move_relative(group, displacement)` (fire on control socket, best-effort 1 s reply read, timeout swallowed — David's flow); `get_position(group) -> float` (monitor); `get_sgamma(positioner) -> list[float] (len 4)`; `set_sgamma(positioner, velocity, acceleration, min_jerk, max_jerk)`; `abort_move(group)` (disable + 1 s + enable + 1 s); `disable_group(group)` / `enable_group(group)`; sim dicts `positions: dict[str, float]` (by group) and `sgamma: dict[str, list[float]]` (by positioner).

- [ ] **Step 1: Write the failing tests**

Create `tests/iocs/test_xps_driver.py`:

```python
"""Unit tests for the rewritten Newport XPS driver (fake sockets, no I/O)."""
import pytest


class FakeXPSSocket:
    """Scripted socket: records sends, replays canned replies.

    Each queued reply may be a str (returned whole) or a tuple of str
    fragments (returned across successive recv calls, to exercise the
    read-until-EndOfAPI reassembly). An empty queue raises socket.timeout
    to mimic a silent controller.
    """
    def __init__(self, replies=(), default=None):
        self.sent = []
        self._chunks = []
        self.default = default   # reply repeated forever once queue empties
        self.timeouts = []       # settimeout history
        for r in replies:
            if isinstance(r, tuple):
                self._chunks.extend(r)
            else:
                self._chunks.append(r)

    def send(self, data: bytes):
        self.sent.append(data.decode())
        return len(data)

    def recv(self, n: int) -> bytes:
        import socket as _socket
        if not self._chunks:
            if self.default is not None:
                return self.default.encode()
            raise _socket.timeout()
        return self._chunks.pop(0).encode()

    def settimeout(self, t):
        self.timeouts.append(t)

    def gettimeout(self):
        return 1.0


def make_controller(control_replies=(), monitor_replies=()):
    from pystxmcontrol.drivers.xpsController import xpsController
    ctrl = xpsController(address="10.0.0.1")
    ctrl.simulation = False
    ctrl._control = FakeXPSSocket(control_replies)
    ctrl._monitor = FakeXPSSocket(monitor_replies)
    return ctrl


def test_transact_frames_and_parses_payload():
    ctrl = make_controller(monitor_replies=["0,12.500000,EndOfAPI"])
    payload = ctrl._transact(ctrl._monitor, "GroupPositionCurrentGet(G1,double *)")
    assert payload == "12.500000"
    assert ctrl._monitor.sent == ["GroupPositionCurrentGet(G1,double *)"]


def test_transact_reassembles_fragmented_reply():
    ctrl = make_controller(monitor_replies=[("0,3.14", "1592,EndOf", "API")])
    assert ctrl._transact(ctrl._monitor, "Q") == "3.141592"


def test_transact_nonzero_code_raises():
    from pystxmcontrol.drivers.xpsController import XPSError
    ctrl = make_controller(monitor_replies=["-22,,EndOfAPI"])
    with pytest.raises(XPSError, match="-22"):
        ctrl._transact(ctrl._monitor, "Q")


def test_transact_timeout_raises():
    from pystxmcontrol.drivers.xpsController import XPSError
    ctrl = make_controller()  # empty queue -> socket.timeout
    with pytest.raises(XPSError, match="timeout"):
        ctrl._transact(ctrl._monitor, "Q")


def test_transact_malformed_errcode_raises():
    from pystxmcontrol.drivers.xpsController import XPSError
    ctrl = make_controller(monitor_replies=["garbage,EndOfAPI"])
    with pytest.raises(XPSError, match="malformed"):
        ctrl._transact(ctrl._monitor, "Q")


def test_get_position_parses_float():
    ctrl = make_controller(monitor_replies=["0,-7.250000,EndOfAPI"])
    assert ctrl.get_position("G1") == pytest.approx(-7.25)
    assert "GroupPositionCurrentGet(G1,double *)" in ctrl._monitor.sent[0]


def test_get_sgamma_parses_four_floats_without_eval():
    ctrl = make_controller(control_replies=["0,10.0,80.0,0.02,0.04,EndOfAPI"])
    assert ctrl.get_sgamma("G1.P") == pytest.approx([10.0, 80.0, 0.02, 0.04])


def test_set_sgamma_frames_command():
    ctrl = make_controller(control_replies=["0,,EndOfAPI"])
    ctrl.set_sgamma("G1.P", 5.0, 80.0, 0.02, 0.04)
    assert ctrl._control.sent[-1] == \
        "PositionerSGammaParametersSet(G1.P,5.0,80.0,0.02,0.04)"


def test_move_relative_swallows_reply_timeout():
    # David's flow: the move reply arrives at motion END; a 1 s read timeout
    # is NOT an error -- completion is polled on the monitor socket.
    ctrl = make_controller()  # no reply queued -> recv times out
    ctrl.move_relative("G1", 2.5)
    assert ctrl._control.sent == ["GroupMoveRelative(G1,2.5)"]


def test_move_relative_surfaces_immediate_error_reply():
    from pystxmcontrol.drivers.xpsController import XPSError
    ctrl = make_controller(control_replies=["-17,,EndOfAPI"])  # e.g. disabled
    with pytest.raises(XPSError, match="-17"):
        ctrl.move_relative("G1", 2.5)


def test_abort_move_is_disable_sleep_enable(monkeypatch):
    import pystxmcontrol.drivers.xpsController as mod
    sleeps = []
    monkeypatch.setattr(mod.time, "sleep", lambda s: sleeps.append(s))
    ctrl = make_controller(control_replies=["0,,EndOfAPI", "0,,EndOfAPI"])
    ctrl.abort_move("G1")
    assert ctrl._control.sent == ["GroupMotionDisable(G1)", "GroupMotionEnable(G1)"]
    assert sleeps == [1, 1]


def test_simulation_initialize_opens_no_sockets():
    from pystxmcontrol.drivers.xpsController import xpsController
    ctrl = xpsController(address="10.0.0.1")
    ctrl.initialize(simulation=True)
    assert ctrl.simulation is True
    assert ctrl._control is None and ctrl._monitor is None
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `C:\Users\rp\PycharmProjects\ncs\.venv\Scripts\python.exe -m pytest tests\iocs\test_xps_driver.py -q`
Expected: FAIL/ERROR — legacy module has no `XPSError`, `_transact`, etc.

- [ ] **Step 3: Rewrite the controller**

Replace `pystxmcontrol/drivers/xpsController.py` entirely:

```python
"""Newport XPS controller driver (TCP, port 5001).

Structure-only rewrite of David's driver: same XPS function calls, same
dual-socket flow (control socket for motion/parameter commands, monitor
socket for position polling), same disable/enable abort. What changed is
code structure: ONE framing/parsing site (_transact) with typed XPSError
(no [-2, ''] sentinels, no eval()), lock-agnostic (the IOC layer's
io_lock serializes), and a working simulation mode.

Known device-interaction quirks are deliberately preserved -- see the
spec's "Itemized device-interaction weaknesses"
(docs/superpowers/specs/2026-07-24-xps-fly-integration-design.md).
"""
import socket
import time

from pystxmcontrol.controller.hardwareController import hardwareController


class XPSError(IOError):
    """Socket failure, malformed reply, or nonzero XPS error code."""


class xpsController(hardwareController):

    def __init__(self, address="192.168.168.253", port=5001, simulation=False):
        self.address = address
        self.port = int(port) if port else 5001
        self.simulation = simulation
        self._control = None   # motion commands, SGamma set, disable/enable
        self._monitor = None   # position queries during moves
        # Simulation state shared by all xpsMotor instances:
        self.positions = {}    # group -> position (controller units)
        self.sgamma = {}       # positioner -> [vel, accel, minJerk, maxJerk]

    def initialize(self, simulation=False):
        self.simulation = simulation
        if self.simulation:
            return
        print(f"Connecting to XPS controller on {self.address}:{self.port}",
              flush=True)
        self._control = self._open_socket()
        self._monitor = self._open_socket()

    def _open_socket(self):
        try:
            sock = socket.create_connection((str(self.address), self.port),
                                            timeout=5.0)
        except OSError as exc:
            raise XPSError(
                f"failed to connect to XPS on {self.address}:{self.port}: {exc}")
        sock.settimeout(1.0)
        return sock

    # ---- framing/parsing: the ONE place XPS wire format lives ----------
    def _transact(self, sock, command, timeout=None) -> str:
        """Send a command and read its full ``err,payload,EndOfAPI`` reply.

        Raises XPSError on socket error/timeout, malformed reply, or a
        nonzero XPS error code. ``timeout`` temporarily overrides the
        socket timeout for this transaction.
        """
        if self.simulation:
            raise XPSError("_transact has no meaning in simulation mode")
        old = sock.gettimeout()
        if timeout is not None:
            sock.settimeout(timeout)
        try:
            sock.send(command.encode())
            response = ""
            while ",EndOfAPI" not in response:
                chunk = sock.recv(1024).decode(errors="replace")
                if not chunk:
                    raise XPSError(f"connection closed during {command!r}")
                response += chunk
        except socket.timeout:
            raise XPSError(f"timeout waiting for reply to {command!r}")
        except OSError as exc:
            raise XPSError(f"socket error during {command!r}: {exc}")
        finally:
            if timeout is not None:
                sock.settimeout(old)
        err_str, _, rest = response.partition(",")
        payload = rest[: rest.rfind(",EndOfAPI")] if rest.rfind(",EndOfAPI") >= 0 \
            else rest[: rest.rfind("EndOfAPI")]
        try:
            err = int(err_str)
        except ValueError:
            raise XPSError(f"malformed XPS reply to {command!r}: {response!r}")
        if err != 0:
            raise XPSError(f"XPS error {err} for {command!r}: {payload!r}")
        return payload

    # ---- protocol wrappers (legacy XPS function set, unchanged) ---------
    def move_relative(self, group, displacement):
        """Fire GroupMoveRelative on the control socket.

        David's flow: the XPS answers a motion command only when the move
        COMPLETES, so we attempt a short (socket-default 1 s) reply read --
        an immediate error reply (e.g. group disabled) surfaces as
        XPSError, while a read timeout means "move in progress" and is
        swallowed; completion is polled via get_position on the monitor
        socket by the motor. (Spec weakness #6: the eventual reply is left
        unread; the next control-socket _transact may need to tolerate it
        -- preserved behavior.)
        """
        if self.simulation:
            self.positions[group] = self.positions.get(group, 0.0) + displacement
            return
        try:
            self._transact(self._control,
                           f"GroupMoveRelative({group},{displacement})")
        except XPSError as exc:
            if "timeout waiting for reply" in str(exc):
                return  # move in progress; completion is polled
            raise

    def get_position(self, group) -> float:
        if self.simulation:
            return self.positions.get(group, 0.0)
        payload = self._transact(
            self._monitor, f"GroupPositionCurrentGet({group},double *)")
        try:
            return float(payload.split(",")[0])
        except ValueError:
            raise XPSError(f"unparseable position payload {payload!r}")

    def get_sgamma(self, positioner) -> list:
        if self.simulation:
            return list(self.sgamma.get(positioner, [10.0, 80.0, 0.02, 0.04]))
        payload = self._transact(
            self._control,
            f"PositionerSGammaParametersGet({positioner},double *,double *,"
            f"double *,double *)")
        try:
            values = [float(v) for v in payload.split(",")]
        except ValueError:
            raise XPSError(f"unparseable SGamma payload {payload!r}")
        if len(values) != 4:
            raise XPSError(f"expected 4 SGamma values, got {payload!r}")
        return values

    def set_sgamma(self, positioner, velocity, acceleration, min_jerk, max_jerk):
        if self.simulation:
            self.sgamma[positioner] = [float(velocity), float(acceleration),
                                       float(min_jerk), float(max_jerk)]
            return
        self._transact(
            self._control,
            f"PositionerSGammaParametersSet({positioner},{velocity},"
            f"{acceleration},{min_jerk},{max_jerk})")

    def disable_group(self, group):
        if not self.simulation:
            self._transact(self._control, f"GroupMotionDisable({group})")

    def enable_group(self, group):
        if not self.simulation:
            self._transact(self._control, f"GroupMotionEnable({group})")

    def abort_move(self, group):
        """David's abort: disable, 1 s, enable, 1 s. (Spec weakness #1:
        GroupMoveAbort is the candidate improvement -- NOT used yet.)"""
        self.disable_group(group)
        time.sleep(1)
        self.enable_group(group)
        time.sleep(1)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `C:\Users\rp\PycharmProjects\ncs\.venv\Scripts\python.exe -m pytest tests\iocs\test_xps_driver.py -q`
Expected: 12 passed.

- [ ] **Step 5: Commit**

```bash
git add pystxmcontrol/drivers/xpsController.py tests/iocs/test_xps_driver.py
git commit -m "feat(xps): rewrite xpsController as typed transaction layer (David's wire flow preserved)"
```

---

### Task 2: xpsMotor rewrite (point-to-point + fly)

**Files:**
- Modify (full rewrite): `pystxmcontrol/drivers/xpsMotor.py`
- Test: append to `tests/iocs/test_xps_driver.py`

**Interfaces:**
- Consumes: Task 1's controller API (exact names above).
- Produces: `xpsMotor(controller=None, config=None)` with `connect(axis=, **kwargs)` (axis = `"Group.Positioner"`; sets `.group`; latches SGamma when not simulating), `checkLimits(pos)` → `SoftwareLimitError`, `moveTo(pos, timeout=None)` (relative move + monitor-poll completion; timeout default `config.get("timeout", 1)`; timeout → `abort_move` + `XPSError`), `getPos()`, `getStatus()` (driver-local flag), `stop()` (= `abort_move`), `setAxisParams(velocity=)` (legacy ×1000 preserved), `get_velocity()`, `moveBy(step)`, fly interface identical in contract to `mmcMotor`: class attr `line_trigger = "IMM"`, `trajectory_*` attrs, `update_trajectory(direction=...)`, `prepareLine()`, `moveLine()`, `npositions`, `line_velocity`; internal `_set_line_velocity(v)` writes SGamma velocity RAW (no ×1000).

- [ ] **Step 1: Write the failing tests**

Append to `tests/iocs/test_xps_driver.py`:

```python
XENTRY = {"axis": "G1.P", "minValue": -20.0, "maxValue": 20.0, "offset": 0.0,
          "units": 1.0, "max velocity": 50.0, "timeout": 5,
          "position_tolerance": 0.01, "simulation": 0}

SGAMMA_OK = "0,10.0,80.0,0.02,0.04,EndOfAPI"


def make_motor(control_replies=(), monitor_replies=(), entry=None):
    from pystxmcontrol.drivers.xpsMotor import xpsMotor
    # connect() latches SGamma (control) and initial position (monitor)
    ctrl = make_controller(
        control_replies=[SGAMMA_OK, *control_replies],
        monitor_replies=["0,0.000000,EndOfAPI", *monitor_replies])
    m = xpsMotor()
    m.controller = ctrl
    m.config = dict(XENTRY, **(entry or {}))
    m.connect(axis=m.config["axis"])
    return m, ctrl


def test_connect_splits_group_and_latches_sgamma():
    m, ctrl = make_motor()
    assert m.group == "G1" and m.axis == "G1.P"
    assert m.velocity == pytest.approx(10.0)
    assert m.acceleration == pytest.approx(80.0)


def test_check_limits_raises():
    from pystxmcontrol.controller.motor import SoftwareLimitError
    m, _ = make_motor()
    with pytest.raises(SoftwareLimitError):
        m.checkLimits(20.5)


def test_move_to_composes_relative_and_polls_to_tolerance():
    # monitor replies: pre-move position 1.0, then poll 4.9 (outside tol),
    # then 5.0 (inside) -> done.
    m, ctrl = make_motor(
        monitor_replies=["0,1.000000,EndOfAPI", "0,4.900000,EndOfAPI",
                         "0,5.000000,EndOfAPI"])
    m.moveTo(5.0)
    assert "GroupMoveRelative(G1,4.0)" in ctrl._control.sent


def test_move_to_timeout_aborts_and_raises(monkeypatch):
    from pystxmcontrol.drivers.xpsController import XPSError
    import pystxmcontrol.drivers.xpsController as cmod
    import pystxmcontrol.drivers.xpsMotor as mmod
    monkeypatch.setattr(cmod.time, "sleep", lambda s: None)
    monkeypatch.setattr(mmod.time, "sleep", lambda s: None)
    # position stuck at 0 forever (default reply repeats once queue empties)
    m, ctrl = make_motor(
        entry={"timeout": 0.05},
        control_replies=["0,,EndOfAPI", "0,,EndOfAPI"])  # disable/enable
    ctrl._monitor.default = "0,0.000000,EndOfAPI"
    with pytest.raises(XPSError, match="timed out"):
        m.moveTo(5.0)
    assert "GroupMotionDisable(G1)" in ctrl._control.sent


def test_stop_uses_davids_disable_enable(monkeypatch):
    import pystxmcontrol.drivers.xpsController as cmod
    monkeypatch.setattr(cmod.time, "sleep", lambda s: None)
    m, ctrl = make_motor(control_replies=["0,,EndOfAPI", "0,,EndOfAPI"])
    m.stop()
    assert ctrl._control.sent[-2:] == ["GroupMotionDisable(G1)",
                                       "GroupMotionEnable(G1)"]


def test_set_axis_params_keeps_legacy_x1000():
    m, ctrl = make_motor(control_replies=["0,,EndOfAPI"])
    m.setAxisParams(velocity=2.0)
    # legacy quirk preserved: velocity * 1000, accel/jerk from connect latch
    assert ctrl._control.sent[-1] == \
        "PositionerSGammaParametersSet(G1.P,2000.0,80.0,0.02,0.04)"


def test_simulation_move_and_readback():
    from pystxmcontrol.drivers.xpsController import xpsController
    from pystxmcontrol.drivers.xpsMotor import xpsMotor
    ctrl = xpsController(address="10.0.0.1")
    ctrl.initialize(simulation=True)
    m = xpsMotor()
    m.controller = ctrl
    m.config = dict(XENTRY, offset=1.0, units=2.0)
    m.connect(axis="G1.P")
    m.moveTo(7.0)
    assert m.getPos() == pytest.approx(7.0)
    assert m.getStatus() is False


# ---- fly interface ------------------------------------------------------

def test_xps_declares_immediate_trigger():
    from pystxmcontrol.drivers.xpsMotor import xpsMotor
    assert xpsMotor.line_trigger == "IMM"


def test_update_trajectory_velocity_and_guards():
    from pystxmcontrol.drivers.xpsController import XPSError
    m, _ = make_motor()
    m.trajectory_start = (-1.0, 0.0)
    m.trajectory_stop = (1.0, 0.0)
    m.trajectory_pixel_count = 20
    m.trajectory_pixel_dwell = 100.0     # 2 s line, span 2 -> 1.0 u/s
    m.update_trajectory()
    assert m.line_velocity == pytest.approx(1.0)
    assert (m._line_start, m._line_stop) == (-1.0, 1.0)
    assert m.npositions == 20
    m.trajectory_stop = (-1.0, 0.0)      # zero span
    with pytest.raises(XPSError):
        m.update_trajectory()
    m.trajectory_stop = (1.0, 0.0)
    m.trajectory_pixel_dwell = 0.1       # 2 units in 2 ms -> 1000 u/s > 50
    with pytest.raises(XPSError):
        m.update_trajectory()


def test_prepare_then_move_line_sequence_and_sgamma_restore():
    # cruise velocity = 10.0 (SGamma read); line velocity = 1.0.
    # Control-socket reply consumption order (after connect's SGAMMA_OK):
    #   get_velocity (SGAMMA_OK), move_relative-to-start ack, SGamma line
    #   set ack, move_relative-to-stop ack, SGamma restore ack.
    # NOTE move_relative attempts a 1 s reply read, so when a reply IS
    # queued it consumes one -- the sequences below account for that.
    m, ctrl = make_motor(
        control_replies=[SGAMMA_OK,           # prepareLine get_velocity
                         "0,,EndOfAPI",       # move_relative(start) ack
                         "0,,EndOfAPI",       # SGamma set (line velocity)
                         "0,,EndOfAPI",       # move_relative(stop) ack
                         "0,,EndOfAPI"],      # SGamma restore (cruise)
        monitor_replies=["0,0.000000,EndOfAPI", "0,-1.000000,EndOfAPI",
                         "0,-1.000000,EndOfAPI", "0,1.000000,EndOfAPI"])
    m.trajectory_start = (-1.0, 0.0)
    m.trajectory_stop = (1.0, 0.0)
    m.trajectory_pixel_count = 20
    m.trajectory_pixel_dwell = 100.0
    m.update_trajectory()
    m.prepareLine()
    assert m._prepared is True
    assert "PositionerSGammaParametersSet(G1.P,1.0,80.0,0.02,0.04)" \
        in ctrl._control.sent
    m.moveLine()
    assert m._prepared is False
    assert ctrl._control.sent[-1] == \
        "PositionerSGammaParametersSet(G1.P,10.0,80.0,0.02,0.04)"


def test_re_prepare_after_abort_keeps_original_cruise():
    # Control replies (after connect's SGAMMA_OK): prepare1 get_velocity,
    # prepare1 move ack + SGamma set; prepare2 (NO get_velocity: stash
    # guarded) move ack + SGamma set; moveLine move ack + restore.
    m, ctrl = make_motor(
        control_replies=[SGAMMA_OK,
                         "0,,EndOfAPI", "0,,EndOfAPI",   # prep1 ack + set
                         "0,,EndOfAPI", "0,,EndOfAPI",   # prep2 ack + set
                         "0,,EndOfAPI", "0,,EndOfAPI"],  # move ack + restore
        monitor_replies=["0,0.000000,EndOfAPI", "0,-1.000000,EndOfAPI",
                         "0,-1.000000,EndOfAPI", "0,-1.000000,EndOfAPI",
                         "0,-1.000000,EndOfAPI", "0,1.000000,EndOfAPI"])
    m.trajectory_start = (-1.0, 0.0)
    m.trajectory_stop = (1.0, 0.0)
    m.trajectory_pixel_count = 20
    m.trajectory_pixel_dwell = 100.0
    m.update_trajectory()
    m.prepareLine()
    assert m._cruise_velocity == pytest.approx(10.0)
    m.prepareLine()                       # re-prepare after aborted line
    assert m._cruise_velocity == pytest.approx(10.0)   # NOT the line velocity
    m.moveLine()
    assert ctrl._control.sent[-1] == \
        "PositionerSGammaParametersSet(G1.P,10.0,80.0,0.02,0.04)"


def test_move_line_sim_lands_on_stop():
    from pystxmcontrol.drivers.xpsController import xpsController
    from pystxmcontrol.drivers.xpsMotor import xpsMotor
    ctrl = xpsController(address="10.0.0.1")
    ctrl.initialize(simulation=True)
    m = xpsMotor()
    m.controller = ctrl
    m.config = dict(XENTRY)
    m.connect(axis="G1.P")
    m.trajectory_start = (-1.0, 0.0)
    m.trajectory_stop = (1.0, 0.0)
    m.trajectory_pixel_count = 20
    m.trajectory_pixel_dwell = 100.0
    m.update_trajectory()
    m.prepareLine()
    m.moveLine()
    assert m.getPos() == pytest.approx(1.0)
```

- [ ] **Step 2: Run to verify failure**

Run: `C:\Users\rp\PycharmProjects\ncs\.venv\Scripts\python.exe -m pytest tests\iocs\test_xps_driver.py -q`
Expected: Task 1 tests pass; new tests FAIL (legacy motor lacks the API).

- [ ] **Step 3: Rewrite the motor**

Replace `pystxmcontrol/drivers/xpsMotor.py` entirely:

```python
"""Newport XPS axis driver (point-to-point + constant-velocity fly lines).

David's device-interaction semantics preserved verbatim (see the spec's
itemized weaknesses): relative moves composed from the current position,
completion by position tolerance polled on the monitor socket, the
disable/enable abort, and setAxisParams' legacy velocity*1000 scale.
Fly lines (line_trigger = "IMM": free-running DAQ) reuse moveTo at the
line velocity -- SGamma velocity is set RAW for lines.
"""
import time

from pystxmcontrol.controller.motor import motor, SoftwareLimitError
from pystxmcontrol.drivers.xpsController import XPSError


class xpsMotor(motor):

    #: DAQ trigger source for fly lines (fly_ioc reads this); the XPS PVT
    #: trajectory + position-compare EXT trigger is a recorded follow-up.
    line_trigger = "IMM"

    def __init__(self, controller=None, config=None):
        self.controller = controller
        self.config = config
        self.simulation = False
        self.axis = None            # full "Group.Positioner"
        self.group = None
        self.moving = False
        self.velocity = 0.0
        self.acceleration = 0.0
        self.minimumJerkTime = 0.0
        self.maximumJerkTime = 0.0
        # fly interface (contract identical to mmcMotor)
        self.lineMode = "raster"
        self.trajectory_start = (0.0, 0.0)
        self.trajectory_stop = (0.0, 0.0)
        self.trajectory_pixel_count = 10
        self.trajectory_pixel_dwell = 1.0   # ms per pixel
        self.npositions = 10
        self.line_velocity = 0.0
        self._line_start = 0.0
        self._line_stop = 0.0
        self._prepared = False
        self._cruise_velocity = None
        self._poll = 0.1                    # legacy 100 ms position poll

    # ---- helpers --------------------------------------------------------
    def _to_controller(self, pos):
        return (pos - self.config["offset"]) / self.config["units"]

    def _from_controller(self, pos):
        return pos * self.config["units"] + self.config["offset"]

    @property
    def _tolerance(self):
        return float(self.config.get("position_tolerance", 5.0))

    # ---- point-to-point interface ---------------------------------------
    def connect(self, axis=None, **kwargs):
        if "logger" in kwargs:
            self.logger = kwargs["logger"]
        self.simulation = self.controller.simulation
        self.axis = axis
        self.group = axis.split(".")[0]
        (self.velocity, self.acceleration, self.minimumJerkTime,
         self.maximumJerkTime) = self.controller.get_sgamma(self.axis)
        if not self.simulation:
            self.position = self.getPos()
        return True

    def checkLimits(self, pos):
        lo, hi = self.config["minValue"], self.config["maxValue"]
        if pos < lo:
            self.moving = False
            raise SoftwareLimitError(self.axis, pos, lo, limit_type="lower")
        if pos > hi:
            self.moving = False
            raise SoftwareLimitError(self.axis, pos, hi, limit_type="upper")
        return True

    def getPos(self):
        return self._from_controller(self.controller.get_position(self.group))

    def getStatus(self, **kwargs):
        return self.moving

    def moveTo(self, pos, timeout=None):
        """David's semantics: relative move composed from the current
        position, completion = position within tolerance polled on the
        monitor socket, timeout -> disable/enable abort + raise."""
        self.checkLimits(pos)
        target = self._to_controller(pos)
        if self.simulation:
            self.controller.positions[self.group] = target
            self.moving = False
            return
        if timeout is None:
            timeout = float(self.config.get("timeout", 1))
        current = self.controller.get_position(self.group)
        self.moving = True
        try:
            self.controller.move_relative(self.group,
                                          round(target - current, 6))
            t0 = time.time()
            while abs(target - self.controller.get_position(self.group)) \
                    > self._tolerance:
                if time.time() - t0 > timeout:
                    self.controller.abort_move(self.group)
                    raise XPSError(
                        f"XPS axis {self.axis} move to {pos} timed out "
                        f"after {timeout}s")
                time.sleep(self._poll)
        finally:
            self.moving = False

    def moveBy(self, step):
        self.moveTo(self.getPos() + step)

    def stop(self):
        if not self.simulation:
            self.controller.abort_move(self.group)
        self.moving = False

    def setAxisParams(self, velocity):
        """Legacy quirk preserved: velocity is scaled x1000 on the wire
        (spec weakness #4). Fly lines use _set_line_velocity (raw)."""
        self.velocity = float(velocity)
        self.controller.set_sgamma(self.axis, self.velocity * 1000,
                                   self.acceleration, self.minimumJerkTime,
                                   self.maximumJerkTime)

    def get_velocity(self):
        (vel, self.acceleration, self.minimumJerkTime,
         self.maximumJerkTime) = self.controller.get_sgamma(self.axis)
        return vel

    def disable(self):
        self.controller.disable_group(self.group)

    def enable(self):
        self.controller.enable_group(self.group)

    # ---- fly interface (contract identical to mmcMotor) ------------------
    def _set_line_velocity(self, v):
        """RAW SGamma velocity write (no legacy x1000) for fly lines."""
        self.controller.set_sgamma(self.axis, v, self.acceleration,
                                   self.minimumJerkTime, self.maximumJerkTime)

    def update_trajectory(self, direction="forward", include_return=False):
        x0, y0 = self.trajectory_start
        x1, y1 = self.trajectory_stop
        if abs(x1 - x0) >= abs(y1 - y0):
            start, stop = x0, x1
        else:
            start, stop = y0, y1
        if direction == "backward":
            start, stop = stop, start
        if start == stop:
            raise XPSError("fly line has zero span")
        line_time = self.trajectory_pixel_count * \
            self.trajectory_pixel_dwell / 1000.0
        if line_time <= 0:
            raise XPSError("fly line has non-positive duration")
        velocity = abs(stop - start) / line_time / abs(self.config["units"])
        max_v = self.config.get("max velocity")
        if max_v and velocity > float(max_v):
            raise XPSError(
                f"fly line needs {velocity:.3f} units/s > max velocity "
                f"{max_v}; increase dwell or shorten the line")
        self._line_start, self._line_stop = start, stop
        self.line_velocity = velocity
        self.npositions = self.trajectory_pixel_count

    def prepareLine(self):
        """Pre-position + set line velocity BEFORE the DAQ is armed
        (line_trigger IMM: arming starts acquisition immediately)."""
        if self.simulation:
            self._prepared = True
            return
        # Abort window: a line aborted between prepareLine and moveLine
        # leaves the axis at line velocity with _prepared True until the
        # next completed line restores cruise. Only stash cruise when not
        # already prepared, so a re-prepare keeps the ORIGINAL cruise.
        if not self._prepared:
            self._cruise_velocity = self.get_velocity()
        self.moveTo(self._line_start)
        self._set_line_velocity(self.line_velocity)
        self._prepared = True

    def moveLine(self, **kwargs):
        line_time = self.trajectory_pixel_count * \
            self.trajectory_pixel_dwell / 1000.0
        deadline = max(5.0, line_time * 4.0 + 5.0)
        if self.simulation:
            time.sleep(min(line_time, 0.1))
            self.controller.positions[self.group] = \
                self._to_controller(self._line_stop)
            return
        if not self._prepared:
            self._cruise_velocity = self.get_velocity()
            self.moveTo(self._line_start)
            self._set_line_velocity(self.line_velocity)
        try:
            self.moveTo(self._line_stop, timeout=deadline)
        finally:
            self._prepared = False
            try:
                if self._cruise_velocity is not None:
                    self._set_line_velocity(self._cruise_velocity)
            except Exception as exc:  # noqa: BLE001 - never mask the line error
                print(f"[xpsMotor] cruise velocity restore failed: {exc!r}",
                      flush=True)
```

- [ ] **Step 4: Run to verify pass**

Run: `C:\Users\rp\PycharmProjects\ncs\.venv\Scripts\python.exe -m pytest tests\iocs\test_xps_driver.py -q`
Expected: all pass (24).

- [ ] **Step 5: Commit**

```bash
git add pystxmcontrol/drivers/xpsMotor.py tests/iocs/test_xps_driver.py
git commit -m "feat(xps): modern xpsMotor with David's move semantics + constant-velocity fly"
```

---

### Task 3: fly registration + supervisor test expectations

**Files:**
- Modify: `pystxmcontrol/iocs/config.py` (FLY_CAPABLE_CONTROLLERS)
- Modify: `tests/iocs/test_supervisor.py` (`test_plan_fleet_modules` — XPS moves from motor_ioc to fly_ioc)
- Test: `tests/iocs/test_xps_fly.py` (new, registration + slice build)

**Interfaces:**
- Consumes: Tasks 1-2 drivers; existing `fly_ioc.build_pvdb_from_slice` (slice keys `daq_pvs`, `shutters`).
- Produces: `"xpsController"` in `FLY_CAPABLE_CONTROLLERS`; supervisor routes XPS groups to `pystxmcontrol.iocs.fly_ioc`.

- [ ] **Step 1: Write the failing tests**

Create `tests/iocs/test_xps_fly.py`:

```python
"""XPS fly-capability registration + slice build (no hardware)."""
import pytest


def test_xps_is_fly_capable():
    from pystxmcontrol.iocs.config import FLY_CAPABLE_CONTROLLERS
    assert "xpsController" in FLY_CAPABLE_CONTROLLERS


def test_fly_slice_builds_for_xps(tmp_path):
    import json
    from pystxmcontrol.iocs.fly_ioc import build_pvdb_from_slice
    entry = {"type": "primary", "driver": "xpsMotor",
             "controller": "xpsController", "controllerID": "10.0.0.1",
             "axis": "G1.P", "minValue": -20.0, "maxValue": 20.0,
             "offset": 0.0, "units": 1.0, "max velocity": 50.0,
             "simulation": 1}
    s = {"kind": "controller", "station": "SIM", "controller_id": "10.0.0.1",
         "controller_cls": "xpsController", "label": "XPS", "port": 5001,
         "simulation": True,
         "motors": [{"key": "CoarseX", "entry": entry,
                     "pv": "STXMSIM:XPS:CoarseX"}],
         "derived": [], "motor_pv": {"CoarseX": "STXMSIM:XPS:CoarseX"},
         "daq_pvs": {"default": "STXMSIM:DEFAULT"}}
    p = tmp_path / "slice.json"
    p.write_text(json.dumps(s))
    from pystxmcontrol.iocs.config import read_slice
    pvdb = build_pvdb_from_slice(read_slice(str(p)))
    names = set(pvdb)
    assert "STXMSIM:XPS:CoarseX" in names
    assert "STXMSIM:XPS:FLY:GO" in names
```

- [ ] **Step 2: Run to verify failure**

Run: `C:\Users\rp\PycharmProjects\ncs\.venv\Scripts\python.exe -m pytest tests\iocs\test_xps_fly.py -q`
Expected: FAIL — `xpsController` not in `FLY_CAPABLE_CONTROLLERS`.

- [ ] **Step 3: Implement**

In `pystxmcontrol/iocs/config.py`, extend the set:

```python
FLY_CAPABLE_CONTROLLERS = {"E712Controller", "nptController",
                           "mmcController", "xpsController"}
```

In `tests/iocs/test_supervisor.py::test_plan_fleet_modules`, update the
expectations: the shipped config's XPS group now routes to `fly_ioc`, so
move `'XPS'` from the `motor_ioc` expectation to a fly_ioc assertion.
Read the test first; the shape after editing should be:

```python
        # shipped config: XPS is fly-capable -> fly_ioc; XER groups (and MCL
        # where its driver is importable) stay on motor_ioc
        assert "XPS" in by_module.get("pystxmcontrol.iocs.fly_ioc", [])
        assert len(by_module.get("pystxmcontrol.iocs.motor_ioc", [])) >= 2
```

(The old `>= 4` count included XPS and the env-dependent MCL; `>= 2`
covers the XER pair that is always importable. Keep the rest of the test
unchanged.)

- [ ] **Step 4: Run tests**

Run: `C:\Users\rp\PycharmProjects\ncs\.venv\Scripts\python.exe -m pytest tests\iocs\test_xps_fly.py tests\iocs\test_supervisor.py tests\iocs\test_xps_driver.py -q`
Expected: all pass — including `test_plan_fleet_modules`, which stops being
an env failure once its expectations match the new routing.

- [ ] **Step 5: Commit**

```bash
git add pystxmcontrol/iocs/config.py tests/iocs/test_xps_fly.py tests/iocs/test_supervisor.py
git commit -m "feat(xps): register xpsController as fly-capable; update plan_fleet expectations"
```

---

### Task 4: IOC-level simulation tests over real CA

**Files:**
- Test: append to `tests/iocs/test_xps_fly.py`

**Interfaces:**
- Consumes: everything above; `ioc_harness` from `tests/iocs/conftest.py`; DAQ service via `daq_ioc.build_pvdb_for_entry` (same-process pvdb, reached over CA loopback — pattern of `tests/iocs/test_mmc_fly.py`).

- [ ] **Step 1: Write the tests**

Append to `tests/iocs/test_xps_fly.py`:

```python
FLY_PREFIX = "STXMSIM:XPS:FLY"


@pytest.fixture
def xps_fly_ioc(ioc_harness):
    from pystxmcontrol.iocs.base import (MotorRecordGroup, build_controller,
                                         build_motor)
    from pystxmcontrol.iocs.daq_ioc import build_pvdb_for_entry
    from pystxmcontrol.iocs.fly_ioc import FlyGroup

    ctrl = build_controller({"controller": "xpsController",
                             "address": "10.0.0.1", "port": 5001,
                             "simulation": True})
    entry = {"axis": "G1.P", "minValue": -50.0, "maxValue": 50.0,
             "offset": 0.0, "units": 1.0, "max velocity": 1000.0,
             "simulation": 1}
    mx = build_motor("xpsMotor", ctrl, dict(entry), "G1.P")
    my = build_motor("xpsMotor", ctrl, dict(entry, axis="G2.P"), "G2.P")

    daq_entry = {"name": "Counter1", "driver": "keysight53230A",
                 "address": "sim", "port": 5025, "channel": 1, "ndim": 0,
                 "gate": False, "record": True, "simulation": True}
    daq_pvdb, _ = build_pvdb_for_entry(daq_entry, "STXMSIM:DEFAULT")

    pvdb = {}
    pvdb.update(MotorRecordGroup("STXMSIM:XPS:CoarseX", driver=mx,
                                 motor_config=dict(entry)).pvdb)
    fly = FlyGroup(FLY_PREFIX, motors={"G1.P": mx, "G2.P": my},
                   daq_pvs={"default": "STXMSIM:DEFAULT"}, simulation=True)
    pvdb.update(daq_pvdb)
    pvdb.update(fly.pvdb)
    ioc_harness.start(pvdb)
    return ioc_harness, ioc_harness.client()


def _pvs(ctx, *names):
    pvs = ctx.get_pvs(*names, timeout=15)
    for pv in pvs:
        pv.wait_for_connection(timeout=15)
    return pvs


def test_xps_motor_record_moves(xps_fly_ioc):
    _, ctx = xps_fly_ioc
    val, rbv = _pvs(ctx, "STXMSIM:XPS:CoarseX", "STXMSIM:XPS:CoarseX.RBV")
    val.write(3.75, wait=True, timeout=30)
    assert rbv.read().data[0] == pytest.approx(3.75, abs=1e-6)


def test_xps_motor_record_rejects_out_of_limits(xps_fly_ioc):
    _, ctx = xps_fly_ioc
    val, rbv = _pvs(ctx, "STXMSIM:XPS:CoarseX", "STXMSIM:XPS:CoarseX.RBV")
    before = rbv.read().data[0]
    with pytest.raises(Exception):
        val.write(500.0, wait=True, timeout=15)
    assert rbv.read().data[0] == pytest.approx(before, abs=1e-6)


def test_xps_fly_line_sim(xps_fly_ioc):
    import numpy as np
    _, ctx = xps_fly_ioc
    start, stop, npts, dwell, arm, go, index, data, pos = _pvs(
        ctx, *[f"{FLY_PREFIX}:{s}" for s in (
            "START", "STOP", "NPOINTS", "DWELL", "ARM", "GO",
            "INDEX", "DATA:default", "POS")])
    start.write(-5.0, wait=True); stop.write(5.0, wait=True)
    npts.write(25, wait=True); dwell.write(1.0, wait=True)
    arm.write(1, wait=True, timeout=15)
    go.write(1, wait=True, timeout=60)
    assert index.read().data[0] == 1
    p = np.asarray(pos.read().data, dtype=float)
    d = np.asarray(data.read().data, dtype=float)
    assert len(p) == 25 and len(d) == 25
    assert p[0] == pytest.approx(-5.0) and p[-1] == pytest.approx(5.0)
```

- [ ] **Step 2: Run the new tests**

Run: `C:\Users\rp\PycharmProjects\ncs\.venv\Scripts\python.exe -m pytest tests\iocs\test_xps_fly.py -q`
Expected: all pass (CA tests take ~10-30 s).

- [ ] **Step 3: Regression sweep**

Run: `C:\Users\rp\PycharmProjects\ncs\.venv\Scripts\python.exe -m pytest tests\iocs --continue-on-collection-errors -q`
Expected: green except the documented pre-existing env failures/errors
(test_config x2, e712 errors, npt_read_framing collection). Note
`test_plan_fleet_modules` should now PASS (fixed in Task 3).

- [ ] **Step 4: Commit**

```bash
git add tests/iocs/test_xps_fly.py
git commit -m "test(xps): IOC-level motor record + fly line over real CA (simulation)"
```

---

### Task 5: docs

**Files:**
- Modify: `pystxmcontrol/iocs/README.md` (fly-capable controllers section)

**Interfaces:** none (docs only).

- [ ] **Step 1: README**

In the fly-capable controllers section of `pystxmcontrol/iocs/README.md`
(where E712/nPoint/MMC are listed), add:

```markdown
- `xpsController` (Newport XPS): constant-velocity software-timed fly lines
  (`xpsMotor.line_trigger = "IMM"`, free-running DAQ). The driver preserves
  the legacy device-interaction semantics (relative moves + position-
  tolerance completion, disable/enable abort); known quirks are itemized in
  docs/superpowers/specs/2026-07-24-xps-fly-integration-design.md for later
  hardware review. PVT trajectories + position-compare EXT triggering are a
  recorded follow-up.
```

- [ ] **Step 2: Commit**

```bash
git add pystxmcontrol/iocs/README.md
git commit -m "docs(iocs): document XPS fly support"
```

---

## Hardware checklist (from the spec — not part of local execution)

Jog/limits/stop-mid-move regression vs legacy; long-move timeout margins
(config `timeout` default is 1 s — set real per-motor values in motor.json);
SGamma cruise restore after fly lines and aborts; review the spec's six
itemized device-interaction weaknesses with David.
