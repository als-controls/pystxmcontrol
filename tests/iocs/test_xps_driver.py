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


# ---- motor tests -------------------------------------------------------

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
