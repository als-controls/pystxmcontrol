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
