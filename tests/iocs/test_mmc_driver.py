"""Unit tests for the rewritten Micronix MMC driver (fake transport, no I/O)."""
import pytest


class FakeMMC:
    """Scripted line transport: records writes, replays canned replies."""
    def __init__(self, replies=()):
        self.writes = []
        self.replies = list(replies)

    def write(self, data: bytes):
        self.writes.append(data.decode())

    def readline(self) -> bytes:
        return (self.replies.pop(0) if self.replies else "").encode()

    def close(self):
        pass


def make_controller(replies=()):
    from pystxmcontrol.drivers.mmcController import mmcController
    ctrl = mmcController(address="COM99")
    ctrl.simulation = False
    ctrl._transport = FakeMMC(replies)
    return ctrl


def test_driver_importable_without_optional_deps():
    # Legacy module imported pylibftdi at module scope, which knocked the
    # whole driver out of pystxmcontrol.drivers in envs without it.
    import pystxmcontrol.drivers as drv
    assert hasattr(drv, "mmcController")


def test_command_frames_axis_prefix_and_cr():
    ctrl = make_controller()
    ctrl.command(2, "MVA1.234")
    assert ctrl._transport.writes == ["2MVA1.234\r"]


def test_query_strips_hash_and_returns_payload():
    ctrl = make_controller(replies=["#0.500000,0.498400\n"])
    assert ctrl.query(1, "POS?") == "0.500000,0.498400"
    assert ctrl._transport.writes == ["1POS?\r"]


def test_query_malformed_reply_raises_mmcerror():
    from pystxmcontrol.drivers.mmcController import MMCError
    ctrl = make_controller(replies=["garbage\n"])
    with pytest.raises(MMCError):
        ctrl.query(1, "POS?")


def test_query_empty_reply_raises_mmcerror():
    from pystxmcontrol.drivers.mmcController import MMCError
    ctrl = make_controller(replies=[""])
    with pytest.raises(MMCError):
        ctrl.query(1, "POS?")


def test_get_errors_never_raises():
    ctrl = make_controller(replies=[""])  # empty reply would raise in query()
    assert ctrl.get_errors(1) == ""


def test_simulation_initialize_opens_no_transport():
    from pystxmcontrol.drivers.mmcController import mmcController
    ctrl = mmcController(address="COM99")
    ctrl.initialize(simulation=True)
    assert ctrl.simulation is True and ctrl._transport is None


def test_transport_selection_serial_vs_tcp():
    from pystxmcontrol.drivers.mmcController import mmcController
    assert mmcController(address="COM3")._is_serial_address()
    assert mmcController(address="/dev/ttyUSB0")._is_serial_address()
    assert not mmcController(address="192.168.1.50", port=4001)._is_serial_address()


ENTRY = {"axis": "x", "minValue": -10.0, "maxValue": 10.0, "offset": 0.0,
         "units": 1.0, "max velocity": 2.0, "timeout": 10, "simulation": 0}


def make_motor(replies=(), entry=None, axis="x"):
    from pystxmcontrol.drivers.mmcMotor import mmcMotor
    ctrl = make_controller(replies)
    m = mmcMotor()
    m.controller = ctrl
    m.config = dict(ENTRY, **(entry or {}), axis=axis)
    m.connect(axis=axis)
    return m, ctrl._transport


def test_connect_maps_axis_and_enables_servo():
    m, t = make_motor()
    assert m._axis == 1
    assert "1FBK3\r" in t.writes


def test_connect_axis_y_is_2():
    m, _ = make_motor(axis="y")
    assert m._axis == 2


def test_get_pos_uses_encoder_field_and_units():
    m, t = make_motor(replies=["#2.000000,1.998000\n"],
                      entry={"units": 2.0, "offset": 1.0})
    assert m.getPos() == pytest.approx(1.998000 * 2.0 + 1.0)
    assert t.writes[-1] == "1POS?\r"


def test_status_decodes_moving_bit():
    # bit 3 (0x08) set = idle. 8 and 136 are the historically observed
    # idle bytes; 1 (bit 3 clear) means moving.
    m, _ = make_motor(replies=["#8\n", "#136\n", "#1\n"])
    assert m.getStatus() is False
    assert m.getStatus() is False
    assert m.getStatus() is True


def test_check_limits_raises_software_limit_error():
    from pystxmcontrol.controller.motor import SoftwareLimitError
    m, _ = make_motor()
    with pytest.raises(SoftwareLimitError):
        m.checkLimits(10.5)


def test_move_to_frames_float_target_no_truncation():
    m, t = make_motor(replies=["#8\n"])  # immediately idle
    m.moveTo(1.2345)
    # writes[0] is connect()'s "1FBK3\r"
    assert t.writes[1] == "1MVA1.234\r"  # 3-decimal round, NOT int()


def test_move_to_timeout_stops_and_raises():
    from pystxmcontrol.drivers.mmcController import MMCError
    moving = "#1\n"
    m, t = make_motor(replies=[moving] * 10000, entry={"timeout": 0.05})
    with pytest.raises(MMCError):
        m.moveTo(1.0)
    assert "1STP\r" in t.writes


def test_stop_sends_stp():
    m, t = make_motor()
    m.stop()
    assert t.writes[-1] == "1STP\r"


def test_velocity_set_and_get():
    m, t = make_motor(replies=["#0,1.500000\n"])
    m.setAxisParams(velocity=1.5)
    assert t.writes[-1] == "1VEL1.5\r"
    assert m.get_velocity() == pytest.approx(1.5)


def test_simulation_move_and_readback():
    from pystxmcontrol.drivers.mmcController import mmcController
    from pystxmcontrol.drivers.mmcMotor import mmcMotor
    ctrl = mmcController(address="COM99")
    ctrl.initialize(simulation=True)
    m = mmcMotor()
    m.controller = ctrl
    m.config = dict(ENTRY, offset=1.0, units=2.0)
    m.connect(axis="x")
    m.moveTo(5.0)
    assert m.getPos() == pytest.approx(5.0)
    assert m.getStatus() is False
