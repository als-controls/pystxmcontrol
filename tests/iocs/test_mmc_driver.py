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
