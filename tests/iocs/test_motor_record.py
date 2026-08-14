import time

import pytest


def _make_group(driver):
    from pystxmcontrol.iocs.base import MotorRecordGroup
    return MotorRecordGroup("TEST:M1", driver=driver,
                            motor_config=driver.config,
                            idle_poll=0.05, moving_poll=0.01)


@pytest.fixture
def motor_ioc(ioc_harness, slow_sim_motor):
    group = _make_group(slow_sim_motor)
    ioc_harness.start(group.pvdb)
    ctx = ioc_harness.client()
    (val,) = ctx.get_pvs("TEST:M1", timeout=10)
    val.wait_for_connection(timeout=10)
    return ioc_harness, ctx, slow_sim_motor


def test_val_seeded_from_readback_at_startup(ioc_harness, slow_sim_motor):
    """At startup .VAL must be synced to the hardware position (real
    motorRecord init_record() semantics), not left at the pvproperty default
    of 0.0 -- and that sync must not command a move."""
    drv = slow_sim_motor
    drv._controller_position = 12.5
    drv.controller.positions[drv.group] = 12.5

    moves = []
    original_move_to = drv.moveTo
    drv.moveTo = lambda pos: (moves.append(pos), original_move_to(pos))[1]

    group = _make_group(drv)
    ioc_harness.start(group.pvdb)
    ctx = ioc_harness.client()
    val, rbv, dmov = ctx.get_pvs("TEST:M1", "TEST:M1.RBV", "TEST:M1.DMOV",
                                 timeout=10)
    val.wait_for_connection(timeout=10)

    deadline = time.time() + 5
    while time.time() < deadline and val.read().data[0] == 0.0:
        time.sleep(0.05)
    assert abs(val.read().data[0] - 12.5) < 1e-6, (
        "startup .VAL was not seeded from the driver position")
    assert abs(rbv.read().data[0] - 12.5) < 1e-6
    # Seeding must be a pure PV update: no move queued, so DMOV never drops
    # and the axis has not been driven anywhere.
    assert dmov.read().data[0] == 1
    assert moves == [], f"startup seeding commanded moves: {moves}"
    time.sleep(0.3)
    assert abs(drv.getPos() - 12.5) < 1e-6

    # A subsequent real put still moves (the write hook was installed after
    # the seed, not skipped).
    val.write(3.0, wait=True, timeout=15)
    assert moves == [3.0]
    assert abs(drv.getPos() - 3.0) < 1e-6


def test_move_and_readback(motor_ioc):
    h, ctx, drv = motor_ioc
    val, rbv, dmov = ctx.get_pvs("TEST:M1", "TEST:M1.RBV", "TEST:M1.DMOV")
    val.write(5.0, wait=True, timeout=15)
    deadline = time.time() + 5
    while time.time() < deadline and dmov.read().data[0] != 1:
        time.sleep(0.05)
    assert dmov.read().data[0] == 1
    assert abs(rbv.read().data[0] - 5.0) < 1e-6
    assert abs(drv.getPos() - 5.0) < 1e-6


def test_val_put_completion_blocks_until_move_done(motor_ioc):
    """A wait=True .VAL put must not return until the physical move is done
    (DMOV=1) -- true motor-record busy semantics. The slow sim motor takes
    move_duration=0.5 s per move, so the put must take at least that long,
    and DMOV/RBV must already be final the instant it returns."""
    h, ctx, drv = motor_ioc
    val, rbv, dmov = ctx.get_pvs("TEST:M1", "TEST:M1.RBV", "TEST:M1.DMOV")
    t0 = time.monotonic()
    val.write(6.0, wait=True, timeout=15)
    elapsed = time.monotonic() - t0
    assert elapsed >= drv.move_duration, (
        f"put-completion returned after {elapsed:.3f}s, before the "
        f"{drv.move_duration}s move finished")
    # No settling loop: completion means the move is already over.
    assert dmov.read().data[0] == 1
    assert abs(rbv.read().data[0] - 6.0) < 1e-6
    assert abs(drv.getPos() - 6.0) < 1e-6


def test_dmov_transitions_during_move(motor_ioc):
    h, ctx, drv = motor_ioc
    val, dmov, movn = ctx.get_pvs("TEST:M1", "TEST:M1.DMOV", "TEST:M1.MOVN")
    val.write(10.0, wait=False)
    time.sleep(0.15)  # mid-move (move_duration=0.5)
    assert movn.read().data[0] == 1
    assert dmov.read().data[0] == 0
    deadline = time.time() + 5
    while time.time() < deadline and dmov.read().data[0] != 1:
        time.sleep(0.05)
    assert dmov.read().data[0] == 1
    assert movn.read().data[0] == 0


def test_limits_enforced(motor_ioc):
    h, ctx, drv = motor_ioc
    val, rbv, hlm, llm = ctx.get_pvs("TEST:M1", "TEST:M1.RBV", "TEST:M1.HLM", "TEST:M1.LLM")
    assert hlm.read().data[0] == 40.0 and llm.read().data[0] == -40.0
    before = rbv.read().data[0]
    with pytest.raises(Exception):
        val.write(41.0, wait=True, timeout=10)
    time.sleep(0.3)
    assert abs(rbv.read().data[0] - before) < 1e-6  # no motion happened


def test_stop_mid_move(motor_ioc):
    h, ctx, drv = motor_ioc
    val, stop, dmov, rbv = ctx.get_pvs("TEST:M1", "TEST:M1.STOP", "TEST:M1.DMOV", "TEST:M1.RBV")
    val.write(20.0, wait=False)
    time.sleep(0.15)
    stop.write(1, wait=True, timeout=10)
    deadline = time.time() + 5
    while time.time() < deadline and dmov.read().data[0] != 1:
        time.sleep(0.05)
    assert dmov.read().data[0] == 1
    final = rbv.read().data[0]
    assert final < 19.0  # stopped short of the 20.0 target
    time.sleep(0.3)
    assert abs(rbv.read().data[0] - final) < 0.5  # and stayed put


def test_driver_exception_recovers(motor_ioc):
    """An unexpected moveTo exception must not kill the polling loop:
    DMOV returns to 1 and a subsequent valid move still works."""
    h, ctx, drv = motor_ioc
    val, rbv, dmov = ctx.get_pvs("TEST:M1", "TEST:M1.RBV", "TEST:M1.DMOV")

    original_move_to = drv.moveTo
    fail_next = {"armed": True}

    def flaky_move_to(pos):
        if fail_next["armed"]:
            fail_next["armed"] = False
            raise RuntimeError("driver blew up")
        return original_move_to(pos)

    drv.moveTo = flaky_move_to

    val.write(5.0, wait=True, timeout=15)  # this move raises in the driver
    deadline = time.time() + 5
    while time.time() < deadline and dmov.read().data[0] != 1:
        time.sleep(0.05)
    assert dmov.read().data[0] == 1  # loop recovered, not frozen

    # A subsequent valid move must still work end-to-end.
    val.write(7.0, wait=True, timeout=15)
    deadline = time.time() + 5
    while time.time() < deadline and (
            dmov.read().data[0] != 1
            or abs(rbv.read().data[0] - 7.0) > 1e-6):
        time.sleep(0.05)
    assert dmov.read().data[0] == 1
    assert abs(rbv.read().data[0] - 7.0) < 1e-6
    assert abs(drv.getPos() - 7.0) < 1e-6


def test_stop_exception_recovers(motor_ioc):
    """F1b: a raising driver.stop() must not kill the record's polling loop
    (and thus the whole caproto server) -- log it and keep going. The
    record must still respond after the failed STOP, and a subsequent move
    must still complete normally."""
    h, ctx, drv = motor_ioc
    val, stop, dmov, rbv = ctx.get_pvs(
        "TEST:M1", "TEST:M1.STOP", "TEST:M1.DMOV", "TEST:M1.RBV")

    def raising_stop():
        raise RuntimeError("stop blew up")

    drv.stop = raising_stop

    val.write(20.0, wait=False)
    time.sleep(0.15)
    stop.write(1, wait=True, timeout=10)  # STOP raises inside the driver

    # The move was NOT actually stopped (stop() raised before taking
    # effect), but the record must still be alive: DMOV eventually settles
    # and RBV keeps being readable.
    deadline = time.time() + 5
    while time.time() < deadline and dmov.read().data[0] != 1:
        time.sleep(0.05)
    assert dmov.read().data[0] == 1

    # A subsequent valid move must still work end-to-end (loop not dead).
    val.write(7.0, wait=True, timeout=15)
    deadline = time.time() + 5
    while time.time() < deadline and (
            dmov.read().data[0] != 1
            or abs(rbv.read().data[0] - 7.0) > 1e-6):
        time.sleep(0.05)
    assert dmov.read().data[0] == 1
    assert abs(rbv.read().data[0] - 7.0) < 1e-6
    assert abs(drv.getPos() - 7.0) < 1e-6


def test_egu_and_velo(motor_ioc):
    h, ctx, drv = motor_ioc
    egu, velo = ctx.get_pvs("TEST:M1.EGU", "TEST:M1.VELO")
    assert egu.read(data_type="native").data[0].decode() == "um"
    assert velo.read().data[0] == 1000.0
