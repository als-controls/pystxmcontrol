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


def test_egu_and_velo(motor_ioc):
    h, ctx, drv = motor_ioc
    egu, velo = ctx.get_pvs("TEST:M1.EGU", "TEST:M1.VELO")
    assert egu.read(data_type="native").data[0].decode() == "um"
    assert velo.read().data[0] == 1000.0
