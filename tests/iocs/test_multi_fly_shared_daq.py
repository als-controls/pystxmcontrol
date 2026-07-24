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
