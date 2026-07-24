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
