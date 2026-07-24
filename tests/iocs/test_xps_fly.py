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
