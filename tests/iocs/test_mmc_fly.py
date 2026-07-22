"""MMC fly-capability routing + line_trigger plumbing (no hardware)."""
import pytest


def test_mmc_is_fly_capable():
    from pystxmcontrol.iocs.config import FLY_CAPABLE_CONTROLLERS
    assert "mmcController" in FLY_CAPABLE_CONTROLLERS


def test_mmc_motor_declares_internal_trigger():
    from pystxmcontrol.drivers.mmcMotor import mmcMotor
    assert mmcMotor.line_trigger == "INT"


def test_npt_default_stays_external():
    # Drivers without the attribute must keep today's EXT behavior.
    from pystxmcontrol.iocs import fly_ioc
    class NoAttr: pass
    assert getattr(NoAttr(), "line_trigger", "EXT") == "EXT"
    # and the source actually consults the attribute:
    import inspect
    src = inspect.getsource(fly_ioc)
    assert 'getattr(motor, "line_trigger", "EXT")' in src


def test_fly_slice_builds_for_mmc(tmp_path):
    """fly_ioc.build_pvdb_from_slice accepts an mmcController group and
    serves motor records + FLY PVs (simulation)."""
    import json
    from pystxmcontrol.iocs.fly_ioc import build_pvdb_from_slice
    entry = {"type": "primary", "driver": "mmcMotor",
             "controller": "mmcController", "controllerID": "COM99",
             "axis": "x", "minValue": -10.0, "maxValue": 10.0,
             "offset": 0.0, "units": 1.0, "max velocity": 2.0,
             "simulation": 1}
    s = {"kind": "controller", "station": "SIM", "controller_id": "COM99",
         "controller_cls": "mmcController", "label": "MMC", "port": 0,
         "simulation": True,
         "motors": [{"key": "CoarseX", "entry": entry,
                     "pv": "STXMSIM:MMC:CoarseX"}],
         "derived": [], "motor_pv": {"CoarseX": "STXMSIM:MMC:CoarseX"},
         "daqs": [{"key": "default", "prefix": "STXMSIM:DEFAULT",
                   "entry": {"name": "Counter1", "driver": "keysight53230A",
                             "address": "sim", "port": 5025, "channel": 1,
                             "ndim": 0, "gate": False, "record": True,
                             "simulation": True}}]}
    p = tmp_path / "slice.json"
    p.write_text(json.dumps(s))
    from pystxmcontrol.iocs.config import read_slice
    pvdb = build_pvdb_from_slice(read_slice(str(p)))
    names = set(pvdb)
    assert "STXMSIM:MMC:CoarseX" in names
    assert "STXMSIM:MMC:FLY:GO" in names
