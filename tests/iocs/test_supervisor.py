import sys
import time
from pathlib import Path

import pytest

from pystxmcontrol.iocs.config import load_fleet

REPO = Path(__file__).resolve().parents[2]


@pytest.fixture
def fleet():
    return load_fleet(str(REPO / "config" / "motor.json"),
                      str(REPO / "config" / "daq.json"), station="SIM")


def test_plan_fleet_modules(fleet, tmp_path):
    from pystxmcontrol.iocs.supervisor import plan_fleet
    plans = plan_fleet(fleet, str(tmp_path))
    by_module = {}
    for p in plans:
        by_module.setdefault(p.module, []).append(p.name)
    # shipped config has no E712 entry -> all controllers use motor_ioc
    assert len(by_module.get("pystxmcontrol.iocs.motor_ioc", [])) >= 4
    assert len(by_module.get("pystxmcontrol.iocs.daq_ioc", [])) == 1
    assert len(by_module.get("pystxmcontrol.iocs.shutter_ioc", [])) == 1
    assert len(by_module.get("pystxmcontrol.iocs.derived_ioc", [])) == 3  # SampleX, SampleY, Energy
    # derived plans come last
    assert all(p.module != "pystxmcontrol.iocs.derived_ioc" for p in plans[:-3])
    for p in plans:
        assert Path(p.slice_path).exists()


def test_plan_fleet_e712_absorbs_daqs(tmp_path):
    import json
    base = json.loads((REPO / "config" / "motor.json").read_text())
    cfg = {
        "FlyX": {
            "index": 0, "type": "primary", "axis": "x", "driver": "E712Motor",
            "controllerID": "192.168.1.201", "port": 5000,
            "controller": "E712Controller", "max velocity": 1000.0,
            "minValue": -50.0, "maxValue": 50.0, "offset": 0.0, "units": 1.0,
            "display": True, "simulation": 1,
        },
    }
    mp = tmp_path / "motor.json"
    mp.write_text(json.dumps(cfg))
    fleet = load_fleet(str(mp), str(REPO / "config" / "daq.json"), station="SIM")
    if not fleet.controller_groups:
        pytest.skip("E712Controller unavailable in this env (pipython guard)")
    from pystxmcontrol.iocs.supervisor import plan_fleet
    from pystxmcontrol.iocs.config import read_slice
    plans = plan_fleet(fleet, str(tmp_path / "slices"))
    e712 = [p for p in plans if p.module == "pystxmcontrol.iocs.e712_ioc"]
    assert len(e712) == 1
    s = read_slice(e712[0].slice_path)
    assert [d["key"] for d in s["daqs"]] == ["default"]
    assert not [p for p in plans if p.module == "pystxmcontrol.iocs.daq_ioc"]


def test_supervisor_restarts_crashed_ioc(tmp_path, free_port):
    """Use a tiny fake IOC module that exits after N seconds to test restart."""
    from pystxmcontrol.iocs.supervisor import IocPlan, Supervisor
    fake = tmp_path / "fake_ioc.py"
    fake.write_text("import sys, time\ntime.sleep(1.0)\nsys.exit(1)\n")
    plan = IocPlan(name="fake", module=None, slice_path=str(fake))
    # Supervisor must support module=None -> spawn [sys.executable, slice_path]
    sup = Supervisor([plan], restart_backoff=(0.5, 0.5), status_prefix=None)
    sup.start()
    try:
        time.sleep(4.0)
        st = sup.status()["fake"]
        assert st["restarts"] >= 1
    finally:
        sup.stop()


def test_supervisor_status_pvs(tmp_path, free_port):
    from pystxmcontrol.iocs.supervisor import IocPlan, Supervisor
    fake = tmp_path / "fake_ioc.py"
    fake.write_text("import time\ntime.sleep(60)\n")
    sup = Supervisor([IocPlan(name="fake", module=None, slice_path=str(fake))],
                     status_prefix="STXMSIM:SUP")
    sup.start()
    try:
        from caproto.threading.client import Context
        ctx = Context()
        running, restarts = ctx.get_pvs("STXMSIM:SUP:fake:RUNNING",
                                        "STXMSIM:SUP:fake:RESTARTS", timeout=20)
        running.wait_for_connection(timeout=20)
        assert running.read().data[0] == 1
        assert restarts.read().data[0] == 0
    finally:
        sup.stop()


def test_console_script_declared():
    import tomllib
    py = tomllib.loads((REPO / "pyproject.toml").read_text())
    assert py["project"]["scripts"]["stxm-iocs"] == "pystxmcontrol.iocs.supervisor:main"
    assert "caproto>=1.1" in py["project"]["optional-dependencies"]["iocs"]
