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


def test_supervisor_per_ioc_ports_env(tmp_path):
    """Each spawned IOC gets a distinct EPICS_CAS_SERVER_PORT and the full
    accumulated EPICS_CA_ADDR_LIST across all plans (Windows UDP-broadcast
    fix: without this, only one subprocess bound to 5064 ever receives
    searches)."""
    from pystxmcontrol.iocs.supervisor import IocPlan, Supervisor

    plans = [IocPlan(name="a", module=None, slice_path="a.py"),
             IocPlan(name="b", module=None, slice_path="b.py")]
    sup = Supervisor(plans, status_prefix=None, slice_dir=str(tmp_path))
    sup._allocate_ports()

    assert set(sup._ports) == {"a", "b"}
    assert sup._ports["a"] != sup._ports["b"]

    # Build the env the same way _spawn does, without actually launching a
    # process (IocPlan.slice_path here isn't a real script).
    import os as _os
    envs = {}
    for name, port in sup._ports.items():
        plan = next(p for p in plans if p.name == name)
        e = dict(_os.environ)
        e["EPICS_CAS_SERVER_PORT"] = str(sup._ports[plan.name])
        e["EPICS_CA_SERVER_PORT"] = str(sup._ports[plan.name])
        e["EPICS_CA_ADDR_LIST"] = sup._addr_list
        e["EPICS_CA_AUTO_ADDR_LIST"] = "NO"
        envs[name] = e

    assert envs["a"]["EPICS_CAS_SERVER_PORT"] != envs["b"]["EPICS_CAS_SERVER_PORT"]
    assert envs["a"]["EPICS_CA_SERVER_PORT"] == envs["a"]["EPICS_CAS_SERVER_PORT"]
    for name in ("a", "b"):
        assert envs[name]["EPICS_CA_AUTO_ADDR_LIST"] == "NO"
        addr_list = envs[name]["EPICS_CA_ADDR_LIST"]
        assert f"127.0.0.1:{sup._ports['a']}" in addr_list
        assert f"127.0.0.1:{sup._ports['b']}" in addr_list

    addr_file = tmp_path / "EPICS_CA_ADDR_LIST.txt"
    assert addr_file.exists()
    assert addr_file.read_text().strip() == sup._addr_list


def test_supervisor_spawn_sets_per_ioc_env(tmp_path, monkeypatch):
    """Directly exercise Supervisor._spawn (the real code path, not a
    reimplementation) by capturing the env passed to subprocess.Popen."""
    from pystxmcontrol.iocs.supervisor import IocPlan, Supervisor
    import pystxmcontrol.iocs.supervisor as supervisor_mod

    captured = {}

    class FakePopen:
        def __init__(self, cmd, env=None):
            captured[cmd[-1]] = env

    monkeypatch.setattr(supervisor_mod.subprocess, "Popen", FakePopen)

    fake_a = tmp_path / "a.py"
    fake_a.write_text("")
    fake_b = tmp_path / "b.py"
    fake_b.write_text("")
    plans = [IocPlan(name="a", module=None, slice_path=str(fake_a)),
             IocPlan(name="b", module=None, slice_path=str(fake_b))]
    sup = Supervisor(plans, status_prefix=None, slice_dir=str(tmp_path))
    sup._allocate_ports()
    sup._spawn(plans[0])
    sup._spawn(plans[1])

    env_a = captured[str(fake_a)]
    env_b = captured[str(fake_b)]
    assert env_a["EPICS_CAS_SERVER_PORT"] != env_b["EPICS_CAS_SERVER_PORT"]
    assert env_a["EPICS_CA_SERVER_PORT"] == env_a["EPICS_CAS_SERVER_PORT"]
    assert env_b["EPICS_CA_SERVER_PORT"] == env_b["EPICS_CAS_SERVER_PORT"]
    assert env_a["EPICS_CA_ADDR_LIST"] == env_b["EPICS_CA_ADDR_LIST"] == sup._addr_list
    assert env_a["EPICS_CA_AUTO_ADDR_LIST"] == "NO"


def test_supervisor_shared_ca_port_escape_hatch(tmp_path):
    from pystxmcontrol.iocs.supervisor import IocPlan, Supervisor
    plans = [IocPlan(name="a", module=None, slice_path="a.py")]
    sup = Supervisor(plans, status_prefix=None, shared_ca_port=True,
                     slice_dir=str(tmp_path))
    sup._allocate_ports()
    assert sup._ports == {}
    assert not (tmp_path / "EPICS_CA_ADDR_LIST.txt").exists()


def test_console_script_declared():
    import tomllib
    py = tomllib.loads((REPO / "pyproject.toml").read_text())
    assert py["project"]["scripts"]["stxm-iocs"] == "pystxmcontrol.iocs.supervisor:main"
    assert "caproto>=1.1" in py["project"]["optional-dependencies"]["iocs"]
