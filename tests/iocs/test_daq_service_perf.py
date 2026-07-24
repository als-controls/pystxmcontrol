"""Performance gate for the shared-DAQ CA hop (spec 2026-07-24, section 4).

Component gates (median over 20 reps, localhost):
  - DaqClient.arm() round-trip           <= 5 ms
  - DaqClient.read_line() waveform fetch <= 5 ms
End-to-end GO overhead is REPORTED (printed) with a generous pathology
ceiling only -- Windows timer granularity (~15.6 ms) sits inside both the
sim DAQ's asyncio.sleep and the fly loop's 20 ms poll, so a tight
end-to-end gate would measure the OS clock, not the CA hop.
"""
import statistics
import time

import pytest

DAQ_PREFIX = "STXMSIM:PERF"
FLY_PREFIX = "STXMSIM:PERFFLY:FLY"
N, DWELL_MS, REPS = 50, 1.0, 20


@pytest.fixture
def perf_ioc(ioc_harness):
    from pystxmcontrol.iocs.base import build_controller, build_motor
    from pystxmcontrol.iocs.daq_ioc import build_pvdb_for_entry
    from pystxmcontrol.iocs.fly_ioc import FlyGroup

    daq_entry = {"name": "Counter1", "driver": "keysight53230A",
                 "address": "sim", "port": 5025, "channel": 1, "ndim": 0,
                 "gate": False, "record": True, "simulation": True}
    pvdb, _ = build_pvdb_for_entry(daq_entry, DAQ_PREFIX)
    ctrl = build_controller({"controller": "mmcController", "address": "COM97",
                             "port": 0, "simulation": True})
    entry = {"axis": "x", "minValue": -50.0, "maxValue": 50.0, "offset": 0.0,
             "units": 1.0, "max velocity": 1000.0, "simulation": 1}
    mx = build_motor("mmcMotor", ctrl, dict(entry), "x")
    fly = FlyGroup(FLY_PREFIX, motors={"x": mx}, daq_pvs={"d": DAQ_PREFIX},
                   simulation=True)
    pvdb.update(fly.pvdb)
    ioc_harness.start(pvdb)
    return ioc_harness, ioc_harness.client()


def test_ca_hop_component_overhead(perf_ioc):
    from caproto.threading.client import Context
    from pystxmcontrol.iocs.fly_ioc import DaqClient
    _, _ctx_unused = perf_ioc
    ctx = Context()
    client = DaqClient(DAQ_PREFIX, ctx)
    client.configure(DWELL_MS, N, "EXT")

    arm_times, read_times = [], []
    for _ in range(REPS):
        t0 = time.perf_counter()
        client.arm()
        arm_times.append(time.perf_counter() - t0)
        deadline = time.monotonic() + 10.0
        assert client.wait_line(deadline)
        t0 = time.perf_counter()
        line = client.read_line(N)
        read_times.append(time.perf_counter() - t0)
        assert len(line) == N

    arm_med = statistics.median(arm_times) * 1000
    read_med = statistics.median(read_times) * 1000
    print(f"\n[perf] arm median {arm_med:.2f} ms  "
          f"(min {min(arm_times)*1000:.2f}, max {max(arm_times)*1000:.2f})")
    print(f"[perf] read median {read_med:.2f} ms  "
          f"(min {min(read_times)*1000:.2f}, max {max(read_times)*1000:.2f})")
    assert arm_med <= 5.0, f"ARM CA round-trip median {arm_med:.2f} ms > 5 ms gate"
    assert read_med <= 5.0, f"WF read median {read_med:.2f} ms > 5 ms gate"


def test_end_to_end_line_overhead_reported(perf_ioc):
    _, ctx = perf_ioc
    names = [f"{FLY_PREFIX}:{s}" for s in
             ("START", "STOP", "NPOINTS", "DWELL", "ARM", "GO", "INDEX")]
    pvs = ctx.get_pvs(*names, timeout=15)
    for pv in pvs:
        pv.wait_for_connection(timeout=15)
    start, stop, npts, dwell, arm, go, index = pvs
    start.write(-5.0, wait=True); stop.write(5.0, wait=True)
    npts.write(N, wait=True); dwell.write(DWELL_MS, wait=True)
    arm.write(1, wait=True, timeout=15)
    nominal = N * DWELL_MS / 1000.0
    overheads = []
    for _ in range(REPS):
        t0 = time.perf_counter()
        go.write(1, wait=True, timeout=60)
        overheads.append(time.perf_counter() - t0 - nominal)
    med = statistics.median(overheads) * 1000
    print(f"\n[perf] end-to-end line overhead median {med:.1f} ms over "
          f"{REPS} lines (n={N}, dwell={DWELL_MS} ms; includes fly-loop "
          f"20 ms poll + OS timer granularity, NOT just the CA hop)")
    assert med < 200.0, f"pathological per-line overhead: {med:.1f} ms"
