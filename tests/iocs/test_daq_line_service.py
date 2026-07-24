"""Line-mode CA surface of the standalone DAQ service (simulation)."""
import time

import pytest

PREFIX = "STXMSIM:DAQSVC"


def _read_str(pv):
    data = pv.read(data_type="native").data
    if isinstance(data, bytes):
        return data.decode().rstrip("\x00")
    if isinstance(data, str):
        return data.rstrip("\x00")
    # Handle array-like data (list or numpy array)
    # Try to join if it's a list of bytes
    try:
        if hasattr(data, '__iter__') and len(data) > 0:
            first_elem = next(iter(data))
            if isinstance(first_elem, bytes):
                return b''.join(data).decode().rstrip("\x00")
        # Otherwise treat as byte values
        return bytes(int(x) for x in data if int(x) != 0).decode()
    except (ValueError, TypeError, StopIteration):
        # Last resort: just convert to string and clean
        return str(data).rstrip("\x00")


@pytest.fixture
def daq_service(ioc_harness):
    from pystxmcontrol.iocs.daq_ioc import build_pvdb_for_entry
    entry = {"name": "Counter1", "driver": "keysight53230A", "address": "sim",
             "port": 5025, "channel": 1, "ndim": 0, "gate": False,
             "record": True, "simulation": True}
    pvdb, _ = build_pvdb_for_entry(entry, PREFIX)
    ioc_harness.start(pvdb)
    return ioc_harness, ioc_harness.client()


def _pvs(ctx, *suffixes):
    pvs = ctx.get_pvs(*[f"{PREFIX}:{s}" for s in suffixes], timeout=15)
    for pv in pvs:
        pv.wait_for_connection(timeout=15)
    return pvs


def _wait_index(index_pv, target, timeout=15.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if index_pv.read().data[0] >= target:
            return True
        time.sleep(0.02)
    return False


def test_line_arm_completes_and_publishes(daq_service):
    _, ctx = daq_service
    npts, dwell, arm, index, wf, status = _pvs(
        ctx, "LINE:NPOINTS", "DWELL", "LINE:ARM", "LINE:INDEX",
        "COUNTS:WF", "LINE:STATUS")
    npts.write(25, wait=True)
    dwell.write(1.0, wait=True)
    assert index.read().data[0] == 0
    arm.write(1, wait=True, timeout=15)          # put-completion = armed
    assert _wait_index(index, 1)                  # sim line: ~25 ms later
    data = wf.read().data
    assert len(data) == 25 and all(v > 0 for v in data)
    assert status.read().data[0] == 0             # back to IDLE


def test_arm_while_busy_rejected(daq_service):
    _, ctx = daq_service
    npts, dwell, arm, err, abort, status = _pvs(
        ctx, "LINE:NPOINTS", "DWELL", "LINE:ARM", "LINE:ERROR",
        "LINE:ABORT", "LINE:STATUS")
    npts.write(200, wait=True)
    dwell.write(20.0, wait=True)                  # 4 s sim line
    arm.write(1, wait=True, timeout=15)
    arm.write(1, wait=True, timeout=15)           # second arm: no-op + error
    assert "rejected" in _read_str(err)
    abort.write(1, wait=True, timeout=15)
    t0 = time.monotonic()
    while status.read().data[0] not in (0, 3) and time.monotonic() - t0 < 10:
        time.sleep(0.05)
    assert status.read().data[0] == 0             # IDLE after abort


def test_abort_leaves_index_unchanged(daq_service):
    _, ctx = daq_service
    npts, dwell, arm, abort, index = _pvs(
        ctx, "LINE:NPOINTS", "DWELL", "LINE:ARM", "LINE:ABORT", "LINE:INDEX")
    before = index.read().data[0]
    npts.write(200, wait=True)
    dwell.write(20.0, wait=True)
    arm.write(1, wait=True, timeout=15)
    abort.write(1, wait=True, timeout=15)
    time.sleep(0.2)
    assert index.read().data[0] == before


def test_point_acquire_rejected_during_line(daq_service):
    _, ctx = daq_service
    npts, dwell, arm, acquire, err, abort = _pvs(
        ctx, "LINE:NPOINTS", "DWELL", "LINE:ARM", "ACQUIRE", "LINE:ERROR",
        "LINE:ABORT")
    npts.write(200, wait=True)
    dwell.write(20.0, wait=True)
    arm.write(1, wait=True, timeout=15)
    acquire.write(1, wait=True, timeout=15)       # no-op + error, NOT hang
    assert "ACQUIRE rejected" in _read_str(err)
    abort.write(1, wait=True, timeout=15)


def test_arm_validation_errors(daq_service):
    _, ctx = daq_service
    npts, arm, status, err = _pvs(
        ctx, "LINE:NPOINTS", "LINE:ARM", "LINE:STATUS", "LINE:ERROR")
    npts.write(0, wait=True)
    arm.write(1, wait=True, timeout=15)
    assert status.read().data[0] == 3             # ERROR
    assert "NPOINTS" in _read_str(err)
