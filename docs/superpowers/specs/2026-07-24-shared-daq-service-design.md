# Shared DAQ CA service — design

Date: 2026-07-24. Branch: `feature/caproto-iocs` (on top of the MMC work).
Retires two follow-ups from `docs/superpowers/2026-07-12-caproto-iocs-followups.md`:
multi-fly DAQ ownership (plan_fleet raises on >1 fly-capable group) and the
"configure ALL DAQs, fly ONCE" hardware restructure.

## Problem

DAQ hardware is hosted *in-process* by the fly IOC: `fly_ioc` builds
`DaqGroup` objects and its line loop calls `daq.config`/`initLine`/`getLine`
directly. Consequently a fly-capable controller group absorbs ALL daq
entries, and `plan_fleet` must reject fleets with more than one fly-capable
group (supervisor.py) — now guaranteed to happen since both nPoint and MMC
are fly-capable. Requirement (Ron): both motor drivers must be able to fly
with a DAQ; per-controller static ownership (option A) is not acceptable.

## Approach

Make each DAQ a standalone CA service (its own `daq_ioc` process, always).
Fly IOCs become CA *clients* of DAQ services — the same pattern already used
for beam shutters. Any fly scanner can use any detector; concurrent use is
serialized by a busy guard on the DAQ side.

**Performance gate:** added per-line overhead of the CA hop (vs in-process)
must be ≤ 5 ms median on localhost, measured by a benchmark test (below).
Mitigations designed in: persistent client Context, subscribe-once monitors,
pre-line parameter writes only-when-changed.

## Components

### 1. `daq_ioc.DaqGroup` — line-mode CA surface

New PVs (existing point-mode `:DWELL`/`:MODE`/`:ACQUIRE`/`:COUNTS`/
`:COUNTS:WF`/`:RATE` unchanged):

- `:LINE:NPOINTS` (int, 1..MAX_LINE)
- `:LINE:TRIGGER` (enum `EXT`/`IMM`/`BUS`, default `EXT`)
- `:LINE:ARM` (int put; **put-completion = armed**): validates NPOINTS/DWELL,
  rejects if busy, then in an executor runs
  `daq.config(dwell, count=1, samples=n, trigger=<TRIGGER>)` + `daq.initLine()`
  (hardware; sim: `config(dwell, count=n, samples=1)` matching today's sim
  contract), then spawns an internal asyncio task awaiting `daq.getLine()`.
  The ARM put returns once armed — the client may then command its move.
- `:LINE:INDEX` (int, read-only): when the internal task completes, the line
  is written to `:COUNTS:WF` **first**, then `:LINE:INDEX` increments
  (write-then-increment, same contract as the fly IOC's `:INDEX`). Clients
  monitor `:LINE:INDEX`, then read `:COUNTS:WF`.
- `:LINE:ABORT` (int put): cancels the in-flight task, disarms, clears busy.
- `:LINE:STATUS` (enum `IDLE`/`ARMED`/`ACQUIRING`/`ERROR`, read-only) and
  `:LINE:ERROR` (char string, read-only) for diagnostics.

Busy guard: one line (or point `:ACQUIRE`) at a time. `ARM` while
ARMED/ACQUIRING, or `ACQUIRE` while a line is in flight, is rejected with a
descriptive error on `:LINE:ERROR` (put still completes, status untouched) —
this is the serialization between two fly scanners sharing one counter.

Auto-disarm watchdog: an armed line that produces no data within
`max(5 s, 4 × n × dwell + 5 s)` is aborted and `:LINE:STATUS` = ERROR — a
crashed client cannot wedge the detector.

`write_line()` stays (used by the internal line task); it is no longer called
from another process's IOC.

### 2. `fly_ioc` — CA client of DAQ services

- The fly slice no longer embeds `daqs`; it carries `daq_pvs`:
  `{key: "<daq prefix>"}` (written by the supervisor), alongside the existing
  `shutters` map.
- `FlyGroup` gains a `DaqClient` per key (module-level helper class in
  fly_ioc.py): persistent `caproto.threading.client.Context` (shared with
  `_command_shutters`' context), PVs connected lazily on first use and kept;
  `:LINE:INDEX` monitor subscribed once.
  - `configure(dwell, npoints, trigger)` — writes only values that changed.
  - `arm()` — `write(1, wait=True)` on `:LINE:ARM`.
  - `wait_line(deadline)` — waits for the INDEX monitor to advance past the
    pre-arm value; on deadline returns None.
  - `read_line()` — `read()` of `:COUNTS:WF` (first NPOINTS elements).
  - `abort()` — best-effort `:LINE:ABORT`.
- `_fly_one_line` hardware path becomes: for ALL daq keys: configure + arm →
  (non-EXT) prepare stage as today → beam open → command the move ONCE →
  wait_line on every daq → beam close → read_line from every daq → republish
  on `:DATA:<key>` / `:POS` / `:INDEX` exactly as today (ordering contract to
  clients unchanged). This is the "configure all, fly once" restructure.
- Simulation path: same CA client flow against the (sim) DAQ services — the
  sim/hardware split in the fly loop shrinks to the motor handling. This
  gives the CA path full test coverage in simulation.
- Abort path: on `:ABORT`/timeout, fly IOC calls `DaqClient.abort()` for
  armed daqs (best-effort, mirrors `_close_beam`).

### 3. `supervisor.plan_fleet` — simplification

- Delete the >1-fly-group `ValueError` and the absorb-all logic entirely.
- Every daq entry ALWAYS gets a standalone `daq_ioc` plan (started before
  fly IOCs; derived stay last).
- Every fly-group slice gets `daq_pvs` = {key: prefix} for ALL daqs (plus
  `shutters` as today). Gate handling moves with it: the `gate=False`
  override now applies in daq_ioc slice writing (the shutter IOC still owns
  the Arduino; fly IOC still commands shutters over CA around lines).
- Multiple fly-capable groups (E712 + npt + MMC in one fleet) are now
  routine; add a supervisor test proving it.

### 4. Performance benchmark (gate)

New test (marked, runs in the normal suite with modest n): sim DAQ service +
fly line over CA; measure `t(GO put-completion) − n × dwell − motor-sim time`
per line across ~20 lines at dwell=1 ms, n=50; assert median added overhead
≤ 5 ms and log the distribution. A comparison harness function (not a test)
times the raw in-process `getLine` path for reference numbers in the report.

## Error handling

- DAQ service rejects contention loudly (ERROR status + message), never
  corrupts an in-flight line.
- Fly IOC surfaces DAQ-side errors on its `:ERROR` with the stage name
  (existing stage mechanism; stages gain "daq_config"/"daq_arm"/"daq_read").
- Watchdog auto-disarm on the DAQ side; abort propagation from the fly side.
- CA disconnect mid-line → wait_line deadline fires → existing timeout path.

## Out of scope

- Point-mode `:ACQUIRE` behavior (unchanged).
- Lightfall client-facing PV surface of the fly IOC (unchanged).
- True concurrent multi-client acquisition (busy guard serializes; that is
  the semantics of one physical counter).

## Migration notes

- `fly_ioc.build_pvdb_from_slice` stops building DAQ pvdbs; existing tests
  that hand `daq_groups` to `FlyGroup` are rewritten to spin up a DAQ
  service pvdb in the same test harness and pass `daq_pvs`.
- README fly section updated (DAQ ownership paragraph).
