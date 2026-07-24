# Newport XPS modernization + fly integration — design

Date: 2026-07-24. Branch: `feature/caproto-iocs` (after the shared-DAQ-service
work). Pattern: the MMC integration
(`2026-07-22-mmc-integration-design.md`), adapted to the XPS's TCP protocol.
Hardware validation: next beamline visit (Ron).

## Goal

Bring the Newport XPS to full parity with MMC: a rewritten, lock-agnostic
`xpsController` transaction layer, a modern `xpsMotor` with the duck-typed
fly interface (constant-velocity, free-running DAQ), and registration in
`FLY_CAPABLE_CONTROLLERS`. Since the shared-DAQ service landed, the IOC side
needs nothing else — fly_ioc and the DAQ services are controller-agnostic.

## XPS protocol facts

TCP to `address:5001` (config `port` respected). ASCII commands
`Function(arg1,arg2,...)`; replies `errcode,payload,EndOfAPI` (reply may span
multiple recv calls — read until `,EndOfAPI`). `errcode` 0 = success.
**Motion-command replies arrive at move COMPLETION** — `GroupMoveAbsolute`
blocks on the wire for the duration of the move. Key functions used:
`GroupMoveAbsolute(positioner, target)`, `GroupMoveAbort(group)`,
`GroupPositionCurrentGet(group, double *)`,
`PositionerSGammaParametersGet/Set(positioner, vel, accel, minJerkT, maxJerkT)`,
`GroupStatusGet(group, int *)`, `GroupMotionDisable/Enable(group)`.
Axis naming: motor.json `axis` = `"Group.Positioner"`; group = the prefix
before the dot; position reads and aborts address the group, SGamma the full
positioner name.

## Components

### 1. `drivers/xpsController.py` (full rewrite)

- `xpsController(hardwareController)`, `__init__(address, port=5001,
  simulation=False)`, `initialize(simulation=False)`.
- `XPSError(IOError)` raised on socket failure/timeout, malformed reply
  (no `,EndOfAPI`), or nonzero XPS error code (message includes the code and
  the command). No more `[-2, '']` sentinel tuples, no `eval()`.
- **Two persistent TCP sockets**, both created in `initialize()`:
  - `_motion` — motion commands only. Its reply blocks until the move
    completes, so a blocking `moveTo` is just "send + read reply" with a
    per-call timeout (`settimeout(move_timeout)` around the transaction).
  - `_control` — queries and `GroupMoveAbort`. Exists so `stop()` works
    WHILE the motion socket is blocked mid-move (the IOC layer deliberately
    calls `driver.stop()` without the io_lock so it can interrupt a
    blocking move).
- `_transact(sock, command, timeout=None) -> str payload`: encode, send,
  recv-loop until `,EndOfAPI`, split off errcode, raise `XPSError` if
  nonzero. All framing/parsing lives HERE.
- Thin protocol wrappers (all raise `XPSError` on failure):
  `move_absolute(positioner, target, timeout)` (motion socket),
  `abort_group(group)`, `get_position(group) -> float`,
  `get_status(group) -> int`, `get_sgamma(positioner) -> (vel, accel,
  min_jerk, max_jerk)` (float parsing), `set_sgamma(positioner, vel, accel,
  min_jerk, max_jerk)`, `disable_group(group)` / `enable_group(group)`.
- Lock-agnostic: NO threading locks (IOC `io_lock` serializes; the sole
  intentional concurrency — abort during move — uses the separate socket).
- Simulation: no I/O; per-positioner `positions` dict and `sgamma` dict on
  the controller so multiple motors share state.

### 2. `drivers/xpsMotor.py` (full rewrite)

Point-to-point (what `MotorRecordGroup` drives):

- `connect(axis=, **kwargs)` — stores `axis` (full `Group.Positioner`) and
  `group`; inherits `controller.simulation`; latches initial position and
  SGamma params (velocity/accel/jerk) when not simulating.
- `checkLimits(pos)` — raises `SoftwareLimitError` (lower/upper), as today.
- `moveTo(pos)` — limits check; offset/units transform (`(pos - offset) /
  units`, 3-decimal round); `controller.move_absolute(...,
  timeout=config.get("timeout", 60))`. Blocking-reply semantics: the reply
  IS move completion — no polling loop, no position-tolerance heuristic,
  ABSOLUTE moves (the legacy relative-move workaround for "absolute moves
  get inaccurate" is dropped; re-check on hardware and revert one method if
  the pathology reappears). On `XPSError` timeout: `abort_group` then
  re-raise.
- `getPos()` — `get_position(group) * units + offset`.
- `getStatus()` — `get_status(group)`; moving = status code in the XPS
  "moving" class (43-44 for SGamma moves; expose as module constant
  `_MOVING_STATUS = frozenset({43, 44})`, confirm/extend at the beamline).
- `stop()` — `abort_group(group)` (GroupMoveAbort). The legacy
  disable/enable abort is GONE from stop(); `disable()`/`enable()` remain
  as explicit methods.
- `setAxisParams(velocity=)` / `get_velocity()` — SGamma read-modify-write:
  set velocity, keep accel/jerk untouched.
- Simulation: controller sim dicts, same contract as mmcMotor.

Fly interface (identical contract to `mmcMotor`):

- `line_trigger = "IMM"` (free-running DAQ; the XPS PVT trajectory engine +
  position-compare EXT trigger is explicitly OUT of scope — follow-up with
  beamline time).
- `trajectory_*` attrs, `lineMode`; `update_trajectory(direction=...)` —
  varying-slot fast axis, `line_velocity = |span| / (n × dwell_ms/1000) /
  |units|`, zero-span → `XPSError`, > `config["max velocity"]` →
  `XPSError` (never stretch dwell).
- `prepareLine()` — stash cruise SGamma velocity (guarded: only when not
  already `_prepared`, so a re-prepare after an aborted line keeps the
  ORIGINAL cruise value), `moveTo(line start)`, set line velocity.
- `moveLine()` — prepared: single blocking `move_absolute` to the stop
  position with timeout `max(5, 4 × line_time + 5)`; unprepared: fully
  self-contained (stash, position, velocity, move); cruise velocity
  restored in `finally` (best-effort, never masks the original error);
  `_prepared` cleared in `finally`.
- No `positions` attribute → fly_ioc publishes nominal `np.linspace`.

### 3. Registration

`"xpsController"` joins `config.FLY_CAPABLE_CONTROLLERS`. The supervisor
then routes XPS groups to the fly IOC (same motor records + FLY group with
`daq_pvs`). `tests/iocs/test_supervisor.py::test_plan_fleet_modules`
expectations move: XPS leaves the motor_ioc list and joins fly_ioc.

### 4. Tests (mirror the MMC suite)

- `tests/iocs/test_xps_driver.py`: `FakeXPSSocket` scripted transport
  (records sends; replays canned `...,EndOfAPI` replies, including
  multi-recv fragmentation) injected as `_motion`/`_control`. Unit tests:
  framing, errcode≠0 → `XPSError`, fragmented-reply reassembly, SGamma
  parse (no eval), moveTo blocking-reply semantics + timeout → abort +
  raise, stop uses control socket while motion pending, sim mode,
  update_trajectory math (velocity/zero-span/max-velocity), prepareLine/
  moveLine split write-sequence + cruise restore + re-prepare guard.
- `tests/iocs/test_xps_fly.py`: registration test; fly-slice build for an
  xpsController group; IOC-level sim motor record move/limits + fly line
  over real CA with a DAQ service (patterned on test_mmc_fly.py).

## Error handling

`XPSError`/`SoftwareLimitError` typed raises; timeouts abort the group
before raising; IOC layer already survives driver exceptions per poll/move
and surfaces fly errors on `:ERROR`.

## Out of scope

- PVT trajectories + position-compare EXT triggering (follow-up).
- Backlash/kill/home surface beyond what exists today.
- GPIO analog functions (dropped from the legacy driver; unused).

## Hardware checklist (next beamline visit)

- Absolute vs relative move accuracy (legacy workaround dropped) — verify
  repeatability; revert `moveTo` to relative-move composition if needed.
- `GroupStatusGet` moving-status codes for the actual stage groups (43/44
  assumed for SGamma moves).
- Long-move socket-timeout margins; abort-during-move over the control
  socket (`GroupMoveAbort` vs the old disable/enable).
- SGamma restore after fly lines (cruise velocity intact after abort).
- First-line DaqClient connect latency (shared-DAQ follow-up) applies to
  XPS fly lines too.
