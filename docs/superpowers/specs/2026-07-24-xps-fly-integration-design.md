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
Note: motion-command replies arrive at move COMPLETION on the issuing
socket — which is why David's flow polls position on a second (monitor)
socket rather than waiting for the reply. Functions used (the legacy set,
unchanged): `GroupMoveRelative(group, displacement)`,
`GroupPositionCurrentGet(group, double *)`,
`PositionerSGammaParametersGet/Set(positioner, vel, accel, minJerkT, maxJerkT)`,
`GroupMotionDisable/Enable(group)`.
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
**Guiding rule (Ron): keep David's known-good DEVICE-INTERACTION approaches
verbatim; the rewrite is code structure only** (typed errors, no `eval()`,
one framing/parsing site, lock-agnostic, simulation mode). Anything that
looks like a device-interaction weakness is itemized for later review (see
"Itemized device-interaction weaknesses"), NOT changed in this pass.

- **Two persistent TCP sockets**, both created in `initialize()` — same as
  David's control + monitor socket pair:
  - `_control` — motion commands, SGamma set, disable/enable.
  - `_monitor` — position/status queries, so completion polling reads
    positions while a move command's reply is still pending on `_control`.
- `_transact(sock, command, timeout=None) -> str payload`: encode, send,
  recv-loop until `,EndOfAPI`, split off errcode, raise `XPSError` if
  nonzero. All framing/parsing lives HERE (this replaces the scattered
  `__sendAndReceive` copies and the `eval()` in getParameters — structure
  only, the wire traffic is unchanged).
- Thin protocol wrappers (all raise `XPSError` on failure), each emitting
  EXACTLY the same XPS function calls the legacy driver used:
  `move_relative(group, displacement)` (fire on `_control`; completion is
  polled, David's semantics), `get_position(group) -> float` (`_monitor`),
  `get_sgamma(positioner)` / `set_sgamma(positioner, ...)`,
  `disable_group(group)` / `enable_group(group)`.
- `abort_move(group)` — David's abort: `GroupMotionDisable`, 1 s pause,
  `GroupMotionEnable`, 1 s pause. (GroupMoveAbort is itemized below as a
  candidate improvement, NOT used now.)
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
- `moveTo(pos)` — **David's semantics preserved exactly**: limits check;
  offset/units transform; RELATIVE move composed from the current position
  (`GroupMoveRelative(group, target - current)` — the documented workaround
  for "an XPS can get in a state where absolute moves are inaccurate");
  completion by polling `get_position(group)` on the monitor socket until
  `|target - current| <= position_tolerance` (default 5.0, config
  `"position_tolerance"` override) with `config.get("timeout", ...)`
  seconds budget (legacy default preserved); on timeout: `abort_move`
  (disable/enable) then raise `XPSError`.
- `getPos()` — `get_position(group) * units + offset` (monitor socket).
- `getStatus()` — driver-local moving flag, as in the legacy driver (no
  new GroupStatusGet dependency in this pass).
- `stop()` — `abort_move(group)` (David's disable/enable abort, unchanged).
- `setAxisParams(velocity=)` / `get_velocity()` — SGamma read-modify-write
  keeping accel/jerk untouched, INCLUDING the legacy `velocity * 1000`
  scale factor in setAxisParams (itemized below; do not "fix" silently).
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
- `moveLine()` — prepared: one `moveTo(line stop)` call (David's
  relative-move + position-poll semantics) with the line deadline
  `max(5, 4 × line_time + 5)` as the timeout budget; unprepared: fully
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

## Itemized device-interaction weaknesses (kept as-is; review later with David)

Per Ron's rule these legacy behaviors are PRESERVED in this pass. Each is a
candidate improvement to review at the beamline:

1. `abort_move` = GroupMotionDisable + 1 s + GroupMotionEnable + 1 s (servo
   drop + 2 s dead time) instead of `GroupMoveAbort`.
2. Relative-move composition (`target - current`) instead of
   `GroupMoveAbsolute` — accumulates the readback error of the pre-move
   position read into the target.
3. Completion by position tolerance (default 5.0 units — coarse) rather
   than `GroupStatusGet` motion status; a move that stalls INSIDE tolerance
   reads as success.
4. `setAxisParams` multiplies velocity by 1000 (units quirk, undocumented).
5. `getStatus` is a driver-local flag, not a controller query.
6. Move command's reply (which arrives at motion end) is never read on the
   control socket before the next command on that socket — relies on the
   XPS tolerating a pending reply + new command on one connection (David's
   flow; works in practice).

## Hardware checklist (next beamline visit)

- Regression check against legacy behavior: jog, limits, stop mid-move
  (disable/enable abort), long-move timeout margins.
- SGamma restore after fly lines (cruise velocity intact after abort).
- Review the itemized weaknesses above with David; promote fixes
  individually with hardware verification.
- First-line DaqClient connect latency (shared-DAQ follow-up) applies to
  XPS fly lines too.
