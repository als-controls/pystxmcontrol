"""Newport XPS axis driver (point-to-point + constant-velocity fly lines).

David's device-interaction semantics preserved verbatim (see the spec's
itemized weaknesses): relative moves composed from the current position,
completion by position tolerance polled on the monitor socket, the
disable/enable abort, and setAxisParams' legacy velocity*1000 scale.
Fly lines (line_trigger = "IMM": free-running DAQ) reuse moveTo at the
line velocity -- SGamma velocity is set RAW for lines.
"""
import time

from pystxmcontrol.controller.motor import motor, SoftwareLimitError
from pystxmcontrol.drivers.xpsController import XPSError


class xpsMotor(motor):

    #: DAQ trigger source for fly lines (fly_ioc reads this); the XPS PVT
    #: trajectory + position-compare EXT trigger is a recorded follow-up.
    line_trigger = "IMM"

    def __init__(self, controller=None, config=None):
        self.controller = controller
        self.config = config
        self.simulation = False
        self.axis = None            # full "Group.Positioner"
        self.group = None
        self.moving = False
        self.velocity = 0.0
        self.acceleration = 0.0
        self.minimumJerkTime = 0.0
        self.maximumJerkTime = 0.0
        # fly interface (contract identical to mmcMotor)
        self.lineMode = "raster"
        self.trajectory_start = (0.0, 0.0)
        self.trajectory_stop = (0.0, 0.0)
        self.trajectory_pixel_count = 10
        self.trajectory_pixel_dwell = 1.0   # ms per pixel
        self.npositions = 10
        self.line_velocity = 0.0
        self._line_start = 0.0
        self._line_stop = 0.0
        self._prepared = False
        self._cruise_velocity = None
        self._poll = 0.1                    # legacy 100 ms position poll

    # ---- helpers --------------------------------------------------------
    def _to_controller(self, pos):
        return (pos - self.config["offset"]) / self.config["units"]

    def _from_controller(self, pos):
        return pos * self.config["units"] + self.config["offset"]

    @property
    def _tolerance(self):
        return float(self.config.get("position_tolerance", 5.0))

    # ---- point-to-point interface ---------------------------------------
    def connect(self, axis=None, **kwargs):
        if "logger" in kwargs:
            self.logger = kwargs["logger"]
        self.simulation = self.controller.simulation
        self.axis = axis
        self.group = axis.split(".")[0]
        (self.velocity, self.acceleration, self.minimumJerkTime,
         self.maximumJerkTime) = self.controller.get_sgamma(self.axis)
        if not self.simulation:
            self.position = self.getPos()
        return True

    def checkLimits(self, pos):
        lo, hi = self.config["minValue"], self.config["maxValue"]
        if pos < lo:
            self.moving = False
            raise SoftwareLimitError(self.axis, pos, lo, limit_type="lower")
        if pos > hi:
            self.moving = False
            raise SoftwareLimitError(self.axis, pos, hi, limit_type="upper")
        return True

    def getPos(self):
        return self._from_controller(self.controller.get_position(self.group))

    def getStatus(self, **kwargs):
        return self.moving

    def moveTo(self, pos, timeout=None):
        """David's semantics: relative move composed from the current
        position, completion = position within tolerance polled on the
        monitor socket, timeout -> disable/enable abort + raise."""
        self.checkLimits(pos)
        target = self._to_controller(pos)
        if self.simulation:
            self.controller.positions[self.group] = target
            self.moving = False
            return
        if timeout is None:
            timeout = float(self.config.get("timeout", 1))
        current = self.controller.get_position(self.group)
        self.moving = True
        try:
            self.controller.move_relative(self.group,
                                          round(target - current, 6))
            t0 = time.time()
            while abs(target - self.controller.get_position(self.group)) \
                    > self._tolerance:
                if time.time() - t0 > timeout:
                    self.controller.abort_move(self.group)
                    raise XPSError(
                        f"XPS axis {self.axis} move to {pos} timed out "
                        f"after {timeout}s")
                time.sleep(self._poll)
        finally:
            self.moving = False

    def moveBy(self, step):
        self.moveTo(self.getPos() + step)

    def stop(self):
        if not self.simulation:
            self.controller.abort_move(self.group)
        self.moving = False

    def setAxisParams(self, velocity):
        """Legacy quirk preserved: velocity is scaled x1000 on the wire
        (spec weakness #4). Fly lines use _set_line_velocity (raw)."""
        self.velocity = float(velocity)
        self.controller.set_sgamma(self.axis, self.velocity * 1000,
                                   self.acceleration, self.minimumJerkTime,
                                   self.maximumJerkTime)

    def get_velocity(self):
        (vel, self.acceleration, self.minimumJerkTime,
         self.maximumJerkTime) = self.controller.get_sgamma(self.axis)
        return vel

    def disable(self):
        self.controller.disable_group(self.group)

    def enable(self):
        self.controller.enable_group(self.group)

    # ---- fly interface (contract identical to mmcMotor) ------------------
    def _set_line_velocity(self, v):
        """RAW SGamma velocity write (no legacy x1000) for fly lines."""
        self.controller.set_sgamma(self.axis, v, self.acceleration,
                                   self.minimumJerkTime, self.maximumJerkTime)

    def update_trajectory(self, direction="forward", include_return=False):
        x0, y0 = self.trajectory_start
        x1, y1 = self.trajectory_stop
        if abs(x1 - x0) >= abs(y1 - y0):
            start, stop = x0, x1
        else:
            start, stop = y0, y1
        if direction == "backward":
            start, stop = stop, start
        if start == stop:
            raise XPSError("fly line has zero span")
        line_time = self.trajectory_pixel_count * \
            self.trajectory_pixel_dwell / 1000.0
        if line_time <= 0:
            raise XPSError("fly line has non-positive duration")
        velocity = abs(stop - start) / line_time / abs(self.config["units"])
        max_v = self.config.get("max velocity")
        if max_v and velocity > float(max_v):
            raise XPSError(
                f"fly line needs {velocity:.3f} units/s > max velocity "
                f"{max_v}; increase dwell or shorten the line")
        self._line_start, self._line_stop = start, stop
        self.line_velocity = velocity
        self.npositions = self.trajectory_pixel_count

    def prepareLine(self):
        """Pre-position + set line velocity BEFORE the DAQ is armed
        (line_trigger IMM: arming starts acquisition immediately)."""
        if self.simulation:
            self._prepared = True
            return
        # Abort window: a line aborted between prepareLine and moveLine
        # leaves the axis at line velocity with _prepared True until the
        # next completed line restores cruise. Only stash cruise when not
        # already prepared, so a re-prepare keeps the ORIGINAL cruise.
        if not self._prepared:
            self._cruise_velocity = self.get_velocity()
        self.moveTo(self._line_start)
        self._set_line_velocity(self.line_velocity)
        self._prepared = True

    def moveLine(self, **kwargs):
        line_time = self.trajectory_pixel_count * \
            self.trajectory_pixel_dwell / 1000.0
        deadline = max(5.0, line_time * 4.0 + 5.0)
        if self.simulation:
            time.sleep(min(line_time, 0.1))
            self.controller.positions[self.group] = \
                self._to_controller(self._line_stop)
            return
        if not self._prepared:
            self._cruise_velocity = self.get_velocity()
            self.moveTo(self._line_start)
            self._set_line_velocity(self.line_velocity)
        try:
            self.moveTo(self._line_stop, timeout=deadline)
        finally:
            self._prepared = False
            try:
                if self._cruise_velocity is not None:
                    self._set_line_velocity(self._cruise_velocity)
            except Exception as exc:  # noqa: BLE001 - never mask the line error
                print(f"[xpsMotor] cruise velocity restore failed: {exc!r}",
                      flush=True)
