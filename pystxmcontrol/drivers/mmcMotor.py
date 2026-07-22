# -*- coding: utf-8 -*-
"""Micronix MMC axis driver (point-to-point + constant-velocity fly lines).

Lock-agnostic: the IOC layer's per-controller io_lock serializes all link
I/O. Software-timed fly lines (line_trigger = "INT"): the MMC has no
trigger output, so the DAQ free-runs during the constant-velocity move.
"""
import time

from pystxmcontrol.controller.motor import motor, SoftwareLimitError
from pystxmcontrol.drivers.mmcController import MMCError

_AXIS_NUMBERS = {"x": 1, "y": 2, "z": 3}
_STATUS_STOPPED_BIT = 0x08  # STA? bit 3: axis stopped (legacy idle 8/136)


class mmcMotor(motor):

    #: DAQ trigger source for fly lines (fly_ioc reads this; the MMC has no
    #: hardware trigger output, so lines are software-timed by default).
    line_trigger = "INT"

    def __init__(self, controller=None, config=None):
        self.controller = controller
        self.config = config
        self.simulation = False
        self.axis = None
        self._axis = 1
        self.moving = False
        self.velocity = 0.0
        # fly interface (duck-typed; see iocs/fly_ioc.py _run_line)
        self.lineMode = "raster"
        self.trajectory_start = (0.0, 0.0)
        self.trajectory_stop = (0.0, 0.0)
        self.trajectory_pixel_count = 10
        self.trajectory_pixel_dwell = 1.0  # ms per pixel
        self.npositions = 10
        self.line_velocity = 0.0
        self._line_start = 0.0
        self._line_stop = 0.0
        self._poll = 0.005

    # ---- helpers -------------------------------------------------------
    def _to_controller(self, pos):
        return round((pos - self.config["offset"]) / self.config["units"], 3)

    def _from_controller(self, pos):
        return pos * self.config["units"] + self.config["offset"]

    # ---- point-to-point interface --------------------------------------
    def connect(self, axis=None, **kwargs):
        if "logger" in kwargs:
            self.logger = kwargs["logger"]
        self.simulation = self.controller.simulation
        self.axis = axis
        self._axis = int(self.config.get("controller_index",
                                         _AXIS_NUMBERS.get(axis, 1)))
        self.setServo(True)
        return True

    def checkLimits(self, pos):
        lo, hi = self.config["minValue"], self.config["maxValue"]
        if pos < lo:
            raise SoftwareLimitError(self.axis, pos, lo, limit_type="lower")
        if pos > hi:
            raise SoftwareLimitError(self.axis, pos, hi, limit_type="upper")
        return True

    def getStatus(self, **kwargs):
        if self.simulation:
            return self.moving
        status = int(self.controller.query(self._axis, "STA?"))
        self.moving = not (status & _STATUS_STOPPED_BIT)
        return self.moving

    def getPos(self):
        if self.simulation:
            return self._from_controller(
                self.controller.positions.get(self._axis, 0.0))
        payload = self.controller.query(self._axis, "POS?")
        # closed-loop reply: "<theoretical>,<encoder>"; use the encoder
        # (last) field, which is also correct for single-field replies.
        return self._from_controller(float(payload.split(",")[-1]))

    def moveTo(self, pos):
        self.checkLimits(pos)
        if self.simulation:
            self.controller.positions[self._axis] = \
                (pos - self.config["offset"]) / self.config["units"]
            self.moving = False
            return
        self.controller.command(self._axis, f"MVA{self._to_controller(pos)}")
        timeout = float(self.config.get("timeout", 10))
        t0 = time.time()
        while self.getStatus():
            if time.time() - t0 > timeout:
                self.stop()
                raise MMCError(
                    f"MMC axis {self.axis} move to {pos} timed out after "
                    f"{timeout}s; controller errors: "
                    f"{self.controller.get_errors(self._axis)!r}")
            time.sleep(self._poll)

    def moveBy(self, step):
        self.moveTo(self.getPos() + step)

    def stop(self):
        if not self.simulation:
            self.controller.command(self._axis, "STP")
        self.moving = False

    def setAxisParams(self, velocity):
        self.velocity = round(float(velocity), 3)
        if self.simulation:
            self.controller.velocities[self._axis] = self.velocity
        else:
            self.controller.command(self._axis, f"VEL{self.velocity}")

    def get_velocity(self):
        if self.simulation:
            return self.controller.velocities.get(self._axis, self.velocity)
        payload = self.controller.query(self._axis, "VEL?")
        self.velocity = float(payload.split(",")[-1])
        return self.velocity

    def setServo(self, servo=True):
        if not self.simulation:
            self.controller.command(self._axis, f"FBK{3 if servo else 0}")

    def home(self):
        if not self.simulation:
            self.controller.command(self._axis, "HOM")

    def configure_home(self, direction=0):
        if not self.simulation:
            self.controller.command(self._axis, f"HCG{int(direction)}")
