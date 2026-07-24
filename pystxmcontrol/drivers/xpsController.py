"""Newport XPS controller driver (TCP, port 5001).

Structure-only rewrite of David's driver: same XPS function calls, same
dual-socket flow (control socket for motion/parameter commands, monitor
socket for position polling), same disable/enable abort. What changed is
code structure: ONE framing/parsing site (_transact) with typed XPSError
(no [-2, ''] sentinels, no eval()), lock-agnostic (the IOC layer's
io_lock serializes), and a working simulation mode.

Known device-interaction quirks are deliberately preserved -- see the
spec's "Itemized device-interaction weaknesses"
(docs/superpowers/specs/2026-07-24-xps-fly-integration-design.md).
"""
import socket
import time

from pystxmcontrol.controller.hardwareController import hardwareController


class XPSError(IOError):
    """Socket failure, malformed reply, or nonzero XPS error code."""


class xpsController(hardwareController):

    def __init__(self, address="192.168.168.253", port=5001, simulation=False):
        self.address = address
        self.port = int(port) if port else 5001
        self.simulation = simulation
        self._control = None   # motion commands, SGamma set, disable/enable
        self._monitor = None   # position queries during moves
        # Simulation state shared by all xpsMotor instances:
        self.positions = {}    # group -> position (controller units)
        self.sgamma = {}       # positioner -> [vel, accel, minJerk, maxJerk]

    def initialize(self, simulation=False):
        self.simulation = simulation
        if self.simulation:
            return
        print(f"Connecting to XPS controller on {self.address}:{self.port}",
              flush=True)
        self._control = self._open_socket()
        self._monitor = self._open_socket()

    def _open_socket(self):
        try:
            sock = socket.create_connection((str(self.address), self.port),
                                            timeout=5.0)
        except OSError as exc:
            raise XPSError(
                f"failed to connect to XPS on {self.address}:{self.port}: {exc}")
        sock.settimeout(1.0)
        return sock

    # ---- framing/parsing: the ONE place XPS wire format lives ----------
    def _transact(self, sock, command, timeout=None) -> str:
        """Send a command and read its full ``err,payload,EndOfAPI`` reply.

        Raises XPSError on socket error/timeout, malformed reply, or a
        nonzero XPS error code. ``timeout`` temporarily overrides the
        socket timeout for this transaction.
        """
        if self.simulation:
            raise XPSError("_transact has no meaning in simulation mode")
        old = sock.gettimeout()
        if timeout is not None:
            sock.settimeout(timeout)
        try:
            sock.send(command.encode())
            response = ""
            while ",EndOfAPI" not in response:
                chunk = sock.recv(1024).decode(errors="replace")
                if not chunk:
                    raise XPSError(f"connection closed during {command!r}")
                response += chunk
        except socket.timeout:
            raise XPSError(f"timeout waiting for reply to {command!r}")
        except OSError as exc:
            raise XPSError(f"socket error during {command!r}: {exc}")
        finally:
            if timeout is not None:
                sock.settimeout(old)
        err_str, _, rest = response.partition(",")
        payload = rest[: rest.rfind(",EndOfAPI")] if rest.rfind(",EndOfAPI") >= 0 \
            else rest[: rest.rfind("EndOfAPI")]
        try:
            err = int(err_str)
        except ValueError:
            raise XPSError(f"malformed XPS reply to {command!r}: {response!r}")
        if err != 0:
            raise XPSError(f"XPS error {err} for {command!r}: {payload!r}")
        return payload

    # ---- protocol wrappers (legacy XPS function set, unchanged) ---------
    def move_relative(self, group, displacement):
        """Fire GroupMoveRelative on the control socket.

        David's flow: the XPS answers a motion command only when the move
        COMPLETES, so we attempt a short (socket-default 1 s) reply read --
        an immediate error reply (e.g. group disabled) surfaces as
        XPSError, while a read timeout means "move in progress" and is
        swallowed; completion is polled via get_position on the monitor
        socket by the motor. (Spec weakness #6: the eventual reply is left
        unread; the next control-socket _transact may need to tolerate it
        -- preserved behavior.)
        """
        if self.simulation:
            self.positions[group] = self.positions.get(group, 0.0) + displacement
            return
        try:
            self._transact(self._control,
                           f"GroupMoveRelative({group},{displacement})")
        except XPSError as exc:
            if "timeout waiting for reply" in str(exc):
                return  # move in progress; completion is polled
            raise

    def get_position(self, group) -> float:
        if self.simulation:
            return self.positions.get(group, 0.0)
        payload = self._transact(
            self._monitor, f"GroupPositionCurrentGet({group},double *)")
        try:
            return float(payload.split(",")[0])
        except ValueError:
            raise XPSError(f"unparseable position payload {payload!r}")

    def get_sgamma(self, positioner) -> list:
        if self.simulation:
            return list(self.sgamma.get(positioner, [10.0, 80.0, 0.02, 0.04]))
        payload = self._transact(
            self._control,
            f"PositionerSGammaParametersGet({positioner},double *,double *,"
            f"double *,double *)")
        try:
            values = [float(v) for v in payload.split(",")]
        except ValueError:
            raise XPSError(f"unparseable SGamma payload {payload!r}")
        if len(values) != 4:
            raise XPSError(f"expected 4 SGamma values, got {payload!r}")
        return values

    def set_sgamma(self, positioner, velocity, acceleration, min_jerk, max_jerk):
        if self.simulation:
            self.sgamma[positioner] = [float(velocity), float(acceleration),
                                       float(min_jerk), float(max_jerk)]
            return
        self._transact(
            self._control,
            f"PositionerSGammaParametersSet({positioner},{velocity},"
            f"{acceleration},{min_jerk},{max_jerk})")

    def disable_group(self, group):
        if not self.simulation:
            self._transact(self._control, f"GroupMotionDisable({group})")

    def enable_group(self, group):
        if not self.simulation:
            self._transact(self._control, f"GroupMotionEnable({group})")

    def abort_move(self, group):
        """David's abort: disable, 1 s, enable, 1 s. (Spec weakness #1:
        GroupMoveAbort is the candidate improvement -- NOT used yet.)

        enable_group is ALWAYS attempted, even if disable_group raises (e.g.
        it misattributes a pending move reply arriving at disable time) --
        otherwise the servo is stranded disabled. A disable-side XPSError
        still propagates after the finally so callers see the abort failed.
        """
        try:
            self.disable_group(group)
        finally:
            time.sleep(1)
            try:
                self.enable_group(group)
            except XPSError as exc:
                print(f"[xpsController] enable after abort failed: {exc!r}",
                      flush=True)
            time.sleep(1)

