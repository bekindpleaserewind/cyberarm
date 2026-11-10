# -*- coding: utf-8 -*-
"""
CyberBrick robotic arm controller (MicroPython, CyberBrick Multi-Function Core Board).

Hardware (Receiver Shield)
--------------------------
    S1  -> 180 deg servo, lower arm (bottom joint)
    S2  -> 180 deg servo, upper arm
    S3  -> 360 deg continuous servo, claw open/close
    M2  -> DC motor, base rotation (360 deg)

Libraries used (CyberBrick firmware, docs at
https://makerworld.com/en/cyberbrick/api-doc/cyberbrick_core/lib/index.html):
    bbl.servos.ServosController   its PWM pins (servos_map) / set_speed / stop
    bbl.motors.MotorsController   set_speed(-2048..2048) / stop

How to run
----------
From a PC (recommended), over the radio link or the USB serial port:
    python arm_serial.py demo               # see arm_serial.py for commands

Over USB, arm_serial.py installs this file on the board (compiled) and
creates `arm` there. In radio mode, arm_radio.py starts it when the board
boots. To drive the arm with the remote's sticks, use `arm_serial.py remote`.

From the board's REPL, once it is installed:
    >>> import cyberbrick_arm as ca
    >>> arm = ca.CyberBrickArm()
    >>> arm.move_lower_arm(120)
    >>> arm.move_upper_arm(60, speed=10)
    >>> arm.turn_base(45)                   # face 45 deg right of centre
    >>> arm.turn_base(-90)                  # face 90 deg left (the limit)
    >>> arm.open_claw(); arm.close_claw()
    >>> arm.shutdown()

Calibration
-----------
The base motor and the 360 deg claw servo have no position feedback, so their
movement is time-based. The constants in the CONFIG section are the defaults;
the calibration commands save measured values in CAL_FILE on the board, which
override them:
    * CLAW_OPEN_MS / CLAW_CLOSE_MS  full open / close time at CLAW_SPEED
                      (measure with `python arm_serial.py claw-calibrate`)
    * CLAW_START_OPEN whether the claw is open when the script starts
    * BASE_RIGHT_MS / BASE_LEFT_MS  time to turn 90 deg at BASE_TURN_SPEED
                      (measure with `python arm_serial.py base-tune`)
    * CLAW_OPEN_DIRECTION / BASE_DIRECTION flip if a part turns the wrong way

The base angle is an estimate (0 = centre, negative = left, positive = right)
and the base never turns more than BASE_LIMIT_DEG either side of centre.
    * *_MIN / *_MAX   joint limits that keep the arm from hitting itself
"""

import sys
import time
from machine import Timer

try:
    # Reuse the servo/motor library the stock firmware has already loaded.
    ServosController = sys.modules["bbl.servos"].ServosController
    MotorsController = sys.modules["bbl.motors"].MotorsController
except KeyError:
    try:
        # With /bbl on sys.path this loads just these two files, not the whole
        # bbl package (LEDs, executor, sleep module), which saves RAM in radio
        # mode (see arm_radio.py).
        from servos import ServosController
        from motors import MotorsController
    except ImportError:
        from bbl.servos import ServosController
        from bbl.motors import MotorsController


# =============================================================================
# CONFIG
# =============================================================================

# Ports
LOWER_ARM_SERVO = 1   # S1, 180 deg
UPPER_ARM_SERVO = 2   # S2, 180 deg
CLAW_SERVO = 3        # S3, 360 deg continuous
BASE_MOTOR = 2        # M2

# Joint limits and home pose, in servo degrees (0..180). Narrow these if the
# arm can collide with itself or the base at the extremes. `arm_serial.py
# arm-calibrate` measures them and saves them on the board (CAL_FILE).
#
# Joint positions: when a joint's HOME is at one end of its MIN..MAX travel
# (as arm-calibrate sets it), every command and status() use a position
# instead of servo degrees: 0 = home (folded), 180 = fully extended (the other
# end), spread evenly in between. Otherwise positions are plain servo degrees.
LOWER_ARM_MIN = 0
LOWER_ARM_MAX = 180
LOWER_ARM_HOME = 90
UPPER_ARM_MIN = 0
UPPER_ARM_MAX = 180
UPPER_ARM_HOME = 90

# Which way each joint moves as its position goes up (extending it), as
# found by arm-calibrate. For information only:
#   LOWER_ARM_DIRECTION  1 = extending leans the lower arm forward
#                       -1 = extending leans it back
#   UPPER_ARM_DIRECTION  1 = extending lifts the claw
#                       -1 = extending lowers the claw
#   0 = not calibrated yet
LOWER_ARM_DIRECTION = 0
UPPER_ARM_DIRECTION = 0

# Default joint speed, 1..100 (100 = jump straight to the angle).
# The library moves about 4.6 deg per 10 ms at 100, so 20 is ~90 deg/s.
JOINT_SPEED = 20

# Claw (360 deg servo, time-based position estimate: 0.0 closed, 1.0 open)
CLAW_SPEED = 50             # servo speed (%) used for automatic open/close
CLAW_OPEN_MS = 4619        # closed -> open time at CLAW_SPEED (calibrate!)
CLAW_CLOSE_MS = 4689       # open -> closed time at CLAW_SPEED (calibrate!)
CLAW_OPEN_DIRECTION = -1    # -1 on this arm: positive servo speed closes it
CLAW_START_OPEN = False     # claw state when the script starts

# Base (DC motor, speed in %, -100..100, positive turns right). There is no
# position sensor, so the angle is estimated from how long it has turned.
BASE_DIRECTION = 1          # set to -1 if positive speed turns it left
BASE_SPEED_LIMIT = 100      # hard cap on base speed (%)
BASE_DEFAULT_SPEED = 40
BASE_TURN_SPEED = 40        # speed (%) used by turn_base()
BASE_RIGHT_MS = 2000        # centre -> 90 deg right at BASE_TURN_SPEED (calibrate!)
BASE_LEFT_MS = 2000         # 90 deg right -> centre at BASE_TURN_SPEED (calibrate!)
BASE_LIMIT_DEG = 90         # never turn further than this either side of centre

# Timer driving the joint glides and the claw/base logic, 100 times a second.
TIMER_ID = 0
TICK_MS = 10

# Joint servo pulse, the same scale as the servo library (angle * 102/180 + 25
# in 1/1024ths of 20 ms) but set in nanoseconds: the library's 10-bit duty
# moves in 1.8 degree jumps, which makes slow moves jerky. Integers, so the
# timer never makes a float.
SERVO_NS0 = 488281
SERVO_NS_PER_DEG = 11068
# Joint speed 1 is SERVO_DEG_S deg/s on average (the library's step rate).
SERVO_DEG_S = 4.61

# Calibration saved on the board by claw-calibrate / base-tune /
# arm-calibrate. Values in this file replace the defaults above when the
# library starts.
CAL_FILE = "/arm_cal.json"
CAL_KEYS = ("CLAW_OPEN_MS", "CLAW_CLOSE_MS", "CLAW_OPEN_DIRECTION",
            "BASE_RIGHT_MS", "BASE_LEFT_MS", "BASE_DIRECTION",
            "LOWER_ARM_MIN", "LOWER_ARM_MAX", "LOWER_ARM_HOME", "LOWER_ARM_DIRECTION",
            "UPPER_ARM_MIN", "UPPER_ARM_MAX", "UPPER_ARM_HOME", "UPPER_ARM_DIRECTION")
JOINT_RANGE_MIN = 10    # calibrated limits must be at least this far apart


# =============================================================================
# Calibration file
# =============================================================================

def _calibration_ok(key, value):
    if not isinstance(value, int) or isinstance(value, bool):
        return False
    if key.endswith("_DIRECTION"):
        return value in (-1, 1)
    if key[-4:] in ("_MIN", "_MAX") or key[-5:] == "_HOME":
        return 0 <= value <= 180
    return 100 <= value <= 60000


def _joint_problem(values, joint):
    """Why a joint's MIN / HOME / MAX do not fit together, or None."""
    lo, home, hi = (values[joint + "_MIN"], values[joint + "_HOME"],
                    values[joint + "_MAX"])
    if not lo <= home <= hi:
        return "%s: need MIN <= HOME <= MAX, got %d / %d / %d" % (joint, lo, home, hi)
    if hi - lo < JOINT_RANGE_MIN:
        return "%s: MIN and MAX must be at least %d degrees apart" % (joint, JOINT_RANGE_MIN)
    return None


def read_calibration():
    """The values saved in CAL_FILE ({} if there are none)."""
    try:
        import json
        with open(CAL_FILE) as f:
            cal = json.load(f)
    except (OSError, ValueError):
        return {}
    return cal if isinstance(cal, dict) else {}


def _apply_calibration(cal):
    # Bad values are skipped, so a damaged file can never stop the arm starting.
    g = globals()
    new = {key: g[key] for key in CAL_KEYS}
    for key in CAL_KEYS:
        value = cal.get(key)
        if _calibration_ok(key, value):
            new[key] = value
    for joint in ("LOWER_ARM", "UPPER_ARM"):
        if _joint_problem(new, joint):
            for part in ("_MIN", "_HOME", "_MAX"):
                new[joint + part] = g[joint + part]
    g.update(new)


_apply_calibration(read_calibration())


# =============================================================================
# Arm controller
# =============================================================================

def _clamp(value, lo, hi):
    return lo if value < lo else hi if value > hi else value


class CyberBrickArm:
    """Controls the two arm joints, the claw and the rotating base."""

    def __init__(self):
        self.servos = ServosController()
        self.motors = MotorsController()

        self.set_joint_limits(True)

        # Make sure nothing is spinning before we start.
        self.motors.stop(BASE_MOTOR)
        self.servos.stop(CLAW_SERVO)

        # Claw state. The position is kept as whole milliseconds of travel
        # (0 = closed, CLAW_OPEN_MS = open) so the 10 ms timer only does
        # integer maths: on this board every float creates a new object, and
        # doing that 100 times a second fragments the little free RAM.
        # _claw_cmd is replaced in one assignment so the timer never sees a
        # half-written command:
        #   None                    idle
        #   ("to", target_ms)       move to that position
        #   ("run", speed, free)    spin at speed; free=True ignores limits
        self._claw_pos = CLAW_OPEN_MS if CLAW_START_OPEN else 0
        self._claw_cmd = None
        self._claw_applied = 0
        self._claw_last_ms = time.ticks_ms()

        # Last error raised inside the timer (shown by status()).
        self.timer_error = None

        # Base state, kept like the claw's: _base_pos is whole milliseconds of
        # right turn at BASE_TURN_SPEED from centre (BASE_RIGHT_MS = 90 deg
        # right, negative = left). _base_cmd:
        #   None                         idle
        #   ("to", target, direction)    turn to that position
        #   ("run", speed, stop_at)      spin at speed; stop_at is a tick or None
        # Every command stops at +-_base_lim, the BASE_LIMIT_DEG estimate.
        self._base_lim = BASE_RIGHT_MS * BASE_LIMIT_DEG // 90
        self._base_pos = 0
        self._base_cmd = None
        self._base_applied = 0
        self._base_last_ms = time.ticks_ms()

        self._running = True

        # Joints: _cd is each servo's angle in hundredths of a degree, so the
        # timer works in integers. _glide holds the moves in progress:
        #   servo -> (start_cd, change_cd, start tick, duration ms)
        # The servo library's own stepping is not used; switch it off in case
        # the stock firmware left a step running.
        self._joints = (LOWER_ARM_SERVO, UPPER_ARM_SERVO)
        self._cd = {}
        self._glide = {}
        for info in self.servos.servos_info_map:
            info["step_en"] = False
        # No position feedback, so start by sending the joints home.
        for servo, angle in self._home.items():
            self._set_servo(servo, angle)

        self._timer = Timer(TIMER_ID)
        self._timer.init(period=TICK_MS, mode=Timer.PERIODIC,
                         callback=self._tick)

    # ------------------------------------------------------------------ timer

    def _tick(self, _timer):
        # Runs 100 times a second, so it must not allocate: no floats, no new
        # tuples or lists. Garbage made here would fill the heap and make the
        # board stop for a collection every second or so while moving.
        try:
            now = time.ticks_ms()
            if self._glide:
                self._update_joints(now)
            self._update_claw(now)
            self._update_base(now)
        except Exception as e:
            self.timer_error = e
            self.motors.stop(BASE_MOTOR)
            self.servos.stop(CLAW_SERVO)
            self._claw_cmd = None
            self._claw_applied = 0
            self._base_cmd = None
            self._base_applied = 0

    def _write_cd(self, servo, cd):
        """Drive a joint servo to `cd` hundredths of a degree."""
        self._cd[servo] = cd
        self.servos.servos_map[servo - 1].duty_ns(
            SERVO_NS0 + cd * SERVO_NS_PER_DEG // 100)

    def _set_servo(self, servo, angle):
        """Put a joint servo at `angle` degrees now, stopping any glide."""
        self._glide.pop(servo, None)
        self._write_cd(servo, int(angle * 100 + 0.5))

    def _update_joints(self, now):
        # Each glide eases in and out (smoothstep), so a joint speeds up and
        # slows down gently instead of jolting at the start, end and turns.
        # u and s are fractions in 1/1024ths: s = u * u * (3 - 2 * u).
        glide = self._glide
        for servo in self._joints:
            g = glide.get(servo)
            if g is None:
                continue
            start, change, t0, dur = g
            t = time.ticks_diff(now, t0)
            if t >= dur:
                del glide[servo]
                self._write_cd(servo, start + change)
                continue
            u = t * 1024 // dur
            s = ((u * u) >> 10) * (3072 - 2 * u) >> 10
            self._write_cd(servo, start + change * s // 1024)

    def _update_claw(self, now):
        dt = time.ticks_diff(now, self._claw_last_ms)
        self._claw_last_ms = now

        applied = self._claw_applied
        if applied:
            if applied > 0:
                step = applied * dt // CLAW_SPEED
            else:
                # Closing may be faster or slower than opening; scale it
                # into the same units.
                step = (applied * dt * CLAW_OPEN_MS
                        // (CLAW_SPEED * CLAW_CLOSE_MS))
            self._claw_pos = _clamp(self._claw_pos + step, 0, CLAW_OPEN_MS)

        cmd = self._claw_cmd
        want = 0
        if cmd is not None:
            pos = self._claw_pos
            if cmd[0] == "to":
                err = cmd[1] - pos
                if -TICK_MS <= err <= TICK_MS:
                    self._claw_cmd = None
                else:
                    want = CLAW_SPEED if err > 0 else -CLAW_SPEED
            else:  # "run"
                speed, free = cmd[1], cmd[2]
                at_limit = ((speed > 0 and pos >= CLAW_OPEN_MS)
                            or (speed < 0 and pos <= 0))
                if at_limit and not free:
                    self._claw_cmd = None
                else:
                    want = speed

        if want != applied:
            if want == 0:
                self.servos.stop(CLAW_SERVO)
            else:
                self.servos.set_speed(CLAW_SERVO, want * CLAW_OPEN_DIRECTION)
            self._claw_applied = want

    def _update_base(self, now):
        dt = time.ticks_diff(now, self._base_last_ms)
        self._base_last_ms = now

        applied = self._base_applied
        if applied > 0:
            self._base_pos += applied * dt // BASE_TURN_SPEED
        elif applied < 0:
            # Turning left may be faster or slower than right; scale it
            # into the same units.
            self._base_pos += (applied * dt * BASE_RIGHT_MS
                               // (BASE_TURN_SPEED * BASE_LEFT_MS))

        cmd = self._base_cmd
        want = 0
        if cmd is not None:
            pos = self._base_pos
            lim = self._base_lim
            if cmd[0] == "to":
                target, direction = cmd[1], cmd[2]
                if ((direction > 0 and pos >= target - TICK_MS)
                        or (direction < 0 and pos <= target + TICK_MS)):
                    self._base_cmd = None
                else:
                    want = direction * BASE_TURN_SPEED
            else:  # "run"
                speed, stop_at = cmd[1], cmd[2]
                # Stop one tick early so the next tick can't carry it past.
                lim -= abs(speed) * TICK_MS // BASE_TURN_SPEED
                if ((speed > 0 and pos >= lim) or (speed < 0 and pos <= -lim)
                        or (stop_at is not None
                            and time.ticks_diff(now, stop_at) >= 0)):
                    self._base_cmd = None
                else:
                    want = speed

        if want != applied:
            if want == 0:
                self.motors.stop(BASE_MOTOR)
            else:
                self.motors.set_speed(BASE_MOTOR,
                                      want * BASE_DIRECTION * 2047 // 100)
            self._base_applied = want

    # ----------------------------------------------------------------- joints

    def _check_joint(self, servo):
        if servo not in self._limits:
            raise ValueError("servo must be LOWER_ARM_SERVO (1) or UPPER_ARM_SERVO (2)")

    def _position_range(self, servo):
        return (0, 180) if self._map[servo] else self._limits[servo]

    def _to_servo(self, servo, position):
        """Servo degrees for a joint position."""
        m = self._map[servo]
        if m is None:
            return position
        return m[0] + (m[1] - m[0]) * position / 180

    def _to_position(self, servo, angle):
        """Joint position for servo degrees."""
        m = self._map[servo]
        if m is None:
            return angle
        return (angle - m[0]) * 180 / (m[1] - m[0])

    def servo_angle(self, servo):
        """Current commanded angle of a joint's servo (degrees)."""
        self._check_joint(servo)
        cd = self._cd[servo]
        return cd // 100 if cd % 100 == 0 else cd / 100

    def joint_angle(self, servo):
        """Current position of a joint: 0 (home) .. 180 (fully extended) once
        calibrated, plain servo degrees before."""
        angle = self.servo_angle(servo)
        want = self._want.get(servo)
        # The servo is driven in whole degrees; report the position that was
        # asked for once it is there, not the rounded one.
        if want is not None and abs(self._to_servo(servo, want) - angle) <= 0.5:
            return want
        return self._to_position(servo, angle)

    def home_position(self, servo):
        """The joint's home as a position (0 once calibrated)."""
        self._check_joint(servo)
        return 0 if self._map[servo] else self._home[servo]

    def move_joint(self, servo, angle, speed=JOINT_SPEED, wait=True):
        """Move a joint to a position (`angle`), kept within its limits:
        0 (home) .. 180 (fully extended) once calibrated, else servo degrees.

        speed: 1..100, where 100 jumps straight there.
        """
        self._check_joint(servo)
        lo, hi = self._position_range(servo)
        position = _clamp(angle, lo, hi)
        self._want[servo] = position
        target = int(self._to_servo(servo, position) + 0.5)
        speed = int(_clamp(speed, 1, 100))
        start = self._cd[servo]
        change = target * 100 - start
        dur = int(abs(change) * 10 / (speed * SERVO_DEG_S))
        if speed >= 100 or dur < TICK_MS:
            self._set_servo(servo, target)
        else:
            self._glide[servo] = (start, change, time.ticks_ms(), dur)
        if wait:
            self.wait_joints()

    def move_lower_arm(self, angle, speed=JOINT_SPEED, wait=True):
        """Move the lower arm (S1)."""
        self.move_joint(LOWER_ARM_SERVO, angle, speed, wait)

    def move_upper_arm(self, angle, speed=JOINT_SPEED, wait=True):
        """Move the upper arm (S2)."""
        self.move_joint(UPPER_ARM_SERVO, angle, speed, wait)

    def move_arm(self, lower=None, upper=None, speed=JOINT_SPEED, wait=True):
        """Move both joints at the same time. None leaves a joint where it is."""
        if lower is not None:
            self.move_joint(LOWER_ARM_SERVO, lower, speed, wait=False)
        if upper is not None:
            self.move_joint(UPPER_ARM_SERVO, upper, speed, wait=False)
        if wait:
            self.wait_joints()

    def joints_moving(self):
        return bool(self._glide)

    def wait_joints(self, timeout_ms=15000):
        """Block until both joints reach their targets. Returns False on timeout."""
        start = time.ticks_ms()
        while self.joints_moving():
            if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                return False
            time.sleep_ms(10)
        return True

    def hold_joints(self):
        """Stop any joint movement in progress and hold the current angles."""
        for servo in self._limits:
            self._glide.pop(servo, None)
            self._write_cd(servo, self._cd[servo])
            self._want.pop(servo, None)

    def release_joints(self):
        """Cut the signal to S1/S2 so the joints go limp (the arm may drop)."""
        for servo in self._limits:
            self._glide.pop(servo, None)
            self.servos.stop(servo)

    # ------------------------------------------------------------------- claw

    def claw_position(self):
        """Estimated claw opening: 0.0 closed .. 1.0 open."""
        return self._claw_pos / CLAW_OPEN_MS

    def claw_moving(self):
        return self._claw_cmd is not None or self._claw_applied != 0

    def set_claw(self, position, wait=True):
        """Move the claw to an estimated opening (0.0 closed .. 1.0 open)."""
        position = _clamp(float(position), 0.0, 1.0)
        self._claw_cmd = ("to", int(position * CLAW_OPEN_MS + 0.5))
        if wait:
            self.wait_claw()

    def open_claw(self, wait=True):
        self.set_claw(1.0, wait)

    def close_claw(self, wait=True):
        self.set_claw(0.0, wait)

    def claw_run(self, speed, ignore_limits=False):
        """Spin the claw servo at `speed` % (positive opens) until claw_stop().

        Stops by itself at fully open/closed unless ignore_limits=True, which
        is useful for squeezing an object or calibrating.
        """
        speed = int(_clamp(speed, -100, 100))
        self._claw_cmd = ("run", speed, ignore_limits) if speed else None

    def claw_stop(self):
        self._claw_cmd = None

    def wait_claw(self, timeout_ms=None):
        """Block until an open/close/set_claw move finishes."""
        if timeout_ms is None:
            timeout_ms = CLAW_OPEN_MS + CLAW_CLOSE_MS + 1000
        start = time.ticks_ms()
        while self.claw_moving():
            if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                self.claw_stop()
                return False
            time.sleep_ms(10)
        return True

    def calibrate_claw(self, position):
        """Tell the controller where the claw is now: True/1.0 fully open,
        False/0.0 fully closed, or anything in between."""
        self._claw_cmd = None
        self._claw_pos = int(_clamp(float(position), 0.0, 1.0) * CLAW_OPEN_MS + 0.5)

    # ------------------------------------------------------------------- base

    def base_angle(self):
        """Estimated base angle in degrees: 0 centre, negative left, positive right."""
        return self._base_pos * 90 / BASE_RIGHT_MS

    def set_base_angle(self, angle=0):
        """Tell the controller where the base is now (degrees from centre)."""
        self._base_cmd = None
        self._base_pos = int(float(angle) * BASE_RIGHT_MS / 90)

    def turn_base(self, angle, wait=True):
        """Turn the base to face `angle` degrees from centre.

        Negative is left, positive is right. Clamped to +-BASE_LIMIT_DEG.
        """
        angle = _clamp(float(angle), -BASE_LIMIT_DEG, BASE_LIMIT_DEG)
        target = int(angle * BASE_RIGHT_MS / 90)
        diff = target - self._base_pos
        if -TICK_MS <= diff <= TICK_MS:
            self._base_cmd = None
        else:
            self._base_cmd = ("to", target, 1 if diff > 0 else -1)
        if wait:
            self.wait_base()

    def turn_base_by(self, degrees, wait=True):
        """Turn the base `degrees` from where it is (negative = left).

        The result is still kept within +-BASE_LIMIT_DEG of centre.
        """
        self.turn_base(self.base_angle() + float(degrees), wait)

    def base_speed(self, speed):
        """Spin the base at `speed` % (-100..100, positive = right) until
        base_stop(). It still stops by itself at +-BASE_LIMIT_DEG."""
        speed = int(_clamp(speed, -BASE_SPEED_LIMIT, BASE_SPEED_LIMIT))
        self._base_cmd = ("run", speed, None) if speed else None

    def rotate_base(self, speed, duration_ms, wait=True):
        """Spin the base at `speed` % for `duration_ms` (or until the limit)."""
        speed = int(_clamp(speed, -BASE_SPEED_LIMIT, BASE_SPEED_LIMIT))
        if speed:
            stop_at = time.ticks_add(time.ticks_ms(), int(duration_ms))
            self._base_cmd = ("run", speed, stop_at)
        else:
            self._base_cmd = None
        if wait:
            self.wait_base()

    def base_stop(self):
        self._base_cmd = None
        self.motors.stop(BASE_MOTOR)
        self._base_applied = 0

    def base_moving(self):
        return self._base_cmd is not None or self._base_applied != 0

    def set_joint_limits(self, enabled=True):
        """Use the calibrated joint settings, or (enabled=False, only for
        calibration) drive the servos in plain degrees over their whole
        0..180 range."""
        g = globals()
        self._limits, self._map, self._home, self._want = {}, {}, {}, {}
        for servo, joint in ((LOWER_ARM_SERVO, "LOWER_ARM"), (UPPER_ARM_SERVO, "UPPER_ARM")):
            lo, hi, home = g[joint + "_MIN"], g[joint + "_MAX"], g[joint + "_HOME"]
            self._home[servo] = home                     # servo degrees
            if not enabled:
                self._limits[servo], self._map[servo] = (0, 180), None
                continue
            self._limits[servo] = (lo, hi)
            # (home, fully extended) in servo degrees when home is at one end;
            # positions then run 0 (home) .. 180 (extended).
            self._map[servo] = ((home, hi) if home == lo else
                                (home, lo) if home == hi else None)

    def set_base_limit(self, enabled=True):
        """Turn the +-BASE_LIMIT_DEG stop off (only for calibration) or back on."""
        self._base_lim = (BASE_RIGHT_MS * BASE_LIMIT_DEG // 90) if enabled else 1 << 28

    def wait_base(self, timeout_ms=None):
        """Block until a base turn finishes. Returns False on timeout."""
        if timeout_ms is None:
            timeout_ms = 2 * (BASE_RIGHT_MS + BASE_LEFT_MS) + 1000
        start = time.ticks_ms()
        while self.base_moving():
            if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                self.base_stop()
                return False
            time.sleep_ms(10)
        return True

    # ---------------------------------------------------------------- general

    def home(self, speed=JOINT_SPEED, wait=True):
        """Move both joints to the home pose and turn the base to centre."""
        self.move_arm(self.home_position(LOWER_ARM_SERVO),
                      self.home_position(UPPER_ARM_SERVO), speed, wait=False)
        self.turn_base(0, wait=False)
        if wait:
            self.wait_joints()
            self.wait_base()

    def stop_all(self):
        """Stop the base and claw and freeze the joints where they are."""
        self.base_stop()
        self.claw_stop()
        self.hold_joints()

    def status(self):
        s = {
            "lower_arm": round(self.joint_angle(LOWER_ARM_SERVO), 1),
            "upper_arm": round(self.joint_angle(UPPER_ARM_SERVO), 1),
            "claw": round(self.claw_position(), 2),
            "base_angle": round(self.base_angle()),
            "base_speed": self._base_applied,
        }
        if self.timer_error is not None:
            s["timer_error"] = repr(self.timer_error)
        return s

    def calibration(self, keys=None):
        """The calibration values in use: {name: value} for `keys` (default
        CAL_KEYS). Over the radio, ask for a few keys at a time so the reply
        fits in one message."""
        g = globals()
        return {key: g[key] for key in (keys or CAL_KEYS) if key in CAL_KEYS}

    def save_calibration(self, values):
        """Save calibration values in CAL_FILE on the board and use them now.

        values: {name: int} for names in CAL_KEYS. The base angle and claw
        opening estimates are kept.
        """
        cal = read_calibration()
        for key, value in values.items():
            value = int(value)
            if key not in CAL_KEYS or not _calibration_ok(key, value):
                raise ValueError("bad calibration value %s=%r" % (key, value))
            cal[key] = value
        g = globals()
        merged = {key: cal.get(key, g[key]) for key in CAL_KEYS}
        for joint in ("LOWER_ARM", "UPPER_ARM"):
            problem = _joint_problem(merged, joint)
            if problem:
                raise ValueError(problem)
        import json
        import os
        base, claw = self.base_angle(), self.claw_position()
        tmp = CAL_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(cal, f)
        try:
            os.remove(CAL_FILE)
        except OSError:
            pass
        os.rename(tmp, CAL_FILE)
        _apply_calibration(cal)
        # Positions are kept in milliseconds of travel, so re-express them in
        # the new timings.
        self.set_base_limit(True)
        self.set_joint_limits(True)
        self.set_base_angle(base)
        self.calibrate_claw(claw)
        return self.calibration(list(values))

    def shutdown(self, release=False):
        """Stop the timer, base and claw. Joints keep holding unless release=True."""
        if not self._running:
            return
        self._running = False
        self._timer.deinit()
        self.motors.stop(BASE_MOTOR)
        self.servos.stop(CLAW_SERVO)
        self._claw_cmd = None
        self._claw_applied = 0
        self._base_cmd = None
        self._base_applied = 0
        if release:
            self.release_joints()
        else:
            self.hold_joints()
