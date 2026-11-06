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
    bbl.servos.ServosController   set_angle / set_angle_stepping / set_speed / stop
    bbl.motors.MotorsController   set_speed(-2048..2048) / stop
    rc_module                     rc_slave_init / rc_slave_data (optional remote)

How to run
----------
From a PC over the USB serial port (recommended):
    python arm_serial.py --port COM5 demo   # see arm_serial.py for commands

arm_serial.py uploads this file into the board's RAM and creates `arm` there.

Standalone on the board:
    mpremote run cyberbrick_arm.py          # runs MODE below (default: demo)

Or copy it to the board and use it from the REPL:
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
movement is time-based. Tune the constants in the CONFIG section:
    * CLAW_OPEN_MS / CLAW_CLOSE_MS  full open / close time at CLAW_SPEED
                      (measure with `python arm_serial.py claw-calibrate`)
    * CLAW_START_OPEN whether the claw is open when the script starts
    * BASE_RIGHT_MS / BASE_LEFT_MS  time to turn 90 deg at BASE_TURN_SPEED
                      (measure with `python arm_serial.py base-calibrate`)
    * CLAW_OPEN_DIRECTION / BASE_DIRECTION flip if a part turns the wrong way

The base angle is an estimate (0 = centre, negative = left, positive = right)
and the base never turns more than BASE_LIMIT_DEG either side of centre.
    * *_MIN / *_MAX   joint limits that keep the arm from hitting itself
"""

import time
from machine import Timer

try:
    from bbl.servos import ServosController
    from bbl.motors import MotorsController
except ImportError:
    # Fallback when the bbl files sit directly on sys.path (e.g. /bbl).
    from servos import ServosController
    from motors import MotorsController


# =============================================================================
# CONFIG
# =============================================================================

# What to do when the file is run directly:
#   "demo"       run a scripted demo of every joint, the claw and the base
#   "rc"         drive the arm from a paired CyberBrick transmitter
#   "rc_monitor" print raw transmitter values (to find which stick is which)
#   "none"       set up the arm and hold position (for REPL use)
MODE = "demo"

# Ports
LOWER_ARM_SERVO = 1   # S1, 180 deg
UPPER_ARM_SERVO = 2   # S2, 180 deg
CLAW_SERVO = 3        # S3, 360 deg continuous
BASE_MOTOR = 2        # M2

# Joint limits and home pose (degrees, 0..180). Narrow these if the arm
# can collide with itself or the base at the extremes.
LOWER_ARM_MIN = 0
LOWER_ARM_MAX = 180
LOWER_ARM_HOME = 90
UPPER_ARM_MIN = 0
UPPER_ARM_MAX = 180
UPPER_ARM_HOME = 90

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

# Timer driving smooth servo steps and the claw/base logic. The servo library
# assumes it is called 100 times a second (10 ms).
TIMER_ID = 0
TICK_MS = 10

# Remote control mapping: (channel index, direction). Channels in
# rc_slave_data(): 0=L1 1=L2 2=L3 3=R1 4=R2 5=R3, 6..9 = K1..K4 buttons.
# Use MODE = "rc_monitor" to see which channel each stick moves.
RC_BASE = (0, 1)
RC_LOWER_ARM = (1, 1)
RC_UPPER_ARM = (4, 1)
RC_CLAW = (3, 1)
RC_BTN_HOME = 6         # K1: go to home pose
RC_BTN_OPEN = 7         # K2: open claw fully
RC_BTN_CLOSE = 8        # K3: close claw fully
RC_BTN_STOP = 9         # K4: stop everything
RC_CENTER = 2048        # stick centre (ADC 0..4095)
RC_DEADZONE = 200
RC_JOINT_RATE = 90      # deg/s at full stick deflection
RC_BASE_MAX = 60        # base speed (%) at full stick deflection
RC_LOOP_MS = 20


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

        self._limits = {
            LOWER_ARM_SERVO: (LOWER_ARM_MIN, LOWER_ARM_MAX),
            UPPER_ARM_SERVO: (UPPER_ARM_MIN, UPPER_ARM_MAX),
        }
        self._home = {
            LOWER_ARM_SERVO: LOWER_ARM_HOME,
            UPPER_ARM_SERVO: UPPER_ARM_HOME,
        }

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

        # Joints have no position feedback, so start by sending them home.
        for servo, angle in self._home.items():
            self.servos.set_angle(servo, angle)

        self._timer = Timer(TIMER_ID)
        self._timer.init(period=TICK_MS, mode=Timer.PERIODIC,
                         callback=self._tick)

    # ------------------------------------------------------------------ timer

    def _tick(self, _timer):
        try:
            self.servos.timing_proc()
            now = time.ticks_ms()
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

    def joint_angle(self, servo):
        """Current commanded angle of a joint (degrees)."""
        self._check_joint(servo)
        return self.servos.servos_info_map[servo - 1]["c_ang"]

    def joint_target(self, servo):
        """Angle the joint is moving towards (degrees)."""
        self._check_joint(servo)
        return self.servos.servos_info_map[servo - 1]["s_ang"]

    def move_joint(self, servo, angle, speed=JOINT_SPEED, wait=True):
        """Move a joint to `angle` (clamped to its limits).

        speed: 1..100, where 100 jumps straight to the angle.
        """
        self._check_joint(servo)
        lo, hi = self._limits[servo]
        # Whole degrees keep the servo library's 10 ms update on integers
        # once the joint has arrived.
        angle = int(_clamp(angle, lo, hi) + 0.5)
        speed = int(_clamp(speed, 1, 100))
        if speed >= 100:
            self.servos.set_angle(servo, angle)
        else:
            self.servos.set_angle_step(servo, speed)
            self.servos.set_angle_stepping(servo, angle)
        if wait:
            self.wait_joints()

    def jog_joint(self, servo, delta):
        """Nudge a joint by `delta` degrees immediately (used for live control)."""
        self._check_joint(servo)
        lo, hi = self._limits[servo]
        angle = _clamp(self.joint_angle(servo) + delta, lo, hi)
        self.servos.set_angle(servo, angle)

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
        for servo in self._limits:
            info = self.servos.servos_info_map[servo - 1]
            if info["step_en"] and abs(info["s_ang"] - info["c_ang"]) > 0.01:
                return True
        return False

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
            self.servos.set_angle(servo, self.joint_angle(servo))

    def release_joints(self):
        """Cut the signal to S1/S2 so the joints go limp (the arm may drop)."""
        for servo in self._limits:
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

    def center_base(self, wait=True):
        self.turn_base(0, wait)

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
        self.move_arm(LOWER_ARM_HOME, UPPER_ARM_HOME, speed, wait=False)
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


# =============================================================================
# Demo
# =============================================================================

def demo(arm):
    """Exercise every joint, the claw and the base, then pick and place."""
    print("[demo] home")
    arm.home()
    time.sleep_ms(500)

    print("[demo] claw open / close")
    arm.open_claw()
    time.sleep_ms(300)
    arm.close_claw()
    time.sleep_ms(300)

    print("[demo] lower arm (S1) sweep")
    arm.move_lower_arm(LOWER_ARM_HOME + 30)
    arm.move_lower_arm(LOWER_ARM_HOME - 30)
    arm.move_lower_arm(LOWER_ARM_HOME)

    print("[demo] upper arm (S2) sweep")
    arm.move_upper_arm(UPPER_ARM_HOME + 30)
    arm.move_upper_arm(UPPER_ARM_HOME - 30)
    arm.move_upper_arm(UPPER_ARM_HOME)

    print("[demo] base (M2) left / right")
    arm.turn_base(45)
    time.sleep_ms(300)
    arm.turn_base(-45)
    time.sleep_ms(300)
    arm.turn_base(0)

    print("[demo] pick and place")
    arm.open_claw()
    arm.move_arm(lower=LOWER_ARM_HOME + 35, upper=UPPER_ARM_HOME - 25)   # reach
    arm.close_claw()                                                     # grab
    arm.move_arm(lower=LOWER_ARM_HOME, upper=UPPER_ARM_HOME)             # lift
    arm.turn_base(60)                                                    # turn
    arm.move_arm(lower=LOWER_ARM_HOME + 35, upper=UPPER_ARM_HOME - 25)   # lower
    arm.open_claw()                                                      # drop
    arm.move_arm(lower=LOWER_ARM_HOME, upper=UPPER_ARM_HOME)             # lift
    arm.turn_base(0)                                                     # turn back
    arm.close_claw()

    print("[demo] done:", arm.status())


# =============================================================================
# Remote control (CyberBrick transmitter)
# =============================================================================

def _stick(raw, direction):
    """Convert an ADC value (0..4095) to -1.0..1.0 with a dead zone."""
    d = raw - RC_CENTER
    if -RC_DEADZONE <= d <= RC_DEADZONE:
        return 0.0
    if d > 0:
        v = (d - RC_DEADZONE) / (4095 - RC_CENTER - RC_DEADZONE)
    else:
        v = (d + RC_DEADZONE) / (RC_CENTER - RC_DEADZONE)
    return _clamp(v, -1.0, 1.0) * direction


def run_rc(arm):
    """Drive the arm live from a paired CyberBrick transmitter."""
    import rc_module

    if rc_module.rc_slave_init() is False:
        print("[rc] receiver init failed")
        return

    print("[rc] sticks: base, lower arm, upper arm, claw")
    print("[rc] K1 home | K2 open claw | K3 close claw | K4 stop all")
    print("[rc] Ctrl-C to quit")

    prev_buttons = [1, 1, 1, 1]
    link = None
    claw_by_stick = False
    last = time.ticks_ms()

    while True:
        now = time.ticks_ms()
        dt = time.ticks_diff(now, last) / 1000
        last = now

        data = rc_module.rc_slave_data()
        if not data or not any(data):
            if link is not False:
                print("[rc] no transmitter data, waiting...")
                arm.base_stop()
                arm.claw_stop()
                claw_by_stick = False
                link = False
            time.sleep_ms(RC_LOOP_MS)
            continue
        if link is not True:
            print("[rc] transmitter connected")
            link = True

        # Base
        speed = round(_stick(data[RC_BASE[0]], RC_BASE[1]) * RC_BASE_MAX)
        if speed != arm._base_applied:
            arm.base_speed(speed)

        # Joints (stick sets the rate of movement)
        for servo, (ch, direction) in ((LOWER_ARM_SERVO, RC_LOWER_ARM),
                                       (UPPER_ARM_SERVO, RC_UPPER_ARM)):
            v = _stick(data[ch], direction)
            if v:
                arm.jog_joint(servo, v * RC_JOINT_RATE * dt)

        # Claw
        v = _stick(data[RC_CLAW[0]], RC_CLAW[1])
        if v:
            arm.claw_run(v * CLAW_SPEED)
            claw_by_stick = True
        elif claw_by_stick:
            arm.claw_stop()
            claw_by_stick = False

        # Buttons (0 = pressed), act on press
        buttons = list(data[6:10])
        for idx in range(4):
            if buttons[idx] == 0 and prev_buttons[idx] != 0:
                ch = idx + 6
                if ch == RC_BTN_HOME:
                    arm.home(wait=False)
                elif ch == RC_BTN_OPEN:
                    arm.open_claw(wait=False)
                elif ch == RC_BTN_CLOSE:
                    arm.close_claw(wait=False)
                elif ch == RC_BTN_STOP:
                    arm.stop_all()
        prev_buttons = buttons

        time.sleep_ms(RC_LOOP_MS)


def rc_monitor():
    """Print raw transmitter data so you can map sticks to channels."""
    import rc_module

    if rc_module.rc_slave_init() is False:
        print("[rc] receiver init failed")
        return
    print("[rc] L1 L2 L3 R1 R2 R3 K1 K2 K3 K4  (Ctrl-C to quit)")
    while True:
        print(rc_module.rc_slave_data())
        time.sleep_ms(200)


# =============================================================================
# Entry point
# =============================================================================

def main(mode=MODE):
    arm = CyberBrickArm()
    try:
        if mode == "demo":
            demo(arm)
        elif mode == "rc":
            run_rc(arm)
        elif mode == "rc_monitor":
            rc_monitor()
        else:
            print("[arm] ready:", arm.status())
            return arm
    except KeyboardInterrupt:
        print("[arm] interrupted")
    finally:
        if mode in ("demo", "rc", "rc_monitor"):
            arm.shutdown()
    return arm


if __name__ == "__main__":
    main()
