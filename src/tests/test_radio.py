"""
Tests for the radio link, run on the PC with plain Python:

    cd src && .venv/bin/python -m unittest discover -s tests -v

The board files (cyberbrick_arm.py, arm_radio.py, arm_bridge.py, arm_boot.py)
run here against small fakes of the MicroPython modules: a clock, the servo
and motor library, and an ESP-NOW "air" that carries messages between the
bridge (remote) and the server (arm). The PC side (arm_serial.RadioSession)
talks to the bridge through a fake USB link that runs its code locally.
"""

import contextlib
import io
import json
import os
import sys
import tempfile
import time
import traceback
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(HERE)
sys.path.insert(0, SRC)


# =============================================================================
# Fakes of the MicroPython modules
# =============================================================================

class Clock:
    """Fake time.ticks_*: time only moves when the board code waits."""

    def __init__(self):
        self.now = 0
        self.timers = []

    def ticks_ms(self):
        return self.now

    @staticmethod
    def ticks_add(a, b):
        return a + b

    @staticmethod
    def ticks_diff(a, b):
        return a - b

    def advance(self, ms):
        end = self.now + ms
        while self.now < end:
            step = min(10, end - self.now)
            self.now += step
            for timer in list(self.timers):
                timer.callback(timer)

    def sleep_ms(self, ms):
        self.advance(ms)


import gc  # noqa: E402
gc.mem_free = lambda: 51234           # MicroPython only

CLOCK = Clock()
for _name in ("ticks_ms", "ticks_add", "ticks_diff", "sleep_ms"):
    setattr(time, _name, getattr(CLOCK, _name))


class Timer:
    PERIODIC = 1

    def __init__(self, _id):
        self.callback = None

    def init(self, period, mode, callback):
        self.callback = callback
        CLOCK.timers.append(self)

    def deinit(self):
        if self in CLOCK.timers:
            CLOCK.timers.remove(self)


class Pin:
    IN, OUT = 0, 1
    role = 1                      # GPIO10: 1 = transmitter (the remote)

    def __init__(self, n, mode=None):
        self.n = n

    def value(self):
        return Pin.role if self.n == 10 else 0


machine = types.ModuleType("machine")
machine.Timer, machine.Pin = Timer, Pin
machine.reset = lambda: (_ for _ in ()).throw(SystemExit("machine.reset"))
sys.modules["machine"] = machine


class PWM:
    def __init__(self):
        self.ns = 0
        self.history = []          # every pulse written, in ns

    def duty_ns(self, ns):
        self.ns = ns
        self.history.append(ns)


class ServosController:
    def __init__(self):
        self.servos_info_map = [{"c_ang": 90.0, "s_ang": 90.0, "step_en": False,
                                 "step": 1.0} for _ in range(4)]
        self.servos_map = [PWM() for _ in range(4)]
        self.speeds = [0] * 4

    def set_angle(self, i, angle):
        self.servos_info_map[i - 1].update(c_ang=float(angle), s_ang=float(angle),
                                           step_en=False)

    def set_angle_step(self, i, speed):
        self.servos_info_map[i - 1]["step"] = 4.6 * speed / 100

    def set_angle_stepping(self, i, angle):
        self.servos_info_map[i - 1].update(s_ang=float(angle), step_en=True)

    def timing_proc(self):
        for m in self.servos_info_map:
            if m["step_en"]:
                d = m["s_ang"] - m["c_ang"]
                m["c_ang"] = m["s_ang"] if abs(d) <= m["step"] else \
                    m["c_ang"] + (m["step"] if d > 0 else -m["step"])

    def set_speed(self, i, speed):
        self.speeds[i - 1] = speed

    def stop(self, i):
        self.speeds[i - 1] = 0


class MotorsController:
    def __init__(self):
        self.speeds = [0, 0]

    def set_speed(self, i, speed):
        self.speeds[i - 1] = speed

    def stop(self, i):
        self.speeds[i - 1] = 0


for _name, _cls in (("servos", ServosController), ("motors", MotorsController)):
    _m = types.ModuleType(_name)
    setattr(_m, _cls.__name__, _cls)
    sys.modules[_name] = _m


class WLAN:
    def __init__(self, _if):
        self.channel = None

    def active(self, on=None):
        return True

    def disconnect(self):
        pass

    def config(self, *args, **kw):
        if "channel" in kw:
            self.channel = kw["channel"]
        return b"\x00" * 6


network = types.ModuleType("network")
network.WLAN, network.STA_IF = WLAN, 0
sys.modules["network"] = network


class Air:
    """Carries ESP-NOW messages between the fake nodes."""

    def __init__(self):
        self.nodes = {}
        self.next_mac = None
        self.idle = lambda node, timeout: CLOCK.advance(timeout)
        self.drop = None          # drop(sender_mac, msg) -> True to lose it


AIR = Air()


class ESPNow:
    def __init__(self):
        self.mac = AIR.next_mac
        AIR.nodes[self.mac] = self
        self.queue = []
        self.peers = {}
        self.pmk = None
        self.peers_table = {}

    def active(self, on):
        pass

    def set_pmk(self, pmk):
        self.pmk = pmk

    def add_peer(self, mac, lmk=None):
        if mac in self.peers:
            raise OSError("ESP_ERR_ESPNOW_EXIST")
        self.peers[mac] = lmk

    def mod_peer(self, mac, lmk=None):
        self.peers[mac] = lmk

    def send(self, mac, msg, sync=True):
        data = msg.encode() if isinstance(msg, str) else bytes(msg)
        assert len(data) <= 250, f"ESP-NOW message too long ({len(data)} bytes)"
        node = AIR.nodes.get(bytes(mac))
        # Encrypted link: both ends must have the same keys for each other.
        if (node is None or node.pmk != self.pmk
                or node.peers.get(self.mac) != self.peers.get(bytes(mac))):
            return False
        if AIR.drop and AIR.drop(self.mac, data):
            return True           # sent, but lost on the way
        node.queue.append((self.mac, data))
        return True

    def irecv(self, timeout_ms):
        if not self.queue:
            AIR.idle(self, timeout_ms)
        if self.queue:
            mac, data = self.queue.pop(0)
            self.peers_table[mac] = [-42, CLOCK.now]
            return bytearray(mac), bytearray(data)
        return None, None


espnow = types.ModuleType("espnow")
espnow.ESPNow = ESPNow
sys.modules["espnow"] = espnow

import arm_boot          # noqa: E402  (the fakes must exist first)
import arm_bridge        # noqa: E402
import arm_radio         # noqa: E402
import arm_serial        # noqa: E402
import cyberbrick_arm    # noqa: E402

ARM_MAC = b"\xaa\x00\x00\x00\x00\x01"
REMOTE_MAC = b"\xbb\x00\x00\x00\x00\x02"
PAIR = {"arm_mac": "aa:00:00:00:00:01", "remote_mac": "bb:00:00:00:00:02",
        "pmk": "11" * 16, "lmk": "22" * 16, "channel": 6, "arm_mpy": 6}


class FakeLink:
    """USB link to the remote: runs the code the PC sends right here."""

    def __init__(self):
        self.ns = {}

    def exec(self, code, timeout=30, echo=False):
        out = io.StringIO()
        try:
            with contextlib.redirect_stdout(out):
                exec(code, self.ns)
        except Exception:
            raise arm_serial.DeviceError(traceback.format_exc(limit=0).strip())
        return out.getvalue()

    def interrupt(self):
        return True


class RadioTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        tmp = self.tmp.name
        AIR.nodes.clear()
        AIR.drop = None
        CLOCK.timers.clear()
        cyberbrick_arm.CAL_FILE = os.path.join(tmp, "arm_cal.json")
        for key in cyberbrick_arm.CAL_KEYS:          # back to the defaults
            setattr(cyberbrick_arm, key, DEFAULTS[key])
        arm_radio.UPDATES = {
            name: tuple(os.path.join(tmp, os.path.basename(p)) for p in paths)
            for name, paths in ORIGINAL_UPDATES.items()}
        arm_serial.STATE_FILE = os.path.join(tmp, "state.json")

        # The arm (receiver)
        AIR.next_mac = ARM_MAC
        radio = arm_radio.Radio({"peer": PAIR["remote_mac"], "pmk": PAIR["pmk"],
                                 "lmk": PAIR["lmk"], "channel": PAIR["channel"]})
        self.arm = cyberbrick_arm.CyberBrickArm()
        self.server = arm_radio.Server(self.arm, radio, 1234)
        self.calls = []
        real_call = self.server.call
        self.server.call = lambda *a: (self.calls.append(a[0]), real_call(*a))[1]
        AIR.idle = self._run_arm

        # The remote (transmitter) with the bridge, and the PC session
        AIR.next_mac = REMOTE_MAC
        self.link = FakeLink()
        self.link.ns["_b"] = arm_bridge.Bridge(PAIR["arm_mac"], PAIR["pmk"],
                                               PAIR["lmk"], PAIR["channel"])
        self.session = arm_serial.RadioSession(self.link, dict(PAIR))

    def tearDown(self):
        self.arm.shutdown()
        self.tmp.cleanup()

    def _run_arm(self, node, timeout_ms):
        """While the remote waits, let the arm run its main loop."""
        end = CLOCK.now + timeout_ms
        while CLOCK.now < end:
            msg = self.server.radio.recv(0)
            if msg:
                self.server.handle(msg)
            self.server.poll()
            CLOCK.advance(10)
            if node.queue:
                return

    def quiet(self, fn, *args, **kw):
        with contextlib.redirect_stdout(io.StringIO()):
            return fn(*args, **kw)

    # ------------------------------------------------------------- commands

    def test_move_waits_until_the_joints_arrive(self):
        call, timeout = arm_serial.cmd_move(["110", "70", "50"])
        self.quiet(self.session.run_call, call, timeout)
        s = self.session.last_status
        self.assertEqual((s["lower_arm"], s["upper_arm"]), (110, 70))
        self.assertEqual(s["boot"], 1234)

    def test_status_has_the_usb_shape(self):
        status = self.session.status()
        self.assertEqual(set(status), {"lower_arm", "upper_arm", "claw",
                                       "base_angle", "base_speed"})

    def test_turn_base_respects_the_limit(self):
        call, timeout = arm_serial.cmd_turn(["170"])
        self.quiet(self.session.run_call, call, timeout)
        self.assertLessEqual(abs(self.session.last_status["base_angle"]), 90)
        self.assertGreater(self.session.last_status["base_angle"], 80)

    def test_stop_ends_a_free_base_spin(self):
        self.session.call("set_base_limit", False)
        self.session.call("base_speed", 40)
        self.assertTrue(self.arm.base_moving())
        self.session.call("stop_all")
        self.assertFalse(self.arm.base_moving())

    def test_free_spin_stops_by_itself_if_the_link_is_lost(self):
        self.session.call("claw_run", 50, ignore_limits=True)
        self.assertTrue(self.arm.claw_moving())
        self._run_arm(AIR.nodes[REMOTE_MAC], arm_radio.FREE_RUN_MAX_MS + 100)
        self.assertFalse(self.arm.claw_moving())

    def test_methods_outside_the_list_are_refused(self):
        with self.assertRaises(arm_serial.DeviceError):
            self.session.call("shutdown")
        with self.assertRaises(arm_serial.DeviceError):
            self.session.call("__init__")

    def test_raw_code_is_refused_over_the_radio(self):
        with self.assertRaises(arm_serial.UsageError):
            self.session.run_call(*arm_serial.cmd_raw(["print(1)"]))

    def test_move_timeout_stops_and_reports(self):
        with self.assertRaises(arm_serial.DeviceError) as e:
            self.session.call("move_lower_arm", 0, speed=1, timeout=1)
        self.assertIn("timed out", str(e.exception))
        self.assertFalse(self.arm.joints_moving())

    # -------------------------------------------------------------- the link

    def test_lost_reply_is_sent_again_and_not_run_twice(self):
        lost = []

        def drop(sender, data):
            # Lose the arm's first final reply.
            if sender == ARM_MAC and b'"D"' in data and not lost:
                lost.append(data)
                return True
            return False

        AIR.drop = drop
        self.session.call("turn_base_by", 10)
        self.assertEqual(len(lost), 1)
        self.assertEqual(self.calls.count("turn_base_by"), 1)
        self.assertAlmostEqual(self.arm.base_angle(), 10, delta=1)

    def test_lost_request_is_sent_again(self):
        lost = []
        AIR.drop = lambda sender, data: (sender == REMOTE_MAC and not lost
                                         and not lost.append(data))
        self.session.call("open_claw")
        self.assertEqual(len(lost), 1)
        self.assertAlmostEqual(self.arm.claw_position(), 1.0, delta=0.01)

    def test_no_arm_gives_a_radio_error(self):
        del AIR.nodes[ARM_MAC]
        with self.assertRaises(arm_serial.RadioError):
            self.session.ping()

    def test_wrong_key_is_not_heard(self):
        self.server.radio.e.peers[REMOTE_MAC] = b"x" * 16
        with self.assertRaises(arm_serial.RadioError):
            self.session.ping()

    def test_messages_from_other_boards_are_ignored(self):
        AIR.next_mac = b"\xcc" * 6
        stranger = ESPNow()
        stranger.pmk = self.server.radio.e.pmk
        stranger.peers[ARM_MAC] = REMOTE_MAC   # whatever; it is not the peer
        self.server.radio.e.queue.append((b"\xcc" * 6, b'[1, "c", "stop_all", [], {}, 5]'))
        self.assertIsNone(self.server.radio.recv(0))

    def test_garbage_is_ignored(self):
        for junk in (b"[", b"[1]", b'["x"', b"[1, 2, 3"):
            self.server.handle(junk)
        self.assertEqual(self.session.ping()["boot"], 1234)

    # ----------------------------------------------------------- calibration

    def test_save_calibration_applies_and_persists(self):
        self.session.call("set_base_angle", 45)
        saved = self.session.call("save_calibration",
                                  {"BASE_RIGHT_MS": 3000, "BASE_LEFT_MS": 2500})
        self.assertEqual(saved["BASE_RIGHT_MS"], 3000)
        self.assertEqual(cyberbrick_arm.BASE_RIGHT_MS, 3000)
        self.assertAlmostEqual(self.arm.base_angle(), 45, delta=0.1)
        self.assertEqual(self.arm._base_lim, 3000)
        with open(cyberbrick_arm.CAL_FILE) as f:
            self.assertEqual(json.load(f), {"BASE_RIGHT_MS": 3000, "BASE_LEFT_MS": 2500})

    def test_bad_calibration_is_refused_and_bad_file_ignored(self):
        with self.assertRaises(arm_serial.DeviceError):
            self.session.call("save_calibration", {"BASE_DIRECTION": 5})
        with open(cyberbrick_arm.CAL_FILE, "w") as f:
            json.dump({"CLAW_OPEN_MS": -3, "BASE_LEFT_MS": "x", "CLAW_CLOSE_MS": 4000}, f)
        cyberbrick_arm._apply_calibration(cyberbrick_arm.read_calibration())
        self.assertEqual(cyberbrick_arm.CLAW_OPEN_MS, DEFAULTS["CLAW_OPEN_MS"])
        self.assertEqual(cyberbrick_arm.CLAW_CLOSE_MS, 4000)

    def test_calibration_replies_fit_in_one_message(self):
        # get_calibration asks for 4 keys at a time; try the longest four.
        keys = sorted(cyberbrick_arm.CAL_KEYS, key=len)[-4:]
        cal = {k: -1 if k.endswith("DIRECTION") else 60000 for k in keys}
        self.assertLess(len(json.dumps([65535, "D", None, cal])), 250)
        self.assertLess(len(json.dumps([65535, "c", "calibration", [keys], {}, 5])), 250)

    def test_get_calibration_fetches_every_key(self):
        cal = arm_serial.get_calibration(self.session)
        self.assertEqual(set(cal), set(cyberbrick_arm.CAL_KEYS))
        self.assertEqual(cal["LOWER_ARM_HOME"], 90)

    # ----------------------------------------------------- joint calibration

    def test_joint_calibration_applies_limits_now(self):
        self.session.call("save_calibration", {
            "LOWER_ARM_MIN": 40, "LOWER_ARM_MAX": 150, "LOWER_ARM_HOME": 100,
            "LOWER_ARM_DIRECTION": -1})
        self.session.call("move_lower_arm", 0, speed=100)
        self.assertEqual(self.arm.joint_angle(1), 40)
        self.session.call("move_lower_arm", 180, speed=100)
        self.assertEqual(self.arm.joint_angle(1), 150)
        self.session.call("home", speed=100)
        self.assertEqual(self.arm.joint_angle(1), 100)
        self.assertEqual(cyberbrick_arm.LOWER_ARM_DIRECTION, -1)

    def test_joint_limits_can_be_lifted_for_calibration(self):
        self.session.call("save_calibration", {"UPPER_ARM_MIN": 30, "UPPER_ARM_MAX": 120,
                                               "UPPER_ARM_HOME": 90})
        self.session.call("set_joint_limits", False)
        self.session.call("move_upper_arm", 170, speed=100)
        self.assertEqual(self.arm.joint_angle(2), 170)
        self.session.call("set_joint_limits", True)
        self.session.call("move_upper_arm", 170, speed=100)
        self.assertEqual(self.arm.joint_angle(2), 120)

    def test_joint_calibration_must_fit_together(self):
        for bad in ({"LOWER_ARM_MIN": 100, "LOWER_ARM_HOME": 90},     # home below min
                    {"UPPER_ARM_MIN": 85, "UPPER_ARM_MAX": 92},       # under 10 apart
                    {"LOWER_ARM_MAX": 200},                           # past the servo
                    {"UPPER_ARM_DIRECTION": 0}):
            with self.assertRaises(arm_serial.DeviceError, msg=bad):
                self.session.call("save_calibration", bad)
        self.assertFalse(os.path.exists(cyberbrick_arm.CAL_FILE))

    def test_bad_joint_values_in_the_file_are_ignored(self):
        with open(cyberbrick_arm.CAL_FILE, "w") as f:
            json.dump({"LOWER_ARM_MIN": 120, "LOWER_ARM_MAX": 60, "LOWER_ARM_HOME": 90,
                       "UPPER_ARM_MIN": 20, "UPPER_ARM_MAX": 160, "UPPER_ARM_HOME": 70}, f)
        cyberbrick_arm._apply_calibration(cyberbrick_arm.read_calibration())
        self.assertEqual((cyberbrick_arm.LOWER_ARM_MIN, cyberbrick_arm.LOWER_ARM_MAX), (0, 180))
        self.assertEqual((cyberbrick_arm.UPPER_ARM_MIN, cyberbrick_arm.UPPER_ARM_HOME), (20, 70))

    def _run_arm_calibrate(self, answers, step=None):
        step = step or arm_serial.calibrate_arm
        answers = iter(answers)
        builtins = __builtins__ if isinstance(__builtins__, dict) else vars(__builtins__)
        real_input = builtins["input"]
        builtins["input"] = lambda prompt="": next(answers)
        try:
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                step(self.session)
        finally:
            builtins["input"] = real_input
        return out.getvalue()

    def test_arm_calibrate_walkthrough(self):
        # Like the real arm: lower arm folded at servo 0 and extended at 90,
        # upper arm folded at servo 100 and extended at 180.
        out = self._run_arm_calibrate([
            # lower arm: home (servo 0), extended (+++ ... = 90), test move, answer
            "0", "", "+++", "+++", "+++", "+++", "+++", "+++", "", "", "f",
            # upper arm: home 100 (from 90: ++), extended 180, test move, answer
            "++", "", "180", "", "", "d",
        ])
        cal = arm_serial.get_calibration(self.session, arm_serial.ARM_KEYS)
        self.assertEqual(cal, {
            "LOWER_ARM_MIN": 0, "LOWER_ARM_MAX": 90, "LOWER_ARM_HOME": 0,
            "LOWER_ARM_DIRECTION": 1,
            "UPPER_ARM_MIN": 100, "UPPER_ARM_MAX": 180, "UPPER_ARM_HOME": 100,
            "UPPER_ARM_DIRECTION": -1})
        # Back at home: position 0 on both joints, servos at 0 and 100.
        status = self.session.status()
        self.assertEqual((status["lower_arm"], status["upper_arm"]), (0, 0))
        self.assertEqual((self.arm.servo_angle(1), self.arm.servo_angle(2)), (0, 100))
        self.assertIn("0 = home (servo 100 deg), 180 = fully extended (servo 180 deg)", out)
        self.assertIn("extending lowers the claw", out)

    def test_arm_calibrate_refuses_tiny_range(self):
        self._run_arm_calibrate(["", "+", "", "", "+", ""])
        cal = arm_serial.get_calibration(self.session, arm_serial.ARM_KEYS)
        self.assertEqual(cal["LOWER_ARM_HOME"], 90)            # nothing saved
        self.assertFalse(os.path.exists(cyberbrick_arm.CAL_FILE))

    def test_arm_align_moves_home_to_the_end_of_the_servo(self):
        # Lower arm pressed onto its servo mid-way: folded at 100, so only 80
        # degrees were left for extending.
        self.session.call("save_calibration", {
            "LOWER_ARM_MIN": 100, "LOWER_ARM_MAX": 180, "LOWER_ARM_HOME": 100,
            "LOWER_ARM_DIRECTION": 1})
        out = self._run_arm_calibrate([
            "l", "",        # lower arm; extending goes higher (the default)
            "", "",         # part off -> servo to 0; part back on
            "y",            # run arm-calibrate now
            "", "170", "", "", "f",     # lower: home 0, extended 170
            "", "",                     # upper: skipped
        ], step=arm_serial.align_joint)
        self.assertIn("Moving the bare servo to 0 deg", out)
        cal = arm_serial.get_calibration(self.session, arm_serial.ARM_KEYS)
        self.assertEqual((cal["LOWER_ARM_MIN"], cal["LOWER_ARM_MAX"], cal["LOWER_ARM_HOME"],
                          cal["LOWER_ARM_DIRECTION"]), (0, 170, 0, 1))
        self.assertEqual(cal["UPPER_ARM_HOME"], 90)           # untouched
        self.assertEqual((self.arm.servo_angle(1), self.session.status()["lower_arm"]), (0, 0))

    def test_arm_align_when_extending_lowers_the_angle(self):
        self._run_arm_calibrate(["u", "l", "", "", "n"], step=arm_serial.align_joint)
        cal = arm_serial.get_calibration(self.session, arm_serial.ARM_KEYS)
        self.assertEqual((cal["UPPER_ARM_MIN"], cal["UPPER_ARM_MAX"], cal["UPPER_ARM_HOME"]),
                         (0, 180, 180))
        self.assertEqual(self.arm.servo_angle(2), 180)

    def test_wave_is_one_even_gesture_and_ends_where_it_started(self):
        self.session.call("move_arm", 10, 20, speed=100)
        self.calls.clear()
        self.quiet(arm_serial.gesture, self.session, ["wave", "2"])
        moves = [c for c in self.calls if c == "move_arm"]
        self.assertEqual(len(moves), 1 + 2 * 2 + 1)     # raise, swings, return
        status = self.session.status()
        self.assertEqual((status["lower_arm"], status["upper_arm"]), (10, 20))

    def test_shake_turns_the_base_and_comes_back(self):
        self.quiet(arm_serial.gesture, self.session, ["shake", "1"])
        self.assertEqual(self.calls.count("turn_base"), 3)
        self.assertEqual(self.session.status()["base_angle"], 0)

    def test_gesture_checks_its_arguments(self):
        for args in ([], ["dance"], ["wave", "0"], ["wave", "2", "200"]):
            with self.assertRaises(arm_serial.UsageError):
                arm_serial.gesture(self.session, args)

    def test_joint_moves_ease_in_and_out_in_fine_steps(self):
        pwm = self.arm.servos.servos_map[1]
        self.session.call("move_upper_arm", 30, speed=100)
        pwm.history.clear()
        start = CLOCK.now
        self.session.call("move_upper_arm", 150, speed=25)
        # Same average speed as before: 120 servo deg at 25 * 4.61 deg/s.
        self.assertAlmostEqual(CLOCK.now - start, 120 * 1000 / (25 * 4.61), delta=60)
        steps = [b - a for a, b in zip(pwm.history, pwm.history[1:])]
        self.assertTrue(all(d >= 0 for d in steps))                 # one way only
        # Slow near the ends, fast in the middle, never a 1.8 degree jump at the ends.
        self.assertLess(steps[0], 0.5 * max(steps))
        self.assertLess(steps[-2], 0.5 * max(steps))
        self.assertLess(steps[0], cyberbrick_arm.SERVO_NS_PER_DEG * 1.8)
        self.assertEqual(pwm.ns, int(cyberbrick_arm.SERVO_NS0 + 150 * cyberbrick_arm.SERVO_NS_PER_DEG))
        self.assertEqual(self.arm.servo_angle(2), 150)

    # ------------------------------------------------------- joint positions

    def test_positions_run_from_home_to_extended(self):
        self.session.call("save_calibration", {
            "UPPER_ARM_MIN": 100, "UPPER_ARM_MAX": 180, "UPPER_ARM_HOME": 100})
        for position, servo in ((0, 100), (180, 180), (90, 140), (100, 144)):
            self.session.call("move_upper_arm", position, speed=100)
            self.assertEqual(self.arm.servo_angle(2), servo)
            # The reported position is the one asked for, not the rounded one.
            self.assertEqual(self.session.status()["upper_arm"], position)
        self.session.call("move_upper_arm", 500, speed=100)     # past the end
        self.assertEqual((self.arm.servo_angle(2), self.arm.joint_angle(2)), (180, 180))
        self.session.call("move_upper_arm", 130, speed=100)
        self.session.call("home", speed=100)
        self.assertEqual((self.arm.servo_angle(2), self.arm.joint_angle(2)), (100, 0))

    def test_positions_when_home_is_the_high_end(self):
        self.session.call("save_calibration", {
            "LOWER_ARM_MIN": 20, "LOWER_ARM_MAX": 160, "LOWER_ARM_HOME": 160})
        self.session.call("move_lower_arm", 180, speed=100)
        self.assertEqual(self.arm.servo_angle(1), 20)
        self.session.call("move_lower_arm", 90, speed=100)
        self.assertEqual(self.arm.servo_angle(1), 90)
        self.assertEqual(self.session.status()["lower_arm"], 90)

    def test_moving_report_is_a_position(self):
        self.session.call("save_calibration", {
            "UPPER_ARM_MIN": 100, "UPPER_ARM_MAX": 180, "UPPER_ARM_HOME": 100})
        self.session.call("move_upper_arm", 180, speed=100)
        self.session.call("stop_all")                 # hold: no target to report
        self.assertEqual(self.arm.joint_angle(2), 180)
        self.arm.servos.servos_info_map[1]["c_ang"] = 120  # e.g. half way there
        self.assertAlmostEqual(self.arm.joint_angle(2), 45)

    # --------------------------------------------------------------- updates

    def test_update_sends_files_and_keeps_the_old_copy(self):
        path, ver, old = arm_radio.UPDATES["cyberbrick_arm"]
        with open(path, "wb") as f:
            f.write(b"old code")
        reboots = []
        self.session.reboot = reboots.append
        self.quiet(self.session.update)
        self.assertEqual(reboots, ["radio"])
        with open(path, "rb") as f:
            new = f.read()
        self.assertEqual(new, arm_serial.compile_library(6))
        with open(old, "rb") as f:
            self.assertEqual(f.read(), b"old code")
        with open(ver) as f:
            self.assertEqual(f.read(), arm_serial.source_version())

    def test_damaged_update_changes_nothing(self):
        path = arm_radio.UPDATES["arm_radio"][0]
        self.session.request(["fo", "arm_radio"])
        self.session.request(["fw", "aGVsbG8="])
        with self.assertRaises(arm_serial.DeviceError):
            self.session.request(["fc", "0" * 64, "v1"])
        self.assertFalse(os.path.exists(path))

    def test_update_cannot_write_other_files(self):
        with self.assertRaises(arm_serial.DeviceError):
            self.session.request(["fo", "boot"])

    def test_restore_after_reboot(self):
        arm_serial._write_state({"base_angle": -30, "claw": 0.5}, boot=999)
        self.quiet(self.session._restore_state, self.session.ping())
        self.assertAlmostEqual(self.arm.base_angle(), -30, delta=0.1)
        self.assertAlmostEqual(self.arm.claw_position(), 0.5, delta=0.01)
        self.assertEqual(arm_serial._read_state()["boot"], 1234)


DEFAULTS = {k: getattr(cyberbrick_arm, k) for k in cyberbrick_arm.CAL_KEYS}
ORIGINAL_UPDATES = dict(arm_radio.UPDATES)


class UsbCodeTest(unittest.TestCase):
    """The USB path still sends the same MicroPython as before."""

    def test_code(self):
        cases = {
            ("home", ()): "arm.home()",
            ("home", ("30",)): "arm.home(speed=30.0)",
            ("lower", ("120",)): "arm.move_lower_arm(120.0)",
            ("upper", ("60", "10")): "arm.move_upper_arm(60.0, speed=10.0)",
            ("move", ("110", "70")): "arm.move_arm(110.0, 70.0)",
            ("claw", ("open",)): "arm.open_claw()",
            ("claw", ("0.5",)): "arm.set_claw(0.5)",
            ("claw-spin", ("40",)): "arm.claw_run(40.0, ignore_limits=True)",
            ("claw-cal", ("open",)): "arm.calibrate_claw(True)",
            ("turn", ("-45",)): "arm.turn_base(-45.0)",
            ("base", ("40", "1.5")): "arm.rotate_base(40.0, 1500)",
            ("base-set", ()): "arm.set_base_angle(0.0)",
            ("stop", ()): "arm.stop_all()",
        }
        for (name, args), code in cases.items():
            call, _ = arm_serial.COMMANDS[name][0](list(args))
            self.assertEqual(arm_serial.usb_code(call), code + arm_serial.STATUS, name)
        self.assertEqual(arm_serial.usb_code(arm_serial.cmd_status([])[0]),
                         "print(arm.status())")
        self.assertEqual(arm_serial.usb_code(arm_serial.cmd_demo([])[0]), "demo(arm)")


class BootHookTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        arm_boot.COUNT_FILE = os.path.join(self.tmp.name, "arm_boot")
        self.tried = []

    def tearDown(self):
        self.tmp.cleanup()
        for name in ("arm_radio_fake", "arm_radio_fake_old"):
            sys.modules.pop(name, None)

    def _modules(self, fails):
        for name in ("arm_radio_fake", "arm_radio_fake_old"):
            m = types.ModuleType(name)

            def run(lib, on_ready, name=name):
                self.tried.append((name, lib))
                if (name, lib) in fails:
                    raise ImportError("broken " + lib)
                on_ready()
                raise KeyboardInterrupt    # ends the "radio mode" right away
            m.run = run
            sys.modules[name] = m

    def test_falls_back_through_the_old_copies(self):
        arm_boot.TRIES = (("arm_radio_fake", "lib"), ("arm_radio_fake", "lib_old"),
                          ("arm_radio_fake_old", "lib_old"))
        self._modules({("arm_radio_fake", "lib"), ("arm_radio_fake", "lib_old")})
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(arm_boot._receiver())
        self.assertEqual(self.tried[-1], ("arm_radio_fake_old", "lib_old"))
        self.assertFalse(os.path.exists(arm_boot.COUNT_FILE))   # started fine

    def test_crash_counter_skips_what_failed_before(self):
        arm_boot.TRIES = (("arm_radio_fake", "lib"), ("arm_radio_fake_old", "lib"))
        self._modules(set())
        with open(arm_boot.COUNT_FILE, "w") as f:
            f.write("1")                  # the first try crashed the board
        with contextlib.redirect_stdout(io.StringIO()):
            arm_boot._receiver()
        self.assertEqual(self.tried, [("arm_radio_fake_old", "lib")])

    def test_everything_broken_means_stock(self):
        arm_boot.TRIES = (("arm_radio_fake", "lib"),)
        self._modules({("arm_radio_fake", "lib")})
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(arm_boot._receiver())
        self.assertFalse(os.path.exists(arm_boot.COUNT_FILE))   # try again next boot


class CompileTest(unittest.TestCase):
    """Every board file compiles for MicroPython 1.23 and stays small."""

    BUDGET = {"cyberbrick_arm": 9000, "arm_radio": 6000, "arm_bridge": 2000,
              "arm_boot": 2000}

    def test_sizes(self):
        sizes = {"cyberbrick_arm": len(arm_serial.compile_library(6))}
        for path in (arm_serial.RADIO_LIB, arm_serial.BRIDGE_LIB, arm_serial.BOOT_LIB):
            name = os.path.splitext(os.path.basename(path))[0]
            sizes[name] = len(arm_serial.compile_file(path, 6))
        for name, size in sizes.items():
            self.assertLess(size, self.BUDGET[name], name)


if __name__ == "__main__":
    unittest.main()
