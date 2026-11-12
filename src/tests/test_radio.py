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
machine.PWRON_RESET, machine.HARD_RESET, machine.WDT_RESET = 1, 2, 3
machine.SOFT_RESET = 5
machine.reset_cause = lambda: machine.HARD_RESET
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
        self.proc_calls = 0

    def set_angle(self, i, angle):
        self.servos_info_map[i - 1].update(c_ang=float(angle), s_ang=float(angle),
                                           step_en=False)

    def set_angle_step(self, i, speed):
        self.servos_info_map[i - 1]["step"] = 4.6 * speed / 100

    def set_angle_stepping(self, i, angle):
        self.servos_info_map[i - 1].update(s_ang=float(angle), step_en=True)

    def timing_proc(self):
        self.proc_calls += 1
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
    # One interface per kind, as on the board: WLAN(STA_IF) twice is the
    # same interface. (Both boards share these fakes, on the same channel.)
    channels = {}

    def __init__(self, _if):
        self._if = _if

    @property
    def channel(self):
        return WLAN.channels.get(self._if)

    @channel.setter
    def channel(self, value):
        WLAN.channels[self._if] = value

    def active(self, on=None):
        return True

    def disconnect(self):
        pass

    def config(self, *args, **kw):
        if "channel" in kw:
            self.channel = kw["channel"]
        if args == ("channel",):
            return self.channel
        if args == ("pm",):
            return 1
        return b"\x00" * 6


network = types.ModuleType("network")
network.WLAN, network.STA_IF, network.AP_IF = WLAN, 0, 1
sys.modules["network"] = network


class Air:
    """Carries ESP-NOW messages between the fake nodes."""

    def __init__(self):
        self.nodes = {}
        self.next_mac = None
        self.idle = lambda node, timeout: CLOCK.advance(timeout)
        self.drop = None          # drop(sender_mac, msg) -> True to lose it
        self.deaf = set()         # MACs whose radio neither sends nor hears


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
        if self.mac in AIR.deaf or bytes(mac) in AIR.deaf:
            return False          # no ack, nothing delivered
        node = AIR.nodes.get(bytes(mac))
        # Encrypted link: both ends must have the same keys for each other.
        if (node is None or node.pmk != self.pmk
                or node.peers.get(self.mac) != self.peers.get(bytes(mac))):
            return False
        if AIR.drop and AIR.drop(self.mac, data):
            return True           # sent, but lost on the way
        node.queue.append((self.mac, data))
        return True

    def stats(self):
        return (7, 7, 0, 3, 0)

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
        AIR.deaf.clear()
        CLOCK.timers.clear()
        cyberbrick_arm.CAL_FILE = os.path.join(tmp, "arm_cal.json")
        for key in cyberbrick_arm.CAL_KEYS:          # back to the defaults
            setattr(cyberbrick_arm, key, DEFAULTS[key])
        arm_radio.UPDATES = {
            name: tuple(os.path.join(tmp, os.path.basename(p)) for p in paths)
            for name, paths in ORIGINAL_UPDATES.items()}
        arm_serial.STATE_FILE = os.path.join(tmp, "state.json")
        arm_radio.DIAG_FILE = os.path.join(tmp, "arm_diag.json")
        arm_radio.HOLD_FILE = os.path.join(tmp, "arm_hold.json")

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
        self.remote_restarts = 0

        def restart_remote():
            self.remote_restarts += 1
            for hook in self.on_remote_restart:
                hook()
        self.on_remote_restart = []
        self.session._reboot_remote = restart_remote
        self.session._start_bridge = lambda: None

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
            self.server.recover()
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

    def _radio_outage(self, ms):
        """Lose every message for the first `ms` of the next request."""
        end = CLOCK.now + ms
        AIR.drop = lambda sender, data: CLOCK.now < end

    def test_a_burst_of_lost_messages_is_ridden_out(self):
        # Interference can wipe out a couple of seconds of the channel.
        self._radio_outage(3500)
        self.assertEqual(self.session.ping()["boot"], 1234)

    def test_a_long_outage_gives_a_radio_error_after_about_5_s(self):
        # The arm's radio acknowledged the messages (they were lost after
        # that), so it is not the remote's fault: no restart, no resend.
        self._radio_outage(60000)
        start = CLOCK.now
        with self.assertRaises(arm_serial.RadioError) as cm:
            self.session.ping()
        self.assertAlmostEqual(CLOCK.now - start, arm_bridge.NO_REPLY_MS, delta=400)
        self.assertIn("no reply from the arm", str(cm.exception))
        self.assertEqual(self.remote_restarts, 0)

    def test_a_jammed_remote_radio_is_restarted_and_the_request_resent(self):
        bridge = self.link.ns["_b"]
        real_send = bridge.e.send

        def jammed(*args):
            raise OSError(116)              # ETIMEDOUT, as seen on the remote
        bridge.e.send = jammed
        self.on_remote_restart.append(lambda: setattr(bridge.e, "send", real_send))
        self.quiet(self.session.call, "turn_base_by", 10)
        self.assertEqual(self.remote_restarts, 1)
        self.assertEqual(self.calls.count("turn_base_by"), 1)   # run once only
        self.assertAlmostEqual(self.arm.base_angle(), 10, delta=1)

    def test_status_always_fits_in_one_message(self):
        self.server.error = "OSError: " + "x" * 300
        self.arm.timer_error = ValueError("y" * 300)
        self.server.rst = 3
        self.server.code = "radio_old+arm_old"
        self.arm._base_pos = -10 ** 8            # silly numbers, long digits
        self.assertEqual(self.session.ping()["rst"], 3)
        status = self.session.last_status
        self.assertEqual(status["code"], "radio_old+arm_old")   # kept over errors
        self.assertNotIn("rst", arm_serial._public(status))
        self.assertNotIn("code", arm_serial._public(status))
        # With room to spare, a (shortened) error is reported too.
        self.arm.timer_error = None
        self.arm._base_pos = 0
        self.assertEqual(self.session.ping()["radio_error"], ("OSError: " + "x" * 31))

    def test_debug_reads_both_boards(self):
        self.session.ping()
        self._snapshot_due()                       # a snapshot from "before"
        self.server.prev = arm_radio._read(arm_radio.DIAG_FILE)
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            got = self.session.debug()
        self.assertIsNone(got["error"])
        self.assertEqual(got["remote"]["st"], [7, 7, 0, 3, 0])
        self.assertEqual(got["remote"]["ch"], PAIR["channel"])
        self.assertEqual(got["arm"]["boot"], 1234)
        self.assertEqual(got["prev"]["boot"], 1234)
        text = out.getvalue()
        self.assertIn("Remote: ESP-NOW sent 7, send results 7, failed results 0", text)
        self.assertIn("Arm, this boot:", text)
        self.assertIn("last snapshot from before this boot", text)
        self.assertIn("channel 6, power save 1, AP on", text)

    def test_debug_on_a_dead_link_keeps_the_remote_running(self):
        AIR.deaf.add(ARM_MAC)
        got = self.quiet(self.session.debug)
        self.assertEqual(self.remote_restarts, 0)   # its counters survive
        self.assertIn("no radio contact", got["error"])
        self.assertEqual(got["remote"]["st"], [7, 7, 0, 3, 0])
        self.assertIsNone(got["arm"])

    def test_debug_text_copes_with_missing_fields(self):
        d = {"up": 3723000, "boot": 9, "loop": 5, "last": None, "rx": 0,
             "sf": 1, "rc": [2, 1, 1, 0], "ev": [[60, 1, 3]], "rb": 1}
        lines = arm_serial.describe_diag(d, snapshot=True)
        self.assertIn("saved after 1h 2m 3s", lines[0])
        self.assertIn("recovery reboot", lines[0])
        self.assertIn("none yet", lines[1])
        self.assertIn("radio restart fixed 1 of 2, Wi-Fi restart fixed 0 of 1",
                      lines[2])
        self.assertIn("incident at 1m 0s while moving: reboot", lines[3])

    def test_diagnostics_count_the_radio_traffic_and_fit_in_one_message(self):
        for _ in range(3):
            self.session.ping()
        self.server.radio.e.queue.append((b"\xcc" * 6, b"[1]"))   # a stranger
        self.server.radio.e.queue.append((REMOTE_MAC, b"x"))       # not a request
        self.server.radio.recv(0)
        self.server.radio.recv(0)
        self.server.error = "OSError: " + "x" * 300
        d = self.session.request(["d"], 5)[1]
        self.assertEqual(d["rx"], 4)              # 3 pings and this request
        self.assertEqual(d["ign"], 2)
        self.assertEqual(d["sf"], 0)
        self.assertNotIn("sr", d)                 # same as st[2]
        self.assertEqual(d["ch"], PAIR["channel"])
        self.assertEqual(d["pm"], 1)
        self.assertEqual(d["ap"], 1)
        self.assertEqual(d["st"], [7, 7, 0, 3, 0])
        self.assertNotIn("err", d)                # it is in the status instead
        self.assertEqual(d["boot"], 1234)
        self.assertEqual(d["last"], 0)            # this request, just now
        self.assertNotIn("d", self.calls)         # not an arm method call
        # Even with huge counters the reply fits in one ESP-NOW message, with
        # the spare fields left out first.
        radio = self.server.radio
        radio.rx = radio.ign = radio.tx_retry = radio.tx_fail = 10 ** 9
        radio.e.stats = lambda: (10 ** 9,) * 5
        self.server.loops = 10 ** 9
        CLOCK.now = 2 ** 30 - 1
        d = self.session.request(["d"], 5)[1]    # the fake air checks the size
        self.assertEqual(d["up"], 2 ** 30 - 1)
        for key in ("boot", "loop", "last", "rx", "st", "ch", "rc"):
            self.assertIn(key, d)

    def test_a_busy_day_of_diagnostics_fits_with_every_field(self):
        server, radio = self.server, self.server.radio
        CLOCK.now = 6 * 3600 * 1000               # 6 hours up
        server.loops, radio.rx, radio.ign = 2000000, 5000, 12
        radio.tx_retry, radio.tx_fail, radio.mv = 900, 40, [2500, 700]
        radio.last_rx = CLOCK.now - 123456
        radio.e.stats = lambda: (6000, 6000, 900, 5012, 3)
        server.rc, server.rec_reboot = [40, 31, 9, 6], True
        server.ev = [[21000, 1, 1], [21500, 0, 2], [21599, 1, 3]]
        d = server.diag()
        self.assertLessEqual(len(arm_radio._dumps([65535, "D", None, d])), 250)
        for key in arm_radio.DIAG_SPARE + ("rb",):
            self.assertIn(key, d)

    def _snapshot_due(self):
        CLOCK.advance(arm_radio.DIAG_EVERY_MS)
        self.server.save_diag()

    def test_snapshot_is_saved_while_idle_and_read_after_a_reboot(self):
        self.session.ping()
        self.server.save_diag()                   # not due yet
        self.assertFalse(os.path.exists(arm_radio.DIAG_FILE))
        self._snapshot_due()
        with open(arm_radio.DIAG_FILE) as f:
            saved = json.load(f)
        self.assertEqual((saved["boot"], saved["rx"]), (1234, 1))
        self.assertLess(len(json.dumps([65535, "D", None, saved])), 251)
        # The file is overwritten in place, never added to.
        self.session.ping()
        self._snapshot_due()
        with open(arm_radio.DIAG_FILE) as f:
            self.assertEqual(json.load(f)["rx"], 2)
        self.assertEqual(os.listdir(os.path.dirname(arm_radio.DIAG_FILE)).count(
            "arm_diag.json"), 1)
        # Before the snapshot of this boot is overwritten, "d prev" has the
        # one from the boot before; on the first boot there is none.
        self.assertIsNone(self.session.request(["d", "prev"], 5)[1])
        self.server = arm_radio.Server(self.arm, self.server.radio, 999)
        prev = self.session.request(["d", "prev"], 5)[1]
        self.assertEqual((prev["boot"], prev["rx"]), (1234, 2))

    def test_snapshot_waits_until_nothing_moves(self):
        CLOCK.advance(arm_radio.DIAG_EVERY_MS)    # due...
        self.arm.move_arm(150, 150, speed=1, wait=False)
        self.assertTrue(self.arm.joints_moving())
        self.server.save_diag()                   # ...but a joint is moving
        self.assertFalse(os.path.exists(arm_radio.DIAG_FILE))
        self.arm.hold_joints()
        self.server.save_diag()                   # due, and now idle
        self.assertTrue(os.path.exists(arm_radio.DIAG_FILE))

    def test_snapshot_is_skipped_when_flash_is_short(self):
        from unittest import mock
        short = (4096, 4096, 100, 10, 10)         # 40 KB free
        with mock.patch.object(arm_radio.os, "statvfs", lambda _p: short):
            self._snapshot_due()
        self.assertFalse(os.path.exists(arm_radio.DIAG_FILE))
        self._snapshot_due()                      # plenty free again
        self.assertTrue(os.path.exists(arm_radio.DIAG_FILE))

    def test_failed_reply_sends_are_counted(self):
        radio = self.server.radio
        radio.e.send = lambda *a: False
        self.assertFalse(radio.send([1, "D", None]))
        self.assertEqual((radio.tx_retry, radio.tx_fail), (3, 1))

    # ------------------------------------------------------- radio recovery

    def _arm_goes_deaf_after_next_request(self):
        """What the arm showed: it takes in one request, then its radio
        neither sends nor hears until something is restarted."""
        e = self.server.radio.e
        real_irecv = e.irecv

        def irecv(timeout_ms):
            mac, data = real_irecv(timeout_ms)
            if data:
                AIR.deaf.add(ARM_MAC)
                e.irecv = real_irecv
            return mac, data
        e.irecv = irecv

    def _cured_by(self, name):
        radio = self.server.radio
        real = getattr(radio, name)

        def restart():
            real()                        # the real code, against the fakes
            AIR.deaf.discard(ARM_MAC)
        setattr(radio, name, restart)

    def test_recovery_step_1_restarts_esp_now_and_the_command_still_works(self):
        self._cured_by("restart_espnow")
        self._arm_goes_deaf_after_next_request()
        status = self.session.ping()              # no error for the PC
        self.assertEqual(status["boot"], 1234)
        self.assertEqual(self.server.rc, [1, 1, 0, 0])
        self.assertEqual(self.server.radio.tx_fail, 1)
        self.assertIsNone(self.server.reboot)
        self.assertIsNone(self.server.rec)
        d = self.session.request(["d"], 5)[1]
        self.assertEqual(d["rc"], [1, 1, 0, 0])
        self.assertEqual([e[1:] for e in d["ev"]], [[0, 1]])   # idle, step 1
        self.assertFalse(os.path.exists(arm_radio.HOLD_FILE))

    def test_recovery_step_2_restarts_wifi_when_esp_now_was_not_enough(self):
        self._cured_by("restart_wifi")
        self._arm_goes_deaf_after_next_request()
        self.session.ping()                       # still answered in time
        self.assertEqual(self.server.rc, [1, 0, 1, 1])
        self.assertIsNone(self.server.reboot)

    def test_recovery_reboots_into_radio_mode_and_says_so(self):
        self._arm_goes_deaf_after_next_request()  # nothing cures it
        with self.assertRaises(arm_serial.RadioError) as cm:
            self.quiet(self.session.ping)
        self.assertIn("no reply from the arm", str(cm.exception))
        self.assertEqual(self.server.reboot, "radio")
        self.assertEqual(self.server.rc, [1, 0, 1, 0])
        with open(arm_radio.DIAG_FILE) as f:
            saved = json.load(f)
        self.assertEqual((saved["rb"], saved["rc"]), (1, [1, 0, 1, 0]))
        self.assertEqual([e[1:] for e in saved["ev"]], [[0, 3]])
        # The joints are held and their angles kept for after the reboot.
        with open(arm_radio.HOLD_FILE) as f:
            held = json.load(f)
        self.assertEqual(held, {"1": self.arm.servo_angle(1),
                                "2": self.arm.servo_angle(2), "b": 1234})
        # Radio mode starting again after that reset puts them back.
        fresh = cyberbrick_arm.CyberBrickArm()
        try:
            arm_radio.restore_hold(fresh, machine.HARD_RESET)
            self.assertEqual((fresh.servo_angle(1), fresh.servo_angle(2)),
                             (held["1"], held["2"]))
        finally:
            fresh.shutdown()

    def _hold(self, boot=1234, snapshot_boot=1234, rb=1):
        with open(arm_radio.HOLD_FILE, "w") as f:
            json.dump({"1": 150, "2": 30, "b": boot}, f)
        with open(arm_radio.DIAG_FILE, "w") as f:
            json.dump({"boot": snapshot_boot, "rb": rb} if rb else
                      {"boot": snapshot_boot}, f)

    def test_held_joints_are_put_back_only_after_our_own_reset(self):
        self._hold()
        fresh = cyberbrick_arm.CyberBrickArm()          # starts at home
        try:
            self.assertEqual(fresh.servo_angle(1), 90)
            arm_radio.restore_hold(fresh, machine.HARD_RESET)
            self.assertEqual((fresh.servo_angle(1), fresh.servo_angle(2)), (150, 30))
            self.assertFalse(os.path.exists(arm_radio.HOLD_FILE))   # used up
        finally:
            fresh.shutdown()
        # After switching on, or with a hold file that does not belong to the
        # last recovery reboot, the joints go home and the file is discarded.
        for rst, hold in ((machine.PWRON_RESET, {}),
                          (machine.HARD_RESET, {"snapshot_boot": 999}),
                          (machine.HARD_RESET, {"rb": 0})):
            self._hold(**hold)
            fresh = cyberbrick_arm.CyberBrickArm()
            try:
                arm_radio.restore_hold(fresh, rst)
                self.assertEqual(fresh.servo_angle(1), 90, (rst, hold))
                self.assertFalse(os.path.exists(arm_radio.HOLD_FILE))
            finally:
                fresh.shutdown()
        arm_radio.restore_hold(self.arm, machine.HARD_RESET)   # no file: no-op

    def test_failed_sends_are_counted_as_moving_or_idle(self):
        radio = self.server.radio
        radio.e.send = lambda *a: False
        radio.send([1, "D", None])                       # idle
        self.assertEqual((radio.mv, radio.fail_moving), ([0, 0], 0))
        self.arm.move_arm(150, 150, speed=1, wait=False)
        radio.send([2, "A"])                             # while moving
        self.assertEqual((radio.mv, radio.fail_moving), ([3, 3], 1))
        self.assertEqual(radio.tx_retry, 6)

    def test_recovery_during_a_move_rides_on_the_remote_check_ins(self):
        self._cured_by("restart_espnow")
        e = self.server.radio.e
        real_send = e.send

        def send(mac, msg, sync=True):            # deaf from the "done" reply
            if '"D"' in msg:
                AIR.deaf.add(ARM_MAC)
                e.send = real_send
            return real_send(mac, msg, sync)
        e.send = send
        self.session.call("move_arm", 150, 150, speed=50)
        self.assertEqual(self.server.rc, [1, 1, 0, 0])
        self.assertGreater(self.server.radio.mv[0], 0)  # "A" went out mid-move
        self.assertIsNone(self.server.reboot)

    def test_ordinary_traffic_never_starts_recovery(self):
        for _ in range(20):
            self.session.ping()
        self.assertEqual(self.server.rc, [0, 0, 0, 0])

    def test_backup_copies_are_reported(self):
        server = arm_radio.Server(self.arm, self.server.radio, 1, "cyberbrick_arm_old", 1)
        self.assertEqual(server.code, "radio+arm_old")
        self.assertIsNone(self.server.code)
        self.server.code = server.code
        self.session._check_remote = self.session._ensure_bridge = lambda: None
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.session.load()
        self.assertIn("running backup copies", out.getvalue())

    def test_no_arm_gives_a_radio_error(self):
        del AIR.nodes[ARM_MAC]
        with self.assertRaises(arm_serial.RadioError) as cm:
            self.quiet(self.session.ping)
        self.assertIn("even after restarting the remote", str(cm.exception))
        self.assertEqual(self.remote_restarts, 1)              # tried once

    def test_wrong_key_is_not_heard(self):
        self.server.radio.e.peers[REMOTE_MAC] = b"x" * 16
        with self.assertRaises(arm_serial.RadioError):
            self.quiet(self.session.ping)

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

    # ------------------------------------------------------------ base tuning

    def _record_base_turns(self):
        turns = []
        real = self.arm.rotate_base
        self.arm.rotate_base = lambda *a, **kw: (turns.append(a), real(*a, **kw))[1]
        return turns

    def _base_cal(self):
        return arm_serial.get_calibration(
            self.session, ["BASE_RIGHT_MS", "BASE_LEFT_MS", "BASE_DIRECTION"])

    def test_base_tune_scales_the_turn_until_it_is_90(self):
        turns = self._record_base_turns()
        out = self._run_arm_calibrate([
            "rr", "",       # centre: one double nudge right
            "r", "45",      # turned right, only 45 deg -> twice as long
            "", "",         # centre; exactly 90 now
            "",             # same time for left turns
            "",             # final centre
        ], step=arm_serial.tune_base)
        self.assertEqual(turns, [(40, 500), (40, 2000), (-40, 2000),
                                 (40, 4000), (-40, 4000)])
        self.assertEqual(self._base_cal(), {"BASE_RIGHT_MS": 4000, "BASE_LEFT_MS": 4000,
                                            "BASE_DIRECTION": 1})
        self.assertIn("90 deg should take about 4000 ms", out)
        self.assertEqual(self.session.status()["base_angle"], 0)
        self.assertEqual(self.arm._base_lim, 4000)          # 90 deg limit back on
        self.assertEqual(self.arm.motors.speeds, [0, 0])

    def test_base_tune_fixes_swapped_direction_and_tunes_left(self):
        turns = self._record_base_turns()
        self._run_arm_calibrate([
            "", "l", "80",  # positive speed turned it LEFT, 80 deg
            "", "",         # now turning right: exactly 90
            "n",            # tune left separately
            "", "100",      # left turn went 100 deg
            "", "",         # exactly 90
            "",             # final centre
        ], step=arm_serial.tune_base)
        self.assertEqual(turns, [(40, 2000), (-40, 2000),
                                 (-40, 2250), (40, 2250),
                                 (40, 2250), (-40, 2250),
                                 (40, 2025), (-40, 2025)])
        self.assertEqual(self._base_cal(), {"BASE_RIGHT_MS": 2250, "BASE_LEFT_MS": 2025,
                                            "BASE_DIRECTION": -1})

    def test_base_tune_cancel_saves_nothing(self):
        answers = iter(["", "r"])

        def fake_input(prompt=""):
            try:
                return next(answers)
            except StopIteration:
                raise EOFError
        builtins = __builtins__ if isinstance(__builtins__, dict) else vars(__builtins__)
        real_input, builtins["input"] = builtins["input"], fake_input
        try:
            self.quiet(arm_serial.tune_base, self.session)
        finally:
            builtins["input"] = real_input
        self.assertFalse(os.path.exists(cyberbrick_arm.CAL_FILE))
        self.assertEqual(self.arm._base_lim, 2000)          # limit back on
        self.assertEqual(self.arm.motors.speeds, [0, 0])

    def test_base_tune_finishes_when_told_90(self):
        # Typing 90 (or close to it) must finish, not repeat the same turn.
        turns = self._record_base_turns()
        self._run_arm_calibrate([
            "", "r", "75°",     # 75 deg -> 2400 ms
            "", "90.5",         # close enough: done
            "y",                # same time for left
            "",
        ], step=arm_serial.tune_base)
        self.assertEqual(len(turns), 4)
        self.assertEqual(self._base_cal()["BASE_RIGHT_MS"], 2400)
        self._run_arm_calibrate(["", "r", "ok", "", ""], step=arm_serial.tune_base)
        self.assertEqual(self._base_cal()["BASE_RIGHT_MS"], 2400)

    def _fail_on_turn(self, n, error):
        """Make the n-th rotate_base call from the PC raise `error`."""
        real = self.session.call
        count = [0]

        def call(method, *args, **kw):
            if method == "rotate_base":
                count[0] += 1
                if count[0] == n:
                    real(method, *args, **kw)      # the arm starts the turn...
                    raise error                    # ...then the PC loses it
            return real(method, *args, **kw)
        self.session.call = call

    def test_base_tune_ctrl_c_during_a_turn_puts_the_limit_back(self):
        self._fail_on_turn(1, KeyboardInterrupt())
        self._run_arm_calibrate(["", "r"], step=arm_serial.tune_base)
        self.assertEqual(self.arm._base_lim, 2000)
        self.assertFalse(self.arm.base_moving())
        self.assertFalse(os.path.exists(cyberbrick_arm.CAL_FILE))

    def test_base_tune_lost_link_puts_the_limit_back(self):
        self._fail_on_turn(2, arm_serial.RadioError("no reply from the arm"))
        with self.assertRaises(arm_serial.RadioError):
            self._run_arm_calibrate(["", "r", "45"], step=arm_serial.tune_base)
        self.assertEqual(self.arm._base_lim, 2000)
        self.assertFalse(self.arm.base_moving())

    def test_base_tune_ignores_impossible_answers(self):
        self._run_arm_calibrate([
            "x", "",        # not a nudge: asked again
            "r", "abc", "5", "",    # not a number, too small, then 90
            "", "",
        ], step=arm_serial.tune_base)
        self.assertEqual(self._base_cal()["BASE_RIGHT_MS"], 2000)

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
        self.arm._cd[2] = 12000                       # e.g. half way there
        self.assertAlmostEqual(self.arm.joint_angle(2), 45)

    def test_timer_works_in_integers_only(self):
        # The 10 ms timer must not make floats (each one is garbage on the
        # board) and does not need the servo library's stepping.
        pwm = self.arm.servos.servos_map[0]
        pwm.history.clear()
        self.arm.servos.proc_calls = 0
        self.session.call("move_lower_arm", 37.3, speed=7)
        self.session.call("set_claw", 0.5)
        self.session.call("turn_base", 33)
        self.assertGreater(len(pwm.history), 100)
        self.assertTrue(all(type(ns) is int for ns in pwm.history))
        self.assertTrue(all(type(cd) is int for cd in self.arm._cd.values()))
        self.assertTrue(all(type(v) is int for v in (
            self.arm._claw_pos, self.arm._base_pos, self.arm._claw_applied,
            self.arm._base_applied)))
        self.assertEqual(self.arm.servos.proc_calls, 0)
        self.assertIsNone(self.arm.timer_error)
        self.assertEqual(self.arm.servo_angle(1), 37)

    def test_demo_runs_from_the_pc(self):
        self.quiet(arm_serial.run_demo, self.session)
        moves = [c for c in self.calls if c != "calibration"]
        self.assertEqual(moves[:3], ["home", "open_claw", "close_claw"])
        self.assertEqual(moves[-1], "close_claw")
        status = self.session.status()
        self.assertEqual((status["base_angle"], status["claw"]), (0, 0.0))

    def test_the_arm_library_has_no_standalone_code(self):
        # The demo runs from the PC and the sticks go through 'remote'.
        for name in ("demo", "run_rc", "rc_monitor", "main", "MODE"):
            self.assertFalse(hasattr(cyberbrick_arm, name), name)
        self.assertNotIn("center_base", arm_radio.METHODS)

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
            ("claw-set", ("open",)): "arm.calibrate_claw(True)",
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

    def test_old_names_still_work(self):
        for old, new in arm_serial.ALIASES.items():
            self.assertTrue(new in arm_serial.COMMANDS or new in arm_serial.SESSION_COMMANDS)
            self.assertNotIn(old, arm_serial.COMMANDS)
            self.assertNotIn(old, arm_serial.SESSION_COMMANDS)
        seen = []
        real = arm_serial._main
        arm_serial._main = lambda opts: (seen.append(opts.command), (0, {}))[1]
        try:
            arm_serial.main(["claw-cal", "open"])
            arm_serial.main(["base-calibrate"])
        finally:
            arm_serial._main = real
        self.assertEqual(seen, ["claw-set", "base-tune"])


class UsbLoadTest(unittest.TestCase):
    """Over USB the arm library always runs from the copy saved on the board."""

    class Link:
        def __init__(self, installed):
            self.installed = installed
            self.code = []

        def exec(self, code, timeout=30, echo=False):
            self.code.append(code)
            if "Pin(10" in code:
                return "0\n"                      # a receiver
            if "'arm' in globals()" in code:
                return "False\n"
            if "mem_free" in code:
                return "40000 1\n"
            if "open('/cyberbrick_arm.ver')" in code:
                return self.installed + "\n"
            if "json.dumps(arm.status())" in code:
                return '{"lower_arm": 0, "upper_arm": 0, "claw": 0.0, ' \
                       '"base_angle": 0, "base_speed": 0}\n'
            return ""

    def _load(self, installed):
        link = self.Link(installed)
        session = arm_serial.ArmSession(link)
        saved = []
        session._save_library = saved.append
        real = arm_serial.STATE_FILE
        arm_serial.STATE_FILE = os.path.join(tempfile.gettempdir(), "no-such-state.json")
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                session.load()
        finally:
            arm_serial.STATE_FILE = real
        return link, saved

    def test_saves_the_library_when_it_changed(self):
        link, saved = self._load("-")
        self.assertEqual(saved, [arm_serial.source_version()])
        self.assertTrue(any("from cyberbrick_arm import CyberBrickArm" in c
                            for c in link.code))

    def test_uses_the_saved_copy_when_it_is_current(self):
        link, saved = self._load(arm_serial.source_version())
        self.assertEqual(saved, [])
        self.assertTrue(any("arm = CyberBrickArm()" in c for c in link.code))


class PortLockTest(unittest.TestCase):
    """A second arm_serial.py waits for the port instead of breaking in."""

    def _open(self, errors):
        attempts = []

        def fake_serial(*args, **kw):
            attempts.append(kw.get("exclusive"))
            if errors:
                raise arm_serial.serial.SerialException(errors.pop(0))
            return object()
        link = arm_serial.CyberBrickSerial("/dev/ttyACM9")
        link.port = "/dev/ttyACM9"
        real, arm_serial.serial.Serial = arm_serial.serial.Serial, fake_serial
        real_sleep, time.sleep = time.sleep, lambda s: None
        try:
            with contextlib.redirect_stdout(io.StringIO()) as out:
                ok = link._open()
        finally:
            arm_serial.serial.Serial, time.sleep = real, real_sleep
        return ok, attempts, out.getvalue()

    def test_waits_while_another_command_has_the_port(self):
        ok, attempts, out = self._open([
            "Could not exclusively lock port /dev/ttyACM9: [Errno 11] Resource "
            "temporarily unavailable"] * 3)
        self.assertTrue(ok)
        self.assertEqual(attempts, [True] * 4)
        self.assertEqual(out.count("in use by another program"), 1)

    def test_other_errors_fail_at_once(self):
        ok, attempts, out = self._open(["[Errno 2] No such file or directory"])
        self.assertFalse(ok)
        self.assertEqual(len(attempts), 1)


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

    def test_power_cycle_during_start_tries_radio_mode_again(self):
        # Switched off while still starting up: the counter says 1, but that
        # was not a crash, so radio mode (the first try) runs again.
        arm_boot.TRIES = (("arm_radio_fake", "lib"), ("arm_radio_fake_old", "lib"))
        self._modules(set())
        with open(arm_boot.COUNT_FILE, "w") as f:
            f.write("1")
        real = machine.reset_cause
        machine.reset_cause = lambda: machine.PWRON_RESET
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                arm_boot._receiver()
        finally:
            machine.reset_cause = real
        self.assertEqual(self.tried, [("arm_radio_fake", "lib")])

    def test_soft_reset_restarts_the_board_first(self):
        # A soft reset leaves Wi-Fi unable to send, so radio mode must not
        # start on top of it.
        arm_boot.TRIES = (("arm_radio_fake", "lib"),)
        self._modules(set())
        real = machine.reset_cause
        machine.reset_cause = lambda: machine.SOFT_RESET
        try:
            with self.assertRaises(SystemExit):
                arm_boot._receiver()
        finally:
            machine.reset_cause = real
        self.assertEqual(self.tried, [])

    def test_everything_broken_means_stock(self):
        arm_boot.TRIES = (("arm_radio_fake", "lib"),)
        self._modules({("arm_radio_fake", "lib")})
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(arm_boot._receiver())
        self.assertFalse(os.path.exists(arm_boot.COUNT_FILE))   # try again next boot


class CompileTest(unittest.TestCase):
    """Every board file compiles for MicroPython 1.23 and stays small."""

    # arm_radio: 6000 -> 7500 for the radio diagnostics, recovery and joint
    # hold (the snapshot part is meant to come out again once the radio
    # fault is understood). The arm had 55 KB of RAM free with it at 6.5 KB.
    BUDGET = {"cyberbrick_arm": 9000, "arm_radio": 7500, "arm_bridge": 2000,
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
