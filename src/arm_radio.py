# -*- coding: utf-8 -*-
"""
Radio mode for the CyberBrick arm (MicroPython, runs on the arm's receiver).

The boot hook (arm_boot.py, installed as /lib/rc_main.mpy) starts this instead
of the stock remote-control firmware. It listens for commands from the paired
CyberBrick remote over ESP-NOW and runs them on the arm:

    PC --USB--> remote (arm_bridge.py) --ESP-NOW--> this file --> CyberBrickArm

Messages are JSON lists, at most 250 bytes (the ESP-NOW limit):

    request  [seq, op, args...]
    replies  [seq, "A"]                     accepted, still running
             [seq, "D", status, result]     done
             [seq, "E", "error text"]       failed

Ops (there is no way to run arbitrary code over the radio):
    "p"                         ping: reply with the status
    "v"                         versions of the files in UPDATES
    "c", method, args, kwargs, timeout_s
                                call one of the arm methods in METHODS
    "fo", name                  start replacing a file in UPDATES
    "fw", base64                write the next piece of it
    "fc", sha256_hex, version   check it and swap it in (old copy kept)
    "r", "radio" | "stock"      reboot, into radio mode or stock remote mode
    "d"                         radio diagnostics (read only): see Server.diag
    "d", "prev"                 the last diagnostics snapshot saved before
                                this boot (DIAG_FILE), or None

A request repeated with the same seq is never run twice: the arm answers with
the reply it already has. Only messages from the paired remote are accepted,
and the link is encrypted with the keys in RADIO_FILE.
"""

import gc
import json
import os
import sys
import time

import machine

RADIO_FILE = "/arm_radio.json"   # {"peer", "pmk", "lmk", "channel"}
MODE_FILE = "/arm_mode"          # read by the boot hook
MAX_MSG = 250                    # ESP-NOW payload limit
STATUS_MAX = MAX_MSG - 16        # room for the status in [seq, "D", status]
FREE_RUN_MAX_MS = 20000          # free spins stop by themselves after this
# Diagnostics snapshot, so a radio outage that needs a power cycle leaves
# evidence: one small file (under MAX_MSG bytes), overwritten in place every
# DIAG_EVERY_MS while nothing moves, and skipped when flash is short.
DIAG_FILE = "/arm_diag.json"
DIAG_EVERY_MS = 30000
DIAG_MIN_FREE = 64 * 1024
# Radio recovery (Server.recover): after a reply fails every try, wait this
# long after each step to hear the remote again. The remote resends every
# 300 ms while waiting for a reply and checks in every 1500 ms during a move,
# and gives up after 5 s, so both steps fit before it does.
RECOVER_WAIT_MS = 1600
# Joint angles saved just before a recovery reboot and put back as radio mode
# starts, so the joints stay where they were instead of going home.
HOLD_FILE = "/arm_hold.json"
JOINT_SERVOS = (1, 2)
# Dropped in this order if a diagnostics reply would not fit in one message.
DIAG_SPARE = ("ign", "pm", "ap", "sf", "ev", "mv")

# Parts of the arm, as bit flags.
JOINTS, CLAW, BASE = 1, 2, 4

# Arm methods the radio may call: name -> (parts to wait for, parts it uses).
# Methods with something to wait for are called with wait=False and the reply
# is sent when those parts stop, so a "stop" can still get through.
METHODS = {
    "move_lower_arm": (JOINTS, JOINTS),
    "move_upper_arm": (JOINTS, JOINTS),
    "move_arm": (JOINTS, JOINTS),
    "home": (JOINTS | BASE, JOINTS | BASE),
    "set_claw": (CLAW, CLAW),
    "open_claw": (CLAW, CLAW),
    "close_claw": (CLAW, CLAW),
    "turn_base": (BASE, BASE),
    "turn_base_by": (BASE, BASE),
    "rotate_base": (BASE, BASE),
    "claw_run": (0, CLAW),
    "claw_stop": (0, CLAW),
    "calibrate_claw": (0, CLAW),
    "base_speed": (0, BASE),
    "base_stop": (0, BASE),
    "set_base_angle": (0, BASE),
    "set_base_limit": (0, BASE),
    "set_joint_limits": (0, JOINTS),
    "stop_all": (0, JOINTS | CLAW | BASE),
    "hold_joints": (0, JOINTS),
    "release_joints": (0, JOINTS),
    "status": (0, 0),
    "calibration": (0, 0),
    "save_calibration": (0, CLAW | BASE),
}
# Spins that only end when told to; they get a FREE_RUN_MAX_MS deadline in
# case the radio link is lost before the stop arrives.
FREE_RUNS = {"claw_run": CLAW, "base_speed": BASE}

# Files the radio may replace: name -> (path, version file, old copy).
UPDATES = {
    "cyberbrick_arm": ("/cyberbrick_arm.mpy", "/cyberbrick_arm.ver",
                       "/cyberbrick_arm_old.mpy"),
    "arm_radio": ("/arm_radio.mpy", "/arm_radio.ver", "/arm_radio_old.mpy"),
}


def _dumps(obj):
    """Compact JSON (no spaces), so more fits in one ESP-NOW message."""
    return json.dumps(obj, separators=(",", ":"))


def _remove(path):
    try:
        os.remove(path)
    except OSError:
        pass


def _exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


class Radio:
    """ESP-NOW link to the one paired remote."""

    def __init__(self, cfg):
        import binascii
        import espnow
        import network

        self.wlan = network.WLAN(network.STA_IF)
        # active(True) never returns if the stock firmware already started
        # Wi-Fi (e.g. after a soft reset), so only switch it on when it is off.
        if not self.wlan.active():
            self.wlan.active(True)
        try:
            self.wlan.disconnect()
        except OSError:
            pass
        self.channel = int(cfg["channel"])
        self.wlan.config(channel=self.channel)
        self.pmk = binascii.unhexlify(cfg["pmk"])
        self.lmk = binascii.unhexlify(cfg["lmk"])
        self.peer = binascii.unhexlify(cfg["peer"].replace(":", ""))
        self.e = espnow.ESPNow()
        self._start_espnow()
        # Counters for the "d" diagnostics request.
        self.rx = 0            # requests taken from the peer
        self.ign = 0           # messages ignored (other boards, not a request)
        self.tx_retry = 0      # reply sends that were not acked (each try)
        self.tx_fail = 0       # replies given up on after every try failed
        self.last_rx = None    # tick of the last request from the peer
        self.busy = None       # callable: is any part of the arm moving?
        self.mv = [0, 0]       # send tries while moving, of them not acked
        self.fail_moving = 0   # 1 if the last reply given up on was sent
                               # while something moved

    def _start_espnow(self):
        self.e.active(True)
        self.e.set_pmk(self.pmk)
        try:
            self.e.add_peer(self.peer, self.lmk)
        except OSError:
            # Already added (an earlier start attempt in this boot).
            self.e.mod_peer(self.peer, self.lmk)

    def restart_espnow(self):
        """Recovery step 1: switch ESP-NOW off and on, and add the peer again."""
        try:
            self.e.active(False)
        except OSError:
            pass
        self._start_espnow()

    def restart_wifi(self):
        """Recovery step 2: switch the Wi-Fi interface off and on, back on our
        channel, then ESP-NOW. (We switched it on ourselves, so active(True)
        is not the stock-firmware case that hangs.)"""
        try:
            self.e.active(False)
        except OSError:
            pass
        self.wlan.active(False)
        self.wlan.active(True)
        try:
            self.wlan.disconnect()
        except OSError:
            pass
        self.wlan.config(channel=self.channel)
        self._start_espnow()

    def recv(self, timeout_ms):
        """The next message from the peer as bytes, or None."""
        mac, msg = self.e.irecv(timeout_ms)
        if not msg:
            return None
        if mac == self.peer and msg[0] == 0x5B:   # '['
            self.rx += 1
            self.last_rx = time.ticks_ms()
            return bytes(msg)
        self.ign += 1
        return None

    def send(self, reply):
        data = _dumps(reply)
        if len(data) > MAX_MSG:
            data = _dumps([reply[0], "E", "reply too long"])
        moving = 1 if self.busy and self.busy() else 0
        for _ in range(3):
            self.mv[0] += moving
            try:
                if self.e.send(self.peer, data, True):
                    return True
            except OSError:
                pass
            self.tx_retry += 1
            self.mv[1] += moving
            time.sleep_ms(5)
        self.tx_fail += 1
        self.fail_moving = moving
        return False


class Server:
    """Runs radio requests on the arm, one at a time."""

    def __init__(self, arm, radio, boot_id, lib="cyberbrick_arm", rst=None):
        self.arm = arm
        self.radio = radio
        self.boot = boot_id
        self.rst = rst         # machine.reset_cause() at boot: why it restarted
        # Which files are running, only when they are the backup copies:
        # e.g. "radio+arm_old".
        self.code = (None if (__name__, lib) == ("arm_radio", "cyberbrick_arm")
                     else __name__[4:] + "+" + lib[11:])
        self.seq = None        # seq of the request running or last run
        self.reply = None      # its final reply, once there is one
        self.wait = 0          # parts the running request waits for
        self.deadline = 0
        self.free = {}         # part -> tick a free spin must stop by
        self.reboot = None     # "radio" / "stock" once a reboot was asked for
        self.file = None       # update in progress: [name, file object, sha256]
        self.error = None      # last error from the main loop
        self.loops = 0         # main-loop rounds: shows the loop kept running
        # Radio recovery: the step running as [step, until tick, rx and
        # tx_fail counts when it started], or None; the tx_fail count
        # already dealt with; and [step 1 tries, step 1 fixes, step 2 tries,
        # step 2 fixes].
        self.rec = None
        self.fails_seen = 0
        self.rc = [0, 0, 0, 0]
        self.rec_reboot = False    # rebooting because recovery failed
        # The last 3 recoveries: [uptime s, 1 if the failed reply was sent
        # while something moved, step that fixed it (3 = reboot; 0 = still
        # trying)].
        self.ev = []
        radio.busy = self.moving
        # The snapshot saved before this boot, read before it is overwritten.
        self.prev = _read(DIAG_FILE)
        self.next_save = time.ticks_add(time.ticks_ms(), DIAG_EVERY_MS)

    def moving(self):
        arm = self.arm
        return bool(self.wait or arm.joints_moving() or arm.claw_moving()
                    or arm.base_moving())

    # ----------------------------------------------------------------- replies

    def status(self):
        s = self.arm.status()
        s["boot"] = self.boot
        s["mem"] = gc.mem_free()
        if self.rst is not None:
            s["rst"] = self.rst
        if self.code:
            s["code"] = self.code
        if self.error is not None:
            s["radio_error"] = self.error
        # The status must fit in one message: shorten the error texts, and
        # if it still does not fit, leave out the extras.
        for key in ("radio_error", "timer_error"):
            if key in s:
                s[key] = s[key][:40]
        for key in ("radio_error", "timer_error", "code"):
            if len(_dumps(s)) <= STATUS_MAX:
                break
            s.pop(key, None)
        return s

    def finish(self, reply):
        self.wait = 0
        self.reply = reply
        self.radio.send(reply)

    def done(self, result=None):
        # A result (e.g. calibration values) goes without the status, so the
        # reply stays under MAX_MSG.
        if result is None:
            self.finish([self.seq, "D", self.status()])
        else:
            self.finish([self.seq, "D", None, result])

    # --------------------------------------------------------------- requests

    def handle(self, msg):
        try:
            req = json.loads(msg)
            seq, op = req[0], req[1]
        except (ValueError, TypeError, IndexError, KeyError):
            return
        if seq == self.seq:
            # The remote did not hear us; say again where we are.
            self.radio.send(self.reply or [seq, "A"])
            return
        self.seq, self.reply, self.wait = seq, None, 0
        try:
            result = self.run(op, req[2:])
        except Exception as e:
            self.finish([seq, "E", "%s: %s" % (type(e).__name__, e)])
            return
        if self.wait:
            self.radio.send([seq, "A"])
        else:
            self.done(result)

    def run(self, op, args):
        if op == "p":
            return None
        if op == "c":
            return self.call(*args)
        if op == "v":
            return {name: _read(UPDATES[name][1]) for name in UPDATES}
        if op == "fo":
            return self.file_open(args[0])
        if op == "fw":
            return self.file_write(args[0])
        if op == "fc":
            return self.file_commit(args[0], args[1])
        if op == "r":
            if args[0] not in ("radio", "stock"):
                raise ValueError("reboot mode must be radio or stock")
            self.reboot = args[0]
            return None
        if op == "d":
            if args and args[0] == "prev":
                return json.loads(self.prev) if self.prev else None
            return self.diag()
        raise ValueError("unknown op %r" % op)

    def diag(self):
        """Radio diagnostics, as seen from the arm. Read only.

        up   ms since boot          st   ESP-NOW stats(): sends, send
        boot boot id                     results, not acked (every reply
        loop main-loop rounds            try goes through Radio.send, so
        last ms since the last           this is tx_retry), received,
             request (None: none)        dropped
        rx   requests heard         ch   Wi-Fi channel
        ign  messages ignored       pm   Wi-Fi power-save mode
        sf   replies given up on    ap   1 if the Wi-Fi AP interface is on
        rc   radio recovery: [step 1 tries, step 1 fixes, step 2 tries,
             step 2 fixes] (see recover)
        rb   1: saved just before a recovery reboot (only then)
        ev   the last 3 recoveries: [uptime s, moving when the reply failed,
             step that fixed it (3 = reboot, 0 = still trying)]
        mv   send tries while something moved, and of them not acked
             (idle = the totals in st minus these)

        Fields in DIAG_SPARE are left out if the reply would not fit in one
        message.
        """
        r = self.radio
        now = time.ticks_ms()
        d = {"up": now, "boot": self.boot, "loop": self.loops,
             "last": (None if r.last_rx is None
                      else time.ticks_diff(now, r.last_rx)),
             "rx": r.rx, "ign": r.ign, "sf": r.tx_fail,
             "rc": self.rc, "ev": self.ev, "mv": r.mv}
        if self.rec_reboot:
            d["rb"] = 1
        try:
            d["st"] = list(r.e.stats())
        except Exception:
            pass
        try:
            d["ch"] = r.wlan.config("channel")
        except Exception:
            pass
        try:
            d["pm"] = r.wlan.config("pm")
        except Exception:
            pass
        try:
            import network
            d["ap"] = int(network.WLAN(network.AP_IF).active())
        except Exception:
            pass
        # (The last main-loop error is in status() as radio_error.)
        # Keep the whole reply, [seq, "D", None, d], to one message.
        for key in DIAG_SPARE:
            if len(_dumps([65535, "D", None, d])) <= MAX_MSG:
                break
            d.pop(key, None)
        return d

    # --------------------------------------------------------------- recovery

    def recover(self):
        """Bring the radio back when a reply fails every try. Called from the
        main loop.

        Seen on the arm: after a while idle, the radio takes in one request,
        then neither its replies nor anything sent to it get through, while
        this loop keeps running, until the arm is switched off and on. So:
          step 1  restart ESP-NOW,
          step 2  restart the Wi-Fi interface (and ESP-NOW),
          step 3  reboot into radio mode, like "r" (the snapshot says so).
        A step has worked when the remote is heard again and the reply to it
        gets through; the remote is still resending, so the request that hit
        the problem is answered. self.rc counts tries and fixes.
        """
        r = self.radio
        rec = self.rec
        if rec is None:
            if r.tx_fail != self.fails_seen:
                self.fails_seen = r.tx_fail
                self.ev.append([time.ticks_ms() // 1000, r.fail_moving, 0])
                del self.ev[:-3]
                self._recover_step(1)
            return
        step, until, rx0, fails0 = rec
        if r.tx_fail != fails0:
            self.fails_seen = r.tx_fail
            self._recover_step(step + 1)        # still cannot send
        elif r.rx != rx0:
            self.rc[2 * step - 1] += 1          # heard, and the reply got out
            self.ev[-1][2] = step
            self.rec = None
            self.next_save = time.ticks_ms()    # record it soon
        elif time.ticks_diff(time.ticks_ms(), until) >= 0:
            self._recover_step(step + 1)        # heard nothing

    def _recover_step(self, step):
        r = self.radio
        if step > 2:
            self.rec = None
            self.rec_reboot = True
            self.ev[-1][2] = 3
            try:
                self.next_save = time.ticks_ms()
                self.save_diag(force=True)
            except Exception:
                pass
            try:
                # Keep the joints where they are across the reboot (run()
                # puts them back).
                arm = self.arm
                arm.hold_joints()
                held = {str(s): arm.servo_angle(s) for s in JOINT_SERVOS}
                held["b"] = self.boot
                with open(HOLD_FILE, "w") as f:
                    f.write(_dumps(held))
            except Exception:
                pass
            self.reboot = "radio"
            return
        self.rc[2 * step - 2] += 1
        try:
            if step == 1:
                r.restart_espnow()
            else:
                r.restart_wifi()
        except Exception as e:
            self.error = "recover %d: %s: %s" % (step, type(e).__name__, e)
        self.rec = [step, time.ticks_add(time.ticks_ms(), RECOVER_WAIT_MS),
                    r.rx, r.tx_fail]

    def save_diag(self, force=False):
        """Every DIAG_EVERY_MS while nothing moves, overwrite DIAG_FILE with
        diag(), unless flash is short. Called from the main loop."""
        now = time.ticks_ms()
        if time.ticks_diff(now, self.next_save) < 0:
            return
        arm = self.arm
        if not force and (self.wait or self.free or self.file
                          or arm.joints_moving() or arm.claw_moving()
                          or arm.base_moving()):
            return             # try again on the next round
        self.next_save = time.ticks_add(now, DIAG_EVERY_MS)
        st = os.statvfs("/")
        if st[0] * st[4] < DIAG_MIN_FREE:     # block size * free blocks
            return
        with open(DIAG_FILE, "w") as f:
            f.write(_dumps(self.diag()))

    def call(self, name, args=(), kwargs=None, timeout_s=60):
        if name not in METHODS:
            raise ValueError("not allowed: %s" % name)
        wait, parts = METHODS[name]
        kwargs = kwargs or {}
        if wait:
            kwargs["wait"] = False
        result = getattr(self.arm, name)(*args, **kwargs)
        now = time.ticks_ms()
        for part in (CLAW, BASE):
            if parts & part:
                self.free.pop(part, None)
        if name in FREE_RUNS and args and args[0]:
            self.free[FREE_RUNS[name]] = time.ticks_add(now, FREE_RUN_MAX_MS)
        if wait:
            self.wait = wait
            self.deadline = time.ticks_add(now, int(timeout_s * 1000))
            return None
        return None if name == "status" else result

    def poll(self):
        """Stop expired free spins and reply once a running move is done."""
        now = time.ticks_ms()
        arm = self.arm
        for part in list(self.free):
            if time.ticks_diff(now, self.free[part]) >= 0:
                del self.free[part]
                if part == CLAW:
                    arm.claw_stop()
                else:
                    arm.base_stop()
        wait = self.wait
        if not wait:
            return
        busy = ((wait & JOINTS and arm.joints_moving())
                or (wait & CLAW and arm.claw_moving())
                or (wait & BASE and arm.base_moving()))
        if not busy:
            self.done()
        elif time.ticks_diff(now, self.deadline) >= 0:
            if wait & CLAW:
                arm.claw_stop()
            if wait & BASE:
                arm.base_stop()
            if wait & JOINTS:
                arm.hold_joints()
            self.finish([self.seq, "E", "timed out; stopped"])

    # ---------------------------------------------------------------- updates

    def file_open(self, name):
        if name not in UPDATES:
            raise ValueError("cannot update %r" % name)
        import hashlib
        if self.file:
            self.file[1].close()
        path, ver_path, _ = UPDATES[name]
        self.file = [name, open(path + ".tmp", "wb"), hashlib.sha256()]
        return {"ver": _read(ver_path)}

    def file_write(self, b64):
        import binascii
        if not self.file:
            raise ValueError("no update started")
        data = binascii.a2b_base64(b64)
        self.file[1].write(data)
        self.file[2].update(data)
        return len(data)

    def file_commit(self, sha_hex, version):
        import binascii
        if not self.file:
            raise ValueError("no update started")
        name, f, h = self.file
        self.file = None
        f.close()
        path, ver_path, old = UPDATES[name]
        if binascii.hexlify(h.digest()).decode() != sha_hex:
            _remove(path + ".tmp")
            raise ValueError("file was damaged on the way; nothing changed")
        # Keep the copy that is running now; the boot hook falls back to it
        # if the new one does not start.
        if _exists(path):
            _remove(old)
            os.rename(path, old)
        os.rename(path + ".tmp", path)
        with open(ver_path, "w") as vf:
            vf.write(version)
        return {"ver": version}


def serve(server):
    radio = server.radio
    while True:
        try:
            server.loops += 1
            msg = radio.recv(10)
            if msg:
                server.handle(msg)
                if not server.wait:
                    # Tidy up after each request while no move is running:
                    # garbage never piles up into a long pause at a bad
                    # moment, and status() "mem" shows what is really free.
                    gc.collect()
            server.poll()
            server.recover()
            server.save_diag()
            if server.reboot:
                if server.reboot == "stock":
                    with open(MODE_FILE, "w") as f:
                        f.write("stock")
                server.arm.shutdown()
                time.sleep_ms(100)
                machine.reset()
        except KeyboardInterrupt:
            raise
        except Exception as e:
            # Keep listening whatever happens; status() reports it.
            server.error = "%s: %s" % (type(e).__name__, e)
            time.sleep_ms(10)


def _led(rgb):
    try:
        import neopixel
        np = neopixel.NeoPixel(machine.Pin(8, machine.Pin.OUT), 1)
        np[0] = rgb
        np.write()
    except Exception:
        pass


def restore_hold(arm, rst):
    """After a recovery reboot, put the joints back where they were held.

    Only after a reset by this code (machine.HARD_RESET, as machine.reset()
    gives), and only if DIAG_FILE is the recovery-reboot snapshot of the same
    boot that held them (so a file left behind, e.g. when a backup copy ran
    in between, is never used later); after power-on the joints go home as
    usual. The file is used up either way.
    """
    data = _read(HOLD_FILE)
    if data is None:
        return
    _remove(HOLD_FILE)
    try:
        held = json.loads(data)
        snap = json.loads(_read(DIAG_FILE) or "{}")
        if (rst != machine.HARD_RESET or not snap.get("rb")
                or snap.get("boot") != held.get("b")):
            return
        for servo in JOINT_SERVOS:
            arm._set_servo(servo, held[str(servo)])
    except Exception:
        pass               # the joints stay at home


def run(lib="cyberbrick_arm", on_ready=None):
    """Start radio mode. Returns only after Ctrl-C (from a USB session).

    lib: the arm library module to use (the boot hook passes the old copy if
    the new one failed). on_ready: called once the radio and arm are up.
    """
    if "/bbl" not in sys.path:
        sys.path.append("/bbl")
    with open(RADIO_FILE) as f:
        cfg = json.load(f)
    try:
        rst = machine.reset_cause()
    except Exception:
        rst = None
    radio = Radio(cfg)
    gc.collect()
    arm = __import__(lib).CyberBrickArm()
    restore_hold(arm, rst)     # straight away: the library just sent them home
    gc.collect()
    server = Server(arm, radio, int.from_bytes(os.urandom(2), "big"), lib, rst)
    if on_ready:
        on_ready()
    _led((0, 0, 24))           # dim blue: listening for the radio
    print("[arm] radio mode, listening for the paired remote")
    try:
        serve(server)
    finally:
        arm.shutdown()
        _led((0, 0, 0))
