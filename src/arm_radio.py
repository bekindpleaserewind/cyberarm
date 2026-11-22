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
    "m"                         memory and Wi-Fi state (read only): see
                                Server.memory
    "t", seconds[, echo]        restart into radio test mode for that long
                                (RadioTest), then back into radio mode
    "tr", offset[, "prev"]      the last radio test's result (TEST_RESULT;
                                "prev": the one before), a piece at a time:
                                {"n": length, "d": text}
    "s", {"channel", "txpower", "enc"}
                                save new radio settings in RADIO_FILE; they
                                take effect at the next start ("r", "radio")

A request repeated with the same seq is never run twice: the arm answers with
the reply it already has. Only messages from the paired remote are accepted,
and the link is encrypted with the keys in RADIO_FILE (unless "enc" is 0
there: a test setting, as any nearby board could then send commands).
"""

import gc
import json
import os
import sys
import time

import machine

RADIO_FILE = "/arm_radio.json"   # {"peer", "pmk", "lmk", "channel", "txpower", "enc"}
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
# and gives up after 10 s once the arm's radio has acked, so both steps fit
# before it does.
RECOVER_WAIT_MS = 1600
# Joint angles saved just before a recovery reboot and put back as radio mode
# starts, so the joints stay where they were instead of going home.
HOLD_FILE = "/arm_hold.json"
JOINT_SERVOS = (1, 2)
# Present while wifi_reset() switches Wi-Fi off and on at startup. If it is
# still there at the next start, that step hung the board (the stock firmware
# may already own Wi-Fi), so from then on the board starts the gentle way.
# radio-install removes it.
WIFI_MARK = "/arm_wifi_try"
# ESP-NOW receive buffer: room for 7 full-size messages (the default holds
# 2); the same as the remote (arm_bridge.RXBUF). Set at every ESP-NOW start:
# start-up (START_RXBUF), radio test mode, and recovery.
RXBUF = 2048
# Start-up steps added on 2026-11-21, switched off one by one to find which
# leaves the arm deaf after power-on. Measured 2026-11-22 with all three on:
# radio mode ran within 5 s of power-on, but the arm's radio acked nothing
# from the remote for about 45 s (once over 2 minutes). With all off, the
# start is the one from before: station on only if it is off, disconnect,
# set the channel. Recovery (restart_wifi) still does the full wifi_reset.
START_FULL = False      # switch the station off and on (wifi_reset full)
START_PM = True         # power saving off (as the ESP-NOW docs say), and no
                        # auto-reconnect (reconnects=0), as CyberBrick
                        # community ESP-NOW code does
START_RXBUF = True      # ESP-NOW receive buffer RXBUF
# With nothing heard from the remote for this long and nothing moving, add
# the remote as a peer again, and again every REKEY_IDLE_MS after that. The
# remote may have restarted, leaving this side's link state for it stale: its
# messages are then acked by the radio but never delivered. The remote keeps
# resending for NO_REPLY_MS (10 s, arm_bridge.py), so this lands well inside.
REKEY_IDLE_MS = 3000
# Switched off to test whether the idle re-key triggers the deaf spells
# (2026-11-22: the arm stopped hearing right after the first idle gap long
# enough for one).
IDLE_REKEY = False
# Recovery step 3, restarting the arm, is off: it moves the arm without
# warning (seen twice on 2026-11-22). Recovery then stops after the Wi-Fi
# restart, and starts again at step 1 with the next failed reply.
RECOVER_REBOOT = False
# Radio test mode (RadioTest). TEST_FILE asks for it at the next start; it
# is removed before the test begins, so every later start (including one
# after power-off during the test) is normal radio mode. Never after a
# power-on, even if the file was left. TEST_RESULT keeps the last result;
# as a test starts, the one before is kept as TEST_PREV.
TEST_FILE = "/arm_rtest"
TEST_RESULT = "/arm_rtest.json"
TEST_PREV = "/arm_rtest_prev.json"
TEST_MAX_S = 1800
TEST_CHUNK = 100        # result text per "tr" reply: even all quotes (each
                        # doubles in JSON) fits in MAX_MSG with the framing
# Memory low points (Server.mem_sample) are sampled this often.
MEM_EVERY_MS = 1000
# Transmit power limits for "s" (dBm; the stock remote uses 20).
TXPOWER_MIN, TXPOWER_MAX = 2, 20
# Dropped in this order if a diagnostics reply would not fit in one message.
DIAG_SPARE = ("rk", "wr", "ign", "pm", "on", "sf", "ev", "mv")

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


def wifi_info():
    """Wi-Fi interface state, read only: [station on, its channel, its
    status(), saved network name, AP on, its channel]; -1 for what could not
    be read."""
    import network
    out = []
    for i, keys in ((network.STA_IF, ("channel", "status", "ssid")),
                    (network.AP_IF, ("channel",))):
        w = network.WLAN(i)
        for k in ("on",) + keys:
            try:
                v = (int(w.active()) if k == "on" else
                     w.status() if k == "status" else w.config(k))
                if isinstance(v, bytes):
                    v = v.decode()
            except Exception:
                v = -1
            out.append(v)
    return out


def _sys_free():
    """Free bytes in the system (ESP-IDF) heap, all HEAP_DATA regions added
    up, or None if this firmware cannot tell."""
    try:
        import esp32
        return sum(r[1] for r in esp32.idf_heap_info(esp32.HEAP_DATA))
    except Exception:
        return None


def _write(path, text):
    try:
        with open(path, "w") as f:
            f.write(text)
    except OSError:
        pass


def wifi_reset(wlan, channel, full, pm=True):
    """Put Wi-Fi in a known state for ESP-NOW: station on, not connected, on
    `channel`, power saving off.

    The stock firmware starts Wi-Fi before this code runs (the arm showed the
    access point on and power saving on). Power saving turns the radio off
    for hundreds of ms at a time, so messages are missed.

    pm: turn power saving off (False: leave it as it is).

    full: switch the station off and on even if it is already on. Plain
    active(True) on a station the stock firmware started never returns; off
    and then on is what restart_wifi has always done safely. Without full,
    the station is only switched on if it is off, as before.

    The access point is left alone: on this firmware the station and the
    access point are one radio, and switching the access point off stops
    Wi-Fi altogether (measured on the remote: the station then reads off and
    the channel cannot be set), so ESP-NOW neither sends nor hears.

    Raises OSError if Wi-Fi did not end up on and on `channel`, so a board
    that would be deaf fails at start-up (the boot hook then falls back)
    instead of looking alive.
    (Copy in arm_bridge.py: keep them the same.)
    """
    if full and wlan.active():
        wlan.active(False)
    if not wlan.active():
        wlan.active(True)
    try:
        wlan.config(reconnects=0)    # never try to rejoin a saved network
    except Exception:
        pass
    try:
        wlan.disconnect()
    except OSError:
        pass
    wlan.config(channel=channel)
    if pm:
        try:
            wlan.config(pm=wlan.PM_NONE)
        except Exception:
            pass
    if not wlan.active() or wlan.config("channel") != channel:
        raise OSError("Wi-Fi did not start on channel %d" % channel)


class Radio:
    """ESP-NOW link to the one paired remote."""

    def __init__(self, cfg):
        import binascii
        import espnow
        import network

        self.wlan = network.WLAN(network.STA_IF)
        self.channel = int(cfg["channel"])
        if START_FULL or START_PM:
            # The full reset, unless it hung the board last time (WIFI_MARK).
            self.wr = 0 if not START_FULL or _exists(WIFI_MARK) else 1
            if self.wr:
                _write(WIFI_MARK, "1")
            try:
                wifi_reset(self.wlan, self.channel, self.wr, START_PM)
            except Exception:
                # Only a hang leaves the mark; an error is not a reason to
                # start the gentle way.
                if self.wr:
                    _remove(WIFI_MARK)
                raise
            if self.wr:
                _remove(WIFI_MARK)
        else:
            # The start from before 2026-11-21. active(True) never returns
            # if the stock firmware already started Wi-Fi (e.g. after a soft
            # reset), so only switch it on when it is off.
            self.wr = 0
            if not self.wlan.active():
                self.wlan.active(True)
            try:
                self.wlan.disconnect()
            except OSError:
                pass
            self.wlan.config(channel=self.channel)
        # Transmit power in dBm, or None: the firmware's default.
        self.txpower = cfg.get("txpower")
        self.tx_err = None
        self._apply_txpower()
        self.pmk = binascii.unhexlify(cfg["pmk"])
        # No LMK: an unencrypted peer (cfg "enc": 0; the stock firmware's
        # peers are unencrypted).
        self.enc = cfg.get("enc", 1)
        self.lmk = binascii.unhexlify(cfg["lmk"]) if self.enc else None
        self.peer = binascii.unhexlify(cfg["peer"].replace(":", ""))
        self.e = espnow.ESPNow()
        self._start_espnow(START_RXBUF)
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
        self.rk = 0            # idle re-keys (Server.idle_rekey)

    def _apply_txpower(self):
        """Set the transmit power from RADIO_FILE (after every Wi-Fi start:
        switching Wi-Fi off and on may put it back to the default)."""
        if self.txpower is None:
            return
        try:
            self.wlan.config(txpower=self.txpower)
        except Exception as e:
            self.tx_err = "%s: %s" % (type(e).__name__, e)

    def _start_espnow(self, rxbuf=True):
        if rxbuf:
            try:
                self.e.config(rxbuf=RXBUF)     # takes effect on active(True)
            except Exception:
                pass
        self.e.active(True)
        self.e.set_pmk(self.pmk)
        try:
            self.e.add_peer(self.peer, self.lmk)
        except OSError:
            # Already added (an earlier start attempt in this boot).
            self.e.mod_peer(self.peer, self.lmk)

    def rekey(self):
        """Remove the remote as a peer and add it again, so this side's link
        state for it starts afresh (see REKEY_IDLE_MS)."""
        try:
            self.e.del_peer(self.peer)
        except OSError:
            pass
        self.e.add_peer(self.peer, self.lmk)
        self.rk += 1

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
        wifi_reset(self.wlan, self.channel, True)
        self._apply_txpower()
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
        # Lowest free memory seen by mem_sample, as [bytes, uptime ms]:
        # MicroPython's heap, and the system (ESP-IDF) heap the Wi-Fi driver
        # takes its buffers from (all HEAP_DATA regions added up).
        self.gc_low = self.sys_low = None
        self.next_mem = time.ticks_ms()
        self.next_rekey = time.ticks_add(time.ticks_ms(), REKEY_IDLE_MS)

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
        if op == "s":
            return self.settings(args[0])
        if op == "m":
            return self.memory()
        if op == "t":
            return self.start_test(args[0], args[1] if len(args) > 1 else 1)
        if op == "tr":
            text = _read(TEST_PREV if args[1:] == ["prev"] else TEST_RESULT)
            if text is None:
                return None
            off = int(args[0])
            return {"n": len(text), "d": text[off:off + TEST_CHUNK]}
        raise ValueError("unknown op %r" % op)

    def start_test(self, seconds, echo):
        """Restart into radio test mode for `seconds` (see RadioTest).
        Returns the test's id, which its result carries."""
        if not isinstance(seconds, int) or not 1 <= seconds <= TEST_MAX_S:
            raise ValueError("seconds must be 1-%d" % TEST_MAX_S)
        test_id = int.from_bytes(os.urandom(2), "big")
        _write(TEST_FILE, _dumps({"s": seconds, "e": 1 if echo else 0,
                                  "id": test_id, "b": self.boot}))
        if not _exists(TEST_FILE):
            raise OSError("could not save %s" % TEST_FILE)
        self.reboot = "radio"
        return {"s": seconds, "id": test_id}

    def mem_sample(self):
        """Every MEM_EVERY_MS, note the lowest free memory. Called from the
        main loop. Read only."""
        now = time.ticks_ms()
        if time.ticks_diff(now, self.next_mem) < 0:
            return
        self.next_mem = time.ticks_add(now, MEM_EVERY_MS)
        free = gc.mem_free()
        if self.gc_low is None or free < self.gc_low[0]:
            self.gc_low = [free, now]
        free = _sys_free()
        if free is not None and (self.sys_low is None or free < self.sys_low[0]):
            self.sys_low = [free, now]

    def memory(self):
        """Memory now and its low points. Read only.

        up   ms since boot
        gc   MicroPython heap: [free now, in use now, lowest free seen,
             uptime ms of that] (free includes garbage not yet collected)
        sys  system heap, all HEAP_DATA regions added up: [free now, lowest
             seen, uptime ms of that]
        wifi wifi_info(): [station on, channel, status, saved network name,
             AP on, AP channel]
        hp   per HEAP_DATA region, from esp32.idf_heap_info: [total, free,
             largest free block, lowest free since boot (kept by ESP-IDF)]
        """
        low = self.gc_low or [None, None]
        sys_low = self.sys_low or [None, None]
        d = {"up": time.ticks_ms(),
             "gc": [gc.mem_free(), gc.mem_alloc(), low[0], low[1]],
             "sys": [_sys_free(), sys_low[0], sys_low[1]],
             "wifi": wifi_info()}
        try:
            import esp32
            d["hp"] = [list(r) for r in esp32.idf_heap_info(esp32.HEAP_DATA)]
        except Exception as e:
            d["hp"] = "%s: %s" % (type(e).__name__, e)
        return d

    def settings(self, new):
        """Save a new channel and/or transmit power in RADIO_FILE, for the
        next start. Returns the saved channel and txpower. The keys stay."""
        with open(RADIO_FILE) as f:
            cfg = json.load(f)
        for key in new:
            if key not in ("channel", "txpower", "enc"):
                raise ValueError("cannot set %r" % key)
        if "enc" in new:
            if new["enc"] not in (0, 1):
                raise ValueError("enc must be 0 or 1")
            cfg["enc"] = int(new["enc"])
        if "channel" in new:
            ch = new["channel"]
            if not isinstance(ch, int) or not 1 <= ch <= 13:
                raise ValueError("channel must be 1-13")
            cfg["channel"] = ch
        if "txpower" in new:
            tx = new["txpower"]
            if tx is not None and not TXPOWER_MIN <= tx <= TXPOWER_MAX:
                raise ValueError("txpower must be %d-%d dBm" % (TXPOWER_MIN,
                                                               TXPOWER_MAX))
            cfg["txpower"] = tx
        with open(RADIO_FILE + ".tmp", "w") as f:
            f.write(_dumps(cfg))
        os.rename(RADIO_FILE + ".tmp", RADIO_FILE)
        return {"channel": cfg["channel"], "txpower": cfg.get("txpower"),
                "enc": cfg.get("enc", 1)}

    def diag(self):
        """Radio diagnostics, as seen from the arm. Read only.

        up   ms since boot          st   ESP-NOW stats(): sends, send
        boot boot id                     results, not acked (every reply
        loop main-loop rounds            try goes through Radio.send, so
        last ms since the last           this is tx_retry), received,
             request (None: none)        dropped
        rx   requests heard         ch   Wi-Fi channel
        ign  messages ignored       pm   Wi-Fi power-save mode
        sf   replies given up on    on   1 if Wi-Fi is on
        rc   radio recovery: [step 1 tries, step 1 fixes, step 2 tries,
             step 2 fixes] (see recover)
        rb   1: saved just before a recovery reboot (only then)
        ev   the last 3 recoveries: [uptime s, moving when the reply failed,
             step that fixed it (3 = reboot, 0 = still trying)]
        mv   send tries while something moved, and of them not acked
             (idle = the totals in st minus these)
        rk   idle re-keys (see idle_rekey); only if IDLE_REKEY is on
        wr   1: Wi-Fi was fully reset at startup; 0: not, because WIFI_MARK
             was left by a start that hung; only if START_FULL is on
        tx   transmit power now (dBm); txe the error setting it, if any

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
        # Only when the feature is on (room in one message for the rest).
        if IDLE_REKEY:
            d["rk"] = r.rk
        if START_FULL:
            d["wr"] = r.wr
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
            d["on"] = int(r.wlan.active())
        except Exception:
            pass
        try:
            d["tx"] = r.wlan.config("txpower")
        except Exception:
            pass
        if r.tx_err:
            d["txe"] = r.tx_err[:40]
        if not r.enc:
            d["en"] = 0             # unencrypted (only shown then)
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
          step 3  reboot into radio mode, like "r" (the snapshot says so),
                  only if RECOVER_REBOOT is on. It is off: recovery stops
                  after step 2 and starts again with the next failed reply.
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
        if step > 2 and not RECOVER_REBOOT:
            self.rec = None         # ev keeps step 0: not fixed
            return
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

    def idle_rekey(self):
        """While nothing is heard from the remote and nothing moves, add it as
        a peer again every REKEY_IDLE_MS. Called from the main loop.

        If the remote restarted while the arm stayed on, the arm's radio can
        go on acking its messages without delivering them; without this only
        switching the arm off and on would cure it. During a move the remote
        checks in every 1.5 s, so this never runs then; nor while a recovery
        step is waiting to hear the remote.
        """
        if not IDLE_REKEY or self.rec is not None or self.moving():
            return
        r = self.radio
        now = time.ticks_ms()
        if time.ticks_diff(now, self.next_rekey) < 0:
            return
        if r.last_rx is not None and \
                time.ticks_diff(now, r.last_rx) < REKEY_IDLE_MS:
            self.next_rekey = time.ticks_add(r.last_rx, REKEY_IDLE_MS)
            return
        self.next_rekey = time.ticks_add(now, REKEY_IDLE_MS)
        try:
            r.rekey()
        except Exception as e:
            self.error = "rekey: %s: %s" % (type(e).__name__, e)

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


class RadioTest:
    """Radio test mode: only the radio runs (no arm library, no servo timer,
    no diagnostics writes, no recovery), to tell the radio and its
    surroundings apart from the rest of radio mode.

    The remote (arm_bridge.Bridge.rtest) sends numbered test packets,
    b"T" + 4-byte sequence number; each one from the paired remote is
    counted and, with echo, sent straight back. Anything else is counted as
    ignored. Result (TEST_RESULT, see result()):
        s, e   seconds, echo on
        rx     test packets received; per: received in each second
        lost   missed, by sequence number (sent by the remote, never heard)
        ok/nak echoes acked / not acked by the remote's radio
        ign    other messages
        first  ms from the start to the first test packet
        gap    longest time without a test packet: [ms, start ms]
        id     the id "t" returned for this test
        w0     wifi_info() as the test started
    """

    def __init__(self, radio, seconds, echo=1):
        self.r = radio
        self.seconds = seconds
        self.echo = echo
        self.start = time.ticks_ms()
        self.per = [0] * seconds
        self.res = {"s": seconds, "e": echo, "rx": 0, "lost": 0, "ok": 0,
                    "nak": 0, "ign": 0, "first": None, "gap": [0, 0]}
        self.last = None
        self.seq = None

    def done(self):
        return time.ticks_diff(time.ticks_ms(), self.start) >= self.seconds * 1000

    def step(self, timeout_ms):
        """Take one message, if one comes within timeout_ms."""
        r, res = self.r, self.res
        mac, msg = r.e.irecv(timeout_ms)
        if not msg:
            return
        now = time.ticks_ms()
        if mac != r.peer or msg[0] != 0x54 or len(msg) < 5:     # 'T'
            res["ign"] += 1
            return
        t = time.ticks_diff(now, self.start)
        res["rx"] += 1
        if t // 1000 < self.seconds:
            self.per[t // 1000] += 1
        seq = int.from_bytes(bytes(msg[1:5]), "big")
        if self.seq is not None and seq > self.seq + 1:
            res["lost"] += seq - self.seq - 1
        self.seq = seq
        if self.last is None:
            res["first"] = t
        else:
            gap = time.ticks_diff(now, self.last)
            if gap > res["gap"][0]:
                res["gap"] = [gap, time.ticks_diff(self.last, self.start)]
        self.last = now
        if self.echo:
            try:
                ok = r.e.send(r.peer, msg, True)
            except OSError:
                ok = False
            res["ok" if ok else "nak"] += 1

    def result(self):
        res = dict(self.res)
        if self.last is not None:          # a silent end is a gap too
            at = time.ticks_diff(self.last, self.start)
            if self.seconds * 1000 - at > res["gap"][0]:
                res["gap"] = [self.seconds * 1000 - at, at]
        res["per"] = self.per
        return res

    def run(self):
        """Run for the whole time, then save the result."""
        while not self.done():
            self.step(50)
        with open(TEST_RESULT, "w") as f:
            f.write(_dumps(self.result()))


def _test_start(cfg, rst, on_ready):
    """At start-up: if TEST_FILE asks for a radio test, run it and restart
    into radio mode. Returns (without testing) after a power-on or if there
    is no test to run. TEST_FILE goes first, so this runs once at most."""
    text = _read(TEST_FILE)
    if text is None:
        return
    _remove(TEST_FILE)
    try:
        test = json.loads(text)
        seconds, echo = int(test["s"]), test.get("e", 1)
    except (ValueError, KeyError, TypeError):
        return
    if rst == machine.PWRON_RESET or not 1 <= seconds <= TEST_MAX_S:
        return
    if _exists(TEST_RESULT):        # keep the last result as the one before
        _remove(TEST_PREV)
        try:
            os.rename(TEST_RESULT, TEST_PREV)
        except OSError:
            pass
    radio = Radio(cfg)
    if on_ready:
        on_ready()
    _led((16, 0, 16))          # dim purple: radio test mode
    print("[arm] radio test mode for %d s, then radio mode" % seconds)
    try:
        tester = RadioTest(radio, seconds, echo)
        tester.res["id"] = test.get("id")
        tester.res["w0"] = wifi_info()
        tester.run()
    finally:
        _led((0, 0, 0))
        time.sleep_ms(100)
        machine.reset()


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
            server.idle_rekey()
            server.save_diag()
            server.mem_sample()
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
    _test_start(cfg, rst, on_ready)     # returns unless a test was asked for
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
