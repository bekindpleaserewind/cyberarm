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
    "center_base": (BASE, BASE),
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
        self.wlan.config(channel=int(cfg["channel"]))
        self.e = espnow.ESPNow()
        self.e.active(True)
        self.e.set_pmk(binascii.unhexlify(cfg["pmk"]))
        self.peer = binascii.unhexlify(cfg["peer"].replace(":", ""))
        lmk = binascii.unhexlify(cfg["lmk"])
        try:
            self.e.add_peer(self.peer, lmk)
        except OSError:
            # Already added by an earlier start attempt in this boot.
            self.e.mod_peer(self.peer, lmk)

    def recv(self, timeout_ms):
        """The next message from the peer as bytes, or None."""
        mac, msg = self.e.irecv(timeout_ms)
        if msg and mac == self.peer and msg[0] == 0x5B:   # '['
            return bytes(msg)
        return None

    def send(self, reply):
        data = json.dumps(reply)
        if len(data) > MAX_MSG:
            data = json.dumps([reply[0], "E", "reply too long"])
        for _ in range(3):
            try:
                if self.e.send(self.peer, data, True):
                    return True
            except OSError:
                pass
            time.sleep_ms(5)
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
            if len(json.dumps(s)) <= STATUS_MAX:
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
        raise ValueError("unknown op %r" % op)

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
            msg = radio.recv(10)
            if msg:
                server.handle(msg)
            server.poll()
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


def run(lib="cyberbrick_arm", on_ready=None):
    """Start radio mode. Returns only after Ctrl-C (from a USB session).

    lib: the arm library module to use (the boot hook passes the old copy if
    the new one failed). on_ready: called once the radio and arm are up.
    """
    if "/bbl" not in sys.path:
        sys.path.append("/bbl")
    with open(RADIO_FILE) as f:
        cfg = json.load(f)
    radio = Radio(cfg)
    gc.collect()
    arm = __import__(lib).CyberBrickArm()
    gc.collect()
    try:
        rst = machine.reset_cause()
    except Exception:
        rst = None
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
