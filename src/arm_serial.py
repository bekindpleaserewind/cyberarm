#!/usr/bin/env python3
"""
Control the CyberBrick robotic arm from a PC, over the radio or over USB.

Radio (after a one-time setup):
    PC --USB--> CyberBrick remote (arm_bridge.py) --ESP-NOW--> arm (arm_radio.py)
The arm boots into radio mode by itself (arm_boot.py), so its USB cable is
only needed once, for 'radio-install'. Each command is sent as a small
request (an arm method name and its arguments) and the arm replies with its
status when the move is done. See arm_radio.py for the messages.

USB (as before): the PC connects to the arm's receiver board, stops the
firmware that is running, switches MicroPython into raw REPL mode, loads the
arm library (cyberbrick_arm.py) and sends each command as MicroPython code.

Serial handling follows unbug/CyberClaw's cyberbrick_driver:
  * port from --port, then $CYBERBRICK_PORT ($CYBERBRICK_REMOTE_PORT for the
    remote), then the board's MAC (its USB serial number), a cached port, or
    auto-detection (ESP32-C3 USB VID 0x303A / PID 0x1001 first)
  * 115200 baud, Ctrl-C to break into the REPL, DTR/RTS hard reset as fallback
  * code is sent through the raw REPL (Ctrl-A ... Ctrl-D)

Setup
-----
    pip install -r requirements.txt           # pyserial, mpy-cross
    python arm_serial.py radio-pair           # USB in the remote
    python arm_serial.py radio-install        # USB in the arm, once
    (or, USB only: python arm_serial.py install)

Examples
--------
    python arm_serial.py ping                 # check the radio link
    python arm_serial.py status
    python arm_serial.py home
    python arm_serial.py lower 120            # S1 to position 120 (0 = home, 180 = extended)
    python arm_serial.py upper 60 10          # S2 to position 60 at speed 10
    python arm_serial.py move 110 70          # both joints together
    python arm_serial.py claw open            # S3: open / close / 0.0..1.0
    python arm_serial.py turn -45             # base: face 45 deg left of centre
    python arm_serial.py turn-by 30           # base: 30 deg further right
    python arm_serial.py --json status        # one JSON object on stdout
    python arm_serial.py demo
    python arm_serial.py shell                # interactive, one connection
    python arm_serial.py update               # radio: send edited board code
    python arm_serial.py remote               # radio: hand over to the sticks

The arm code runs on the receiver board that is wired to the servos and motor
(Receiver Shield with S1/S2/S3/M2), because it drives that board's own pins.

The base never turns more than 90 degrees either side of centre. Its angle and
the claw opening are estimates; they are saved in .arm_state.json after every
command so they survive a board reboot. If the base was moved some other way
(by hand or with the remote), centre it and run 'base-set 0'.
"""

import argparse
import ast
import base64
import collections
import contextlib
import hashlib
import json
import os
import re
import shlex
import subprocess
import sys
import tempfile
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial is required: pip install pyserial")


HERE = os.path.dirname(os.path.abspath(__file__))
DEVICE_LIB = os.path.join(HERE, "cyberbrick_arm.py")
PORT_CACHE = os.path.join(HERE, ".port_cache")
STATE_FILE = os.path.join(HERE, ".arm_state.json")
BAUD = 115200
ESP32_C3_USB = (0x303A, 0x1001)

RAW_BANNER = b"raw REPL; CTRL-B to exit\r\n>"
CHUNK_BYTES = 256      # write size when sending code to the board
WRITE_BYTES = 512      # file bytes per step when installing

# Where 'install' saves the compiled arm library on the board.
INSTALL_MODULE = "cyberbrick_arm"
INSTALL_PATH = "/cyberbrick_arm.mpy"
INSTALL_VERSION = "/cyberbrick_arm.ver"

# Radio mode (see arm_radio.py, arm_bridge.py, arm_boot.py).
PAIR_FILE = os.path.join(HERE, ".radio_pair.json")   # MACs and link keys
RADIO_LIB = os.path.join(HERE, "arm_radio.py")
BRIDGE_LIB = os.path.join(HERE, "arm_bridge.py")
BOOT_LIB = os.path.join(HERE, "arm_boot.py")
RADIO_PATH, RADIO_VERSION = "/arm_radio.mpy", "/arm_radio.ver"
BRIDGE_PATH = "/arm_bridge.mpy"
BOOT_PATH = "/lib/rc_main.mpy"       # found before the stock /app/rc_main.py
RADIO_CONFIG = "/arm_radio.json"
MODE_FILE, COUNT_FILE = "/arm_mode", "/arm_boot"
RADIO_CHANNEL = 1
RADIO_CHUNK = 150                    # file bytes per radio message (250 max)


class DeviceError(Exception):
    """The code sent to the board raised an exception there."""


class UsageError(Exception):
    """A command was typed with the wrong arguments."""


class WrongBoardError(Exception):
    """Connected to a board that cannot drive the arm."""


# =============================================================================
# Serial link + raw REPL
# =============================================================================

def _same_mac(a, b):
    norm = lambda m: (m or "").replace(":", "").replace("-", "").lower()
    return bool(a and b) and norm(a) == norm(b)


class CyberBrickSerial:
    """USB serial link to one board.

    want_mac picks the board by its MAC (the ESP32-C3 reports it as the USB
    serial number); avoid_mac keeps auto-detection away from a board, so the
    PC never interrupts the paired remote or the arm by mistake.
    """

    def __init__(self, port=None, baud=BAUD, verbose=False, want_mac=None,
                 avoid_mac=None, name="CyberBrick board",
                 env_var="CYBERBRICK_PORT", use_cache=True):
        self.port = port
        self.fixed_port = bool(port)
        self.baud = baud
        self.verbose = verbose
        self.want_mac = want_mac
        self.avoid_mac = avoid_mac
        self.name = name
        self.env_var = env_var
        self.use_cache = use_cache
        self.serial = None
        self._rx = b""

    def _log(self, *args):
        if self.verbose:
            print("[serial]", *args)

    # ---------------------------------------------------------------- connect

    def _resolve_port(self):
        if self.port:
            return self.port

        port = os.environ.get(self.env_var)
        if port:
            print(f"Using port from {self.env_var}: {port}")
            return port

        ports = list(serial.tools.list_ports.comports())
        if self.want_mac:
            for p in ports:
                if _same_mac(p.serial_number, self.want_mac):
                    print(f"Found the {self.name} at {p.device}")
                    return p.device
            return None
        avoid = {p.device for p in ports if _same_mac(p.serial_number, self.avoid_mac)}

        if self.use_cache and os.path.exists(PORT_CACHE):
            try:
                with open(PORT_CACHE) as f:
                    cached = f.read().strip()
                available = {p.device for p in ports}
                if cached not in avoid and (cached in available or os.path.exists(cached)):
                    print(f"Using cached port: {cached}")
                    return cached
            except OSError:
                pass

        print(f"Auto-detecting the {self.name}...")
        candidates = []
        for p in ports:
            if p.device in avoid:
                continue
            if (p.vid, p.pid) == ESP32_C3_USB:
                candidates.insert(0, p)
            elif any(s in (p.description or "") for s in ("CyberBrick", "USB Serial", "CP210")) \
                    or "usbmodem" in p.device:
                candidates.append(p)
        if candidates:
            port = candidates[0].device
            print(f"Found: {port} ({candidates[0].description})")
            if self.use_cache:
                try:
                    with open(PORT_CACHE, "w") as f:
                        f.write(port)
                except OSError:
                    pass
            return port
        return None

    def _print_ports(self):
        print(f"Could not find the {self.name}"
              + (f" (MAC {self.want_mac})" if self.want_mac else "")
              + ". Specify the port with:")
        print("  --port COM5            (Windows)")
        print("  --port /dev/ttyACM0    (Linux)")
        print(f"  or set {self.env_var}")
        print("\nAvailable ports:")
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print("  (none)")
        for p in ports:
            vid = hex(p.vid) if p.vid else "N/A"
            pid = hex(p.pid) if p.pid else "N/A"
            print(f"  - {p.device} ({p.description}) VID={vid} PID={pid}")

    def connect(self):
        self.port = self._resolve_port()
        if not self.port:
            self._print_ports()
            return False

        print(f"Connecting to {self.port}...")
        try:
            self.serial = serial.Serial(self.port, self.baud, timeout=0.1)
        except (serial.SerialException, OSError) as e:
            print(f"Connection failed: {e}")
            return False

        if self.enter_raw_repl():
            print("Connected.")
            return True

        print("No response, resetting the board...")
        if self._hard_reset() and self.enter_raw_repl():
            print("Connected after reset.")
            return True

        print("Could not reach the MicroPython REPL on", self.port)
        return False

    def _hard_reset(self):
        """Toggle DTR/RTS to reboot the board (same sequence as CyberClaw)."""
        try:
            self.serial.dtr = False
            self.serial.rts = False
            time.sleep(0.1)
            self.serial.dtr = True
            self.serial.rts = True
        except (serial.SerialException, OSError) as e:
            self._log("hard reset not supported:", e)
            return False
        time.sleep(2.0)
        return True

    def close(self):
        if self.serial:
            self.serial.close()
            self.serial = None

    # --------------------------------------------------------------- raw REPL

    def _drain(self, settle=0.2):
        """Throw away whatever the board is printing right now."""
        time.sleep(settle)
        while self.serial.in_waiting:
            self.serial.read(self.serial.in_waiting)
            time.sleep(0.05)
        self._rx = b""

    def enter_raw_repl(self):
        """Stop any running program and switch to raw REPL. Returns True on success."""
        # Ctrl-C a few times to stop the firmware's main loop (like CyberClaw).
        self.serial.write(b"\r")
        for _ in range(5):
            self.serial.write(b"\x03")
            time.sleep(0.05)
        self._drain()
        self.serial.write(b"\r\x01")   # Ctrl-A: raw REPL
        try:
            self._read_until(RAW_BANNER, timeout=2)
            return True
        except TimeoutError:
            return False

    def _read_until(self, ending, timeout, echo=False):
        """Read until `ending` and return everything before it."""
        deadline = time.monotonic() + timeout
        shown = 0
        while True:
            idx = self._rx.find(ending)
            if echo:
                stop = idx if idx >= 0 else max(shown, len(self._rx) - len(ending))
                if stop > shown:
                    sys.stdout.write(self._rx[shown:stop].decode("utf-8", "replace"))
                    sys.stdout.flush()
                    shown = stop
            if idx >= 0:
                data = self._rx[:idx]
                self._rx = self._rx[idx + len(ending):]
                return data
            if time.monotonic() > deadline:
                raise TimeoutError(f"timed out waiting for {ending!r}")
            chunk = self.serial.read(max(1, self.serial.in_waiting))
            if chunk:
                self._rx += chunk

    def exec(self, code, timeout=30, echo=False):
        """Run MicroPython `code` on the board and return what it printed.

        Raises DeviceError if the code raised an exception on the board.
        """
        data = code.encode("utf-8")
        for i in range(0, len(data), CHUNK_BYTES):
            self.serial.write(data[i:i + CHUNK_BYTES])
            time.sleep(0.01)
        self.serial.write(b"\x04")   # Ctrl-D: execute

        try:
            self._read_until(b"OK", timeout=3)
        except TimeoutError:
            raise ConnectionError("board did not accept the code (not in raw REPL?)")

        out = self._read_until(b"\x04", timeout, echo=echo)
        err = self._read_until(b"\x04", timeout=5)
        self._read_until(b">", timeout=2)
        if err:
            raise DeviceError(err.decode("utf-8", "replace").strip())
        return out.decode("utf-8", "replace")

    def interrupt(self):
        """Ctrl-C the code running on the board and get back to raw REPL."""
        self.serial.write(b"\x03")
        self._drain(0.3)
        return self.enter_raw_repl()

    def send_and_reboot(self):
        """Hard-reset the board from code; it reboots into its normal firmware."""
        self.serial.write(b"import machine\nmachine.reset()\x04")
        time.sleep(0.5)


# =============================================================================
# Arm session
# =============================================================================

def source_version(path=DEVICE_LIB):
    """Short hash of a local board file, used to spot stale installs."""
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()[:16]


def _is_serial_only(node):
    """True for top-level code only needed when the library runs standalone
    with the handheld remote (run_rc, RC_* settings, MODE, main)."""
    if isinstance(node, ast.If):                      # if __name__ == "__main__"
        return True
    if isinstance(node, ast.FunctionDef):
        return node.name in SKIP_FUNCTIONS
    if isinstance(node, ast.Assign):
        names = [getattr(t, "id", "") for t in node.targets]
        return any(n == "MODE" or n.startswith("RC_") for n in names)
    return False


def library_tree(path):
    """The board library as needed over serial: no docstrings, no remote code."""
    with open(path, encoding="utf-8") as f:
        tree = ast.parse(f.read())
    _strip_docstrings(tree)
    tree.body = [n for n in tree.body if not _is_serial_only(n)]
    return tree


def compile_library(board_mpy):
    """Compile the board library to .mpy bytecode for a board.

    The board loads .mpy files without compiling them, which needs far less
    RAM than importing a .py file. -O3 leaves out line numbers to make the
    file smaller (errors from the board then show no line number).
    `board_mpy` is sys.implementation._mpy from the board; the compiled file
    must use the same .mpy version.
    """
    return compile_source(ast.unparse(library_tree(DEVICE_LIB)), DEVICE_LIB, board_mpy)


def compile_file(path, board_mpy):
    """Compile another board file (radio, bridge, boot hook) to .mpy."""
    with open(path, encoding="utf-8") as f:
        tree = ast.parse(f.read())
    _strip_docstrings(tree)
    return compile_source(ast.unparse(tree), path, board_mpy)


def compile_source(source, path, board_mpy):
    try:
        import mpy_cross
    except ImportError:
        raise UsageError("installing needs mpy-cross: "
                         "pip install mpy-cross==1.23.0.post2")

    name = os.path.splitext(os.path.basename(path))[0]
    with tempfile.TemporaryDirectory() as tmp:
        src_path = os.path.join(tmp, name + ".py")
        out_path = os.path.join(tmp, name + ".mpy")
        with open(src_path, "w", encoding="utf-8") as f:
            f.write(source)
        proc = mpy_cross.run(src_path, "-O3", "-o", out_path,
                             "-s", os.path.basename(path),
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        _, err = proc.communicate()
        if proc.returncode != 0:
            raise UsageError("mpy-cross failed:\n" + err.decode(errors="replace"))
        with open(out_path, "rb") as f:
            data = f.read()

    # .mpy header: b'M', major version, feature flags, small-int bits.
    # Plain bytecode (no native code) only has to match the major version.
    if data[:1] != b"M" or data[1] != board_mpy & 0xFF:
        raise UsageError(
            f"mpy-cross makes .mpy v{data[1]} but the board needs "
            f"v{board_mpy & 0xFF}. Install the mpy-cross version that matches "
            "the board's MicroPython (1.23 -> pip install mpy-cross==1.23.0.post2).")
    return data


# Board-library functions only used when it runs standalone with the RC
# remote; not needed over serial, so they are not sent to the board.
SKIP_FUNCTIONS = {"_stick", "run_rc", "rc_monitor", "main"}

# The stock firmware already has Bambu's bbl servo/motor library loaded, so
# the arm library reuses it for free. If it is missing (board interrupted
# during start-up), importing it would compile it from source, which does not
# fit in the RAM left beside the stock firmware.
# In radio mode (interrupted over USB) the two files were loaded as plain
# 'servos' / 'motors' from /bbl instead; that works just as well.
PREP_CODE = """\
import gc, sys
gc.collect()
_m = sys.modules
print(gc.mem_free(), int(('bbl.servos' in _m and 'bbl.motors' in _m)
                         or ('servos' in _m and 'motors' in _m)))
del _m
"""


# 'probe': measure the board's free RAM and flash and what the radio and boot
# setup look like. Read-only: nothing is written and nothing moves.
PROBE_CODE = """\
import gc, sys, os, json, machine
gc.collect()
_r = {'mem_free': gc.mem_free(), 'mem_alloc': gc.mem_alloc()}
try:
 import esp32
 _r['idf_heap'] = esp32.idf_heap_info(esp32.HEAP_DATA)
except Exception as _e:
 _r['idf_heap'] = repr(_e)
_s = os.statvfs('/')
_r['flash_total'] = _s[0] * _s[2]
_r['flash_free'] = _s[0] * _s[3]
_r['impl'] = [sys.implementation.name, list(sys.implementation.version),
              sys.implementation._mpy, getattr(sys.implementation, '_machine', '')]
_r['role'] = 'transmitter' if machine.Pin(10, machine.Pin.IN).value() else 'receiver'
_r['modules'] = sorted(sys.modules.keys())
_r['path'] = list(sys.path)
_files = []
def _walk(p):
 for _e in os.ilistdir(p):
  _f = (p.rstrip('/') + '/' + _e[0])
  if _e[1] == 0x4000:
   _walk(_f)
  else:
   _files.append([_f, _e[3] if len(_e) > 3 else -1])
_walk('/')
_r['files'] = _files
try:
 import espnow
 _r['espnow'] = True
except ImportError:
 _r['espnow'] = False
try:
 import network, binascii
 _w = network.WLAN(network.STA_IF)
 _r['mac'] = binascii.hexlify(_w.config('mac'), ':').decode()
 _r['wlan_active'] = _w.active()
 try:
  _r['channel'] = _w.config('channel')
 except Exception as _e:
  _r['channel'] = repr(_e)
except Exception as _e:
 _r['mac'] = repr(_e)
for _m in ('bbl', 'bbl.servos', 'bbl.motors', 'rc_module'):
 _x = sys.modules.get(_m)
 if _x is not None:
  _r['file_' + _m] = getattr(_x, '__file__', 'frozen/builtin')
print(json.dumps(_r))
"""

PROBE_FILES = ("/boot.py", "/main.py")


def _strip_docstrings(tree):
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef,
                             ast.FunctionDef, ast.AsyncFunctionDef)):
            body = node.body
            if (body and isinstance(body[0], ast.Expr)
                    and isinstance(body[0].value, ast.Constant)
                    and isinstance(body[0].value.value, str)):
                node.body = body[1:] or [ast.Pass()]


def _code(node):
    """Source for one statement, without comments, indented one space per level."""
    lines = ast.unparse(node).splitlines()
    out = []
    for line in lines:
        stripped = line.lstrip(" ")
        out.append(" " * ((len(line) - len(stripped)) // 4) + stripped)
    return "\n".join(out)


def board_chunks(path, max_chars=1500):
    """Split the board library into small pieces to run one after another.

    The board compiles each piece separately, so peak RAM use depends on the
    largest piece rather than the whole file. Classes are sent as an empty
    class followed by one method at a time.
    """
    pieces = []
    for node in library_tree(path).body:
        if isinstance(node, ast.ClassDef):
            methods = [n for n in node.body
                       if isinstance(n, ast.FunctionDef) and not n.decorator_list]
            others = [n for n in node.body if n not in methods] or [ast.Pass()]
            shell_class = ast.ClassDef(
                name=node.name, bases=node.bases, keywords=node.keywords,
                body=others, decorator_list=node.decorator_list)
            if "type_params" in ast.ClassDef._fields:   # Python 3.12+
                shell_class.type_params = []
            pieces.append(_code(shell_class))
            for m in methods:
                pieces.append(f"{_code(m)}\n{node.name}.{m.name} = {m.name}\n"
                              f"del {m.name}")
            continue
        pieces.append(_code(node))

    chunks, current = [], ""
    for piece in pieces:
        if current and len(current) + len(piece) > max_chars:
            chunks.append(current)
            current = ""
        current += piece + "\n"
    if current:
        chunks.append(current)
    return chunks


# -----------------------------------------------------------------------------
# State kept on the PC
# -----------------------------------------------------------------------------

def _read_state():
    """The saved base angle and claw estimate (and radio boot id), or None."""
    try:
        with open(STATE_FILE) as f:
            state = json.load(f)
        return {"base_angle": float(state["base_angle"]),
                "claw": float(state["claw"]),
                "boot": state.get("boot")}
    except (OSError, KeyError, ValueError, TypeError):
        return None


def _write_state(status, boot=None):
    with open(STATE_FILE, "w") as f:
        json.dump({"base_angle": status["base_angle"],
                   "claw": status["claw"], "boot": boot}, f)


def load_pair():
    """The radio pairing (.radio_pair.json), or {} before radio-pair."""
    try:
        with open(PAIR_FILE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save_pair(pair):
    with open(PAIR_FILE, "w") as f:
        json.dump(pair, f, indent=1)
    with contextlib.suppress(OSError):
        os.chmod(PAIR_FILE, 0o600)    # it holds the radio link keys


def _source_cal_keys():
    """CAL_KEYS from cyberbrick_arm.py: the calibration values the arm keeps."""
    with open(DEVICE_LIB, encoding="utf-8") as f:
        tree = ast.parse(f.read())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                getattr(t, "id", "") == "CAL_KEYS" for t in node.targets):
            return list(ast.literal_eval(node.value))
    return []


def get_calibration(session, keys=None):
    """The calibration values the arm uses, fetched a few at a time so each
    radio reply fits in one message."""
    keys = list(keys or _source_cal_keys())
    cal = {}
    for i in range(0, len(keys), 4):
        cal.update(session.call("calibration", keys[i:i + 4], timeout=5) or {})
    return cal


ARM_KEYS = ("LOWER_ARM_MIN", "LOWER_ARM_MAX", "LOWER_ARM_HOME", "LOWER_ARM_DIRECTION",
            "UPPER_ARM_MIN", "UPPER_ARM_MAX", "UPPER_ARM_HOME", "UPPER_ARM_DIRECTION")


def _source_constant(name, default):
    with open(DEVICE_LIB, encoding="utf-8") as f:
        m = re.search(rf"^{name} = (-?\d+)", f.read(), re.M)
    return int(m.group(1)) if m else default


MAC_CODE = ("import network, binascii\n"
            "print(binascii.hexlify(network.WLAN(network.STA_IF).config('mac'), ':')"
            ".decode())")
ROLE_CODE = "import machine\nprint(machine.Pin(10, machine.Pin.IN).value())"


# =============================================================================
# Arm session over USB
# =============================================================================

class ArmSession:
    """Keeps the arm library loaded on the board and runs arm commands (USB)."""

    transport = "usb"

    def __init__(self, link):
        self.link = link

    def arm_loaded(self):
        return self.link.exec("print('arm' in globals())").strip() == "True"

    def check_receiver(self):
        """Refuse to run on a transmitter.

        The stock firmware picks its role from GPIO10: high = transmitter,
        low = receiver. The arm code drives this board's own pins, and on a
        transmitter those pins are the joystick inputs, so the receiver would
        see fake stick movements and move the wrong parts of the arm.
        """
        if self.link.exec(ROLE_CODE).strip() == "1":
            raise WrongBoardError(
                "This board is a CyberBrick transmitter (GPIO10 is high).\n"
                "The arm code must run on the receiver board that the servos\n"
                "and motor are plugged into. Connect USB to that board instead.")

    def status(self):
        """The arm's status() as a dict."""
        out = self.link.exec("import json\nprint(json.dumps(arm.status()))")
        return json.loads(out.strip().splitlines()[-1])

    def save_state(self, status=None):
        """Remember the base angle and claw opening on the PC.

        The board forgets them when it reboots; load() puts them back.
        """
        try:
            status = status or self.status()
            _write_state(status)
        except (OSError, KeyError, ValueError, DeviceError,
                ConnectionError, TimeoutError) as e:
            print(f"Could not save the arm state: {e}")
        return status

    def _restore_state(self):
        state = _read_state()
        if state is None:
            return
        base, claw = state["base_angle"], state["claw"]
        self.link.exec(f"arm.set_base_angle({base!r})\narm.calibrate_claw({claw!r})")
        print(f"Restored last known base angle {base:g} deg and claw {claw:g} "
              f"from {os.path.basename(STATE_FILE)}.")
        if base:
            print("If the base was moved by hand or with the remote since, "
                  "centre it and run 'base-set 0'.")

    def _prepare(self):
        """Stop any previous arm and check the stock bbl library is loaded."""
        if self.arm_loaded():
            self.save_state()
            # Remove it too, so the next command creates a fresh arm instead
            # of reusing this stopped one.
            self.link.exec("arm.shutdown()\ndel arm")
        free, bbl_loaded = self.link.exec(PREP_CODE, timeout=15).split()
        print(f"Board RAM free: {int(free) // 1024} KB")
        if bbl_loaded != "1":
            raise WrongBoardError(
                "The receiver's stock firmware had not finished starting, so its\n"
                "servo/motor library is not loaded. Run 'reset', wait about 5\n"
                "seconds, then try again.")

    def installed_version(self, path=INSTALL_VERSION):
        """Version tag of a file saved on the board, or None."""
        out = self.link.exec(
            "try:\n"
            f" print(open({path!r}).read().strip())\n"
            "except OSError:\n"
            " print('-')").strip()
        return None if out == "-" else out

    def load(self, force=False):
        """Make `arm` available on the board.

        Uses the copy installed in flash when it matches the local
        cyberbrick_arm.py; otherwise uploads the library into RAM.
        """
        self.check_receiver()
        version = source_version()
        if self.arm_loaded() and not force:
            running = self.link.exec("print(globals().get('_arm_ver'))").strip()
            if running == version:
                return
            print("The arm library running on the board is not your local "
                  "cyberbrick_arm.py; reloading it.")
        self._prepare()

        installed = self.installed_version()
        if installed == version:
            print("Using the arm library installed on the board.")
            self.link.exec(f"import sys\nsys.modules.pop({INSTALL_MODULE!r}, None)\n"
                           f"from {INSTALL_MODULE} import CyberBrickArm, demo",
                           timeout=15)
        else:
            if installed:
                print("The arm library on the board is older than your local "
                      "cyberbrick_arm.py.\nRun 'install' to update it. "
                      "Uploading to RAM for now.")
            else:
                print("Tip: run 'install' once to save the arm library on the "
                      "board for faster, more reliable starts.")
            chunks = board_chunks(DEVICE_LIB)
            size = sum(len(c) for c in chunks)
            print(f"Uploading {os.path.basename(DEVICE_LIB)} "
                  f"({size} bytes in {len(chunks)} pieces)...")
            for chunk in chunks:
                self.link.exec(chunk, timeout=15)

        # Create the arm. This homes S1/S2 and stops S3 and M2. _arm_ver says
        # which cyberbrick_arm.py it came from, so a stale one gets replaced.
        self.link.exec("import gc\ngc.collect()\narm = CyberBrickArm()\n"
                       f"_arm_ver = {version!r}", timeout=30)
        self._restore_state()
        print("Arm ready:", self.status())

    # ------------------------------------------------------------- board files

    def board_mpy(self):
        return int(self.link.exec("import sys\nprint(sys.implementation._mpy)").strip())

    def board_mac(self):
        return self.link.exec(MAC_CODE).strip()

    def write_file(self, path, data):
        """Save bytes as a file on the board: written to PATH.tmp, checked
        against its sha256, then renamed, so a broken copy never replaces a
        good one."""
        tmp = path + ".tmp"
        print(f"Writing {path} ({len(data)} bytes)...")
        self.link.exec(f"_wf = open({tmp!r}, 'wb')")
        for i in range(0, len(data), WRITE_BYTES):
            self.link.exec(f"_wf.write({data[i:i + WRITE_BYTES]!r})")
        self.link.exec("_wf.close()\ndel _wf")

        board_hash = self.link.exec(
            "import hashlib, binascii\n"
            "_h = hashlib.sha256()\n"
            f"with open({tmp!r}, 'rb') as _rf:\n"
            " while True:\n"
            "  _b = _rf.read(512)\n"
            "  if not _b:\n"
            "   break\n"
            "  _h.update(_b)\n"
            "print(binascii.hexlify(_h.digest()).decode())\n"
            "del _h, _rf, _b").strip()
        if board_hash != hashlib.sha256(data).hexdigest():
            self.link.exec(f"import os\nos.remove({tmp!r})")
            raise ConnectionError(f"{path} was corrupted while writing; not changed")

        self.link.exec(
            "import os\n"
            "try:\n"
            f" os.remove({path!r})\n"
            "except OSError:\n"
            " pass\n"
            f"os.rename({tmp!r}, {path!r})")

    def remove_files(self, *paths):
        """Delete files on the board; returns how many existed."""
        out = self.link.exec(
            "import os\n"
            "_n = 0\n"
            f"for _p in {list(paths)!r}:\n"
            " try:\n"
            "  os.remove(_p)\n"
            "  _n += 1\n"
            " except OSError:\n"
            "  pass\n"
            "print(_n)\n"
            "del _n, _p")
        return int(out.strip())

    def mkdir(self, path):
        self.link.exec(f"import os\ntry:\n os.mkdir({path!r})\nexcept OSError:\n pass")

    def check_boot_hook(self):
        """Import the boot hook from /lib to prove the board can load it."""
        self.link.exec(
            "import sys\n"
            "_o = sys.modules.pop('rc_main', None)\n"
            "sys.path.insert(0, '/lib')\n"
            "try:\n"
            " import rc_main as _h\n"
            " assert callable(_h.main)\n"
            "finally:\n"
            " sys.path.pop(0)\n"
            " sys.modules.pop('rc_main', None)\n"
            " if _o is not None:\n"
            "  sys.modules['rc_main'] = _o\n"
            "del _o", timeout=15)

    def install_boot_hook(self, board_mpy):
        self.mkdir("/lib")
        self.write_file(BOOT_PATH, compile_file(BOOT_LIB, board_mpy))
        try:
            self.check_boot_hook()
        except DeviceError:
            # A hook the board cannot load would stop it booting at all.
            self.remove_files(BOOT_PATH)
            raise

    # ---------------------------------------------------------------- install

    def install(self, quiet=False):
        """Compile cyberbrick_arm.py and save it in the board's flash.

        Only adds files; boot.py and the stock firmware are not changed, so
        the receiver still starts in its normal remote-control mode.
        """
        self.check_receiver()
        data = compile_library(self.board_mpy())
        version = source_version()

        self._prepare()
        self.remove_files(INSTALL_VERSION)
        self.write_file(INSTALL_PATH, data)

        # Check it imports (defines code only; nothing moves). Only then write
        # the version tag, so a file the board can't load never counts.
        try:
            self.link.exec(f"import sys\nsys.modules.pop({INSTALL_MODULE!r}, None)\n"
                           f"import {INSTALL_MODULE}\n"
                           f"print({INSTALL_MODULE}.CyberBrickArm)", timeout=15)
        except DeviceError:
            self.remove_files(INSTALL_PATH)
            raise
        self.link.exec(f"with open({INSTALL_VERSION!r}, 'w') as _vf:\n"
                       f" _vf.write({version!r})\n"
                       "del _vf")
        print(f"Installed {INSTALL_PATH} (version {version}).")
        if not quiet:
            print("The receiver still boots into its normal remote-control mode.")

    def install_radio(self, pair):
        """The one-time USB install for radio mode on the arm's receiver.

        Saves the arm library, the radio server, the link settings and the
        boot hook, then reboots the board into radio mode.
        """
        if not pair.get("remote_mac"):
            raise UsageError("Pair the remote first: connect USB to the CyberBrick "
                             "remote and run 'radio-pair'.")
        self.check_receiver()
        board_mpy = self.board_mpy()
        self.install(quiet=True)

        self.write_file(RADIO_PATH, compile_file(RADIO_LIB, board_mpy))
        try:
            self.link.exec("import sys\nsys.modules.pop('arm_radio', None)\n"
                           "import arm_radio\nprint(arm_radio.run)", timeout=15)
        except DeviceError:
            self.remove_files(RADIO_PATH)
            raise
        self.link.exec(f"with open({RADIO_VERSION!r}, 'w') as _vf:\n"
                       f" _vf.write({source_version(RADIO_LIB)!r})\n"
                       "del _vf")
        config = {"peer": pair["remote_mac"], "pmk": pair["pmk"],
                  "lmk": pair["lmk"], "channel": pair["channel"]}
        self.write_file(RADIO_CONFIG, json.dumps(config).encode())
        self.install_boot_hook(board_mpy)
        self.remove_files(MODE_FILE, COUNT_FILE)

        pair["arm_mac"] = self.board_mac()
        pair["arm_mpy"] = board_mpy
        save_pair(pair)
        # The base/claw state was saved when the arm was stopped for the
        # install (_prepare); the arm library is no longer running here.
        print(f"Radio mode installed on the arm ({pair['arm_mac']}).")
        print("Rebooting the arm into radio mode...")
        self.link.send_and_reboot()
        print("Done. You can unplug the arm's USB cable for good. With USB in "
              "the remote, check the link with: python arm_serial.py ping")

    def uninstall(self):
        """Delete the arm library and radio mode from the board's flash."""
        self.check_receiver()
        n = self.remove_files(
            BOOT_PATH, BOOT_PATH + ".tmp", INSTALL_PATH, INSTALL_PATH + ".tmp",
            INSTALL_VERSION, "/cyberbrick_arm_old.mpy", RADIO_PATH,
            RADIO_PATH + ".tmp", RADIO_VERSION, "/arm_radio_old.mpy",
            RADIO_CONFIG, MODE_FILE, COUNT_FILE)
        print("Removed the arm library and radio mode from the board; it boots "
              "into its normal remote-control mode." if n
              else "The arm library was not installed.")

    # --------------------------------------------------------------- commands

    def run(self, code, timeout=60):
        """Run arm code on the board, streaming its output. Ctrl-C stops the arm."""
        try:
            out = self.link.exec(code, timeout=timeout, echo=True)
            if out and not out.endswith("\n"):
                print()
            return out
        except KeyboardInterrupt:
            print("\nInterrupted, stopping the arm...")
            if self.link.interrupt():
                self.link.exec("arm.stop_all()")
                self.save_state()
            raise

    def run_call(self, call, timeout):
        return self.run(usb_code(call), timeout)

    def call(self, method, *args, timeout=60, **kwargs):
        """Call one arm method and return its result (moves wait to finish)."""
        out = self.link.exec(f"import json\nprint(json.dumps(arm.{_params(method, args, kwargs)}))",
                             timeout=timeout)
        return json.loads(out.strip().splitlines()[-1])


# =============================================================================
# Arm session over the radio
# =============================================================================

class RadioError(ConnectionError):
    """The remote could not reach the arm over the radio."""


class RadioSession:
    """Runs arm commands over the radio: PC --USB--> remote --ESP-NOW--> arm.

    The USB link goes to the CyberBrick remote, which runs arm_bridge.py; the
    arm runs arm_radio.py from flash and never needs its USB cable.
    """

    transport = "radio"

    def __init__(self, link, pair):
        self.link = link
        self.pair = pair
        self.last_status = None

    # ------------------------------------------------------------- the remote

    def _check_remote(self):
        if self.link.exec(ROLE_CODE).strip() != "1":
            raise WrongBoardError(
                "Radio commands need USB in the CyberBrick remote (transmitter),\n"
                "but this board is a receiver. Plug the cable into the remote,\n"
                "or use --transport usb to drive the arm over its own USB.")

    def _reboot_remote(self):
        """Reboot the remote and reconnect once it is back."""
        self.link.send_and_reboot()
        self.link.close()
        deadline = time.monotonic() + 15
        time.sleep(2)
        while time.monotonic() < deadline:
            if not self.link.fixed_port:
                self.link.port = None     # it may come back under a new name
            if self.link.connect():
                return
            time.sleep(1)
        raise ConnectionError("the remote did not come back after rebooting")

    def _ensure_bridge(self):
        """Start the bridge on the remote, rebooting it into bridge mode if the
        stock firmware (which owns the radio) is running."""
        state = self.link.exec(
            "import sys, os\n"
            "try:\n"
            f" os.stat({BRIDGE_PATH!r})\n"
            " _i = 1\n"
            "except OSError:\n"
            " _i = 0\n"
            "print(_i, int('rc_module' in sys.modules))\n"
            "del _i").split()
        if state[0] != "1":
            raise UsageError("The bridge is not installed on this remote. Run "
                             "'radio-pair' with USB in the remote.")
        if state[1] == "1":
            print("Switching the remote to bridge mode (it reboots once)...")
            self.link.exec(f"with open({MODE_FILE!r}, 'w') as _f:\n _f.write('bridge')")
            self._reboot_remote()
        p = self.pair
        self.link.exec(f"import arm_bridge\n_b = arm_bridge.get({p['arm_mac']!r}, "
                       f"{p['pmk']!r}, {p['lmk']!r}, {p['channel']!r})", timeout=15)

    # ------------------------------------------------------------- the link

    def request(self, body, timeout_s=10):
        """Send one request to the arm; returns (status, result)."""
        text = json.dumps(body)[1:]          # the bridge adds "[seq, "
        try:
            out = self.link.exec(
                f"print(_b.call({text!r}, {int(timeout_s * 1000) + 3000}))",
                timeout=timeout_s + 15)
        except DeviceError as e:
            raise RadioError(
                f"{str(e).splitlines()[-1]}.\nIs the arm switched on and in radio "
                "mode (it boots into it)? After 'remote' it stays in stick mode "
                "until it is switched off and on again.")
        reply = json.loads(out.strip().splitlines()[-1])
        if reply[1] == "E":
            raise DeviceError(reply[2])
        status = reply[2] if len(reply) > 2 else None
        if status:
            self.last_status = status
        return status, reply[3] if len(reply) > 3 else None

    def ping(self):
        return self.request(["p"], 5)[0]

    def call(self, method, *args, timeout=60, **kwargs):
        """Call one arm method and return its result (moves wait to finish)."""
        return self.request(["c", method, list(args), kwargs, timeout], timeout)[1]

    def reboot(self, mode="radio"):
        """Reboot the arm; for "radio", wait until it answers again."""
        old = (self.last_status or self.ping()).get("boot")
        self.request(["r", mode], 5)
        if mode != "radio":
            return
        print("Waiting for the arm to restart...")
        deadline = time.monotonic() + 25
        time.sleep(2)
        while time.monotonic() < deadline:
            try:
                status = self.ping()
            except (RadioError, DeviceError):
                continue
            if status.get("boot") != old:
                self._restore_state(status)
                return
        raise RadioError("the arm did not answer after rebooting")

    # ---------------------------------------------------------------- session

    def load(self, force=False):
        self._check_remote()
        self._ensure_bridge()
        status = self.ping()
        self._restore_state(status)
        print("Arm ready:", _public(self.last_status))
        if status.get("code"):
            print(f"Warning: the arm is running backup copies ({status['code']}) "
                  "because radio mode did not start with the current files.\n"
                  "Switch the arm off and on; if this stays, run 'update --force'.")

    def arm_loaded(self):
        return True

    def status(self):
        return _public(self.ping())

    def save_state(self, status=None):
        try:
            status = status or self.last_status or self.ping()
            _write_state(status, status.get("boot"))
        except (OSError, KeyError, ValueError, DeviceError,
                ConnectionError, TimeoutError) as e:
            print(f"Could not save the arm state: {e}")
        return status

    def _restore_state(self, status):
        """After the arm rebooted, give it back its base and claw estimates."""
        state = _read_state()
        if state is None:
            self.save_state(status)
            return
        if state["boot"] == status.get("boot"):
            return
        base, claw = state["base_angle"], state["claw"]
        self.call("set_base_angle", base, timeout=5)
        self.call("calibrate_claw", claw, timeout=5)
        self.save_state()
        print(f"The arm restarted; restored base angle {base:g} deg and claw "
              f"{claw:g} from {os.path.basename(STATE_FILE)}.")
        if base:
            print("If the base was moved by hand or with the remote since, "
                  "centre it and run 'base-set 0'.")

    def run_call(self, call, timeout):
        if call.method == "raw":
            raise UsageError("'raw' runs code on the board, so it only works over "
                             "USB (--transport usb with USB in the arm).")
        try:
            if call.method == "demo":
                return self._demo()
            if call.method == "status":
                status = self.ping()
            else:
                status = self.request(["c", call.method, list(call.args),
                                       call.kwargs, timeout], timeout)[0]
        except KeyboardInterrupt:
            print("\nInterrupted, stopping the arm...")
            if self.link.interrupt():
                self.request(["c", "stop_all", [], {}, 10])
                self.save_state()
            raise
        text = repr(_public(status))
        print(text)
        return text + "\n"

    def _demo(self):
        """The board library's demo(), run one step at a time over the radio."""
        lo, up = home_positions(get_calibration(self, ARM_KEYS))
        reach = ("move_arm", lo + 35, up - 25)
        rest = ("move_arm", lo, up)
        steps = [
            "home", ("home",), 0.5,
            "claw open / close", ("open_claw",), 0.3, ("close_claw",), 0.3,
            "lower arm (S1) sweep", ("move_lower_arm", lo + 30),
            ("move_lower_arm", lo - 30), ("move_lower_arm", lo),
            "upper arm (S2) sweep", ("move_upper_arm", up + 30),
            ("move_upper_arm", up - 30), ("move_upper_arm", up),
            "base (M2) left / right", ("turn_base", 45), 0.3, ("turn_base", -45), 0.3,
            ("turn_base", 0),
            "pick and place", ("open_claw",), reach, ("close_claw",), rest,
            ("turn_base", 60), reach, ("open_claw",), rest, ("turn_base", 0),
            ("close_claw",),
        ]
        for step in steps:
            if isinstance(step, str):
                print("[demo]", step)
            elif isinstance(step, float):
                time.sleep(step)
            else:
                self.call(step[0], *step[1:])
        text = f"[demo] done: {_public(self.last_status)}"
        print(text)
        return text + "\n"

    # ---------------------------------------------------------------- updates

    def update(self, force=False):
        """Send changed board files to the arm over the radio, then restart it."""
        board_mpy = self.pair.get("arm_mpy") or int(self.link.exec(
            "import sys\nprint(sys.implementation._mpy)").strip())
        installed = self.request(["v"], 5)[1] or {}
        changed = False
        for name, path, build in (
                (INSTALL_MODULE, DEVICE_LIB, lambda: compile_library(board_mpy)),
                ("arm_radio", RADIO_LIB, lambda: compile_file(RADIO_LIB, board_mpy))):
            version = source_version(path)
            if installed.get(name) == version and not force:
                print(f"{name} on the arm is up to date ({version}).")
                continue
            data = build()
            print(f"Sending {name} ({len(data)} bytes) over the radio...")
            self.request(["fo", name], 5)
            for i in range(0, len(data), RADIO_CHUNK):
                b64 = base64.b64encode(data[i:i + RADIO_CHUNK]).decode()
                self.request(["fw", b64], 5)
            self.request(["fc", hashlib.sha256(data).hexdigest(), version], 10)
            print(f"Saved {name} {version} on the arm (the previous copy is kept "
                  "as a fallback).")
            changed = True
        if changed:
            self.reboot("radio")
            print("Arm restarted with the new code:", _public(self.last_status))

    def hand_to_remote(self):
        """Give control back to the stock remote and its sticks."""
        print("Rebooting the arm into stick mode (until it is switched off and on)...")
        self.request(["r", "stock"], 5)
        self.link.exec(f"import os\ntry:\n os.remove({MODE_FILE!r})\nexcept OSError:\n pass")
        print("Rebooting the remote into its normal firmware...")
        self.link.send_and_reboot()
        print("Done. The sticks drive the arm now. Switch the arm off and on to "
              "go back to radio mode.\nIf you move the base with the sticks, "
              "centre it afterwards and run 'base-set 0'.")

    def report_link(self):
        """Print round-trip time, signal strength and the arm's free RAM."""
        t0 = time.monotonic()
        status = self.ping()
        rtt = (time.monotonic() - t0) * 1000
        rssi = self.link.exec("print(_b.rssi())").strip()
        versions = self.request(["v"], 5)[1] or {}
        print(f"Arm answered in {rtt:.0f} ms (including USB to the remote); "
              f"signal {rssi} dBm.")
        print(f"Arm free RAM: {status.get('mem', 0) // 1024} KB, boot id "
              f"{status.get('boot')}, versions {versions}.")
        print("Last restart:", RESET_CAUSES.get(status.get("rst"), "unknown"))
        if status.get("radio_error"):
            print("Last radio error on the arm:", status["radio_error"])
        if status.get("timer_error"):
            print("Timer error on the arm:", status["timer_error"])
        print("Status:", _public(status))


# machine.reset_cause() on the ESP32-C3, as reported by the arm.
RESET_CAUSES = {
    1: "switched on (or the battery voltage dipped)",
    2: "restarted by a command or an update, or the code crashed",
    3: "watchdog: the arm froze and restarted itself",
    5: "soft reset",
}


def _public(status):
    """status() without the radio-only fields, same shape as over USB."""
    if not status:
        return status
    return {k: v for k, v in status.items()
            if k not in ("boot", "mem", "radio_error", "rst", "code")}


def radio_pair(link, pair, channel=None):
    """Set up the remote as the radio bridge (USB to the remote)."""
    session = ArmSession(link)
    if link.exec(ROLE_CODE).strip() != "1":
        raise WrongBoardError(
            "radio-pair needs USB in the CyberBrick remote (transmitter); this\n"
            "board is a receiver. Plug the cable into the remote.")
    mac = session.board_mac()
    if pair.get("remote_mac") and not _same_mac(pair["remote_mac"], mac):
        print(f"Replacing the paired remote {pair['remote_mac']} with {mac}.")
        if pair.get("arm_mac"):
            print("The arm only accepts its paired remote: run 'radio-install' "
                  "over the arm's USB once more after this.")
            pair.pop("arm_mac", None)
    if not pair.get("pmk"):
        pair["pmk"] = os.urandom(16).hex()
        pair["lmk"] = os.urandom(16).hex()
    pair["channel"] = channel or pair.get("channel") or RADIO_CHANNEL
    pair["remote_mac"] = mac

    board_mpy = session.board_mpy()
    session.write_file(BRIDGE_PATH, compile_file(BRIDGE_LIB, board_mpy))
    link.exec("import sys\nsys.modules.pop('arm_bridge', None)\nimport arm_bridge", timeout=15)
    session.install_boot_hook(board_mpy)
    link.exec(f"with open({MODE_FILE!r}, 'w') as _f:\n _f.write('bridge')")
    save_pair(pair)
    print(f"Remote {mac} is set up as the radio bridge (channel {pair['channel']}).")
    print("Rebooting the remote into bridge mode...")
    link.send_and_reboot()
    if pair.get("arm_mac"):
        print("Done. Try: python arm_serial.py ping")
    else:
        print("Next, once only: plug USB into the ARM's receiver (the remote can stay "
              "plugged in) and run:\n  python arm_serial.py radio-install")


def probe(link):
    """Print the board's free RAM and flash, its files, boot scripts and radio."""
    info = json.loads(link.exec(PROBE_CODE, timeout=20).strip().splitlines()[-1])
    files = info.pop("files")
    modules = info.pop("modules")
    print(json.dumps(info, indent=1))
    print("loaded modules:", " ".join(modules))
    print("files:")
    for name, size in files:
        print(f"  {size:>8}  {name}")
    for path in PROBE_FILES:
        out = link.exec("try:\n"
                        f" print(open({path!r}).read()[:4000])\n"
                        "except OSError:\n"
                        " print('(missing)')").rstrip()
        print(f"----- {path} -----\n{out}")
    print("----- frozen / built-in modules -----")
    print(link.exec("help('modules')", timeout=10).rstrip())


# =============================================================================
# Commands
# =============================================================================

STATUS = "\nprint(arm.status())"

Call = collections.namedtuple("Call", "method args kwargs")


def _call(method, *args, **kwargs):
    return Call(method, args, kwargs)


def _params(method, args, kwargs):
    params = [repr(a) for a in args] + [f"{k}={v!r}" for k, v in kwargs.items()]
    return f"{method}({', '.join(params)})"


def usb_code(call):
    """The MicroPython code that runs a command over USB."""
    if call.method == "status":
        return "print(arm.status())"
    if call.method == "demo":
        return "demo(arm)"
    if call.method == "raw":
        return call.args[0]
    return "arm." + _params(call.method, call.args, call.kwargs) + STATUS


def _num(value, name):
    try:
        return float(value)
    except (TypeError, ValueError):
        raise UsageError(f"{name} must be a number, got {value!r}")


def _opt_speed(args, i):
    return {"speed": _num(args[i], "speed")} if len(args) > i else {}


def _need(args, n, usage):
    if len(args) < n:
        raise UsageError("usage: " + usage)


def cmd_home(a):
    return _call("home", **_opt_speed(a, 0)), 60


def cmd_lower(a):
    _need(a, 1, "lower POSITION [SPEED]   (0 = home .. 180 = fully extended)")
    return _call("move_lower_arm", _num(a[0], "position"), **_opt_speed(a, 1)), 60


def cmd_upper(a):
    _need(a, 1, "upper POSITION [SPEED]   (0 = home .. 180 = fully extended)")
    return _call("move_upper_arm", _num(a[0], "position"), **_opt_speed(a, 1)), 60


def cmd_move(a):
    _need(a, 2, "move LOWER UPPER [SPEED]   (positions, 0 = home .. 180 = fully extended)")
    lower, upper = _num(a[0], "lower position"), _num(a[1], "upper position")
    return _call("move_arm", lower, upper, **_opt_speed(a, 2)), 60


def cmd_claw(a):
    _need(a, 1, "claw open|close|POSITION(0.0-1.0)")
    if a[0] == "open":
        return _call("open_claw"), 30
    if a[0] in ("close", "closed"):
        return _call("close_claw"), 30
    return _call("set_claw", _num(a[0], "position")), 30


def cmd_claw_spin(a):
    _need(a, 1, "claw-spin SPEED   (-100..100, ignores limits; 0 stops)")
    speed = _num(a[0], "speed")
    if speed == 0:
        return _call("claw_stop"), 10
    return _call("claw_run", speed, ignore_limits=True), 10


def cmd_claw_cal(a):
    _need(a, 1, "claw-cal open|closed   (tell the arm the claw's current state)")
    if a[0] not in ("open", "closed", "close"):
        raise UsageError("claw-cal takes 'open' or 'closed'")
    return _call("calibrate_claw", a[0] == "open"), 10


def cmd_turn(a):
    _need(a, 1, "turn ANGLE   (-90 = full left .. 0 = centre .. 90 = full right)")
    return _call("turn_base", _num(a[0], "angle")), 60


def cmd_turn_by(a):
    _need(a, 1, "turn-by DEGREES   (negative = left; stays within 90 of centre)")
    return _call("turn_base_by", _num(a[0], "degrees")), 60


def cmd_base_set(a):
    return _call("set_base_angle", _num(a[0], "angle") if a else 0.0), 10


def cmd_base(a):
    _need(a, 1, "base SPEED [SECONDS]   (-100..100; no SECONDS = keep spinning)")
    speed = _num(a[0], "speed")
    if len(a) > 1:
        secs = _num(a[1], "seconds")
        return _call("rotate_base", speed, int(secs * 1000)), secs + 15
    return _call("base_speed", speed), 10


def cmd_stop(a):
    return _call("stop_all"), 10


def cmd_release(a):
    return _call("release_joints"), 10


def cmd_status(a):
    return _call("status"), 10


def cmd_demo(a):
    return _call("demo"), 300


def cmd_raw(a):
    _need(a, 1, "raw CODE")
    return _call("raw", " ".join(a)), 60


COMMANDS = {
    "home":      (cmd_home,      "home [SPEED]                 joints to home pose, base to centre"),
    "lower":     (cmd_lower,     "lower POS [SPEED]            S1 lower arm, 0 = home .. 180 = extended"),
    "upper":     (cmd_upper,     "upper POS [SPEED]            S2 upper arm, 0 = home .. 180 = extended"),
    "move":      (cmd_move,      "move LOWER UPPER [SPEED]     both joints together"),
    "claw":      (cmd_claw,      "claw open|close|0.0-1.0      S3 claw"),
    "claw-spin": (cmd_claw_spin, "claw-spin SPEED              spin S3 freely (0 stops)"),
    "claw-cal":  (cmd_claw_cal,  "claw-cal open|closed         set claw's known state"),
    "turn":      (cmd_turn,      "turn ANGLE                   base to -90 (left)..90 (right)"),
    "turn-by":   (cmd_turn_by,   "turn-by DEGREES              base relative turn (- = left)"),
    "base-set":  (cmd_base_set,  "base-set [ANGLE]             tell the arm the base angle (0)"),
    "base":      (cmd_base,      "base SPEED [SECONDS]         spin M2 (stops at +-90 deg)"),
    "stop":      (cmd_stop,      "stop                         stop base/claw, hold joints"),
    "release":   (cmd_release,   "release                      let S1/S2 go limp"),
    "status":    (cmd_status,    "status                       print joint/claw/base state"),
    "demo":      (cmd_demo,      "demo                         run the full demo"),
    "raw":       (cmd_raw,       "raw CODE                     run MicroPython on the board (USB only)"),
}
SESSION_COMMANDS = {
    "shell": "shell                        interactive prompt",
    "reset": "reset                        reboot the arm (USB: into stock firmware)",
    "install":   "install                      save arm library on the board (USB)",
    "uninstall": "uninstall                    remove it and radio mode from the board (USB)",
    "claw-calibrate": "claw-calibrate               time a full claw open and close and save them",
    "base-calibrate": "base-calibrate               time 90 deg base turns and save them",
    "base-tune": "base-tune                    set 90 deg base turns by how far timed turns go",
    "arm-calibrate": "arm-calibrate                set each joint's home and fully extended position",
    "arm-align": "arm-align                    re-seat a joint on its servo so it can use the full 0-180",
    "gesture":   "gesture wave|nod|shake [TIMES] [SPEED]  smooth gesture, then back to the start pose",
    "calibration": "calibration                  show the calibration values the arm uses",
    "probe":     "probe                        report free RAM/flash, boot files, radio (read-only)",
    "radio-pair": "radio-pair [CHANNEL]         USB in the remote: set it up as the radio bridge",
    "radio-install": "radio-install                USB in the arm, once: install radio mode",
    "ping":      "ping                         radio: check the link to the arm",
    "update":    "update [--force]             radio: send changed board code to the arm",
    "remote":    "remote                       radio: hand the arm to the stick remote",
}
USB_ONLY = {"install", "uninstall", "probe", "radio-pair", "radio-install"}
RADIO_ONLY = {"ping", "update", "remote"}
CALIBRATIONS = {}   # filled in below: name -> function(session)


def _wait_enter(prompt):
    input(prompt)
    return time.monotonic()


def calibrate_claw(session):
    """Time a full claw open and close and save CLAW_OPEN_MS / CLAW_CLOSE_MS.

    The claw servo has no position sensor, so open/close are timed. This times
    a real open and close at CLAW_SPEED while the user watches and saves the
    results on the board (CAL_FILE in cyberbrick_arm.py).
    """
    speed = float(_source_constant("CLAW_SPEED", 50))

    def spin(direction):
        session.call("claw_run", direction * speed, ignore_limits=True, timeout=5)

    def stop():
        session.call("claw_stop", timeout=5)

    print("Claw calibration. Watch the claw and press Enter at each step.")
    try:
        _wait_enter("1) Press Enter to start CLOSING the claw...")
        spin(-1)
        _wait_enter("   Press Enter the moment the claw STOPS MOVING (fully closed).")
        stop()

        _wait_enter("2) Press Enter to start OPENING the claw (this is timed)...")
        spin(1)
        t0 = time.monotonic()
        t1 = _wait_enter("   Press Enter the moment the claw STOPS MOVING (fully open).")
        stop()
        open_ms = int((t1 - t0) * 1000)

        _wait_enter("3) Press Enter to close it again...")
        spin(-1)
        t0 = time.monotonic()
        t1 = _wait_enter("   Press Enter the moment the claw STOPS MOVING (fully closed).")
        stop()
        close_ms = int((t1 - t0) * 1000)
    except (KeyboardInterrupt, EOFError):
        stop()
        print("\nCalibration cancelled; nothing saved.")
        return

    session.call("calibrate_claw", False, timeout=5)
    print(f"Open took {open_ms} ms, close took {close_ms} ms.")
    if min(open_ms, close_ms) < 200:
        session.save_state()
        print("That is too short to be right; nothing saved.")
        return

    saved = session.call("save_calibration",
                         {"CLAW_OPEN_MS": open_ms, "CLAW_CLOSE_MS": close_ms}, timeout=10)
    session.save_state()
    print(f"Saved on the arm: CLAW_OPEN_MS = {saved['CLAW_OPEN_MS']}, "
          f"CLAW_CLOSE_MS = {saved['CLAW_CLOSE_MS']}.")
    print("The claw is closed and the arm knows it.")


def _end_base_calibration(session):
    """Stop the base and put its 90 degree limit back on after a base
    calibration ends early: Ctrl-C (perhaps in the middle of a radio call,
    which leaves the remote busy) or a lost link. Never raises."""
    try:
        session.link.interrupt()
    except Exception:
        pass
    try:
        session.call("base_stop", timeout=5)
        session.call("set_base_limit", True, timeout=5)
    except Exception as e:
        print(f"Could not put the base's 90 deg limit back on ({e}).\n"
              "Switch the arm off and on before using the base.")


def calibrate_base(session):
    """Time 90 degree base turns and save BASE_RIGHT_MS / BASE_LEFT_MS.

    The base motor has no position sensor, so its angle is worked out from
    how long it has turned. This also checks which way is right and fixes
    BASE_DIRECTION if needed. The 90 degree limit is off while it runs.
    """
    speed = _source_constant("BASE_TURN_SPEED", 40)
    direction = get_calibration(session, ["BASE_DIRECTION"])["BASE_DIRECTION"]

    def spin(sign):
        session.call("base_speed", sign * speed, timeout=5)

    def stop():
        session.call("base_stop", timeout=5)

    session.call("set_base_limit", False, timeout=5)   # no limit while calibrating
    print("Base calibration. Watch the arm and press Enter at each step.")
    finished = False
    try:
        _nudge_base(session, speed, "1) Nudge the base until the arm faces straight ahead:\n")

        _wait_enter("2) Press Enter to start turning (this is timed)...")
        spin(1)
        t0 = time.monotonic()
        t1 = _wait_enter("   Press Enter the moment the arm has turned a quarter "
                         "turn (90 deg).")
        stop()
        first_ms = int((t1 - t0) * 1000)

        while True:
            way = input("   Which way did it turn, [r]ight or [l]eft? "
                        "(as seen from behind the arm) ").strip().lower()
            if way[:1] in ("r", "l"):
                break

        _wait_enter("3) Press Enter to turn back to centre (this is timed)...")
        spin(-1)
        t0 = time.monotonic()
        t1 = _wait_enter("   Press Enter the moment it faces straight ahead again.")
        stop()
        back_ms = int((t1 - t0) * 1000)
        finished = True
    except (KeyboardInterrupt, EOFError):
        print("\nCalibration cancelled; nothing saved.")
    finally:
        if not finished:
            _end_base_calibration(session)
    if not finished:
        return

    if way[0] == "r":
        right_ms, left_ms = first_ms, back_ms
    else:
        # Positive speed turned it left: swap the direction and the timings.
        direction = -direction
        right_ms, left_ms = back_ms, first_ms
    print(f"90 deg right took {right_ms} ms, 90 deg left took {left_ms} ms.")
    if min(right_ms, left_ms) < 200:
        session.call("set_base_limit", True, timeout=5)
        session.call("set_base_angle", 0, timeout=5)
        session.save_state()
        print("That is too short to be right; nothing saved.")
        return
    saved = session.call("save_calibration",
                         {"BASE_RIGHT_MS": right_ms, "BASE_LEFT_MS": left_ms,
                          "BASE_DIRECTION": direction}, timeout=10)
    session.call("set_base_angle", 0, timeout=5)
    session.save_state()
    print("Saved on the arm: " + ", ".join(
        f"{k} = {saved[k]}" for k in ("BASE_RIGHT_MS", "BASE_LEFT_MS", "BASE_DIRECTION")) + ".")
    print("The base is centred and the arm knows it.")


NUDGE_MS = 250      # one base nudge (l or r) while centring


def _nudge_base(session, speed, heading):
    """Let the user nudge the base with l / r until it faces straight ahead.

    "rrr" nudges three times as far in one go. Returns on an empty answer.
    """
    while True:
        key = input(heading + "   type l or r then Enter to nudge (rrr = 3 nudges), "
                    "or just Enter when centred: ").strip().lower()
        heading = ""
        if not key:
            return
        if key[0] in "lr" and key == key[0] * len(key):
            sign = -1 if key[0] == "l" else 1
            ms = NUDGE_MS * len(key)
            session.call("rotate_base", sign * speed, ms, timeout=ms / 1000 + 10)
        else:
            print("   Type l or r (or several, like rrr), or just Enter.")


def _ask_degrees(prompt):
    """How far the user says the base turned: degrees, or None when it was a
    quarter turn (Enter, "ok", or 89-91: nobody judges finer than that)."""
    while True:
        text = input(prompt).strip().lower()
        if text in ("", "ok", "y", "yes", "done"):
            return None
        try:
            degrees = float(text.rstrip("°").removesuffix("deg").strip())
        except ValueError:
            print("   Type how many degrees it turned, e.g. 75, or 90 (or just Enter) "
                  "if it was a quarter turn.")
            continue
        if abs(degrees - 90) <= 1:
            return None
        if 10 <= degrees <= 270:
            return degrees
        print("   Give a number of degrees between 10 and 270.")


def tune_base(session):
    """Set BASE_RIGHT_MS / BASE_LEFT_MS from turns the arm times itself.

    Unlike base-calibrate there is no stopwatch: the arm runs the motor for
    an exact time on its own clock, the user says how far the base turned,
    and the time is scaled to make that 90 degrees. Repeat until it is. The
    turns stop the same way normal turns do, so the motor's start-up and
    coasting are counted just as they are in use. Also checks which way is
    right. The 90 degree limit is off while it runs.
    """
    speed = _source_constant("BASE_TURN_SPEED", 40)
    cal = get_calibration(session, ["BASE_RIGHT_MS", "BASE_DIRECTION"])
    direction = cal["BASE_DIRECTION"]
    right = None        # the motor sign that turns the base right, once known

    def turn(sign, ms):
        session.call("rotate_base", sign * speed, ms, timeout=ms / 1000 + 10)

    def tune(name, sign, ms):
        """Turn `name` from centre for ms, ask how far it went, scale, repeat."""
        nonlocal right
        while True:
            _nudge_base(session, speed, "Nudge the base until it faces straight ahead:\n")
            print(f"   Turning {name} for {ms} ms...")
            turn(sign, ms)
            went_left = False
            if right is None:
                while True:
                    way = input("   Which way did it turn, [r]ight or [l]eft? (as seen "
                                "from behind the arm) ").strip().lower()[:1]
                    if way in ("r", "l"):
                        break
                right = 1 if way == "r" else -1
                went_left = right == -1
            degrees = _ask_degrees("   How many degrees did it turn? (90 or just Enter "
                                   "if it was a quarter turn: that finishes) ")
            turn(-sign, ms)                     # back towards centre
            if went_left:
                # Left and right were swapped; check it again turning right.
                print("   Left and right are swapped on this arm; fixed. "
                      "Next turn goes right.")
                sign = right
                if degrees is None:
                    continue
            elif degrees is None:
                return ms
            new = round(ms * 90 / degrees)
            if new == ms:
                return ms
            if not 100 <= new <= 60000:
                print(f"   That would make 90 deg {new} ms, which can't be right; "
                      f"trying {ms} ms again.")
                continue
            print(f"   {degrees:g} deg in {ms} ms, so 90 deg should take about "
                  f"{new} ms. Let's check.")
            ms = new

    session.call("set_base_limit", False, timeout=5)   # no limit while tuning
    print("Base tuning. The arm times each turn itself; you just say how far it went.\n"
          "Tip: mark straight ahead and 90 deg right and left on the table first.")
    finished = False
    try:
        right_ms = tune("right", 1, cal["BASE_RIGHT_MS"])
        same = input(f"Right turns are set: 90 deg = {right_ms} ms. Use the same time "
                     "for left turns? [Y/n] ").strip().lower()[:1] != "n"
        left_ms = right_ms if same else tune("left", -right, right_ms)
        _nudge_base(session, speed, "Last step: nudge the base to straight ahead.\n")
        finished = True
    except (KeyboardInterrupt, EOFError):
        print("\nTuning cancelled; nothing saved. Centre the base and run "
              "'base-set 0'.")
    finally:
        if not finished:
            _end_base_calibration(session)
    if not finished:
        return

    if right == -1:
        direction = -direction     # positive speed turned it left
    saved = session.call("save_calibration",
                         {"BASE_RIGHT_MS": right_ms, "BASE_LEFT_MS": left_ms,
                          "BASE_DIRECTION": direction}, timeout=10)
    session.call("set_base_angle", 0, timeout=5)
    session.save_state()
    print("Saved on the arm: " + ", ".join(
        f"{k} = {saved[k]}" for k in ("BASE_RIGHT_MS", "BASE_LEFT_MS", "BASE_DIRECTION")) + ".")
    print("The base is centred and the arm knows it. Try 'turn 90' and 'turn -90'.")


# Joint calibration: (key prefix, name, arm method, question after a move
# towards "extended", answer meaning DIRECTION = 1, the other answer, and
# what DIRECTION 1 / -1 mean in words)
JOINT_STEPS = (
    ("LOWER_ARM", "lower arm (S1)", "move_lower_arm",
     "Did the lower arm lean [f]orward or [b]ack?", "f", "b",
     "leans the lower arm forward", "leans the lower arm back"),
    ("UPPER_ARM", "upper arm (S2)", "move_upper_arm",
     "Did the claw go [u]p or [d]own?", "u", "d",
     "lifts the claw", "lowers the claw"),
)
JOINT_CAL_SPEED = 10      # slow, so there is time to react
NUDGE_DEG = 5


def _nudge(session, method, angle, prompt):
    """Let the user step a joint to a position from the keyboard; returns the
    servo angle they accept. (The servos hold their angle and cannot report
    where they are, so the joint cannot be positioned by hand.)"""
    print(prompt)
    while True:
        key = input(f"   [servo {angle:g} deg]  + / - = {NUDGE_DEG} deg (+++ = 3 steps), "
                    "+N / -N, an angle 0-180, Enter = done: ").strip()
        if not key:
            return angle
        try:
            if set(key) <= {"+"} or set(key) <= {"-"}:
                new = angle + len(key) * (NUDGE_DEG if key[0] == "+" else -NUDGE_DEG)
            elif key[0] in "+-":
                new = angle + float(key)
            else:
                new = float(key)
        except ValueError:
            print("   Type +, -, +N, -N or an angle.")
            continue
        new = int(max(0, min(180, round(new))))
        session.call(method, new, speed=JOINT_CAL_SPEED, timeout=30)
        angle = new


def _joint_ends(cal, joint):
    """(home, extended) in servo degrees once a joint is calibrated, else None."""
    lo, hi, home = cal[joint + "_MIN"], cal[joint + "_MAX"], cal[joint + "_HOME"]
    return (home, hi) if home == lo else (home, lo) if home == hi else None


def home_positions(cal):
    """Each joint's home as a position: 0 once calibrated, else servo degrees."""
    return [0 if _joint_ends(cal, joint) else cal[joint + "_HOME"]
            for joint in ("LOWER_ARM", "UPPER_ARM")]


def describe_joints(cal):
    """Plain-words description of the joint calibration."""
    lines = []
    for joint, name, _, _, _, _, plus, minus in JOINT_STEPS:
        ends = _joint_ends(cal, joint)
        if not ends:
            lines.append(f"{name}: not calibrated; positions are servo degrees "
                         f"{cal[joint + '_MIN']}-{cal[joint + '_MAX']}, home "
                         f"{cal[joint + '_HOME']}")
            continue
        d = cal.get(joint + "_DIRECTION", 0)
        way = plus if d == 1 else minus if d == -1 else "goes an unknown way"
        lines.append(f"{name}: 0 = home (servo {ends[0]} deg), 180 = fully extended "
                     f"(servo {ends[1]} deg); extending {way}")
    return lines


def calibrate_arm(session):
    """Set each joint's home and fully extended positions, and save them.

    The joints are servos, so their angles are exact; what the arm does not
    know is which servo angle is "folded at home" and which is "fully
    extended" without hitting the table, the base or the other arm part. The
    user steps each joint to both while watching. From then on the joint is
    driven in positions 0 (home) .. 180 (fully extended), and the results are
    saved on the arm (CAL_FILE).
    """
    cal = get_calibration(session, ARM_KEYS)
    print("Arm calibration. You move each joint from the KEYBOARD, not by hand:\n"
          "the servos hold the arm firmly in place while the battery is on, and pushing\n"
          "them can strip their gears. Type + or - and press Enter to step a joint\n"
          f"{NUDGE_DEG} degrees, +++ for three steps, +20 / -20 for 20 degrees, or a servo\n"
          "angle such as 120. Joints move slowly; keep hands clear. Ctrl-C cancels\n"
          "(joints already saved stay saved).\n"
          "Here you work in servo degrees; afterwards each joint uses positions\n"
          "0 (home) to 180 (fully extended).")
    print("Now: " + "; ".join(describe_joints(cal)) + ".")
    session.call("set_joint_limits", False, timeout=5)   # plain servo degrees, 0-180
    saved = {}
    try:
        session.call("move_arm", cal["LOWER_ARM_HOME"], cal["UPPER_ARM_HOME"],
                     speed=JOINT_CAL_SPEED, timeout=60)
        for joint, name, method, question, plus, minus, plus_text, minus_text in JOINT_STEPS:
            print(f"\n== {name} ==")
            home = _nudge(session, method, cal[joint + "_HOME"],
                          f"1) Using + and -, step the {name} to its HOME position: folded, "
                          "where it rests.")
            ext = _nudge(session, method, home,
                         f"2) Using + and -, step the {name} to FULLY EXTENDED. Stop a few "
                         "degrees before it would touch the table, the base or the other "
                         "part of the arm (if you hear the servo strain, step back).")
            if abs(ext - home) < 10:
                session.call(method, home, speed=JOINT_CAL_SPEED, timeout=30)
                print(f"   Home ({home}) and fully extended ({ext}) must be at least 10 "
                      f"degrees apart; {name} not saved.")
                continue

            session.call(method, home, speed=JOINT_CAL_SPEED, timeout=30)
            test = home + (15 if ext > home else -15)
            _wait_enter(f"3) Press Enter to move the {name} 15 degrees out from home; "
                        "watch which way it goes...")
            session.call(method, test, speed=JOINT_CAL_SPEED, timeout=30)
            while True:
                answer = input("   " + question + " ").strip().lower()[:1]
                if answer in (plus, minus):
                    break
            direction = 1 if answer == plus else -1
            session.call(method, home, speed=JOINT_CAL_SPEED, timeout=30)

            values = {joint + "_MIN": min(home, ext), joint + "_MAX": max(home, ext),
                      joint + "_HOME": home, joint + "_DIRECTION": direction}
            saved.update(session.call("save_calibration", values, timeout=10))
            session.call("set_joint_limits", False, timeout=5)   # saving turned them on
            print(f"   Saved: {name} position 0 = home (servo {home} deg), 180 = fully "
                  f"extended (servo {ext} deg); extending "
                  f"{plus_text if direction == 1 else minus_text}.")
    except (KeyboardInterrupt, EOFError):
        session.link.interrupt()
        print("\nCalibration cancelled." + ("" if saved else " Nothing saved."))
    finally:
        session.call("set_joint_limits", True, timeout=5)
        cal = get_calibration(session, ARM_KEYS)
        session.call("move_arm", *home_positions(cal), speed=JOINT_CAL_SPEED, timeout=60)
        session.save_state()
    if saved:
        print("The arm now uses: " + "; ".join(describe_joints(cal)) + ".")
        print("'python arm_serial.py home' puts the arm back in this home pose.")

def align_joint(session):
    """Re-seat a joint on its servo so home is at one end of the servo's travel.

    A servo turns 180 degrees in total. If an arm part was pressed onto the
    servo shaft while the servo sat mid-way (say 100), the part is folded at
    100 and can only extend 80 degrees before the servo runs out. No setting
    can fix that: the part has to come off, the servo goes to its end (0 or
    180), and the part goes back on folded. Then arm-calibrate sets the ends.
    """
    cal = get_calibration(session, ARM_KEYS)
    print("Joint alignment. Use this when a joint is folded somewhere in the middle of\n"
          "its servo's 0-180 range, so it cannot extend far enough. The arm part comes\n"
          "off the servo, the servo goes to the end of its range, and the part goes back\n"
          "on folded down. Ctrl-C cancels.")
    while True:
        key = input("Which joint, [l]ower arm (S1) or [u]pper arm (S2)? ").strip().lower()[:1]
        if key in ("l", "u"):
            break
    joint, name, method = JOINT_STEPS[0 if key == "l" else 1][:3]
    ends = _joint_ends(cal, joint)
    if ends:
        print(f"Saved now: home is servo {ends[0]} deg, fully extended is servo {ends[1]} deg.")
    guess = "l" if ends and ends[1] < ends[0] else "h"
    while True:
        way = input(f"When the {name} extends, does its servo angle go [h]igher (towards 180)\n"
                    f"or [l]ower (towards 0)? [{guess}] ").strip().lower()[:1] or guess
        if way in ("h", "l"):
            break
    target = 0 if way == "h" else 180

    session.call("set_joint_limits", False, timeout=5)   # plain servo degrees, 0-180
    try:
        _wait_enter(f"\n1) Take the {name} OFF its servo: undo the screw in the middle of the\n"
                    "   servo horn, if there is one, and pull the part straight off the shaft.\n"
                    "   Support the arm so nothing falls. Press Enter once it is off...")
        print(f"   Moving the bare servo to {target} deg...")
        session.call(method, target, speed=JOINT_CAL_SPEED, timeout=60)
        _wait_enter(f"2) Fold the {name} all the way to its HOME (resting) position and push\n"
                    "   it back onto the shaft there. The shaft's teeth only let it sit in\n"
                    "   certain spots; pick the one closest to fully folded (calibration takes\n"
                    "   up the rest). Refit the screw, then press Enter...")
    except (KeyboardInterrupt, EOFError):
        session.link.interrupt()
        session.call("set_joint_limits", True, timeout=5)
        print("\nAlignment cancelled; calibration unchanged. If the part is off its servo,\n"
              "put it back on before moving the arm.")
        return

    # Rough ends until arm-calibrate pins them down: home at the servo's end,
    # fully extended at the other end.
    values = {joint + "_MIN": 0, joint + "_MAX": 180, joint + "_HOME": target}
    if cal.get(joint + "_DIRECTION") in (-1, 1):
        values[joint + "_DIRECTION"] = cal[joint + "_DIRECTION"]
    session.call("save_calibration", values, timeout=10)
    session.save_state()
    print(f"   The {name} is now at home = servo {target} deg, with the whole servo range\n"
          "   left for extending.")
    answer = input("3) Now set its exact home and fully extended positions. Run arm-calibrate "
                   "now? [Y/n] ").strip().lower()
    if answer[:1] != "n":
        print("(For a joint you do not want to change, press Enter twice to skip it.)")
        calibrate_arm(session)
    else:
        print("Run 'python arm_serial.py arm-calibrate' before using the arm.")


CALIBRATIONS["claw-calibrate"] = calibrate_claw
CALIBRATIONS["base-calibrate"] = calibrate_base
CALIBRATIONS["base-tune"] = tune_base
CALIBRATIONS["arm-calibrate"] = calibrate_arm
CALIBRATIONS["arm-align"] = align_joint


# Gestures run every swing back to back in one command, at one speed, so the
# arm moves evenly. (Sent as separate tool calls, each swing would snap and
# then wait seconds for the agent's next call.)
GESTURE_SPEED = 25        # joints, ~115 deg/s
GESTURE_POSE = (60, 90)   # raised: lower, upper position
GESTURES = {
    # name: (what swings, one end, other end, default swings)
    "wave": ("upper", 50, 130, 3),
    "nod": ("lower", 45, 75, 2),
    "shake": ("base", -20, 20, 2),
}
GESTURE_USAGE = "gesture wave|nod|shake [TIMES] [SPEED]"


def gesture(session, args):
    """Run a gesture and return the arm to the pose it started in."""
    _need(args, 1, GESTURE_USAGE)
    if args[0] not in GESTURES:
        raise UsageError("usage: " + GESTURE_USAGE)
    part, a, b, times = GESTURES[args[0]]
    if len(args) > 1:
        times = int(_num(args[1], "times"))
        if not 1 <= times <= 10:
            raise UsageError("times must be 1-10")
    speed = int(_num(args[2], "speed")) if len(args) > 2 else GESTURE_SPEED
    if not 1 <= speed <= 100:
        raise UsageError("speed must be 1-100")
    start = session.status()

    def joints(lower, upper):
        session.call("move_arm", lower, upper, speed=speed, timeout=30)

    try:
        if part == "base":
            centre = max(-70, min(70, start["base_angle"]))
            for _ in range(times):
                for end in (a, b):
                    session.call("turn_base", centre + end, timeout=30)
            session.call("turn_base", start["base_angle"], timeout=30)
        else:
            lower, upper = GESTURE_POSE
            joints(lower, upper)
            for _ in range(times):
                for end in (a, b):
                    if part == "lower":
                        joints(end, upper)
                    else:
                        joints(lower, end)
            joints(start["lower_arm"], start["upper_arm"])
    except KeyboardInterrupt:
        session.call("stop_all", timeout=5)
        raise
    finally:
        session.save_state()
    print(f"Done: {args[0]} x{times}, back where it started.")


def print_help():
    print("Commands (SPEED for joints is 1-100):")
    for _, text in COMMANDS.values():
        print("  " + text)
    for text in SESSION_COMMANDS.values():
        print("  " + text)


def run_command(session, name, args):
    if name not in COMMANDS:
        raise UsageError(f"unknown command {name!r} (try 'help')")
    call, timeout = COMMANDS[name][0](args)
    try:
        return session.run_call(call, timeout)
    finally:
        if name != "raw" or session.arm_loaded():
            session.save_state()


def shell(session):
    print("CyberBrick arm shell. Type 'help' for commands, 'quit' to exit.")
    while True:
        try:
            line = input("arm> ").strip()
        except EOFError:
            print()
            return
        except KeyboardInterrupt:
            print()
            continue
        if not line:
            continue
        if line in ("quit", "exit"):
            return
        if line in ("help", "?"):
            print_help()
            continue
        try:
            parts = shlex.split(line)
            if parts[0] in CALIBRATIONS:
                CALIBRATIONS[parts[0]](session)
            elif parts[0] == "gesture":
                gesture(session, parts[1:])
            else:
                run_command(session, parts[0], parts[1:])
        except UsageError as e:
            print(e)
        except DeviceError as e:
            print("Board error:\n" + str(e))
        except KeyboardInterrupt:
            pass


# =============================================================================
# Entry point
# =============================================================================

def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Control the CyberBrick robotic arm over the radio (through the "
                    "CyberBrick remote) or over the arm's own USB serial port.",
        epilog="Run 'python arm_serial.py help' to list commands.")
    parser.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyACM0 (the "
                        "remote's port for radio commands)")
    parser.add_argument("--transport", choices=("auto", "radio", "usb"),
                        default=os.environ.get("CYBERBRICK_TRANSPORT", "auto"),
                        help="radio: through the paired remote; usb: USB in the arm; "
                             "auto (default): radio once radio-install was run")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--reload", action="store_true",
                        help="reload the arm library even if already loaded")
    parser.add_argument("--json", action="store_true",
                        help="print one JSON result object on stdout (progress "
                             "messages go to stderr); for programs such as the "
                             "OpenClaw plugin")
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument("command", nargs="?", default="help")
    parser.add_argument("args", nargs=argparse.REMAINDER)
    opts = parser.parse_args(argv)

    if not opts.json:
        code, _ = _main(opts)
        return code

    # JSON mode: everything human-readable goes to stderr, and stdout gets a
    # single object: {"ok", "command", "before", "status", "output", "error"}
    # where before/status are arm.status() before and after the command.
    result = {"ok": False, "command": opts.command, "before": None,
              "status": None, "output": "", "error": None}
    if opts.command in SESSION_COMMANDS and opts.command not in (
            "reset", "ping", "calibration", "gesture"):
        result["error"] = f"{opts.command!r} is interactive or changes the " \
                          "board; run it without --json"
        print(json.dumps(result))
        return 2
    log = []

    class Tee:
        def write(self, text):
            log.append(text)
            return sys.stderr.write(text)

        def flush(self):
            sys.stderr.flush()

    with contextlib.redirect_stdout(Tee()):
        code, extra = _main(opts)
    result.update(extra)
    result["ok"] = code == 0
    if not result["ok"] and not result["error"]:
        lines = [l for l in "".join(log).splitlines() if l.strip()]
        result["error"] = lines[-1] if lines else f"failed with exit code {code}"
    print(json.dumps(result))
    return code


def _main(opts):
    """Run one command. Returns (exit code, result fields for --json)."""
    extra = {}
    if opts.command in ("help", "-h"):
        print_help()
        return 0, extra
    if opts.command not in COMMANDS and opts.command not in SESSION_COMMANDS:
        print(f"Unknown command {opts.command!r}\n")
        print_help()
        extra["error"] = f"unknown command {opts.command!r}"
        return 2, extra

    pair = load_pair()
    transport = opts.transport
    if transport == "auto":
        transport = "radio" if pair.get("arm_mac") else "usb"
    if opts.command in USB_ONLY:
        transport = "usb"
    if opts.command in RADIO_ONLY and transport != "radio":
        msg = (f"'{opts.command}' works over the radio. Set it up first: "
               "'radio-pair' (USB in the remote), then 'radio-install' (USB in the arm)."
               if not pair.get("arm_mac") else
               f"'{opts.command}' works over the radio; drop --transport usb.")
        print(msg)
        extra["error"] = msg
        return 2, extra

    if opts.command == "radio-install" and not pair.get("remote_mac"):
        msg = ("Pair the remote first: connect USB to the CyberBrick remote and run "
               "'radio-pair'.")
        print(msg)
        extra["error"] = msg
        return 2, extra

    if transport == "radio":
        link = CyberBrickSerial(opts.port, opts.baud, opts.verbose,
                                want_mac=pair.get("remote_mac"), name="CyberBrick remote",
                                env_var="CYBERBRICK_REMOTE_PORT", use_cache=False)
    elif opts.command == "radio-pair":
        link = CyberBrickSerial(opts.port, opts.baud, opts.verbose,
                                avoid_mac=pair.get("arm_mac"), name="CyberBrick remote",
                                env_var="CYBERBRICK_REMOTE_PORT", use_cache=False)
    else:
        link = CyberBrickSerial(opts.port, opts.baud, opts.verbose,
                                avoid_mac=pair.get("remote_mac"),
                                name="arm's receiver board")
    if not link.connect():
        extra["error"] = f"could not connect to the {link.name}"
        return 1, extra
    try:
        if opts.command == "probe":
            probe(link)
            return 0, extra
        if opts.command == "radio-pair":
            channel = int(_num(opts.args[0], "channel")) if opts.args else None
            if channel is not None and not 1 <= channel <= 13:
                raise UsageError("channel must be 1-13")
            radio_pair(link, pair, channel)
            return 0, extra

        if transport == "usb":
            session = ArmSession(link)
            if opts.command == "reset":
                print("Rebooting the board (into radio mode if radio-install was run, "
                      "otherwise its normal firmware)...")
                link.send_and_reboot()
                return 0, extra
            if opts.command == "install":
                session.install()
                return 0, extra
            if opts.command == "uninstall":
                session.uninstall()
                return 0, extra
            if opts.command == "radio-install":
                session.install_radio(pair)
                return 0, extra
        else:
            session = RadioSession(link, pair)

        session.load(force=opts.reload)
        if opts.command in CALIBRATIONS:
            CALIBRATIONS[opts.command](session)
            return 0, extra
        if opts.command == "reset":
            session.reboot("radio")
            print("Arm restarted:", _public(session.last_status))
            return 0, extra
        if opts.command == "calibration":
            keys = _source_cal_keys()
            cal = get_calibration(session, keys)
            for key in keys:
                print(f"  {key:20} {cal.get(key)}")
            for line in describe_joints(cal):
                print("  " + line)
            extra["calibration"] = cal
            extra["status"] = session.status()
            return 0, extra
        if opts.command == "ping":
            session.report_link()
            extra["status"] = _public(session.last_status)
            return 0, extra
        if opts.command == "update":
            session.update(force="--force" in opts.args)
            return 0, extra
        if opts.command == "remote":
            session.hand_to_remote()
            return 0, extra
        if opts.command == "gesture":
            extra["before"] = session.status()
            gesture(session, opts.args)
            extra["status"] = session.status()
            return 0, extra

        if opts.command == "shell":
            shell(session)
        else:
            if opts.json:
                extra["before"] = session.status()
            extra["output"] = run_command(session, opts.command, opts.args) or ""
            if opts.json:
                extra["status"] = session.status()
                if opts.command == "status":
                    # Joint limits and directions, so an agent knows which way
                    # "up" and "forward" are.
                    with contextlib.suppress(DeviceError):
                        extra["calibration"] = get_calibration(session, ARM_KEYS)
        return 0, extra
    except UsageError as e:
        print(e)
        extra["error"] = str(e)
        return 2, extra
    except WrongBoardError as e:
        print(e)
        extra["error"] = str(e)
        return 1, extra
    except DeviceError as e:
        print("Board error:\n" + str(e))
        extra["error"] = "board error: " + str(e).splitlines()[-1]
        return 1, extra
    except RadioError as e:
        print("Radio error:", e)
        extra["error"] = f"radio error: {str(e).splitlines()[0]}"
        return 1, extra
    except (ConnectionError, TimeoutError, serial.SerialException) as e:
        print("Serial error:", e)
        extra["error"] = f"serial error: {e}"
        return 1, extra
    except KeyboardInterrupt:
        extra["error"] = "interrupted"
        return 130, extra
    finally:
        # Stay in raw REPL (like CyberClaw) so the board's own firmware does
        # not restart and fight over the servos. Use 'reset' to hand back.
        link.close()


if __name__ == "__main__":
    sys.exit(main())
