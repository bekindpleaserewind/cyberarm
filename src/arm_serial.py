#!/usr/bin/env python3
"""
Control the CyberBrick robotic arm from a PC over the board's USB serial port.

This runs on your computer (regular Python 3 + pyserial). It connects to the
CyberBrick receiver board on the arm, stops the firmware that is running,
switches MicroPython into raw REPL mode, loads the arm library
(cyberbrick_arm.py) and then sends arm commands as MicroPython code.

The arm library is loaded from the board's flash if you ran 'install' (it
stays there across power cycles), otherwise it is uploaded into RAM each time.
Installing only adds two files (/cyberbrick_arm.mpy, /cyberbrick_arm.ver);
the receiver still boots into its normal remote-control mode.

Serial handling follows unbug/CyberClaw's cyberbrick_driver:
  * port from --port, then $CYBERBRICK_PORT, then a cached port, then
    auto-detection (ESP32-C3 USB VID 0x303A / PID 0x1001 first)
  * 115200 baud, Ctrl-C to break into the REPL, DTR/RTS hard reset as fallback
  * code is sent through the raw REPL (Ctrl-A ... Ctrl-D)

Setup
-----
    pip install -r requirements.txt           # pyserial, mpy-cross
    python arm_serial.py install              # once (and after editing
                                              # cyberbrick_arm.py)

Examples
--------
    python arm_serial.py --port COM5 status
    python arm_serial.py home
    python arm_serial.py lower 120            # S1 to 120 deg
    python arm_serial.py upper 60 10          # S2 to 60 deg at speed 10
    python arm_serial.py move 110 70          # both joints together
    python arm_serial.py claw open            # S3: open / close / 0.0..1.0
    python arm_serial.py turn -45             # base: face 45 deg left of centre
    python arm_serial.py turn-by 30           # base: 30 deg further right
    python arm_serial.py --json status        # one JSON object on stdout
    python arm_serial.py demo
    python arm_serial.py shell                # interactive, one connection
    python arm_serial.py reset                # reboot back to normal firmware

The board must be the one wired to the servos and motor (Receiver Shield with
S1/S2/S3/M2), because the uploaded code drives that board's own pins.

The base never turns more than 90 degrees either side of centre. Its angle and
the claw opening are estimates; they are saved in .arm_state.json after every
command so they survive a board reboot. If the base was moved some other way
(by hand or with the remote), centre it and run 'base-set 0'.
"""

import argparse
import ast
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


class DeviceError(Exception):
    """The code sent to the board raised an exception there."""


class UsageError(Exception):
    """A command was typed with the wrong arguments."""


class WrongBoardError(Exception):
    """Connected to a board that cannot drive the arm."""


# =============================================================================
# Serial link + raw REPL
# =============================================================================

class CyberBrickSerial:
    def __init__(self, port=None, baud=BAUD, verbose=False):
        self.port = port
        self.baud = baud
        self.verbose = verbose
        self.serial = None
        self._rx = b""

    def _log(self, *args):
        if self.verbose:
            print("[serial]", *args)

    # ---------------------------------------------------------------- connect

    def _resolve_port(self):
        if self.port:
            return self.port

        port = os.environ.get("CYBERBRICK_PORT")
        if port:
            print(f"Using port from CYBERBRICK_PORT: {port}")
            return port

        if os.path.exists(PORT_CACHE):
            try:
                with open(PORT_CACHE) as f:
                    cached = f.read().strip()
                available = {p.device for p in serial.tools.list_ports.comports()}
                if cached in available or os.path.exists(cached):
                    print(f"Using cached port: {cached}")
                    return cached
            except OSError:
                pass

        print("Auto-detecting CyberBrick board...")
        candidates = []
        for p in serial.tools.list_ports.comports():
            if (p.vid, p.pid) == ESP32_C3_USB:
                candidates.insert(0, p)
            elif any(s in (p.description or "") for s in ("CyberBrick", "USB Serial", "CP210")) \
                    or "usbmodem" in p.device:
                candidates.append(p)
        if candidates:
            port = candidates[0].device
            print(f"Found: {port} ({candidates[0].description})")
            try:
                with open(PORT_CACHE, "w") as f:
                    f.write(port)
            except OSError:
                pass
            return port
        return None

    @staticmethod
    def _print_ports():
        print("Could not find the CyberBrick board. Specify the port with:")
        print("  --port COM5            (Windows)")
        print("  --port /dev/ttyACM0    (Linux)")
        print("  or set CYBERBRICK_PORT")
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

def source_version():
    """Short hash of the local cyberbrick_arm.py, used to spot stale installs."""
    with open(DEVICE_LIB, "rb") as f:
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
    try:
        import mpy_cross
    except ImportError:
        raise UsageError("'install' needs mpy-cross: "
                         "pip install mpy-cross==1.23.0.post2")

    with tempfile.TemporaryDirectory() as tmp:
        src_path = os.path.join(tmp, INSTALL_MODULE + ".py")
        out_path = os.path.join(tmp, INSTALL_MODULE + ".mpy")
        with open(src_path, "w", encoding="utf-8") as f:
            f.write(ast.unparse(library_tree(DEVICE_LIB)))
        proc = mpy_cross.run(src_path, "-O3", "-o", out_path,
                             "-s", os.path.basename(DEVICE_LIB),
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
PREP_CODE = """\
import gc, sys
gc.collect()
print(gc.mem_free(), int('bbl.servos' in sys.modules and 'bbl.motors' in sys.modules))
"""


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


class ArmSession:
    """Keeps the arm library loaded on the board and runs arm commands."""

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
        role = self.link.exec(
            "import machine\nprint(machine.Pin(10, machine.Pin.IN).value())").strip()
        if role == "1":
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
            with open(STATE_FILE, "w") as f:
                json.dump({"base_angle": status["base_angle"],
                           "claw": status["claw"]}, f)
        except (OSError, KeyError, ValueError, DeviceError,
                ConnectionError, TimeoutError) as e:
            print(f"Could not save the arm state: {e}")
        return status

    def _restore_state(self):
        try:
            with open(STATE_FILE) as f:
                state = json.load(f)
            base, claw = float(state["base_angle"]), float(state["claw"])
        except (OSError, KeyError, ValueError, TypeError):
            return
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

    def installed_version(self):
        """Version tag of the library saved on the board, or None."""
        out = self.link.exec(
            "try:\n"
            f" print(open({INSTALL_VERSION!r}).read().strip())\n"
            "except OSError:\n"
            " print('-')").strip()
        return None if out == "-" else out

    def load(self, force=False):
        """Make `arm` available on the board.

        Uses the copy installed in flash when it matches the local
        cyberbrick_arm.py; otherwise uploads the library into RAM.
        """
        self.check_receiver()
        if self.arm_loaded() and not force:
            return
        self._prepare()

        installed = self.installed_version()
        if installed == source_version():
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

        # Create the arm. This homes S1/S2 and stops S3 and M2.
        self.link.exec("import gc\ngc.collect()\narm = CyberBrickArm()", timeout=30)
        self._restore_state()
        print("Arm ready:", self.status())

    def install(self):
        """Compile cyberbrick_arm.py and save it in the board's flash.

        Only adds files; boot.py and the stock firmware are not changed, so
        the receiver still starts in its normal remote-control mode.
        """
        self.check_receiver()
        board_mpy = int(self.link.exec(
            "import sys\nprint(sys.implementation._mpy)").strip())
        data = compile_library(board_mpy)
        version = source_version()

        self._prepare()
        tmp = INSTALL_PATH + ".tmp"
        print(f"Writing {INSTALL_PATH} ({len(data)} bytes)...")
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
            raise ConnectionError("file was corrupted while writing; nothing installed")

        self.link.exec(
            "import os\n"
            "for _p in (%r, %r):\n"
            " try:\n"
            "  os.remove(_p)\n"
            " except OSError:\n"
            "  pass\n"
            "os.rename(%r, %r)\n"
            "del _p" % (INSTALL_PATH, INSTALL_VERSION, tmp, INSTALL_PATH))

        # Check it imports (defines code only; nothing moves). Only then write
        # the version tag, so a file the board can't load never counts.
        try:
            self.link.exec(f"import sys\nsys.modules.pop({INSTALL_MODULE!r}, None)\n"
                           f"import {INSTALL_MODULE}\n"
                           f"print({INSTALL_MODULE}.CyberBrickArm)", timeout=15)
        except DeviceError:
            self.link.exec(f"import os\nos.remove({INSTALL_PATH!r})")
            raise
        self.link.exec(f"with open({INSTALL_VERSION!r}, 'w') as _vf:\n"
                       f" _vf.write({version!r})\n"
                       "del _vf")
        print(f"Installed {INSTALL_PATH} (version {version}).")
        print("The receiver still boots into its normal remote-control mode.")

    def uninstall(self):
        """Delete the installed arm library from the board's flash."""
        self.check_receiver()
        out = self.link.exec(
            "import os\n"
            "_n = 0\n"
            "for _p in (%r, %r, %r):\n"
            " try:\n"
            "  os.remove(_p)\n"
            "  _n += 1\n"
            " except OSError:\n"
            "  pass\n"
            "print(_n)\n"
            "del _n, _p" % (INSTALL_PATH, INSTALL_PATH + ".tmp", INSTALL_VERSION))
        print("Removed the arm library from the board." if out.strip() != "0"
              else "The arm library was not installed.")

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


# =============================================================================
# Commands
# =============================================================================

STATUS = "\nprint(arm.status())"


def _num(value, name):
    try:
        return float(value)
    except (TypeError, ValueError):
        raise UsageError(f"{name} must be a number, got {value!r}")


def _opt_speed(args, i):
    return f", speed={_num(args[i], 'speed')!r}" if len(args) > i else ""


def _need(args, n, usage):
    if len(args) < n:
        raise UsageError("usage: " + usage)


def cmd_home(a):
    return f"arm.home({_opt_speed(a, 0).lstrip(', ')})" + STATUS, 60


def cmd_lower(a):
    _need(a, 1, "lower ANGLE [SPEED]")
    return f"arm.move_lower_arm({_num(a[0], 'angle')!r}{_opt_speed(a, 1)})" + STATUS, 60


def cmd_upper(a):
    _need(a, 1, "upper ANGLE [SPEED]")
    return f"arm.move_upper_arm({_num(a[0], 'angle')!r}{_opt_speed(a, 1)})" + STATUS, 60


def cmd_move(a):
    _need(a, 2, "move LOWER_ANGLE UPPER_ANGLE [SPEED]")
    lower, upper = _num(a[0], "lower angle"), _num(a[1], "upper angle")
    return f"arm.move_arm({lower!r}, {upper!r}{_opt_speed(a, 2)})" + STATUS, 60


def cmd_claw(a):
    _need(a, 1, "claw open|close|POSITION(0.0-1.0)")
    if a[0] == "open":
        code = "arm.open_claw()"
    elif a[0] in ("close", "closed"):
        code = "arm.close_claw()"
    else:
        code = f"arm.set_claw({_num(a[0], 'position')!r})"
    return code + STATUS, 30


def cmd_claw_spin(a):
    _need(a, 1, "claw-spin SPEED   (-100..100, ignores limits; 0 stops)")
    speed = _num(a[0], "speed")
    if speed == 0:
        return "arm.claw_stop()" + STATUS, 10
    return f"arm.claw_run({speed!r}, ignore_limits=True)" + STATUS, 10


def cmd_claw_cal(a):
    _need(a, 1, "claw-cal open|closed   (tell the arm the claw's current state)")
    if a[0] not in ("open", "closed", "close"):
        raise UsageError("claw-cal takes 'open' or 'closed'")
    return f"arm.calibrate_claw({a[0] == 'open'})" + STATUS, 10


def cmd_turn(a):
    _need(a, 1, "turn ANGLE   (-90 = full left .. 0 = centre .. 90 = full right)")
    return f"arm.turn_base({_num(a[0], 'angle')!r})" + STATUS, 60


def cmd_turn_by(a):
    _need(a, 1, "turn-by DEGREES   (negative = left; stays within 90 of centre)")
    return f"arm.turn_base_by({_num(a[0], 'degrees')!r})" + STATUS, 60


def cmd_base_set(a):
    angle = _num(a[0], "angle") if a else 0.0
    return f"arm.set_base_angle({angle!r})" + STATUS, 10


def cmd_base(a):
    _need(a, 1, "base SPEED [SECONDS]   (-100..100; no SECONDS = keep spinning)")
    speed = _num(a[0], "speed")
    if len(a) > 1:
        secs = _num(a[1], "seconds")
        return (f"arm.rotate_base({speed!r}, {int(secs * 1000)})" + STATUS,
                secs + 15)
    return f"arm.base_speed({speed!r})" + STATUS, 10


def cmd_stop(a):
    return "arm.stop_all()" + STATUS, 10


def cmd_release(a):
    return "arm.release_joints()" + STATUS, 10


def cmd_status(a):
    return "print(arm.status())", 10


def cmd_demo(a):
    return "demo(arm)", 300


def cmd_raw(a):
    _need(a, 1, "raw CODE")
    return " ".join(a), 60


COMMANDS = {
    "home":      (cmd_home,      "home [SPEED]                 joints to home pose, base to centre"),
    "lower":     (cmd_lower,     "lower ANGLE [SPEED]          S1 lower arm, 0-180"),
    "upper":     (cmd_upper,     "upper ANGLE [SPEED]          S2 upper arm, 0-180"),
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
    "raw":       (cmd_raw,       "raw CODE                     run MicroPython on the board"),
}
SESSION_COMMANDS = {
    "shell": "shell                        interactive prompt",
    "reset": "reset                        reboot board into normal firmware",
    "install":   "install                      save arm library on the board",
    "uninstall": "uninstall                    remove it from the board",
    "claw-calibrate": "claw-calibrate               time a full claw open and close and save them",
    "base-calibrate": "base-calibrate               time 90 deg base turns and save them",
}
CALIBRATIONS = {}   # filled in below: name -> function(session)


def _wait_enter(prompt):
    input(prompt)
    return time.monotonic()


def calibrate_claw(session):
    """Time a full claw open and close and save CLAW_OPEN_MS / CLAW_CLOSE_MS.

    The claw servo has no position sensor, so open/close are timed. This times
    a real open and close at CLAW_SPEED while the user watches, writes the
    results into cyberbrick_arm.py and reinstalls it if it was installed.
    """
    with open(DEVICE_LIB, encoding="utf-8") as f:
        src = f.read()
    speed = float(re.search(r"^CLAW_SPEED = (\d+)", src, re.M).group(1))
    link = session.link

    def spin(direction):
        link.exec(f"arm.claw_run({direction * speed!r}, ignore_limits=True)")

    def stop():
        link.exec("arm.claw_stop()")

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

    link.exec("arm.calibrate_claw(False)")
    session.save_state()
    print(f"Open took {open_ms} ms, close took {close_ms} ms.")
    if min(open_ms, close_ms) < 200:
        print("That is too short to be right; nothing saved.")
        return

    if not _save_constants({"CLAW_OPEN_MS": open_ms, "CLAW_CLOSE_MS": close_ms}):
        return
    _reinstall(session)
    print("The claw is closed and the arm knows it.")


def _reinstall(session):
    """After a calibration changed cyberbrick_arm.py: update the installed
    copy (if any) and reload the arm so it uses the new values."""
    if session.installed_version() is not None:
        print("Updating the copy installed on the board...")
        session.install()
    session.load(force=True)   # homes S1/S2, restores the saved state


def _save_constants(values):
    """Write NAME = value lines into cyberbrick_arm.py. False if one is missing."""
    with open(DEVICE_LIB, encoding="utf-8") as f:
        src = f.read()
    for name, value in values.items():
        src, n = re.subn(rf"^{name} = -?\d+", f"{name} = {value}",
                         src, count=1, flags=re.M)
        if n != 1:
            print(f"Could not find {name} in cyberbrick_arm.py; nothing saved.")
            return False
    with open(DEVICE_LIB, "w", encoding="utf-8") as f:
        f.write(src)
    print("Saved " + ", ".join(f"{k} = {v}" for k, v in values.items())
          + f" in {os.path.basename(DEVICE_LIB)}.")
    return True


def calibrate_base(session):
    """Time 90 degree base turns and save BASE_RIGHT_MS / BASE_LEFT_MS.

    The base motor has no position sensor, so its angle is worked out from
    how long it has turned. This also checks which way is right and fixes
    BASE_DIRECTION if needed. The 90 degree limit is off while it runs.
    """
    with open(DEVICE_LIB, encoding="utf-8") as f:
        src = f.read()
    speed = int(re.search(r"^BASE_TURN_SPEED = (\d+)", src, re.M).group(1))
    direction = int(re.search(r"^BASE_DIRECTION = (-?\d+)", src, re.M).group(1))
    link = session.link

    def spin(sign):
        link.exec(f"arm.base_speed({sign * speed})")

    def stop():
        link.exec("arm.base_stop()")

    limit = link.exec("print(arm._base_lim)").strip()
    link.exec("arm._base_lim = 1 << 28")   # no limit while calibrating
    print("Base calibration. Watch the arm and press Enter at each step.")
    try:
        while True:
            key = input("1) Nudge the base until the arm faces straight ahead:\n"
                        "   type l or r then Enter to nudge, or just Enter when "
                        "centred: ").strip().lower()
            if not key:
                break
            if key[0] in "lr":
                sign = -1 if key[0] == "l" else 1
                link.exec(f"arm.rotate_base({sign * speed}, 250)")

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
    except (KeyboardInterrupt, EOFError):
        stop()
        link.exec(f"arm._base_lim = {limit}")
        print("\nCalibration cancelled; nothing saved.")
        return

    if way[0] == "r":
        right_ms, left_ms = first_ms, back_ms
    else:
        # Positive speed turned it left: swap the direction and the timings.
        direction = -direction
        right_ms, left_ms = back_ms, first_ms
    print(f"90 deg right took {right_ms} ms, 90 deg left took {left_ms} ms.")
    link.exec(f"arm._base_lim = {limit}\narm.set_base_angle(0)")
    session.save_state()
    if min(right_ms, left_ms) < 200:
        print("That is too short to be right; nothing saved.")
        return
    if not _save_constants({"BASE_RIGHT_MS": right_ms, "BASE_LEFT_MS": left_ms,
                            "BASE_DIRECTION": direction}):
        return
    _reinstall(session)
    print("The base is centred and the arm knows it.")


CALIBRATIONS["claw-calibrate"] = lambda session: calibrate_claw(session)
CALIBRATIONS["base-calibrate"] = calibrate_base


def print_help():
    print("Commands (SPEED for joints is 1-100):")
    for _, text in COMMANDS.values():
        print("  " + text)
    for text in SESSION_COMMANDS.values():
        print("  " + text)


def run_command(session, name, args):
    if name not in COMMANDS:
        raise UsageError(f"unknown command {name!r} (try 'help')")
    code, timeout = COMMANDS[name][0](args)
    try:
        return session.run(code, timeout)
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
        description="Control the CyberBrick robotic arm over USB serial.",
        epilog="Run 'python arm_serial.py help' to list commands.")
    parser.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyACM0")
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
    if opts.command in SESSION_COMMANDS and opts.command not in ("reset",):
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

    link = CyberBrickSerial(opts.port, opts.baud, opts.verbose)
    if not link.connect():
        extra["error"] = "could not connect to the CyberBrick board"
        return 1, extra
    try:
        if opts.command == "reset":
            print("Rebooting the board into its normal firmware...")
            link.send_and_reboot()
            return 0, extra

        session = ArmSession(link)
        if opts.command in CALIBRATIONS:
            session.load(force=opts.reload)
            CALIBRATIONS[opts.command](session)
            return 0, extra
        if opts.command == "install":
            session.install()
            return 0, extra
        if opts.command == "uninstall":
            session.uninstall()
            return 0, extra
        session.load(force=opts.reload)

        if opts.command == "shell":
            shell(session)
        else:
            if opts.json:
                extra["before"] = session.status()
            extra["output"] = run_command(session, opts.command, opts.args) or ""
            if opts.json:
                extra["status"] = session.status()
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
