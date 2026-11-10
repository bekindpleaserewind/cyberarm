# -*- coding: utf-8 -*-
"""
Boot hook for the CyberBrick arm (MicroPython). Installed as /lib/rc_main.mpy
by `arm_serial.py radio-install` (on the arm) and `radio-pair` (on the remote).

The stock boot.py ends with `import rc_main; rc_main.main()`. /lib comes
before /app on sys.path, so this file is found first and decides what runs.
No stock file is changed; deleting /lib/rc_main.mpy restores stock behaviour.

On the arm (receiver):
  * normally: radio mode (arm_radio.run), waiting for the paired remote;
  * after `arm_serial.py remote` (MODE_FILE says "stock"): the stock
    remote-control firmware, for that one boot;
  * if radio mode fails to start: first the copies kept from before the last
    update, then the stock firmware. COUNT_FILE counts the attempts so a
    crash that reboots the board moves on to the next one. Switching the
    board off and on always starts again with radio mode, so turning it off
    while it is still starting up is not mistaken for a crash;
  * after a soft reset (Ctrl-D, mpremote, Thonny): a real restart first. Wi-Fi
    stays half on through a soft reset and then sends nothing, so radio mode
    would look like it is running while the remote gets no reply.

On the remote (transmitter):
  * MODE_FILE says "bridge": do nothing and wait at the REPL for the PC
    (arm_bridge.py does the radio work);
  * otherwise: the stock firmware.
"""

import os
import sys

import machine

MODE_FILE = "/arm_mode"
COUNT_FILE = "/arm_boot"

# (radio module, arm library) to try, in order.
TRIES = (
    ("arm_radio", "cyberbrick_arm"),
    ("arm_radio", "cyberbrick_arm_old"),
    ("arm_radio_old", "cyberbrick_arm_old"),
    ("arm_radio_old", "cyberbrick_arm"),
)


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return ""


def _write(path, text):
    try:
        with open(path, "w") as f:
            f.write(text)
    except OSError:
        pass


def _remove(path):
    try:
        os.remove(path)
    except OSError:
        pass


def stock():
    """Run the stock CyberBrick firmware (/app/rc_main.py). Does not return."""
    sys.modules.pop("rc_main", None)
    hidden = [p for p in sys.path if p == "/lib"]
    for p in hidden:
        sys.path.remove(p)
    try:
        import rc_main
    finally:
        sys.path.extend(hidden)
    rc_main.main()


def _ready():
    _remove(COUNT_FILE)


def _power_on():
    try:
        return machine.reset_cause() == machine.PWRON_RESET
    except Exception:
        return False


def _soft_reset():
    try:
        return machine.reset_cause() == machine.SOFT_RESET
    except Exception:
        return False


def _receiver():
    if _soft_reset():
        machine.reset()                 # see the module docstring
    try:
        start = int(_read(COUNT_FILE) or 0)
    except ValueError:
        start = 0
    if start and _power_on():
        start = 0       # switched off and on, not a crash: radio mode first
    for i in range(start, len(TRIES)):
        _write(COUNT_FILE, str(i + 1))
        radio, lib = TRIES[i]
        try:
            __import__(radio).run(lib, _ready)
            return True                 # Ctrl-C over USB: stay at the REPL
        except KeyboardInterrupt:
            return True
        except Exception as e:
            print("[arm] %s with %s did not start: %r" % (radio, lib, e))
            for name in (radio, lib):
                sys.modules.pop(name, None)
    _remove(COUNT_FILE)
    return False


def main():
    try:
        mode = _read(MODE_FILE)
        if machine.Pin(10, machine.Pin.IN).value():     # the remote
            if mode == "bridge":
                print("[arm] bridge mode: waiting for arm_serial.py")
                return
        elif mode == "stock":
            _remove(MODE_FILE)                          # one boot only
        elif _receiver():
            return
    except KeyboardInterrupt:
        return
    except Exception as e:
        print("[arm] boot hook error:", repr(e))
    stock()
