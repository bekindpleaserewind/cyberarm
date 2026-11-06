---
name: cyberbrick-arm-setup
description: Set up, calibrate and troubleshoot the CyberBrick robotic arm - re-centre the base, fix a claw that does not open or close fully, fix wrong turn direction, serial port or connection errors, and hand the arm back to the CyberBrick remote. Use when an arm tool fails, the arm is not where its status says, or the user asks to calibrate or reset the arm.
metadata: {"openclaw": {"emoji": "🔧", "requires": {"anyBins": ["python3", "python"]}}}
---

# CyberBrick arm setup and troubleshooting

The arm tools run the PC driver `arm_serial.py` (in the CyberBrick project's `src` folder;
the plugin's `driverPath` setting points at it). The driver talks to the arm's **receiver**
board over USB serial. Run driver commands from that `src` folder with the project's
Python, e.g. `.venv/bin/python arm_serial.py status` (Windows:
`.venv\Scripts\python arm_serial.py status`).

Some steps below need the user to watch the arm and press Enter at the right moment.
Those (`claw-calibrate`, `base-calibrate`, `shell`) are interactive: **do not run them
yourself**; give the user the exact command to run in their own terminal.

## The base is not where the status says

The base angle is estimated from timing, so it can drift.

1. Ask the user to look at the arm. If it is a little off, use
   `cyberbrick_arm_face_angle` to put it where the user says it should be, then tell the
   arm that is centre: run `arm_serial.py base-set 0` once it physically faces straight
   ahead. (`base-set ANGLE` tells it any known angle.)
2. If it always ends up short or overshoots, the timing is off: the user should run
   `arm_serial.py base-calibrate` (interactive). It also fixes left/right being swapped.

## The claw does not open or close fully, or keeps pushing at the end

Timing is off. The user should run `arm_serial.py claw-calibrate` (interactive): it closes
the claw, then times a full open and a full close while they press Enter at the moment the
claw stops moving. If the claw estimate is just out of sync, put it in a known state and
tell the arm: `arm_serial.py claw-cal closed` (or `claw-cal open`).

## Nothing moves, but tools succeed

- The servos and motor are powered by the **battery** on the arm, not USB. Ask the user
  to check the battery is on and charged. Using battery and USB together is fine.
- Run `arm_serial.py status`; a `timer_error` entry means the board's control loop
  crashed. Run `arm_serial.py --reload status` to restart it.

## Connection errors

| Error says                                  | Do this                                                       |
| ------------------------------------------- | ------------------------------------------------------------- |
| could not connect / Could not find the board | Check the USB cable goes to the **receiver** on the arm. Set the port with `--port` or the plugin's `port` setting (`/dev/ttyACM0`, `COM5`). |
| This board is a CyberBrick transmitter      | USB is plugged into the remote. Move it to the arm's receiver. |
| stock firmware had not finished starting    | Run `arm_serial.py reset`, wait 5 seconds, try again.          |
| MemoryError                                 | Run `arm_serial.py reset`, wait 5 seconds, then `arm_serial.py install`. |
| Permission denied on /dev/ttyACM0 (Linux)   | The user needs to be in the `dialout` group.                  |
| timed out                                   | Run `arm_serial.py reset`, wait 5 seconds, retry once.        |

Do not retry a failing command more than once; report the error and the fix to the user.

## After editing cyberbrick_arm.py

Run `arm_serial.py install` so the board uses the new version (it is stored on the board
and survives power cycles).

## Hand back to the CyberBrick remote

`arm_serial.py reset` reboots the receiver into its normal remote-control mode. Moving the
base with the remote makes the base estimate wrong: afterwards, centre the base and run
`arm_serial.py base-set 0`.

## Safety settings (in cyberbrick_arm.py)

- `BASE_LIMIT_DEG = 90`: the base's left/right limit. Do not raise it; the user asked for
  the arm to stay within 90 degrees of centre.
- `LOWER_ARM_MIN/MAX`, `UPPER_ARM_MIN/MAX`: narrow these if a joint can hit the table or
  the arm itself. Run `install` after changing them.
