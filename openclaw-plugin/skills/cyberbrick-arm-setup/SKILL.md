---
name: cyberbrick-arm-setup
description: Set up, calibrate and troubleshoot the CyberBrick robotic arm - re-centre the base, fix a claw that does not open or close fully, fix wrong turn direction, set the arm joints' home and fully extended positions, radio link, serial port or connection errors, update the arm over the radio, and hand the arm back to the CyberBrick remote. Use when an arm tool fails, the arm is not where its status says, or the user asks to calibrate or reset the arm.
metadata: {"openclaw": {"emoji": "🔧", "requires": {"anyBins": ["python3", "python"]}}}
---

# CyberBrick arm setup and troubleshooting

The arm tools run the PC driver `arm_serial.py` (in the CyberBrick project's `src` folder;
the plugin's `driverPath` setting points at it). In **radio mode** (after `radio-install`)
the USB cable goes to the **CyberBrick remote**, and the remote sends commands to the arm
over the radio; the arm has no cable. In **USB mode** the cable goes to the arm's
**receiver** board. Run driver commands from that `src` folder with the project's
Python, e.g. `.venv/bin/python arm_serial.py status` (Windows:
`.venv\Scripts\python arm_serial.py status`).

Some steps below need the user to watch the arm and press Enter at the right moment.
Those (`claw-calibrate`, `base-tune`, `arm-calibrate`, `arm-align`, `shell`) are interactive: **do not run them
yourself**; give the user the exact command to run in their own terminal. The same goes
for `radio-pair` and `radio-install`, which need the user to move USB cables.

## The base is not where the status says

The base angle is estimated from timing, so it can drift.

1. Ask the user to look at the arm. If it is a little off, use
   `cyberbrick_arm_face_angle` to put it where the user says it should be, then tell the
   arm that is centre: run `arm_serial.py base-set 0` once it physically faces straight
   ahead. (`base-set ANGLE` tells it any known angle.)
2. If it always ends up short or overshoots, the timing is off: the user should run
   `arm_serial.py base-tune` (interactive): the arm makes timed turns and the user says how
   far each one went, until it is 90°. It also fixes left/right being swapped.

## The claw does not open or close fully, or keeps pushing at the end

Timing is off. The user should run `arm_serial.py claw-calibrate` (interactive): it closes
the claw, then times a full open and a full close while they press Enter at the moment the
claw stops moving. If the claw estimate is just out of sync, put it in a known state and
tell the arm: `arm_serial.py claw-set closed` (or `claw-set open`).

## A joint hits the table, the base or the arm, or its direction is unknown

The joints are servos, so their angles are exact; what the arm needs to be told is where
each joint is folded at home and where it is fully extended. The user should run
`arm_serial.py arm-calibrate` (interactive, works over the radio or USB). For each joint
it:

1. lets them step the joint to its home (folded) position;
2. lets them step it to fully extended, stopping before it touches anything;
3. moves it 15 degrees out from home and asks which way it went.

After that the joint is driven in positions 0 (home) to 180 (fully extended), and
`arm_serial.py home` (or `cyberbrick_arm_home`) puts the arm back in its home pose.

The joints are moved **from the keyboard** (`+`, `-`, `+++`, `+20`, or an angle), never by
hand: while the battery is on, the servos hold their angle firmly and cannot report where
they are, so a joint that "won't move by hand" or "feels stuck" is normal. Pushing it can
strip the servo's gears. If a joint does not move when commanded, or buzzes, check the
battery, and step it back in case it is pressing on something.

If a joint cannot extend far enough because its home is in the middle of the servo's
range (for example folded at servo 100, leaving only 80 degrees), the arm part was pressed
onto the servo shaft in the wrong place. No setting fixes that. The user runs
`arm_serial.py arm-align`: it has them take the part off, moves the bare servo to the end
of its range, has them refit the part folded at home, and then runs `arm-calibrate`.

The results are saved on the arm and are used at once. `arm_serial.py calibration` shows
all the saved values, and `cyberbrick_arm_status` reports the joint positions and directions.

## Nothing moves, but tools succeed

- The servos and motor are powered by the **battery** on the arm, not USB. Ask the user
  to check the battery is on and charged. Using battery and USB together is fine.
- Run `arm_serial.py status`; a `timer_error` entry means the board's control loop
  crashed. Run `arm_serial.py reset` (radio) or `arm_serial.py --reload status` (USB) to restart it.

## Connection errors

Run `arm_serial.py ping` first in radio mode: it shows whether the arm answers, the signal
strength and the arm's free RAM.

| Error says                                  | Do this                                                       |
| ------------------------------------------- | ------------------------------------------------------------- |
| radio error: no reply from the arm          | The arm's radio hears the remote but radio mode is not answering. If `remote` was used, the arm stays in stick mode until it is switched off and on; otherwise ask the user to switch the arm off and on. |
| radio error: no radio contact with the arm  | The driver already restarted the remote once. Ask the user to check the arm is switched on and to move the remote closer. |
| Could not find the CyberBrick remote        | USB must go to the paired remote. Set `--port` / the plugin's `port` if needed. |
| needs USB in the CyberBrick remote ... this board is a receiver | The cable is in the arm; move it to the remote. |
| The bridge is not installed on this remote  | The user runs `arm_serial.py radio-pair` with USB in the remote. |
| could not connect / Could not find the board | Check the USB cable goes to the **receiver** on the arm. Set the port with `--port` or the plugin's `port` setting (`/dev/ttyACM0`, `COM5`). |
| This board is a CyberBrick transmitter      | USB is plugged into the remote. Move it to the arm's receiver. |
| stock firmware had not finished starting    | Run `arm_serial.py reset`, wait 5 seconds, try again.          |
| MemoryError                                 | Run `arm_serial.py reset`, wait 5 seconds, then `arm_serial.py install`. |
| Permission denied on /dev/ttyACM0 (Linux)   | The user needs to be in the `dialout` group.                  |
| timed out                                   | Run `arm_serial.py reset`, wait 5 seconds, retry once.        |

Do not retry a failing command more than once; report the error and the fix to the user.

## After editing cyberbrick_arm.py

Radio mode: run `arm_serial.py update`. It sends the new version over the radio and
restarts the arm; if it fails to start, the arm falls back to the previous copy by
itself. USB mode: run `arm_serial.py install`. Calibration results are saved on the arm
and need neither.

## Hand back to the CyberBrick remote

Radio mode: `arm_serial.py remote` puts the arm and the remote back into stick mode.
Switching the arm off and on returns it to radio mode, and the next arm command switches
the remote back into a bridge. USB mode: `arm_serial.py reset` reboots the receiver into
its normal remote-control mode. Moving the base with the sticks makes the base estimate
wrong: afterwards, centre the base and run `arm_serial.py base-set 0`.

## Safety settings (in cyberbrick_arm.py)

- `BASE_LIMIT_DEG = 90`: the base's left/right limit. Do not raise it; the user asked for
  the arm to stay within 90 degrees of centre.
- `LOWER_ARM_MIN/MAX`, `UPPER_ARM_MIN/MAX`: the default joint limits. `arm-calibrate` is
  the easier way to set them; values it saves on the arm override these. After editing
  the defaults, run `update` (radio) or `install` (USB).
