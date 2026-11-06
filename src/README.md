# CyberBrick arm: PC driver and board library

There are two scripts:

| File                | Runs on                         | Job                                                  |
| ------------------- | ------------------------------- | ---------------------------------------------------- |
| `arm_serial.py`     | your PC (Python 3 + pyserial)   | Command-line tool that sends arm commands over USB   |
| `cyberbrick_arm.py` | the arm's receiver board (MicroPython) | Drives the servos and motor; does the timing and limits |

You normally only run `arm_serial.py`. It connects to the receiver board, loads
`cyberbrick_arm.py` on it, and runs one command at a time.

## Hardware

| Port | Part      | Notes                                           |
| ---- | --------- | ----------------------------------------------- |
| S1   | Lower arm | 180° servo, 0-180, 90 = home                    |
| S2   | Upper arm | 180° servo, 0-180, 90 = home                    |
| S3   | Claw      | 360° continuous servo; position is timed        |
| M2   | Base      | DC motor; angle is timed; limited to ±90°       |

Plug the USB cable into the **receiver** on the arm (not the remote). The servos and motor
run on the arm's **battery**, so switch it on; USB alone powers only the board.

## Setup

```bash
pip install -r requirements.txt   # pyserial, mpy-cross
python arm_serial.py install      # store the library on the board
python arm_serial.py base-calibrate
```

- `install` compiles `cyberbrick_arm.py` and saves it on the board's flash. It stays there
  across power cycles, and the board still boots into its normal remote-control mode.
  Run it again after any edit to `cyberbrick_arm.py`. Without it, the library is uploaded
  into RAM on every command, which is slower.
- `base-calibrate` measures how long the base takes to turn 90°. Until you run it, the base
  uses a guessed time and its "90°" may not be a real 90°.

## arm_serial.py

```
python arm_serial.py [--port PORT] [--json] [--reload] [-v] COMMAND [ARGS...]
python arm_serial.py help        # list every command
```

### Arm joints

| Command                      | Does                                            |
| ---------------------------- | ----------------------------------------------- |
| `lower ANGLE [SPEED]`        | Lower arm (S1) to 0-180°                        |
| `upper ANGLE [SPEED]`        | Upper arm (S2) to 0-180°                        |
| `move LOWER UPPER [SPEED]`   | Both joints together                            |
| `home [SPEED]`               | Joints to 90/90 and base to centre              |
| `release`                    | Let S1/S2 go limp so you can move them by hand  |

`SPEED` is 1-100 (default 20). Use 10-20 when carrying something.

### Claw

| Command                 | Does                                                          |
| ----------------------- | ------------------------------------------------------------- |
| `claw open`             | Open fully                                                    |
| `claw close`            | Close fully                                                   |
| `claw 0.5`              | Go to a position between 0 (closed) and 1 (open)              |
| `claw-cal open\|closed` | Tell the arm the claw is fully open or closed right now       |
| `claw-spin SPEED`       | Spin S3 freely at -100..100, ignoring limits; `0` stops       |

### Base

The base never goes further than 90° left or right of centre. Angles are seen from behind
the arm: negative is left, positive is right, 0 is straight ahead.

| Command               | Does                                                          |
| --------------------- | ------------------------------------------------------------- |
| `turn ANGLE`          | Face an angle, -90..90                                        |
| `turn-by DEGREES`     | Turn relative to where it is (negative = left); stops at ±90  |
| `base-set [ANGLE]`    | Tell the arm what angle it is at now (default 0 = centre)     |
| `base SPEED [SECONDS]`| Spin at -100..100; with no SECONDS it runs until the ±90 limit |

### Other

| Command      | Does                                                                |
| ------------ | ------------------------------------------------------------------- |
| `status`     | Print joint angles, claw position and base angle                    |
| `stop`       | Stop the base and claw, hold the joints where they are              |
| `demo`       | Run through every joint, the claw and the base                      |
| `raw CODE`   | Run MicroPython on the board, e.g. `raw "print(arm.status())"`      |
| `shell`      | Interactive `arm>` prompt; type the same commands without `python arm_serial.py` |
| `reset`      | Reboot the board back into normal remote-control mode               |
| `uninstall`  | Remove the installed library from the board                         |

### Calibration (interactive)

These ask you to watch the arm and press Enter at the right moment. They save the
measured times into `cyberbrick_arm.py` and reinstall it.

- `claw-calibrate`: closes the claw, then times a full open and a full close. Run it if
  the claw stops short or keeps pushing after it is fully open or closed.
- `base-calibrate`: lets you nudge the base to centre, then times a 90° turn and the turn
  back. It asks which way the base turned and fixes the direction if left and right are
  swapped.

### Options

| Option        | Does                                                                  |
| ------------- | --------------------------------------------------------------------- |
| `--port PORT` | Serial port, e.g. `/dev/ttyACM0` or `COM5`                           |
| `--json`      | Print one JSON object on stdout (for programs such as the OpenClaw plugin) |
| `--reload`    | Reload the library on the board even if it is already running         |
| `-v`          | Print serial connection details, for debugging                        |

Without `--port`, the port comes from `$CYBERBRICK_PORT`, then the last port that worked
(`.port_cache`), then auto-detection of the ESP32-C3.

`--json` output looks like this:

```json
{"ok": true, "command": "turn", "before": {...}, "status": {...}, "output": "...", "error": null}
```

`before` and `status` hold the arm state before and after the command. Interactive
commands (`shell`, `claw-calibrate`, `base-calibrate`) are refused in JSON mode.

### Examples

```bash
python arm_serial.py status
python arm_serial.py move 110 70 15      # both joints, slowly
python arm_serial.py claw open
python arm_serial.py turn -45            # face 45° left
python arm_serial.py turn-by 30          # 30° further right
python arm_serial.py --port COM5 home
```

## cyberbrick_arm.py

The board library. `arm_serial.py` creates an `arm = CyberBrickArm()` object on the board
and calls its methods, so you only edit this file to change settings or add features.

### Settings

The settings are in the `CONFIG` section at the top. Run `python arm_serial.py install`
after changing any of them.

| Setting                             | Meaning                                                   |
| ----------------------------------- | --------------------------------------------------------- |
| `LOWER_ARM_MIN/MAX`, `UPPER_ARM_MIN/MAX` | Joint limits; narrow them if a joint hits the table or the arm |
| `LOWER_ARM_HOME`, `UPPER_ARM_HOME`  | Home pose (90/90)                                          |
| `JOINT_SPEED`                       | Default joint speed (20)                                  |
| `CLAW_SPEED`                        | Claw servo speed for open/close                           |
| `CLAW_OPEN_MS`, `CLAW_CLOSE_MS`     | Full open / close time; set by `claw-calibrate`           |
| `CLAW_OPEN_DIRECTION`               | Flip to 1 if `claw open` closes it                        |
| `CLAW_START_OPEN`                   | Whether the claw is open when the library starts fresh    |
| `BASE_TURN_SPEED`                   | Motor speed used for `turn` / `turn-by`                   |
| `BASE_RIGHT_MS`, `BASE_LEFT_MS`     | Time for a 90° turn; set by `base-calibrate`              |
| `BASE_DIRECTION`                    | Flip to -1 if right and left are swapped                  |
| `BASE_LIMIT_DEG`                    | Base limit either side of centre (90). Do not raise it.   |
| `MODE`                              | What runs when the file is started directly (see below)   |

### Python API

Use these from `raw`, from your own MicroPython code, or in a REPL on the board:

```python
import cyberbrick_arm as ca
arm = ca.CyberBrickArm()

arm.move_lower_arm(120)              # S1
arm.move_upper_arm(60, speed=10)     # S2
arm.move_arm(lower=110, upper=70)    # both
arm.home()

arm.open_claw(); arm.close_claw()
arm.set_claw(0.5)                    # 0 closed .. 1 open
arm.calibrate_claw(False)            # "the claw is closed now"

arm.turn_base(-45)                   # face 45° left
arm.turn_base_by(30)                 # 30° further right
arm.center_base()
arm.set_base_angle(0)                # "the base is at centre now"
arm.base_speed(40)                   # spin until stopped or the limit

arm.status()                         # dict: lower_arm, upper_arm, claw, base_angle, base_speed
arm.stop_all()
arm.shutdown()
```

Movement methods wait until the move is done. Pass `wait=False` to return straight away,
then use `joints_moving()`, `claw_moving()`, `base_moving()` or the `wait_*()` methods.

### Running it on the board without the PC

```bash
mpremote run cyberbrick_arm.py
```

This runs whatever `MODE` is set to:

| `MODE`         | Does                                                         |
| -------------- | ------------------------------------------------------------ |
| `"demo"`       | Scripted demo of every part (default)                        |
| `"rc"`         | Drive the arm from a paired CyberBrick remote                |
| `"rc_monitor"` | Print raw remote values, to find which stick is which        |
| `"none"`       | Set up the arm and hold it, for REPL use                     |

## Position estimates

The base and claw have no position sensors. Their positions come from how long they have
been moving. The PC saves them to `.arm_state.json` after every command, so they survive
a board reboot.

If the base was moved by hand or with the remote, centre it and run
`python arm_serial.py base-set 0`. If the claw estimate is wrong, open or close it fully
and run `claw-cal open` or `claw-cal closed`.

## Troubleshooting

| Problem                                  | Fix                                                          |
| ---------------------------------------- | ------------------------------------------------------------ |
| Could not find the board                 | Check the USB goes to the receiver; pass `--port`            |
| "This board is a CyberBrick transmitter" | USB is in the remote; move it to the arm's receiver          |
| Commands succeed but nothing moves       | Switch the battery on                                        |
| `status` shows `timer_error`             | `python arm_serial.py --reload status`                       |
| Timed out, MemoryError, or firmware still starting | `reset`, wait 5 seconds, try again (after a MemoryError, also run `install`) |
| Permission denied on `/dev/ttyACM0`      | Add your user to the `dialout` group                         |
| Base turns the wrong way or stops short  | `base-calibrate`                                             |
| Claw stops short or keeps pushing        | `claw-calibrate`                                             |
