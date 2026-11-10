# CyberBrick arm: PC driver and board code

| File                | Runs on                                 | Job                                                          |
| ------------------- | --------------------------------------- | ------------------------------------------------------------ |
| `arm_serial.py`     | your PC (Python 3 + pyserial)           | Command-line tool that sends arm commands over the radio or USB |
| `cyberbrick_arm.py` | the arm's receiver board (MicroPython)  | Drives the servos and motor; does the timing and limits      |
| `arm_radio.py`      | the arm's receiver board                | Radio mode: takes commands from the remote over ESP-NOW      |
| `arm_bridge.py`     | the CyberBrick remote (transmitter)     | Passes commands from the PC to the arm over ESP-NOW          |
| `arm_boot.py`       | both boards (as `/lib/rc_main.mpy`)     | Boot hook: starts radio / bridge mode, or the stock firmware |

You normally only run `arm_serial.py`.

## Two ways to reach the arm

```
Radio:  PC --USB--> CyberBrick remote --ESP-NOW radio--> arm's receiver --> servos, motor
USB:    PC --USB--> arm's receiver --> servos, motor
```

- **Radio** (recommended). The arm's USB cable is needed **once**, for `radio-install`.
  After that the arm boots straight into radio mode and takes commands from its paired
  remote. The PC's cable goes to the remote. The radio is the ESP-NOW link the CyberBrick
  kit already uses, encrypted and paired to your remote only.
- **USB**. The original way: the cable goes into the arm's receiver, and each command is
  sent as MicroPython code.

`arm_serial.py` picks radio once `radio-install` has been run (`--transport auto`). Use
`--transport usb` or `--transport radio` to choose.

## Hardware

| Port | Part      | Notes                                           |
| ---- | --------- | ----------------------------------------------- |
| S1   | Lower arm | 180° servo, 0-180, 90 = home                    |
| S2   | Upper arm | 180° servo, 0-180, 90 = home                    |
| S3   | Claw      | 360° continuous servo; position is timed        |
| M2   | Base      | DC motor; angle is timed; limited to ±90°       |

The servos and motor run on the arm's **battery**, so switch it on. USB only powers the
board.

## Setup

```bash
pip install -r requirements.txt   # pyserial, mpy-cross
```

### Radio (USB to the arm only once)

1. **Pair the remote.** Plug USB into the CyberBrick **remote** and run:

   ```bash
   python arm_serial.py radio-pair        # optional: radio-pair CHANNEL (1-13, default 1)
   ```

   This records the remote's MAC, makes the link keys (saved in `.radio_pair.json`; keep
   it private, it is git-ignored), and installs the bridge on the remote.

2. **Install radio mode on the arm, once.** Plug a USB cable into the **arm's receiver**.
   The remote can stay plugged in; the driver tells the two boards apart by MAC. Run:

   ```bash
   python arm_serial.py radio-install
   ```

   This saves the arm library, the radio server, the link settings and the boot hook on
   the arm, then reboots it into radio mode.
   **Unplug the arm's USB cable; it is not needed again.**

3. **Check the link** (USB in the remote):

   ```bash
   python arm_serial.py ping
   python arm_serial.py arm-calibrate     # once, interactive: joint home and fully extended
   python arm_serial.py base-tune         # once, interactive
   ```

From then on every command goes over the radio.

### USB only

```bash
python arm_serial.py arm-calibrate
python arm_serial.py base-tune
```

- The first USB command compiles `cyberbrick_arm.py` and saves it on the board's flash,
  and so does the next one after any edit to it. It stays there across power cycles, and
  the board still boots into its normal remote-control mode. (`install` does the same
  straight away.)
- `base-tune` sets how long the base takes to turn 90°. Until you run it, the base
  uses a guessed time and its "90°" may not be a real 90°.

## arm_serial.py

```
python arm_serial.py [--port PORT] [--transport auto|radio|usb] [--json] [--reload] [-v] COMMAND [ARGS...]
python arm_serial.py help        # list every command
```

### Arm joints

| Command                      | Does                                            |
| ---------------------------- | ----------------------------------------------- |
| `lower POS [SPEED]`          | Lower arm (S1) to a position, 0 (home) - 180 (fully extended) |
| `upper POS [SPEED]`          | Upper arm (S2) to a position, 0 (home) - 180 (fully extended) |
| `move LOWER UPPER [SPEED]`   | Both joints together                            |
| `home [SPEED]`               | Joints to home (position 0) and base to centre  |
| `release`                    | Let S1/S2 go limp so you can move them by hand  |

`SPEED` is 1-100 (default 20). Use 10-20 when carrying something.

Joint positions are not degrees. After `arm-calibrate`, 0 is the joint's home (folded,
at rest) and 180 is fully extended, spread evenly over the servo angles in between. For
example, an upper arm folded at servo 100° and extended at 180° is at servo 140° at
position 90. Before calibration, positions are the raw servo angles 0-180, with home at
90.

### Claw

| Command                 | Does                                                          |
| ----------------------- | ------------------------------------------------------------- |
| `claw open`             | Open fully                                                    |
| `claw close`            | Close fully                                                   |
| `claw 0.5`              | Go to a position between 0 (closed) and 1 (open)              |
| `claw-set open\|closed` | Tell the arm the claw is fully open or closed right now (old name `claw-cal`) |
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

Over the radio, free spins (`base SPEED` with no SECONDS, `claw-spin`) also stop by
themselves after 20 seconds, in case the stop command is lost.

### Radio

| Command             | Does                                                                 |
| ------------------- | -------------------------------------------------------------------- |
| `radio-pair [CH]`   | USB in the remote: set it up as the radio bridge                     |
| `radio-install`     | USB in the arm, once: install radio mode and reboot into it          |
| `ping`              | Check the link: reply time, signal strength, free RAM, why the arm last restarted |
| `update [--force]`  | Send edited `cyberbrick_arm.py` / `arm_radio.py` to the arm over the radio |
| `remote`            | Hand the arm to the stick remote until the arm is switched off and on |

### Other

| Command      | Does                                                                |
| ------------ | ------------------------------------------------------------------- |
| `status`     | Print joint angles, claw position and base angle                    |
| `calibration`| Print every calibration value the arm uses (joints, claw, base)     |
| `stop`       | Stop the base and claw, hold the joints where they are              |
| `demo`       | Run through every joint, the claw and the base                      |
| `gesture wave\|nod\|shake [TIMES] [SPEED]` | One smooth gesture (raises the arm first if needed), then back to the starting pose |
| `raw CODE`   | USB only: run MicroPython on the board, e.g. `raw "print(arm.status())"` |
| `shell`      | Interactive `arm>` prompt; type the same commands without `python arm_serial.py` |
| `probe`      | USB: report the board's free RAM and flash, files and boot scripts (read-only) |
| `reset`      | Restart the arm: into radio mode once `radio-install` was run, otherwise its normal remote-control mode |
| `install`    | USB: save the arm library on the board now (USB commands do it anyway when it changed) |
| `uninstall`  | USB: remove the arm library and radio mode from the board           |

### Calibration (interactive)

These ask you to watch the arm and answer at each step. They work over the radio or USB.
They save their results on the arm (`/arm_cal.json`), which overrides the defaults in
`cyberbrick_arm.py`, and the arm uses them straight away. Nothing needs reinstalling.

- `arm-calibrate`: sets up the two arm joints, one at a time, moving slowly. You move the
  joints **from the keyboard**, not by hand. While the battery is on the servos hold their
  angle firmly (pushing them can strip the gears), and they cannot report where they are.
  During calibration you work in servo degrees. For each joint:
  1. step it to **home**, folded where it rests. Type `+` or `-` (5°), `+++` (three
     steps), `+20` / `-20`, or a servo angle such as `120`, then Enter;
  2. step it to **fully extended**, stopping a few degrees before it would touch the table,
     the base or the other part of the arm;
  3. it moves 15° out from home and asks which way it went (does extending lean the lower
     arm forward, or lift the claw?).

  From then on the joint uses positions 0 (home) to 180 (fully extended), `home` folds it
  back to 0, and the OpenClaw status tool tells the agent which way is forward and up.
  Joints already saved stay saved if you cancel part way.

  **Putting the arm back in its home pose:** `python arm_serial.py home`. It folds both
  joints to position 0 and centres the base. The arm also goes to its home pose whenever
  the arm library starts (a new USB session, or a reboot in radio mode).

- `arm-align`: use this when a joint's home sits in the middle of its servo's range, for
  example folded at servo 100 so that only 80 degrees are left for extending. A servo turns
  180 degrees in total, so no setting can fix that; the arm part has to go back on the
  servo shaft in a different place. `arm-align` asks which joint, has you take that part
  off its servo, drives the bare servo to the end of its range (0, or 180 if extending
  lowers the angle), and has you push the part back on folded at home. Then it runs
  `arm-calibrate`, so home is near servo 0 and the joint can extend through the whole
  range. The shaft's teeth only let the part sit in certain spots (about every 15
  degrees), so pick the one closest to folded.

- `claw-calibrate`: closes the claw, then times a full open and a full close. Run it if
  the claw stops short or keeps pushing after it is fully open or closed.
- `base-tune` (recommended for the base): sets how long a 90° turn takes, with no
  stopwatch. For each try:
  1. nudge the base to straight ahead (`l` / `r`, or `rrr` for three nudges at once);
  2. the arm turns right for an exact time, measured by its own clock;
  3. say how many degrees it actually turned (`70`, say), or just Enter if it was 90°.

  It scales the time by your answer (90 ÷ 70 here) and tries again until you press Enter.
  The first try also asks which way it turned and fixes left and right if they are
  swapped. Then it asks whether to use the same time for left turns (the default) or
  tune those the same way. The tries stop like normal turns do, so the motor's start-up
  and coasting are counted the same way as in use. Mark straight ahead and 90° on the
  table to judge the angle.
- `base-calibrate` is the old name and now runs `base-tune`. (The stopwatch method it
  used, pressing Enter when the base reached 90°, put your reaction time and the radio
  delay into the result, so it was removed.)

### Options

| Option        | Does                                                                  |
| ------------- | --------------------------------------------------------------------- |
| `--port PORT` | Serial port, e.g. `/dev/ttyACM0` or `COM5` (the remote's, for radio)  |
| `--transport` | `radio`, `usb`, or `auto` (default: radio once `radio-install` was run); also `$CYBERBRICK_TRANSPORT` |
| `--json`      | Print one JSON object on stdout (for programs such as the OpenClaw plugin) |
| `--reload`    | USB: reload the library on the board even if it is already running   |
| `--baud N`    | Serial speed (default 115200)                                         |
| `-v`          | Print serial connection details, for debugging                        |

Without `--port`, the radio uses `$CYBERBRICK_REMOTE_PORT` or finds the paired remote by
its MAC. USB uses `$CYBERBRICK_PORT`, then the last port that worked (`.port_cache`),
then auto-detection of the ESP32-C3 (skipping the paired remote).

`--json` output looks like this, over the radio or USB:

```json
{"ok": true, "command": "turn", "before": {...}, "status": {...}, "output": "...", "error": null}
```

`before` and `status` hold the arm state before and after the command. Interactive
commands (`shell`, `claw-calibrate`, `base-tune`, `arm-calibrate`) are refused in JSON mode.

### Examples

```bash
python arm_serial.py status
python arm_serial.py move 90 120 15      # both joints (positions), slowly
python arm_serial.py claw open
python arm_serial.py turn -45            # face 45° left
python arm_serial.py turn-by 30          # 30° further right
python arm_serial.py --transport usb --port COM5 home
```

## How radio mode works

- **Boot hook.** The stock `boot.py` ends with `import rc_main; rc_main.main()`. `/lib`
  comes before `/app` on `sys.path`, so `/lib/rc_main.mpy` (built from `arm_boot.py`) is
  found first. No stock file is changed.
  - On the arm it starts `arm_radio`.
  - On the remote it waits for the PC (bridge mode).
  - Otherwise it runs the stock firmware.
- **Commands.** The arm only runs a fixed list of arm methods (`METHODS` in
  `arm_radio.py`); it never runs code sent over the radio. Each request has a sequence
  number, so one that is sent again is never run twice. The arm sends its status back when
  the move has finished, and `stop` still gets through while a move is running.
- **Security.** The link is ESP-NOW unicast, encrypted with the keys from `radio-pair`,
  and the arm ignores every board except its paired remote.
- **Recovery without USB.**
  - If radio mode fails to start, the hook tries the copies kept from before the last
    `update`, then falls back to the stock firmware. A crash that reboots the board moves on
    to the next option. Switching the arm off and on always starts with radio mode again,
    even if it was switched off while still starting up. `ping` warns when the arm is
    running the backup copies.
  - After `remote` the arm runs the stock firmware for one boot.
  - `uninstall` over USB removes everything.
- **Updates.** `update` sends the compiled files in 150-byte pieces, checks their sha256,
  and swaps them in. The previous copy is kept as `*_old.mpy` for the fallback.

## Choosing a radio channel

The link uses one 2.4 GHz Wi-Fi channel (default 1). Wi-Fi networks on the same or a
nearby channel (up to 4 channels away) collide with it. The link sends again and keeps
trying for 5 seconds, but a busy network close by can still cause "no reply from the
arm" now and then. Pick a channel at least 5 away from the strong networks around you
(1, 6 and 11 are the usual choices), then set it on both boards:

```bash
python arm_serial.py radio-pair 6      # USB in the remote; keeps the same keys
python arm_serial.py radio-install     # USB in the arm, once more
```

The arm cannot be reached over the radio between those two steps.

## Space on the board

Measured on the receiver (ESP32-C3, MicroPython 1.23):

| Resource                         | USB mode (beside the stock firmware) | Radio mode, arm and radio running |
| -------------------------------- | ------------------------------------ | --------------------------------- |
| Free MicroPython RAM             | 30.5 KB                              | **60.5 KB** (steady over 400 messages) |
| Flash free (of 956 KB)           | 768 KB                               | ~756 KB                           |

Our files in flash:

| File                        | Size                                      |
| --------------------------- | ----------------------------------------- |
| `cyberbrick_arm.mpy`        | 7.0 KB                                    |
| `arm_radio.mpy`             | 4.7 KB                                    |
| `/lib/rc_main.mpy`          | 1.1 KB                                    |
| Settings and version files  | < 1 KB                                    |
| `*_old.mpy` after an update | up to 12 KB more                          |
| Remote: `arm_bridge.mpy`    | 1.1 KB, plus the 1.1 KB hook              |

Radio mode has more RAM than USB mode because the stock remote-control app (and the
LED, executor and sleep parts of `bbl`) are not loaded; only `bbl`'s servo and motor files
are. `python arm_serial.py ping` shows the arm's free RAM at any time.

Keeping RAM steady:

- The 10 ms timer that drives the joints, claw and base uses whole numbers only, so it
  creates no garbage. (Floats are separate objects on this board: the old joint glide
  made about 29 KB of garbage a second while both joints moved, measured on the chip.)
- The arm tidies its memory (`gc.collect()`) after each radio request while nothing is
  moving, so a collection never lands in the middle of a move, and the free RAM shown by
  `ping` is what is really free. It should stay about the same over time; if it keeps
  falling, something is leaking.
- `demo` runs from the PC one command at a time, so its code is not stored on the arm.

## cyberbrick_arm.py

The board library. `arm_serial.py` (USB) or `arm_radio.py` (radio) creates an
`arm = CyberBrickArm()` object on the board and calls its methods, so you only edit this
file to change settings or add features.

### Settings

The settings are in the `CONFIG` section at the top. After changing any of them, run
`python arm_serial.py update` (radio). Over USB, the next command installs the changed
file by itself.

| Setting                             | Meaning                                                   |
| ----------------------------------- | --------------------------------------------------------- |
| `LOWER_ARM_MIN/MAX`, `UPPER_ARM_MIN/MAX` | Servo-degree travel of each joint (`arm-calibrate` saves home and fully extended as these) |
| `LOWER_ARM_HOME`, `UPPER_ARM_HOME`  | Home servo angle (90 by default). When it equals MIN or MAX, positions run 0 (home) - 180 (the other end) |
| `LOWER_ARM_DIRECTION`, `UPPER_ARM_DIRECTION` | Which way extending moves the joint, for information (set by `arm-calibrate`) |
| `JOINT_SPEED`                       | Default joint speed (20)                                  |
| `CLAW_SPEED`                        | Claw servo speed for open/close                           |
| `CLAW_OPEN_MS`, `CLAW_CLOSE_MS`     | Full open / close time (default; `claw-calibrate` saves measured ones on the board) |
| `CLAW_OPEN_DIRECTION`               | Flip to 1 if `claw open` closes it                        |
| `CLAW_START_OPEN`                   | Whether the claw is open when the library starts fresh    |
| `BASE_TURN_SPEED`                   | Motor speed used for `turn` / `turn-by`                   |
| `BASE_RIGHT_MS`, `BASE_LEFT_MS`     | Time for a 90° turn (default; `base-tune` saves measured ones on the board) |
| `BASE_DIRECTION`                    | Flip to -1 if right and left are swapped                  |
| `BASE_LIMIT_DEG`                    | Base limit either side of centre (90). Do not raise it.   |

### Python API

Use these from `raw`, from your own MicroPython code, or in a REPL on the board:

```python
import cyberbrick_arm as ca
arm = ca.CyberBrickArm()

arm.move_lower_arm(120)              # S1, position 0 (home) .. 180 (extended)
arm.move_upper_arm(60, speed=10)     # S2
arm.move_arm(lower=110, upper=70)    # both
arm.home()                           # joints to position 0, base to centre
arm.servo_angle(2)                   # the servo angle behind S2's position

arm.open_claw(); arm.close_claw()
arm.set_claw(0.5)                    # 0 closed .. 1 open
arm.calibrate_claw(False)            # "the claw is closed now"

arm.turn_base(-45)                   # face 45° left
arm.turn_base_by(30)                 # 30° further right
arm.set_base_angle(0)                # "the base is at centre now"
arm.base_speed(40)                   # spin until stopped or the limit

arm.calibration()                    # dict of the calibration values in use
arm.save_calibration({"BASE_RIGHT_MS": 2100})   # save on the board and use now
arm.set_joint_limits(False)          # plain servo degrees 0-180 (calibration only); True restores

arm.status()                         # dict: lower_arm, upper_arm, claw, base_angle, base_speed
arm.stop_all()
arm.shutdown()
```

Movement methods wait until the move is done. Pass `wait=False` to return straight away,
then use `joints_moving()`, `claw_moving()`, `base_moving()` or the `wait_*()` methods.

To drive the arm with the remote's sticks, use `python arm_serial.py remote`; the demo
is `python arm_serial.py demo`. The library has no standalone mode of its own.

## Position estimates

The base and claw have no position sensors. Their positions come from how long they have
been moving. The PC saves them to `.arm_state.json` after every command and gives them
back to the arm after it restarts, so they survive a reboot.

If the base was moved by hand or with the remote, centre it and run
`python arm_serial.py base-set 0`. If the claw estimate is wrong, open or close it fully
and run `claw-set open` or `claw-set closed`.

## Tests

The radio protocol, the bridge, the boot hook fallbacks and the command mapping are tested
on the PC against fakes of the MicroPython modules:

```bash
.venv/bin/python -m unittest discover -s tests -v
```

## Troubleshooting

| Problem                                  | Fix                                                          |
| ---------------------------------------- | ------------------------------------------------------------ |
| A joint hits the table, base or arm      | `arm-calibrate`                                              |
| "No reply from the arm" now and then     | Wi-Fi nearby is crowding the radio channel: pick a clear one (see below) |
| Home pose is wrong                       | `arm-calibrate`, then `home`                                 |
| A joint can't extend far enough; its home is mid-way (e.g. servo 100) | `arm-align`: re-seats the arm part on the servo so home is at servo 0 |
| A joint won't move by hand / feels stuck | Normal while the battery is on: the servo holds its angle. Move it with commands (or `+`/`-` in `arm-calibrate`), or `release` to let the joints go limp |
| A joint doesn't move when commanded, or buzzes | Check the battery is on and charged; the joint may be pressing on something, so step it back the other way |
| Radio: "no reply from the arm"           | The arm's radio hears the remote but radio mode is not answering. If you ran `remote`, the arm stays in stick mode until it is switched off and on; otherwise switch it off and on. |
| Radio: "no radio contact with the arm, even after restarting the remote" | Nothing reached the arm. The PC already restarted the remote once (its radio can jam, for example after a soft reset from mpremote or Thonny). Check the arm is switched on and move the remote closer. |
| Radio: "Could not find the CyberBrick remote" | Plug USB into the paired remote, or pass `--port`       |
| Radio: "this board is a receiver"        | The cable is in the arm; move it to the remote               |
| Radio: stick remote does nothing         | The remote is in bridge mode: run `remote` to hand back to the sticks |
| Could not find the board (USB)           | Check the USB goes to the receiver; pass `--port`            |
| "This board is a CyberBrick transmitter" | USB is in the remote; use the radio, or move it to the arm's receiver |
| Commands succeed but nothing moves       | Switch the battery on                                        |
| `status` shows `timer_error`             | Radio: `reset`. USB: `python arm_serial.py --reload status`  |
| Timed out, MemoryError, or firmware still starting (USB) | `reset`, wait 5 seconds, try again (after a MemoryError, also run `install`) |
| Permission denied on `/dev/ttyACM0`      | Add your user to the `dialout` group                         |
| Base turns the wrong way or stops short  | `base-tune`                                                  |
| Claw stops short or keeps pushing        | `claw-calibrate`                                             |
