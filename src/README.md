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

On Linux your user must be in the `dialout` group to open `/dev/ttyACM0`. On WSL, attach
each board to Linux with `usbipd.exe list` and `usbipd.exe attach --wsl --busid <id>`;
the remote and the arm are separate USB devices, so attach whichever one is plugged in.

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
| `debug`             | Read both boards' radio counters, the remote's heartbeat log, the arm's memory and Wi-Fi state, and its last snapshot from before it restarted (read-only). Run it right after a radio failure, before switching anything off: see [Radio diagnostics](#radio-diagnostics) |
| `update [--force]`  | Send edited `cyberbrick_arm.py` / `arm_radio.py` to the arm over the radio |
| `radio-set CH [TXPOWER] [--plain\|--encrypted]` | Move both boards to another channel (1-13), transmit power (2-20 dBm) and/or encryption, over the radio: see [Radio settings](#radio-settings) |
| `radio-test SECONDS [RATE] [--no-echo]` | Measure the bare radio link for 2-1800 s, with nothing else running on the arm: see [Radio test](#radio-test) |
| `radio-test-result [--prev]` | Show the arm's counts from its last radio test (or the one before) |
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
| `restart`    | Recover from a board error and keep control: radio reboots the arm and waits for it; USB reloads the arm library (the joints go home) |
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
  and the arm ignores every board except its paired remote. (`radio-set ... --plain`
  turns the encryption off for testing; then anyone nearby who knows the remote's address
  could send the arm commands, so switch it back with `--encrypted`.)
- **Message size.** ESP-NOW carries at most 250 bytes per message. The largest request
  is an `update` piece (217 bytes); the remote refuses anything longer with "request too
  long" instead of sending it, and the arm replaces a reply that would be too long with
  an error.
- **A remote that was just switched on is restarted once.** After a power-on (plugged
  in, or switched on) the remote's radio loses more and more messages: about 1 in 10 at
  first, every one within 2 minutes, at any transmit power, and switching Wi-Fi off and
  on does not cure it. One restart of the chip does. So the first radio command after
  the remote is switched on prints "The remote was just switched on ... restarting it"
  and takes about 3 s longer; after that the remote is used as it is.
- **Wi-Fi at start-up.** The stock firmware starts Wi-Fi before radio mode or the bridge
  runs, with power saving on (the radio dozes and misses messages). Both boards then:
  - switch the station on (the remote switches it off and on first; the arm only
    switches it on if it is off),
  - set `reconnects=0` and disconnect, so the station never looks for a network,
  - put it on the paired channel, turn power saving off (`pm=PM_NONE`) and set the
    transmit power from the pairing (`radio-set`; otherwise the firmware's 20 dBm),
  - start ESP-NOW with a 2048-byte receive buffer (room for 7 full-size messages).

  The access point stays on. On this firmware it follows the station: switching it off
  stops the station too, and switching the station back on brings it back (measured on
  both boards). If Wi-Fi does not end up on and on the paired channel, the board stops
  with "Wi-Fi did not start on channel N" instead of running deaf (the arm then falls
  back to its previous radio files). If the remote ever hangs at its off-and-on step,
  switch it off and on: it finds `/arm_wifi_try` left behind and from then on starts the
  gentle way; `radio-pair` clears the file.
- **Either board can restart on its own.** The remote, powered over USB, usually outlives
  the arm being switched off and on, and the other way round. Afterwards the side that
  stayed on can hold stale link state for the other. So the remote adds the arm as a
  peer again at the start of every command session, and when an acked request gets no
  answer for 2 s. (The arm can do the same after 3 s without hearing the remote,
  `IDLE_REKEY` in `arm_radio.py`; it is off, since it made no difference.) The remote
  keeps resending (same sequence number) for 10 s once the arm's radio has acked, which
  also covers a command sent while the arm is still starting up; with no ack at all it
  gives up after 5 s ("no radio contact").
- **Heartbeat.** While the bridge runs, the remote sends the arm a 1-byte message once a
  second (`HEARTBEAT_MS` in `arm_bridge.py`), which the arm ignores. The remote counts
  which ones the arm's radio acknowledges and logs every stretch of 2 or more in a row
  that it didn't, so `debug` can show when the link went quiet and for how long.
- **Recovery without USB.**
  - If radio mode fails to start, the hook tries the copies kept from before the last
    `update`, then falls back to the stock firmware. A crash that reboots the board moves on
    to the next option. Switching the arm off and on always starts with radio mode again,
    even if it was switched off while still starting up. `ping` warns when the arm is
    running the backup copies.
  - After `remote` the arm runs the stock firmware for one boot.
  - `uninstall` over USB removes everything.
- **Updates.** `update` sends the compiled files in 150-byte pieces, checks their sha256,
  and swaps them in. The previous copy is kept as `*_old.mpy` for the fallback. Changing
  a comment counts as a change too: the next `update` sends the file again.

## Radio diagnostics

When a radio command fails, run this **before switching either board off**:

```bash
python arm_serial.py debug
```

A board loses its counters when it restarts, and `debug` is careful not to cause one: it
sends nothing again, and if nothing reaches the arm it doesn't restart the remote (other
commands restart it once then). It only restarts the remote when the remote is in
stick mode, was soft-reset or was just switched on, because then it has to before it can talk to the bridge. It reads the remote first, before its
own requests add any traffic, then asks the arm for its counters (`"d"`) and for the
snapshot it saved before its last restart (`"d", "prev"`). If the arm doesn't answer,
the remote's half is still printed, along with the error. With `--json`, the readings are
in a `debug` object: `remote`, `status`, `arm`, `prev`, `mem` and `error`.

Example output:

```text
Remote: ESP-NOW sent 138, send results 138, failed results 46, received 91, dropped 0; signal -39 dBm.
Remote Wi-Fi: on, channel 1, power save 0, transmit power 20 dBm, full reset at start-up
Remote heartbeat: sent 1185, acked by the arm 945; unacked in a row now 0
  gap of 19.7 s, 61 s ago
  gap of 2.5 s, 41 s ago
Arm, this boot:
  boot 14740, up 2m 49s, main loop 16050 rounds
  requests heard 36, ignored 957, last one 0s ago
  ESP-NOW sent 46, send results 46, failed results 13, received 994, dropped 0
  reply sends while moving 0, of them not acked 0
  replies given up on 2; recovery: radio restart fixed 0 of 2, Wi-Fi restart fixed 2 of 2
    incident at 16s: Wi-Fi restart
    incident at 2m 9s: Wi-Fi restart
  Wi-Fi: on, channel 1, power save 0, transmit power 20 dBm
Arm, last snapshot from before this boot:
  boot 31553, saved after 3m 5s, main loop 17717 rounds
  ...
Arm memory: MicroPython heap 47.0 KB free, 41.3 KB in use, lowest 18.3 KB (at 7s after boot)
  system heap (Wi-Fi buffers) 41.1 KB free, lowest 41.1 KB (at 1s after boot)
Arm Wi-Fi: station on, channel 1, idle; access point on, channel 1
    region 118.2 KB: 18.7 KB free, largest block 18.0 KB, lowest since boot 18.6 KB
    ...
```

What the lines mean:

| Line | Meaning |
| ---- | ------- |
| `main loop N rounds` | How often the arm's radio loop has run. If it keeps rising while nothing gets through, the arm's code is running and only the radio is stuck |
| `requests heard`, `ignored` | Requests from the paired remote, and messages dropped (other boards, or not a request) |
| `last one ... ago` | Time since the arm last heard a request |
| `reply sends while moving` | Reply sends tried while a joint, the claw or the base was moving, and how many of them got no ack. Each reply is tried up to 3 times |
| `replies given up on` | Replies that failed all 3 tries. Each one starts a recovery (below) |
| `recovery`, `incident` | How often each recovery step was tried and how often it worked, then the last 3 incidents: uptime, whether something was moving, and the step that fixed it (`not fixed yet` when both steps failed) |
| `Remote heartbeat` | Heartbeats sent and acked by the arm's radio, how many in a row are unacked right now, and the last 8 gaps of 2 s or more with how long ago they were |
| `Wi-Fi` | `on` or `OFF` (OFF: the board can neither send nor hear), channel (should match the pair, 1 by default), power-save mode (should be 0), transmit power, `UNENCRYPTED` if the link is set to plain, and for the remote how its Wi-Fi started (`full reset at start-up`, or `gentle start` after a start that hung) |
| `Arm memory` | MicroPython heap and the system heap the Wi-Fi driver uses: free now, and the lowest seen this boot (sampled every second; each region's lowest point is ESP-IDF's own exact count) |
| `Arm Wi-Fi` | Station on/off, channel, status (`idle` = not looking for a network), a saved network name if any, and the access point |
| `link set up again while idle` | Only if `IDLE_REKEY` is on: times the arm re-added the remote as a peer |
| `ESP-NOW ...` | The raw counters from the radio driver (`ESPNow.stats()`) |

The raw ESP-NOW counters don't count sends one for one on this firmware. Measured on the
remote, a send acked on its first try can still add 0-2 to both "send results" and
"failed results". So don't read a "% lost" from them. Compare two readings taken a short
time apart instead, and use `reply sends while moving` and `replies given up on` for the
arm's own sends.

The arm sometimes leaves fields out (`ignored`, power save, Wi-Fi on/off,
`replies given up on`, the incidents, the moving counts), because the reply has to fit in
one radio message. The memory and Wi-Fi interface lines come from a separate request, so
they are always complete.

**Self-recovery.** When a reply fails all 3 tries, the arm works to get its radio
back. After each step it waits 1.6 s to hear the remote again:

1. Restart ESP-NOW.
2. Restart Wi-Fi, then ESP-NOW.

If neither works, the arm carries on and starts again at step 1 with the next failed
reply. It never restarts itself: a third step that rebooted into radio mode
(`RECOVER_REBOOT` in `arm_radio.py`) is off, because it moved the arm without warning.
(When on, it saves the joint angles to `/arm_hold.json` first and puts them back after
that reboot only.)

**The snapshot.** Every 30 s, while nothing is moving, the arm overwrites one small file,
`/arm_diag.json`, with the same counters `debug` shows (always under 250 bytes). It skips
this if the flash has less than 64 KB free, and `uninstall` removes the file. After an
outage that needed the arm switched off and on, the "last snapshot from before this
boot" shows what the arm saw just before.

## Radio settings

The link uses one 2.4 GHz Wi-Fi channel (default 1). Wi-Fi networks on the same or a
nearby channel (up to 4 channels away) can collide with it; 1, 6 and 11 are the usual
choices. Change the channel, transmit power or encryption on both boards over the radio,
with USB in the remote only:

```bash
python arm_serial.py radio-set 6             # channel 6
python arm_serial.py radio-set 1 8.5         # channel 1, 8.5 dBm (2-20; the default is 20)
python arm_serial.py radio-set 1 --plain     # unencrypted, for testing only
python arm_serial.py radio-set 1 --encrypted # back to encrypted
```

The arm saves the new settings in `/arm_radio.json` and restarts on them, the remote
follows, and the PC saves them in `.radio_pair.json` once the arm answers. If the arm does
not answer within 2 minutes, the remote goes back to the old settings and the error says
which ones the arm saved: run `radio-set` with those to move the remote to it.

## Radio test

`radio-test` measures the bare radio link, with nothing else running on the arm:

```bash
python arm_serial.py radio-test 30           # 30 s at 50 packets a second, echoed back
python arm_serial.py radio-test 30 20        # 20 packets a second
python arm_serial.py radio-test 30 --no-echo # one way only: does the arm's radio hear them?
python arm_serial.py radio-test-result --prev
```

- **The arm** restarts into radio-test mode (dim purple light): only Wi-Fi and ESP-NOW
  run, with no arm library, servo timer, diagnostics or recovery. **The servos are not
  driven, so the joints may sag; put the arm in a resting position first.** It counts each
  numbered test packet and echoes it back, then saves its counts and restarts into radio
  mode by itself (the joints go home).
- **Switching the arm off and on always starts radio mode.** The test request is removed
  before the test begins, and a power-on never starts a test.
- **The remote** sends numbered packets at the rate asked and counts, per second, how many
  the arm's radio acknowledged and how many came back. Each unacknowledged send blocks
  while the radio retries (about 80 ms), so the real rate can be lower than asked.
- **The PC** prints both sides: totals, per-second low / average / high, the seconds in
  which nothing got through, the longest gap, signal strength, and the arm's Wi-Fi state as
  the test started. The raw data goes to `.radio_test_last.json` (git-ignored). If the
  arm's test mode stayed silent, the remote's half still says whether the arm's radio
  acknowledged anything while it waited.
- **The arm keeps** its last result and the one before (`radio-test-result --prev`).

The remote's heartbeat keeps running during a test; the arm counts those as "other
messages".

## Known problem: deaf spells

The radio link loses packets and has "deaf spells": for anything from a few seconds to
two minutes the arm's radio acknowledges nothing, although the arm keeps running. They
come and go, after a power-on, a restart or in the middle of a session. Then commands fail
with "no radio contact" or are slow, and `debug` shows heartbeat gaps and recovery
incidents. When the arm hears the remote again, its first reply usually fails and the
Wi-Fi restart step brings the link back.

Measured so far (2026-11-11 to 11-22; one or a few short runs each):

- **Not the cause:** the arm's start-up order, power saving, the receive-buffer size,
  the idle re-key, the transmit power, the channel (1 and 6), encryption, constant traffic
  (a heartbeat at 10 a second), and memory (the system heap's exact low point did not
  move during a 20 s spell). Restarting the remote doesn't end a spell; the arm side is
  the stuck one.
- **Not the rest of radio mode:** in radio-test mode, with only the radio running, about
  30-40 % of packets went unacknowledged even at a strong signal, and one test heard
  nothing at all for 30 s from start-up.
- **Not possible to test from MicroPython on this firmware:** station-only Wi-Fi (the
  access point always comes back with the station), and whether the firmware has
  Bluetooth running (no `bluetooth` module; Bluetooth shares the radio). Espressif's
  ESP32-C3 coexistence table calls ESP-NOW reception stable only in station mode.
- **Open:** interference from nearby networks (try `radio-test` on channels 1, 6 and 11,
  or in another room), and how the stock firmware sets up its radio on the arm. On the
  stock remote: channel 1, power saving off, 20 dBm, 2 peers, both unencrypted.

The arm's battery ran flat once during these measurements, so some numbers may have been
taken on a weak battery.

## Space on the board

Measured on the receiver (ESP32-C3, MicroPython 1.23):

| Resource                         | USB mode (beside the stock firmware) | Radio mode, arm and radio running |
| -------------------------------- | ------------------------------------ | --------------------------------- |
| Free MicroPython RAM             | 30.5 KB                              | **~43 KB** (2026-11-22, with the 2048-byte ESP-NOW buffer; 60.5 KB with the first, smaller radio code) |
| Flash free (of 956 KB)           | 768 KB                               | ~750 KB (not re-measured since the radio code grew) |

Our files in flash:

| File                        | Size                                      |
| --------------------------- | ----------------------------------------- |
| `cyberbrick_arm.mpy`        | 7.0 KB                                    |
| `arm_radio.mpy`             | 11.3 KB                                   |
| `/lib/rc_main.mpy`          | 1.3 KB                                    |
| Settings, version, diagnostics and radio-test files | < 2 KB            |
| `*_old.mpy` after an update | up to 18.5 KB more                        |
| Remote: `arm_bridge.mpy`    | 3.4 KB, plus the 1.3 KB hook              |

The system heap that the Wi-Fi driver uses is separate from MicroPython's; `debug` shows
both. On the arm it stays at about 41 KB free.

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

## Limitations

- Each joint's fully extended position is set with the other joint at home. Whether the
  upper arm can reach the table also depends on where the lower arm is, so leave a few
  degrees spare.
- Each command opens the serial port again (to the remote, or to the arm over USB),
  which adds about a second. `shell` keeps it open.
- Radio mode and the stick remote take turns. `remote` gives the arm to the sticks until
  it is switched off and on. While the remote is the radio bridge, its sticks do nothing.
- Pairing a different remote, or removing radio mode completely (`uninstall`), needs the
  arm's USB cable again.
- The radio link has intermittent deaf spells: see
  [Known problem: deaf spells](#known-problem-deaf-spells).

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
| "No reply from the arm" now and then     | Run `debug` right away, before switching anything off, to see which side lost the messages ([Radio diagnostics](#radio-diagnostics)). If nearby Wi-Fi uses the same channel, try a clearer one ([Radio settings](#radio-settings)). See also [Known problem: deaf spells](#known-problem-deaf-spells) |
| Commands slow, or "no radio contact" for up to two minutes, then fine again | A deaf spell ([Known problem](#known-problem-deaf-spells)). Wait for it to end; the arm's Wi-Fi restart step usually brings the link back. `debug` afterwards shows the heartbeat gap |
| Home pose is wrong                       | `arm-calibrate`, then `home`                                 |
| A joint can't extend far enough; its home is mid-way (e.g. servo 100) | `arm-align`: re-seats the arm part on the servo so home is at servo 0 |
| A joint won't move by hand / feels stuck | Normal while the battery is on: the servo holds its angle. Move it with commands (or `+`/`-` in `arm-calibrate`), or `release` to let the joints go limp |
| A joint doesn't move when commanded, or buzzes | Check the battery is on and charged; the joint may be pressing on something, so step it back the other way |
| Radio: "no reply from the arm"           | The arm's radio hears the remote, but nothing came back for 10 s, even after the remote set up the link again. If you ran `remote`, the arm stays in stick mode until it is switched off and on. Otherwise run `debug` first, then switch the arm off and on, then run `debug` again: its snapshot shows what the arm saw before the restart. If `debug` shows power save other than 0, say so in a bug report. |
| Radio: the remote hung while starting, or `debug` says `gentle start` | Switch it off and on; it then starts Wi-Fi the gentle way. To try the full reset again, run `radio-pair` with USB in the remote |
| Radio: "request too long"                | A request over ESP-NOW's 250-byte limit was refused before sending; nothing reached the arm. Shorter arguments, or report it as a bug |
| `radio-test`: "no echo from the arm's test mode" | The arm's test mode stayed silent (a deaf spell, or the arm is off). The printout still shows whether its radio acknowledged anything; the arm goes back to radio mode by itself after the test time |
| Radio: "no radio contact with the arm, even after restarting the remote" | Nothing reached the arm. The PC already restarted the remote once (its radio can jam, for example after a soft reset from mpremote or Thonny). Check the arm is switched on and move the remote closer. Then run `debug`: if the `Remote Wi-Fi` line shows `OFF` or a channel other than the pair's, the remote's radio is not where the arm listens; run `radio-pair` with USB in the remote. |
| Radio: "Could not find the CyberBrick remote" | Plug USB into the paired remote, or pass `--port`       |
| Radio: "this board is a receiver"        | The cable is in the arm; move it to the remote               |
| Radio: stick remote does nothing         | The remote is in bridge mode: run `remote` to hand back to the sticks |
| Could not find the board (USB)           | Check the USB goes to the receiver; pass `--port`            |
| "This board is a CyberBrick transmitter" | USB is in the remote; use the radio, or move it to the arm's receiver |
| Commands succeed but nothing moves       | Switch the battery on                                        |
| `status` shows `timer_error`             | `python arm_serial.py restart`                               |
| Timed out, MemoryError, or firmware still starting (USB) | `reset`, wait 5 seconds, try again (after a MemoryError, also run `install`) |
| Permission denied on `/dev/ttyACM0`      | Add your user to the `dialout` group                         |
| Base turns the wrong way or stops short  | `base-tune`                                                  |
| Claw stops short or keeps pushing        | `claw-calibrate`                                             |
