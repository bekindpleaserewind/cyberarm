<p align="center">
  <img src="contrib/cyberarm.png" alt="OpenClaw AI robotic arm" width="100%">
</p>

# OpenClaw AI Robotic Arm

Control the [Robotic Arm - CyberBrick](https://makerworld.com/en/models/3379322-robotic-arm-cyberbrick)
from your PC and let an [OpenClaw](https://github.com/openclaw/openclaw) AI agent drive it
in plain language: *"turn left a bit and open the claw"*, *"pick up the block on the right
and hand it to me"*, *"wave"*.

> **The arm:** this project was built for the
> [Robotic Arm - CyberBrick](https://makerworld.com/en/models/3379322-robotic-arm-cyberbrick)
> model on MakerWorld, a 3D-printed desktop arm powered by the
> [CyberBrick](https://makerworld.com/en/cyberbrick) electronics kit. Print and assemble
> it from that page first, then use this project to control it from your PC or an AI agent.

The project has three parts:

- **Board code** (MicroPython):
  - [src/cyberbrick_arm.py](src/cyberbrick_arm.py) runs on the arm's receiver board. It
    drives the servos and the motor, keeps track of where everything is, and enforces the
    safety limits.
  - [src/arm_radio.py](src/arm_radio.py) and [src/arm_bridge.py](src/arm_bridge.py) carry
    commands over the radio, from the CyberBrick remote to the arm.
- **PC driver** ([src/arm_serial.py](src/arm_serial.py)): a command-line tool that sends
  commands to the arm over the radio (through the remote) or over USB. You can use it on
  its own, or have it print JSON for other programs.
- **OpenClaw plugin** ([openclaw-plugin/](openclaw-plugin/)): turns the driver into
  agent tools, and adds skills that teach the agent how to use the arm.

## Features

- **Arm joints:** move the lower and upper arm at any speed, one at a time or together,
  by position: 0 = home (folded) to 180 = fully extended. `arm-calibrate` records which
  servo angles those are for your arm, so the arm never drives past folded or extended,
  and the agent knows which way is up.
- **Claw:** open it fully, close it fully, or stop part way (for example half open).
- **Base:** face an angle or turn by a number of degrees. It never goes more than
  **90° left or right of centre**.
- **Position tracking without sensors:** the base and claw positions are estimated from
  timing, calibrated once, and saved between runs.
- **AI control:** 10 OpenClaw tools and 3 skills, covering basic moves, pick-and-place,
  hand-over, gestures (wave, nod, shake, scan) and troubleshooting.
- **Safe by default:**
  - The 90° base limit is checked in three places: the tool schema, the plugin and the
    board.
  - The joints have configurable limits.
  - Commands run one at a time.
- **Wireless:** after a one-time USB install, the arm takes commands over the radio
  through the CyberBrick remote, so its USB cable is never needed again. Updates and
  calibration work over the radio too.
- **Remote still works:** no stock file is replaced. `remote` hands the arm back to the
  stick remote. If radio mode ever fails to start, the arm falls back to the stock
  firmware by itself.

## How it works

```mermaid
flowchart LR
    U[You] -->|chat| A[OpenClaw agent]
    A -->|tool call| P[cyberbrick-arm plugin]
    P -->|runs| D[arm_serial.py --json]
    U -.->|or run it yourself| D
    D -->|USB| R[CyberBrick remote: arm_bridge.py]
    R -->|ESP-NOW radio, encrypted| B[arm_radio.py + cyberbrick_arm.py on the receiver]
    D -.->|or USB serial, raw REPL| B
    B --> S1[S1 lower arm]
    B --> S2[S2 upper arm]
    B --> S3[S3 claw]
    B --> M2[M2 base motor]
```

The receiver board and the remote are ESP32-C3 boards running MicroPython, and the
CyberBrick kit already links them over ESP-NOW radio.

- **In radio mode**, a small boot hook starts `arm_radio.py` on the arm instead of the
  stock firmware. `arm_serial.py` sends each command to the remote over USB, and the remote
  sends it to the arm over the radio. The arm only accepts a fixed list of arm commands,
  only from its paired remote, and answers with its status when the move is done.
- **Over USB**, `arm_serial.py` interrupts the stock firmware, loads the arm library, and
  sends each command as a short MicroPython snippet.

Either way, a 10 ms timer on the board moves the joints smoothly, easing each move in and
out, and stops the claw and base when they reach their targets.

## Hardware

The code is written for the
[Robotic Arm - CyberBrick](https://makerworld.com/en/models/3379322-robotic-arm-cyberbrick)
model from MakerWorld. Print and build it as described on the model page, then wire it to
the CyberBrick receiver like this:

| Port on receiver | Part      | Type                                       |
| ---------------- | --------- | ------------------------------------------ |
| S1               | Lower arm | 180° servo                                 |
| S2               | Upper arm | 180° servo                                 |
| S3               | Claw      | 360° continuous servo                      |
| M2               | Base      | DC motor                                   |

You also need:

- the **CyberBrick remote** (transmitter) with a USB cable to your PC, for radio control;
- a USB cable to the arm's **receiver** board, once, for the first install (or all the
  time, if you use USB mode);
- the arm's battery, switched on, because USB alone does not power the servos and motor.

Tested on Linux (WSL, `/dev/ttyACM0`).

If you built a different CyberBrick arm, or wired it differently, change the port numbers
and joint limits in the `CONFIG` section of
[src/cyberbrick_arm.py](src/cyberbrick_arm.py). Then send it to the arm with
`arm_serial.py update` (radio). Over USB, the next command installs it by itself.

## Quick start

Run the driver from the `src/` folder. Switch the arm's battery on before you start:
USB powers the boards but not the servos or the motor.

### 1. Install the PC driver

```bash
cd src
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt    # pyserial, and mpy-cross for installing board code
```

On Linux your user must be in the `dialout` group to open `/dev/ttyACM0`. On WSL, attach
each board to Linux with `usbipd.exe list` and `usbipd.exe attach --wsl --busid <id>`;
the remote and the arm are separate USB devices, so attach whichever one is plugged in.

### 2. Set up wireless control (once)

1. **Pair the remote.** Plug USB into the CyberBrick **remote** (the transmitter) and run:

   ```bash
   python arm_serial.py radio-pair       # optional: radio-pair CHANNEL (1-13, default 1)
   ```

   This makes the link keys (saved in `src/.radio_pair.json`, which is git-ignored; keep it
   private) and installs the radio bridge on the remote.

2. **Install radio mode on the arm.** Plug USB into the **arm's receiver** board and run:

   ```bash
   python arm_serial.py radio-install
   ```

   The arm reboots into radio mode, and its light turns dim blue. Unplug the arm's USB
   cable; it is not needed again.

3. **Check the link.** With USB in the remote:

   ```bash
   python arm_serial.py ping
   ```

From now on every command goes over the radio automatically.

> **Prefer a cable?** Skip this step and plug USB into the arm. Commands then go over
> USB (the first one saves the arm library on the arm), and the stock stick remote still
> works when the arm is not plugged in.

### 3. Calibrate (once)

These are interactive: watch the arm and answer at each step. They save their results on
the arm, so they survive restarts and updates.

```bash
python arm_serial.py arm-calibrate     # required: each joint's home and fully extended position
python arm_serial.py base-tune         # required: sets 90° base turns (the defaults are placeholders)
python arm_serial.py claw-calibrate    # recommended: times a full claw open and close
python arm_serial.py calibration       # show what the arm is using
```

`base-tune` needs no stopwatch: the arm times each turn itself and you say how far the
base went (for example `70`), until a turn is exactly 90°. Mark straight ahead and 90°
on the table first. (`base-calibrate` is the old name and now runs `base-tune`.)

`arm-calibrate` moves the joints from the keyboard (`+`, `-`, `+20`, or a servo angle),
not by hand: while the battery is on, the servos hold their position firmly.

If a joint is folded in the middle of its servo's range (for example at servo 100), it
can only extend 80° before the servo runs out. Run `python arm_serial.py arm-align`
first: it has you take that arm part off its servo and refit it folded at the servo's
end, then runs `arm-calibrate`.

### 4. Drive the arm

```bash
python arm_serial.py status
python arm_serial.py move 90 120       # joint positions: 0 = home (folded) .. 180 = fully extended
python arm_serial.py claw open
python arm_serial.py turn -45          # face 45° left
python arm_serial.py gesture wave
python arm_serial.py home
```

See [Commands](#commands) for an overview, and [src/README.md](src/README.md) for every
command, the
calibration steps, radio details, the measured RAM and flash budget, the Python API and
troubleshooting.

### 5. Let OpenClaw control it

```bash
cd openclaw-plugin
npm install
openclaw plugins install --link "/path/to/openclaw-plugin"
```

Allow the plugin's tools in `~/.openclaw/openclaw.json`:

```json5
{
  tools: {
    alsoAllow: ["cyberbrick-arm"],
  },
}
```

Then restart the gateway and start talking to it:

```bash
openclaw gateway restart
openclaw agent -m "Wave with the arm"
```

You can also call a skill directly in chat, for example `/cyberbrick-arm-routines wave`.
See [openclaw-plugin/README.md](openclaw-plugin/README.md) for the tool list and settings.

## Commands

Usage: `python arm_serial.py [OPTIONS] COMMAND [ARGS]`, and `python arm_serial.py help`
lists them all. Joint positions run from 0 (home, folded) to 180 (fully extended).
`SPEED` is 1-100 (default 20, about 90°/s).

| For | Commands |
| --- | -------- |
| Joints | `home`, `lower POS`, `upper POS`, `move LOWER UPPER`, `gesture wave\|nod\|shake`, `release` |
| Claw | `claw open\|close\|0.0-1.0`, `claw-spin SPEED`, `claw-set open\|closed` |
| Base | `turn ANGLE`, `turn-by DEGREES`, `base SPEED [SECONDS]`, `base-set [ANGLE]` |
| Everyday | `status`, `stop`, `demo`, `shell` |
| Calibration | `arm-calibrate`, `arm-align`, `base-tune`, `claw-calibrate`, `calibration` |
| Wireless | `radio-pair`, `radio-install`, `ping`, `update`, `remote` |
| Board | `reset`, `install`, `uninstall`, `probe`, `raw CODE` |

Every command and option is described in
[src/README.md](src/README.md#arm_serialpy).

## Agent tools

| Tool                         | Does                                              |
| ---------------------------- | ------------------------------------------------- |
| `cyberbrick_arm_status`      | Report joint angles, claw opening and base angle  |
| `cyberbrick_arm_rotate_base` | Turn left or right by 1-90°, stopping at the limit |
| `cyberbrick_arm_face_angle`  | Point the base at -90..90° (0 = straight ahead)   |
| `cyberbrick_arm_move`        | Move the lower and/or upper joint                 |
| `cyberbrick_arm_gesture`     | Wave, nod or shake in one smooth motion           |
| `cyberbrick_arm_open_claw`   | Open the claw                                     |
| `cyberbrick_arm_close_claw`  | Close the claw                                    |
| `cyberbrick_arm_set_claw`    | Open the claw part way                            |
| `cyberbrick_arm_home`        | Return to the home pose                           |
| `cyberbrick_arm_stop`        | Stop immediately                                  |

## Project layout

```
.
├── contrib/
│   └── cyberarm.png           header image
├── src/
│   ├── cyberbrick_arm.py      MicroPython arm library (runs on the board)
│   ├── arm_radio.py           radio mode on the arm (ESP-NOW server)
│   ├── arm_bridge.py          radio bridge on the CyberBrick remote
│   ├── arm_boot.py            boot hook for both boards
│   ├── arm_serial.py          PC driver / CLI
│   ├── tests/                 radio and protocol tests (run on the PC)
│   ├── requirements.txt       pyserial, mpy-cross
│   └── README.md              driver and library documentation
└── openclaw-plugin/
    ├── index.ts               tool definitions
    ├── openclaw.plugin.json   plugin manifest and config schema
    ├── skills/                cyberbrick-arm, -routines, -setup
    └── README.md              plugin documentation
```

## Limitations

- The base and claw have no position sensors. Their positions are estimates and can drift
  over many moves, or be wrong after the arm is moved by hand or with the remote.
  Re-centre the base with `arm_serial.py base-set 0`.
- Joint positions are a scale from folded to fully extended, not degrees. Until
  `arm-calibrate` is run they are raw servo angles (home 90), and the agent asks you
  which way a joint moves the first time it needs to know.
- Each joint's fully extended position is set with the other joint at home. Whether the
  upper arm can reach the table also depends on where the lower arm is, so leave a few
  degrees spare.
- Each command opens the serial port again (to the remote, or to the arm over USB),
  which adds about a second.
- Radio mode and the stick remote take turns. `remote` gives the arm to the sticks
  until it is switched off and on. While the remote is the radio bridge, its sticks do
  nothing.
- Pairing a different remote, or removing radio mode completely, needs the arm's USB
  cable again.

## Acknowledgements

- Arm design: [Robotic Arm - CyberBrick](https://makerworld.com/en/models/3379322-robotic-arm-cyberbrick)
  on MakerWorld.
- The serial connection code follows the approach of
  [unbug/CyberClaw](https://github.com/unbug/CyberClaw)'s CyberBrick driver.
- Built on the CyberBrick MicroPython firmware and its `bbl` servo and motor libraries.
