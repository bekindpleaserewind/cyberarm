# CyberBrick Arm plugin for OpenClaw

Lets an OpenClaw agent control the CyberBrick robotic arm: turn the base (never more than
90° left or right of centre), move the two arm joints, and open or close the claw.

The tools run the PC driver `../src/arm_serial.py`, which talks to the arm's receiver board
over USB serial.

## Tools

| Tool                         | Does                                                |
| ---------------------------- | --------------------------------------------------- |
| `cyberbrick_arm_status`      | Report base angle, joint angles, claw opening       |
| `cyberbrick_arm_rotate_base` | Turn `left`/`right` by 1-90° (stops at the ±90° limit) |
| `cyberbrick_arm_face_angle`  | Point the base at -90..90° (0 = straight ahead)     |
| `cyberbrick_arm_move`        | Lower and/or upper joint to 0-180°, optional speed  |
| `cyberbrick_arm_open_claw`   | Open the claw fully                                 |
| `cyberbrick_arm_close_claw`  | Close the claw fully                                |
| `cyberbrick_arm_set_claw`    | Open the claw part way (0-1)                        |
| `cyberbrick_arm_home`        | Joints to 90°, base to centre                       |
| `cyberbrick_arm_stop`        | Stop base and claw, hold joints                     |

The 90° base limit is enforced three times: the tool schemas, the tool code, and the
board library itself (`BASE_LIMIT_DEG` in `src/cyberbrick_arm.py`).

## Skills

- `cyberbrick-arm`: how to use the tools, the base limit, and the arm's ranges.
- `cyberbrick-arm-routines`: pick and place, hand-over, wave, nod, scan.
- `cyberbrick-arm-setup`: calibration and troubleshooting.

## Setup

1. Prepare the arm (receiver board plugged into USB, battery on):

   ```bash
   cd ~/Cyberbrick/src
   .venv/bin/python arm_serial.py install          # store the arm library on the board
   .venv/bin/python arm_serial.py base-calibrate   # time 90° turns (needed once)
   ```

   `base-calibrate` is interactive: centre the base, then press Enter when it has turned
   a quarter turn and again when it is back at centre. Until this is done, the base uses a
   guessed timing and its 90° may not be a real 90°.

2. Install the plugin:

   ```bash
   cd ~/Cyberbrick/openclaw-plugin
   npm install
   openclaw plugins install --link ~/Cyberbrick/openclaw-plugin
   ```

3. Allow its tools. The `coding` tool profile only includes built-in tools, so add the
   plugin to `tools.alsoAllow` in `~/.openclaw/openclaw.json`:

   ```json5
   {
     tools: {
       profile: "coding",
       alsoAllow: ["cyberbrick-arm"],
     },
   }
   ```

4. Restart the gateway and check:

   ```bash
   openclaw gateway restart
   openclaw plugins inspect cyberbrick-arm --runtime
   ```

## Config

Optional, under `plugins.entries.cyberbrick-arm.config`:

| Key              | Default                                         |
| ---------------- | ----------------------------------------------- |
| `driverPath`     | `../src/arm_serial.py` next to this plugin      |
| `python`         | `.venv/bin/python` next to the driver, else `python3` |
| `port`           | auto-detect (e.g. `/dev/ttyACM0`, `COM5`)       |
| `timeoutSeconds` | `90`                                            |

## Notes

- The base and claw have no position sensors; their positions are estimated from timing
  and saved in `src/.arm_state.json` so they survive a board reboot. If the base was moved
  by hand or with the remote, centre it and run `arm_serial.py base-set 0`.
- Commands are queued; only one uses the serial port at a time.
- Each command takes about a second longer than the movement itself to open the serial
  port.
