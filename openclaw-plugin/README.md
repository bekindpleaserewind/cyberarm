# CyberBrick Arm plugin for OpenClaw

Lets an OpenClaw agent control the CyberBrick robotic arm: turn the base (never more than
90° left or right of centre), move the two arm joints, open or close the claw, and gesture
(wave, nod, shake). It can also fix drifted base and claw estimates, check the radio link
and restart the arm after a board error.

Interactive or board-changing driver commands (calibration, pairing, `install`, `update`,
`remote`) and unsafe ones (`release`, `claw-spin`, `raw`) are left out: the agent tells the
user the command to run instead.

The tools run the PC driver `./src/arm_serial.py`. Once radio mode is set up, it sends
commands through the CyberBrick remote over the radio, so the arm needs no USB cable.
Before that, it talks to the arm's receiver board over USB serial.

## Tools

| Tool                         | Does                                                |
| ---------------------------- | --------------------------------------------------- |
| `cyberbrick_arm_status`      | Report base angle, joint positions (0 = home .. 180 = extended), claw opening, joint directions |
| `cyberbrick_arm_rotate_base` | Turn `left`/`right` by 1-90° (stops at the ±90° limit) |
| `cyberbrick_arm_face_angle`  | Point the base at -90..90° (0 = straight ahead)     |
| `cyberbrick_arm_move`        | Lower and/or upper joint to position 0 (home) - 180 (extended), optional speed |
| `cyberbrick_arm_open_claw`   | Open the claw fully                                 |
| `cyberbrick_arm_close_claw`  | Close the claw fully                                |
| `cyberbrick_arm_set_claw`    | Open the claw part way (0-1)                        |
| `cyberbrick_arm_gesture`     | `wave`, `nod` or `shake` in one smooth motion, then back to the start pose |
| `cyberbrick_arm_home`        | Joints to home (position 0), base to centre, optional speed |
| `cyberbrick_arm_stop`        | Stop base and claw, hold joints                     |
| `cyberbrick_arm_set_base_reference` | Tell the arm where the base really faces (re-centre after drift; nothing moves) |
| `cyberbrick_arm_set_claw_reference` | Tell the arm the claw is really open or closed (nothing moves) |
| `cyberbrick_arm_check_link`  | Radio: reply time, signal, free RAM, last restart cause |
| `cyberbrick_arm_restart`     | Restart the arm's control code after a board error |

The 90° base limit is enforced three times: the tool schemas, the tool code, and the
board library itself (`BASE_LIMIT_DEG` in `src/cyberbrick_arm.py`).

## Skills

- `cyberbrick-arm`: how to use the tools, the base limit, and the arm's ranges.
- `cyberbrick-arm-routines`: pick and place, hand-over, wave, nod, scan.
- `cyberbrick-arm-setup`: calibration and troubleshooting.

## Setup

1. Prepare the arm (battery on), from the `src` folder:

   ```bash
   python arm_serial.py radio-pair       # USB in the CyberBrick remote
   python arm_serial.py radio-install    # USB in the arm's receiver, once; then unplug it
   python arm_serial.py arm-calibrate    # joint home and fully extended positions (needed once)
   python arm_serial.py base-tune        # set 90° turns (needed once)
   ```

   For USB control instead, run `python arm_serial.py install` with USB in the arm.

   `base-tune` is interactive: centre the base, and the arm makes timed turns while you
   say how far each one went (for example `70`) until it turns exactly 90°. Until this is
   done, the base uses a guessed timing and its 90° may not be a real 90°.

2. Install the plugin:

   ```bash
   cd ./openclaw-plugin
   npm install
   openclaw plugins install --link /path/to/openclaw-plugin
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
| `driverPath`     | `./src/arm_serial.py` next to this plugin      |
| `python`         | `.venv/bin/python` next to the driver, else `python3` |
| `port`           | auto-detect (the remote's port in radio mode, e.g. `/dev/ttyACM0`, `COM5`) |
| `transport`      | `auto`: radio once `radio-install` was run, else USB (`radio`, `usb`) |
| `timeoutSeconds` | `90`                                            |

## Notes

- The base and claw have no position sensors; their positions are estimated from timing
  and saved in `src/.arm_state.json` so they survive a board reboot. If the base was moved
  by hand or with the remote, centre it and run `arm_serial.py base-set 0`.
- Commands are queued; only one uses the link at a time.
- Every tool in `index.ts` must also be listed under `contracts.tools` in
  `openclaw.plugin.json`. OpenClaw drops undeclared tools with only a diagnostic, so the
  agent never sees them.
- Each command takes about a second longer than the movement itself to open the serial
  port.
