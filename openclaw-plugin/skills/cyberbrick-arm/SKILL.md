---
name: cyberbrick-arm
description: Control the CyberBrick robotic arm - turn (spin) the base left or right up to 90 degrees, move the lower and upper arm joints, open or close the claw, and wave, nod or shake. Use when the user asks the arm to move, point, turn, spin, reach, grab, release, open, close, wave, nod, shake its head, say hello, yes or no, go home or stop, or asks where the arm is.
metadata: {"openclaw": {"emoji": "🦾"}}
---

# CyberBrick robotic arm

You control a small desktop robotic arm through the `cyberbrick_arm_*` tools. Every tool
waits until the movement has finished, then returns the arm's new state.

## The arm

| Part      | What it does                                      | Range                                    |
| --------- | ------------------------------------------------- | ---------------------------------------- |
| Base      | Turns the whole arm left/right (DC motor)         | -90 (full left) .. 0 (centre) .. 90 (full right) |
| Lower arm | Shoulder joint, servo S1                          | position 0 = home (folded) .. 180 = fully extended |
| Upper arm | Elbow joint, servo S2                             | position 0 = home (folded) .. 180 = fully extended |
| Claw      | Gripper                                           | 0 = closed .. 1 = fully open             |

Left and right are as seen from **behind** the arm, looking the way it faces at centre.

## Tools

| Tool                         | Use it to                                                   |
| ---------------------------- | ----------------------------------------------------------- |
| `cyberbrick_arm_status`      | Check where everything is. Nothing moves.                   |
| `cyberbrick_arm_rotate_base` | Turn/spin `left` or `right` by 1-90 degrees from where it is |
| `cyberbrick_arm_face_angle`  | Point the base at an exact angle, -90..90 (0 = straight ahead) |
| `cyberbrick_arm_move`        | Move `lower` and/or `upper` joint to a position 0-180 (optional `speed` 1-100) |
| `cyberbrick_arm_open_claw`   | Open the claw fully                                         |
| `cyberbrick_arm_close_claw`  | Close the claw fully                                        |
| `cyberbrick_arm_set_claw`    | Open the claw part way (`position` 0-1)                     |
| `cyberbrick_arm_gesture`     | `wave`, `nod` or `shake` in one smooth call, then back to the start pose (optional `times`, `speed`) |
| `cyberbrick_arm_home`        | Joints to position 0 (home, folded) and base to centre (claw unchanged; optional `speed`) |
| `cyberbrick_arm_stop`        | Stop base and claw now, hold joints where they are          |
| `cyberbrick_arm_set_base_reference` | Nothing moves: tell the arm where the base really faces (`angle`, default 0) |
| `cyberbrick_arm_set_claw_reference` | Nothing moves: tell the arm the claw is really `open` or `closed` |
| `cyberbrick_arm_check_link`  | Nothing moves: radio link health (reply time, signal, free RAM) |
| `cyberbrick_arm_restart`     | Restart the arm's control code after a board error (joints may go home) |

For wave, nod (yes) and shake (no), always use `cyberbrick_arm_gesture`: one call does the
whole gesture smoothly. See the `cyberbrick-arm-routines` skill for longer routines, and
`cyberbrick-arm-setup` for when to use the last four tools.

## Base rotation rule (hard limit)

The base must **never** turn more than **90 degrees left or right of centre**. That is the
whole working range: a half circle in front of the arm. Full spins are not possible.

- `cyberbrick_arm_rotate_base` is relative: "turn right 30" from 70 right would stop at
  the 90 right limit, and the result says so.
- When the user asks for more than the limit allows ("spin around", "turn 180",
  "do a full circle"), do not try to get there with several turns. Explain the 90 degree
  limit and offer the nearest allowed move (e.g. full left to full right sweep).
- To sweep from one side to the other, use `cyberbrick_arm_face_angle` with -90 then 90,
  not repeated relative turns.
- Check `base_angle` in the last result before a relative turn when it matters.

## How to work

1. If you do not know the current state (start of a conversation, or after an error),
   call `cyberbrick_arm_status` first.
2. Translate the request into one tool call per step. "Turn left a bit" = about 20-30
   degrees; "turn left" with no amount = 45; "all the way left" = face angle -90.
   "Open the claw a little" = `set_claw` 0.3; "half open" = 0.5.
3. For anything near people, objects or the table, prefer slower joint moves
   (`speed` 10-20) and moves of 30 degrees or less at a time.
4. After each step read the returned state and tell the user briefly what the arm did,
   in plain words ("Turned 45 degrees left; the claw is open.").
5. If the user says "stop", "wait" or anything urgent, call `cyberbrick_arm_stop`
   immediately before anything else.

## Joint positions and directions

Each joint is driven by a **position from 0 to 180**: 0 = home (folded, at rest), 180 =
fully extended, 90 = half way. These are not degrees: the arm turns them into the right
servo angle for each joint, and it can never be driven past folded or fully extended.
`cyberbrick_arm_home` folds both joints back to 0.

`cyberbrick_arm_status` also says which way extending each joint moves the arm, for
example: "Lower arm: position 0 = home (folded), 180 = fully extended; extending leans it
forward. Upper arm: ...; extending lifts the claw." These come from the user's
`arm-calibrate` run.

- "Reach out" or "stretch" means higher positions; "pull in" or "fold up" means lower.
- For "lift" or "lower", use the direction the status gives.
- If the status says a joint is not calibrated, its position is a raw servo angle and
  the direction is unknown. Make a small test move (15 from the current position) the
  first time you need it, and ask the user which way it went. Suggest they run
  `arm_serial.py arm-calibrate` (the `cyberbrick-arm-setup` skill).

## Limits of what the arm knows

The base and claw have no position sensors. Their positions are **estimates** from how
long they have moved, so:

- `base_angle` and `claw` can drift a few degrees / percent over many moves. If the user
  says the arm is not where the status says, use the `cyberbrick-arm-setup` skill to
  re-centre it (`cyberbrick_arm_set_base_reference` / `cyberbrick_arm_set_claw_reference`).
- If the arm was moved by hand or with the CyberBrick remote, the estimates are wrong
  until re-centred.
- A claw holding an object stops before "closed"; the claw value still reads 0. That is
  normal.

## Errors

If a tool fails, tell the user the error in one sentence and use the
`cyberbrick-arm-setup` skill to troubleshoot. Do not keep retrying a failing move.
