---
name: cyberbrick-arm
description: Control the CyberBrick robotic arm - turn (spin) the base left or right up to 90 degrees, move the lower and upper arm joints, and open or close the claw. Use when the user asks the arm to move, point, turn, spin, reach, grab, release, open, close, go home or stop, or asks where the arm is.
metadata: {"openclaw": {"emoji": "🦾"}}
---

# CyberBrick robotic arm

You control a small desktop robotic arm through the `cyberbrick_arm_*` tools. Every tool
waits until the movement has finished, then returns the arm's new state.

## The arm

| Part      | What it does                                      | Range                                    |
| --------- | ------------------------------------------------- | ---------------------------------------- |
| Base      | Turns the whole arm left/right (DC motor)         | -90 (full left) .. 0 (centre) .. 90 (full right) |
| Lower arm | Shoulder joint, servo S1                          | 0-180 degrees, 90 = home                 |
| Upper arm | Elbow joint, servo S2                             | 0-180 degrees, 90 = home                 |
| Claw      | Gripper                                           | 0 = closed .. 1 = fully open             |

Left and right are as seen from **behind** the arm, looking the way it faces at centre.

## Tools

| Tool                         | Use it to                                                   |
| ---------------------------- | ----------------------------------------------------------- |
| `cyberbrick_arm_status`      | Check where everything is. Nothing moves.                   |
| `cyberbrick_arm_rotate_base` | Turn/spin `left` or `right` by 1-90 degrees from where it is |
| `cyberbrick_arm_face_angle`  | Point the base at an exact angle, -90..90 (0 = straight ahead) |
| `cyberbrick_arm_move`        | Move `lower` and/or `upper` joint to an angle (optional `speed` 1-100) |
| `cyberbrick_arm_open_claw`   | Open the claw fully                                         |
| `cyberbrick_arm_close_claw`  | Close the claw fully                                        |
| `cyberbrick_arm_set_claw`    | Open the claw part way (`position` 0-1)                     |
| `cyberbrick_arm_home`        | Joints to 90/90 and base to centre (claw unchanged)          |
| `cyberbrick_arm_stop`        | Stop base and claw now, hold joints where they are          |

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

## Joint directions

Which way a joint moves when its angle goes up depends on how the servo is mounted, so
it is not built in. If the notes below are filled in, use them. Otherwise, the first time
you need a direction (e.g. "reach forward", "lift up"), make a small test move (15
degrees from the current angle), ask the user which way it went, and remember the answer
for the rest of the conversation.

<!-- Fill these in for your arm, e.g. "Lower arm: higher angle leans forward". -->
- Lower arm: higher angle = (unknown)
- Upper arm: higher angle = (unknown)

## Limits of what the arm knows

The base and claw have no position sensors. Their positions are **estimates** from how
long they have moved, so:

- `base_angle` and `claw` can drift a few degrees / percent over many moves. If the user
  says the arm is not where the status says, use the `cyberbrick-arm-setup` skill to
  re-centre it.
- If the arm was moved by hand or with the CyberBrick remote, the estimates are wrong
  until re-centred.
- A claw holding an object stops before "closed"; the claw value still reads 0. That is
  normal.

## Errors

If a tool fails, tell the user the error in one sentence and use the
`cyberbrick-arm-setup` skill to troubleshoot. Do not keep retrying a failing move.
