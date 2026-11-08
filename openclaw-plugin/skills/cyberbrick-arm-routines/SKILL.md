---
name: cyberbrick-arm-routines
description: Multi-step CyberBrick arm routines - pick up and place an object, move something from one side to the other, hand an object to the user, wave, nod, and scan side to side. Use when the user asks the arm to do a task or gesture made of several moves rather than a single movement.
metadata: {"openclaw": {"emoji": "🤖"}}
---

# CyberBrick arm routines

These routines combine the `cyberbrick_arm_*` tools (see the `cyberbrick-arm` skill for
what each tool does and the joint ranges). Run the steps one at a time, read each result,
and stop the routine if a step fails or the user says stop.

All base angles stay within **-90 (left) .. 90 (right)**. Never plan a step outside it.

## Before any routine

1. `cyberbrick_arm_status` to know the starting point.
2. If the user has not said where the object is, ask: left, right or in front, and roughly
   how far (degrees) to the side.
3. Use joint `speed` 15-20 while carrying something so it does not get flung.

Joint positions run from 0 (home, folded) to 180 (fully extended); see the
`cyberbrick-arm` skill. The "reach" and "lift" poses below use **REACH** and **LIFT**
placeholders. Start with REACH = lower 100 / upper 140 (stretched out towards the table)
and LIFT = lower 60 / upper 90 (raised, clear of the table). Use the directions from
`cyberbrick_arm_status` to check they go the right way. Adjust in steps of 10 based on the
user's feedback, and reuse what works for the rest of the conversation.

## Pick up

1. `cyberbrick_arm_face_angle` toward the object (e.g. -45 for "front-left").
2. `cyberbrick_arm_open_claw`.
3. `cyberbrick_arm_move` to REACH, speed 15.
4. `cyberbrick_arm_close_claw`.
5. `cyberbrick_arm_move` to LIFT, speed 15.
6. Ask the user whether the object is in the claw if you cannot tell.

## Place

1. `cyberbrick_arm_face_angle` toward the drop spot.
2. `cyberbrick_arm_move` to REACH, speed 15.
3. `cyberbrick_arm_open_claw`.
4. `cyberbrick_arm_move` to LIFT.
5. `cyberbrick_arm_close_claw` (optional, to tidy up).

## Move an object from one side to the other

Pick up at the first angle, then place at the second. Both angles must be within
-90..90; "from the far left to the far right" = pick at -90, place at 90.

## Hand something to the user

Pick up, then `cyberbrick_arm_face_angle` 0, `cyberbrick_arm_move` to REACH, and tell the
user to take it. Wait for them to confirm before `cyberbrick_arm_open_claw`, then
`cyberbrick_arm_home`.

## Gestures

Use `cyberbrick_arm_gesture` for wave, nod and shake: **one call** does the whole
gesture. It raises the arm if needed, swings at one even speed and returns to the pose it
started in. Do not build these from separate `cyberbrick_arm_move` calls: each call
snaps, then the arm waits seconds for the next one, so it looks jerky and uneven.

- **Wave**: `cyberbrick_arm_gesture` gesture `wave` (3 swings by default).
- **Nod (yes)**: `cyberbrick_arm_gesture` gesture `nod`.
- **Shake (no)**: `cyberbrick_arm_gesture` gesture `shake`.
- Pass `times` for more or fewer swings, and `speed` (default 25) only if the user asks
  for a faster or slower gesture.
- **Scan / look around**: face angle -90, then 0, then 90, then 0. Pause one step at each.
- **Clap**: open claw, close claw, twice.

## Finish

End every routine with `cyberbrick_arm_home` unless the user wants the arm left in place,
and summarise what was done in one or two sentences.
