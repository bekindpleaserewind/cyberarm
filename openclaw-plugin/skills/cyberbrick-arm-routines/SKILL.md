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

The "reach" and "lift" poses below use **REACH** and **LIFT** placeholders. Use the joint
directions from the `cyberbrick-arm` skill notes (or that you learned with the user) to
choose them. Start with REACH = lower 120 / upper 60 and LIFT = lower 90 / upper 90 only if
the user confirms higher lower-arm angles lean forward; otherwise mirror them (lower 60 /
upper 120). Adjust in steps of 10 degrees based on the user's feedback, and reuse what
works for the rest of the conversation.

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

- **Wave**: `cyberbrick_arm_move` upper 60 then upper 120, three times, speed 40; then
  upper 90.
- **Nod (yes)**: lower 80 then lower 100, twice, speed 30; then lower 90.
- **Shake (no)**: face angle -20 then 20, twice; then face angle 0.
- **Scan / look around**: face angle -90, then 0, then 90, then 0. Pause one step at each.
- **Clap**: open claw, close claw, twice.

## Finish

End every routine with `cyberbrick_arm_home` unless the user wants the arm left in place,
and summarise what was done in one or two sentences.
