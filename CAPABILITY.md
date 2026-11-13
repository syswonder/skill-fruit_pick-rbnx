---
description: Pick one fruit the camera sees on the table, put it at a named location or back where it was, and return home.
---

# Fruit pick

`pick_fruit(destination)` moves to the home (photo) pose, opens the gripper,
compares one camera frame with the empty-table background, picks the fruit
nearest the middle of the view, releases it at `destination` and returns home.

- Input: `destination` is a configured location name, or empty to put the
  fruit back where it was (a test of the whole grasp).
- Fruit is recognised by being round and within the configured size range,
  not by kind; the camera is monochrome. One fruit per call; the result says
  how many more are in view.
- Precondition: `capture_background` was called with the table empty, and the
  arm is near home. Call `capture_background` again after the table, lighting
  or camera exposure changes.
- Success: every motion reached its target by measured feedback and the
  gripper reported that it is holding the fruit before the arm lifted it.
- Failure leaves the arm where it stopped. If the jaws closed on nothing they
  open again first.
- Status/cancel: `pick_fruit/status` and `pick_fruit/cancel` with the returned
  `run_id`. Cancel stops further commands; the arm completes its current
  motion and the gripper keeps its state.
