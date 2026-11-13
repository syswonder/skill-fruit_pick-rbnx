# skill-fruit_pick-rbnx

Robonix skill that picks one fruit a camera sees on a table. It finds the
fruit by comparing a frame with a background of the empty table, places it
in the arm's base frame from a hand-eye calibration and a known table height,
grips it, and releases it at a named location or where it was. It drives any
providers of the standard contracts and never imports a vendor SDK:

| Provider | Contracts used |
|---|---|
| arm (`arm_provider_id`) | `robonix/primitive/arm/{pos_command, end_pose, joint_states}` |
| gripper (`gripper_provider_id`) | `robonix/primitive/gripper/{info, open, grip, state}` |
| camera (`camera_provider_id`) | `robonix/primitive/camera/{rgb, intrinsics}` (mono8) |

The skill activates only when all of them are offered (`pos_command` needs
`motion_enabled` on the arm); until then activation is deferred.

## Capabilities

| Contract | Request | Response |
|---|---|---|
| `robonix/skill/fruit_pick/pick_fruit` | `destination` | `run_id`, `detail` |
| `robonix/skill/fruit_pick/pick_fruit/status` | `run_id` | `state`, `detail` |
| `robonix/skill/fruit_pick/pick_fruit/cancel` | `run_id` | `accepted`, `detail` |
| `robonix/skill/fruit_pick/capture_background` | | `detail` |

`pick_fruit` is the Executor's long-task shape: the Executor polls `status`
and forwards plan cancellation to `cancel`.

## Sequence

`capture_background` moves to `home`, opens the jaws and stores one frame in
`rbnx-build/data/<instance>-background.png` with the pose it was taken from.
It survives restarts.

`pick_fruit`:

1. Settle on `home` and open the jaws (they must report reaching the open
   endpoint).
2. Take the first frame acquired after that and difference it against the
   background. Blobs that are round enough and, located on the table plane,
   within `fruit_diameter_m` are fruit; the one nearest the image centre is
   picked. The run fails without moving if there is none, if the arm is not
   where the background was taken, or if the fruit plus `clearance_m` per side
   is wider than the gripper's `info.max_opening_width_m`.
3. Move above the fruit at home's height, descend vertically until the
   fingertips are `grasp_below_centre_m` below its centre (never lower than
   `min_fingertip_height_m` above the table), and `grip`. Anything but
   HOLDING opens the jaws and fails the run with the arm at the fruit.
4. Lift back to home's height and release: back where it was
   (`release_lift_m` above the grasp height), or above a named location, down
   to it, open, up.
5. Return home.

Every move is a `pos_command` target with home's orientation; it completes
when `end_pose` is within tolerance with the joints stationary. A failure or
cancel stops further commands.

## Calibration it relies on

- `camera_in_tcp`: the hand-eye result, camera optical frame in the arm's
  `end_pose` frame.
- `table_z_m`: the table surface in the base frame. The camera gives no depth;
  an error here moves the fruit along the camera ray and scales its size.
- `fingertip_offset_m`: measured from the flange face to the fingertips.
- The camera's intrinsics come from its `intrinsics` contract.

See `config.spec` for every field.

`ArmClient.move` and the run bookkeeping in `runner.py` follow the
`pick_place` skill of the AUBO i5H cell; a fix to either belongs in both until
they share a library.

## Build and tests

`scripts/build.sh` runs `rbnx codegen --mcp --ros2` and colcon-builds the
generated `gripper` interfaces, so it needs ROS 2 (`RBNX_ROS_SETUP`, default
Humble). The runtime needs `cv2` and `numpy`.

```bash
PYTHONPATH=.:tests python3 -m unittest discover -s tests -v
```

The tests render synthetic table photos and drive fake providers; no ROS or
hardware is needed.
