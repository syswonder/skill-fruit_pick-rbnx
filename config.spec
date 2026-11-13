# Instance config for the fruit_pick skill (documentation, not a schema).
# Unknown keys are rejected at INIT. Poses are the arm's end_pose poses in its
# base frame: metres, and roll/pitch/yaw in radians.
config:
  arm_provider_id: aubo_arm           # required; Atlas provider ids
  gripper_provider_id: jodell_gripper
  camera_provider_id: mv_camera
  home: {position: [x, y, z], rpy: [r, p, y]}   # required; the photo pose, high above the table,
                                                # tool pointing straight down; every run starts and ends here
  camera_in_tcp: [x, y, z, r, p, y]   # required; camera optical frame in the end_pose frame (hand-eye)
  fingertip_offset_m: 0.203           # required; fingertip distance along the end_pose z axis
  table_z_m: -0.0184                  # required; table surface height in the base frame
  fruit_diameter_m: [0.02, 0.10]      # blobs outside this size are not fruit
  min_contrast: 20                    # least grey-level difference from the background that counts
  min_roundness: 0.6                  # blob area over its enclosing circle's area
  clearance_m: 0.003                  # least gap per side between the fruit and the open jaws
  grasp_below_centre_m: 0.008         # fingertips close this far below the fruit centre ...
  min_fingertip_height_m: 0.015       # ... but never lower than this above the table
  release_lift_m: 0.003               # put back this much above the grasp height
  locations:                          # optional; named places to release fruit at
    basket: {position: [x, y, z], rpy: [r, p, y], approach_height_m: 0.06}
  approach_height_m: 0.06             # vertical approach to a location
  start_tolerance_m: 0.02             # how close to home the arm must be to start
  start_tolerance_rad: 0.1
  position_tolerance_m: 0.002         # arrival tolerance for each move
  orientation_tolerance_rad: 0.01
  stationary_speed_rad_s: 0.003       # joints below this speed count as stopped
  move_timeout_s: 60
  motion_start_timeout_s: 3           # no TCP change within this time: the arm refused the target
  stall_timeout_s: 2                  # stationary short of the target this long: the arm halted it
  feedback_timeout_s: 1               # no end_pose sample within this time: fail
  command_ready_timeout_s: 5          # wait for the arm to subscribe to pos_command
  gripper_timeout_s: 5                # open/grip must report stopped jaws within this time
  frame_timeout_s: 10                 # wait for a camera frame taken after the arm settled
