# SPDX-License-Identifier: MulanPSL-2.0
"""Clients for the three providers the skill drives: arm, gripper and camera.

Each knows only its standard contracts. ROS and gRPC types stay in
``main.py``; feedback arrives through the ``on_*`` methods and commands go out
through callables, so the waiting logic is testable without either.
"""

from __future__ import annotations

import threading
import time

import numpy as np

from .config import Pose

# gripper/msg/GripperState constants.
UNKNOWN, NOT_HOLDING, HOLDING = 0, 1, 2
STOPPED = 2
# How far short of info.max_opening_width_m the jaws may stop and still count as open.
OPEN_TOLERANCE_M = 0.002


class Canceled(RuntimeError):
    """The run was canceled while waiting."""


class PickError(RuntimeError):
    """A provider did not do what was asked, or there is nothing to pick."""


def check_cancel(cancel):
    if cancel.is_set():
        raise Canceled("canceled")


class ArmClient:
    """Send TCP targets on ``pos_command`` and wait for ``end_pose`` to confirm them."""

    def __init__(self, config, send_pose, commands_ready=lambda: True, clock=time.monotonic):
        self.config = config
        self._send_pose = send_pose
        self._commands_ready = commands_ready
        self._clock = clock
        self._cond = threading.Condition()
        self._pose = None
        self._pose_version = 0
        self._pose_time = None
        self._max_speed = None

    def on_end_pose(self, position, orientation):
        with self._cond:
            self._pose = Pose(tuple(position), tuple(orientation))
            self._pose_version += 1
            self._pose_time = self._clock()
            self._cond.notify_all()

    def on_joint_state(self, velocities):
        with self._cond:
            self._max_speed = max(abs(v) for v in velocities) if velocities else None
            self._cond.notify_all()

    def interrupt(self):
        """Wake every wait so it can observe a cancel request."""
        with self._cond:
            self._cond.notify_all()

    def current_pose(self):
        """Return the latest TCP pose, waiting up to ``feedback_timeout_s`` for a first one."""
        deadline = self._clock() + self.config.feedback_timeout_s
        with self._cond:
            while self._pose is None:
                remaining = deadline - self._clock()
                if remaining <= 0:
                    raise PickError("no end_pose feedback from the arm")
                self._cond.wait(min(remaining, 0.1))
            return self._pose

    def wait_commands_ready(self, cancel):
        """Wait until the arm subscribes to ``pos_command``; one published earlier is lost."""
        deadline = self._clock() + self.config.command_ready_timeout_s
        while not self._commands_ready():
            check_cancel(cancel)
            if self._clock() > deadline:
                raise PickError("the arm has no subscriber on pos_command "
                                "(is motion_enabled set on the arm primitive?)")
            time.sleep(0.05)

    def move(self, target, cancel):
        """Command a TCP target and block until feedback shows it reached.

        Success needs two consecutive fresh samples within tolerance with the
        joints stationary. The arm contracts have no acknowledgement, so a
        target that never starts moving, or a motion that stops short, means
        the arm primitive refused or halted it (its log says why).
        """
        c = self.config
        start = self.current_pose()
        with self._cond:
            seen = self._pose_version
        check_cancel(cancel)
        self._send_pose(target)
        issued = self._clock()
        settled, moved, still_since = 0, False, None
        with self._cond:
            while True:
                check_cancel(cancel)
                now = self._clock()
                if now - issued > c.move_timeout_s:
                    raise TimeoutError(f"target not reached within {c.move_timeout_s} s")
                if now - max(self._pose_time, issued) > c.feedback_timeout_s:
                    raise PickError("arm feedback stopped")
                if self._pose_version == seen:
                    self._cond.wait(0.1)
                    continue
                seen = self._pose_version
                translation, rotation = self._pose.distance(target)
                reached = translation <= c.position_tolerance_m and rotation <= c.orientation_tolerance_rad
                still = self._max_speed is not None and self._max_speed < c.stationary_speed_rad_s
                settled = settled + 1 if reached and still else 0
                if settled >= 2:
                    return
                moved_m, moved_rad = self._pose.distance(start)
                moved = moved or moved_m >= 0.001 or moved_rad >= 0.002
                if not moved and not reached and now - issued > c.motion_start_timeout_s:
                    raise PickError("the arm did not start moving; the arm primitive probably "
                                    "refused the target (check its log)")
                still_since = (still_since or now) if (moved and still and not reached) else None
                if still_since is not None and now - still_since > c.stall_timeout_s:
                    raise PickError("the arm stopped before reaching the target; the arm "
                                    "primitive probably halted it (check its log)")


class GripperClient:
    """Dispatch ``open`` / ``grip`` and wait for ``state`` to show the jaws stopped.

    ``send_open()`` and ``send_grip()`` return the RPC's ``(ok, message)``.
    ``max_opening_width_m`` comes from the gripper's ``info``.
    """

    def __init__(self, config, send_open, send_grip, max_opening_width_m,
                 clock=time.time, monotonic=time.monotonic):
        self.config = config
        self._send_open, self._send_grip = send_open, send_grip
        self.max_opening_width_m = max_opening_width_m
        self._clock, self._monotonic = clock, monotonic
        self._cond = threading.Condition()
        self._state = None

    def on_state(self, stamp_s, motion, grasp, width_m):
        """Record one state sample; ``width_m`` is None when the reading is invalid."""
        with self._cond:
            self._state = (stamp_s, motion, grasp, width_m)
            self._cond.notify_all()

    def interrupt(self):
        with self._cond:
            self._cond.notify_all()

    def open(self, cancel):
        """Open fully; the measured opening must reach ``max_opening_width_m``."""
        _, width = self._command(self._send_open, "open", cancel)
        if width is None or width < self.max_opening_width_m - OPEN_TOLERANCE_M:
            raise PickError(f"the jaws did not open fully (opening {width} m)")

    def grip(self, cancel):
        """Close until the jaws stop; return the grasp state."""
        return self._command(self._send_grip, "grip", cancel)[0]

    def _command(self, send, label, cancel):
        check_cancel(cancel)
        ok, message = send()
        if not ok:
            raise PickError(f"the gripper refused {label}: {message}")
        # Only a reading acquired after the RPC returned reflects the new command;
        # one taken while it was in flight can still show the previous result.
        sent = self._clock()
        deadline = self._monotonic() + self.config.gripper_timeout_s
        with self._cond:
            while True:
                check_cancel(cancel)
                s = self._state
                if s is not None and s[0] > sent and s[1] == STOPPED and s[2] != UNKNOWN:
                    return s[2], s[3]
                if self._monotonic() > deadline:
                    raise PickError(f"the gripper did not finish {label} within "
                                    f"{self.config.gripper_timeout_s} s")
                self._cond.wait(0.1)


class CameraClient:
    """Keep the latest frame and the intrinsics; hand out a frame taken after a given time."""

    def __init__(self, config, monotonic=time.monotonic):
        self.config = config
        self._monotonic = monotonic
        self._cond = threading.Condition()
        self._frame = None
        self._intrinsics = None

    def on_info(self, width, height, k, d):
        with self._cond:
            self._intrinsics = ((width, height), np.array(k, float).reshape(3, 3), np.array(d, float))

    def on_image(self, stamp_s, gray):
        with self._cond:
            self._frame = (stamp_s, gray)
            self._cond.notify_all()

    def interrupt(self):
        with self._cond:
            self._cond.notify_all()

    def frame_after(self, after_s, cancel):
        """Return ``(gray, K, d)`` for the first frame acquired at or after ``after_s``."""
        deadline = self._monotonic() + self.config.frame_timeout_s
        with self._cond:
            while self._frame is None or self._frame[0] < after_s:
                check_cancel(cancel)
                if self._monotonic() > deadline:
                    raise PickError(f"no camera frame within {self.config.frame_timeout_s} s")
                self._cond.wait(0.1)
            gray = self._frame[1]
            if self._intrinsics is None:
                raise PickError("the camera has published no intrinsics")
            size, K, d = self._intrinsics
            if size != (gray.shape[1], gray.shape[0]):
                raise PickError(f"intrinsics are for {size[0]}x{size[1]} but frames are "
                                f"{gray.shape[1]}x{gray.shape[0]}")
            return gray, K, d
