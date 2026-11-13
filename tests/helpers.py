# SPDX-License-Identifier: MulanPSL-2.0
"""Shared fixtures: a renderer for synthetic table photos and fake providers."""

from __future__ import annotations

import math
import threading
import time

import cv2
import numpy as np

from fruit_pick.clients import HOLDING, NOT_HOLDING, STOPPED
from fruit_pick.config import Pose, quaternion
from fruit_pick.vision import matrix

DOWN = [math.pi, 0.0, 1.2896]
DOWN_Q = quaternion(DOWN)
# A photo pose: TCP (flange, no tool offset) 0.47 m up, pointing down.
HOME = {"position": [-0.3257, -0.4333, 0.4695], "rpy": DOWN}
CAMERA_IN_TCP = [0.0012, 0.1216, 0.0039, 0.0, 0.0, -1.57]
CONFIG = {
    "arm_provider_id": "aubo_arm",
    "gripper_provider_id": "jodell_gripper",
    "camera_provider_id": "mv_camera",
    "home": HOME,
    "camera_in_tcp": CAMERA_IN_TCP,
    "fingertip_offset_m": 0.203,
    "table_z_m": -0.0184,
    "fruit_diameter_m": [0.02, 0.06],
    "locations": {"basket": {"position": [-0.20, -0.45, 0.25], "rpy": DOWN}},
    "move_timeout_s": 2.0,
    "motion_start_timeout_s": 0.3,
    "feedback_timeout_s": 0.3,
    "command_ready_timeout_s": 0.3,
    "stall_timeout_s": 0.3,
    "gripper_timeout_s": 0.5,
    "frame_timeout_s": 0.5,
}


def render(width, height, K, d, camera, fruits):
    """A Mono8 photo of a dark textured table with bright spheres ``(centre, diameter)``."""
    rng = np.random.default_rng(0)
    gray = (60 + rng.integers(0, 8, (height, width))).astype(np.uint8)
    to_camera = np.linalg.inv(camera)
    rvec, _ = cv2.Rodrigues(to_camera[:3, :3])
    for centre, diameter in fruits:
        p = to_camera[:3, :3] @ centre + to_camera[:3, 3]
        (u, v), = cv2.projectPoints(np.array([centre], float), rvec, to_camera[:3, 3], K, d)[0][:, 0]
        r = diameter / 2 * K[0, 0] / p[2]
        cv2.circle(gray, (int(round(u)), int(round(v))), int(round(r)), 200, -1, cv2.LINE_AA)
    return gray


class FakeRobot:
    """Arm, gripper and camera fakes driven by one 100 Hz thread.

    The arm reaches each target after a few ticks. ``grip`` stops on a fruit
    unless ``empty_grip`` is set. The camera publishes ``scene`` (a list of
    fruits) as seen from the current TCP pose.
    """

    def __init__(self, start, K, d, size=(1224, 1024)):
        self.pose, self.target, self.ticks_left = start, None, 0
        self.K, self.d, self.size = K, d, size
        self.log, self.refuse = [], set()
        self.scene, self.empty_grip = [], False
        self.jaws = (STOPPED, NOT_HOLDING, 0.040)
        self.arm = self.gripper = self.camera = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def attach(self, arm, gripper, camera):
        self.arm, self.gripper, self.camera = arm, gripper, camera
        camera.on_info(*self.size, self.K.ravel().tolist(), self.d.tolist())
        self.thread.start()

    def send_pose(self, pose):
        self.log.append(("move", pose.position))
        if "pose" not in self.refuse:
            self.target, self.ticks_left = pose, 5

    def _jaws(self, grasp, width):
        self.jaws = (STOPPED, grasp, width)

    def send_open(self):
        self.log.append(("open",))
        self._jaws(NOT_HOLDING, 0.040)
        return True, ""

    def send_grip(self):
        self.log.append(("grip",))
        if "grip" in self.refuse:
            return False, "gripper fault byte 0x08"
        self._jaws(NOT_HOLDING if self.empty_grip else HOLDING, 0.0 if self.empty_grip else 0.033)
        return True, ""

    def photo(self):
        cam_in_tcp = matrix(CAMERA_IN_TCP[:3], quaternion(CAMERA_IN_TCP[3:]))
        camera = matrix(self.pose.position, self.pose.orientation) @ cam_in_tcp
        return render(*self.size, self.K, self.d, camera, self.scene)

    def _loop(self):
        tick = 0
        while not self.stop.wait(0.01):
            moving = self.target is not None
            if moving:
                self.ticks_left -= 1
                if self.ticks_left <= 0:
                    self.pose, self.target, moving = self.target, None, False
            self.arm.on_end_pose(self.pose.position, self.pose.orientation)
            self.arm.on_joint_state([0.2 if moving else 0.0] * 6)
            self.gripper.on_state(time.time(), *self.jaws)
            tick += 1
            if tick % 5 == 0 and not moving:
                self.camera.on_image(time.time(), self.photo())


def home_pose():
    return Pose(tuple(HOME["position"]), DOWN_Q)
