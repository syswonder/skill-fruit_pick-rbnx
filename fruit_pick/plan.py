# SPDX-License-Identifier: MulanPSL-2.0
"""The step sequence for picking one located fruit.

The arm travels at the height of ``home`` (the photo pose, high above the
table), descends vertically to the fruit, and climbs back vertically. Heights
are fingertip heights; the TCP sits ``fingertip_offset_m`` above the fingertip
along the tool axis.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import Pose
from .vision import matrix


@dataclass(frozen=True)
class Step:
    """One action: ``move`` to ``pose``, ``open`` or ``grip``."""

    kind: str
    label: str
    pose: Pose | None = None


def tip_pose(config, x, y, z):
    """The TCP pose, with home's orientation, that puts the fingertip at (x, y, z)."""
    home = config.home
    axis = matrix(home.position, home.orientation)[:3, 2]
    return Pose(tuple(float(v) for v in np.array([x, y, z]) - config.fingertip_offset_m * axis),
                home.orientation)


def fingertip_z(config, pose):
    """Fingertip height of a TCP pose."""
    return float((matrix(pose.position, pose.orientation) @ [0, 0, config.fingertip_offset_m, 1])[2])


def plan_steps(config, fruit, destination):
    """Steps to pick ``fruit`` and release it at ``destination`` (None: where it was)."""
    x, y, z = fruit.centre
    travel = tip_pose(config, x, y, fingertip_z(config, config.home))
    grasp_z = max(z - config.grasp_below_centre_m, config.table_z_m + config.min_fingertip_height_m)
    steps = [
        Step("move", "move above the fruit", travel),
        Step("move", "descend to the fruit", tip_pose(config, x, y, grasp_z)),
        Step("grip", "close the gripper"),
        Step("move", "lift the fruit", travel),
    ]
    if destination is None:
        steps += [Step("move", "lower the fruit back",
                       tip_pose(config, x, y, grasp_z + config.release_lift_m)),
                  Step("open", "open the gripper to release"),
                  Step("move", "lift clear", travel)]
    else:
        above = destination.pose.raised(destination.approach_height_m)
        steps += [Step("move", f"move above {destination.name}", above),
                  Step("move", f"descend to {destination.name}", destination.pose),
                  Step("open", "open the gripper to release"),
                  Step("move", f"lift from {destination.name}", above)]
    return steps + [Step("move", "return home", config.home)]
