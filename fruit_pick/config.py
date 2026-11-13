# SPDX-License-Identifier: MulanPSL-2.0
"""Skill configuration (see ``config.spec``) and base-frame poses.

Poses are ``end_pose`` poses of the arm provider in its base frame: position
in metres and fixed-axis XYZ roll/pitch/yaw in radians (what the AUBO
pendant shows). The fingertip sits ``fingertip_offset_m`` along that frame's
z axis, so a ``home`` pointing straight down keeps the jaws vertical.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields
import math
import re

NAME = re.compile(r"^[a-z][a-z0-9_]{0,47}$")


def _vector(value, size, label):
    """Return ``value`` as a tuple of ``size`` finite numbers or raise ValueError."""
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{label} must contain {size} numbers")
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise ValueError(f"{label} must contain only finite numbers")
    return tuple(float(v) for v in value)


def quaternion(rpy):
    """Convert roll/pitch/yaw to a ROS ``(x, y, z, w)`` quaternion."""
    r, p, y = rpy
    cr, cp, cy = (math.cos(v / 2) for v in (r, p, y))
    sr, sp, sy = (math.sin(v / 2) for v in (r, p, y))
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


@dataclass(frozen=True)
class Pose:
    """A base-frame pose: position (m) and ROS quaternion (x, y, z, w)."""

    position: tuple
    orientation: tuple

    def raised(self, dz):
        """The same pose moved ``dz`` metres along base +Z."""
        x, y, z = self.position
        return Pose((x, y, z + dz), self.orientation)

    def distance(self, other):
        """Return (translation in m, rotation in rad) to another pose."""
        dot = min(1.0, abs(sum(a * b for a, b in zip(self.orientation, other.orientation))))
        return math.dist(self.position, other.position), 2 * math.acos(dot)


@dataclass(frozen=True)
class Location:
    """A named place to put a fruit: the pose where the jaws open."""

    name: str
    pose: Pose
    approach_height_m: float


# Durations, distances and tolerances that must be positive.
POSITIVE = ("fingertip_offset_m", "clearance_m", "grasp_below_centre_m",
            "min_fingertip_height_m", "release_lift_m", "approach_height_m",
            "start_tolerance_m", "start_tolerance_rad", "position_tolerance_m",
            "orientation_tolerance_rad", "stationary_speed_rad_s", "move_timeout_s",
            "motion_start_timeout_s", "stall_timeout_s", "feedback_timeout_s",
            "command_ready_timeout_s", "gripper_timeout_s", "frame_timeout_s")


@dataclass(frozen=True)
class SkillConfig:
    """Validated instance config."""

    arm_provider_id: str
    gripper_provider_id: str
    camera_provider_id: str
    home: Pose
    camera_in_tcp: tuple
    fingertip_offset_m: float
    table_z_m: float
    fruit_diameter_m: tuple = (0.02, 0.10)
    min_contrast: int = 20
    min_roundness: float = 0.6
    clearance_m: float = 0.003
    grasp_below_centre_m: float = 0.008
    min_fingertip_height_m: float = 0.015
    release_lift_m: float = 0.003
    locations: dict = field(default_factory=dict)
    approach_height_m: float = 0.06
    start_tolerance_m: float = 0.02
    start_tolerance_rad: float = 0.1
    position_tolerance_m: float = 0.002
    orientation_tolerance_rad: float = 0.01
    stationary_speed_rad_s: float = 0.003
    move_timeout_s: float = 60.0
    motion_start_timeout_s: float = 3.0
    stall_timeout_s: float = 2.0
    feedback_timeout_s: float = 1.0
    command_ready_timeout_s: float = 5.0
    gripper_timeout_s: float = 5.0
    frame_timeout_s: float = 10.0

    @classmethod
    def parse(cls, cfg):
        """Validate a deployment ``config:`` mapping; raise ValueError on any problem."""
        cfg = dict(cfg or {})
        unknown = set(cfg) - {f.name for f in fields(cls)}
        if unknown:
            raise ValueError(f"unknown config keys: {sorted(unknown)}")
        for key in ("arm_provider_id", "gripper_provider_id", "camera_provider_id"):
            if not isinstance(cfg.get(key), str) or not cfg[key]:
                raise ValueError(f"{key} is required")
        for key in ("fingertip_offset_m", "table_z_m"):
            if key not in cfg:
                raise ValueError(f"{key} is required")
        cfg["home"] = _pose("home", cfg.get("home"))
        cfg["camera_in_tcp"] = _vector(cfg.get("camera_in_tcp"), 6, "camera_in_tcp")
        cfg["fruit_diameter_m"] = _vector(cfg.get("fruit_diameter_m", cls.fruit_diameter_m),
                                          2, "fruit_diameter_m")
        height = cfg.get("approach_height_m", cls.approach_height_m)
        raw = cfg.get("locations") or {}
        if not isinstance(raw, dict):
            raise ValueError("locations must map names to poses")
        cfg["locations"] = {name: _location(name, spec, height) for name, spec in raw.items()}
        config = cls(**cfg)
        for name in POSITIVE:
            value = getattr(config, name)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if type(config.table_z_m) not in (int, float) or not math.isfinite(config.table_z_m):
            raise ValueError("table_z_m must be a finite number")
        low, high = config.fruit_diameter_m
        # A fruit lower than the fingertip floor would be gripped above its top.
        if not config.min_fingertip_height_m < low < high:
            raise ValueError("fruit_diameter_m must be [min, max] with "
                             "min_fingertip_height_m < min < max")
        if type(config.min_contrast) is not int or not 1 <= config.min_contrast <= 255:
            raise ValueError("min_contrast must be an integer in 1..255")
        if type(config.min_roundness) not in (int, float) or not 0 < config.min_roundness <= 1:
            raise ValueError("min_roundness must be in (0, 1]")
        return config

    def location(self, name):
        """Look up a destination by name, listing the valid names on failure."""
        if name not in self.locations:
            raise ValueError(f"unknown location {name!r}; configured: {sorted(self.locations)}")
        return self.locations[name]


def _pose(label, spec):
    """Parse a ``{position, rpy}`` mapping into a Pose."""
    if not isinstance(spec, dict) or set(spec) != {"position", "rpy"}:
        raise ValueError(f"{label} needs position and rpy")
    return Pose(_vector(spec["position"], 3, f"{label} position"),
                quaternion(_vector(spec["rpy"], 3, f"{label} rpy")))


def _location(name, spec, default_height):
    """Parse one ``locations`` entry."""
    if not isinstance(name, str) or not NAME.match(name):
        raise ValueError(f"location name {name!r} must be lower_snake_case")
    if not isinstance(spec, dict):
        raise ValueError(f"location {name} needs position and rpy")
    spec = dict(spec)
    height = spec.pop("approach_height_m", default_height)
    if type(height) not in (int, float) or not 0 < height <= 0.3:
        raise ValueError(f"location {name}: approach_height_m must be in (0, 0.3]")
    return Location(name, _pose(f"location {name}", spec), float(height))
