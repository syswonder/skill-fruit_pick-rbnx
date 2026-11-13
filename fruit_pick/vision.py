# SPDX-License-Identifier: MulanPSL-2.0
"""Find fruit on the table in one camera frame and place it in the base frame.

A fruit is a round blob that differs from a reference frame of the empty
table, taken from the same pose. Its centre is where the ray through the blob
centre meets the horizontal plane one radius above the table, and its radius
follows from the blob radius at that depth. The camera gives no depth, so the
table height must be known.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np

UNDISTORT = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 50, 1e-12)


@dataclass(frozen=True)
class Fruit:
    """One located fruit: centre in the base frame (m), diameter (m), blob in pixels."""

    centre: tuple
    diameter_m: float
    pixel: tuple


def matrix(position, orientation):
    """4x4 transform from a position and a ROS ``(x, y, z, w)`` quaternion."""
    x, y, z, w = orientation
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = position
    return T


def blobs(gray, background, min_contrast):
    """``(u, v, radius_px, roundness)`` for every blob that differs from ``background``.

    Blobs touching the image border are dropped: their size is unknown.
    """
    h, w = gray.shape
    # Kernels scale with the image so the same config works at any published size.
    sigma = max(1.0, w / 400)
    diff = cv2.GaussianBlur(cv2.absdiff(gray, background), (0, 0), sigma)
    otsu, _ = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    _, mask = cv2.threshold(diff, max(otsu, min_contrast), 255, cv2.THRESH_BINARY)
    size = max(3, int(w / 100) | 1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((size, size), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    found = []
    for contour in contours:
        x, y, cw, ch = cv2.boundingRect(contour)
        if x == 0 or y == 0 or x + cw == w or y + ch == h:
            continue
        (u, v), r = cv2.minEnclosingCircle(contour)
        found.append((u, v, r, cv2.contourArea(contour) / (math.pi * r * r)))
    return found


def table_point(uv, K, d, camera, z):
    """Base-frame point where the ray through pixel ``uv`` meets the plane at height ``z``."""
    m = cv2.undistortPointsIter(np.array([[uv]], float), K, d, None, None, UNDISTORT)[0, 0]
    ray = camera[:3, :3] @ [m[0], m[1], 1.0]
    origin = camera[:3, 3]
    return origin + (z - origin[2]) / ray[2] * ray


def locate(u, v, r_px, K, d, camera, table_z):
    """Return (centre, radius in m) of a sphere-like fruit resting on the table."""
    to_camera = np.linalg.inv(camera)
    centre_z, radius = table_z, 0.0
    # The centre height depends on the radius, which depends on the depth: iterate.
    for _ in range(5):
        p = table_point((u, v), K, d, camera, centre_z)
        depth = (to_camera[:3, :3] @ p + to_camera[:3, 3])[2]
        radius = r_px * depth / K[0, 0]
        centre_z = table_z + radius
    return table_point((u, v), K, d, camera, centre_z), radius


def find_fruits(gray, background, K, d, camera, config):
    """Every fruit in view, nearest the image centre first.

    ``camera`` is the base-frame pose of the camera optical frame (4x4).
    Blobs that are not round enough or outside ``fruit_diameter_m`` are not fruit.
    """
    if gray.shape != background.shape:
        raise ValueError(f"the frame is {gray.shape[1]}x{gray.shape[0]} but the background "
                         f"is {background.shape[1]}x{background.shape[0]}; capture it again")
    low, high = config.fruit_diameter_m
    h, w = gray.shape
    fruits = []
    for u, v, r_px, roundness in blobs(gray, background, config.min_contrast):
        if roundness < config.min_roundness:
            continue
        centre, radius = locate(u, v, r_px, K, d, camera, config.table_z_m)
        if low <= 2 * radius <= high:
            fruits.append(Fruit(tuple(float(c) for c in centre), 2 * radius, (u, v, r_px)))
    return sorted(fruits, key=lambda f: math.hypot(f.pixel[0] - w / 2, f.pixel[1] - h / 2))
