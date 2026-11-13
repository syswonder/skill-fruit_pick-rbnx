# SPDX-License-Identifier: MulanPSL-2.0
import math
import unittest

import cv2
import numpy as np

from fruit_pick.config import SkillConfig
from fruit_pick.vision import find_fruits, matrix
from helpers import CONFIG, DOWN_Q, render

W, H = 1224, 1024
K = np.array([[1761.5, 0, 615.0], [0, 1761.5, 520.0], [0, 0, 1]])
D = np.array([-0.08, 0.12, 0.0, 0.0, 0.0])


class FindFruitsTest(unittest.TestCase):
    def setUp(self):
        self.config = SkillConfig.parse(CONFIG)
        # Camera 0.45 m above the table, looking straight down.
        self.camera = matrix((-0.33, -0.43, 0.45), DOWN_Q)
        self.empty = render(W, H, K, D, self.camera, [])

    def test_locates_a_fruit_within_a_millimetre(self):
        centre, diameter = (-0.31, -0.45, CONFIG["table_z_m"] + 0.0165), 0.033
        frame = render(W, H, K, D, self.camera, [(centre, diameter)])
        fruits = find_fruits(frame, self.empty, K, D, self.camera, self.config)
        self.assertEqual(len(fruits), 1)
        self.assertLess(math.dist(fruits[0].centre, centre), 0.001)
        self.assertAlmostEqual(fruits[0].diameter_m, diameter, delta=0.001)

    def test_nearest_the_image_centre_comes_first(self):
        z = CONFIG["table_z_m"] + 0.02
        frame = render(W, H, K, D, self.camera, [((-0.27, -0.40, z), 0.04), ((-0.335, -0.425, z), 0.04)])
        fruits = find_fruits(frame, self.empty, K, D, self.camera, self.config)
        self.assertEqual([round(f.centre[0], 2) for f in fruits], [-0.33, -0.27])

    def test_unchanged_table_shapes_and_sizes_are_not_fruit(self):
        noisy = np.clip(self.empty.astype(int) + np.random.default_rng(1).integers(-6, 7, self.empty.shape),
                        0, 255).astype(np.uint8)
        self.assertEqual(find_fruits(noisy, self.empty, K, D, self.camera, self.config), [])
        bar = self.empty.copy()
        cv2.rectangle(bar, (300, 500), (700, 540), 255, -1)
        tiny = render(W, H, K, D, self.camera, [((-0.31, -0.45, CONFIG["table_z_m"] + 0.005), 0.01)])
        edge = self.empty.copy()
        cv2.circle(edge, (5, 500), 60, 255, -1)
        for frame in (bar, tiny, edge):
            self.assertEqual(find_fruits(frame, self.empty, K, D, self.camera, self.config), [])

    def test_background_of_another_size_is_refused(self):
        with self.assertRaisesRegex(ValueError, "capture it again"):
            find_fruits(self.empty, self.empty[:-2], K, D, self.camera, self.config)


if __name__ == "__main__":
    unittest.main()
