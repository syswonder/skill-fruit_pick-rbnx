# SPDX-License-Identifier: MulanPSL-2.0
import tempfile
import threading
import time
import unittest
from pathlib import Path

import numpy as np

from fruit_pick.clients import ArmClient, CameraClient, Canceled, GripperClient
from fruit_pick.config import Pose, SkillConfig
from fruit_pick.plan import fingertip_z
from fruit_pick.runner import Runner
from helpers import CONFIG, DOWN_Q, FakeRobot, home_pose

K = np.array([[1761.5, 0, 612.0], [0, 1761.5, 512.0], [0, 0, 1]])
D = np.zeros(5)
TABLE = CONFIG["table_z_m"]
FRUIT = ((-0.30, -0.47, TABLE + 0.0165), 0.033)


class RunnerTest(unittest.TestCase):
    def setUp(self):
        self.config = SkillConfig.parse(CONFIG)
        self.dir = tempfile.TemporaryDirectory()
        self.start_at(home_pose())

    def start_at(self, pose):
        self.robot = FakeRobot(pose, K, D)
        arm = ArmClient(self.config, self.robot.send_pose)
        gripper = GripperClient(self.config, self.robot.send_open, self.robot.send_grip, 0.040)
        camera = CameraClient(self.config)
        self.robot.attach(arm, gripper, camera)
        self.runner = Runner(self.config, arm, gripper, camera, Path(self.dir.name) / "background.png")

    def tearDown(self):
        self.robot.stop.set()
        self.dir.cleanup()

    def wait_done(self, run_id, limit=10):
        deadline = time.monotonic() + limit
        while time.monotonic() < deadline:
            state, detail = self.runner.status(run_id)
            if state in ("SUCCEEDED", "FAILED", "CANCELED", "TIMEOUT"):
                return state, detail
            time.sleep(0.02)
        self.fail("run did not finish")

    def captured(self, fruits=(FRUIT,)):
        self.runner.capture_background()
        self.robot.scene = list(fruits)
        self.robot.log.clear()

    def moves(self):
        return [entry[1] for entry in self.robot.log if entry[0] == "move"]

    def tip_z(self, position):
        return fingertip_z(self.config, Pose(position, DOWN_Q))

    def test_picks_the_fruit_and_puts_it_back(self):
        self.captured()
        state, detail = self.wait_done(self.runner.start(""))
        self.assertEqual(state, "SUCCEEDED", detail)
        self.assertIn("33 mm across", detail)
        kinds = [entry[0] for entry in self.robot.log]
        self.assertEqual(kinds, ["move", "open", "move", "move", "grip", "move", "move",
                                 "open", "move", "move"])
        moves = self.moves()
        grasp = self.tip_z(moves[2])
        # A 33 mm fruit is gripped at the 15 mm fingertip floor, not 8 mm below its centre.
        self.assertAlmostEqual(grasp, TABLE + 0.015, delta=1e-6)
        self.assertAlmostEqual(self.tip_z(moves[4]), grasp + 0.003, delta=1e-6)
        self.assertEqual(moves[-1], tuple(CONFIG["home"]["position"]))

    def test_places_at_a_named_location(self):
        self.captured()
        state, detail = self.wait_done(self.runner.start("basket"))
        self.assertEqual(state, "SUCCEEDED", detail)
        self.assertEqual(self.moves()[5], (-0.20, -0.45, 0.25))

    def test_no_fruit_fails_before_the_arm_leaves_home(self):
        self.captured(fruits=())
        state, detail = self.wait_done(self.runner.start(""))
        self.assertEqual(state, "FAILED")
        self.assertIn("no fruit in view", detail)
        self.assertEqual(self.moves(), [tuple(CONFIG["home"]["position"])])

    def test_fruit_wider_than_the_open_jaws_is_refused(self):
        self.captured(fruits=[((-0.30, -0.47, TABLE + 0.019), 0.038)])
        state, detail = self.wait_done(self.runner.start(""))
        self.assertEqual(state, "FAILED")
        self.assertIn("jaws are 40 mm apart", detail)
        self.assertNotIn(("grip",), self.robot.log)

    def test_empty_grip_opens_again_and_stops_at_the_fruit(self):
        self.captured()
        self.robot.empty_grip = True
        state, detail = self.wait_done(self.runner.start(""))
        self.assertEqual(state, "FAILED")
        self.assertIn("closed on nothing", detail)
        self.assertEqual([e[0] for e in self.robot.log][-2:], ["grip", "open"])
        self.assertEqual(len(self.moves()), 3)

    def test_refused_grip_fails_with_the_gripper_reason(self):
        self.captured()
        self.robot.refuse.add("grip")
        state, detail = self.wait_done(self.runner.start(""))
        self.assertEqual(state, "FAILED")
        self.assertIn("fault byte 0x08", detail)

    def test_needs_a_background_and_the_arm_at_home(self):
        state, detail = self.wait_done(self.runner.start(""))
        self.assertEqual(state, "FAILED")
        self.assertIn("capture_background first", detail)
        self.robot.stop.set()
        self.start_at(Pose((-0.30, -0.43, 0.4695), DOWN_Q))
        state, detail = self.wait_done(self.runner.start(""))
        self.assertIn("away from home", detail)
        self.assertEqual(self.moves(), [])

    def test_background_survives_a_restart(self):
        self.runner.capture_background()
        self.robot.stop.set()
        self.start_at(home_pose())
        self.robot.scene = [FRUIT]
        state, detail = self.wait_done(self.runner.start(""))
        self.assertEqual(state, "SUCCEEDED", detail)

    def test_cancel_stops_issuing_commands(self):
        self.captured()
        run_id = self.runner.start("")
        time.sleep(0.4)
        self.assertTrue(self.runner.cancel(run_id))
        state, _ = self.wait_done(run_id)
        self.assertEqual(state, "CANCELED")
        issued = len(self.robot.log)
        time.sleep(0.2)
        self.assertEqual(len(self.robot.log), issued)

    def test_cancel_all_stops_a_background_capture(self):
        self.robot.refuse.add("pose")   # 10 mm from home and never gets there
        self.robot.pose = Pose((-0.3257, -0.4333, 0.4595), DOWN_Q)
        errors = []

        def capture():
            try:
                self.runner.capture_background()
            except Exception as exc:
                errors.append(exc)

        worker = threading.Thread(target=capture, daemon=True)
        worker.start()
        time.sleep(0.1)
        self.assertTrue(self.runner.cancel_all(timeout_s=1.0))
        worker.join(1.0)
        self.assertEqual([type(e) for e in errors], [Canceled])

    def test_only_one_run_at_a_time(self):
        self.captured()
        run_id = self.runner.start("")
        with self.assertRaisesRegex(RuntimeError, "still"):
            self.runner.start("")
        with self.assertRaisesRegex(RuntimeError, "in progress"):
            self.runner.capture_background()
        self.runner.cancel(run_id)
        self.wait_done(run_id)
        with self.assertRaisesRegex(ValueError, "configured"):
            self.runner.start("nowhere")


class ConfigTest(unittest.TestCase):
    def test_invalid_configs_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "unknown config keys"):
            SkillConfig.parse(dict(CONFIG, speed=1))
        with self.assertRaisesRegex(ValueError, "camera_provider_id"):
            SkillConfig.parse({k: v for k, v in CONFIG.items() if k != "camera_provider_id"})
        with self.assertRaisesRegex(ValueError, "fruit_diameter_m"):
            SkillConfig.parse(dict(CONFIG, fruit_diameter_m=[0.01, 0.06]))
        with self.assertRaisesRegex(ValueError, "lower_snake_case"):
            SkillConfig.parse(dict(CONFIG, locations={"Basket": CONFIG["locations"]["basket"]}))


if __name__ == "__main__":
    unittest.main()
