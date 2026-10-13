import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from kinematics import (
    RobotArm, check_transform, from_xyz_rpy,
    inverse_transform, load_transform,
)

ROOT = Path(__file__).resolve().parent


class TransformTests(unittest.TestCase):
    def test_nine_dh_rows_and_home_position(self):
        robot = RobotArm(ROOT / "rb5_850_dh.csv")
        poses = robot.get_all_link_poses([0] * 6)
        self.assertEqual(len(poses), 10)
        np.testing.assert_allclose(
            poses[-1][:3, :3], np.eye(3), atol=1e-10
        )
        np.testing.assert_allclose(
            poses[-1][:3, 3], [0, -207.4, 1096.9], atol=1e-8
        )
        with self.assertRaises(ValueError):
            robot.get_all_link_poses([0] * 9)

    def test_joint_one_rotates_the_entire_chain(self):
        robot = RobotArm(ROOT / "rb5_850_dh.csv")
        home = robot.get_all_link_poses([0] * 6)[-1]
        rotated = robot.get_all_link_poses(
            [90, 0, 0, 0, 0, 0]
        )[-1]
        np.testing.assert_allclose(
            rotated,
            from_xyz_rpy(0, 0, 0, 0, 0, 90) @ home,
            atol=1e-8,
        )

    def test_json_metres_and_chain_direction(self):
        matrix = [
            [-0.99971223, -0.02162889, 0.01037517, 0.02914021],
            [-0.01050652, 0.00596831, -0.99992699, -0.0311972],
            [0.02156539, -0.99974825, -0.00619383, 0.06017359],
            [0, 0, 0, 1],
        ]
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / "tcp_cam.json"
            filename.write_text(
                json.dumps(matrix), encoding="utf-8"
            )
            tcp_cam = load_transform(filename, "m")

        cam_board = from_xyz_rpy(0, 0, 500, 0, 0, 0)
        np.testing.assert_allclose(
            tcp_cam[:3, 3], [29.14021, -31.1972, 60.17359]
        )
        T = tcp_cam @ cam_board
        np.testing.assert_allclose(
            T[:3, 3], [34.327795, -531.160695, 57.076675]
        )
        np.testing.assert_allclose(
            T @ inverse_transform(T), np.eye(4), atol=1e-5
        )

    def test_zyx_rotation_order(self):
        T = from_xyz_rpy(0, 0, 0, 90, 0, 90)
        np.testing.assert_allclose(
            T[:3, :3] @ [0, 1, 0], [0, 0, 1], atol=1e-8
        )

    def test_invalid_transform_is_rejected(self):
        bad = np.eye(4)
        bad[0, 0] = -1
        with self.assertRaises(ValueError):
            check_transform(bad)

        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / "bad.json"
            filename.write_text(
                "[[1, 2], [3, 4]]", encoding="utf-8"
            )
            with self.assertRaises(ValueError):
                load_transform(filename, "mm")


if __name__ == "__main__":
    unittest.main()
