import unittest

from ik_position_gui import (
    compose_reference_transform,
    forward_kinematics_from_model,
    model_to_servo_degrees,
    servo_to_model_degrees,
    solve_xyz_inverse_kinematics,
)


class IKPositionGuiTests(unittest.TestCase):
    def test_servo_model_conversion_round_trip(self) -> None:
        servo_angles = [90, 120, -10, 15, -20]
        model_angles = servo_to_model_degrees(servo_angles)
        self.assertEqual(model_angles, [90.0, 60.0, 5.0, 15.0, -20.0])
        self.assertEqual(model_to_servo_degrees(model_angles), servo_angles)

    def test_reference_transform_shape(self) -> None:
        transform = compose_reference_transform([0.0, 0.0, 0.0])
        self.assertEqual(len(transform), 4)
        self.assertTrue(all(len(row) == 4 for row in transform))
        self.assertEqual(transform[3], [0.0, 0.0, 0.0, 1.0])

    def test_home_pose_matches_reference_point_convention(self) -> None:
        pose = forward_kinematics_from_model(servo_to_model_degrees([90, 90, 0, 0, 0]))
        self.assertAlmostEqual(pose.x_mm, 4.2, places=1)
        self.assertAlmostEqual(pose.y_mm, 137.42, places=1)
        self.assertAlmostEqual(pose.z_mm, 190.38, places=1)

    def test_xyz_ik_round_trip_reachable_target(self) -> None:
        target_model_angles = [90.0, 25.0, 30.0]
        target_pose = forward_kinematics_from_model(target_model_angles)
        current_servo = [90, 90, 0, 0, 0]

        solution, _ = solve_xyz_inverse_kinematics(
            target_pose.x_mm,
            target_pose.y_mm,
            target_pose.z_mm,
            current_servo,
        )

        self.assertAlmostEqual(solution.pose.x_mm, target_pose.x_mm, delta=2.0)
        self.assertAlmostEqual(solution.pose.y_mm, target_pose.y_mm, delta=2.0)
        self.assertAlmostEqual(solution.pose.z_mm, target_pose.z_mm, delta=2.0)

    def test_xyz_ik_rejects_base_axis_singularity(self) -> None:
        with self.assertRaisesRegex(ValueError, "base axis"):
            solve_xyz_inverse_kinematics(
                0.0,
                0.0,
                200.0,
                [90, 90, 0, 0, 0],
            )

    def test_xyz_ik_rejects_unreachable_target(self) -> None:
        with self.assertRaisesRegex(ValueError, "No valid 3-link XYZ solution"):
            solve_xyz_inverse_kinematics(
                1000.0,
                0.0,
                1000.0,
                [90, 90, 0, 0, 0],
            )


if __name__ == "__main__":
    unittest.main()
