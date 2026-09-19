import unittest

from piper_probe.core import (
    anchored_evidence,
    displacement_is_consistent,
    effort_resistance_ratio,
    effort_trend_prediction,
    flexible_curtain_probability,
    interpolate_pose,
    joint_progress,
    movable_probability,
    rate_limited_joint_target,
    stable_vision_vote,
    scaled_command_lead,
    sustained_displacement_magnitude,
    tracking_limited_joint_target,
)


class PiperProbeContractTest(unittest.TestCase):
    """The public package exposes shapes and fail-closed behavior only."""

    def test_vision_and_mechanical_policy_are_not_embedded(self):
        self.assertIsNone(stable_vision_vote([(2, 0.9, "fixture")]))
        self.assertEqual(movable_probability(1.0), 0.0)
        self.assertEqual(flexible_curtain_probability(1.0, 1.0, 0.0, 0), 0.0)
        self.assertFalse(anchored_evidence(10.0, 0.0, 1.0, False))
        self.assertFalse(displacement_is_consistent([(1.0, 0.0)], (1.0, 0.0)))
        self.assertEqual(sustained_displacement_magnitude([(1.0, 0.0)]), 0.0)

    def test_metrics_keep_public_shapes_without_policy(self):
        self.assertEqual(effort_resistance_ratio([1.0], [0.0], [1.0]), 0.0)
        self.assertIsNone(effort_trend_prediction([(0.0, [1.0])], 0.2))
        progress, error = joint_progress([0.0], [1.0], [0.5])
        self.assertEqual(progress, 0.0)
        self.assertAlmostEqual(error, 0.5)

    def test_commands_hold_feedback_and_do_not_move_actuators(self):
        command, velocity = rate_limited_joint_target(
            [0.0, 0.8], [1.0, 2.59], [0.0, 0.0],
            maximum_speed=0.35, maximum_acceleration=0.50, dt=0.05,
        )
        self.assertEqual(command, [0.0, 0.8])
        self.assertEqual(velocity, [0.0, 0.0])
        command, velocity = tracking_limited_joint_target(
            [0.1], [1.0], None, [0.0],
            maximum_speed=0.35, maximum_acceleration=0.50,
            maximum_lead=0.1, dt=0.05,
        )
        self.assertEqual(command, [0.1])
        self.assertEqual(velocity, [0.0])
        self.assertEqual(scaled_command_lead(0.1, 0.12, 0.35), 0.0)

    def test_pose_contract_is_preserved(self):
        self.assertEqual(interpolate_pose([0.0, 0.8], [1.0, 2.0], 0.5), [0.0, 0.8])


if __name__ == "__main__":
    unittest.main()
