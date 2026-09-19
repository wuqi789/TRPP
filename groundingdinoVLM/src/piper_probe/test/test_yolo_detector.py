from types import SimpleNamespace
import unittest

from obstacle_traversal_interfaces.msg import ObstacleDetection
from obstacle_traversal_interfaces.msg import MechanicalAssessment
from obstacle_traversal_interfaces.action import ProbePushability
from piper_probe.probe_server import PiperProbeServer
from piper_probe.yolo_obstacle_detector import YoloObstacleDetector


class Values:
    def __init__(self, values):
        self._values = values

    def detach(self):
        return self

    def cpu(self):
        return self

    def tolist(self):
        return list(self._values)


def detector():
    value = object.__new__(YoloObstacleDetector)
    value._curtain_labels = {"curtain"}
    value._wall_labels = {"wall"}
    return value


def result(classes, confidences):
    return SimpleNamespace(
        boxes=SimpleNamespace(cls=Values(classes), conf=Values(confidences)),
        names={0: "curtain", 1: "wall"},
    )


class YoloDetectorTest(unittest.TestCase):
    def test_yolo_selects_highest_configured_detection(self):
        classification, confidence, reason, count = detector()._select_result(
            result([0, 1], [0.72, 0.91])
        )

        self.assertEqual(classification, ObstacleDetection.WALL)
        self.assertEqual(confidence, 0.91)
        self.assertEqual(reason, "detected wall")
        self.assertEqual(count, 2)

    def test_yolo_no_detection_is_valid_unknown(self):
        classification, confidence, reason, count = detector()._select_result(
            result([], [])
        )

        self.assertEqual(classification, ObstacleDetection.UNKNOWN)
        self.assertEqual(confidence, 0.0)
        self.assertIn("no configured", reason)
        self.assertEqual(count, 0)

    def test_ready_state_changes_are_latched_without_duplicate_publications(self):
        class Publisher:
            def __init__(self):
                self.values = []

            def publish(self, message):
                self.values.append(bool(message.data))

        value = object.__new__(YoloObstacleDetector)
        value._ready = None
        value._ready_publisher = Publisher()

        value._set_ready(True)
        value._set_ready(True)
        value._set_ready(False)

        self.assertEqual(value._ready_publisher.values, [True, False])

    def test_wall_vision_conflict_cannot_keep_positive_mechanical_evidence(self):
        assessment = SimpleNamespace(
            state=MechanicalAssessment.FLEXIBLE,
            mechanical_probability=0.9,
            error_code="",
            message="",
        )
        vision = SimpleNamespace(classification=ObstacleDetection.WALL)

        PiperProbeServer._enforce_vision_consistency(assessment, vision)

        self.assertEqual(assessment.state, MechanicalAssessment.UNKNOWN)
        self.assertEqual(assessment.mechanical_probability, 0.0)
        self.assertEqual(
            assessment.error_code, "PIPER_VISION_MECHANICAL_CONFLICT"
        )

    def test_zero_probability_positive_state_is_rejected(self):
        assessment = SimpleNamespace(
            state=MechanicalAssessment.MOVABLE,
            mechanical_probability=0.0,
            error_code="",
            message="",
        )

        PiperProbeServer._enforce_positive_probability(assessment)

        self.assertEqual(assessment.state, MechanicalAssessment.ERROR)
        self.assertEqual(assessment.error_code, "INVALID_MECHANICAL_PROBABILITY")

    def test_curtain_positive_result_waits_for_real_sweep_motion(self):
        required = PiperProbeServer._positive_requires_more_sweep

        self.assertTrue(required(
            True, ProbePushability.Feedback.PROBING, 1.0, 0.15
        ))
        self.assertTrue(required(
            True, ProbePushability.Feedback.SWEEPING, 0.149, 0.15
        ))
        self.assertFalse(required(
            True, ProbePushability.Feedback.SWEEPING, 0.15, 0.15
        ))
        self.assertFalse(required(
            False, ProbePushability.Feedback.PROBING, 0.0, 0.15
        ))

    def test_latest_stable_vision_uses_gripper_view_after_forward_probe(self):
        value = object.__new__(PiperProbeServer)
        value._lock = __import__("threading").RLock()
        value._vision_window = 5
        value._vision_min_votes = 3
        value._vision_error = ""
        value._vision_samples = [
            (ObstacleDetection.WALL, 0.60, "background wall"),
            (ObstacleDetection.CURTAIN, 0.41, "curtain in gripper view"),
            (ObstacleDetection.CURTAIN, 0.44, "curtain in gripper view"),
            (ObstacleDetection.CURTAIN, 0.46, "curtain in gripper view"),
            (ObstacleDetection.CURTAIN, 0.43, "curtain in gripper view"),
        ]

        refreshed = value._latest_stable_vision()

        self.assertEqual(refreshed.classification, ObstacleDetection.CURTAIN)
        self.assertEqual(refreshed.votes, 4)
        self.assertEqual(refreshed.samples, 5)

    def test_confirmed_curtain_semantics_are_not_revoked_by_background_wall(self):
        initial = SimpleNamespace(classification=ObstacleDetection.CURTAIN)
        refreshed = SimpleNamespace(classification=ObstacleDetection.WALL)

        self.assertTrue(PiperProbeServer._curtain_seen(initial, refreshed))
