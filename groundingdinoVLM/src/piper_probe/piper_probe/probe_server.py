#!/usr/bin/env python3
"""Public mechanical-probe action boundary.

The private probe policy is intentionally absent from this repository.  This
node keeps the historical action and assessment topics available for launch
and integration checks, while failing closed without sending joint commands.
Install a separately maintained provider and expose it through the same ROS
action boundary for real hardware deployments.
"""

from __future__ import annotations

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from obstacle_traversal_interfaces.action import ProbePushability
from obstacle_traversal_interfaces.msg import MechanicalAssessment


class PiperProbeServer(Node):
    """No-op public action server with a stable failure contract."""

    def __init__(self) -> None:
        super().__init__("piper_probe_server")
        self.declare_parameter("action_name", "/piper/probe_pushability")
        self.declare_parameter(
            "assessment_topic", "/obstacle_traversal/mechanical_assessment"
        )
        self.declare_parameter("provider", "external")
        self.declare_parameter("adapter_env", "SCOUT_MECHANICAL_PROBE_ADAPTER")
        self._assessment = self.create_publisher(
            MechanicalAssessment,
            str(self.get_parameter("assessment_topic").value),
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._server = ActionServer(
            self,
            ProbePushability,
            str(self.get_parameter("action_name").value),
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )

    @staticmethod
    def _goal(goal: ProbePushability.Goal) -> GoalResponse:
        return GoalResponse.ACCEPT if str(goal.request_id).strip() else GoalResponse.REJECT

    def _execute(self, handle):
        goal = handle.request
        for phase, message in (
            (ProbePushability.Feedback.POSITIONING, "Provider boundary; no actuator command"),
            (ProbePushability.Feedback.RETURNING, "No-op public probe"),
        ):
            feedback = ProbePushability.Feedback()
            feedback.phase = phase
            feedback.progress = 0.0
            feedback.resistance_ratio = 0.0
            feedback.message = message
            handle.publish_feedback(feedback)

        assessment = MechanicalAssessment()
        assessment.request_id = str(goal.request_id)
        assessment.state = MechanicalAssessment.ERROR
        assessment.contact_probability = 0.0
        assessment.mobility_probability = 0.0
        assessment.mechanical_probability = 0.0
        assessment.resistance_ratio = 0.0
        assessment.vision_classification = MechanicalAssessment.VISION_UNKNOWN
        assessment.vision_confidence = 0.0
        assessment.vision_backend_ok = False
        assessment.vision_votes = 0
        assessment.vision_samples = 0
        assessment.vision_reason = "external mechanical provider not configured"
        assessment.arm_returned = True
        assessment.error_code = "MECHANICAL_PROVIDER_UNAVAILABLE"
        assessment.message = "Public build does not include mechanical probing policy"
        self._assessment.publish(assessment)

        result = ProbePushability.Result()
        result.assessment = assessment
        handle.abort()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PiperProbeServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
