#!/usr/bin/env python3
"""Bidirectional, loop-safe adapters for the pre-refactor topic contracts."""

from __future__ import annotations

import hashlib
import math
import time
import uuid

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from llm_module1.msg import NavigationIntent as LegacyIntent
from llm_module2.msg import SemanticGoal as LegacyResolution
from llm_module3.msg import VerificationResult as LegacyVerification
from llm_module4.msg import NavigationStatus as LegacyStatus
from semantic_navigation_interfaces.msg import (
    NavigationIntent,
    NavigationTaskStatus,
    ResolutionResult,
    VerificationResult,
)


class _LoopGate:
    def __init__(self) -> None:
        self._values: dict[tuple[str, str], tuple[tuple[object, ...], float]] = {}

    def mark(self, channel: str, direction: str, signature: tuple[object, ...]) -> None:
        self._values[(channel, direction)] = (signature, time.monotonic())

    def reflected(
        self, channel: str, opposite: str, signature: tuple[object, ...]
    ) -> bool:
        value = self._values.get((channel, opposite))
        return bool(
            value is not None
            and value[0] == signature
            and time.monotonic() - value[1] < 1.0
        )


class CompatibilityNode(Node):
    def __init__(self) -> None:
        super().__init__("semantic_navigation_compatibility")
        share = get_package_share_directory("llm_module2")
        self.declare_parameter(
            "semantic_map_path", f"{share}/maps/scout_kujiale_0021.yaml"
        )
        map_path = str(self.get_parameter("semantic_map_path").value)
        with open(map_path, "rb") as stream:
            raw = stream.read()
        self._map_revision = hashlib.sha256(raw).hexdigest()
        data = yaml.safe_load(raw) or {}
        self._entities = {
            str(entry["id"]): str(entry.get("name", entry["id"]))
            for section in ("rooms", "objects", "waypoints", "doors", "corridors")
            for entry in data.get(section, [])
        }
        self._gate = _LoopGate()

        self._canonical_intent = self.create_publisher(
            NavigationIntent, "/semantic_navigation/intent", 10
        )
        self._legacy_intent = self.create_publisher(
            LegacyIntent, "/navigation_intent", 10
        )
        self._canonical_resolution = self.create_publisher(
            ResolutionResult, "/semantic_navigation/resolution", 10
        )
        self._legacy_resolution = self.create_publisher(
            LegacyResolution, "/semantic_goal", 10
        )
        self._canonical_verification = self.create_publisher(
            VerificationResult, "/semantic_navigation/verification", 10
        )
        self._legacy_verification = self.create_publisher(
            LegacyVerification, "/verified_goal", 10
        )
        self._canonical_status = self.create_publisher(
            NavigationTaskStatus, "/semantic_navigation/status", 10
        )
        self._legacy_status = self.create_publisher(
            LegacyStatus, "/navigation_status", 10
        )

        self.create_subscription(
            NavigationIntent, "/semantic_navigation/intent", self._intent_to_legacy, 10
        )
        self.create_subscription(
            LegacyIntent, "/navigation_intent", self._intent_to_canonical, 10
        )
        self.create_subscription(
            ResolutionResult,
            "/semantic_navigation/resolution",
            self._resolution_to_legacy,
            10,
        )
        self.create_subscription(
            LegacyResolution, "/semantic_goal", self._resolution_to_canonical, 10
        )
        self.create_subscription(
            VerificationResult,
            "/semantic_navigation/verification",
            self._verification_to_legacy,
            10,
        )
        self.create_subscription(
            LegacyVerification,
            "/verified_goal",
            self._verification_to_canonical,
            10,
        )
        self.create_subscription(
            NavigationTaskStatus,
            "/semantic_navigation/status",
            self._status_to_legacy,
            10,
        )
        self.create_subscription(
            LegacyStatus, "/navigation_status", self._status_to_canonical, 10
        )
        self.get_logger().info("Legacy semantic-navigation topic bridges enabled")

    @staticmethod
    def _intent_signature(value) -> tuple[object, ...]:
        return (
            value.goal_object,
            value.reference_object,
            value.relation,
            value.strategy,
            *list(value.constraints),
        )

    @staticmethod
    def _resolution_signature(value) -> tuple[object, ...]:
        x = value.goal_pose.pose.position.x if hasattr(value, "goal_pose") else value.x
        y = value.goal_pose.pose.position.y if hasattr(value, "goal_pose") else value.y
        return (value.goal_id, round(float(x), 3), round(float(y), 3))

    @staticmethod
    def _verification_signature(value) -> tuple[object, ...]:
        x = value.goal_pose.pose.position.x if hasattr(value, "goal_pose") else value.goal_x
        y = value.goal_pose.pose.position.y if hasattr(value, "goal_pose") else value.goal_y
        return (value.goal_id, round(float(x), 3), round(float(y), 3), bool(value.verified))

    @staticmethod
    def _status_signature(value) -> tuple[object, ...]:
        state = value.state if hasattr(value, "state") else value.status
        return (state, value.message, round(float(value.distance_remaining), 2))

    def _skip(self, channel: str, direction: str, signature) -> bool:
        opposite = "legacy" if direction == "canonical" else "canonical"
        if self._gate.reflected(channel, opposite, signature):
            return True
        self._gate.mark(channel, direction, signature)
        return False

    def _intent_to_legacy(self, value: NavigationIntent) -> None:
        signature = self._intent_signature(value)
        if self._skip("intent", "canonical", signature):
            return
        output = LegacyIntent()
        output.goal_object = value.goal_object
        output.reference_object = value.reference_object
        output.relation = value.relation
        output.strategy = value.strategy
        output.constraints = list(value.constraints)
        self._legacy_intent.publish(output)

    def _intent_to_canonical(self, value: LegacyIntent) -> None:
        signature = self._intent_signature(value)
        if self._skip("intent", "legacy", signature):
            return
        output = NavigationIntent()
        output.request_id = str(uuid.uuid4())
        output.goal_object = value.goal_object
        output.reference_object = value.reference_object
        output.relation = value.relation
        output.strategy = value.strategy
        output.constraints = list(value.constraints)
        self._canonical_intent.publish(output)

    def _resolution_to_legacy(self, value: ResolutionResult) -> None:
        if not value.resolved:
            return
        signature = self._resolution_signature(value)
        if self._skip("resolution", "canonical", signature):
            return
        output = LegacyResolution()
        output.goal_id = value.goal_id
        output.x = value.goal_pose.pose.position.x
        output.y = value.goal_pose.pose.position.y
        output.theta = self._yaw(value.goal_pose)
        output.topology_context = list(value.topology_context)
        self._legacy_resolution.publish(output)

    def _resolution_to_canonical(self, value: LegacyResolution) -> None:
        signature = self._resolution_signature(value)
        if self._skip("resolution", "legacy", signature):
            return
        output = ResolutionResult()
        output.resolved = True
        output.request_id = str(uuid.uuid4())
        output.intent.request_id = output.request_id
        output.map_revision = self._map_revision
        output.goal_id = value.goal_id
        output.canonical_name = self._entities.get(value.goal_id, "")
        output.match_method = "legacy"
        output.similarity = 1.0
        output.goal_pose = self._pose(value.x, value.y, value.theta)
        output.topology_context = list(value.topology_context)
        output.strategy = "shortest"
        output.message = "Converted from legacy SemanticGoal"
        self._canonical_resolution.publish(output)

    def _verification_to_legacy(self, value: VerificationResult) -> None:
        signature = self._verification_signature(value)
        if self._skip("verification", "canonical", signature):
            return
        output = LegacyVerification()
        output.verified = value.verified
        output.goal_id = value.goal_id
        output.goal_x = value.goal_pose.pose.position.x
        output.goal_y = value.goal_pose.pose.position.y
        output.goal_theta = self._yaw(value.goal_pose)
        output.failed_checks = [item.code for item in value.checks if item.status == "FAIL"]
        output.explanation = value.message
        self._legacy_verification.publish(output)

    def _verification_to_canonical(self, value: LegacyVerification) -> None:
        signature = self._verification_signature(value)
        if self._skip("verification", "legacy", signature):
            return
        output = VerificationResult()
        output.verified = value.verified
        output.request_id = str(uuid.uuid4())
        output.resolution.resolved = True
        output.resolution.request_id = output.request_id
        output.error_code = value.failed_checks[0] if value.failed_checks else ""
        output.message = value.explanation
        output.goal_id = value.goal_id
        output.canonical_name = self._entities.get(
            value.goal_id, value.goal_id.rsplit("_", 1)[0]
        )
        output.goal_pose = self._pose(value.goal_x, value.goal_y, value.goal_theta)
        output.strategy = "shortest"
        self._canonical_verification.publish(output)

    def _status_to_legacy(self, value: NavigationTaskStatus) -> None:
        signature = self._status_signature(value)
        if self._skip("status", "canonical", signature):
            return
        output = LegacyStatus()
        output.status = value.state
        output.message = value.message
        output.distance_remaining = value.distance_remaining
        self._legacy_status.publish(output)

    def _status_to_canonical(self, value: LegacyStatus) -> None:
        signature = self._status_signature(value)
        if self._skip("status", "legacy", signature):
            return
        output = NavigationTaskStatus()
        output.task_id = f"legacy-{uuid.uuid4()}"
        output.request_id = output.task_id
        output.state = value.status
        output.message = value.message
        output.distance_remaining = value.distance_remaining
        output.terminal = value.status in {"SUCCEEDED", "FAILED", "CANCELED", "PREEMPTED"}
        self._canonical_status.publish(output)

    def _pose(self, x: float, y: float, theta: float):
        from geometry_msgs.msg import PoseStamped

        output = PoseStamped()
        output.header.frame_id = "odom"
        output.header.stamp = self.get_clock().now().to_msg()
        output.pose.position.x = float(x)
        output.pose.position.y = float(y)
        output.pose.orientation.z = math.sin(float(theta) / 2.0)
        output.pose.orientation.w = math.cos(float(theta) / 2.0)
        return output

    @staticmethod
    def _yaw(pose) -> float:
        q = pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y**2 + q.z**2),
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompatibilityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
