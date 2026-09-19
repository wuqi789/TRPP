#!/usr/bin/env python3
"""ROS 2 action server for an externally supplied pushability implementation."""

from __future__ import annotations

import json
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from obstacle_traversal_interfaces.action import AssessPushability
from obstacle_traversal_interfaces.msg import PushabilityAssessment

import main as pipeline
from api.input_schema import DecisionContext
from llm.base import LLMProviderError
from vlm.base import VLMProviderError


class PushabilityActionServer(Node):
    def __init__(self) -> None:
        super().__init__("llmdecision_pushability_server")
        share = Path(get_package_share_directory("llmdecision"))
        # In a symlink install the resource files resolve back into the source
        # tree. Use the resolved config parent as the common trust root so all
        # bundled RAG/example paths stay inside the same physical package.
        pipeline.CONFIG_PATH = (share / "config" / "config.yaml").resolve()
        pipeline.MODULE_ROOT = pipeline.CONFIG_PATH.parent.parent
        self.declare_parameter("action_name", "/llmdecision/assess_pushability")
        self.declare_parameter("ready_topic", "/llmdecision/ready")
        self.declare_parameter("invalid_response_retries", 1)
        self.declare_parameter("transient_provider_retries", 1)
        self._config = pipeline._load_yaml(pipeline.CONFIG_PATH)
        llm_config = pipeline._mapping(self._config.get("llm"), "config.llm")
        vlm_config = pipeline._mapping(self._config.get("vlm"), "config.vlm")
        if str(llm_config.get("provider", "")).casefold() != "external":
            raise RuntimeError("EXTERNAL_ADAPTER_REQUIRED: decision adapter is not selected")
        if str(vlm_config.get("provider", "")).casefold() != "external":
            raise RuntimeError("EXTERNAL_ADAPTER_REQUIRED: perception adapter is not selected")
        self._vlm = pipeline._build_vlm(self._config)
        self._engine = pipeline._build_engine(self._config)
        self._provider_lock = threading.Lock()
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._ready = self.create_publisher(
            Bool, str(self.get_parameter("ready_topic").value), qos
        )
        self._server = ActionServer(
            self,
            AssessPushability,
            str(self.get_parameter("action_name").value),
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.create_timer(1.0, lambda: self._ready.publish(Bool(data=True)))
        self._ready.publish(Bool(data=True))
        self.get_logger().info("External pushability adapters are ready")

    @staticmethod
    def _goal(goal: AssessPushability.Goal) -> GoalResponse:
        if not str(goal.request_id).strip() or not bytes(goal.image.data):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = AssessPushability.Result()
        assessment = PushabilityAssessment()
        assessment.header = goal.image.header
        assessment.request_id = str(goal.request_id)
        assessment.roi = goal.roi
        assessment.target_roi = goal.target_roi
        assessment.perception_provider = "external-adapter"
        assessment.decision_provider = "external-adapter"
        assessment.decision_model = str(self._engine._llm.model)
        total_started = time.perf_counter()
        try:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                assessment.state = PushabilityAssessment.REJECTED
                assessment.error_code = "REQUEST_CANCELED"
                assessment.message = "request canceled before perception"
                result.assessment = assessment
                return result
            crop = self._crop_and_mark(
                bytes(goal.image.data), goal.roi, goal.target_roi
            )
            success, encoded = cv2.imencode(".jpg", crop, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not success:
                raise ValueError("ROI JPEG encoding failed")
            feedback = AssessPushability.Feedback()
            feedback.state = AssessPushability.Feedback.PERCEPTION
            feedback.message = "External perception adapter"
            goal_handle.publish_feedback(feedback)
            with self._provider_lock:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    assessment.state = PushabilityAssessment.REJECTED
                    assessment.error_code = "REQUEST_CANCELED"
                    assessment.message = "request canceled while waiting for perception"
                    result.assessment = assessment
                    return result
                perception_started = time.perf_counter()
                perception = self._describe_perception_with_retry(
                    encoded.tobytes(), str(goal.request_id)
                )
                assessment.mage_vl_latency_ms = (
                    time.perf_counter() - perception_started
                ) * 1000.0
                scene = perception.scene_description
                primary_index = scene.primary_obstacle_index
                if primary_index is not None and scene.obstacles:
                    primary = scene.obstacles[primary_index]
                    self.get_logger().debug(
                        "External perception returned a primary obstacle for request %s",
                        goal.request_id,
                    )
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    assessment.state = PushabilityAssessment.REJECTED
                    assessment.error_code = "REQUEST_CANCELED"
                    assessment.message = "request canceled after perception"
                    result.assessment = assessment
                    return result
                feedback.state = AssessPushability.Feedback.DECISION
                feedback.message = "External pushability adapter"
                goal_handle.publish_feedback(feedback)
                context = self._decision_context(goal.vehicle_state)
                decision_started = time.perf_counter()
                decision = self._engine.decide(
                    context.with_scene_description(perception.scene_description)
                )
                assessment.decision_latency_ms = (
                    time.perf_counter() - decision_started
                ) * 1000.0
            serialized = decision.to_dict()
            assessment.object_category = str(
                serialized["object_assessment"]["class"]
            )
            assessment.pushability_probability = float(
                serialized["pushability_probability"]
            )
            distribution = serialized["decision_distribution"]
            assessment.action_names = ["push", "avoid", "stop"]
            assessment.action_probabilities = [
                float(distribution[name]) for name in assessment.action_names
            ]
            assessment.risk = ",".join(str(x) for x in serialized["risk_flags"])
            assessment.state = PushabilityAssessment.ACCEPTED
            assessment.message = "External pushability assessment completed"
            goal_handle.succeed()
        except VLMProviderError as exc:
            assessment.state = PushabilityAssessment.ERROR
            assessment.error_code = str(exc.code)
            assessment.message = "External perception failed closed"
            goal_handle.abort()
            self.get_logger().error(f"{exc.code}: external perception request failed")
        except LLMProviderError:
            assessment.state = PushabilityAssessment.ERROR
            assessment.error_code = "EXTERNAL_DECISION_FAILED"
            assessment.message = "External decision failed closed"
            goal_handle.abort()
            self.get_logger().error("EXTERNAL_DECISION_FAILED")
        except Exception:
            assessment.state = PushabilityAssessment.ERROR
            assessment.error_code = "LLMDECISION_FAILED"
            assessment.message = "Pushability assessment failed closed"
            goal_handle.abort()
            self.get_logger().error("Unexpected llmdecision action failure")
        assessment.total_latency_ms = (time.perf_counter() - total_started) * 1000.0
        result.assessment = assessment
        return result

    @staticmethod
    def _crop_and_mark(data: bytes, roi, target_roi) -> np.ndarray:
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("compressed image decode failed")
        height, width = image.shape[:2]
        x1 = max(0, min(width - 1, int(roi.x_offset)))
        y1 = max(0, min(height - 1, int(roi.y_offset)))
        x2 = max(x1 + 1, min(width, x1 + int(roi.width)))
        y2 = max(y1 + 1, min(height, y1 + int(roi.height)))
        crop = image[y1:y2, x1:x2].copy()
        raw_target_x1 = int(target_roi.x_offset) - x1
        raw_target_y1 = int(target_roi.y_offset) - y1
        raw_target_x2 = raw_target_x1 + int(target_roi.width)
        raw_target_y2 = raw_target_y1 + int(target_roi.height)
        target_x1 = max(0, min(crop.shape[1], raw_target_x1))
        target_y1 = max(0, min(crop.shape[0], raw_target_y1))
        target_x2 = max(0, min(crop.shape[1], raw_target_x2))
        target_y2 = max(0, min(crop.shape[0], raw_target_y2))
        if target_x2 <= target_x1 or target_y2 <= target_y1:
            raise ValueError("target ROI does not overlap inference crop")
        cv2.rectangle(
            crop, (target_x1, target_y1), (target_x2 - 1, target_y2 - 1),
            (0, 255, 0), 3,
        )
        center = ((target_x1 + target_x2) // 2, (target_y1 + target_y2) // 2)
        cv2.drawMarker(
            crop, center, (0, 255, 0), cv2.MARKER_CROSS,
            markerSize=max(12, min(crop.shape[:2]) // 10), thickness=3,
        )
        return crop

    def _describe_perception_with_retry(self, image: bytes, request_id: str):
        invalid_retries = max(
            0, int(self.get_parameter("invalid_response_retries").value)
        )
        transient_retries = max(
            0, int(self.get_parameter("transient_provider_retries").value)
        )
        retries = max(invalid_retries, transient_retries)
        for attempt in range(retries + 1):
            try:
                return self._vlm.describe_bytes(
                    image,
                    request_id,
                    content_type="image/jpeg",
                    mode="targeted",
                )
            except VLMProviderError as exc:
                if exc.code == "vlm_output_invalid" and attempt < invalid_retries:
                    self.get_logger().warning(
                        "External perception returned invalid structured output; "
                        f"retrying adapter ({attempt + 1}/{invalid_retries})"
                    )
                    continue
                transient_codes = {
                    "adapter_unavailable", "adapter_timeout", "adapter_failed",
                }
                if exc.code in transient_codes and attempt < transient_retries:
                    self.get_logger().warning(
                        "External perception adapter failure; "
                        f"retrying adapter ({attempt + 1}/{transient_retries}, "
                        f"code={exc.code})"
                    )
                    time.sleep(1.0)
                    continue
                raise
        raise VLMProviderError("vlm_output_invalid")

    def _decision_context(self, state) -> DecisionContext:
        path = pipeline.MODULE_ROOT / "examples" / "input_example.json"
        document = json.loads(path.read_text(encoding="utf-8"))
        mass = max(0.1, float(state.mass_kg) + float(state.payload_kg))
        length = max(0.1, float(state.footprint_length_m))
        width = max(0.1, float(state.footprint_width_m))
        document["vehicle_state"] = {
            "mass": mass,
            "size": f"{length:g}m×{width:g}m",
            "velocity": "0m/s",
            "maximum_push_force": max(0.1, float(state.maximum_push_force_n)),
        }
        return DecisionContext.from_dict(document)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PushabilityActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
