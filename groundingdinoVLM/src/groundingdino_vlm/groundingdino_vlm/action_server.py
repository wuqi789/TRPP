#!/usr/bin/env python3
"""Target verification action server with mock or external providers."""

from __future__ import annotations

import copy
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
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, RegionOfInterest
from std_msgs.msg import Bool
import yaml

from groundingdino_vlm_interfaces.action import VerifyTarget
from groundingdino_vlm_interfaces.msg import DetectionCandidate, TargetVerification

from .core import (
    Candidate, ValidationSnapshot, ValidationState, normalize_target,
    overlap_over_smaller,
)
from .detectors import DetectorError, create_detector
from .node import draw_overlay
from .vlm import VLMError, create_vlm


class VerifyTargetActionServer(Node):
    def __init__(self) -> None:
        super().__init__("groundingdino_verify_target_server")
        share = Path(get_package_share_directory("groundingdino_vlm"))
        self.declare_parameter("config_path", str(share / "config" / "mock.yaml"))
        self.declare_parameter("action_name", "/groundingdino_vlm/verify_target")
        self.declare_parameter("ready_topic", "/groundingdino_vlm/ready")
        self.declare_parameter("debug_image_topic", "/groundingdino_vlm/debug_image")
        self.declare_parameter("minimum_roi_overlap", 0.20)
        config = self._load_config(str(self.get_parameter("config_path").value))
        self._detector = create_detector(config.get("detector", {}))
        self._vlm = create_vlm(config.get("vlm", {}))
        self._minimum_overlap = float(self.get_parameter("minimum_roi_overlap").value)
        if not 0.0 <= self._minimum_overlap <= 1.0:
            raise ValueError("minimum_roi_overlap must be in [0, 1]")
        self._gpu_lock = threading.Lock()
        ready_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._ready = self.create_publisher(
            Bool, str(self.get_parameter("ready_topic").value), ready_qos
        )
        self._debug = self.create_publisher(
            CompressedImage,
            str(self.get_parameter("debug_image_topic").value),
            qos_profile_sensor_data,
        )
        self._server = ActionServer(
            self,
            VerifyTarget,
            str(self.get_parameter("action_name").value),
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.create_timer(1.0, lambda: self._ready.publish(Bool(data=True)))
        self._ready.publish(Bool(data=True))
        self.get_logger().info(
            f"VerifyTarget action ready; detector={type(self._detector).__name__}, "
            f"vlm={self._vlm.provider}"
        )

    @staticmethod
    def _load_config(path: str) -> dict:
        with Path(path).expanduser().open("r", encoding="utf-8") as stream:
            value = yaml.safe_load(stream) or {}
        if not isinstance(value, dict):
            raise ValueError("configuration root must be a mapping")
        return value

    @staticmethod
    def _goal(goal: VerifyTarget.Goal) -> GoalResponse:
        if not str(goal.request_id).strip() or not bytes(goal.image.data):
            return GoalResponse.REJECT
        if (
            int(goal.roi.width) <= 0 or int(goal.roi.height) <= 0
            or int(goal.target_roi.width) <= 0 or int(goal.target_roi.height) <= 0
        ):
            return GoalResponse.REJECT
        try:
            _, caption = normalize_target(str(goal.vocabulary))
        except ValueError:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT if caption else GoalResponse.REJECT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = VerifyTarget.Result()
        message = TargetVerification()
        message.header = copy.deepcopy(goal.image.header)
        message.request_id = str(goal.request_id)
        message.roi = copy.deepcopy(goal.roi)
        message.target_roi = copy.deepcopy(goal.target_roi)
        message.raw_target_label = str(goal.vocabulary)
        display, caption = normalize_target(str(goal.vocabulary))
        message.target_label = display
        message.target_revision = 1
        message.vlm_request_id = 1
        message.vlm_provider = self._vlm.provider
        message.vlm_model = self._vlm.model
        try:
            encoded = np.frombuffer(bytes(goal.image.data), dtype=np.uint8)
            image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError("compressed image decode failed")
            feedback = VerifyTarget.Feedback()
            feedback.state = VerifyTarget.Feedback.DETECTING
            feedback.message = "Target detector running"
            goal_handle.publish_feedback(feedback)
            with self._gpu_lock:
                detect_started = time.perf_counter()
                all_candidates = self._detector.detect(image, caption)
                message.dino_latency_ms = (time.perf_counter() - detect_started) * 1000.0
            roi = goal.target_roi
            candidates = list(all_candidates)
            overlaps = [1.0] * len(candidates)
            if int(roi.width) > 0 and int(roi.height) > 0:
                overlaps = [
                    overlap_over_smaller(value, roi)
                    for value in all_candidates
                ]
                candidates = [
                    value for value, overlap in zip(all_candidates, overlaps)
                    if overlap >= self._minimum_overlap
                ]
            self._fill_candidates(message, all_candidates)
            self._publish_debug(
                image, goal.image, goal.roi, goal.target_roi,
                all_candidates, candidates
            )
            highest = max(
                (value.confidence for value in all_candidates), default=0.0
            )
            highest_overlap = max(overlaps, default=0.0)
            phrases = sorted({value.phrase for value in all_candidates})
            self.get_logger().info(
                "VerifyTarget detector summary: "
                f"request={goal.request_id}, raw={len(all_candidates)}, "
                f"roi_matches={len(candidates)}, highest={highest:.3f}, "
                f"highest_overlap={highest_overlap:.3f}, phrases={phrases}"
            )
            message.dino_detected = bool(candidates)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                message.state = TargetVerification.REJECTED
                message.error_code = "REQUEST_CANCELED"
                message.message = "request canceled before cloud validation"
            elif not candidates:
                goal_handle.succeed()
                message.state = TargetVerification.REJECTED
                message.message = (
                    "Target detector returned no candidates"
                    if not all_candidates
                    else "No detector candidate overlaps the LiDAR ROI"
                )
            else:
                feedback.state = VerifyTarget.Feedback.VLM_PENDING
                feedback.message = "Target verification running"
                goal_handle.publish_feedback(feedback)
                overlay = self._overlay(image, candidates, display)
                vlm_started = time.perf_counter()
                accepted = self._vlm.verify(overlay, display, candidates)
                message.vlm_latency_ms = (time.perf_counter() - vlm_started) * 1000.0
                message.vlm_accepted = bool(accepted)
                message.verified = bool(accepted and candidates)
                message.state = (
                    TargetVerification.VERIFIED if message.verified
                    else TargetVerification.REJECTED
                )
                message.message = (
                    "Detector candidate and verifier result match"
                    if message.verified else "External VLM rejected candidate identity"
                )
                self.get_logger().info(
                    "VerifyTarget VLM summary: "
                    f"request={goal.request_id}, accepted={bool(accepted)}, "
                    f"candidates={len(candidates)}"
                )
                goal_handle.succeed()
        except DetectorError as exc:
            goal_handle.abort()
            message.state = TargetVerification.ERROR
            message.error_code = exc.code
            message.message = "Target detector failed closed"
            self.get_logger().error(f"{exc.code}: VerifyTarget detector failure")
        except VLMError as exc:
            goal_handle.abort()
            message.state = TargetVerification.ERROR
            message.error_code = exc.code
            message.message = "External VLM validation failed closed"
            self.get_logger().error(f"{exc.code}: VerifyTarget adapter validation failure")
        except Exception:
            goal_handle.abort()
            message.state = TargetVerification.ERROR
            message.error_code = "VERIFY_TARGET_FAILED"
            message.message = "Target verification failed closed"
            self.get_logger().error("Unexpected VerifyTarget failure")
        result.verification = message
        return result

    @staticmethod
    def _fill_candidates(message: TargetVerification, candidates: list[Candidate]) -> None:
        for value in candidates:
            item = DetectionCandidate()
            item.phrase = value.phrase
            item.grounding_confidence = float(value.confidence)
            item.x_min, item.y_min, item.x_max, item.y_max = value.bbox
            message.candidates.append(item)

    @staticmethod
    def _overlay(image: np.ndarray, candidates: list[Candidate], label: str) -> np.ndarray:
        snapshot = ValidationSnapshot(
            1, 1, label, label, ValidationState.VLM_PENDING, True, False, False,
            tuple(candidates), 0.0, 0.0, "", "",
        )
        return draw_overlay(image, snapshot)

    def _publish_debug(
        self, image: np.ndarray, source: CompressedImage,
        context_roi: RegionOfInterest, target_roi: RegionOfInterest,
        all_candidates: list[Candidate], accepted_candidates: list[Candidate],
    ) -> None:
        overlay = image.copy()
        accepted_ids = {id(value) for value in accepted_candidates}
        for value in all_candidates:
            color = (0, 255, 0) if id(value) in accepted_ids else (0, 0, 255)
            x1, y1, x2, y2 = value.bbox
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                overlay, f"{value.phrase} {value.confidence:.2f}",
                (x1, max(18, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                color, 1, cv2.LINE_AA,
            )
        if int(context_roi.width) > 0 and int(context_roi.height) > 0:
            cv2.rectangle(
                overlay,
                (int(context_roi.x_offset), int(context_roi.y_offset)),
                (
                    int(context_roi.x_offset + context_roi.width),
                    int(context_roi.y_offset + context_roi.height),
                ),
                (0, 255, 255), 2,
            )
        cv2.rectangle(
            overlay,
            (int(target_roi.x_offset), int(target_roi.y_offset)),
            (
                int(target_roi.x_offset + target_roi.width),
                int(target_roi.y_offset + target_roi.height),
            ),
            (255, 255, 0), 2,
        )
        success, encoded = cv2.imencode(
            ".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, 90]
        )
        if not success:
            return
        output = CompressedImage()
        output.header = copy.deepcopy(source.header)
        output.format = "jpeg"
        output.data = encoded.tobytes()
        self._debug.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VerifyTargetActionServer()
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
