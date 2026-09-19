#!/usr/bin/env python3
"""ROS 2 adapter for public target detection and verification contracts."""

from __future__ import annotations

import copy
from pathlib import Path
import threading
import time
from typing import Any

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Header, String
import yaml

from groundingdino_vlm_interfaces.msg import DetectionCandidate, TargetVerification

from .core import TargetValidationEngine, ValidationSnapshot, VLMRequest
from .detectors import DetectorError, create_detector
from .vlm import VLMError, create_vlm


def _transient_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class GroundingDinoVLMNode(Node):
    def __init__(self, config_override: str | None = None) -> None:
        super().__init__("groundingdino_vlm_verifier")
        share = Path(get_package_share_directory("groundingdino_vlm"))
        self.declare_parameter("config_path", str(share / "config" / "mock.yaml"))
        self.declare_parameter("image_topic", "/isaac/color_image_raw")
        self.declare_parameter("target_topic", "/groundingdino_vlm/target_label")
        self.declare_parameter("result_topic", "/groundingdino_vlm/result")
        self.declare_parameter("debug_image_topic", "/groundingdino_vlm/debug_image")
        self.declare_parameter("ready_topic", "/groundingdino_vlm/ready")
        self.declare_parameter("use_compressed", False)
        config_path = config_override or str(self.get_parameter("config_path").value)
        self._config = _load_config(config_path)

        temporal = self._config.get("temporal", {})
        runtime = self._config.get("runtime", {})
        self._engine = TargetValidationEngine(
            window_size=int(temporal.get("window_size", 3)),
            required_positive=int(temporal.get("required_positive", 2)),
        )
        self._maximum_rate = float(runtime.get("maximum_inference_rate", 5.0))
        self._image_freshness = float(runtime.get("image_freshness_timeout", 2.0))
        if self._maximum_rate <= 0.0 or self._image_freshness <= 0.0:
            raise ValueError("runtime inference rate and image freshness must be positive")

        try:
            self._detector = create_detector(self._config.get("detector", {}))
            self._vlm = create_vlm(self._config.get("vlm", {}))
        except (DetectorError, VLMError) as exc:
            code = getattr(exc, "code", "PROVIDER_INITIALIZATION_FAILED")
            raise RuntimeError(code) from None

        self._bridge = CvBridge()
        self._condition = threading.Condition()
        self._latest_frame: tuple[int, Header, np.ndarray] | None = None
        self._latest_sequence = 0
        self._processed_sequence = 0
        self._last_image_monotonic: float | None = None
        self._last_header = Header()
        self._last_debug_image: np.ndarray | None = None
        self._stop = False
        self._vlm_threads: set[threading.Thread] = set()

        self._result_publisher = self.create_publisher(
            TargetVerification,
            str(self.get_parameter("result_topic").value),
            _transient_qos(),
        )
        self._debug_publisher = self.create_publisher(
            Image,
            str(self.get_parameter("debug_image_topic").value),
            qos_profile_sensor_data,
        )
        self._ready_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("ready_topic").value),
            _transient_qos(),
        )
        self.create_subscription(
            String,
            str(self.get_parameter("target_topic").value),
            self._on_target,
            _transient_qos(),
        )
        image_type = CompressedImage if bool(self.get_parameter("use_compressed").value) else Image
        self.create_subscription(
            image_type,
            str(self.get_parameter("image_topic").value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_timer(0.5, self._publish_readiness)
        self._worker = threading.Thread(
            target=self._worker_loop,
            name="groundingdino-latest-frame",
            daemon=True,
        )
        self._worker.start()
        self._publish_snapshot(self._engine.snapshot(), self._last_header)
        self._publish_readiness()
        self.get_logger().info(
            f"Independent target verifier ready (detector={type(self._detector).__name__}, "
            f"vlm={self._vlm.provider}/{self._vlm.model})"
        )

    def _on_target(self, message: String) -> None:
        try:
            snapshot = self._engine.set_target(message.data)
        except ValueError:
            self.get_logger().warning("TARGET_LABEL_INVALID")
            return
        self._publish_snapshot(snapshot, self._last_header)

    def _on_image(self, message: Image | CompressedImage) -> None:
        try:
            if isinstance(message, CompressedImage):
                raw = np.frombuffer(message.data, dtype=np.uint8)
                image = cv2.imdecode(raw, cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError("compressed image decode failed")
            else:
                image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            header = copy.deepcopy(message.header)
        except Exception:
            self.get_logger().warning("IMAGE_DECODE_FAILED")
            return
        with self._condition:
            self._latest_sequence += 1
            self._latest_frame = (self._latest_sequence, header, np.asarray(image).copy())
            self._last_image_monotonic = time.monotonic()
            self._last_header = header
            self._condition.notify()

    def _worker_loop(self) -> None:
        minimum_period = 1.0 / self._maximum_rate
        next_allowed = 0.0
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._stop
                    or (
                        self._latest_frame is not None
                        and self._latest_frame[0] != self._processed_sequence
                    ),
                    timeout=0.5,
                )
                if self._stop:
                    return
                frame = self._latest_frame
            if frame is None:
                continue
            delay = next_allowed - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            with self._condition:
                frame = self._latest_frame
                if frame is None:
                    continue
                sequence, header, image = frame
                self._processed_sequence = sequence
            target_revision, caption = self._engine.target_context
            if not caption:
                continue
            started = time.perf_counter()
            try:
                candidates = self._detector.detect(image, caption)
                latency = (time.perf_counter() - started) * 1000.0
                output = self._engine.process_detection(
                    candidates,
                    latency,
                    target_revision=target_revision,
                )
            except DetectorError as exc:
                current, snapshot = self._engine.detector_error(
                    exc.code,
                    "Target detector failed closed",
                    target_revision=target_revision,
                )
                if current:
                    self._publish_snapshot(snapshot, header)
                next_allowed = time.monotonic() + minimum_period
                continue
            if not output.current:
                next_allowed = time.monotonic() + minimum_period
                continue
            overlay = draw_overlay(image, output.snapshot)
            self._last_debug_image = overlay
            self._publish_debug_image(overlay, header)
            self._publish_snapshot(output.snapshot, header)
            if output.vlm_request is not None:
                self._start_vlm(output.vlm_request, overlay.copy(), header)
            next_allowed = time.monotonic() + minimum_period

    def _start_vlm(self, request: VLMRequest, image: np.ndarray, header: Header) -> None:
        thread = threading.Thread(
            target=self._run_vlm,
            args=(request, image, copy.deepcopy(header)),
            name=f"target-vlm-{request.request_id}",
            daemon=True,
        )
        self._vlm_threads.add(thread)
        thread.start()

    def _run_vlm(self, request: VLMRequest, image: np.ndarray, header: Header) -> None:
        started = time.perf_counter()
        error_code = ""
        message = ""
        try:
            accepted = self._vlm.verify(image, request.target_label, request.candidates)
        except VLMError as exc:
            accepted = False
            error_code = exc.code
            message = "VLM target validation failed closed"
            self.get_logger().warning(f"{exc.code}: VLM target validation failed closed")
        except Exception:
            accepted = False
            error_code = "VLM_UNEXPECTED_FAILURE"
            message = "Unexpected VLM failure"
            self.get_logger().error("Unexpected VLM target validation failure")
        latency = (time.perf_counter() - started) * 1000.0
        current, snapshot = self._engine.complete_vlm(
            request.target_revision,
            request.request_id,
            accepted,
            latency,
            error_code=error_code,
            message=message,
        )
        if current:
            self._publish_snapshot(snapshot, header)
        self._vlm_threads.discard(threading.current_thread())

    def _publish_snapshot(self, snapshot: ValidationSnapshot, header: Header) -> None:
        output = TargetVerification()
        output.header = copy.deepcopy(header)
        if output.header.stamp.sec == 0 and output.header.stamp.nanosec == 0:
            output.header.stamp = self.get_clock().now().to_msg()
        output.target_revision = snapshot.target_revision
        output.vlm_request_id = snapshot.vlm_request_id
        output.raw_target_label = snapshot.raw_target_label
        output.target_label = snapshot.target_label
        output.state = int(snapshot.state)
        output.dino_detected = snapshot.dino_detected
        output.vlm_accepted = snapshot.vlm_accepted
        output.verified = snapshot.verified
        output.vlm_provider = self._vlm.provider
        output.vlm_model = self._vlm.model
        output.dino_latency_ms = snapshot.dino_latency_ms
        output.vlm_latency_ms = snapshot.vlm_latency_ms
        output.error_code = snapshot.error_code
        output.message = snapshot.message
        for value in snapshot.candidates:
            candidate = DetectionCandidate()
            candidate.phrase = value.phrase
            candidate.grounding_confidence = value.confidence
            candidate.x_min, candidate.y_min, candidate.x_max, candidate.y_max = value.bbox
            output.candidates.append(candidate)
        self._result_publisher.publish(output)

    def _publish_debug_image(self, image: np.ndarray, header: Header) -> None:
        output = self._bridge.cv2_to_imgmsg(image, encoding="bgr8")
        output.header = copy.deepcopy(header)
        self._debug_publisher.publish(output)

    def _publish_readiness(self) -> None:
        now = time.monotonic()
        image_ready = (
            self._last_image_monotonic is not None
            and 0.0 <= now - self._last_image_monotonic <= self._image_freshness
        )
        self._ready_publisher.publish(Bool(data=bool(image_ready)))

    def destroy_node(self) -> bool:
        with self._condition:
            self._stop = True
            self._condition.notify_all()
        if hasattr(self, "_worker"):
            self._worker.join(timeout=2.0)
        return super().destroy_node()


def draw_overlay(image: np.ndarray, snapshot: ValidationSnapshot) -> np.ndarray:
    output = image.copy()
    for candidate in snapshot.candidates:
        x_min, y_min, x_max, y_max = candidate.bbox
        cv2.rectangle(output, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        cv2.putText(
            output,
            f"{candidate.phrase} {candidate.confidence:.2f}",
            (x_min, max(15, y_min - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
    cv2.putText(
        output,
        f"target={snapshot.target_label} state={snapshot.state.name}",
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return output


def _load_config(path: str) -> dict[str, Any]:
    config_path = Path(path).expanduser()
    if not config_path.is_file():
        raise FileNotFoundError(f"configuration does not exist: {config_path}")
    with config_path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream) or {}
    if not isinstance(value, dict):
        raise ValueError("configuration root must be a mapping")
    return value


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GroundingDinoVLMNode()
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
