#!/usr/bin/env python3
"""Public obstacle-vision topic with mock or externally supplied detection."""

from __future__ import annotations

import importlib
import os
import re
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from obstacle_traversal_interfaces.msg import ObstacleDetection


CLASS_NAMES = {
    ObstacleDetection.UNKNOWN: "UNKNOWN",
    ObstacleDetection.WALL: "WALL",
    ObstacleDetection.CURTAIN: "CURTAIN",
}
_FACTORY_SPEC = re.compile(r"^[A-Za-z_][A-Za-z0-9_.]*:[A-Za-z_][A-Za-z0-9_]*$")


class YoloObstacleDetector(Node):
    """Keep the historical node and topic names without shipping a model."""

    def __init__(self) -> None:
        super().__init__("piper_yolo_obstacle_detector")
        self.declare_parameter("image_topic", "/isaac/color_image_raw")
        self.declare_parameter("result_topic", "/piper/yolo_obstacle_result")
        self.declare_parameter("ready_topic", "/piper/yolo_ready")
        self.declare_parameter("provider", "mock")
        self.declare_parameter("adapter_env", "SCOUT_OBSTACLE_DETECTOR_ADAPTER")
        self.declare_parameter("mock_classification", "curtain")
        self.declare_parameter("mock_confidence", 0.95)
        self.declare_parameter("inference_hz", 2.0)
        self.declare_parameter("error_publish_interval_s", 2.0)

        self._provider = str(self.get_parameter("provider").value).strip().casefold()
        self._confidence = float(self.get_parameter("mock_confidence").value)
        self._inference_hz = float(self.get_parameter("inference_hz").value)
        self._last_inference_at = float("-inf")
        self._last_error_at = float("-inf")
        self._last_log = None
        self._ready = None
        if not 0.0 <= self._confidence <= 1.0 or self._inference_hz <= 0.0:
            raise ValueError("mock confidence and inference rate are invalid")
        self._adapter = None
        self._backend_error = self._configure_provider()

        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            ObstacleDetection, str(self.get_parameter("result_topic").value), 10
        )
        self._ready_publisher = self.create_publisher(
            Bool, str(self.get_parameter("ready_topic").value), transient
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self._set_ready(self._backend_error is None)
        if self._backend_error:
            self.get_logger().error(f"[OBSTACLE_VISION_UNAVAILABLE] {self._backend_error}")
        else:
            self.get_logger().info(
                f"[OBSTACLE_VISION_READY] provider={self._provider} rate_hz={self._inference_hz:.2f}"
            )

    def _configure_provider(self) -> str | None:
        if self._provider == "mock":
            return None
        if self._provider != "external":
            return "VISION_PROVIDER_INVALID: provider must be mock or external"
        env_name = str(self.get_parameter("adapter_env").value).strip()
        spec = os.getenv(env_name, "").strip()
        if not _FACTORY_SPEC.fullmatch(spec):
            return "VISION_ADAPTER_NOT_CONFIGURED: adapter must be selected with module:factory"
        module_name, factory_name = spec.split(":", 1)
        try:
            factory = getattr(importlib.import_module(module_name), factory_name)
            self._adapter = factory()
        except Exception:
            return "VISION_ADAPTER_INITIALIZATION_FAILED"
        if not callable(getattr(self._adapter, "detect", None)):
            return "VISION_ADAPTER_INVALID: adapter must provide callable detect"
        return None

    def _on_image(self, message: Image) -> None:
        now = time.monotonic()
        if now - self._last_inference_at < 1.0 / self._inference_hz:
            return
        self._last_inference_at = now
        if self._backend_error:
            self._set_ready(False)
            interval = float(self.get_parameter("error_publish_interval_s").value)
            if now - self._last_error_at >= interval:
                self._last_error_at = now
                self._publish(message, ObstacleDetection.UNKNOWN, 0.0, False,
                              "VISION_BACKEND_UNAVAILABLE", "provider_unavailable", {})
            return
        started = time.monotonic()
        try:
            classification, confidence, reason, count = self._detect(message)
            self._set_ready(True)
            self._publish(
                message, classification, confidence, True, "", reason,
                {"recognized_detection_count": float(count),
                 "processing_ms": (time.monotonic() - started) * 1000.0},
            )
        except Exception:
            reason = "VISION_ADAPTER_FAILED"
            self._set_ready(False)
            self._publish(message, ObstacleDetection.UNKNOWN, 0.0, False,
                          "VISION_ADAPTER_FAILED", reason, {})

    def _detect(self, message: Image):
        if self._provider == "mock":
            label = str(self.get_parameter("mock_classification").value).strip().casefold()
            classification = {
                "curtain": ObstacleDetection.CURTAIN,
                "wall": ObstacleDetection.WALL,
                "unknown": ObstacleDetection.UNKNOWN,
            }.get(label, ObstacleDetection.UNKNOWN)
            return classification, self._confidence, "deterministic public mock", int(classification != ObstacleDetection.UNKNOWN)
        request = {
            "contract": "scout.obstacle-detection.v1",
            "image": {"encoding": str(message.encoding), "width": int(message.width), "height": int(message.height)},
            "response_schema": {"classification": "UNKNOWN|WALL|CURTAIN", "confidence": "number"},
        }
        response = self._adapter.detect(message, request)
        if not isinstance(response, dict):
            raise ValueError("adapter response must be a mapping")
        label = str(response.get("classification", "UNKNOWN")).strip().upper()
        classification = {"CURTAIN": ObstacleDetection.CURTAIN, "WALL": ObstacleDetection.WALL}.get(label, ObstacleDetection.UNKNOWN)
        confidence = float(response.get("confidence", 0.0))
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("adapter confidence must be in [0, 1]")
        return classification, confidence, "external adapter result", int(classification != ObstacleDetection.UNKNOWN)

    def _set_ready(self, ready: bool) -> None:
        value = bool(ready)
        if self._ready == value:
            return
        self._ready = value
        self._ready_publisher.publish(Bool(data=value))

    def _publish(self, source, classification, confidence, backend_ok, error_code, reason, metrics) -> None:
        output = ObstacleDetection()
        output.header = source.header
        output.source = "piper_yolo"
        output.classification = int(classification)
        output.confidence = float(confidence)
        output.backend_ok = bool(backend_ok)
        output.error_code = str(error_code)
        output.reason = str(reason)
        output.metric_names = list(metrics)
        output.metric_values = [float(metrics[name]) for name in output.metric_names]
        self._publisher.publish(output)
        signature = (int(classification), bool(backend_ok), str(error_code))
        if signature != self._last_log:
            self._last_log = signature
            self.get_logger().info(
                f"[OBSTACLE_VISION_RESULT] class={CLASS_NAMES.get(int(classification), 'INVALID')} "
                f"confidence={float(confidence):.3f} backend_ok={bool(backend_ok)}"
            )

    def _select_result(self, result):
        """Retain the small result-selection helper for downstream unit tests."""
        candidates = []
        boxes = getattr(result, "boxes", None)
        if boxes is not None:
            class_ids = boxes.cls.detach().cpu().tolist()
            confidences = boxes.conf.detach().cpu().tolist()
            names = result.names
            for class_id, confidence in zip(class_ids, confidences):
                label = str(names[int(class_id)]).strip().casefold()
                if label == "curtain":
                    candidates.append((float(confidence), ObstacleDetection.CURTAIN, label))
                elif label == "wall":
                    candidates.append((float(confidence), ObstacleDetection.WALL, label))
        if not candidates:
            return ObstacleDetection.UNKNOWN, 0.0, "no configured obstacle class detected", 0
        confidence, classification, label = max(candidates)
        return classification, confidence, f"detected {label}", len(candidates)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloObstacleDetector()
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
