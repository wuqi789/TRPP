#!/usr/bin/env python3
"""Asynchronous VLM pushability mapper aligned to the runtime semantic map."""

from __future__ import annotations

import copy
from dataclasses import dataclass
import math
import threading
import time
import uuid

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import message_filters
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from semantic_navigation_adapters import (
    VLMConfigurationError, VLMError, VLMTransportError, create_vlm_adapter,
)
from semantic_navigation_interfaces.msg import (
    CheckResult,
    PushabilityMap,
    PushabilityObject,
    SystemReadiness,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from .core import (
    Detection2D,
    InstanceMap,
    KeyframeScheduler,
    PushabilityResponseError,
    localize_detection,
    parse_pushability_response,
)


@dataclass(frozen=True)
class Keyframe:
    generation: int
    image_bytes: bytes
    image: np.ndarray
    depth: np.ndarray
    class_ids: np.ndarray
    class_confidence: np.ndarray
    self_mask: np.ndarray
    camera: tuple[float, float, float, float]
    transform: np.ndarray
    stamp_ns: int


class _StaleKeyframe(RuntimeError):
    pass


class PushabilityMapper(Node):
    def __init__(self) -> None:
        super().__init__("pushability_mapper")
        share = get_package_share_directory("llm_semantic_affordance")
        self.declare_parameter("config_path", f"{share}/config/affordance.yaml")
        with open(
            str(self.get_parameter("config_path").value), "r", encoding="utf-8"
        ) as stream:
            config = yaml.safe_load(stream) or {}
        vlm = config.get("vlm", {})
        keyframes = config.get("keyframes", {})
        localization = config.get("localization", {})
        topics = config.get("topics", {})

        self._response_retries = max(0, int(vlm.get("response_retries", 1)))
        self._transport_attempts = max(1, int(vlm.get("transport_attempts", 3)))
        self._transport_backoff = max(
            0.0, float(vlm.get("transport_retry_backoff", 1.0))
        )
        self._maximum_objects = int(vlm.get("maximum_objects", 32))
        self._vlm = create_vlm_adapter(
            str(vlm.get("mode", "mock")),
            str(vlm.get("adapter_env", "SCOUT_AFFORDANCE_ADAPTER")),
        )
        self._scheduler = KeyframeScheduler(
            float(keyframes.get("minimum_translation", 0.5)),
            math.radians(float(keyframes.get("minimum_rotation_degrees", 30.0))),
            float(keyframes.get("minimum_interval", 5.0)),
        )
        self._map_frame = str(localization.get("map_frame", "map"))
        self._robot_frame = str(localization.get("robot_frame", "base_link"))
        self._depth_scale = float(localization.get("depth_scale", 0.001))
        self._depth_min = float(localization.get("depth_min", 0.30))
        self._depth_max = float(localization.get("depth_max", 6.0))
        self._class_confidence_min = float(
            localization.get("class_confidence_min", 0.35)
        )
        self._ignored_class_ids = frozenset(
            int(value)
            for value in str(localization.get("ignored_class_ids", "0,3,5")).split(",")
            if value.strip()
        )
        self._pixel_stride = int(localization.get("pixel_stride", 2))
        self._minimum_points = int(localization.get("minimum_points", 20))
        self._maximum_size = float(localization.get("maximum_size", 3.5))
        self._self_mask_polygon = [
            float(value)
            for value in str(localization.get("self_mask_polygon", "")).split(",")
            if value.strip()
        ]
        self._traversal_confidence = float(
            localization.get("traversal_confidence", 0.85)
        )
        self._instances = InstanceMap(
            float(localization.get("association_iou", 0.20)),
            float(localization.get("association_distance", 0.60)),
        )

        self._bridge = CvBridge()
        self._camera_info: CameraInfo | None = None
        self._session_id = str(uuid.uuid4())
        self._revision = 0
        self._generation = 0
        self._frozen = False
        self._image_seen = False
        self._tf_seen = False
        self._cloud_healthy = False
        self._cloud_checked = False
        self._fatal_error = ""
        self._last_error = ""
        self._last_detections: list[Detection2D] = []
        self._pending: Keyframe | None = None
        self._stopping = False
        self._condition = threading.Condition(threading.RLock())

        sensor_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_publisher = self.create_publisher(
            PushabilityMap,
            str(topics.get("pushability_map", "/semantic_mapping/pushability_map")),
            transient_qos,
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray,
            str(topics.get("markers", "/semantic_mapping/pushability_markers")),
            transient_qos,
        )
        self._overlay_publisher = self.create_publisher(
            Image,
            str(topics.get("overlay", "/semantic_mapping/pushability_overlay")),
            sensor_qos,
        )
        self._readiness_publisher = self.create_publisher(
            SystemReadiness,
            str(topics.get("readiness", "/semantic_mapping/pushability_readiness")),
            transient_qos,
        )
        self.create_subscription(
            CameraInfo,
            str(topics.get("camera_info", "/isaac/color/camera_info")),
            self._on_camera_info,
            sensor_qos,
        )
        self.create_subscription(
            Bool,
            str(topics.get("mapping_active", "/semantic_mapping/active")),
            self._on_mapping_active,
            transient_qos,
        )
        self.create_subscription(
            PointStamped,
            str(topics.get("clicked_point", "/clicked_point")),
            self._on_goal_fallback,
            10,
        )
        subscriptions = [
            message_filters.Subscriber(
                self, Image, str(topics.get("rgb", "/isaac/color_image_raw")),
                qos_profile=sensor_qos,
            ),
            message_filters.Subscriber(
                self, Image,
                str(topics.get("depth", "/isaac/aligned_depth_to_color/image_raw")),
                qos_profile=sensor_qos,
            ),
            message_filters.Subscriber(
                self, Image,
                str(topics.get("class_ids", "/semantic_mapping/class_ids")),
                qos_profile=sensor_qos,
            ),
            message_filters.Subscriber(
                self, Image,
                str(topics.get(
                    "class_confidence", "/semantic_mapping/class_confidence"
                )),
                qos_profile=sensor_qos,
            ),
        ]
        self._sync = message_filters.ApproximateTimeSynchronizer(
            subscriptions, queue_size=5, slop=0.12
        )
        self._sync.registerCallback(self._on_frame)
        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=15.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._worker = threading.Thread(
            target=self._worker_loop, name="pushability-vlm", daemon=True
        )
        self._worker.start()
        self.create_timer(1.0, self._publish_readiness)
        self._publish_map()
        self.get_logger().info(
            "Pushability mapping active; sparse keyframes, session-local memory only"
        )

    def _on_camera_info(self, message: CameraInfo) -> None:
        if message.k[0] > 0.0 and message.k[4] > 0.0:
            self._camera_info = copy.deepcopy(message)

    def _on_mapping_active(self, message: Bool) -> None:
        if not message.data:
            self._freeze()

    def _on_goal_fallback(self, _: PointStamped) -> None:
        self._freeze()

    def _freeze(self) -> None:
        with self._condition:
            if self._frozen:
                return
            self._frozen = True
            self._generation += 1
            self._pending = None
            self._revision += 1
            self._condition.notify_all()
        self._publish_map()
        self.get_logger().info(
            f"Pushability map frozen with {len(self._instances)} instances; "
            "late VLM responses will be ignored"
        )

    def _on_frame(
        self, rgb_message: Image, depth_message: Image,
        class_message: Image, confidence_message: Image,
    ) -> None:
        camera_info = self._camera_info
        with self._condition:
            if self._frozen or camera_info is None:
                return
        try:
            robot_transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
            camera_transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                rgb_message.header.frame_id,
                rclpy.time.Time.from_msg(rgb_message.header.stamp),
                timeout=Duration(seconds=0.1),
            )
            position = robot_transform.transform.translation
            yaw = self._yaw(robot_transform.transform.rotation)
            now = time.monotonic()
            if not self._scheduler.should_enqueue(position.x, position.y, yaw, now):
                return
            image = self._bridge.imgmsg_to_cv2(rgb_message, desired_encoding="rgb8")
            depth = np.asarray(
                self._bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough"),
                dtype=np.float32,
            )
            if depth_message.encoding in ("16UC1", "mono16"):
                depth *= self._depth_scale
            class_ids = np.asarray(
                self._bridge.imgmsg_to_cv2(class_message, desired_encoding="mono8"),
                dtype=np.uint8,
            )
            class_confidence = np.asarray(
                self._bridge.imgmsg_to_cv2(
                    confidence_message, desired_encoding="32FC1"
                ),
                dtype=np.float32,
            )
            if not (
                image.shape[:2] == depth.shape == class_ids.shape == class_confidence.shape
            ):
                raise ValueError("synchronized image shapes do not match")
            success, encoded = cv2.imencode(
                ".png", cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            )
            if not success:
                raise ValueError("could not encode the VLM keyframe")
            keyframe = Keyframe(
                self._generation,
                encoded.tobytes(),
                image.copy(), depth.copy(), class_ids.copy(),
                class_confidence.copy(),
                self._make_self_mask(*image.shape[:2]),
                (
                    float(camera_info.k[0]), float(camera_info.k[4]),
                    float(camera_info.k[2]), float(camera_info.k[5]),
                ),
                self._transform_matrix(camera_transform),
                self._stamp_ns(rgb_message.header.stamp),
            )
            with self._condition:
                if self._frozen:
                    return
                self._pending = keyframe
                self._image_seen = True
                self._tf_seen = True
                self._scheduler.mark_enqueued(position.x, position.y, yaw, now)
                self._condition.notify_all()
        except (cv2.error, ValueError, tf2_ros.TransformException):
            self._last_error = "FRAME_PROCESSING_FAILED"
            self.get_logger().warning(
                "Skipping pushability keyframe: frame processing failed",
                throttle_duration_sec=2.0,
            )

    def _worker_loop(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._stopping:
                    self._condition.wait()
                if self._stopping:
                    return
                keyframe = self._pending
                self._pending = None
            self._process_keyframe(keyframe)

    def _process_keyframe(self, keyframe: Keyframe) -> None:
        feedback = ""
        for response_attempt in range(self._response_retries + 1):
            if not self._keyframe_current(keyframe):
                return
            try:
                raw = self._infer_with_transport_retries(keyframe, feedback)
                if not self._keyframe_current(keyframe):
                    return
                self._cloud_checked = True
                self._cloud_healthy = True
                detections = parse_pushability_response(
                    raw, keyframe.image.shape[1], keyframe.image.shape[0],
                    self._maximum_objects,
                )
                localized = []
                localization_errors = []
                for detection in detections:
                    try:
                        localized.append(localize_detection(
                            detection,
                            keyframe.depth,
                            keyframe.class_ids,
                            keyframe.class_confidence,
                            keyframe.self_mask,
                            keyframe.camera,
                            keyframe.transform,
                            keyframe.stamp_ns,
                            depth_min=self._depth_min,
                            depth_max=self._depth_max,
                            class_confidence_min=self._class_confidence_min,
                            ignored_class_ids=self._ignored_class_ids,
                            pixel_stride=self._pixel_stride,
                            minimum_points=self._minimum_points,
                            maximum_size=self._maximum_size,
                        ))
                    except PushabilityResponseError:
                        localization_errors.append("LOCALIZATION_FAILED")
                if detections and not localized:
                    raise PushabilityResponseError(
                        localization_errors[0] if localization_errors
                        else "no VLM detections could be localized"
                    )
                with self._condition:
                    if self._frozen or keyframe.generation != self._generation:
                        return
                    for value in localized:
                        self._instances.update(value)
                    self._revision += 1
                    self._last_error = ""
                    self._last_detections = detections
                self._publish_map()
                self._publish_markers()
                self._publish_overlay(keyframe.image, detections)
                return
            except _StaleKeyframe:
                return
            except VLMConfigurationError as exc:
                if not self._keyframe_current(keyframe):
                    return
                self._cloud_checked = True
                self._cloud_healthy = False
                self._fatal_error = "VLM_PROVIDER_CONFIGURATION_FAILED"
                self.get_logger().error("Pushability provider configuration failed")
                return
            except VLMTransportError as exc:
                if not self._keyframe_current(keyframe):
                    return
                self._cloud_checked = True
                self._cloud_healthy = False
                self._last_error = "VLM_PROVIDER_UNAVAILABLE"
                self.get_logger().warning("Pushability provider unavailable")
                return
            except (VLMError, PushabilityResponseError) as exc:
                if not self._keyframe_current(keyframe):
                    return
                self._last_error = "VLM_RESPONSE_INVALID"
                feedback = "response_contract_rejected"
                if response_attempt >= self._response_retries:
                    self.get_logger().warning("Pushability response rejected")

    def _infer_with_transport_retries(self, keyframe: Keyframe, feedback: str) -> str:
        for attempt in range(self._transport_attempts):
            if not self._keyframe_current(keyframe):
                raise _StaleKeyframe()
            try:
                request = self._request(keyframe.image.shape[1], keyframe.image.shape[0])
                return self._vlm.infer(
                    keyframe.image_bytes, request,
                    feedback={"reason": feedback} if feedback else None,
                    mime_type="image/png",
                )
            except VLMTransportError as exc:
                if attempt + 1 < self._transport_attempts:
                    if not self._keyframe_current(keyframe):
                        raise _StaleKeyframe()
                    time.sleep(self._transport_backoff * (attempt + 1))
        raise VLMTransportError(
            "provider unavailable after retry budget"
        )

    @staticmethod
    def _request(width: int, height: int) -> dict[str, object]:
        return {
            "contract": "scout.pushability-map.v1",
            "image": {"width_px": width, "height_px": height, "origin": "top_left"},
            "response_schema": {
                "objects": [{
                    "label": "string", "bbox_px": ["integer", "integer", "integer", "integer"],
                    "pushable": "boolean", "confidence": "number", "reason": "string",
                }]
            },
        }

    def _publish_map(self) -> None:
        output = PushabilityMap()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self._map_frame
        output.session_id = self._session_id
        output.revision = self._revision
        output.frozen = self._frozen
        for value in self._instances.snapshots():
            item = PushabilityObject()
            item.instance_id = value.instance_id
            item.label = value.label
            item.pose.position.x, item.pose.position.y, item.pose.position.z = value.center
            item.pose.orientation.w = 1.0
            item.size.x, item.size.y, item.size.z = value.size
            item.pushable = value.pushable
            item.confidence = value.confidence
            item.positive_confidence = value.positive_confidence
            item.negative_confidence = value.negative_confidence
            item.observation_count = value.observation_count
            item.last_observed.sec = value.last_observed_ns // 1_000_000_000
            item.last_observed.nanosec = value.last_observed_ns % 1_000_000_000
            # Provider rationale may contain prompts or hidden policy. Keep a
            # stable evidence marker on the public ROS message instead.
            item.evidence = "provider_evidence_redacted" if value.evidence else ""
            output.objects.append(item)
        self._map_publisher.publish(output)

    def _publish_markers(self) -> None:
        header_stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        delete = Marker()
        delete.header.frame_id = self._map_frame
        delete.header.stamp = header_stamp
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)
        marker_id = 0
        for value in self._instances.snapshots():
            confirmed = value.pushable and value.confidence >= self._traversal_confidence
            box = Marker()
            box.header.frame_id = self._map_frame
            box.header.stamp = header_stamp
            box.ns = "pushability_boxes"
            box.id = marker_id
            marker_id += 1
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x, box.pose.position.y, box.pose.position.z = value.center
            box.pose.orientation.w = 1.0
            box.scale.x, box.scale.y, box.scale.z = value.size
            if confirmed:
                box.color.b, box.color.g = 1.0, 0.45
            elif value.pushable:
                box.color.r, box.color.g = 1.0, 0.8
            else:
                box.color.r = 1.0
            box.color.a = 0.35
            markers.markers.append(box)
            text = Marker()
            text.header = box.header
            text.ns = "pushability_labels"
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = value.center[0]
            text.pose.position.y = value.center[1]
            text.pose.position.z = value.center[2] + value.size[2] / 2.0 + 0.15
            text.pose.orientation.w = 1.0
            text.scale.z = 0.20
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = (
                f"{value.label} pushable={str(value.pushable).lower()} "
                f"confidence={value.confidence:.2f}"
            )
            markers.markers.append(text)
        self._marker_publisher.publish(markers)

    def _publish_overlay(self, image: np.ndarray, detections: list[Detection2D]) -> None:
        overlay = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        for detection in detections:
            x1, y1, x2, y2 = detection.bbox
            color = (255, 150, 0) if detection.pushable else (0, 0, 255)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                overlay,
                f"{detection.label} {detection.pushable} {detection.confidence:.2f}",
                (x1, max(16, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA,
            )
        message = self._bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._map_frame
        self._overlay_publisher.publish(message)

    def _publish_readiness(self) -> None:
        adapter_ready = bool(getattr(self._vlm, "ready", False))
        checks = [
            self._check("images", self._image_seen, "IMAGE_UNAVAILABLE", "No synchronized semantic RGB-D frame"),
            self._check("tf", self._tf_seen, "TF_UNAVAILABLE", "No camera and robot transform"),
            self._check("provider", adapter_ready and not self._fatal_error, "VLM_PROVIDER_UNAVAILABLE", self._fatal_error or "Provider is not ready"),
            self._check(
                "response",
                self._cloud_checked and self._cloud_healthy,
                "VLM_RESPONSE_UNAVAILABLE",
                self._last_error or "No valid provider response yet",
            ),
        ]
        output = SystemReadiness()
        output.stamp = self.get_clock().now().to_msg()
        output.ready = all(check.status == "PASS" for check in checks)
        output.checks = checks
        if output.ready:
            output.message = (
                "Pushability map frozen" if self._frozen
                else "Pushability mapping is ready"
            )
        else:
            output.message = self._fatal_error or self._last_error or "Waiting for pushability inputs"
        self._readiness_publisher.publish(output)

    def _keyframe_current(self, keyframe: Keyframe) -> bool:
        with self._condition:
            return (
                not self._frozen
                and not self._stopping
                and keyframe.generation == self._generation
            )

    @staticmethod
    def _check(name: str, passed: bool, code: str, message: str) -> CheckResult:
        output = CheckResult()
        output.name = name
        output.status = "PASS" if passed else "FAIL"
        output.code = "" if passed else code
        output.message = "" if passed else message
        return output

    def _make_self_mask(self, height: int, width: int) -> np.ndarray:
        mask = np.zeros((height, width), dtype=bool)
        if not self._self_mask_polygon:
            return mask
        points = np.asarray(self._self_mask_polygon, dtype=np.float32).reshape(-1, 2)
        pixels = np.column_stack((
            points[:, 0] * (width - 1), points[:, 1] * (height - 1)
        )).round().astype(np.int32)
        cv2.fillPoly(mask.view(np.uint8), [pixels], 1)
        return mask

    @staticmethod
    def _transform_matrix(transform) -> np.ndarray:
        translation = transform.transform.translation
        q = transform.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = np.asarray([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        matrix[:3, 3] = (translation.x, translation.y, translation.z)
        return matrix

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _yaw(quaternion) -> float:
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
        )

    def destroy_node(self) -> bool:
        with self._condition:
            self._stopping = True
            self._pending = None
            self._condition.notify_all()
        self._worker.join(timeout=1.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PushabilityMapper()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
