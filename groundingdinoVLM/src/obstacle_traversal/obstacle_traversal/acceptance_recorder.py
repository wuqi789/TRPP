#!/usr/bin/env python3
"""Record compact, credential-free evidence alongside the full acceptance rosbag."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import struct
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavigationPath
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from obstacle_traversal_interfaces.action import ApproachObstacle, ProbePushability
from obstacle_traversal_interfaces.msg import (
    ApproachVelocity, FilterState, MechanicalAssessment, ObstacleDetection,
    TraversalStatus,
)
from obstacle_traversal.core import minimum_external_scan_clearance
from semantic_navigation_interfaces.msg import (
    NavigationIntent, NavigationTaskStatus, ResolutionResult, VerificationResult,
)


class AcceptanceRecorder(Node):
    def __init__(self) -> None:
        super().__init__("real_traversal_acceptance_recorder")
        self.declare_parameter("scenario", "curtain")
        self.declare_parameter("output_path", "/tmp/traversal_acceptance_events.jsonl")
        self.declare_parameter("exit_on_terminal", True)
        self.declare_parameter("lidar_offset_x_m", 0.21)
        self.declare_parameter("lidar_offset_y_m", 0.0)
        self.declare_parameter("footprint_length_m", 0.62)
        self.declare_parameter("footprint_width_m", 0.586)
        self._scenario = str(self.get_parameter("scenario").value)
        if self._scenario not in {"curtain", "movable_box", "fixed_box"}:
            raise ValueError("scenario must be curtain, movable_box, or fixed_box")
        self._path = Path(str(self.get_parameter("output_path").value)).expanduser()
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self._path.open("x", encoding="utf-8")
        self.done = False
        self._finish_after = 0.0
        self._last_sample = {}
        self._robot = None
        self._furniture = None
        self._sensor_stamps = {}
        self._active_request_id = ""
        self._fixture_visibility_misses = 0
        self._fixture_visibility_reported = False
        transient = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, "/user_instruction", self._instruction, 10)
        self.create_subscription(
            NavigationIntent, "/semantic_navigation/intent", self._intent, 10
        )
        self.create_subscription(
            ResolutionResult, "/semantic_navigation/resolution", self._resolution, 10
        )
        self.create_subscription(
            VerificationResult, "/semantic_navigation/verification", self._verification,
            10,
        )
        self.create_subscription(
            NavigationTaskStatus, "/semantic_navigation/status", self._navigation, transient
        )
        self.create_subscription(
            TraversalStatus, "/obstacle_traversal/status", self._traversal, transient
        )
        self.create_subscription(
            FilterState, "/obstacle_traversal/filter_state", self._filter, transient
        )
        self.create_subscription(
            MechanicalAssessment,
            "/obstacle_traversal/mechanical_assessment",
            self._mechanical,
            transient,
        )
        self.create_subscription(
            ObstacleDetection,
            "/piper/yolo_obstacle_result",
            self._piper_yolo,
            10,
        )
        self.create_subscription(
            ApproachVelocity,
            "/obstacle_traversal/approach_cmd_vel",
            self._approach_command,
            20,
        )
        self.create_subscription(
            ApproachObstacle.Impl.FeedbackMessage,
            "/obstacle_traversal/approach_obstacle/_action/feedback",
            self._approach_feedback,
            20,
        )
        self.create_subscription(
            ProbePushability.Impl.FeedbackMessage,
            "/piper/probe_pushability/_action/feedback",
            self._probe_feedback,
            20,
        )
        self.create_subscription(
            NavigationPath, "/neupan_initial_path", self._navigation_path, transient
        )
        for topic, name in (
            ("/scan_raw", "scan_raw"), ("/scan", "scan_filtered"),
            ("/scan_removed", "scan_removed"),
        ):
            self.create_subscription(
                LaserScan, topic, lambda msg, value=name: self._scan(value, msg),
                qos_profile_sensor_data,
            )
        for topic, name in (
            ("/neupan_cmd_vel_raw", "cmd_raw"),
            ("/neupan_cmd_vel", "cmd_gated"), ("/cmd_vel", "cmd_executed"),
        ):
            self.create_subscription(
                Twist, topic, lambda msg, value=name: self._command(value, msg), 20
            )
        self.create_subscription(Odometry, "/odom", self._odom, qos_profile_sensor_data)
        self.create_subscription(
            Image, "/isaac/color_image_raw",
            lambda msg: self._sensor_stamp("image", msg), qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, "/semantic_mapping/overlay", self._semantic_overlay,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, "/isaac/color/camera_info",
            lambda msg: self._sensor_stamp("camera", msg), qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped, "/isaac/acceptance_furniture_pose", self._furniture_pose,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2, "/semantic_mapping/obstacle_cloud", self._semantic_cloud,
            transient,
        )
        self.create_subscription(
            MarkerArray, "/semantic_mapping/labels", self._semantic_labels,
            transient,
        )
        self.create_subscription(
            Marker, "/robot_marker", self._robot_marker, 10
        )
        for topic, name in (
            ("/semantic_navigation/verification_readiness", "module3_ready"),
            ("/semantic_navigation/readiness", "module4_ready"),
            ("/groundingdino_vlm/ready", "dino_ready"),
            ("/llmdecision/ready", "llmdecision_ready"),
            ("/obstacle_traversal/ready", "traversal_ready"),
            ("/piper/yolo_ready", "piper_yolo_ready"),
        ):
            # Module3 and Module4 readiness are typed messages and are captured in
            # the bag. This compact sidecar only handles the Bool readiness topics.
            if name not in {"module3_ready", "module4_ready"}:
                self.create_subscription(
                    Bool, topic, lambda msg, value=name: self._ready(value, msg), transient
                )
        self.create_timer(1.0, self._audit_graph)
        self.create_timer(0.2, self._finish_tick)
        self._write("recorder_start", scenario=self._scenario)

    def _write(self, event: str, **values) -> None:
        record = {"event": event, "wall_time": time.time(), **values}
        self._stream.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
        self._stream.flush()

    def _instruction(self, message: String) -> None:
        value = str(message.data)
        self._write(
            "instruction", instruction_sha256=hashlib.sha256(value.encode()).hexdigest(),
            contains_expected_entity=(
                "bathroom_basin_0001" if self._scenario == "curtain"
                else "living_room_cabinet_0008"
            ) in value,
        )

    def _intent(self, message: NavigationIntent) -> None:
        self._write("module1", request_id=message.request_id, goal_object=message.goal_object)

    def _resolution(self, message: ResolutionResult) -> None:
        self._write(
            "module2", request_id=message.request_id, resolved=bool(message.resolved),
            goal_id=message.goal_id, error_code=message.error_code,
        )

    def _verification(self, message: VerificationResult) -> None:
        self._write(
            "module3", request_id=message.request_id, verified=bool(message.verified),
            goal_id=message.goal_id, error_code=message.error_code,
        )

    def _navigation(self, message: NavigationTaskStatus) -> None:
        self._write(
            "module4", request_id=message.request_id, task_id=message.task_id,
            state=message.state, terminal=bool(message.terminal),
            failure_code=message.failure_code,
        )
        if message.terminal and bool(self.get_parameter("exit_on_terminal").value):
            self._finish_after = time.monotonic() + 2.0

    def _traversal(self, message: TraversalStatus) -> None:
        if message.request_id:
            self._active_request_id = str(message.request_id)
        robot = self._robot or (None, None)
        furniture = self._furniture or (None, None)
        self._write(
            "traversal", request_id=message.request_id, task_id=message.task_id,
            obstacle_id=message.obstacle_id, state=int(message.state),
            navigation_paused=bool(message.navigation_paused),
            filter_authorized=bool(message.filter_authorized),
            dino_verified=bool(message.dino_verified),
            pushability_probability=float(message.pushability_probability),
            dino_probability=float(message.dino_probability),
            mechanical_probability=float(message.mechanical_probability),
            fusion_probability=float(message.fusion_probability),
            approach_clearance_m=float(message.approach_clearance_m),
            arm_returned=bool(message.arm_returned),
            dino_started_offset_ms=float(message.dino_started_offset_ms),
            llmdecision_started_offset_ms=float(message.llmdecision_started_offset_ms),
            elapsed_ms=float(message.elapsed_ms), removed_points=int(message.removed_points),
            error_code=message.error_code, robot_x=robot[0], robot_y=robot[1],
            obstacle_x=float(message.obstacle_center_map.x),
            obstacle_y=float(message.obstacle_center_map.y),
            furniture_x=furniture[0], furniture_y=furniture[1], message=message.message,
        )

    def _filter(self, message: FilterState) -> None:
        self._write(
            "filter", request_id=message.request_id, active=bool(message.active),
            removed_points=int(message.removed_points), error_code=message.error_code,
        )

    def _mechanical(self, message: MechanicalAssessment) -> None:
        self._write(
            "mechanical_assessment",
            request_id=message.request_id,
            state=int(message.state),
            contact_probability=float(message.contact_probability),
            mobility_probability=float(message.mobility_probability),
            mechanical_probability=float(message.mechanical_probability),
            object_displacement_m=float(message.object_displacement_m),
            resistance_ratio=float(message.resistance_ratio),
            tracking_error=float(message.tracking_error),
            vision_classification=int(message.vision_classification),
            vision_confidence=float(message.vision_confidence),
            vision_backend_ok=bool(message.vision_backend_ok),
            vision_votes=int(message.vision_votes),
            vision_samples=int(message.vision_samples),
            vision_reason=message.vision_reason,
            arm_returned=bool(message.arm_returned),
            error_code=message.error_code,
            message=message.message,
        )

    def _piper_yolo(self, message: ObstacleDetection) -> None:
        self._write(
            "piper_yolo",
            classification=int(message.classification),
            confidence=float(message.confidence),
            backend_ok=bool(message.backend_ok),
            error_code=message.error_code,
            reason=message.reason,
        )

    def _approach_command(self, message: ApproachVelocity) -> None:
        if self._sample_due("approach_cmd", 0.05):
            self._write(
                "approach_cmd",
                request_id=message.request_id,
                linear_x=float(message.command.linear.x),
                angular_z=float(message.command.angular.z),
            )

    def _approach_feedback(self, message) -> None:
        if self._sample_due("approach_feedback", 0.05):
            feedback = message.feedback
            self._write(
                "approach_feedback",
                request_id=self._active_request_id,
                phase=int(feedback.phase),
                target_clearance_m=float(feedback.target_clearance_m),
                external_clearance_m=float(feedback.global_clearance_m),
                cross_track_error_m=float(feedback.cross_track_error_m),
                heading_error_rad=float(feedback.heading_error_rad),
            )

    def _probe_feedback(self, message) -> None:
        if self._sample_due("probe_feedback", 0.05):
            feedback = message.feedback
            self._write(
                "probe_feedback",
                request_id=self._active_request_id,
                phase=int(feedback.phase),
                progress=float(feedback.progress),
                resistance_ratio=float(feedback.resistance_ratio),
                message=feedback.message,
            )

    def _navigation_path(self, message: NavigationPath) -> None:
        self._write("path", frame_id=message.header.frame_id, points=len(message.poses))

    def _scan(self, event: str, message: LaserScan) -> None:
        if event == "scan_raw":
            self._sensor_stamp("scan", message)
            self._check_fixture_visibility(message)
        if not self._sample_due(event, 0.25):
            return
        valid = [
            float(value) for value in message.ranges
            if float(value) < float("inf")
            and float(message.range_min) <= float(value) <= float(message.range_max)
        ]
        self._write(
            event,
            stamp=f"{int(message.header.stamp.sec)}.{int(message.header.stamp.nanosec):09d}",
            ranges_sha256=hashlib.sha256(
                struct.pack(f"<{len(message.ranges)}f", *message.ranges)
            ).hexdigest(),
            finite_points=len(valid),
            minimum_range=min(valid, default=None),
            minimum_external_clearance_m=(
                minimum_external_scan_clearance(
                    message.ranges,
                    message.angle_min,
                    message.angle_increment,
                    message.range_min,
                    message.range_max,
                    lidar_offset_x=float(
                        self.get_parameter("lidar_offset_x_m").value
                    ),
                    lidar_offset_y=float(
                        self.get_parameter("lidar_offset_y_m").value
                    ),
                    footprint_length=float(
                        self.get_parameter("footprint_length_m").value
                    ),
                    footprint_width=float(
                        self.get_parameter("footprint_width_m").value
                    ),
                )
                if event == "scan_raw" else None
            ),
        )

    def _check_fixture_visibility(self, message: LaserScan) -> None:
        if self._scenario == "curtain" or self._robot is None or self._furniture is None:
            return
        robot_x, robot_y, robot_yaw = self._robot
        dx = self._furniture[0] - robot_x
        dy = self._furniture[1] - robot_y
        cosine = math.cos(robot_yaw)
        sine = math.sin(robot_yaw)
        expected_x = cosine * dx + sine * dy - float(
            self.get_parameter("lidar_offset_x_m").value
        )
        expected_y = -sine * dx + cosine * dy - float(
            self.get_parameter("lidar_offset_y_m").value
        )
        expected_range = math.hypot(expected_x, expected_y)
        if expected_x <= 0.0 or expected_range > 1.50:
            self._fixture_visibility_misses = 0
            return
        visible = False
        for index, raw_range in enumerate(message.ranges):
            distance = float(raw_range)
            if not math.isfinite(distance) or not (
                float(message.range_min) <= distance <= float(message.range_max)
            ):
                continue
            angle = float(message.angle_min) + index * float(message.angle_increment)
            point_x = distance * math.cos(angle)
            point_y = distance * math.sin(angle)
            if math.hypot(point_x - expected_x, point_y - expected_y) <= 0.40:
                visible = True
                break
        if visible:
            self._fixture_visibility_misses = 0
            if not self._fixture_visibility_reported:
                self._write(
                    "acceptance_fixture_visibility",
                    visible=True,
                    expected_range_m=expected_range,
                    error_code="",
                )
                self._fixture_visibility_reported = True
            return
        self._fixture_visibility_misses += 1
        if self._fixture_visibility_misses < 10 or self._fixture_visibility_reported:
            return
        self._write(
            "acceptance_fixture_visibility",
            visible=False,
            expected_range_m=expected_range,
            error_code="ACCEPTANCE_FIXTURE_NOT_VISIBLE_TO_LIDAR",
        )
        self._fixture_visibility_reported = True
        self._finish_after = time.monotonic() + 2.0

    def _sensor_stamp(self, name: str, message) -> None:
        stamp = message.header.stamp
        self._sensor_stamps[name] = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if len(self._sensor_stamps) != 3 or not self._sample_due("sensor_sync", 0.25):
            return
        values = list(self._sensor_stamps.values())
        self._write(
            "sensor_sync",
            scan_stamp=self._sensor_stamps["scan"],
            image_stamp=self._sensor_stamps["image"],
            camera_stamp=self._sensor_stamps["camera"],
            span_s=max(values) - min(values),
        )

    def _command(self, event: str, message: Twist) -> None:
        if self._sample_due(event, 0.05):
            self._write(
                event,
                request_id=self._active_request_id,
                linear_x=float(message.linear.x),
                angular_z=float(message.angular.z),
            )

    def _odom(self, message: Odometry) -> None:
        value = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self._robot = (float(value.x), float(value.y), yaw)
        if self._sample_due("odom", 0.25):
            self._write("odom", x=self._robot[0], y=self._robot[1])

    def _furniture_pose(self, message: PoseStamped) -> None:
        value = message.pose.position
        self._furniture = (float(value.x), float(value.y))
        if self._sample_due("furniture", 0.25):
            self._write("furniture", x=self._furniture[0], y=self._furniture[1])

    def _semantic_overlay(self, message: Image) -> None:
        if self._sample_due("semantic_overlay", 1.0):
            self._write(
                "semantic_overlay", width=int(message.width),
                height=int(message.height), encoding=str(message.encoding),
            )

    def _semantic_cloud(self, message: PointCloud2) -> None:
        self._write(
            "semantic_cloud", points=int(message.width) * int(message.height)
        )

    def _semantic_labels(self, message: MarkerArray) -> None:
        self._write("semantic_labels", markers=len(message.markers))

    def _robot_marker(self, message: Marker) -> None:
        if self._sample_due("robot_marker", 0.25):
            self._write(
                "robot_marker", frame_id=message.header.frame_id,
                x=float(message.pose.position.x), y=float(message.pose.position.y),
            )

    def _ready(self, name: str, message: Bool) -> None:
        self._write("readiness", name=name, ready=bool(message.data))

    def _audit_graph(self) -> None:
        nodes = sorted(f"{namespace.rstrip('/')}/{name}" for name, namespace in self.get_node_names_and_namespaces())
        mock_nodes = [name for name in nodes if "mock" in name.casefold()]
        services = {
            name for name, _ in self.get_service_names_and_types()
        }
        piper_publishers = [
            f"{info.node_namespace.rstrip('/')}/{info.node_name}"
            for info in self.get_publishers_info_by_topic("/isaac_joint_command")
        ]
        piper_subscribers = [
            f"{info.node_namespace.rstrip('/')}/{info.node_name}"
            for info in self.get_subscriptions_info_by_topic("/isaac_joint_command")
        ]
        action_services = {
            "dino": "/groundingdino_vlm/verify_target/_action/send_goal" in services,
            "llmdecision": "/llmdecision/assess_pushability/_action/send_goal" in services,
            "approach": "/obstacle_traversal/approach_obstacle/_action/send_goal" in services,
            "piper": "/piper/probe_pushability/_action/send_goal" in services,
        }
        self._write(
            "graph_audit", nodes=nodes, mock_nodes=mock_nodes,
            action_services=action_services,
            piper_command_publishers=sorted(piper_publishers),
            piper_command_subscribers=sorted(piper_subscribers),
        )

    def _finish_tick(self) -> None:
        if self._finish_after and time.monotonic() >= self._finish_after:
            self.done = True

    def _sample_due(self, name: str, interval: float) -> bool:
        now = time.monotonic()
        if now - self._last_sample.get(name, 0.0) < interval:
            return False
        self._last_sample[name] = now
        return True

    def close(self) -> None:
        if not self._stream.closed:
            self._write("recorder_end")
            self._stream.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AcceptanceRecorder()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
