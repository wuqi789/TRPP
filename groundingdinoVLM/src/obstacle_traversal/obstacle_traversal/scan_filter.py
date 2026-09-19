#!/usr/bin/env python3
"""Authorization-scoped complete obstacle-cluster LaserScan filter."""

from __future__ import annotations

import copy
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from obstacle_traversal_interfaces.msg import FilterAuthorization, FilterState
from .core import (
    filter_track_loss_in_completion_grace, path_corridor_indices,
    transform_matrix,
)


class ScanFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_traversal_scan_filter")
        for name, value in (
            ("raw_topic", "/scan_raw"), ("filtered_topic", "/scan"),
            ("removed_topic", "/scan_removed"),
            ("authorization_topic", "/obstacle_traversal/filter_authorization"),
            ("state_topic", "/obstacle_traversal/filter_state"),
            ("map_frame", "map"), ("robot_frame", "base_link"),
        ):
            self.declare_parameter(name, value)
        self.declare_parameter("target_band_half_width_m", 0.433)
        self.declare_parameter("before_obstacle_length", 0.45)
        self.declare_parameter("after_obstacle_length", 0.80)
        self.declare_parameter("track_loss_completion_grace_m", 0.20)
        self.declare_parameter("scan_watchdog", 0.50)
        self.declare_parameter("maximum_authorization_duration_s", 30.0)
        # RTX LiDAR can publish a few empty frames while the scene and
        # particle curtain finish initializing. Keep the authorization alive
        # during that startup window instead of fail-closing before the robot
        # has had a chance to see the target.
        # The public fixture may not produce valid RTX returns until Isaac has
        # finished loading the curtain particles.  Keep the route authorization
        # alive during startup; the bounded authorization timeout still fails
        # closed if the sensor never recovers.
        self.declare_parameter("initial_track_grace_s", 45.0)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        scan_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._filtered = self.create_publisher(
            LaserScan, str(self.get_parameter("filtered_topic").value), scan_qos
        )
        self._removed = self.create_publisher(
            LaserScan, str(self.get_parameter("removed_topic").value), scan_qos
        )
        self._state_pub = self.create_publisher(
            FilterState, str(self.get_parameter("state_topic").value), qos
        )
        self.create_subscription(
            LaserScan, str(self.get_parameter("raw_topic").value), self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            FilterAuthorization,
            str(self.get_parameter("authorization_topic").value),
            self._on_authorization, qos,
        )
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._lock = threading.RLock()
        self._authorization: FilterAuthorization | None = None
        self._authorized_at = 0.0
        self._last_scan_at = time.monotonic()
        self._initial_robot: tuple[float, float] | None = None
        self._initial_obstacle: tuple[float, float] | None = None
        self._path: list[tuple[float, float]] = []
        self.create_timer(0.1, self._watchdog)
        self._publish_state(False, 0, "", "Scan filter initialized in full-pass mode")

    def _on_authorization(self, message: FilterAuthorization) -> None:
        with self._lock:
            if not message.authorized:
                self._authorization = None
                self._initial_robot = None
                self._initial_obstacle = None
                self._path = []
                self._publish_state(False, 0, "", "Filter authorization revoked", message.request_id)
                return
            try:
                self._initial_robot = self._robot_xy()
            except TransformException:
                self._authorization = None
                self._publish_state(
                    False, 0, "FILTER_TF_UNAVAILABLE",
                    "Cannot authorize filtering without robot transform", message.request_id,
                )
                return
            self._authorization = copy.deepcopy(message)
            self._initial_obstacle = (
                float(message.obstacle_center_map.x),
                float(message.obstacle_center_map.y),
            )
            self._authorized_at = time.monotonic()
            map_frame = str(self.get_parameter("map_frame").value).lstrip("/")
            path_frame = message.locked_path.header.frame_id.lstrip("/") or map_frame
            path = [
                (float(pose.pose.position.x), float(pose.pose.position.y))
                for pose in message.locked_path.poses
                if math.isfinite(float(pose.pose.position.x))
                and math.isfinite(float(pose.pose.position.y))
            ]
            if path_frame != map_frame or len(path) < 2:
                self._authorization = None
                self._initial_robot = None
                self._initial_obstacle = None
                self._path = []
                self._publish_state(
                    False, 0, "FILTER_LOCKED_PATH_INVALID",
                    "Authorization does not contain a valid map-frame locked path",
                    message.request_id,
                )
                return
            self._path = path
            self._publish_state(
                True, 0, "", "Authorized path-corridor filtering", message.request_id
            )

    def _on_scan(self, message: LaserScan) -> None:
        self._last_scan_at = time.monotonic()
        with self._lock:
            authorization = copy.deepcopy(self._authorization)
            path = list(self._path)
        if authorization is None:
            self._filtered.publish(message)
            self._publish_empty_removed(message)
            return
        failure = self._expiry_or_pass_failure(authorization)
        if failure:
            self._revoke(*failure, request_id=authorization.request_id)
            self._filtered.publish(message)
            self._publish_empty_removed(message)
            return
        try:
            indices = self._path_corridor_indices(message, path)
        except TransformException:
            self._revoke(
                "FILTER_TF_UNAVAILABLE",
                "Scan transform failed during path-corridor filtering",
                request_id=authorization.request_id,
            )
            self._filtered.publish(message)
            self._publish_empty_removed(message)
            return
        if not indices:
            try:
                pass_progress, pass_distance = self._pass_progress(authorization)
            except TransformException:
                self._revoke(
                    "FILTER_TF_UNAVAILABLE",
                    "Robot transform failed while confirming pass completion",
                    request_id=authorization.request_id,
                )
                self._filtered.publish(message)
                self._publish_empty_removed(message)
                return
            if filter_track_loss_in_completion_grace(
                pass_progress,
                pass_distance,
                float(
                    self.get_parameter("track_loss_completion_grace_m").value
                ),
            ):
                self._filtered.publish(message)
                self._publish_empty_removed(message)
                self._publish_state(
                    True, 0, "",
                    "Target left rear LiDAR view; authorization held until "
                    "the 0.80 m pass distance",
                    authorization.request_id,
                )
                return
            if time.monotonic() - self._authorized_at < float(
                self.get_parameter("initial_track_grace_s").value
            ):
                self._filtered.publish(message)
                self._publish_empty_removed(message)
                self._publish_state(
                    True, 0, "",
                    "Waiting for the target occupancy band to enter LiDAR view",
                    authorization.request_id,
                )
                return
            self._revoke(
                "FILTER_TRACK_LOST",
                "No points remain in the request-locked target occupancy band",
                request_id=authorization.request_id,
            )
            self._filtered.publish(message)
            self._publish_empty_removed(message)
            return
        filtered = copy.deepcopy(message)
        removed = copy.deepcopy(message)
        filtered.ranges = list(filtered.ranges)
        removed.ranges = [math.inf] * len(message.ranges)
        for index in indices:
            removed.ranges[index] = float(message.ranges[index])
            filtered.ranges[index] = math.inf
        self._filtered.publish(filtered)
        self._removed.publish(removed)
        self._publish_state(
            True, len(indices), "",
            "Authorized request-locked target occupancy band removed",
            authorization.request_id,
        )

    def _path_corridor_indices(
        self, scan: LaserScan, path: list[tuple[float, float]]
    ) -> list[int]:
        obstacle = self._initial_obstacle
        if obstacle is None or len(path) < 2:
            return []
        points_map = self._scan_points_map(scan)
        return path_corridor_indices(
            points_map, path, obstacle,
            half_width=float(
                self.get_parameter("target_band_half_width_m").value
            ),
            before_obstacle=float(
                self.get_parameter("before_obstacle_length").value
            ),
            after_obstacle=float(
                self.get_parameter("after_obstacle_length").value
            ),
        )

    def _scan_points_map(self, scan: LaserScan) -> np.ndarray:
        source_frame = scan.header.frame_id.lstrip("/")
        transform = self._tf.lookup_transform(
            str(self.get_parameter("map_frame").value),
            source_frame, rclpy.time.Time(),
        )
        matrix = transform_matrix(
            transform.transform.translation, transform.transform.rotation
        )
        points = np.full((len(scan.ranges), 2), np.nan, dtype=float)
        for index, value in enumerate(scan.ranges):
            distance = float(value)
            if (
                not math.isfinite(distance)
                or distance <= 0.0
                or distance < float(scan.range_min)
                or distance > float(scan.range_max)
            ):
                continue
            angle = float(scan.angle_min) + index * float(scan.angle_increment)
            mapped = matrix @ [
                distance * math.cos(angle), distance * math.sin(angle), 0.0, 1.0
            ]
            points[index] = (float(mapped[0]), float(mapped[1]))
        return points

    def _expiry_or_pass_failure(self, authorization: FilterAuthorization):
        elapsed = time.monotonic() - self._authorized_at
        # Keep the protocol's 30 s safety ceiling, while allowing deployments
        # to configure a shorter local limit.  The public route completes well
        # within this bounded window after Isaac publishes odometry.
        duration = min(30.0, max(0.0, float(authorization.maximum_duration_s)))
        duration = min(
            duration,
            float(self.get_parameter("maximum_authorization_duration_s").value),
        )
        if elapsed > duration:
            return "FILTER_EXPIRED", "Filter authorization reached its maximum duration"
        try:
            robot = self._robot_xy()
        except TransformException:
            return "FILTER_TF_UNAVAILABLE", "Robot transform failed during filtering"
        initial = self._initial_robot
        if initial is None:
            return "FILTER_INITIAL_POSE_MISSING", "Initial robot pose is unavailable"
        obstacle = self._initial_obstacle
        if obstacle is None:
            return "FILTER_INITIAL_OBSTACLE_MISSING", "Initial obstacle pose is unavailable"
        direction = (obstacle[0] - initial[0], obstacle[1] - initial[1])
        length = math.hypot(*direction)
        if length > 1e-6:
            unit = (direction[0] / length, direction[1] / length)
            passed = (robot[0] - obstacle[0]) * unit[0] + (robot[1] - obstacle[1]) * unit[1]
            if passed >= min(0.80, max(0.0, float(authorization.pass_distance_m))):
                return "FILTER_PASS_COMPLETE", "Robot passed the obstacle center"
        return None

    def _pass_progress(
        self, authorization: FilterAuthorization
    ) -> tuple[float, float]:
        pass_distance = min(
            0.80, max(0.0, float(authorization.pass_distance_m))
        )
        initial = self._initial_robot
        obstacle = self._initial_obstacle
        if initial is None or obstacle is None:
            return float("-inf"), pass_distance
        direction = (obstacle[0] - initial[0], obstacle[1] - initial[1])
        length = math.hypot(*direction)
        if length <= 1e-6:
            return float("-inf"), pass_distance
        robot = self._robot_xy()
        unit = (direction[0] / length, direction[1] / length)
        return (
            (robot[0] - obstacle[0]) * unit[0]
            + (robot[1] - obstacle[1]) * unit[1],
            pass_distance,
        )

    def _robot_xy(self) -> tuple[float, float]:
        transform = self._tf.lookup_transform(
            str(self.get_parameter("map_frame").value),
            str(self.get_parameter("robot_frame").value), rclpy.time.Time()
        )
        value = transform.transform.translation
        return float(value.x), float(value.y)

    def _watchdog(self) -> None:
        with self._lock:
            authorization = copy.deepcopy(self._authorization)
        if authorization is None:
            return
        now = time.monotonic()
        # Isaac's RTX LiDAR can be silent while the stage and particle
        # sensors finish initializing.  Keep the authorization alive during
        # that bounded startup window; once it expires, fail closed as usual.
        startup_grace = max(
            0.0, float(self.get_parameter("initial_track_grace_s").value)
        )
        if now - self._authorized_at < startup_grace:
            return
        if now - self._last_scan_at > float(
            self.get_parameter("scan_watchdog").value
        ):
            self._revoke(
                "FILTER_SCAN_WATCHDOG", "Raw scan watchdog expired; full scan restored",
                request_id=authorization.request_id,
            )

    def _revoke(self, code: str, message: str, *, request_id: str) -> None:
        with self._lock:
            self._authorization = None
            self._initial_robot = None
            self._initial_obstacle = None
            self._path = []
        self._publish_state(False, 0, code, message, request_id)
        if code not in {"FILTER_EXPIRED", "FILTER_PASS_COMPLETE"}:
            self.get_logger().error(f"{code}: {message}")

    def _publish_empty_removed(self, source: LaserScan) -> None:
        removed = copy.deepcopy(source)
        removed.ranges = [math.inf] * len(source.ranges)
        self._removed.publish(removed)

    def _publish_state(
        self, active: bool, count: int, code: str, message: str, request_id: str = ""
    ) -> None:
        output = FilterState()
        output.header.stamp = self.get_clock().now().to_msg()
        output.request_id = request_id
        output.active = bool(active)
        output.removed_points = int(count)
        output.error_code = code
        output.message = message
        self._state_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
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
