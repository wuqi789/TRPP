#!/usr/bin/env python3
"""Fail-closed velocity gate between NeuPAN and the robot base."""

from __future__ import annotations

import copy
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from obstacle_traversal_interfaces.msg import (
    ApproachVelocity, FilterAuthorization, FilterState, TraversalStatus,
)
from .core import (
    outside_footprint_mask, point_clearance_from_footprint,
    transform_matrix, traversal_path_command, velocity_gate_mode,
)


class VelocityGateNode(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_traversal_velocity_gate")
        for name, value in (
            ("input_topic", "/neupan_cmd_vel_raw"),
            ("output_topic", "/neupan_cmd_vel"),
            ("status_topic", "/obstacle_traversal/status"),
            ("filter_state_topic", "/obstacle_traversal/filter_state"),
            ("authorization_topic", "/obstacle_traversal/filter_authorization"),
            ("approach_command_topic", "/obstacle_traversal/approach_cmd_vel"),
            ("filtered_scan_topic", "/scan"),
            ("robot_frame", "base_link"),
        ):
            self.declare_parameter(name, value)
        self.declare_parameter("maximum_traversal_linear_speed", 0.15)
        self.declare_parameter("status_watchdog", 0.50)
        self.declare_parameter("command_watchdog", 0.50)
        self.declare_parameter("approach_command_watchdog", 0.20)
        self.declare_parameter("traversal_lookahead_m", 0.35)
        self.declare_parameter("maximum_traversal_angular_speed", 0.25)
        self.declare_parameter("maximum_traversal_cross_track_error", 0.15)
        self.declare_parameter("maximum_traversal_heading_error", 1.20)
        self.declare_parameter("traversal_scan_watchdog", 0.50)
        self.declare_parameter("footprint_length_m", 0.62)
        self.declare_parameter("footprint_width_m", 0.586)
        self.declare_parameter("self_filter_padding_m", 0.05)
        self.declare_parameter("external_emergency_clearance_m", 0.10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self._revoke_publisher = self.create_publisher(
            FilterAuthorization,
            str(self.get_parameter("authorization_topic").value), qos,
        )
        self.create_subscription(
            Twist, str(self.get_parameter("input_topic").value), self._on_command, 10
        )
        self.create_subscription(
            ApproachVelocity,
            str(self.get_parameter("approach_command_topic").value),
            self._on_approach_command, 10,
        )
        self.create_subscription(
            TraversalStatus, str(self.get_parameter("status_topic").value),
            self._on_status, qos,
        )
        self.create_subscription(
            FilterState, str(self.get_parameter("filter_state_topic").value),
            self._on_filter_state, qos,
        )
        self.create_subscription(
            FilterAuthorization,
            str(self.get_parameter("authorization_topic").value),
            self._on_authorization,
            qos,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("filtered_scan_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self._tf = Buffer(cache_time=Duration(seconds=10.0))
        self._listener = TransformListener(self._tf, self)
        self._status: TraversalStatus | None = None
        self._filter_state: FilterState | None = None
        self._authorization: FilterAuthorization | None = None
        self._scan: LaserScan | None = None
        self._scan_at = 0.0
        self._external_clearance = math.nan
        self._last_traversal_log_at = float("-inf")
        self._status_at = 0.0
        self._command_at = time.monotonic()
        self._approach_command_at = 0.0
        self._watchdog_revoke_sent = False
        self.create_timer(0.05, self._watchdog)

    def _on_status(self, message: TraversalStatus) -> None:
        previous_mode = self._mode()
        previous_request = self._status.request_id if self._status is not None else ""
        self._status = copy.deepcopy(message)
        self._status_at = time.monotonic()
        self._watchdog_revoke_sent = False
        if previous_mode != self._mode() or previous_request != message.request_id:
            self._approach_command_at = 0.0
            self._publisher.publish(Twist())

    def _on_filter_state(self, message: FilterState) -> None:
        self._filter_state = copy.deepcopy(message)

    def _on_authorization(self, message: FilterAuthorization) -> None:
        if bool(message.authorized):
            self._authorization = copy.deepcopy(message)
        elif (
            self._authorization is not None
            and str(message.request_id) == str(self._authorization.request_id)
        ):
            self._authorization = None

    def _on_scan(self, message: LaserScan) -> None:
        self._scan = copy.deepcopy(message)
        self._scan_at = time.monotonic()

    def _on_command(self, message: Twist) -> None:
        self._command_at = time.monotonic()
        mode = self._mode()
        if mode == "STOP":
            self._publisher.publish(Twist())
            return
        if mode == "APPROACH":
            return
        if mode == "LIMIT":
            limit = float(self.get_parameter("maximum_traversal_linear_speed").value)
            command = self._locked_path_command(limit)
            clearance_safe = self._external_clearance_safe()
            now = time.monotonic()
            if now - self._last_traversal_log_at >= 0.50:
                self.get_logger().info(
                    "[TRAVERSAL_PATH_GATE] "
                    f"request={self._status.request_id if self._status else '-'} "
                    f"raw_linear={float(message.linear.x):.3f} "
                    f"raw_angular={float(message.angular.z):.3f} "
                    f"command_linear={float(command.linear) if command else 0.0:.3f} "
                    f"command_angular={float(command.angular) if command else 0.0:.3f} "
                    f"cross_track={float(command.cross_track_error) if command else math.nan:.3f} "
                    f"heading_error={float(command.heading_error) if command else math.nan:.3f} "
                    f"external_clearance={self._external_clearance:.3f} "
                    f"clearance_safe={clearance_safe}"
                )
                self._last_traversal_log_at = now
            if command is None or command.stopped or not clearance_safe:
                self._publisher.publish(Twist())
                return
            output = Twist()
            output.linear.x = float(command.linear)
            output.angular.z = float(command.angular)
            self._publisher.publish(output)
            return
        self._publisher.publish(message)

    def _locked_path_command(self, requested_linear):
        status = self._status
        filter_state = self._filter_state
        authorization = self._authorization
        if (
            status is None
            or filter_state is None
            or authorization is None
            or not bool(authorization.authorized)
            or str(status.request_id) != str(filter_state.request_id)
            or str(status.request_id) != str(authorization.request_id)
            or len(authorization.locked_path.poses) < 2
        ):
            return None
        frame = str(authorization.locked_path.header.frame_id).lstrip("/")
        if not frame:
            frame = str(
                authorization.locked_path.poses[0].header.frame_id
            ).lstrip("/")
        if not frame:
            return None
        try:
            transform = self._tf.lookup_transform(
                frame,
                str(self.get_parameter("robot_frame").value),
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            ).transform
        except TransformException:
            return None
        quaternion = transform.rotation
        yaw = math.atan2(
            2.0 * (
                quaternion.w * quaternion.z + quaternion.x * quaternion.y
            ),
            1.0 - 2.0 * (
                quaternion.y * quaternion.y + quaternion.z * quaternion.z
            ),
        )
        path = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in authorization.locked_path.poses
        ]
        return traversal_path_command(
            (float(transform.translation.x), float(transform.translation.y), yaw),
            path,
            requested_linear,
            maximum_linear=float(
                self.get_parameter("maximum_traversal_linear_speed").value
            ),
            lookahead=float(self.get_parameter("traversal_lookahead_m").value),
            maximum_angular=float(
                self.get_parameter("maximum_traversal_angular_speed").value
            ),
            maximum_cross_track_error=float(
                self.get_parameter("maximum_traversal_cross_track_error").value
            ),
            maximum_heading_error=float(
                self.get_parameter("maximum_traversal_heading_error").value
            ),
        )

    def _external_clearance_safe(self) -> bool:
        scan = self._scan
        if (
            scan is None
            or time.monotonic() - self._scan_at
            > float(self.get_parameter("traversal_scan_watchdog").value)
        ):
            self._external_clearance = math.nan
            return False
        points = []
        for index, raw_range in enumerate(scan.ranges):
            distance = float(raw_range)
            if (
                not math.isfinite(distance)
                or distance < float(scan.range_min)
                or distance > float(scan.range_max)
            ):
                continue
            angle = float(scan.angle_min) + index * float(scan.angle_increment)
            points.append((
                distance * math.cos(angle),
                distance * math.sin(angle),
                0.0,
            ))
        if not points:
            self._external_clearance = math.inf
            return True
        source = str(scan.header.frame_id).lstrip("/")
        if not source:
            self._external_clearance = math.nan
            return False
        try:
            transform = self._tf.lookup_transform(
                str(self.get_parameter("robot_frame").value),
                source,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            ).transform
        except TransformException:
            self._external_clearance = math.nan
            return False
        homogeneous = np.column_stack((np.asarray(points, dtype=float), np.ones(len(points))))
        robot_points = (
            transform_matrix(transform.translation, transform.rotation)
            @ homogeneous.T
        ).T[:, :3]
        length = float(self.get_parameter("footprint_length_m").value)
        width = float(self.get_parameter("footprint_width_m").value)
        keep = outside_footprint_mask(
            robot_points,
            length,
            width,
            padding=float(self.get_parameter("self_filter_padding_m").value),
        )
        clearance = point_clearance_from_footprint(
            robot_points[keep], length, width
        )
        self._external_clearance = math.inf if clearance is None else float(clearance)
        return clearance is None or clearance >= float(
            self.get_parameter("external_emergency_clearance_m").value
        )

    def _on_approach_command(self, message: ApproachVelocity) -> None:
        now = time.monotonic()
        status = self._status
        if (
            self._mode() != "APPROACH"
            or status is None
            or str(message.request_id) != str(status.request_id)
            or now - self._status_at > float(
                self.get_parameter("status_watchdog").value
            )
        ):
            self._approach_command_at = 0.0
            self._publisher.publish(Twist())
            return
        self._approach_command_at = now
        self._publisher.publish(copy.deepcopy(message.command))

    def _mode(self) -> str:
        status = self._status
        filter_state = self._filter_state
        filter_active = bool(filter_state is not None and filter_state.active)
        stale = (
            (status is None and filter_active)
            or (
                status is not None
                and time.monotonic() - self._status_at
                > float(self.get_parameter("status_watchdog").value)
            )
        )
        return velocity_gate_mode(
            status.state if status is not None else None,
            status_stale=stale,
            filter_active=filter_active,
            filter_matches=bool(
                status is not None and filter_state is not None
                and filter_state.request_id == status.request_id
            ),
            snapshot_pending=TraversalStatus.SNAPSHOT_PENDING,
            verifying=TraversalStatus.VERIFYING,
            authorized=TraversalStatus.AUTHORIZED,
            traversing=TraversalStatus.TRAVERSING,
            approaching=TraversalStatus.APPROACHING,
            aligning=TraversalStatus.ALIGNING,
            arm_ready=TraversalStatus.ARM_READY,
            probing=TraversalStatus.PROBING,
            arm_returning=TraversalStatus.ARM_RETURNING,
            fusing=TraversalStatus.FUSING,
            recovering=TraversalStatus.RECOVERING,
        )

    def _watchdog(self) -> None:
        mode = self._mode()
        now = time.monotonic()
        command_timeout = float(self.get_parameter("command_watchdog").value)
        approach_timeout = float(
            self.get_parameter("approach_command_watchdog").value
        )
        if (
            mode == "STOP"
            or (mode == "LIMIT" and now - self._command_at > command_timeout)
            or (mode == "APPROACH" and now - self._approach_command_at > approach_timeout)
        ):
            self._publisher.publish(Twist())
        if (
            self._filter_state is not None
            and self._filter_state.active
            and (
                self._status is None
                or time.monotonic() - self._status_at > float(
                    self.get_parameter("status_watchdog").value
                )
            )
            and not self._watchdog_revoke_sent
        ):
            revoke = FilterAuthorization()
            revoke.header.stamp = self.get_clock().now().to_msg()
            revoke.request_id = self._filter_state.request_id
            revoke.authorized = False
            self._revoke_publisher.publish(revoke)
            self._watchdog_revoke_sent = True
            self.get_logger().error(
                "TRAVERSAL_STATUS_WATCHDOG: stopped vehicle and revoked scan filtering"
            )


def main(args=None):
    rclpy.init(args=args)
    node = VelocityGateNode()
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
