#!/usr/bin/env python3
"""Request-scoped low-speed path approach and recovery controller."""

from __future__ import annotations

import copy
from collections import deque
import math
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from obstacle_traversal_interfaces.action import ApproachObstacle
from obstacle_traversal_interfaces.msg import ApproachVelocity

from .core import (
    approach_path_command, median_clearance, outside_footprint_mask,
    point_clearance_from_footprint, rear_sector_observable,
    target_band_observation, transform_matrix,
)


class ApproachController(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_approach_controller")
        defaults = {
            "action_name": "/obstacle_traversal/approach_obstacle",
            "command_topic": "/obstacle_traversal/approach_cmd_vel",
            "scan_topic": "/scan_raw", "odom_topic": "/odom",
            "map_frame": "map", "robot_frame": "base_link",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        numeric = {
            "footprint_length_m": 0.62, "footprint_width_m": 0.586,
            "lookahead_m": 0.35, "far_speed_mps": 0.15,
            "near_speed_mps": 0.08, "near_distance_m": 0.50,
            "maximum_angular_speed": 0.25, "alignment_speed": 0.10,
            "alignment_tolerance": 0.05, "clearance_tolerance": 0.02,
            "maximum_cross_track_error": 0.15,
            "maximum_heading_error": 0.35, "emergency_clearance_m": 0.10,
            "maximum_recoverable_heading_error": 1.20,
            "input_watchdog_s": 0.50, "target_lost_timeout_s": 0.50,
            "approach_timeout_s": 30.0, "alignment_timeout_s": 10.0,
            "stable_hold_s": 0.50, "stable_linear_mps": 0.01,
            "stable_angular_rps": 0.02, "recovery_speed_mps": 0.10,
            "track_match_distance_m": 1.0,
            "clearance_filter_frames": 3.0,
            "clearance_confirm_frames": 3.0,
            "alignment_hysteresis_m": 0.01,
            "alignment_outside_hold_s": 0.30,
            "critical_clearance_m": 0.15,
            "minimum_recoverable_clearance_m": -0.05,
            "critical_clearance_frames": 3.0,
            "correction_speed_mps": 0.03,
            "maximum_correction_distance_m": 0.30,
            "rear_sector_half_angle_rad": 0.60,
            "rear_minimum_coverage": 0.80,
            "rear_preflight_margin_m": 0.10,
            "target_band_half_width_m": 0.433,
            "before_obstacle_length_m": 0.45,
            "after_obstacle_length_m": 0.80,
            "target_face_depth_m": 0.05,
        }
        for name, value in numeric.items():
            self.declare_parameter(name, value)
        self.declare_parameter("rear_negative_one_is_no_return", False)
        self._publisher = self.create_publisher(
            ApproachVelocity, str(self.get_parameter("command_topic").value), 10
        )
        self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value), self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 10
        )
        self._tf = Buffer(cache_time=Duration(seconds=10.0))
        self._listener = TransformListener(self._tf, self)
        self._lock = threading.RLock()
        self._scan = None
        self._scan_at = 0.0
        self._odom = None
        self._odom_at = 0.0
        self._server = ActionServer(
            self, ApproachObstacle,
            str(self.get_parameter("action_name").value),
            execute_callback=self._execute, goal_callback=self._goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )

    @staticmethod
    def _goal(goal: ApproachObstacle.Goal) -> GoalResponse:
        if not str(goal.request_id).strip() or len(goal.locked_path.poses) < 2:
            return GoalResponse.REJECT
        if goal.target_clearance_m <= 0.0 or goal.recovery_distance_m < 0.0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_scan(self, message):
        with self._lock:
            self._scan = copy.deepcopy(message)
            self._scan_at = time.monotonic()

    def _on_odom(self, message):
        with self._lock:
            self._odom = copy.deepcopy(message)
            self._odom_at = time.monotonic()

    def _execute(self, handle):
        goal = handle.request
        result = ApproachObstacle.Result()
        started = time.monotonic()
        last_recovery_log_at = float("-inf")
        initial_robot = None
        stable_since = None
        alignment_started = None
        target_lost_since = None
        final_clearance = float("nan")
        traveled = 0.0
        last_target_log_at = float("-inf")
        clearance_samples = deque(
            maxlen=max(1, int(self.get_parameter("clearance_filter_frames").value))
        )
        last_clearance_scan_at = 0.0
        window_hits = 0
        critical_hits = 0
        aligning = False
        alignment_outside_since = None
        correction_origin = None
        path = [
            (float(x.pose.position.x), float(x.pose.position.y))
            for x in goal.locked_path.poses
        ]
        self.get_logger().info(
            "[APPROACH_GOAL] "
            f"request={goal.request_id} recovery={bool(goal.recovery)} "
            f"target_clearance={float(goal.target_clearance_m):.3f} "
            f"recovery_distance={float(goal.recovery_distance_m):.3f} "
            f"path_points={len(path)}"
        )
        while rclpy.ok():
            if handle.is_cancel_requested:
                self._publish(goal.request_id, 0.0, 0.0)
                handle.canceled()
                self.get_logger().info(
                    "[APPROACH_RESULT] "
                    f"request={goal.request_id} recovery={bool(goal.recovery)} "
                    "success=False reached=False stopped=False "
                    f"clearance={self._format_distance(final_clearance)} "
                    f"traveled={traveled:.3f} "
                    "error=APPROACH_CANCELED message=Approach canceled"
                )
                return self._result(result, False, False, final_clearance, traveled,
                                    "APPROACH_CANCELED", "Approach canceled")
            now = time.monotonic()
            with self._lock:
                scan, odom = copy.deepcopy(self._scan), copy.deepcopy(self._odom)
                scan_at, odom_at = self._scan_at, self._odom_at
            watchdog = float(self.get_parameter("input_watchdog_s").value)
            if scan is None or odom is None or now - scan_at > watchdog or now - odom_at > watchdog:
                return self._abort(handle, result, goal.request_id, final_clearance,
                                   traveled, "APPROACH_INPUT_STALE",
                                   "Scan or odometry is stale")
            try:
                robot = self._robot_pose()
                points_map, points_robot, _ = self._scan_points(scan)
            except TransformException:
                return self._abort(handle, result, goal.request_id, final_clearance,
                                   traveled, "APPROACH_TF_UNAVAILABLE",
                                   "Approach transform unavailable")
            if initial_robot is None:
                initial_robot = robot[:2]
            traveled = math.dist(initial_robot, robot[:2])
            if goal.recovery:
                recovery_speed = float(self.get_parameter("recovery_speed_mps").value)
                recovery_timeout = float(goal.recovery_distance_m) / max(
                    recovery_speed, 1e-3
                ) + 5.0
                if now - started > recovery_timeout:
                    return self._abort(
                        handle, result, goal.request_id, final_clearance,
                        traveled, "RECOVERY_TIMEOUT", "Controlled recovery timed out",
                    )
                if traveled >= float(goal.recovery_distance_m):
                    self._publish(goal.request_id, 0.0, 0.0)
                    twist = odom.twist.twist
                    stable = (
                        abs(twist.linear.x) <= float(
                            self.get_parameter("stable_linear_mps").value
                        )
                        and abs(twist.angular.z) <= float(
                            self.get_parameter("stable_angular_rps").value
                        )
                    )
                    stable_since = (stable_since or now) if stable else None
                    if stable_since is not None and now - stable_since >= float(
                        self.get_parameter("stable_hold_s").value
                    ):
                        handle.succeed()
                        self.get_logger().info(
                            "[APPROACH_RESULT] "
                            f"request={goal.request_id} recovery=True success=True "
                            "reached=True stopped=True "
                            f"clearance={self._format_distance(final_clearance)} "
                            f"traveled={traveled:.3f} error=none "
                            "message=Controlled recovery completed and stopped"
                        )
                        return self._result(
                            result, True, True, final_clearance, traveled, "",
                            "Controlled recovery completed and stopped",
                        )
                    time.sleep(0.05)
                    continue
                half_width = float(self.get_parameter("footprint_width_m").value) / 2.0
                rear_points = points_robot[
                    (points_robot[:, 0] < 0.0)
                    & (np.abs(points_robot[:, 1]) <= half_width + 0.1)
                ]
                rear_clearance = (
                    self._global_clearance(rear_points) if len(rear_points) else None
                )
                rear_observable, rear_coverage, rear_samples = rear_sector_observable(
                    scan.ranges, scan.angle_min, scan.angle_increment,
                    scan.range_min, scan.range_max,
                    half_angle=float(
                        self.get_parameter("rear_sector_half_angle_rad").value
                    ),
                    minimum_coverage=float(
                        self.get_parameter("rear_minimum_coverage").value
                    ),
                    negative_one_is_no_return=bool(
                        self.get_parameter("rear_negative_one_is_no_return").value
                    ),
                )
                if rear_clearance is None and rear_observable:
                    rear_clearance = math.inf
                global_clearance = self._global_clearance(points_robot)
                emergency_clearance = float(
                    self.get_parameter("emergency_clearance_m").value
                )
                if now - last_recovery_log_at >= 0.20:
                    self.get_logger().info(
                        "[RECOVERY_SAMPLE] "
                        f"request={goal.request_id} rear_points={len(rear_points)} "
                        f"rear_clearance={self._format_distance(rear_clearance)} "
                        f"rear_observable={rear_observable} "
                        f"rear_coverage={rear_coverage:.3f} rear_samples={rear_samples} "
                        f"global_clearance={self._format_distance(global_clearance)} "
                        f"traveled={traveled:.3f} "
                        f"target_distance={float(goal.recovery_distance_m):.3f} "
                        f"emergency_threshold={emergency_clearance:.3f}"
                    )
                    last_recovery_log_at = now
                required_rear = max(
                    emergency_clearance,
                    float(goal.recovery_distance_m) - traveled
                    + float(self.get_parameter("rear_preflight_margin_m").value),
                )
                if not rear_observable:
                    return self._abort(
                        handle, result, goal.request_id, final_clearance,
                        traveled, "RECOVERY_REAR_UNOBSERVABLE",
                        "Rear LaserScan sector is not observable",
                    )
                if rear_clearance is None or rear_clearance < required_rear:
                    return self._abort(handle, result, goal.request_id, final_clearance,
                                       traveled, "RECOVERY_REAR_BLOCKED",
                                       "Rear clearance is unsafe")
                path_command = approach_path_command(
                    robot, path, 1.0,
                    lookahead=float(self.get_parameter("lookahead_m").value),
                    far_speed=recovery_speed, near_speed=recovery_speed,
                    max_angular=float(self.get_parameter("maximum_angular_speed").value),
                )
                if abs(path_command.cross_track_error) > float(
                    self.get_parameter("maximum_cross_track_error").value
                ):
                    return self._abort(
                        handle, result, goal.request_id, final_clearance,
                        traveled, "RECOVERY_PATH_DEVIATION",
                        "Recovery left the locked path corridor",
                    )
                angular = max(
                    -float(self.get_parameter("maximum_angular_speed").value),
                    min(
                        float(self.get_parameter("maximum_angular_speed").value),
                        -float(path_command.angular),
                    ),
                )
                self._publish(goal.request_id, -recovery_speed, angular)
                self._feedback(handle, ApproachObstacle.Feedback.RECOVERING,
                               final_clearance, self._global_clearance(points_robot),
                               path_command.cross_track_error,
                               path_command.heading_error)
                time.sleep(0.05)
                continue
            if now - started > float(self.get_parameter("approach_timeout_s").value):
                return self._abort(handle, result, goal.request_id, final_clearance,
                                   traveled, "APPROACH_TIMEOUT",
                                   "Could not reach arm working clearance")
            observation = target_band_observation(
                points_map,
                points_robot,
                path,
                (goal.obstacle_center_map.x, goal.obstacle_center_map.y),
                robot,
                footprint_length=float(
                    self.get_parameter("footprint_length_m").value
                ),
                footprint_width=float(
                    self.get_parameter("footprint_width_m").value
                ),
                half_width=float(
                    self.get_parameter("target_band_half_width_m").value
                ),
                before_obstacle=float(
                    self.get_parameter("before_obstacle_length_m").value
                ),
                after_obstacle=float(
                    self.get_parameter("after_obstacle_length_m").value
                ),
                face_depth=float(
                    self.get_parameter("target_face_depth_m").value
                ),
            )
            if observation is None:
                target_lost_since = target_lost_since or now
                if now - target_lost_since > float(
                    self.get_parameter("target_lost_timeout_s").value
                ):
                    return self._abort(handle, result, goal.request_id, final_clearance,
                                       traveled, "APPROACH_TARGET_LOST",
                                       "Locked target occupancy band was lost")
                self._publish(goal.request_id, 0.0, 0.0)
                time.sleep(0.05)
                continue
            target_lost_since = None
            measured_clearance = float(observation.clearance)
            new_scan = scan_at > last_clearance_scan_at
            if new_scan:
                clearance_samples.append(float(measured_clearance))
                last_clearance_scan_at = scan_at
            filtered = median_clearance(
                clearance_samples,
                max(1, int(self.get_parameter("clearance_filter_frames").value)),
            )
            if filtered is None:
                continue
            final_clearance = filtered
            external_points = points_robot[list(observation.external_indices)]
            global_clearance = self._global_clearance(external_points)
            command = approach_path_command(
                robot, path, final_clearance,
                target_clearance=float(goal.target_clearance_m),
                clearance_tolerance=float(self.get_parameter("clearance_tolerance").value),
                lookahead=float(self.get_parameter("lookahead_m").value),
                far_speed=float(self.get_parameter("far_speed_mps").value),
                near_speed=float(self.get_parameter("near_speed_mps").value),
                near_distance=float(self.get_parameter("near_distance_m").value),
                max_angular=float(self.get_parameter("maximum_angular_speed").value),
            )
            if now - last_target_log_at >= 0.20:
                self.get_logger().info(
                    "[APPROACH_SAMPLE] "
                    f"request={goal.request_id} "
                    f"target_points={len(observation.target_indices)} "
                    f"external_points={len(observation.external_indices)} "
                    f"target_clearance={final_clearance:.3f} "
                    f"external_clearance={self._format_distance(global_clearance)} "
                    f"surface_bearing={observation.bearing:.3f} "
                    f"path_heading_error={command.heading_error:.3f} "
                    f"traveled={traveled:.3f}"
                )
                last_target_log_at = now
            if global_clearance < float(self.get_parameter("emergency_clearance_m").value):
                return self._abort(handle, result, goal.request_id, final_clearance,
                                   traveled, "APPROACH_CLEARANCE_UNSAFE",
                                   "Non-target external clearance is unsafe")
            if abs(command.cross_track_error) > float(
                self.get_parameter("maximum_cross_track_error").value
            ):
                return self._abort(handle, result, goal.request_id, final_clearance,
                                   traveled, "APPROACH_PATH_DEVIATION",
                                   "Locked path cross-track deviation exceeded")
            if abs(command.heading_error) > float(
                self.get_parameter("maximum_recoverable_heading_error").value
            ):
                return self._abort(handle, result, goal.request_id, final_clearance,
                                   traveled, "APPROACH_PATH_DEVIATION",
                                   "Locked path heading deviation is unrecoverable")
            # The fixed first-version Piper probe extends along the chassis
            # forward axis. Keep that axis on the request-locked route instead
            # of steering toward an arbitrary nearest curtain fold.
            target_bearing = float(command.heading_error)
            phase = ApproachObstacle.Feedback.APPROACHING
            linear, angular = command.linear, command.angular
            tolerance = float(self.get_parameter("clearance_tolerance").value)
            minimum = float(goal.target_clearance_m) - tolerance
            maximum = float(goal.target_clearance_m) + tolerance
            hysteresis = float(self.get_parameter("alignment_hysteresis_m").value)
            if new_scan:
                window_hits = window_hits + 1 if minimum <= final_clearance <= maximum else 0
                critical_hits = (
                    critical_hits + 1
                    if final_clearance < float(
                        self.get_parameter("minimum_recoverable_clearance_m").value
                    ) else 0
                )
            if critical_hits >= int(
                self.get_parameter("critical_clearance_frames").value
            ):
                return self._abort(
                    handle, result, goal.request_id, final_clearance,
                    traveled, "APPROACH_TARGET_TOO_CLOSE",
                    "Target penetrated beyond the safe correction envelope",
                )
            if not aligning and window_hits >= int(
                self.get_parameter("clearance_confirm_frames").value
            ):
                aligning = True
                alignment_started = now
                alignment_outside_since = None
                correction_origin = None
            if aligning:
                inside_guard = minimum - hysteresis <= final_clearance <= maximum + hysteresis
                alignment_outside_since = None if inside_guard else (
                    alignment_outside_since or now
                )
                if (
                    alignment_outside_since is not None
                    and now - alignment_outside_since >= float(
                        self.get_parameter("alignment_outside_hold_s").value
                    )
                ):
                    aligning = False
                    stable_since = None
                    alignment_started = None
                    correction_origin = None
                else:
                    phase = ApproachObstacle.Feedback.ALIGNING
                    linear = 0.0
                    angular = 0.0 if not inside_guard else max(
                        -float(self.get_parameter("alignment_speed").value),
                        min(float(self.get_parameter("alignment_speed").value), target_bearing),
                    )
            if aligning:
                phase = ApproachObstacle.Feedback.ALIGNING
                if now - alignment_started > float(
                    self.get_parameter("alignment_timeout_s").value
                ):
                    return self._abort(handle, result, goal.request_id, final_clearance,
                                       traveled, "ALIGNMENT_TIMEOUT",
                                       "Obstacle alignment timed out")
                if (
                    minimum <= final_clearance <= maximum
                    and abs(target_bearing) <= float(
                        self.get_parameter("alignment_tolerance").value
                    )
                ):
                    angular = 0.0
                    twist = odom.twist.twist
                    stable = (
                        abs(twist.linear.x) <= float(self.get_parameter("stable_linear_mps").value)
                        and abs(twist.angular.z) <= float(
                            self.get_parameter("stable_angular_rps").value
                        )
                    )
                    stable_since = (stable_since or now) if stable else None
                    if stable_since is not None and now - stable_since >= float(
                        self.get_parameter("stable_hold_s").value
                    ):
                        self._publish(goal.request_id, 0.0, 0.0)
                        handle.succeed()
                        self.get_logger().info(
                            "[APPROACH_RESULT] "
                            f"request={goal.request_id} recovery=False success=True "
                            "reached=True stopped=True "
                            f"clearance={self._format_distance(final_clearance)} "
                            f"traveled={traveled:.3f} error=none "
                            "message=Arm working pose reached"
                        )
                        return self._result(result, True, True, final_clearance, traveled,
                                            "", "Arm working pose reached")
                else:
                    stable_since = None
            elif final_clearance < minimum:
                if correction_origin is None:
                    correction_origin = robot[:2]
                correction_distance = math.dist(correction_origin, robot[:2])
                if correction_distance > float(
                    self.get_parameter("maximum_correction_distance_m").value
                ):
                    return self._abort(
                        handle, result, goal.request_id, final_clearance,
                        traveled, "APPROACH_CORRECTION_EXCEEDED",
                        "Could not restore the arm working clearance",
                    )
                rear_clearance = self._rear_clearance(points_robot)
                rear_observable, _, _ = rear_sector_observable(
                    scan.ranges, scan.angle_min, scan.angle_increment,
                    scan.range_min, scan.range_max,
                    half_angle=float(
                        self.get_parameter("rear_sector_half_angle_rad").value
                    ),
                    minimum_coverage=float(
                        self.get_parameter("rear_minimum_coverage").value
                    ),
                    negative_one_is_no_return=bool(
                        self.get_parameter("rear_negative_one_is_no_return").value
                    ),
                )
                if rear_clearance is None and rear_observable:
                    rear_clearance = math.inf
                if not rear_observable:
                    return self._abort(
                        handle, result, goal.request_id, final_clearance,
                        traveled, "APPROACH_CORRECTION_REAR_UNOBSERVABLE",
                        "Rear LaserScan sector is not observable for correction",
                    )
                correction_remaining = max(0.0, minimum - final_clearance)
                required_rear = correction_remaining + float(
                    self.get_parameter("rear_preflight_margin_m").value
                )
                if rear_clearance is None or rear_clearance < required_rear:
                    return self._abort(
                        handle, result, goal.request_id, final_clearance,
                        traveled, "APPROACH_CORRECTION_BLOCKED",
                        "Rear clearance prevents restoring the arm working distance",
                    )
                linear = -float(self.get_parameter("correction_speed_mps").value)
                angular = 0.0
            elif final_clearance <= maximum:
                linear, angular = 0.0, 0.0
                correction_origin = None
            else:
                correction_origin = None
            if (
                not aligning
                and final_clearance >= minimum
                and abs(target_bearing) > float(
                    self.get_parameter("maximum_heading_error").value
                )
            ):
                phase = ApproachObstacle.Feedback.ALIGNING
                linear = 0.0
                angular = max(
                    -float(self.get_parameter("alignment_speed").value),
                    min(
                        float(self.get_parameter("alignment_speed").value),
                        target_bearing,
                    ),
                )
            self._publish(goal.request_id, linear, angular)
            self._feedback(handle, phase, final_clearance, global_clearance,
                           command.cross_track_error, target_bearing)
            time.sleep(0.05)
        return self._abort(handle, result, goal.request_id, final_clearance,
                           traveled, "APPROACH_SHUTDOWN", "Controller shut down")

    def _scan_points(self, scan):
        source = scan.header.frame_id.lstrip("/")
        map_tf = self._tf.lookup_transform(
            str(self.get_parameter("map_frame").value), source, rclpy.time.Time(),
            timeout=Duration(seconds=0.1),
        )
        robot_tf = self._tf.lookup_transform(
            str(self.get_parameter("robot_frame").value), source, rclpy.time.Time(),
            timeout=Duration(seconds=0.1),
        )
        raw, indices = [], []
        for index, value in enumerate(scan.ranges):
            distance = float(value)
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                angle = scan.angle_min + index * scan.angle_increment
                raw.append((distance * math.cos(angle), distance * math.sin(angle), 0.0))
                indices.append(index)
        points = np.asarray(raw, dtype=float)
        if not len(points):
            return np.empty((0, 3)), np.empty((0, 3)), []
        homogeneous = np.column_stack((points, np.ones(len(points))))
        mapped = (transform_matrix(map_tf.transform.translation, map_tf.transform.rotation)
                  @ homogeneous.T).T[:, :3]
        robot = (transform_matrix(robot_tf.transform.translation, robot_tf.transform.rotation)
                 @ homogeneous.T).T[:, :3]
        keep = outside_footprint_mask(
            robot, float(self.get_parameter("footprint_length_m").value),
            float(self.get_parameter("footprint_width_m").value), padding=0.0,
        )
        return mapped[keep], robot[keep], [i for i, value in zip(indices, keep) if value]

    def _robot_pose(self):
        value = self._tf.lookup_transform(
            str(self.get_parameter("map_frame").value),
            str(self.get_parameter("robot_frame").value), rclpy.time.Time(),
            timeout=Duration(seconds=0.1),
        ).transform
        q = value.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (float(value.translation.x), float(value.translation.y), yaw)

    def _global_clearance(self, points):
        value = point_clearance_from_footprint(
            points, float(self.get_parameter("footprint_length_m").value),
            float(self.get_parameter("footprint_width_m").value),
        )
        return float(value) if value is not None else math.inf

    def _rear_clearance(self, points):
        half_width = float(self.get_parameter("footprint_width_m").value) / 2.0
        rear = points[(points[:, 0] < 0.0) & (np.abs(points[:, 1]) <= half_width + 0.1)]
        return self._global_clearance(rear) if len(rear) else None

    @staticmethod
    def _format_distance(value):
        if value is None:
            return "none"
        return f"{float(value):.3f}"

    def _publish(self, request_id, linear, angular):
        message = ApproachVelocity()
        message.header.stamp = self.get_clock().now().to_msg()
        message.request_id = str(request_id)
        message.command.linear.x = float(linear)
        message.command.angular.z = float(angular)
        self._publisher.publish(message)

    @staticmethod
    def _feedback(handle, phase, target, global_value, cross, heading):
        value = ApproachObstacle.Feedback()
        value.phase = phase
        value.target_clearance_m = float(target) if target is not None else math.nan
        value.global_clearance_m = float(global_value)
        value.cross_track_error_m = float(cross)
        value.heading_error_rad = float(heading)
        handle.publish_feedback(value)

    @staticmethod
    def _result(result, reached, stopped, clearance, traveled, code, message):
        result.reached = bool(reached)
        result.stopped = bool(stopped)
        result.final_clearance_m = float(clearance) if clearance is not None else math.nan
        result.traveled_distance_m = float(traveled)
        result.error_code = str(code)
        result.message = str(message)
        return result

    def _abort(self, handle, result, request_id, clearance, traveled, code, message):
        self._publish(request_id, 0.0, 0.0)
        handle.abort()
        self.get_logger().error(
            "[APPROACH_RESULT] "
            f"request={request_id} success=False reached=False stopped=False "
            f"clearance={self._format_distance(clearance)} traveled={float(traveled):.3f} "
            f"error={code} message={message}"
        )
        return self._result(result, False, False, clearance, traveled, code, message)


def main(args=None):
    rclpy.init(args=args)
    node = ApproachController()
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
