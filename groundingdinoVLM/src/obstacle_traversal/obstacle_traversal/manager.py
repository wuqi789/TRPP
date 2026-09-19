#!/usr/bin/env python3
"""Coordinate synchronized sensing, parallel real inference, and traversal authorization."""

from __future__ import annotations

import copy
import hashlib
import math
import threading
import time
import uuid

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan, RegionOfInterest
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from groundingdino_vlm_interfaces.action import VerifyTarget
from obstacle_traversal_interfaces.action import (
    ApproachObstacle, AssessPushability, ProbePushability,
)
from obstacle_traversal_interfaces.msg import (
    FilterAuthorization, FilterState, MechanicalAssessment, TraversalStatus,
    VehicleState,
)
from obstacle_traversal_interfaces.srv import ResetTraversal
from semantic_navigation_interfaces.msg import NavigationTaskStatus

from .core import (
    closest_path_obstacle, expand_roi, outside_footprint_mask, project_roi,
    scan_cluster_indices, synchronized, transform_matrix,
    fusion_probability, three_modal_authorized, traversal_sensor_inputs_ready,
    path_tangent_heading_error, rejected_cache_active,
)


VOCABULARY = "box . curtain"


class TraversalManager(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_traversal_manager")
        defaults = {
            "path_topic": "/neupan_initial_path",
            "scan_topic": "/scan_raw",
            "image_topic": "/isaac/color_image_raw",
            "camera_info_topic": "/isaac/color/camera_info",
            "navigation_status_topic": "/semantic_navigation/status",
            "status_topic": "/obstacle_traversal/status",
            "ready_topic": "/obstacle_traversal/ready",
            "debug_image_topic": "/obstacle_traversal/debug_image",
            "filter_authorization_topic": "/obstacle_traversal/filter_authorization",
            "filter_state_topic": "/obstacle_traversal/filter_state",
            "dino_action": "/groundingdino_vlm/verify_target",
            "pushability_action": "/llmdecision/assess_pushability",
            "approach_action": "/obstacle_traversal/approach_obstacle",
            "probe_action": "/piper/probe_pushability",
            "odom_topic": "/odom",
            "reset_service": "/obstacle_traversal/reset_latched",
            "map_frame": "map",
            "robot_frame": "base_link",
            "camera_frame": "d435_color_optical_frame",
            "vocabulary": VOCABULARY,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        numeric = {
            "trigger_distance": 1.50, "corridor_half_width": 0.433,
            "sync_tolerance": 0.20, "decision_threshold": 0.70,
            "deadline": 125.0, "filter_duration": 30.0,
            "pass_distance": 0.80, "cache_motion": 1.0,
            "cache_match_distance": 1.0,
            "input_freshness": 0.50,
            "vehicle_mass_kg": 50.0, "vehicle_payload_kg": 0.0,
            "maximum_push_force_n": 150.0, "footprint_width_m": 0.586,
            "footprint_length_m": 0.62,
            "self_filter_padding": 0.05,
            "roi_minimum_width_px": 320.0,
            "roi_minimum_height_px": 320.0,
            "target_clearance_m": 0.20,
            "clearance_tolerance_m": 0.02,
            "recovery_distance_m": 0.30,
            "fusion_threshold": 0.75,
            "minimum_dino_roi_overlap": 0.20,
            "reset_odom_freshness_s": 0.50,
            "reset_stable_linear_mps": 0.01,
            "reset_stable_angular_rps": 0.02,
            "pretrigger_guard_release_s": 0.30,
            "pretrigger_path_heading_tolerance_rad": 0.35,
        }
        for name, value in numeric.items():
            self.declare_parameter(name, value)
        transient = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            TraversalStatus, str(self.get_parameter("status_topic").value), transient
        )
        self._ready_pub = self.create_publisher(
            Bool, str(self.get_parameter("ready_topic").value), transient
        )
        self._debug_pub = self.create_publisher(
            CompressedImage, str(self.get_parameter("debug_image_topic").value),
            qos_profile_sensor_data,
        )
        self._filter_pub = self.create_publisher(
            FilterAuthorization,
            str(self.get_parameter("filter_authorization_topic").value), transient,
        )
        self.create_subscription(
            Path, str(self.get_parameter("path_topic").value), self._on_path, transient
        )
        self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value), self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value),
            self._on_camera, qos_profile_sensor_data,
        )
        self.create_subscription(
            NavigationTaskStatus,
            str(self.get_parameter("navigation_status_topic").value),
            self._on_navigation_status, transient,
        )
        self.create_subscription(
            FilterState, str(self.get_parameter("filter_state_topic").value),
            self._on_filter_state, transient,
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 10
        )
        callbacks = ReentrantCallbackGroup()
        self._dino = ActionClient(
            self, VerifyTarget, str(self.get_parameter("dino_action").value),
            callback_group=callbacks,
        )
        self._pushability = ActionClient(
            self, AssessPushability,
            str(self.get_parameter("pushability_action").value),
            callback_group=callbacks,
        )
        self._approach = ActionClient(
            self, ApproachObstacle, str(self.get_parameter("approach_action").value),
            callback_group=callbacks,
        )
        self._probe = ActionClient(
            self, ProbePushability, str(self.get_parameter("probe_action").value),
            callback_group=callbacks,
        )
        self._tf = Buffer(cache_time=Duration(seconds=10.0))
        self._listener = TransformListener(self._tf, self)
        self._bridge = CvBridge()
        self._lock = threading.RLock()
        self._tick_lock = threading.Lock()
        self._path: Path | None = None
        self._path_signature = ""
        self._scan: LaserScan | None = None
        self._image: Image | None = None
        self._camera: CameraInfo | None = None
        self._odom: Odometry | None = None
        self._odom_at = 0.0
        self._filter_state: FilterState | None = None
        self._received = {}
        self._task_id = ""
        self._navigation_request_id = ""
        self._navigation_active = False
        self._active: dict | None = None
        self._pretrigger_guard: dict | None = None
        self._cache: dict[
            str, tuple[str, tuple[float, float], tuple[float, float], bool, float]
        ] = {}
        self._last_status = TraversalStatus()
        self._last_status.state = TraversalStatus.IDLE
        self._last_status.message = "Waiting for an active Module4 navigation task"
        self._last_idle_reason = ""
        self._last_idle_log = 0.0
        self._reset_service = self.create_service(
            ResetTraversal,
            str(self.get_parameter("reset_service").value),
            self._reset_latched,
            callback_group=callbacks,
        )
        self.create_timer(0.05, self._tick, callback_group=callbacks)
        self.create_timer(0.2, self._heartbeat, callback_group=callbacks)

    def _on_path(self, message: Path) -> None:
        signature = self._signature(message)
        with self._lock:
            self._path = copy.deepcopy(message)
            self._path_signature = signature
            self._received["path"] = time.monotonic()

    def _on_scan(self, message: LaserScan) -> None:
        with self._lock:
            self._scan = copy.deepcopy(message)
            self._received["scan"] = time.monotonic()

    def _on_image(self, message: Image) -> None:
        with self._lock:
            self._image = copy.deepcopy(message)
            self._received["image"] = time.monotonic()

    def _on_camera(self, message: CameraInfo) -> None:
        with self._lock:
            self._camera = copy.deepcopy(message)
            self._received["camera"] = time.monotonic()

    def _on_navigation_status(self, message: NavigationTaskStatus) -> None:
        with self._lock:
            task_changed = bool(self._task_id and message.task_id != self._task_id)
            self._task_id = str(message.task_id)
            self._navigation_request_id = str(message.request_id)
            self._navigation_active = bool(self._task_id and not message.terminal)
            if task_changed:
                self._cache.clear()
                if self._active is not None:
                    self._cancel_active("TASK_CHANGED", "Module4 task changed")
            if message.terminal and self._active is not None:
                self._cancel_active("TASK_TERMINAL", "Module4 task became terminal")

    def _on_filter_state(self, message: FilterState) -> None:
        with self._lock:
            self._filter_state = copy.deepcopy(message)
            active = self._active
            if active is None or message.request_id != active["request_id"]:
                return
            active["removed_points"] = int(message.removed_points)
            if message.active and active["state"] == TraversalStatus.AUTHORIZED:
                active["state"] = TraversalStatus.TRAVERSING
                self._publish_active("Authorized scan cluster active; speed limited")
            elif not message.active and active["state"] in (
                TraversalStatus.AUTHORIZED, TraversalStatus.TRAVERSING
            ):
                expected = message.error_code in {"FILTER_EXPIRED", "FILTER_PASS_COMPLETE", ""}
                self._cache[active["obstacle_id"]] = (
                    "authorized" if expected else "error",
                    active["robot_at_trigger"], active["track_reference"],
                    bool(active["dino_verified"]),
                    float(active["pushability_probability"]),
                )
                terminal = dict(active)
                self._active = None
                if expected:
                    self._publish_terminal(
                        TraversalStatus.IDLE, "",
                        "Traversal filter withdrawn; NeuPAN full scan restored",
                        dino_ok=bool(terminal["dino_verified"]),
                        probability=float(terminal["pushability_probability"]),
                        context=terminal,
                    )
                else:
                    self._publish_terminal(
                        TraversalStatus.ERROR, message.error_code,
                        "Scan filtering failed; vehicle stopped and full scan restored",
                        dino_ok=bool(terminal["dino_verified"]),
                        probability=float(terminal["pushability_probability"]),
                        context=terminal,
                    )

    def _on_odom(self, message: Odometry) -> None:
        with self._lock:
            self._odom = copy.deepcopy(message)
            self._odom_at = time.monotonic()

    def _reset_latched(self, request, response):
        with self._lock:
            active = self._active
            if active is None or not bool(active.get("latched", False)):
                return self._reset_response(
                    response, False, "RESET_NOT_LATCHED", "No latched traversal request exists"
                )
            if str(request.request_id) != str(active["request_id"]):
                return self._reset_response(
                    response, False, "RESET_REQUEST_ID_MISMATCH",
                    "Reset request_id does not match the latched traversal",
                )
            if self._filter_state is None or bool(self._filter_state.active):
                return self._reset_response(
                    response, False, "RESET_FILTER_NOT_CONFIRMED_OFF",
                    "Scan filtering must be confirmed inactive before release",
                )
            if bool(active.get("arm_deployed", False)) and not bool(
                active.get("arm_returned", False)
            ):
                return self._reset_response(
                    response, False, "RESET_ARM_NOT_RETURNED",
                    "Piper return is unconfirmed; base remains stopped",
                )
            if (
                self._odom is None
                or time.monotonic() - self._odom_at
                > float(self.get_parameter("reset_odom_freshness_s").value)
            ):
                return self._reset_response(
                    response, False, "RESET_ODOM_STALE",
                    "Fresh stationary odometry is required before release",
                )
            twist = self._odom.twist.twist
            if (
                abs(float(twist.linear.x))
                > float(self.get_parameter("reset_stable_linear_mps").value)
                or abs(float(twist.angular.z))
                > float(self.get_parameter("reset_stable_angular_rps").value)
            ):
                return self._reset_response(
                    response, False, "RESET_BASE_MOVING",
                    "Base must be stationary before releasing the traversal latch",
                )
            terminal = dict(active)
            for handle in active["goal_handles"]:
                handle.cancel_goal_async()
            self._cache[active["obstacle_id"]] = (
                "error", active["robot_at_trigger"], active["track_reference"], False, 0.0,
            )
            self._active = None
        reason = str(request.reason).strip() or "operator reset"
        self._publish_terminal(
            TraversalStatus.ERROR,
            "MANUAL_RESET",
            f"Latched traversal released after safe checks: {reason}",
            context=terminal,
        )
        self.get_logger().warning(
            f"[TRAVERSAL_RESET] request={request.request_id} released=True reason={reason}"
        )
        return self._reset_response(
            response, True, "", "Traversal latch released; NeuPAN full scan remains active"
        )

    @staticmethod
    def _reset_response(response, released, code, message):
        response.released = bool(released)
        response.error_code = str(code)
        response.message = str(message)
        return response

    def _tick(self) -> None:
        if not self._tick_lock.acquire(blocking=False):
            return
        try:
            self._tick_once()
        finally:
            self._tick_lock.release()

    def _tick_once(self) -> None:
        with self._lock:
            active = self._active
        if active is not None:
            if bool(active.get("latched", False)):
                return
            if (
                time.monotonic() - active["started"]
                > float(self.get_parameter("deadline").value)
                and active["state"] not in (
                    TraversalStatus.AUTHORIZED, TraversalStatus.TRAVERSING,
                    TraversalStatus.RECOVERING,
                )
            ):
                self._provider_deadline(active["request_id"])
            return
        if not self._navigation_active:
            with self._lock:
                guard_active = self._pretrigger_guard is not None
            if guard_active:
                self._release_pretrigger_guard(
                    "Navigation task ended before the synchronized trigger"
                )
        else:
            candidate, guard_reason, guard_detail = (
                self._path_scan_guard_candidate()
            )
            if candidate is not None:
                self._observe_pretrigger_guard(candidate)
            else:
                with self._lock:
                    guard = copy.deepcopy(self._pretrigger_guard)
                if guard is not None:
                    missing_for = time.monotonic() - float(guard["last_seen"])
                    release_after = float(
                        self.get_parameter("pretrigger_guard_release_s").value
                    )
                    if guard_reason == "OBSTACLE_DECISION_CACHED":
                        self._release_pretrigger_guard(
                            "Fail-closed obstacle decision remains cached; "
                            "NeuPAN released"
                        )
                    elif (
                        guard_reason == "NO_PATH_OBSTACLE"
                        and missing_for > release_after
                    ):
                        self._release_pretrigger_guard(
                            "Path obstacle cleared before the synchronized trigger"
                        )
                    else:
                        self._publish_pretrigger_guard(
                            f"[{guard_reason}] {guard_detail}; holding "
                            "NeuPAN at zero"
                        )
                        return
        snapshot, reason, detail = self._snapshot_with_reason()
        if snapshot is None:
            self._set_idle_reason(reason, detail)
            return
        if not self._navigation_active:
            self._set_idle_reason(
                "NAVIGATION_TASK_MISSING",
                "Sensors are synchronized; waiting for an active Module4 task",
            )
            return
        path, scan, image, camera = snapshot
        with self._lock:
            guard = copy.deepcopy(self._pretrigger_guard)
        if guard is not None:
            path = copy.deepcopy(guard["locked_path"])
        try:
            robot = self._robot_xy()
            map_points, scan_points, valid_indices = self._scan_points_map(scan)
        except TransformException:
            self._set_idle_reason(
                "SCAN_TF_UNAVAILABLE",
                "Cannot transform current LiDAR scan into map/base frames",
            )
            return
        if not len(map_points):
            self._set_idle_reason(
                "NO_EXTERNAL_SCAN_POINTS",
                "All current LiDAR returns are invalid or inside the robot footprint",
            )
            return
        path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
        obstacle = closest_path_obstacle(
            [(point[0], point[1]) for point in map_points], path_points, robot,
            trigger_distance=float(self.get_parameter("trigger_distance").value),
            half_width=float(self.get_parameter("corridor_half_width").value),
        )
        if obstacle is None:
            self._set_idle_reason(
                "NO_PATH_OBSTACLE",
                "No non-self LiDAR point is inside the 1.50 m forward path corridor",
            )
            return
        source_scan_index = valid_indices[obstacle.source_index]
        center_angle = float(scan.angle_min + source_scan_index * scan.angle_increment)
        center_range = float(scan.ranges[source_scan_index])
        cluster_indices = scan_cluster_indices(
            scan.ranges, scan.angle_min, scan.angle_increment, center_angle, center_range,
            angular_expansion=0.0, edge_points=0,
        )
        if not cluster_indices:
            self._set_idle_reason(
                "SCAN_CLUSTER_EMPTY",
                "Path obstacle did not form a bounded continuous LiDAR cluster",
            )
            return
        cluster_points = np.asarray([
            [float(scan.ranges[index]) * math.cos(scan.angle_min + index * scan.angle_increment),
             float(scan.ranges[index]) * math.sin(scan.angle_min + index * scan.angle_increment), 0.0]
            for index in cluster_indices if math.isfinite(float(scan.ranges[index]))
        ])
        mapped_by_index = {
            raw_index: map_points[position]
            for position, raw_index in enumerate(valid_indices)
        }
        mapped_cluster = np.asarray([
            mapped_by_index[index]
            for index in cluster_indices
            if index in mapped_by_index
        ])
        if not len(mapped_cluster):
            self._set_idle_reason(
                "SCAN_CLUSTER_SELF_ONLY",
                "Candidate LiDAR cluster contains only robot-footprint returns",
            )
            return
        observed_center = (
            float(np.median(mapped_cluster[:, 0])),
            float(np.median(mapped_cluster[:, 1])),
        )
        if guard is not None and math.dist(
            observed_center, guard["center"]
        ) > float(self.get_parameter("cache_match_distance").value):
            self._set_idle_reason(
                "GUARD_TARGET_MISMATCH",
                "Synchronized target does not match the locked trigger reference",
            )
            return
        obstacle_center = (
            tuple(guard["center"]) if guard is not None else observed_center
        )
        obstacle_id = (
            str(guard["obstacle_id"])
            if guard is not None else self._obstacle_id(obstacle_center)
        )
        cached_key, cached = self._nearest_cached(obstacle_center)
        if (
            cached is not None
            and cached[0] != "authorized"
            and rejected_cache_active(
                robot, cached[1], cached[2],
                motion_limit=float(self.get_parameter("cache_motion").value),
                obstacle_neighborhood=float(
                    self.get_parameter("cache_match_distance").value
                ),
            )
        ):
            if guard is not None:
                self._release_pretrigger_guard(
                    "Fail-closed obstacle decision remains cached; NeuPAN released"
                )
            else:
                self._set_idle_reason(
                    "OBSTACLE_DECISION_CACHED",
                    "Tracked obstacle already has a fail-closed decision while "
                    "NeuPAN is bypassing its neighborhood",
                )
            return
        if cached_key is not None:
            self._cache.pop(cached_key, None)
        try:
            projected = self._project_cluster(scan, camera, image, cluster_points)
        except TransformException:
            self._set_idle_reason(
                "CAMERA_TF_UNAVAILABLE",
                "Cannot transform LiDAR cluster into the RGB camera frame",
            )
            return
        if projected is None:
            self._set_idle_reason(
                "ROI_OUTSIDE_IMAGE",
                "Path obstacle LiDAR cluster does not project into the current RGB image",
            )
            return
        roi, target_roi = projected
        compressed = self._compress(image)
        if compressed is None:
            self._set_idle_reason(
                "IMAGE_ENCODING_FAILED",
                "Synchronized RGB frame could not be encoded for inference",
            )
            return
        self._start_request(
            compressed, roi, target_roi, obstacle_id, obstacle_center,
            obstacle_center,
            tuple(guard["robot_at_trigger"]) if guard is not None else robot,
            center_angle, center_range,
            float(guard["distance"]) if guard is not None else obstacle.along_path,
            path,
            str(guard["path_signature"]) if guard is not None else self._signature(path),
        )

    def _path_scan_guard_candidate(self):
        """Detect the unchanged corridor trigger before RGB synchronization."""
        with self._lock:
            path = copy.deepcopy(self._path)
            scan = copy.deepcopy(self._scan)
            scan_received = float(self._received.get("scan", 0.0))
        missing = []
        if path is None:
            missing.append("path")
        if scan is None:
            missing.append("scan")
        if missing:
            return (
                None,
                "GUARD_INPUT_MISSING",
                "Waiting for: " + ", ".join(missing),
            )
        freshness = float(self.get_parameter("input_freshness").value)
        if time.monotonic() - scan_received > freshness:
            return None, "GUARD_SCAN_STALE", "Latest raw scan is stale"
        try:
            robot_pose = self._robot_pose()
            robot = robot_pose[:2]
            map_points, _, _ = self._scan_points_map(scan)
        except TransformException:
            return (
                None,
                "GUARD_TF_UNAVAILABLE",
                "Cannot transform the raw scan",
            )
        if not len(map_points):
            return (
                None,
                "GUARD_NO_EXTERNAL_POINTS",
                "Raw scan has no valid non-self returns",
            )
        path_points = [
            (pose.pose.position.x, pose.pose.position.y) for pose in path.poses
        ]
        obstacle = closest_path_obstacle(
            [(point[0], point[1]) for point in map_points], path_points, robot,
            trigger_distance=float(
                self.get_parameter("trigger_distance").value
            ),
            half_width=float(self.get_parameter("corridor_half_width").value),
        )
        if obstacle is None:
            return (
                None,
                "NO_PATH_OBSTACLE",
                "No non-self LiDAR point is inside the 1.50 m forward "
                "path corridor",
            )
        heading_error = path_tangent_heading_error(robot_pose, path_points)
        if heading_error is None:
            return (
                None,
                "GUARD_PATH_DEGENERATE",
                "NeuPAN path has no non-degenerate segment",
            )
        heading_tolerance = float(
            self.get_parameter(
                "pretrigger_path_heading_tolerance_rad"
            ).value
        )
        if abs(heading_error) > heading_tolerance:
            return (
                None,
                "PATH_ALIGNMENT_REQUIRED",
                f"Robot is aligning to the current NeuPAN path: "
                f"heading_error={heading_error:.3f} rad exceeds "
                f"{heading_tolerance:.3f} rad",
            )
        center = (float(obstacle.point[0]), float(obstacle.point[1]))
        cached_key, cached = self._nearest_cached(center)
        if (
            cached is not None
            and cached[0] != "authorized"
            and rejected_cache_active(
                robot, cached[1], cached[2],
                motion_limit=float(self.get_parameter("cache_motion").value),
                obstacle_neighborhood=float(
                    self.get_parameter("cache_match_distance").value
                ),
            )
        ):
            return (
                None,
                "OBSTACLE_DECISION_CACHED",
                "Fail-closed decision is cached while NeuPAN bypasses the "
                "rejected obstacle neighborhood",
            )
        return ({
            "center": center,
            "obstacle_id": self._obstacle_id(center),
            "distance": float(obstacle.along_path),
            "robot_at_trigger": robot,
            "path_heading_error": float(heading_error),
            "locked_path": copy.deepcopy(path),
            "path_signature": self._signature(path),
        }, "", "")

    def _observe_pretrigger_guard(self, candidate) -> None:
        now = time.monotonic()
        created = False
        with self._lock:
            guard = self._pretrigger_guard
            same_obstacle = bool(
                guard is not None
                and math.dist(guard["center"], candidate["center"])
                <= float(self.get_parameter("cache_match_distance").value)
            )
            if not same_obstacle:
                guard = {
                    "request_id": "guard-" + str(uuid.uuid4()),
                    "task_id": self._task_id,
                    "obstacle_id": candidate["obstacle_id"],
                    "center": candidate["center"],
                    "distance": candidate["distance"],
                    "robot_at_trigger": candidate["robot_at_trigger"],
                    "locked_path": copy.deepcopy(candidate["locked_path"]),
                    "path_signature": str(candidate["path_signature"]),
                    "started": now,
                    "last_seen": now,
                    "message": "",
                    "last_log": 0.0,
                }
                self._pretrigger_guard = guard
                created = True
            else:
                guard["last_seen"] = now
        if created:
            self.get_logger().warning(
                "[PRETRIGGER_GUARD] "
                f"request={guard['request_id']} active=True "
                f"obstacle={guard['obstacle_id']} "
                f"along={guard['distance']:.3f} "
                f"heading_error={candidate['path_heading_error']:.3f} "
                "reason=PATH_SCAN_CANDIDATE"
            )
        self._publish_pretrigger_guard(
            "Path/scan candidate detected; waiting for the unchanged "
            "synchronized trigger"
        )

    def _publish_pretrigger_guard(self, message: str) -> None:
        now = time.monotonic()
        with self._lock:
            guard = self._pretrigger_guard
            if guard is None or self._active is not None:
                return
            output = TraversalStatus()
            output.header.stamp = self.get_clock().now().to_msg()
            output.header.frame_id = str(self.get_parameter("map_frame").value)
            output.task_id = str(guard["task_id"])
            output.request_id = str(guard["request_id"])
            output.obstacle_id = str(guard["obstacle_id"])
            output.state = TraversalStatus.SNAPSHOT_PENDING
            output.navigation_paused = True
            output.filter_authorized = False
            output.obstacle_center_map.x = float(guard["center"][0])
            output.obstacle_center_map.y = float(guard["center"][1])
            output.obstacle_distance_m = float(guard["distance"])
            output.elapsed_ms = (now - float(guard["started"])) * 1000.0
            output.message = str(message)
            changed = output.message != guard["message"]
            should_log = changed and now - float(guard["last_log"]) >= 1.0
            guard["message"] = output.message
            if should_log:
                guard["last_log"] = now
            self._last_status = output
        self._status_pub.publish(output)
        if should_log:
            self.get_logger().info(
                f"[PRETRIGGER_GUARD] request={output.request_id} "
                f"detail={output.message}"
            )

    def _release_pretrigger_guard(self, message: str) -> None:
        with self._lock:
            guard = self._pretrigger_guard
            if guard is None:
                return
            self._pretrigger_guard = None
        self.get_logger().info(
            "[PRETRIGGER_GUARD] "
            f"request={guard['request_id']} active=False reason={message}"
        )
        self._publish_terminal(
            TraversalStatus.IDLE, "", message, context=guard
        )

    def _snapshot_with_reason(self):
        with self._lock:
            values = (copy.deepcopy(self._path), copy.deepcopy(self._scan),
                      copy.deepcopy(self._image), copy.deepcopy(self._camera))
            received = dict(self._received)
        names = ("path", "scan", "image", "camera")
        missing = [name for name, value in zip(names, values) if value is None]
        if missing:
            return None, "SENSOR_INPUT_MISSING", "Waiting for: " + ", ".join(missing)
        freshness = float(self.get_parameter("input_freshness").value)
        now = time.monotonic()
        stale = [
            name for name in ("scan", "image", "camera")
            if now - received.get(name, 0.0) > freshness
        ]
        if stale:
            return None, "SENSOR_INPUT_STALE", "Stale input: " + ", ".join(stale)
        path, scan, image, camera = values
        stamps = [
            self._stamp(scan.header), self._stamp(image.header), self._stamp(camera.header)
        ]
        tolerance = float(self.get_parameter("sync_tolerance").value)
        if not synchronized(stamps, tolerance):
            span = max(stamps) - min(stamps)
            return (
                None,
                "SENSOR_STAMPS_UNSYNCED",
                f"RGB/CameraInfo/LiDAR timestamp span {span:.3f}s exceeds {tolerance:.3f}s",
            )
        return values, "", ""

    def _snapshot(self):
        snapshot, _, _ = self._snapshot_with_reason()
        return snapshot

    def _scan_points_map(self, scan: LaserScan):
        source = scan.header.frame_id.lstrip("/")
        transform = self._tf.lookup_transform(
            str(self.get_parameter("map_frame").value), source,
            rclpy.time.Time(), timeout=Duration(seconds=0.1),
        )
        matrix = transform_matrix(transform.transform.translation, transform.transform.rotation)
        scan_points, valid_indices = [], []
        for index, distance in enumerate(scan.ranges):
            distance = float(distance)
            if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            scan_points.append((distance * math.cos(angle), distance * math.sin(angle), 0.0))
            valid_indices.append(index)
        points = np.asarray(scan_points, dtype=float)
        if not len(points):
            return np.empty((0, 3), dtype=float), np.empty((0, 3), dtype=float), []
        homogeneous = np.column_stack((points, np.ones(len(points))))
        mapped = (matrix @ homogeneous.T).T[:, :3]
        robot_transform = self._tf.lookup_transform(
            str(self.get_parameter("robot_frame").value), source,
            rclpy.time.Time(), timeout=Duration(seconds=0.1),
        )
        robot_matrix = transform_matrix(
            robot_transform.transform.translation, robot_transform.transform.rotation
        )
        robot_points = (robot_matrix @ homogeneous.T).T[:, :3]
        keep = outside_footprint_mask(
            robot_points,
            float(self.get_parameter("footprint_length_m").value),
            float(self.get_parameter("footprint_width_m").value),
            padding=float(self.get_parameter("self_filter_padding").value),
        )
        return mapped[keep], points[keep], [
            index for index, retain in zip(valid_indices, keep) if retain
        ]

    def _project_cluster(self, scan, camera, image, points):
        camera_frame = camera.header.frame_id.lstrip("/") or str(
            self.get_parameter("camera_frame").value
        )
        transform = self._tf.lookup_transform(
            camera_frame, scan.header.frame_id.lstrip("/"),
            rclpy.time.Time(), timeout=Duration(seconds=0.1),
        )
        target_roi = project_roi(
            points, transform_matrix(transform.transform.translation, transform.transform.rotation),
            np.asarray(camera.k), int(image.width), int(image.height), padding=0,
        )
        if target_roi is None:
            return None
        roi = project_roi(
            points, transform_matrix(transform.transform.translation, transform.transform.rotation),
            np.asarray(camera.k), int(image.width), int(image.height), padding=24,
        )
        if roi is None:
            return None
        x, y, width, height = expand_roi(
            roi, int(image.width), int(image.height),
            minimum_width=int(self.get_parameter("roi_minimum_width_px").value),
            minimum_height=int(self.get_parameter("roi_minimum_height_px").value),
        )
        expanded = RegionOfInterest()
        expanded.x_offset, expanded.y_offset = int(x), int(y)
        expanded.width, expanded.height = int(width), int(height)
        expanded.do_rectify = False
        target = RegionOfInterest()
        target.x_offset, target.y_offset = map(int, target_roi[:2])
        target.width, target.height = map(int, target_roi[2:])
        target.do_rectify = False
        return expanded, target

    def _compress(self, message: Image):
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            success, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not success:
                return None
            output = CompressedImage()
            output.header = copy.deepcopy(message.header)
            output.format = "jpeg"
            output.data = encoded.tobytes()
            return output
        except Exception:
            self.get_logger().error("Could not encode synchronized RGB snapshot")
            return None

    def _start_request(
        self, image, roi, target_roi, obstacle_id, center, track_reference,
        robot, angle, distance, along, locked_path, locked_path_signature,
    ):
        request_id = str(uuid.uuid4())
        now = time.monotonic()
        active = {
            "request_id": request_id, "task_id": self._task_id,
            "navigation_request_id": self._navigation_request_id,
            "obstacle_id": obstacle_id, "center": center,
            "track_reference": track_reference,
            "robot_at_trigger": robot, "center_angle": angle, "center_range": distance,
            "distance": along, "roi": copy.deepcopy(roi),
            "target_roi": copy.deepcopy(target_roi), "started": now,
            "image_header": copy.deepcopy(image.header),
            "state": TraversalStatus.APPROACHING,
            "dino": None, "pushability": None, "mechanical": None,
            "dino_error": "", "pushability_error": "",
            "dino_verified": False, "pushability_probability": 0.0,
            "dino_probability": 0.0, "mechanical_probability": 0.0,
            "fusion_probability": 0.0, "approach_clearance": math.nan,
            "arm_deployed": False,
            "arm_returned": True,
            "mechanical_action_succeeded": False,
            "dino_offset_ms": 0.0, "pushability_offset_ms": 0.0,
            "goal_handles": [], "removed_points": 0,
            "locked_path": copy.deepcopy(locked_path),
            "locked_path_signature": str(locked_path_signature),
            "approach_traveled": 0.0,
        }
        with self._lock:
            if self._active is not None:
                return
            guard = self._pretrigger_guard
            self._pretrigger_guard = None
            self._active = active
        if guard is not None:
            self.get_logger().info(
                "[PRETRIGGER_GUARD] "
                f"request={guard['request_id']} promoted=True "
                f"wait_ms={(time.monotonic() - guard['started']) * 1000.0:.1f}"
            )
        self._publish_active(
            "Trigger accepted; continuing on the locked path while both inference actions run"
        )
        self._publish_debug(image, roi, target_roi)
        active["dino_offset_ms"] = (time.monotonic() - now) * 1000.0
        if self._dino.server_is_ready():
            dino_goal = VerifyTarget.Goal()
            dino_goal.request_id, dino_goal.image, dino_goal.roi = request_id, image, roi
            dino_goal.target_roi = target_roi
            dino_goal.vocabulary = str(self.get_parameter("vocabulary").value)
            dino_future = self._dino.send_goal_async(dino_goal)
            dino_future.add_done_callback(
                lambda future, rid=request_id: self._goal_response("dino", rid, future)
            )
        else:
            self._provider_result(
                "dino", request_id, None, "ACTION_SERVER_NOT_READY"
            )
        active["pushability_offset_ms"] = (time.monotonic() - now) * 1000.0
        if self._pushability.server_is_ready():
            push_goal = AssessPushability.Goal()
            push_goal.request_id, push_goal.image, push_goal.roi = request_id, image, roi
            push_goal.target_roi = target_roi
            push_goal.vehicle_state = self._vehicle_state()
            push_future = self._pushability.send_goal_async(push_goal)
            push_future.add_done_callback(
                lambda future, rid=request_id: self._goal_response("pushability", rid, future)
            )
        else:
            self._provider_result(
                "pushability", request_id, None, "ACTION_SERVER_NOT_READY"
            )
        if not self._approach.server_is_ready():
            self._fail_request(
                request_id, "APPROACH_SERVER_NOT_READY",
                "Dedicated approach controller is unavailable; vehicle remains fail-closed",
            )
            return
        approach_goal = ApproachObstacle.Goal()
        approach_goal.request_id = request_id
        approach_goal.locked_path = copy.deepcopy(locked_path)
        approach_goal.obstacle_center_map.x = float(center[0])
        approach_goal.obstacle_center_map.y = float(center[1])
        approach_goal.target_clearance_m = float(
            self.get_parameter("target_clearance_m").value
        )
        approach_goal.recovery = False
        approach_goal.recovery_distance_m = 0.0
        future = self._approach.send_goal_async(
            approach_goal,
            feedback_callback=lambda feedback, rid=request_id: self._approach_feedback(
                rid, feedback.feedback
            ),
        )
        future.add_done_callback(
            lambda completed, rid=request_id: self._approach_goal_response(rid, completed)
        )

    def _publish_filter_authorization(self, active):
        authorization = FilterAuthorization()
        authorization.header.stamp = self.get_clock().now().to_msg()
        authorization.header.frame_id = str(self.get_parameter("map_frame").value)
        authorization.request_id = active["request_id"]
        authorization.authorized = True
        authorization.obstacle_center_scan.x = active["center_range"] * math.cos(
            active["center_angle"]
        )
        authorization.obstacle_center_scan.y = active["center_range"] * math.sin(
            active["center_angle"]
        )
        authorization.obstacle_center_map.x, authorization.obstacle_center_map.y = active["center"]
        authorization.center_angle_rad = active["center_angle"]
        authorization.center_range_m = active["center_range"]
        authorization.maximum_duration_s = min(
            30.0, float(self.get_parameter("filter_duration").value)
        )
        authorization.pass_distance_m = min(
            0.80, float(self.get_parameter("pass_distance").value)
        )
        authorization.locked_path = copy.deepcopy(active["locked_path"])
        self._filter_pub.publish(authorization)

    def _goal_response(self, kind, request_id, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self._provider_result(kind, request_id, None, "ACTION_REJECTED")
                return
            with self._lock:
                if self._active is None or self._active["request_id"] != request_id:
                    handle.cancel_goal_async()
                    return
                self._active["goal_handles"].append(handle)
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda completed, k=kind, rid=request_id: self._action_result(k, rid, completed)
            )
        except Exception:
            self._provider_result(kind, request_id, None, "ACTION_TRANSPORT_FAILED")

    def _action_result(self, kind, request_id, future):
        try:
            wrapped = future.result()
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                self._provider_result(kind, request_id, wrapped.result, "ACTION_FAILED")
            else:
                self._provider_result(kind, request_id, wrapped.result, "")
        except Exception:
            self._provider_result(kind, request_id, None, "ACTION_RESULT_FAILED")

    def _approach_feedback(self, request_id, feedback):
        with self._lock:
            active = self._active
            if active is None or active["request_id"] != request_id:
                return
            active["approach_clearance"] = float(feedback.target_clearance_m)
            active["state"] = (
                TraversalStatus.ALIGNING
                if feedback.phase == feedback.ALIGNING else TraversalStatus.APPROACHING
            )
        self._publish_active(
            f"Approach clearance={float(feedback.target_clearance_m):.3f} m"
        )

    def _approach_goal_response(self, request_id, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self._fail_or_recover_approach(
                    request_id, "APPROACH_ACTION_REJECTED",
                    "Approach controller rejected the request", 0.0,
                )
                return
            with self._lock:
                if self._active is None or self._active["request_id"] != request_id:
                    handle.cancel_goal_async()
                    return
                self._active["goal_handles"].append(handle)
            handle.get_result_async().add_done_callback(
                lambda completed, rid=request_id: self._approach_result(rid, completed)
            )
        except Exception:
            self._fail_or_recover_approach(
                request_id, "APPROACH_TRANSPORT_FAILED",
                "Approach action transport failed", self._request_displacement(request_id),
            )

    def _approach_result(self, request_id, future):
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception:
            self._fail_or_recover_approach(
                request_id, "APPROACH_RESULT_FAILED",
                "Approach result transport failed", self._request_displacement(request_id),
            )
            return
        traveled = max(0.0, float(result.traveled_distance_m))
        with self._lock:
            if self._active is not None and self._active["request_id"] == request_id:
                self._active["approach_traveled"] = traveled
        tolerance = float(self.get_parameter("clearance_tolerance_m").value)
        target = float(self.get_parameter("target_clearance_m").value)
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or not result.reached or not result.stopped
            or abs(float(result.final_clearance_m) - target) > tolerance + 1e-6
        ):
            self._fail_or_recover_approach(
                request_id, result.error_code or "APPROACH_NOT_REACHED",
                result.message or "Arm working clearance was not reached safely",
                traveled,
            )
            return
        with self._lock:
            active = self._active
            if active is None or active["request_id"] != request_id:
                return
            active["approach_clearance"] = float(result.final_clearance_m)
            active["state"] = TraversalStatus.ARM_READY
        self._publish_active("Stopped and aligned at the Piper working clearance")
        self._start_probe(request_id)

    def _request_displacement(self, request_id):
        with self._lock:
            active = self._active
            if active is None or active["request_id"] != request_id:
                return 0.0
            start = tuple(active["robot_at_trigger"])
        try:
            return math.dist(start, self._robot_xy())
        except TransformException:
            return 0.0

    def _fail_or_recover_approach(self, request_id, code, message, traveled):
        traveled = max(0.0, float(traveled))
        with self._lock:
            active = self._active
            if active is None or active["request_id"] != request_id:
                return
            active["approach_traveled"] = traveled
        if code == "APPROACH_CLEARANCE_UNSAFE":
            self._latch_stopped(
                request_id, TraversalStatus.APPROACHING, code,
                message + "; external obstacle requires a stationary safety latch",
            )
        elif traveled >= 0.01:
            self._start_recovery(request_id, code, message)
        else:
            self._fail_request(request_id, code, message + "; NeuPAN released")

    def _start_probe(self, request_id):
        if not self._probe.server_is_ready():
            self.get_logger().error(
                f"[PIPER_DISPATCH] request={request_id} server_ready=False "
                "outcome=not_sent"
            )
            self._start_recovery(
                request_id, "PROBE_SERVER_NOT_READY",
                "Piper probe server unavailable and was not deployed",
            )
            return
        with self._lock:
            active = self._active
            if active is None or active["request_id"] != request_id:
                return
            goal = ProbePushability.Goal()
            goal.request_id = request_id
            goal.obstacle_center_map.x = float(active["center"][0])
            goal.obstacle_center_map.y = float(active["center"][1])
            goal.target_roi = copy.deepcopy(active["target_roi"])
            goal.timeout_s = max(
                1.0, float(self.get_parameter("deadline").value)
                - (time.monotonic() - active["started"])
            )
            active["state"] = TraversalStatus.PROBING
        self.get_logger().info(
            "[PIPER_DISPATCH] "
            f"request={request_id} server_ready=True outcome=send_goal "
            f"timeout_s={float(goal.timeout_s):.3f} "
            f"target_map=({float(goal.obstacle_center_map.x):.3f},"
            f"{float(goal.obstacle_center_map.y):.3f}) "
            f"roi=({int(goal.target_roi.x_offset)},{int(goal.target_roi.y_offset)},"
            f"{int(goal.target_roi.width)},{int(goal.target_roi.height)})"
        )
        self._publish_active("Piper constrained mechanical probe started")
        future = self._probe.send_goal_async(
            goal,
            feedback_callback=lambda feedback, rid=request_id: self._probe_feedback(
                rid, feedback.feedback
            ),
        )
        future.add_done_callback(
            lambda completed, rid=request_id: self._probe_goal_response(rid, completed)
        )

    def _probe_feedback(self, request_id, feedback):
        with self._lock:
            active = self._active
            if active is None or active["request_id"] != request_id:
                return
            active["state"] = (
                TraversalStatus.ARM_RETURNING
                if feedback.phase == feedback.RETURNING else TraversalStatus.PROBING
            )
        self._publish_active(str(feedback.message))

    def _probe_goal_response(self, request_id, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.get_logger().error(
                    f"[PIPER_ACTION] request={request_id} accepted=False"
                )
                self._start_recovery(
                    request_id, "PROBE_ACTION_REJECTED",
                    "Piper probe rejected before deployment",
                )
                return
            self.get_logger().info(
                f"[PIPER_ACTION] request={request_id} accepted=True"
            )
            with self._lock:
                if self._active is None or self._active["request_id"] != request_id:
                    handle.cancel_goal_async()
                    return
                self._active["goal_handles"].append(handle)
                self._active["arm_deployed"] = True
                self._active["arm_returned"] = False
                cancel_after_return = bool(
                    self._active.get("cancel_after_return")
                )
            handle.get_result_async().add_done_callback(
                lambda completed, rid=request_id: self._probe_result(rid, completed)
            )
            if cancel_after_return:
                handle.cancel_goal_async()
        except Exception:
            self.get_logger().error(
                f"[PIPER_ACTION] request={request_id} accepted=unknown "
                "error=PROBE_TRANSPORT_FAILED"
            )
            with self._lock:
                if self._active is not None and self._active["request_id"] == request_id:
                    # Goal acceptance is unknown, so release remains forbidden.
                    self._active["arm_deployed"] = True
                    self._active["arm_returned"] = False
            self._latch_stopped(
                request_id, TraversalStatus.ARM_RETURNING,
                "PROBE_TRANSPORT_FAILED",
                "Piper probe transport failed; deployment and return are unconfirmed",
            )

    def _probe_result(self, request_id, future):
        try:
            wrapped = future.result()
            assessment = wrapped.result.assessment
        except Exception:
            self._latch_stopped(request_id, TraversalStatus.ARM_RETURNING,
                                "PROBE_RESULT_FAILED",
                                "Piper result unavailable; return is unconfirmed")
            return
        self.get_logger().info(
            "[MECHANICAL_RESULT] "
            f"request={request_id} action_status={int(wrapped.status)} "
            f"state={int(assessment.state)} "
            f"mechanical_probability={float(assessment.mechanical_probability):.3f} "
            f"contact_probability={float(assessment.contact_probability):.3f} "
            f"mobility_probability={float(assessment.mobility_probability):.3f} "
            f"displacement_m={float(assessment.object_displacement_m):.4f} "
            f"resistance_ratio={float(assessment.resistance_ratio):.3f} "
            f"tracking_error={float(assessment.tracking_error):.4f} "
            f"yolo_class={int(assessment.vision_classification)} "
            f"yolo_confidence={float(assessment.vision_confidence):.3f} "
            f"yolo_backend_ok={bool(assessment.vision_backend_ok)} "
            f"yolo_votes={int(assessment.vision_votes)}/"
            f"{int(assessment.vision_samples)} "
            f"arm_returned={bool(assessment.arm_returned)} "
            f"error={assessment.error_code or 'none'} "
            f"message={assessment.message or 'none'}"
        )
        if str(assessment.request_id) != request_id:
            self._latch_stopped(request_id, TraversalStatus.ARM_RETURNING,
                                "MECHANICAL_REQUEST_ID_MISMATCH",
                                "Piper result request_id mismatch; return is untrusted")
            return
        if not bool(assessment.arm_returned):
            self._latch_stopped(request_id, TraversalStatus.ARM_RETURNING,
                                assessment.error_code or "ARM_RETURN_FAILED",
                                assessment.message or "Piper return was not confirmed")
            return
        with self._lock:
            active = self._active
            cancellation = (
                active.get("cancel_after_return")
                if active is not None and active["request_id"] == request_id else None
            )
        if cancellation is not None:
            with self._lock:
                active = self._active
                if active is None or active["request_id"] != request_id:
                    return
                active["arm_returned"] = True
                terminal = dict(active)
                self._active = None
            self._publish_terminal(
                TraversalStatus.ERROR, cancellation[0],
                cancellation[1] + "; Piper safely returned before task release",
                context=terminal,
            )
            return
        with self._lock:
            active = self._active
            if active is None or active["request_id"] != request_id:
                return
            if active["state"] in (
                TraversalStatus.AUTHORIZED, TraversalStatus.TRAVERSING,
                TraversalStatus.RECOVERING,
            ):
                return
            active["mechanical"] = copy.deepcopy(assessment)
            active["mechanical_probability"] = float(
                assessment.mechanical_probability
                if wrapped.status == GoalStatus.STATUS_SUCCEEDED else 0.0
            )
            active["mechanical_action_succeeded"] = (
                wrapped.status == GoalStatus.STATUS_SUCCEEDED
            )
            active["arm_returned"] = True
            active["state"] = TraversalStatus.FUSING
        self._publish_active("Piper safely returned; waiting for fusion inputs")
        self._maybe_finish_fusion(request_id)

    def _provider_result(self, kind, request_id, result, error):
        with self._lock:
            active = self._active
            if active is None or active["request_id"] != request_id:
                return
            if active["state"] in (
                TraversalStatus.AUTHORIZED, TraversalStatus.TRAVERSING,
                TraversalStatus.RECOVERING,
            ) or bool(active.get("latched", False)):
                return
            if not error and result is not None:
                payload = result.verification if kind == "dino" else result.assessment
                error = self._result_contract_error(payload, active)
                if error:
                    result = None
            if kind == "dino":
                active["dino"] = result.verification if result is not None else error
                active["dino_error"] = error
            else:
                active["pushability"] = result.assessment if result is not None else error
                active["pushability_error"] = error
            arm_complete = active["mechanical"] is not None
            summary = (
                self._dino_summary(active["dino"])
                if kind == "dino" else self._pushability_summary(active["pushability"])
            )
        self.get_logger().info(
            f"[PROVIDER_RESULT] request={request_id} provider={kind} {summary}"
        )
        if arm_complete:
            self._maybe_finish_fusion(request_id)

    @staticmethod
    def _roi_tuple(roi):
        return (
            int(roi.x_offset), int(roi.y_offset), int(roi.width), int(roi.height)
        )

    def _result_contract_error(self, payload, active) -> str:
        if str(payload.request_id) != active["request_id"]:
            return "RESULT_REQUEST_ID_MISMATCH"
        expected_header = active["image_header"]
        if (
            int(payload.header.stamp.sec) != int(expected_header.stamp.sec)
            or int(payload.header.stamp.nanosec) != int(expected_header.stamp.nanosec)
        ):
            return "RESULT_IMAGE_STAMP_MISMATCH"
        if self._roi_tuple(payload.roi) != self._roi_tuple(active["roi"]):
            return "RESULT_ROI_MISMATCH"
        if self._roi_tuple(payload.target_roi) != self._roi_tuple(active["target_roi"]):
            return "RESULT_TARGET_ROI_MISMATCH"
        return ""

    def _provider_deadline(self, request_id):
        with self._lock:
            active = self._active
            if active is None or active["request_id"] != request_id:
                return
            if active["dino"] is None:
                active["dino"] = "PROVIDER_TIMEOUT"
                active["dino_error"] = "PROVIDER_TIMEOUT"
            if active["pushability"] is None:
                active["pushability"] = "PROVIDER_TIMEOUT"
                active["pushability_error"] = "PROVIDER_TIMEOUT"
            mechanical_complete = active["mechanical"] is not None
        if mechanical_complete:
            self._maybe_finish_fusion(request_id)
        else:
            with self._lock:
                current = self._active
                arm_may_be_deployed = bool(
                    current is not None
                    and current["request_id"] == request_id
                    and current.get("arm_deployed", False)
                    and not current.get("arm_returned", False)
                )
            if arm_may_be_deployed:
                self._latch_stopped(
                    request_id, TraversalStatus.ARM_RETURNING,
                    "MECHANICAL_DEADLINE_EXCEEDED",
                    "Mechanical deadline exceeded; arm return is unconfirmed",
                )
            else:
                self._fail_request(
                    request_id, "MECHANICAL_DEADLINE_EXCEEDED",
                    "Approach did not finish within the total deadline",
                )

    def _maybe_finish_fusion(self, request_id):
        with self._lock:
            active = self._active
            if active is None or active["request_id"] != request_id:
                return
            if active["state"] != TraversalStatus.FUSING:
                return
            if active["mechanical"] is None:
                return
            providers_complete = active["dino"] is not None and active["pushability"] is not None
            deadline_reached = time.monotonic() - active["started"] >= float(
                self.get_parameter("deadline").value
            )
            if not providers_complete and not deadline_reached:
                return
            dino, pushability, mechanical = (
                active["dino"], active["pushability"], active["mechanical"]
            )
            p_dino = self._dino_probability(dino, active["target_roi"])
            p_llm = (
                float(pushability.pushability_probability)
                if not isinstance(pushability, str)
                and pushability.state == pushability.ACCEPTED else 0.0
            )
            p_arm = float(active["mechanical_probability"])
            score = fusion_probability(p_dino, p_llm, p_arm)
            mechanical_positive = mechanical.state in (
                MechanicalAssessment.FLEXIBLE, MechanicalAssessment.MOVABLE
            )
            allowed = three_modal_authorized(
                p_dino, p_llm, p_arm,
                mechanical_positive=mechanical_positive,
                mechanical_action_succeeded=bool(
                    active["mechanical_action_succeeded"]
                ),
                arm_returned=bool(active["arm_returned"]),
                threshold=float(self.get_parameter("fusion_threshold").value),
            )
            fusion_threshold = float(self.get_parameter("fusion_threshold").value)
            mechanical_action_succeeded = bool(active["mechanical_action_succeeded"])
            arm_returned = bool(active["arm_returned"])
            dino_error = active.get("dino_error") or "none"
            llm_error = active.get("pushability_error") or "none"
            active["dino_verified"] = p_dino > 0.0
            active["dino_probability"] = p_dino
            active["pushability_probability"] = p_llm
            active["mechanical_probability"] = p_arm
            active["fusion_probability"] = score
            if allowed:
                active["state"] = TraversalStatus.AUTHORIZED
        self.get_logger().info(
            "[FUSION_RESULT] "
            f"request={request_id} p_dino={p_dino:.3f} p_llm={p_llm:.3f} "
            f"p_arm={p_arm:.3f} score={score:.3f} threshold={fusion_threshold:.3f} "
            f"mechanical_state={int(mechanical.state)} "
            f"mechanical_positive={mechanical_positive} "
            f"mechanical_action_succeeded={mechanical_action_succeeded} "
            f"arm_returned={arm_returned} allowed={allowed} "
            f"dino_error={dino_error} llm_error={llm_error}"
        )
        if allowed:
            self._publish_filter_authorization(active)
            self._publish_active(
                f"Three-modal fusion passed ({score:.3f}); scan filtering authorized"
            )
        else:
            self._start_recovery(request_id, "FUSION_REJECTED",
                                 f"Three-modal fusion rejected ({score:.3f})")

    def _start_recovery(self, request_id, code, message):
        with self._lock:
            active = self._active
            if active is None or active["request_id"] != request_id:
                return
            if bool(active.get("arm_deployed", False)) and not bool(
                active.get("arm_returned", False)
            ):
                unsafe_arm = True
            else:
                unsafe_arm = False
            active["state"] = TraversalStatus.RECOVERING
            active["rejection_code"] = code
            active["rejection_message"] = message
            goal = ApproachObstacle.Goal()
            goal.request_id = request_id
            goal.locked_path = copy.deepcopy(active["locked_path"])
            goal.obstacle_center_map.x = float(active["center"][0])
            goal.obstacle_center_map.y = float(active["center"][1])
            goal.target_clearance_m = float(self.get_parameter("target_clearance_m").value)
            goal.recovery = True
            goal.recovery_distance_m = float(
                self.get_parameter("recovery_distance_m").value
            )
        if unsafe_arm:
            self._latch_stopped(
                request_id, TraversalStatus.ARM_RETURNING,
                "RECOVERY_ARM_NOT_RETURNED",
                "Controlled recovery forbidden until Piper return is confirmed",
            )
            return
        if not self._approach.server_is_ready():
            self._latch_stopped(
                request_id, TraversalStatus.RECOVERING,
                "RECOVERY_SERVER_NOT_READY",
                "Approach controller unavailable; base remains stopped",
            )
            return
        self._publish_active(message + "; starting controlled 0.30 m recovery")
        future = self._approach.send_goal_async(
            goal,
            feedback_callback=lambda feedback, rid=request_id: self._recovery_feedback(
                rid, feedback.feedback
            ),
        )
        future.add_done_callback(
            lambda completed, rid=request_id: self._recovery_goal_response(rid, completed)
        )

    def _recovery_feedback(self, request_id, feedback):
        with self._lock:
            active = self._active
            if active is None or active["request_id"] != request_id:
                return
            active["state"] = TraversalStatus.RECOVERING
        self._publish_active("Controlled recovery in progress")

    def _recovery_goal_response(self, request_id, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self._latch_stopped(request_id, TraversalStatus.RECOVERING,
                                    "RECOVERY_ACTION_REJECTED",
                                    "Recovery was rejected; base remains stopped")
                return
            with self._lock:
                if self._active is None or self._active["request_id"] != request_id:
                    handle.cancel_goal_async()
                    return
                self._active["goal_handles"].append(handle)
            handle.get_result_async().add_done_callback(
                lambda completed, rid=request_id: self._recovery_result(rid, completed)
            )
        except Exception:
            self._latch_stopped(request_id, TraversalStatus.RECOVERING,
                                "RECOVERY_TRANSPORT_FAILED",
                                "Recovery transport failed; base remains stopped")

    def _recovery_result(self, request_id, future):
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception:
            self._latch_stopped(request_id, TraversalStatus.RECOVERING,
                                "RECOVERY_RESULT_FAILED",
                                "Recovery result unavailable; base remains stopped")
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not result.stopped:
            self._latch_stopped(
                request_id, TraversalStatus.RECOVERING,
                result.error_code or "RECOVERY_FAILED",
                result.message or "Rear space is insufficient; base remains stopped",
            )
            return
        required = float(self.get_parameter("recovery_distance_m").value)
        traveled = float(result.traveled_distance_m)
        if traveled < max(0.0, required - 0.03):
            self._latch_stopped(
                request_id, TraversalStatus.RECOVERING,
                "RECOVERY_DISTANCE_SHORT",
                f"Recovery stopped after {traveled:.3f} m; base remains stopped",
            )
            return
        try:
            recovery_anchor = self._robot_xy()
        except TransformException:
            self._latch_stopped(
                request_id, TraversalStatus.RECOVERING,
                "RECOVERY_POSE_UNAVAILABLE",
                "Recovery completed but its cache anchor could not be verified",
            )
            return
        with self._lock:
            active = self._active
            if active is None or active["request_id"] != request_id:
                return
            self._cache[active["obstacle_id"]] = (
                "rejected", recovery_anchor, active["track_reference"],
                bool(active["dino_verified"]), float(active["pushability_probability"]),
            )
            terminal = dict(active)
            terminal["recovery_traveled"] = traveled
            self._active = None
        self.get_logger().info(
            "[RECOVERY_CACHE] "
            f"request={request_id} anchor=({recovery_anchor[0]:.3f},"
            f"{recovery_anchor[1]:.3f}) motion_limit="
            f"{float(self.get_parameter('cache_motion').value):.3f}"
        )
        self._publish_terminal(
            TraversalStatus.REJECTED,
            str(terminal.get("rejection_code", "FUSION_REJECTED")),
            str(terminal.get("rejection_message", "Fusion rejected"))
            + "; recovery complete and NeuPAN full scan restored",
            dino_ok=bool(terminal["dino_verified"]),
            probability=float(terminal["pushability_probability"]), context=terminal,
        )

    def _latch_stopped(self, request_id, state, code, message):
        with self._lock:
            active = self._active
            if active is None or active["request_id"] != request_id:
                return
            active["state"] = state
            active["latched"] = True
            active["latched_error"] = code
        self._publish_active(f"[{code}] {message}")

    def _dino_probability(self, result, target_roi):
        if isinstance(result, str) or not bool(result.verified):
            return 0.0
        rx1, ry1 = int(target_roi.x_offset), int(target_roi.y_offset)
        rx2, ry2 = rx1 + int(target_roi.width), ry1 + int(target_roi.height)
        threshold = float(self.get_parameter("minimum_dino_roi_overlap").value)
        scores = []
        for candidate in result.candidates:
            x1, y1 = int(candidate.x_min), int(candidate.y_min)
            x2, y2 = int(candidate.x_max), int(candidate.y_max)
            intersection = max(0, min(x2, rx2) - max(x1, rx1)) * max(
                0, min(y2, ry2) - max(y1, ry1)
            )
            candidate_area = max(1, (x2 - x1) * (y2 - y1))
            roi_area = max(1, (rx2 - rx1) * (ry2 - ry1))
            overlap = float(intersection) / float(min(candidate_area, roi_area))
            if overlap >= threshold:
                scores.append(float(candidate.grounding_confidence))
        return max(scores, default=0.0)

    def _fail_request(self, request_id, code, message):
        with self._lock:
            active = self._active
            if active is None or active["request_id"] != request_id:
                return
            if bool(active.get("arm_deployed", False)) and not bool(
                active.get("arm_returned", False)
            ):
                active["state"] = TraversalStatus.ARM_RETURNING
                active["latched"] = True
                active["latched_error"] = code
                latch = True
            else:
                latch = False
        if latch:
            self._publish_active(
                f"[{code}] {message}; Piper return is unconfirmed and release is forbidden"
            )
            return
        with self._lock:
            active = self._active
            if active is None or active["request_id"] != request_id:
                return
            for handle in active["goal_handles"]:
                handle.cancel_goal_async()
            self._cache[active["obstacle_id"]] = (
                "error", active["robot_at_trigger"], active["track_reference"],
                False, 0.0,
            )
            terminal = dict(active)
            self._active = None
        revoke = FilterAuthorization()
        revoke.header.stamp = self.get_clock().now().to_msg()
        revoke.request_id = request_id
        revoke.authorized = False
        self._filter_pub.publish(revoke)
        self._publish_terminal(TraversalStatus.ERROR, code, message, context=terminal)

    def _cancel_active(self, code, message):
        with self._lock:
            active = self._active
            if active is None:
                return
            request_id = active["request_id"]
            arm_may_be_deployed = bool(
                active.get("arm_deployed", False)
                and not active.get("arm_returned", False)
            )
            if arm_may_be_deployed:
                active["cancel_after_return"] = (code, message)
                active["state"] = TraversalStatus.ARM_RETURNING
                active["latched"] = True
                active["latched_error"] = code
                handles = list(active["goal_handles"])
            else:
                handles = []
        if arm_may_be_deployed:
            for handle in handles:
                handle.cancel_goal_async()
            self._publish_active(
                f"[{code}] Task canceled; waiting for confirmed Piper return"
            )
        else:
            self._fail_request(request_id, code, message)

    def _set_idle_reason(self, reason: str, message: str) -> None:
        with self._lock:
            guard_active = self._pretrigger_guard is not None
        if guard_active:
            self._publish_pretrigger_guard(
                f"[{reason}] {message}; holding NeuPAN at zero"
            )
            return
        now = time.monotonic()
        with self._lock:
            if self._active is not None or self._last_status.state != TraversalStatus.IDLE:
                return
            changed = reason != self._last_idle_reason
            self._last_idle_reason = reason
            output = TraversalStatus()
            output.header.stamp = self.get_clock().now().to_msg()
            output.header.frame_id = str(self.get_parameter("map_frame").value)
            output.task_id = self._task_id
            output.state = TraversalStatus.IDLE
            output.message = f"[{reason}] {message}"
            self._last_status = output
            should_log = changed or now - self._last_idle_log >= 5.0
            if should_log:
                self._last_idle_log = now
        if changed:
            self._status_pub.publish(output)
        if should_log:
            self.get_logger().info(output.message)

    def _heartbeat(self):
        ready = self._ready()
        self._ready_pub.publish(Bool(data=ready))
        with self._lock:
            active = self._active
            guard = self._pretrigger_guard
        if active is not None:
            self._publish_active(self._last_status.message)
        elif guard is not None:
            self._publish_pretrigger_guard(self._last_status.message)
        else:
            self._status_pub.publish(self._last_status)

    def _ready(self):
        now, freshness = time.monotonic(), float(self.get_parameter("input_freshness").value)
        # Startup readiness must be reachable before Module4 has dispatched a
        # goal. A path is required to trigger an obstacle request, but it is not
        # a prerequisite for accepting the first navigation instruction.
        inputs = traversal_sensor_inputs_ready(self._received, now, freshness)
        if not inputs:
            return False
        try:
            self._robot_xy()
            return True
        except TransformException:
            return False

    def _publish_active(self, message, dino_ok=None, probability=None):
        with self._lock:
            active = self._active
            if active is None:
                return
            output = TraversalStatus()
            output.header.stamp = self.get_clock().now().to_msg()
            output.header.frame_id = str(self.get_parameter("map_frame").value)
            output.task_id = active["task_id"]
            output.request_id = active["request_id"]
            output.obstacle_id = active["obstacle_id"]
            output.state = active["state"]
            output.navigation_paused = output.state in (
                TraversalStatus.SNAPSHOT_PENDING, TraversalStatus.VERIFYING,
                TraversalStatus.APPROACHING, TraversalStatus.ALIGNING,
                TraversalStatus.ARM_READY, TraversalStatus.PROBING,
                TraversalStatus.ARM_RETURNING, TraversalStatus.FUSING,
                TraversalStatus.RECOVERING,
            )
            output.filter_authorized = output.state in (
                TraversalStatus.AUTHORIZED, TraversalStatus.TRAVERSING
            )
            if dino_ok is not None:
                active["dino_verified"] = bool(dino_ok)
            if probability is not None:
                active["pushability_probability"] = float(probability)
            output.dino_verified = bool(active["dino_verified"])
            output.pushability_probability = float(active["pushability_probability"])
            output.dino_probability = float(active.get("dino_probability", 0.0))
            output.mechanical_probability = float(
                active.get("mechanical_probability", 0.0)
            )
            output.fusion_probability = float(active.get("fusion_probability", 0.0))
            output.approach_clearance_m = float(
                active.get("approach_clearance", math.nan)
            )
            output.arm_returned = bool(active.get("arm_returned", False))
            output.roi = copy.deepcopy(active["roi"])
            output.obstacle_center_map.x, output.obstacle_center_map.y = active["center"]
            output.obstacle_distance_m = active["distance"]
            output.dino_started_offset_ms = active["dino_offset_ms"]
            output.llmdecision_started_offset_ms = active["pushability_offset_ms"]
            output.elapsed_ms = (time.monotonic() - active["started"]) * 1000.0
            output.removed_points = active["removed_points"]
            output.error_code = str(active.get("latched_error", ""))
            output.message = message
            self._last_status = output
        self._status_pub.publish(output)

    def _publish_terminal(
        self, state, code, message, *, dino_ok=None, probability=None, context=None
    ):
        output = TraversalStatus()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = str(self.get_parameter("map_frame").value)
        context = context or {}
        output.task_id = str(context.get("task_id", self._task_id))
        output.request_id = str(context.get("request_id", ""))
        output.obstacle_id = str(context.get("obstacle_id", ""))
        output.state = state
        output.navigation_paused = False
        output.filter_authorized = False
        output.dino_verified = bool(
            context.get("dino_verified", False) if dino_ok is None else dino_ok
        )
        output.pushability_probability = float(
            context.get("pushability_probability", 0.0)
            if probability is None else probability
        )
        output.dino_probability = float(context.get("dino_probability", 0.0))
        output.mechanical_probability = float(
            context.get("mechanical_probability", 0.0)
        )
        output.fusion_probability = float(context.get("fusion_probability", 0.0))
        output.approach_clearance_m = float(
            context.get("approach_clearance", math.nan)
        )
        output.arm_returned = bool(context.get("arm_returned", False))
        if context.get("roi") is not None:
            output.roi = copy.deepcopy(context["roi"])
        if context.get("center") is not None:
            output.obstacle_center_map.x, output.obstacle_center_map.y = context["center"]
        output.obstacle_distance_m = float(context.get("distance", 0.0))
        output.dino_started_offset_ms = float(context.get("dino_offset_ms", 0.0))
        output.llmdecision_started_offset_ms = float(
            context.get("pushability_offset_ms", 0.0)
        )
        output.elapsed_ms = (
            (time.monotonic() - float(context["started"])) * 1000.0
            if "started" in context else 0.0
        )
        output.removed_points = int(context.get("removed_points", 0))
        output.error_code = code
        output.message = message
        self._last_status = output
        self._status_pub.publish(output)

    def _publish_debug(self, compressed, roi, target_roi):
        image = cv2.imdecode(np.frombuffer(bytes(compressed.data), np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return
        cv2.rectangle(
            image, (int(roi.x_offset), int(roi.y_offset)),
            (int(roi.x_offset + roi.width), int(roi.y_offset + roi.height)), (0, 255, 255), 2,
        )
        cv2.rectangle(
            image, (int(target_roi.x_offset), int(target_roi.y_offset)),
            (
                int(target_roi.x_offset + target_roi.width),
                int(target_roi.y_offset + target_roi.height),
            ),
            (0, 255, 0), 3,
        )
        success, encoded = cv2.imencode(".jpg", image)
        if success:
            output = CompressedImage()
            output.header = compressed.header
            output.format = "jpeg"
            output.data = encoded.tobytes()
            self._debug_pub.publish(output)

    def _robot_pose(self):
        transform = self._tf.lookup_transform(
            str(self.get_parameter("map_frame").value),
            str(self.get_parameter("robot_frame").value), rclpy.time.Time(),
            timeout=Duration(seconds=0.05),
        )
        value = transform.transform.translation
        matrix = transform_matrix(value, transform.transform.rotation)
        yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
        return float(value.x), float(value.y), yaw

    def _robot_xy(self):
        return self._robot_pose()[:2]

    def _vehicle_state(self):
        state = VehicleState()
        state.mass_kg = float(self.get_parameter("vehicle_mass_kg").value)
        state.payload_kg = float(self.get_parameter("vehicle_payload_kg").value)
        state.maximum_push_force_n = float(self.get_parameter("maximum_push_force_n").value)
        state.footprint_width_m = float(self.get_parameter("footprint_width_m").value)
        state.footprint_length_m = float(self.get_parameter("footprint_length_m").value)
        state.battery_fraction = 1.0
        return state

    @staticmethod
    def _stamp(header):
        return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9

    @staticmethod
    def _signature(path):
        geometry = [
            (round(p.pose.position.x, 3), round(p.pose.position.y, 3)) for p in path.poses
        ]
        return hashlib.sha256(repr(geometry).encode()).hexdigest()

    def _obstacle_id(self, point):
        task = self._task_id or "no-task"
        return f"{task}:{round(point[0] / 0.25)}:{round(point[1] / 0.25)}"

    def _nearest_cached(self, point):
        maximum_distance = float(
            self.get_parameter("cache_match_distance").value
        )
        matches = [
            (key, value)
            for key, value in self._cache.items()
            if math.dist(point, value[2]) <= maximum_distance
        ]
        if not matches:
            return None, None
        return min(matches, key=lambda item: math.dist(point, item[1][2]))

    @staticmethod
    def _dino_summary(result):
        if isinstance(result, str):
            return f"dino_error={result}"
        candidates = list(result.candidates)
        highest = max(
            (float(value.grounding_confidence) for value in candidates),
            default=0.0,
        )
        phrases = sorted({str(value.phrase) for value in candidates})
        return (
            f"dino_verified={bool(result.verified)}, "
            f"dino_candidates={len(candidates)}, dino_highest={highest:.3f}, "
            f"dino_phrases={phrases}, dino_error={result.error_code or 'none'}, "
            f"dino_message={result.message}"
        )

    @staticmethod
    def _pushability_summary(result):
        if isinstance(result, str):
            return f"pushability_error={result}"
        return (
            f"pushability_state={int(result.state)}, "
            f"pushability_probability={float(result.pushability_probability):.3f}, "
            f"pushability_category={result.object_category or 'none'}, "
            f"pushability_error={result.error_code or 'none'}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TraversalManager()
    executor = MultiThreadedExecutor(num_threads=6)
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
