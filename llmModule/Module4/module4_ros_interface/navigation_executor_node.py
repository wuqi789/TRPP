#!/usr/bin/env python3
"""Bridge verified semantic goals to FIFO NeuPAN point navigation."""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
import math
import os
import threading
import time
import uuid

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from semantic_navigation_interfaces.action import ExecuteNavigation
from semantic_navigation_interfaces.msg import (
    CheckResult,
    NavigationTaskStatus,
    PushabilityMap,
    SystemReadiness,
    VerificationResult,
)
from obstacle_traversal_interfaces.msg import TraversalStatus
from tf2_ros import Buffer, TransformException, TransformListener

from execution_core import FifoTracker, RequestRegistry, StabilityTracker
from route_planning import (
    ArtifactStore,
    create_vlm_adapter,
    GridMap,
    GridPoint,
    RoutePlanner,
    RoutePlanningError,
    VLMConfigurationError,
    VLMProviderUnavailable,
    VLMResponseError,
)


@dataclass
class TaskRecord:
    task_id: str
    verification: object
    goal_handle: object | None = None
    state: str = "WAITING_INPUTS"
    failure_code: str = ""
    message: str = ""
    terminal: bool = False
    tracker: FifoTracker | None = None
    planning_attempt: int = 0
    frame_id: str = ""
    tf_unavailable_since: float | None = None
    done: threading.Event = field(default_factory=threading.Event)
    traversal_pause_started: float | None = None
    traversal_filter_active: bool = False


class PreparationError(RuntimeError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


class NavigationExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("navigation_executor_node")
        share = get_package_share_directory("llm_module4")
        self.declare_parameter("config_path", f"{share}/config/navigation.yaml")
        with open(
            str(self.get_parameter("config_path").value), "r", encoding="utf-8"
        ) as stream:
            config = yaml.safe_load(stream) or {}
        execution = config.get("execution", {})
        topics = config.get("topics", {})
        vlm_config = config.get("vlm", {})
        grid_config = config.get("grid", {})
        gate_config = config.get("gates", {})
        artifacts = os.environ.get(
            "MODULE4_ARTIFACT_DIR",
            str(config.get("artifacts", {}).get("directory", "runtime/module4_routes")),
        )
        artifacts = os.path.expandvars(os.path.expanduser(artifacts))
        if not os.path.isabs(artifacts):
            artifact_base = os.environ.get("SCOUT_WORKSPACE", os.getcwd())
            artifacts = os.path.join(artifact_base, artifacts)

        self._map_topic = str(topics.get("map", "/semantic_validation/map"))
        self._grid_frame = str(execution.get("global_frame", "odom"))
        self._robot_frame = str(execution.get("robot_frame", "base_link"))
        self._goal_frame = str(execution.get("goal_frame", "map"))
        self._arrival_distance = float(execution.get("arrival_distance", 0.15))
        self._arrival_hold = float(execution.get("arrival_hold_seconds", 0.5))
        self._segment_timeout = float(execution.get("segment_timeout", 120.0))
        self._no_progress_timeout = float(execution.get("no_progress_timeout", 30.0))
        self._progress_distance = float(execution.get("progress_distance", 0.05))
        self._waypoint_pass_lateral_distance = float(
            execution.get("waypoint_pass_lateral_distance", 0.75)
        )
        self._tf_timeout = float(execution.get("tf_timeout", 5.0))
        self._hold_movement_distance = float(
            execution.get("hold_movement_distance", 0.05)
        )
        self._hold_stable_seconds = float(
            execution.get("hold_stable_seconds", 0.5)
        )
        self._hold_timeout = float(execution.get("hold_timeout", 5.0))
        self._start_drift_distance = float(
            execution.get("start_drift_distance", 0.15)
        )
        self._traversal_pause_limit = float(
            execution.get("traversal_pause_limit", 150.0)
        )
        self._require_verified = bool(
            gate_config.get("require_module3_verified", True)
        )
        self._execution_watchdog = bool(
            gate_config.get("execution_watchdog", True)
        )
        self._occupied_threshold = int(grid_config.get("occupied_threshold", 50))
        self._inflation_radius = float(grid_config.get("inflation_radius", 0.10))
        self._snap_max_distance = float(
            grid_config.get("snap_max_distance", 0.50)
        )
        self._render_scale = int(grid_config.get("render_scale", 4))
        self._pushability_confidence = float(
            grid_config.get("pushability_confidence", 0.85)
        )
        self._pushability_padding = float(
            grid_config.get("pushability_padding", 0.05)
        )
        self._pushability_freeze_wait = float(
            grid_config.get("pushability_freeze_wait", 1.0)
        )
        self._map: OccupancyGrid | None = None
        self._pushability: PushabilityMap | None = None
        self._lock = threading.RLock()
        self._active: TaskRecord | None = None
        self._requests = RequestRegistry(
            int(execution.get("request_history_size", 256))
        )

        self._vlm_error = ""
        try:
            self._route_planner = RoutePlanner(
                create_vlm_adapter(
                    str(vlm_config.get("mode", "mock")),
                    str(vlm_config.get("adapter_env", "SCOUT_ROUTE_ADAPTER")),
                ),
                ArtifactStore(artifacts),
                response_retries=int(vlm_config.get("response_retries", 1)),
                transport_attempts=int(vlm_config.get("transport_attempts", 3)),
                maximum_segment_points=int(vlm_config.get("maximum_segment_points", 24)),
                maximum_route_points=int(vlm_config.get("maximum_route_points", 64)),
                snap_max_distance=self._snap_max_distance,
                transport_retry_backoff=float(
                    vlm_config.get("transport_retry_backoff", 1.0)
                ),
            )
        except VLMConfigurationError:
            self._vlm_error = "VLM_PROVIDER_CONFIGURATION_FAILED"
            self._route_planner = None

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_topic = str(
            topics.get("status", "/semantic_navigation/status")
        )
        self._status_publisher = self.create_publisher(
            NavigationTaskStatus,
            self._status_topic, transient_qos
        )
        self._readiness_publisher = self.create_publisher(
            SystemReadiness,
            str(topics.get("readiness", "/semantic_navigation/readiness")), transient_qos
        )
        self._route_publisher = self.create_publisher(
            Path, str(topics.get("route", "/semantic_navigation/execution_route")), transient_qos
        )
        self._current_publisher = self.create_publisher(
            PointStamped,
            str(topics.get("current", "/semantic_navigation/current_waypoint")), transient_qos
        )
        self._goal_publisher = self.create_publisher(
            PointStamped, str(topics.get("goal", "/clicked_point")), 10
        )
        self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, transient_qos)
        self.create_subscription(
            PushabilityMap,
            str(topics.get("pushability", "/semantic_mapping/pushability_map")),
            self._on_pushability,
            transient_qos,
        )
        self.create_subscription(
            VerificationResult,
            str(topics.get("verification", "/semantic_navigation/verification")),
            self._on_verification, 10
        )
        self.create_subscription(
            TraversalStatus,
            str(topics.get("traversal_status", "/obstacle_traversal/status")),
            self._on_traversal_status,
            transient_qos,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        callbacks = ReentrantCallbackGroup()
        action_topic = str(topics.get("action", "/semantic_navigation/execute"))
        self._action_server = ActionServer(
            self, ExecuteNavigation, action_topic,
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel,
            callback_group=callbacks,
        )
        self._ingress_client = ActionClient(
            self, ExecuteNavigation, action_topic, callback_group=callbacks
        )
        self.create_timer(0.1, self._tick, callback_group=callbacks)
        self.create_timer(1.0, self._publish_readiness)
        if self._route_planner is not None:
            self.get_logger().info(
                "Module4 route adapter ready; goal=/clicked_point"
            )
        else:
            self.get_logger().error(f"Module4 VLM configuration failed: {self._vlm_error}")
        if not self._require_verified:
            self.get_logger().warning(
                "RESEARCH VALVE OPEN: Module3 verified=false results may enter Module4"
            )
        if not self._execution_watchdog:
            self.get_logger().warning(
                "RESEARCH VALVE OPEN: Module4 segment/no-progress watchdog is disabled"
            )

    def _on_map(self, message: OccupancyGrid) -> None:
        with self._lock:
            self._map = copy.deepcopy(message)

    def _on_pushability(self, message: PushabilityMap) -> None:
        with self._lock:
            self._pushability = copy.deepcopy(message)

    def _on_traversal_status(self, message: TraversalStatus) -> None:
        now = time.monotonic()
        replan = None
        with self._lock:
            record = self._active
            if (
                record is None or record.terminal or record.state != "NAVIGATING"
                or (message.task_id and message.task_id != record.task_id)
            ):
                return
            tracker = record.tracker
            if tracker is None:
                return
            if message.navigation_paused and record.traversal_pause_started is None:
                record.traversal_pause_started = now
                tracker.pause(now)
                self._publish_status(
                    record, "Navigation watchdogs paused for obstacle verification"
                )
            elif not message.navigation_paused and record.traversal_pause_started is not None:
                tracker.resume(now)
                paused = now - record.traversal_pause_started
                record.traversal_pause_started = None
                self._publish_status(
                    record, f"Obstacle verification finished; watchdogs shifted by {paused:.1f}s"
                )
            filter_active = bool(message.filter_authorized)
            if filter_active and not record.traversal_filter_active:
                record.traversal_filter_active = True
                if tracker.current is not None:
                    replan = (record, tracker.current, record.frame_id or self._grid_frame)
            elif not filter_active:
                record.traversal_filter_active = False
        if replan is not None:
            record, target, frame = replan
            try:
                self._publish_target(target, frame)
                self._publish_status(
                    record,
                    "Tracked scan filter activated; redispatched the current FIFO target",
                )
            except TransformException:
                self.get_logger().warning(
                    "Could not redispatch the current FIFO target when scan filtering activated"
                )

    def _on_verification(self, message: VerificationResult) -> None:
        request_id = str(message.request_id)
        with self._lock:
            if not self._requests.reserve_topic(request_id):
                self.get_logger().warning(
                    f"Ignoring duplicate or empty verification request_id '{request_id}'"
                )
                return
        if not self._single_instance_ready():
            with self._lock:
                self._requests.release_topic(request_id)
            self.get_logger().error(
                "Ignoring verification because multiple Module4 status publishers exist"
            )
            return
        if not self._ingress_client.server_is_ready():
            with self._lock:
                self._requests.release_topic(request_id)
            self.get_logger().error("ExecuteNavigation action server is not ready")
            return
        goal = ExecuteNavigation.Goal()
        goal.verification = message
        future = self._ingress_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed, value=request_id: self._on_ingress_goal(completed, value)
        )

    def _on_ingress_goal(self, future, request_id: str) -> None:
        try:
            accepted = bool(future.result().accepted)
        except Exception:
            accepted = False
            self.get_logger().error("VERIFICATION_SUBMISSION_FAILED")
        if not accepted:
            with self._lock:
                self._requests.release_topic(request_id)

    def _goal_callback(self, request) -> GoalResponse:
        request_id = str(request.verification.request_id)
        if not self._single_instance_ready():
            with self._lock:
                self._requests.release_topic(request_id)
            self.get_logger().error(
                f"Rejecting action request {request_id}: multiple Module4 instances detected"
            )
            return GoalResponse.REJECT
        with self._lock:
            if not self._requests.accept_goal(request_id):
                self.get_logger().warning(
                    f"Rejecting duplicate or empty action request_id '{request_id}'"
                )
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel(self, goal_handle) -> CancelResponse:
        with self._lock:
            record = self._active
            if record is not None and record.goal_handle is goal_handle:
                self._finish(record, "CANCELED", "CANCELED", "Task canceled", hold=True)
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        verification = goal_handle.request.verification
        record = TaskRecord(str(uuid.uuid4()), verification, goal_handle=goal_handle)
        if not verification.verified and self._require_verified:
            self._finish(
                record, "REJECTED",
                verification.error_code or "VERIFICATION_REJECTED",
                verification.message or "Module3 rejected the goal", hold=False
            )
            return self._action_result(goal_handle, record)
        if not verification.verified:
            self.get_logger().warning(
                f"Research bypass accepted rejected request {verification.request_id}: "
                f"{verification.error_code}"
            )

        with self._lock:
            previous = self._active
            if previous is not None and not previous.terminal:
                self._finish(
                    previous, "PREEMPTED", "PREEMPTED_BY_NEW_TASK",
                    "A newer verified task preempted this task", hold=True
                )
            self._active = record
        self._publish_status(record, "Waiting for map and robot transform")
        try:
            self._prepare_and_dispatch(record)
        except VLMConfigurationError:
            self._finish(record, "FAILED", "VLM_CONFIGURATION_ERROR", "VLM provider configuration failed", hold=True)
        except VLMProviderUnavailable:
            self._finish(record, "FAILED", "VLM_PROVIDER_UNAVAILABLE", "VLM provider unavailable", hold=True)
        except VLMResponseError:
            self._finish(record, "FAILED", "VLM_RESPONSE_INVALID", "VLM response contract rejected", hold=True)
        except PreparationError as exc:
            self._finish(
                record,
                "FAILED",
                exc.code,
                exc.code,
                hold=exc.code != "DUPLICATE_MODULE4_INSTANCE",
            )
        except (RoutePlanningError, ValueError, TransformException):
            self._finish(record, "FAILED", "ROUTE_GENERATION_FAILED", "Route generation failed", hold=True)
        except Exception:
            self.get_logger().error("Unexpected Module4 route generation failure")
            self._finish(record, "FAILED", "EXECUTION_ERROR", "Route execution failed", hold=True)

        while not record.done.wait(0.1):
            if not rclpy.ok():
                record.state = "CANCELED"
                record.failure_code = "ROS_SHUTDOWN"
                record.message = "ROS context is shutting down"
                record.terminal = True
                record.done.set()
                return self._action_result(goal_handle, record, update_goal=False)
            if goal_handle.is_cancel_requested:
                self._finish(record, "CANCELED", "CANCELED", "Task canceled", hold=True)
        return self._action_result(goal_handle, record)

    def _prepare_and_dispatch(self, record: TaskRecord) -> None:
        with self._lock:
            grid_message = copy.deepcopy(self._map)
        if grid_message is None:
            raise RoutePlanningError("no normalized occupancy map has been received")
        if self._route_planner is None:
            raise VLMConfigurationError(self._vlm_error or "VLM is not configured")
        if not self._single_instance_ready():
            raise PreparationError(
                "DUPLICATE_MODULE4_INSTANCE",
                "multiple Module4 status publishers are present",
            )
        map_frame = grid_message.header.frame_id.lstrip("/") or self._grid_frame
        with self._lock:
            if not self._is_current(record):
                return
            record.frame_id = map_frame
        start = self._hold_until_stable(record, map_frame)
        pushability = self._wait_for_pushability_freeze(record)
        with self._lock:
            grid_message = copy.deepcopy(self._map)
            if not self._is_current(record):
                return
            if grid_message is None:
                raise RoutePlanningError("normalized occupancy map disappeared")
            record.state = "GENERATING_ROUTE"
            self._publish_status(
                record, "Calling VLM and sanitizing candidate waypoints"
            )
        avoids = []
        vias = []
        for constraint in record.verification.constraints:
            point = self._pose_in_frame(constraint.pose, map_frame)
            kind = str(constraint.type).casefold()
            if kind == "avoid":
                avoids.append((point.x, point.y, float(constraint.radius)))
            elif kind == "via":
                vias.append((int(constraint.order), point))
        vias.sort(key=lambda item: item[0])
        goal = self._pose_in_frame(record.verification.goal_pose, map_frame)
        anchors = [point for _, point in vias] + [goal]
        pushable_regions = self._pushable_regions(pushability, map_frame)
        origin = grid_message.info.origin
        grid = GridMap(
            width=grid_message.info.width,
            height=grid_message.info.height,
            resolution=grid_message.info.resolution,
            origin_x=grid_message.info.origin.position.x,
            origin_y=grid_message.info.origin.position.y,
            origin_yaw=self._yaw(origin.orientation),
            data=grid_message.data,
            frame_id=map_frame,
            occupied_threshold=self._occupied_threshold,
            inflation_radius=self._inflation_radius,
            scale=self._render_scale,
            avoid_regions=avoids,
            pushable_regions=pushable_regions,
        )
        planned = self._route_planner.plan(
            record.task_id,
            grid,
            start,
            anchors,
            attempt_callback=lambda attempt: self._on_planning_attempt(
                record, attempt
            ),
        )
        with self._lock:
            if not self._is_current(record):
                return
        current = self._robot_position(map_frame)
        drift = math.dist((start.x, start.y), (current.x, current.y))
        with self._lock:
            if not self._is_current(record):
                return
            if drift > self._start_drift_distance:
                self._publish_hold(current, map_frame)
                raise PreparationError(
                    "START_POSE_CHANGED",
                    f"robot moved {drift:.3f}m while the VLM route was generated; "
                    f"limit is {self._start_drift_distance:.3f}m",
                )
            if not self._single_instance_ready():
                raise PreparationError(
                    "DUPLICATE_MODULE4_INSTANCE",
                    "another Module4 instance appeared before route dispatch",
                )
            record.tracker = FifoTracker(
                arrival_distance=self._arrival_distance,
                arrival_hold_seconds=self._arrival_hold,
                segment_timeout=self._segment_timeout,
                no_progress_timeout=self._no_progress_timeout,
                progress_distance=self._progress_distance,
                waypoint_pass_lateral_distance=(
                    self._waypoint_pass_lateral_distance
                ),
                watchdog_enabled=self._execution_watchdog,
            )
            self._publish_route(start, list(planned.waypoints), map_frame)
            event = record.tracker.start(
                planned.waypoints, current, time.monotonic()
            )
            if event.kind == "COMPLETE":
                self._finish(record, "SUCCEEDED", "", "All route segments completed")
            else:
                record.state = "NAVIGATING"
                self._publish_target(event.target, map_frame)
                self._publish_status(
                    record, "Navigating to the current FIFO waypoint"
                )

    def _wait_for_pushability_freeze(
        self, record: TaskRecord
    ) -> PushabilityMap | None:
        deadline = time.monotonic() + max(0.0, self._pushability_freeze_wait)
        while True:
            with self._lock:
                snapshot = copy.deepcopy(self._pushability)
            if snapshot is None or snapshot.frozen:
                return snapshot
            if not self._is_current(record):
                return None
            if time.monotonic() >= deadline:
                self.get_logger().warning(
                    "Pushability freeze acknowledgement timed out; using the latest "
                    "completed in-memory snapshot"
                )
                return snapshot
            if record.done.wait(0.05):
                return None

    def _pushable_regions(
        self, message: PushabilityMap | None, target_frame: str
    ) -> list[tuple[float, float, float, float]]:
        if message is None:
            return []
        source_frame = message.header.frame_id.lstrip("/") or target_frame
        regions = []
        for item in message.objects:
            if not item.pushable or float(item.confidence) < self._pushability_confidence:
                continue
            size_x = float(item.size.x)
            size_y = float(item.size.y)
            if (
                not math.isfinite(size_x)
                or not math.isfinite(size_y)
                or size_x <= 0.0
                or size_y <= 0.0
            ):
                continue
            corners = []
            for offset_x, offset_y in (
                (-size_x / 2.0, -size_y / 2.0),
                (-size_x / 2.0, size_y / 2.0),
                (size_x / 2.0, -size_y / 2.0),
                (size_x / 2.0, size_y / 2.0),
            ):
                point = PointStamped()
                point.header.frame_id = source_frame
                point.point.x = item.pose.position.x + offset_x
                point.point.y = item.pose.position.y + offset_y
                transformed = self._transform_point(point, target_frame)
                corners.append((transformed.point.x, transformed.point.y))
            minimum_x = min(value[0] for value in corners)
            maximum_x = max(value[0] for value in corners)
            minimum_y = min(value[1] for value in corners)
            maximum_y = max(value[1] for value in corners)
            padding = max(0.0, self._pushability_padding)
            regions.append((
                (minimum_x + maximum_x) / 2.0,
                (minimum_y + maximum_y) / 2.0,
                maximum_x - minimum_x + 2.0 * padding,
                maximum_y - minimum_y + 2.0 * padding,
            ))
        return regions

    def _hold_until_stable(self, record: TaskRecord, frame: str) -> GridPoint:
        position = self._robot_position(frame)
        self._publish_hold(position, frame)
        tracker = StabilityTracker(
            movement_distance=self._hold_movement_distance,
            hold_seconds=self._hold_stable_seconds,
            timeout=self._hold_timeout,
        )
        tracker.start(position, time.monotonic())
        self._publish_status(record, "Holding position before VLM route generation")
        while True:
            if not self._is_current(record):
                return position
            if not self._single_instance_ready():
                raise PreparationError(
                    "DUPLICATE_MODULE4_INSTANCE",
                    "multiple Module4 instances detected while holding position",
                )
            position = self._robot_position(frame)
            event = tracker.tick(position, time.monotonic())
            if event.kind == "STABLE":
                return position
            if event.kind == "TIMEOUT":
                raise PreparationError(
                    "HOLD_TIMEOUT",
                    f"robot did not remain within {self._hold_movement_distance:.3f}m "
                    f"for {self._hold_stable_seconds:.1f}s before the "
                    f"{self._hold_timeout:.1f}s hold timeout",
                )
            if record.done.wait(0.1):
                return position

    def _on_planning_attempt(self, record: TaskRecord, attempt: int) -> None:
        with self._lock:
            if not self._is_current(record):
                return
            record.planning_attempt = int(attempt)
            self._publish_status(
                record,
                f"Calling VLM response round {attempt} and sanitizing waypoints",
            )

    def _tick(self) -> None:
        with self._lock:
            record = self._active
            if record is None or record.terminal or record.state != "NAVIGATING":
                return
            tracker = record.tracker
        if tracker is None or tracker.current is None:
            return
        if record.traversal_pause_started is not None:
            now = time.monotonic()
            if now - record.traversal_pause_started <= self._traversal_pause_limit:
                return
            with self._lock:
                if self._is_current(record) and record.tracker is not None:
                    record.tracker.resume(now)
                    record.traversal_pause_started = None
                    self.get_logger().error(
                        "Obstacle traversal pause watchdog expired; resuming Module4 timers"
                    )
        if not self._single_instance_ready():
            self._finish(
                record,
                "FAILED",
                "DUPLICATE_MODULE4_INSTANCE",
                "multiple Module4 instances detected during navigation",
                hold=False,
            )
            return
        now = time.monotonic()
        frame = record.frame_id or self._grid_frame
        try:
            robot = self._robot_position(frame)
        except TransformException:
            with self._lock:
                if not self._is_current(record):
                    return
                if record.tf_unavailable_since is None:
                    record.tf_unavailable_since = now
                elif now - record.tf_unavailable_since >= self._tf_timeout:
                    self._finish(
                        record, "FAILED", "TF_UNAVAILABLE",
                        "Robot transform unavailable", hold=True
                    )
            return
        with self._lock:
            if not self._is_current(record) or record.terminal:
                return
            record.tf_unavailable_since = None
            distance = tracker.distance_remaining(robot)
            event = tracker.tick(robot, now)
            if event.kind == "FAILED":
                self._finish(
                    record, "FAILED", event.code, event.message, hold=True
                )
            elif event.kind == "COMPLETE":
                self._finish(record, "SUCCEEDED", "", "All route segments completed")
            elif event.kind == "TARGET":
                self._publish_target(event.target, frame)
                self._publish_status(record, "Navigating to the current FIFO waypoint")
            else:
                self._publish_status(record, record.message, distance)

    def _finish(
        self, record: TaskRecord, state: str, code: str, message: str, *, hold: bool = False
    ) -> None:
        with self._lock:
            if record.terminal:
                return
            final_robot = None
            final_target = record.tracker.current if record.tracker is not None else None
            try:
                final_robot = self._robot_position(record.frame_id or self._grid_frame)
            except TransformException:
                pass
            record.state = state
            record.failure_code = code
            record.message = message
            record.terminal = True
            if record.tracker is not None:
                record.tracker.clear()
            if hold:
                try:
                    frame = record.frame_id or self._grid_frame
                    self._publish_hold(self._robot_position(frame), frame)
                except TransformException:
                    self.get_logger().error(
                        "Could not publish hold target because TF is unavailable"
                    )
            if self._active is record:
                self._active = None
            if self._route_planner is not None:
                try:
                    self._route_planner.artifacts.write_json(
                        record.task_id,
                        "execution.json",
                        {
                            "task_id": record.task_id,
                            "request_id": record.verification.request_id,
                            "state": state,
                            "failure_code": code,
                            "message": message,
                            "robot": (
                                None if final_robot is None
                                else [final_robot.x, final_robot.y]
                            ),
                            "target": (
                                None if final_target is None
                                else [final_target.x, final_target.y]
                            ),
                            "planning_attempt": record.planning_attempt,
                            "segment_collision_validation": "disabled",
                            "research_valves": {
                                "require_module3_verified": self._require_verified,
                                "execution_watchdog": self._execution_watchdog,
                            },
                        },
                    )
                except OSError:
                    self.get_logger().error("MODULE4_AUDIT_PERSIST_FAILED")
            self._publish_status(record, message, terminal_event=True)
            record.done.set()

    def _publish_status(
        self, record: TaskRecord, message: str, distance: float = 0.0,
        *, terminal_event: bool = False,
    ) -> None:
        with self._lock:
            if record.terminal and not terminal_event:
                return
            record.message = message
            output = NavigationTaskStatus()
            output.task_id = record.task_id
            output.request_id = record.verification.request_id
            output.state = record.state
            output.failure_code = record.failure_code
            output.message = message
            output.distance_remaining = float(distance)
            output.planning_attempt = record.planning_attempt
            output.recovery_count = 0
            output.terminal = record.terminal
            self._status_publisher.publish(output)
            handle = record.goal_handle
            if handle is not None and handle.is_active:
                feedback = ExecuteNavigation.Feedback()
                feedback.status = output
                handle.publish_feedback(feedback)

    def _publish_route(
        self, start: GridPoint, points: list[GridPoint], frame: str
    ) -> None:
        output = Path()
        output.header.frame_id = frame
        output.header.stamp = self.get_clock().now().to_msg()
        for point in [start] + points:
            pose = PoseStamped()
            pose.header = output.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.orientation.w = 1.0
            output.poses.append(pose)
        self._route_publisher.publish(output)

    def _publish_target(self, point: GridPoint, source_frame: str) -> None:
        current = self._point_message(point, source_frame)
        current.header.stamp = self.get_clock().now().to_msg()
        self._current_publisher.publish(current)
        target = self._transform_point(current, self._goal_frame)
        target.header.stamp = self.get_clock().now().to_msg()
        self._goal_publisher.publish(target)

    def _publish_hold(self, point: GridPoint, source_frame: str) -> None:
        self._publish_target(point, source_frame)

    @staticmethod
    def _point_message(point: GridPoint, frame: str) -> PointStamped:
        output = PointStamped()
        output.header.frame_id = frame
        output.point.x = point.x
        output.point.y = point.y
        return output

    def _robot_position(self, frame: str) -> GridPoint:
        transform = self._tf_buffer.lookup_transform(frame, self._robot_frame, rclpy.time.Time())
        return GridPoint(transform.transform.translation.x, transform.transform.translation.y)

    def _pose_in_frame(self, pose: PoseStamped, frame: str) -> GridPoint:
        point = PointStamped()
        point.header = pose.header
        point.header.frame_id = pose.header.frame_id.lstrip("/") or frame
        point.point = pose.pose.position
        transformed = self._transform_point(point, frame)
        return GridPoint(transformed.point.x, transformed.point.y)

    def _transform_point(self, point: PointStamped, target_frame: str) -> PointStamped:
        source = point.header.frame_id.lstrip("/") or target_frame
        target = target_frame.lstrip("/")
        if source == target:
            output = copy.deepcopy(point)
            output.header.frame_id = target
            return output
        transform = self._tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        q = transform.transform.rotation
        yaw = self._yaw(q)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        output = PointStamped()
        output.header.frame_id = target
        output.point.x = (
            cosine * point.point.x - sine * point.point.y + transform.transform.translation.x
        )
        output.point.y = (
            sine * point.point.x + cosine * point.point.y + transform.transform.translation.y
        )
        output.point.z = point.point.z + transform.transform.translation.z
        return output

    @staticmethod
    def _yaw(quaternion) -> float:
        return math.atan2(
            2.0 * (
                quaternion.w * quaternion.z + quaternion.x * quaternion.y
            ),
            1.0 - 2.0 * (
                quaternion.y * quaternion.y + quaternion.z * quaternion.z
            ),
        )

    def _is_current(self, record: TaskRecord) -> bool:
        return self._active is record and not record.terminal

    def _single_instance_ready(self) -> bool:
        return self.count_publishers(self._status_topic) == 1

    @staticmethod
    def _action_result(goal_handle, record: TaskRecord, *, update_goal: bool = True):
        if update_goal:
            if record.state == "SUCCEEDED":
                goal_handle.succeed()
            elif record.state == "CANCELED":
                goal_handle.canceled()
            else:
                goal_handle.abort()
        result = ExecuteNavigation.Result()
        result.task_id = record.task_id
        result.terminal_state = record.state
        result.failure_code = record.failure_code
        result.message = record.message
        return result

    def _publish_readiness(self) -> None:
        with self._lock:
            map_message = self._map
        map_ready = map_message is not None
        map_frame = (
            map_message.header.frame_id.lstrip("/")
            if map_message is not None else ""
        ) or self._grid_frame
        try:
            self._robot_position(map_frame)
            tf_ready = True
        except TransformException:
            tf_ready = False
        vlm_ready = self._route_planner is not None and bool(
            getattr(self._route_planner.vlm, "ready", False)
        )
        singleton_ready = self._single_instance_ready()
        checks = []
        for name, ready, code in (
            ("singleton", singleton_ready, "DUPLICATE_MODULE4_INSTANCE"),
            ("map", map_ready, "MAP_UNAVAILABLE"),
            ("tf", tf_ready, "TF_UNAVAILABLE"),
            ("vlm", vlm_ready, "VLM_UNAVAILABLE"),
        ):
            check = CheckResult()
            check.name = name
            check.status = "PASS" if ready else "FAIL"
            check.code = "" if ready else code
            checks.append(check)
        output = SystemReadiness()
        output.stamp = self.get_clock().now().to_msg()
        output.ready = all(check.status == "PASS" for check in checks)
        output.checks = checks
        output.message = (
            "Module4 VLM FIFO bridge is ready"
            if output.ready
            else "Module4 is waiting for a single instance, map, TF, or VLM configuration"
        )
        self._readiness_publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationExecutorNode()
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
