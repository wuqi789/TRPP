"""Synchronous facade over Nav2 planning actions for worker-thread use."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from rclpy.action import ActionClient


@dataclass(frozen=True)
class PlannerOutcome:
    success: bool
    path: object | None = None
    planning_time: float = 0.0
    code: str = ""
    message: str = ""


def path_length(path) -> float:
    points = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
    return sum(math.dist(left, right) for left, right in zip(points, points[1:]))


class Nav2PlannerClient:
    def __init__(
        self,
        node,
        *,
        namespace: str = "",
        planner_id: str = "GridBased",
    ) -> None:
        prefix = "/" + namespace.strip("/") if namespace.strip("/") else ""
        self.planner_id = planner_id
        self._single = ActionClient(
            node, ComputePathToPose, f"{prefix}/compute_path_to_pose"
        )
        self._through = ActionClient(
            node, ComputePathThroughPoses, f"{prefix}/compute_path_through_poses"
        )
        self._active_goal_handle = None
        self._active_lock = threading.Lock()
        self._plan_lock = threading.Lock()

    @property
    def ready(self) -> bool:
        return self.single_ready and self.through_ready

    @property
    def single_ready(self) -> bool:
        return self._single.server_is_ready()

    @property
    def through_ready(self) -> bool:
        return self._through.server_is_ready()

    def cancel_active(self) -> None:
        with self._active_lock:
            handle = self._active_goal_handle
        if handle is not None:
            handle.cancel_goal_async()

    def plan(self, goal_pose, constraints, timeout: float = 10.0) -> PlannerOutcome:
        with self._plan_lock:
            return self._plan(goal_pose, constraints, timeout)

    def _plan(self, goal_pose, constraints, timeout: float) -> PlannerOutcome:
        via = sorted(
            [value for value in constraints if str(value.type).casefold() == "via"],
            key=lambda value: value.order,
        )
        if via:
            request = ComputePathThroughPoses.Goal()
            request.goals = [value.pose for value in via] + [goal_pose]
            request.planner_id = self.planner_id
            request.use_start = False
            client = self._through
        else:
            request = ComputePathToPose.Goal()
            request.goal = goal_pose
            request.planner_id = self.planner_id
            request.use_start = False
            client = self._single
        if not client.wait_for_server(timeout_sec=timeout):
            return PlannerOutcome(False, code="PLANNER_UNAVAILABLE", message="Planner action is unavailable")
        started = time.monotonic()
        future = client.send_goal_async(request)
        if not self._wait(future, timeout):
            return PlannerOutcome(False, code="PLANNER_TIMEOUT", message="Planner did not accept the request")
        try:
            goal_handle = future.result()
        except Exception:
            return PlannerOutcome(
                False,
                code="PLANNER_REQUEST_FAILED",
                message="Planner request failed",
            )
        if goal_handle is None or not goal_handle.accepted:
            return PlannerOutcome(False, code="PLANNER_REJECTED", message="Planner rejected the request")
        with self._active_lock:
            self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        remaining = max(0.0, timeout - (time.monotonic() - started))
        if not self._wait(result_future, remaining):
            goal_handle.cancel_goal_async()
            with self._active_lock:
                if self._active_goal_handle is goal_handle:
                    self._active_goal_handle = None
            return PlannerOutcome(False, code="PLANNER_TIMEOUT", message="Planner result timed out")
        with self._active_lock:
            if self._active_goal_handle is goal_handle:
                self._active_goal_handle = None
        try:
            wrapped = result_future.result()
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                return PlannerOutcome(
                    False,
                    code="PLANNER_CANCELED"
                    if wrapped.status == GoalStatus.STATUS_CANCELED
                    else "PLANNER_FAILED",
                    message=f"Planner action ended with status {wrapped.status}",
                )
            path = wrapped.result.path
            duration = wrapped.result.planning_time
            planning_time = float(duration.sec) + float(duration.nanosec) / 1e9
            if planning_time <= 0.0:
                # Nav2 measures with the ROS clock. A plan completed within one
                # Isaac clock tick reports zero, so retain the observed action
                # latency from the monotonic wall clock instead.
                planning_time = time.monotonic() - started
        except Exception:
            return PlannerOutcome(
                False,
                code="PLANNER_FAILED",
                message="Planner result processing failed",
            )
        if not path.poses:
            return PlannerOutcome(False, code="EMPTY_PATH", message="Planner returned an empty path")
        return PlannerOutcome(True, path=path, planning_time=planning_time)

    @staticmethod
    def _wait(future, timeout: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout)
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.done()
