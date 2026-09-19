#!/usr/bin/env python3
"""Drive a deterministic path through an Isaac acceptance fixture."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from semantic_navigation_interfaces.msg import NavigationTaskStatus

from obstacle_traversal_interfaces.msg import TraversalStatus


class CoreAcceptanceDriver(Node):
    """Own only the acceptance goal; all traversal providers remain production nodes."""

    def __init__(self) -> None:
        super().__init__("core_acceptance_driver")
        self.declare_parameter("scenario", "curtain")
        self.declare_parameter("goal_x", 3.42)
        self.declare_parameter("goal_y", -3.47)
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("arrival_distance_m", 0.30)
        self.declare_parameter("arrival_hold_s", 0.50)
        self.declare_parameter("timeout_s", 300.0)
        self._scenario = str(self.get_parameter("scenario").value)
        if self._scenario not in {"curtain", "movable_box", "fixed_box"}:
            raise ValueError("scenario must be curtain, movable_box, or fixed_box")
        self._goal = (
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
        )
        self._task_id = f"core-acceptance-{self._scenario}"
        self._request_id = f"core-acceptance-{self._scenario}"
        self._started = time.monotonic()
        self._goal_published = False
        self._robot = None
        self._arrival_started = None
        self._terminal = False
        self._traversal_paused = False
        self._traversal_observed = False

        transient = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            NavigationTaskStatus, "/semantic_navigation/status", transient
        )
        self._goal_pub = self.create_publisher(PointStamped, "/clicked_point", 10)
        self.create_subscription(
            Odometry, "/odom", self._on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            TraversalStatus,
            "/obstacle_traversal/status",
            self._on_traversal,
            transient,
        )
        self.create_timer(0.10, self._tick)
        self.get_logger().info(
            "[CORE_ACCEPTANCE_CONFIG] "
            f"scenario={self._scenario} goal=({self._goal[0]:.3f},{self._goal[1]:.3f}) "
            f"frame={self.get_parameter('goal_frame').value}"
        )

    def _on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self._robot = (float(position.x), float(position.y))

    def _on_traversal(self, message: TraversalStatus) -> None:
        self._traversal_paused = bool(message.navigation_paused)
        if message.request_id and int(message.state) != TraversalStatus.IDLE:
            self._traversal_observed = True

    def _tick(self) -> None:
        if self._terminal:
            return
        elapsed = time.monotonic() - self._started
        if elapsed >= float(self.get_parameter("timeout_s").value):
            self._publish_status(
                "FAILED", terminal=True, failure_code="ACCEPTANCE_TIMEOUT",
                message="Core traversal acceptance did not reach the goal in time",
            )
            self._terminal = True
            return

        self._publish_status("NAVIGATING", message="Core traversal acceptance active")
        if (
            not self._goal_published
            and self._robot is not None
            and elapsed >= 1.0
            and self._goal_pub.get_subscription_count() > 0
        ):
            goal = PointStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = str(self.get_parameter("goal_frame").value)
            goal.point.x, goal.point.y = self._goal
            self._goal_pub.publish(goal)
            self._goal_published = True
            self.get_logger().info(
                "[CORE_ACCEPTANCE_GOAL] "
                f"published=true target=({self._goal[0]:.3f},{self._goal[1]:.3f})"
            )

        if not self._goal_published or self._robot is None:
            return
        distance = math.dist(self._robot, self._goal)
        arrived = (
            distance <= float(self.get_parameter("arrival_distance_m").value)
            and not self._traversal_paused
            and self._traversal_observed
        )
        if arrived:
            if self._arrival_started is None:
                self._arrival_started = time.monotonic()
            elif time.monotonic() - self._arrival_started >= float(
                self.get_parameter("arrival_hold_s").value
            ):
                self._publish_status(
                    "SUCCEEDED", terminal=True,
                    message="Core traversal acceptance goal reached",
                    distance=distance,
                )
                self._terminal = True
                self.get_logger().info(
                    f"[CORE_ACCEPTANCE_RESULT] passed=true distance={distance:.3f}"
                )
        else:
            self._arrival_started = None

    def _publish_status(
        self, state: str, *, terminal: bool = False, failure_code: str = "",
        message: str = "", distance: float | None = None,
    ) -> None:
        output = NavigationTaskStatus()
        output.task_id = self._task_id
        output.request_id = self._request_id
        output.state = state
        output.failure_code = failure_code
        output.message = message
        if distance is None:
            distance = (
                math.dist(self._robot, self._goal)
                if self._robot is not None else math.nan
            )
        output.distance_remaining = float(distance)
        output.planning_attempt = 1
        output.recovery_count = 0
        output.terminal = bool(terminal)
        self._status_pub.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoreAcceptanceDriver()
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
