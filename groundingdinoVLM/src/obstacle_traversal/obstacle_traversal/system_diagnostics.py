#!/usr/bin/env python3
"""Human-readable live audit of the four-terminal traversal data path."""

from __future__ import annotations

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.action import get_action_names_and_types
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState, LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from obstacle_traversal_interfaces.msg import (
    FilterState, ObstacleDetection, TraversalStatus,
)
from semantic_navigation_interfaces.msg import NavigationTaskStatus, SystemReadiness


def pose_alignment(tf_pose, marker: Marker) -> tuple[float, float]:
    """Return planar translation and wrapped yaw error for the RViz marker."""
    tf_x, tf_y, tf_yaw = tf_pose
    position = marker.pose.position
    rotation = marker.pose.orientation
    marker_yaw = math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )
    yaw_error = abs(math.atan2(
        math.sin(marker_yaw - tf_yaw), math.cos(marker_yaw - tf_yaw)
    ))
    return math.hypot(position.x - tf_x, position.y - tf_y), yaw_error


class TraversalSystemDiagnostics(Node):
    def __init__(self) -> None:
        super().__init__("traversal_system_diagnostics")
        self.declare_parameter("report_rate", 1.0)
        self.declare_parameter("freshness", 2.0)
        report_rate = float(self.get_parameter("report_rate").value)
        if report_rate <= 0.0:
            raise ValueError("report_rate must be positive")

        self._seen: dict[str, float] = {}
        self._ready: dict[str, bool] = {}
        self._commands: dict[str, Twist] = {}
        self._path: Path | None = None
        self._navigation: NavigationTaskStatus | None = None
        self._traversal: TraversalStatus | None = None
        self._filter: FilterState | None = None
        self._yolo: ObstacleDetection | None = None
        self._robot_marker: Marker | None = None
        self._tf = Buffer(cache_time=Duration(seconds=10.0))
        self._listener = TransformListener(self._tf, self)

        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, "/map", lambda _: self._mark("map"), transient
        )
        for key, topic, message_type in (
            ("scan_raw", "/scan_raw", LaserScan),
            ("scan", "/scan", LaserScan),
            ("rgb", "/isaac/color_image_raw", Image),
            ("semantic", "/semantic_mapping/overlay", Image),
        ):
            self.create_subscription(
                message_type, topic, lambda _, name=key: self._mark(name), sensor
            )
        self.create_subscription(
            MarkerArray, "/semantic_mapping/labels",
            lambda _: self._mark("semantic_labels"), transient,
        )
        self.create_subscription(Marker, "/robot_marker", self._on_robot_marker, 10)
        self.create_subscription(
            JointState, "/isaac_joint_states", lambda _: self._mark("piper_joint_state"), 10
        )
        for key, topic in (
            ("dino", "/groundingdino_vlm/ready"),
            ("llmdecision", "/llmdecision/ready"),
            ("traversal", "/obstacle_traversal/ready"),
            ("piper_yolo", "/piper/yolo_ready"),
        ):
            self.create_subscription(
                Bool, topic, lambda msg, name=key: self._on_ready(name, msg), transient
            )
        for key, topic in (
            ("module3", "/semantic_navigation/verification_readiness"),
            ("module4", "/semantic_navigation/readiness"),
        ):
            self.create_subscription(
                SystemReadiness, topic,
                lambda msg, name=key: self._on_system_ready(name, msg), transient,
            )
        for key, topic in (
            ("raw", "/neupan_cmd_vel_raw"),
            ("gated", "/neupan_cmd_vel"),
            ("selected", "/cmd_vel"),
        ):
            self.create_subscription(
                Twist, topic, lambda msg, name=key: self._on_command(name, msg), 10
            )
        self.create_subscription(Path, "/neupan_initial_path", self._on_path, transient)
        self.create_subscription(
            NavigationTaskStatus, "/semantic_navigation/status",
            self._on_navigation, transient,
        )
        self.create_subscription(
            TraversalStatus, "/obstacle_traversal/status",
            self._on_traversal, transient,
        )
        self.create_subscription(
            FilterState, "/obstacle_traversal/filter_state", self._on_filter, transient
        )
        self.create_subscription(
            ObstacleDetection,
            "/piper/yolo_obstacle_result",
            self._on_yolo,
            10,
        )
        self.create_timer(1.0 / report_rate, self._report)
        self.get_logger().info("Four-terminal traversal diagnostics started")

    def _mark(self, key: str) -> None:
        self._seen[key] = time.monotonic()

    def _on_ready(self, key: str, message: Bool) -> None:
        self._ready[key] = bool(message.data)
        self._mark(key)

    def _on_system_ready(self, key: str, message: SystemReadiness) -> None:
        self._ready[key] = bool(message.ready)
        self._mark(key)

    def _on_command(self, key: str, message: Twist) -> None:
        self._commands[key] = message
        self._mark(f"cmd_{key}")

    def _on_path(self, message: Path) -> None:
        self._path = message
        self._mark("path")

    def _on_navigation(self, message: NavigationTaskStatus) -> None:
        self._navigation = message
        self._mark("navigation")

    def _on_traversal(self, message: TraversalStatus) -> None:
        self._traversal = message
        self._mark("traversal_status")

    def _on_filter(self, message: FilterState) -> None:
        self._filter = message
        self._mark("filter")

    def _on_yolo(self, message: ObstacleDetection) -> None:
        self._yolo = message
        self._mark("piper_yolo_result")

    def _on_robot_marker(self, message: Marker) -> None:
        self._robot_marker = message
        self._mark("robot_marker")

    def _fresh(self, key: str) -> bool:
        return time.monotonic() - self._seen.get(key, float("-inf")) <= float(
            self.get_parameter("freshness").value
        )

    def _sensor(self, key: str, *, latched: bool = False) -> str:
        available = key in self._seen if latched else self._fresh(key)
        return "OK" if available else "WAIT"

    def _ready_value(self, key: str) -> str:
        if key not in self._ready:
            return "WAIT"
        return "READY" if self._ready[key] else "NOT_READY"

    @staticmethod
    def _twist(message: Twist | None) -> str:
        if message is None:
            return "n/a"
        return f"v={message.linear.x:+.3f},w={message.angular.z:+.3f}"

    @staticmethod
    def _moving(message: Twist | None) -> bool:
        return message is not None and (
            abs(float(message.linear.x)) > 1e-4
            or abs(float(message.angular.z)) > 1e-4
        )

    def _control_verdict(self) -> str:
        raw = self._commands.get("raw")
        gated = self._commands.get("gated")
        selected = self._commands.get("selected")
        if self._path is None:
            return "WAITING_FOR_PATH"
        if not self._fresh("cmd_raw"):
            return "NEUPAN_NOT_PUBLISHING"
        if self._moving(raw) and not self._moving(gated):
            return "BLOCKED_BY_TRAVERSAL_GATE"
        if self._moving(gated) and not self._moving(selected):
            return "BLOCKED_BY_CMD_MUX"
        if self._moving(selected):
            return "MOTION_COMMAND_REACHES_ISAAC"
        return "NEUPAN_COMMANDS_ZERO"

    def _robot_pose(self) -> tuple[str, bool, tuple[float, float, float] | None]:
        try:
            transform = self._tf.lookup_transform(
                "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )
        except TransformException:
            return "WAIT(TF_UNAVAILABLE)", False, None
        value = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        pose = float(value.x), float(value.y), float(yaw)
        return f"x={value.x:+.3f},y={value.y:+.3f},yaw={yaw:+.3f}", True, pose

    def _pose_check(self, tf_pose) -> tuple[str, bool]:
        marker = self._robot_marker
        if tf_pose is None or marker is None:
            return "WAITING_FOR_TF_OR_MARKER", False
        if marker.header.frame_id.lstrip("/") != "map":
            return f"FRAME_MISMATCH(marker={marker.header.frame_id or '-'})", False
        translation, yaw = pose_alignment(tf_pose, marker)
        aligned = translation <= 0.10 and yaw <= 0.10
        verdict = "MATCH" if aligned else "MISMATCH"
        return f"delta_xy={translation:.3f}m delta_yaw={yaw:.3f}rad {verdict}", aligned

    def _report(self) -> None:
        pose, tf_ready, tf_pose = self._robot_pose()
        pose_check, marker_aligned = self._pose_check(tf_pose)
        action_names = {name for name, _ in get_action_names_and_types(self)}
        piper_action = "/piper/probe_pushability" in action_names
        piper_publishers = self.count_publishers("/isaac_joint_command")
        piper_subscribers = self.count_subscribers("/isaac_joint_command")
        piper_ready = (
            piper_action and self._fresh("piper_joint_state")
            and piper_publishers == 1 and piper_subscribers >= 1
            and self._ready.get("piper_yolo", False)
        )
        infrastructure = (
            self._fresh("scan_raw") and self._fresh("scan") and self._fresh("rgb")
            and "map" in self._seen and tf_ready and marker_aligned
            and all(self._ready.get(key, False) for key in ("dino", "llmdecision", "traversal"))
            and piper_ready
        )
        modules = all(self._ready.get(key, False) for key in ("module3", "module4"))
        navigation = self._navigation
        traversal = self._traversal
        filter_state = self._filter
        lines = [
            f"[OVERALL] {'READY' if infrastructure and modules else 'WAITING'}",
            "[SENSORS] "
            f"map={self._sensor('map', latched=True)} "
            f"scan_raw={self._sensor('scan_raw')} scan={self._sensor('scan')} "
            f"rgb={self._sensor('rgb')} semantic={self._sensor('semantic')} "
            f"labels={self._sensor('semantic_labels', latched=True)}",
            f"[TF] map->base_link {pose}",
            f"[POSE CHECK] /robot_marker vs TF {pose_check}",
            "[PROVIDERS] "
            f"DINO={self._ready_value('dino')} "
            f"llmdecision={self._ready_value('llmdecision')} "
            f"traversal={self._ready_value('traversal')}",
            "[PIPER] "
            f"action={'READY' if piper_action else 'WAIT'} "
            f"yolo={self._ready_value('piper_yolo')} "
            f"yolo_result={self._yolo_value()} "
            f"joint_state={self._sensor('piper_joint_state')} "
            f"joint_command_publishers={piper_publishers} "
            f"joint_command_subscribers={piper_subscribers} "
            f"verdict={'READY' if piper_ready else 'TOPOLOGY_NOT_READY'}",
            "[MODULES] "
            f"Module3={self._ready_value('module3')} Module4={self._ready_value('module4')}",
            "[NAV] " + (
                f"task={navigation.task_id or '-'} state={navigation.state or '-'} "
                f"terminal={navigation.terminal} error={navigation.failure_code or '-'}"
                if navigation is not None else "waiting for first task"
            ),
            "[TRAVERSAL] " + (
                f"state={traversal.state} paused={traversal.navigation_paused} "
                f"authorized={traversal.filter_authorized} error={traversal.error_code or '-'} "
                f"detail={traversal.message or '-'}"
                if traversal is not None else "idle; no status received"
            ),
            "[TRAVERSAL_SCORES] " + (
                f"request={traversal.request_id or '-'} "
                f"dino={traversal.dino_probability:.3f} "
                f"llm={traversal.pushability_probability:.3f} "
                f"arm={traversal.mechanical_probability:.3f} "
                f"fusion={traversal.fusion_probability:.3f} "
                f"clearance={traversal.approach_clearance_m:.3f} "
                f"arm_returned={traversal.arm_returned} "
                f"elapsed_ms={traversal.elapsed_ms:.0f}"
                if traversal is not None else "waiting"
            ),
            "[FILTER] " + (
                f"active={filter_state.active} removed={filter_state.removed_points} "
                f"error={filter_state.error_code or '-'}"
                if filter_state is not None else "waiting"
            ),
            "[CONTROL] "
            f"raw({self._twist(self._commands.get('raw'))}) "
            f"gated({self._twist(self._commands.get('gated'))}) "
            f"cmd_vel({self._twist(self._commands.get('selected'))}) "
            f"verdict={self._control_verdict()}",
            "[PATH] " + (
                f"points={len(self._path.poses)} frame={self._path.header.frame_id or '-'}"
                if self._path is not None else "waiting for /neupan_initial_path"
            ),
        ]
        self.get_logger().info("\n" + "\n".join(lines))

    def _yolo_value(self) -> str:
        if self._yolo is None:
            return "WAIT"
        names = {
            ObstacleDetection.UNKNOWN: "UNKNOWN",
            ObstacleDetection.WALL: "WALL",
            ObstacleDetection.CURTAIN: "CURTAIN",
        }
        return (
            f"{names.get(int(self._yolo.classification), 'INVALID')}"
            f"({float(self._yolo.confidence):.2f},"
            f"backend_ok={bool(self._yolo.backend_ok)})"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TraversalSystemDiagnostics()
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
