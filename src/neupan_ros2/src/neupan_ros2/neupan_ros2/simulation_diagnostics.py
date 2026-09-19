"""Periodic diagnostics for the Isaac Sim and NeuPAN data path."""

import math
import time

from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def quaternion_to_yaw(orientation) -> float:
    sin_yaw = 2.0 * (
        orientation.w * orientation.z
        + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y
        + orientation.z * orientation.z
    )
    return math.atan2(sin_yaw, cos_yaw)


class SimulationDiagnostics(Node):
    """Report commands, pose, paths, and nearest scan obstacles."""

    def __init__(self) -> None:
        super().__init__('simulation_diagnostics')
        report_rate = float(
            self.declare_parameter('report_rate', 1.0).value
        )
        self._nearest_count = int(
            self.declare_parameter('nearest_points', 12).value
        )
        self._lidar_offset_x = float(
            self.declare_parameter('lidar_offset_x', 0.21).value
        )
        if report_rate <= 0.0 or self._nearest_count < 1:
            raise ValueError('report_rate and nearest_points must be positive')

        self._odom = None
        self._selected_command = None
        self._scan = None
        self._initial_path = None
        self._plan = None
        self._received_at = {}
        self._waiting_logged = False

        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(
            Twist, '/cmd_vel', self._on_selected_command, 10
        )
        scan_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, scan_qos
        )
        self.create_subscription(
            Path, '/neupan_initial_path', self._on_initial_path, 10
        )
        self.create_subscription(Path, '/neupan_plan', self._on_plan, 10)
        self.create_subscription(
            PointStamped, '/clicked_point', self._on_clicked_point, 10
        )
        self.create_timer(1.0 / report_rate, self._report)
        self.get_logger().info(
            f'Detailed diagnostics enabled: rate={report_rate:.1f} Hz, '
            f'nearest_points={self._nearest_count}'
        )

    def _mark_received(self, topic: str) -> None:
        self._received_at[topic] = time.monotonic()

    def _on_odom(self, message: Odometry) -> None:
        self._odom = message
        self._mark_received('odom')

    def _on_selected_command(self, message: Twist) -> None:
        self._selected_command = message
        self._mark_received('selected')

    def _on_scan(self, message: LaserScan) -> None:
        self._scan = message
        self._mark_received('scan')

    def _on_initial_path(self, message: Path) -> None:
        self._initial_path = message
        self._mark_received('initial_path')
        self.get_logger().info(
            f'[INITIAL_PATH] received {self._format_path(message)}'
        )

    def _on_plan(self, message: Path) -> None:
        self._plan = message
        self._mark_received('plan')

    def _on_clicked_point(self, message: PointStamped) -> None:
        self.get_logger().info(
            f'[GOAL] source=publish_point '
            f'frame={message.header.frame_id or "<empty>"} '
            f'x={message.point.x:.3f} y={message.point.y:.3f}'
        )

    def _age(self, topic: str) -> str:
        received_at = self._received_at.get(topic)
        if received_at is None:
            return 'n/a'
        return f'{time.monotonic() - received_at:.2f}s'

    @staticmethod
    def _format_twist(message: Twist) -> str:
        if message is None:
            return 'n/a'
        return f'v={message.linear.x:.3f},w={message.angular.z:.3f}'

    def _control_source(self) -> str:
        if self._age_seconds('selected') < 0.5:
            return 'neupan_direct'
        return 'idle'

    def _age_seconds(self, topic: str) -> float:
        received_at = self._received_at.get(topic)
        if received_at is None:
            return math.inf
        return time.monotonic() - received_at

    @staticmethod
    def _format_path(path: Path) -> str:
        if path is None:
            return 'n/a'
        if not path.poses:
            return 'points=0'
        first = path.poses[0].pose.position
        last = path.poses[-1].pose.position
        return (
            f'points={len(path.poses)},start=({first.x:.2f},{first.y:.2f}),'
            f'end=({last.x:.2f},{last.y:.2f})'
        )

    def _scan_summary(self) -> str:
        if self._scan is None:
            return 'waiting for /scan'

        scan = self._scan
        valid = []
        lower = max(float(scan.range_min), 0.0)
        upper = float(scan.range_max)
        for index, distance in enumerate(scan.ranges):
            if math.isfinite(distance) and lower <= distance <= upper:
                valid.append((float(distance), index))

        if not valid:
            return (
                f'frame={scan.header.frame_id},valid=0/{len(scan.ranges)},'
                f'age={self._age("scan")}'
            )

        distances = [item[0] for item in valid]
        nearest = sorted(valid)[:self._nearest_count]
        points = []
        pose = None if self._odom is None else self._odom.pose.pose
        yaw = 0.0 if pose is None else quaternion_to_yaw(pose.orientation)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        for distance, index in nearest:
            angle = scan.angle_min + index * scan.angle_increment
            lidar_x = distance * math.cos(angle)
            lidar_y = distance * math.sin(angle)
            if pose is None:
                points.append(
                    f'(r={distance:.2f},a={angle:.2f})'
                )
                continue
            base_x = self._lidar_offset_x + lidar_x
            map_x = pose.position.x + cosine * base_x - sine * lidar_y
            map_y = pose.position.y + sine * base_x + cosine * lidar_y
            points.append(
                f'(map={map_x:.2f},{map_y:.2f};'
                f'r={distance:.2f};a={angle:.2f})'
            )

        return (
            f'frame={scan.header.frame_id},valid={len(valid)}/'
            f'{len(scan.ranges)},min={min(distances):.3f},'
            f'mean={sum(distances) / len(distances):.3f},'
            f'max={max(distances):.3f},age={self._age("scan")},'
            f'nearest=[{" ".join(points)}]'
        )

    def _report(self) -> None:
        if self._odom is None and self._scan is None:
            if not self._waiting_logged:
                self.get_logger().info(
                    '[DIAG] waiting for /odom and /scan'
                )
                self._waiting_logged = True
            return

        self._waiting_logged = False
        if self._odom is None:
            self.get_logger().info('[STATE] waiting for /odom')
        else:
            pose = self._odom.pose.pose
            velocity = self._odom.twist.twist
            yaw = quaternion_to_yaw(pose.orientation)
            self.get_logger().info(
                f'[STATE] pose_map=(x={pose.position.x:.3f},'
                f'y={pose.position.y:.3f},yaw={yaw:.3f}) '
                f'odom_velocity=({self._format_twist(velocity)}) '
                f'age={self._age("odom")}'
            )

        self.get_logger().info(
            f'[COMMAND] source={self._control_source()} '
            f'selected=({self._format_twist(self._selected_command)}) '
            f'age={self._age("selected")}'
        )
        self.get_logger().info(f'[SCAN] {self._scan_summary()}')
        self.get_logger().info(
            f'[PATH] initial_cached={self._format_path(self._initial_path)}; '
            f'optimized={self._format_path(self._plan)}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimulationDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
