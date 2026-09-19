"""Select keyboard or NeuPAN velocity commands."""

import time

from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class CmdVelMux(Node):
    """Use keyboard before the first goal and NeuPAN after it."""

    def __init__(self) -> None:
        super().__init__('cmd_vel_mux')
        self.declare_parameter('teleop_timeout', 0.4)
        self._teleop_timeout = float(
            self.get_parameter('teleop_timeout').value
        )
        self._teleop_until = 0.0
        self._teleop_active = False
        self._autonomous = False
        self._publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        phase_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._mapping_active_publisher = self.create_publisher(
            Bool, '/semantic_mapping/active', phase_qos
        )
        self.create_subscription(
            Twist, '/neupan_cmd_vel', self._on_neupan_command, 10
        )
        self.create_subscription(
            Twist, '/teleop_cmd_vel', self._on_teleop_command, 10
        )
        self.create_subscription(
            Bool, '/teleop_active', self._on_teleop_active, 10
        )
        self.create_subscription(
            PointStamped, '/clicked_point', self._on_goal, 10
        )
        self.create_subscription(
            Path, '/neupan_initial_path', self._on_initial_path, 10
        )
        self.create_timer(0.05, self._stop_expired_teleop)
        self._mapping_active_publisher.publish(Bool(data=True))
        self.get_logger().info(
            'Phase 1 active: keyboard control and semantic mapping'
        )

    def _on_neupan_command(self, message: Twist) -> None:
        if self._autonomous:
            self._publisher.publish(message)

    def _on_teleop_command(self, message: Twist) -> None:
        if self._autonomous:
            return
        if self._is_zero(message):
            if self._teleop_active:
                self._teleop_until = time.monotonic() + self._teleop_timeout
                self._publisher.publish(message)
            return

        self._teleop_active = True
        self._teleop_until = time.monotonic() + self._teleop_timeout
        self._publisher.publish(message)

    def _on_teleop_active(self, message: Bool) -> None:
        if self._autonomous:
            return
        if message.data:
            self._teleop_active = True
            self._teleop_until = time.monotonic() + self._teleop_timeout
        elif self._teleop_active:
            self._teleop_active = False
            self._teleop_until = 0.0
            self._publisher.publish(Twist())

    def _stop_expired_teleop(self) -> None:
        if (
            not self._autonomous
            and self._teleop_active
            and time.monotonic() >= self._teleop_until
        ):
            self._teleop_active = False
            self._publisher.publish(Twist())

    def _on_goal(self, _: PointStamped) -> None:
        self._enter_autonomous()

    def _on_initial_path(self, _: Path) -> None:
        self._enter_autonomous()

    def _enter_autonomous(self) -> None:
        if self._autonomous:
            return
        self._autonomous = True
        self._teleop_active = False
        self._teleop_until = 0.0
        self._publisher.publish(Twist())
        self._mapping_active_publisher.publish(Bool(data=False))
        self.get_logger().info(
            'Phase 2 active: semantic map frozen, NeuPAN owns cmd_vel'
        )

    @staticmethod
    def _is_zero(message: Twist) -> bool:
        values = (
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        )
        return all(abs(value) < 1e-6 for value in values)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelMux()
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
