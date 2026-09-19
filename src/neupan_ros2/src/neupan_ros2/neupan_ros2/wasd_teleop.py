"""WASD terminal control for a differential-drive robot."""

import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from std_msgs.msg import Bool


KEY_COMMANDS = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    ' ': (0.0, 0.0),
}

KEY_LABELS = {
    'w': 'FORWARD',
    's': 'BACKWARD',
    'a': 'TURN LEFT',
    'd': 'TURN RIGHT',
    ' ': 'STOP (manual control held)',
}


def make_twist(linear: float, angular: float) -> Twist:
    message = Twist()
    message.linear.x = linear
    message.angular.z = angular
    return message


def main(args=None) -> None:
    if not sys.stdin.isatty():
        raise RuntimeError('WASD teleop must run in an interactive terminal')

    rclpy.init(args=args)
    node = rclpy.create_node('wasd_teleop')
    linear_speed = float(node.declare_parameter('linear_speed', 0.25).value)
    angular_speed = float(node.declare_parameter('angular_speed', 0.8).value)
    publisher = node.create_publisher(Twist, '/teleop_cmd_vel', 10)
    active_publisher = node.create_publisher(Bool, '/teleop_active', 10)
    terminal_settings = termios.tcgetattr(sys.stdin)
    command = None

    print(
        'W/S: forward/backward  A/D: turn  Space: stop/hold  '
        'X: stop/release  Q: quit\n'
        f'linear={linear_speed:.2f} m/s  angular={angular_speed:.2f} rad/s'
    )
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], 0.05)
            if readable:
                key = sys.stdin.read(1).lower()
                if key in ('q', '\x03'):
                    break
                if key == 'x':
                    command = None
                    print('\nControl: STOP / RELEASE TO NEUPAN', flush=True)
                if key in KEY_COMMANDS:
                    command = KEY_COMMANDS[key]
                    print(f'\nControl: {KEY_LABELS[key]}', flush=True)

            active = command is not None
            active_publisher.publish(Bool(data=active))
            if active:
                publisher.publish(make_twist(
                    command[0] * linear_speed,
                    command[1] * angular_speed,
                ))
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            publisher.publish(Twist())
            active_publisher.publish(Bool(data=False))
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.02)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
