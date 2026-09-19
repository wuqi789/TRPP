from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import rclpy

from neupan_ros2.cmd_vel_mux import CmdVelMux


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_initial_path_latches_neupan_control():
    rclpy.init()
    node = CmdVelMux()
    velocity = Recorder()
    phase = Recorder()
    node._publisher = velocity
    node._mapping_active_publisher = phase
    try:
        teleop = Twist()
        teleop.linear.x = 0.2
        node._on_teleop_command(teleop)
        assert velocity.messages[-1].linear.x == 0.2

        node._on_initial_path(Path())
        assert not phase.messages[-1].data
        assert velocity.messages[-1].linear.x == 0.0

        message_count = len(velocity.messages)
        node._on_teleop_command(teleop)
        assert len(velocity.messages) == message_count

        neupan = Twist()
        neupan.linear.x = 0.1
        node._on_neupan_command(neupan)
        assert velocity.messages[-1].linear.x == 0.1
    finally:
        node.destroy_node()
        rclpy.shutdown()
