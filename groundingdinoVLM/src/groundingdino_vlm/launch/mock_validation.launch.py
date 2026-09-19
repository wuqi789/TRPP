"""Launch the verifier without CUDA, model weights, or cloud credentials."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="groundingdino_vlm",
            executable="target_verifier",
            name="groundingdino_vlm_verifier",
            output="screen",
            parameters=[{
                "config_path": PathJoinSubstitution(
                    [FindPackageShare("groundingdino_vlm"), "config", "mock.yaml"]
                ),
                "use_sim_time": False,
            }],
        )
    ])
