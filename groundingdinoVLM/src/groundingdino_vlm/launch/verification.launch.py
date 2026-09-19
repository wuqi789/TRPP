"""Launch the public target verifier with deterministic mock providers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("groundingdino_vlm"), "config", "mock.yaml"]
            ),
        ),
        DeclareLaunchArgument("image_topic", default_value="/isaac/color_image_raw"),
        DeclareLaunchArgument(
            "target_topic", default_value="/groundingdino_vlm/target_label"
        ),
        DeclareLaunchArgument("use_compressed", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="groundingdino_vlm",
            executable="target_verifier",
            name="groundingdino_vlm_verifier",
            output="screen",
            parameters=[{
                "config_path": LaunchConfiguration("config_path"),
                "image_topic": LaunchConfiguration("image_topic"),
                "target_topic": LaunchConfiguration("target_topic"),
                "use_compressed": ParameterValue(
                    LaunchConfiguration("use_compressed"), value_type=bool
                ),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }],
        ),
    ])
