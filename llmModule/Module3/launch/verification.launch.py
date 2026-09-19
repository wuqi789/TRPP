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
                [FindPackageShare("llm_module3"), "config", "verification.yaml"]
            ),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "geometry_check_enabled",
            default_value="true",
            description=(
                "Enable startup-only costmap occupancy validation for goal/via poses"
            ),
        ),
        DeclareLaunchArgument(
            "semantic_map_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("llm_module2"), "maps", "scout_kujiale_0021.yaml"]
            ),
        ),
        Node(
            package="llm_module3",
            executable="verification_node",
            name="verification_node",
            output="screen",
            parameters=[{
                "config_path": LaunchConfiguration("config_path"),
                "semantic_map_path": LaunchConfiguration("semantic_map_path"),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "geometry_check_enabled": ParameterValue(
                    LaunchConfiguration("geometry_check_enabled"),
                    value_type=bool,
                ),
            }],
        ),
    ])
