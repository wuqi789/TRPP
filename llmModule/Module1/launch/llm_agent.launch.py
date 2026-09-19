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
                [FindPackageShare("llm_module1"), "config", "config.yaml"]
            ),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="llm_module1",
            executable="llm_agent_node",
            name="llm_agent_node",
            output="screen",
            parameters=[{
                "config_path": LaunchConfiguration("config_path"),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }],
        )
    ])
