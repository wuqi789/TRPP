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
            default_value=PathJoinSubstitution([
                FindPackageShare("llm_semantic_affordance"),
                "config",
                "affordance.yaml",
            ]),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="llm_semantic_affordance",
            executable="pushability_mapper",
            name="pushability_mapper",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "config_path": LaunchConfiguration("config_path"),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }],
        ),
    ])
