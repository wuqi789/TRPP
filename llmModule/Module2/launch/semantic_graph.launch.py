from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "map_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("llm_module2"), "maps", "scout_kujiale_0021.yaml"]
            ),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="llm_module2",
            executable="semantic_graph_node",
            name="semantic_graph_node",
            output="screen",
            parameters=[{
                "map_path": LaunchConfiguration("map_path"),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }],
        ),
    ])
