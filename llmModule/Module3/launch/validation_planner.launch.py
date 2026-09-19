from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    planner_config = LaunchConfiguration("planner_config")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "planner_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("llm_module3"),
                "config",
                "nav2_validation_astar.yaml",
            ]),
        ),
        Node(
            package="llm_module3",
            executable="validation_map_normalizer",
            name="validation_map_normalizer",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "source_topic": "/map",
                "output_topic": "/semantic_validation/map",
                "output_frame": "odom",
            }],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            namespace="semantic_validation",
            name="planner_server",
            output="screen",
            parameters=[
                planner_config,
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            namespace="semantic_validation",
            name="lifecycle_manager_validation",
            output="screen",
            parameters=[
                planner_config,
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "autostart": True,
                    "node_names": ["planner_server"],
                },
            ],
        ),
    ])
