from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def package_launch(package, filename, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", filename])
        ),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    semantic_map_path = LaunchConfiguration("semantic_map_path")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "llm_config_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("llm_module1"), "config", "config.yaml"
            ]),
        ),
        DeclareLaunchArgument(
            "semantic_map_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("llm_module2"),
                "maps",
                "scout_kujiale_0021.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "verification_config_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("llm_module3"), "config", "verification.yaml"
            ]),
        ),
        DeclareLaunchArgument(
            "planner_config_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("llm_module3"),
                "config",
                "nav2_validation_astar.yaml",
            ]),
        ),
        package_launch(
            "llm_module1",
            "llm_agent.launch.py",
            {
                "use_sim_time": use_sim_time,
                "config_path": LaunchConfiguration("llm_config_path"),
            },
        ),
        package_launch(
            "llm_module2",
            "semantic_graph.launch.py",
            {"use_sim_time": use_sim_time, "map_path": semantic_map_path},
        ),
        package_launch(
            "llm_module3",
            "validation_planner.launch.py",
            {
                "use_sim_time": use_sim_time,
                "planner_config": LaunchConfiguration("planner_config_path"),
            },
        ),
        package_launch(
            "llm_module3",
            "verification.launch.py",
            {
                "use_sim_time": use_sim_time,
                "config_path": LaunchConfiguration("verification_config_path"),
                "semantic_map_path": semantic_map_path,
                "geometry_check_enabled": "true",
            },
        ),
    ])
