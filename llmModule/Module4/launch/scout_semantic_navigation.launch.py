"""Launch the Scout Module1-4 semantic navigation chain."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def package_launch(package: str, filename: str, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", filename])
        ),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_pushability_mapping = LaunchConfiguration("use_pushability_mapping")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_pushability_mapping", default_value="false"),
        DeclareLaunchArgument(
            "module3_verification_config_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("llm_module3"), "config", "verification.yaml"
            ]),
        ),
        DeclareLaunchArgument(
            "module4_config_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("llm_module4"), "config", "navigation.yaml"
            ]),
        ),
        DeclareLaunchArgument(
            "affordance_config_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("llm_semantic_affordance"),
                "config",
                "affordance.yaml",
            ]),
        ),
        package_launch(
            "llm_module3",
            "module1_to_module3.launch.py",
            {
                "use_sim_time": use_sim_time,
                "verification_config_path": LaunchConfiguration(
                    "module3_verification_config_path"
                ),
            },
        ),
        package_launch(
            "llm_semantic_affordance",
            "affordance.launch.py",
            {
                "use_sim_time": use_sim_time,
                "config_path": LaunchConfiguration("affordance_config_path"),
            },
            condition=IfCondition(use_pushability_mapping),
        ),
        package_launch(
            "llm_module4",
            "navigation.launch.py",
            {
                "use_sim_time": use_sim_time,
                "config_path": LaunchConfiguration("module4_config_path"),
            },
        ),
    ])
