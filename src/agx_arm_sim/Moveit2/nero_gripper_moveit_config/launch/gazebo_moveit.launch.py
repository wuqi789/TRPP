from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_name = "nero_gripper_moveit_config"
    package_share = Path(get_package_share_directory(package_name))
    agx_description_share = Path(get_package_share_directory("agx_arm_description"))
    gazebo_share = Path(get_package_share_directory("gazebo_ros"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    verbose = LaunchConfiguration("verbose")

    moveit_config = (
        MoveItConfigsBuilder("nero", package_name=package_name)
        .robot_description(file_path="config/nero.gazebo.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / "launch" / "gazebo.launch.py")),
        launch_arguments={
            "world": world,
            "gui": gui,
            "verbose": verbose,
        }.items(),
    )

    disable_gazebo_model_database = SetEnvironmentVariable(
        name="GAZEBO_MODEL_DATABASE_URI",
        value="",
    )
    gazebo_model_path = AppendEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=str(agx_description_share.parent),
    )
    gazebo_resource_path = AppendEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH",
        value=str(agx_description_share.parent),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            "nero",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    start_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
                gripper_controller_spawner,
            ],
        )
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "allow_trajectory_execution": True,
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        condition=IfCondition(use_rviz),
        arguments=["-d", str(package_share / "config" / "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the MoveIt plugin.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the Gazebo client.",
            ),
            DeclareLaunchArgument(
                "verbose",
                default_value="false",
                description="Enable verbose Gazebo server logging.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=str(package_share / "worlds" / "nero.world"),
                description="Absolute path to the Gazebo world file.",
            ),
            disable_gazebo_model_database,
            gazebo_model_path,
            gazebo_resource_path,
            gazebo,
            robot_state_publisher,
            spawn_robot,
            start_controllers_after_spawn,
            move_group,
            rviz,
        ]
    )
