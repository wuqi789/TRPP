"""Terminal-1 launch: Isaac, RViz, NeuPAN, semantics, and real traversal providers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from obstacle_traversal.preflight import assert_preflight, default_workspace


def _preflight(context):
    assert_preflight(
        LaunchConfiguration("workspace_dir").perform(context),
        LaunchConfiguration("isaac_python").perform(context),
    )
    return []


def generate_launch_description():
    workspace = LaunchConfiguration("workspace_dir")
    isaac_python = LaunchConfiguration("isaac_python")
    use_rviz = LaunchConfiguration("use_rviz")
    headless = LaunchConfiguration("headless")
    dino_config = PathJoinSubstitution([
        FindPackageShare("groundingdino_vlm"), "config", "external.yaml"
    ])
    traversal_config = PathJoinSubstitution([
        FindPackageShare("obstacle_traversal"), "config", "isaac.yaml"
    ])
    piper_config = PathJoinSubstitution([
        FindPackageShare("piper_probe"), "config", "isaac.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "workspace_dir", default_value=default_workspace()
        ),
        DeclareLaunchArgument(
            "isaac_python",
            default_value=EnvironmentVariable(
                "ISAACSIM_PYTHON_EXE",
                default_value="",
            ),
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("yolo_inference_hz", default_value="2.0"),
        OpaqueFunction(function=_preflight),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("neupan_ros2"), "launch", "isaac_sim.launch.py"
            ])),
            launch_arguments={
                "workspace_dir": workspace,
                "isaac_python": isaac_python,
                "full_room": "true",
                "headless": headless,
                "traversal_acceptance": "false",
                "scan_topic": "/scan_raw",
                "neupan_cmd_vel_topic": "/neupan_cmd_vel_raw",
                "use_semantic_mapping": "true",
                "use_diagnostics": "false",
                "use_rviz": use_rviz,
                "start_x": "7.7",
                "start_y": "-4.0",
                "start_yaw": "1.0",
            }.items(),
        ),
        Node(
            package="groundingdino_vlm",
            executable="verify_target_action_server",
            name="groundingdino_verify_target_server",
            output="screen",
            parameters=[{"config_path": dino_config, "use_sim_time": True}],
        ),
        Node(
            package="llmdecision",
            executable="pushability_action_server",
            name="llmdecision_pushability_server",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="obstacle_traversal",
            executable="scan_filter",
            name="obstacle_traversal_scan_filter",
            output="screen",
            parameters=[traversal_config, {"use_sim_time": True}],
        ),
        Node(
            package="obstacle_traversal",
            executable="velocity_gate",
            name="obstacle_traversal_velocity_gate",
            output="screen",
            parameters=[traversal_config],
        ),
        Node(
            package="obstacle_traversal",
            executable="approach_controller",
            name="obstacle_approach_controller",
            output="screen",
            parameters=[traversal_config, {"use_sim_time": True}],
        ),
        Node(
            package="piper_probe",
            executable="yolo_obstacle_detector",
            name="piper_yolo_obstacle_detector",
            output="screen",
            parameters=[
                piper_config,
                {
                    "use_sim_time": True,
                    "inference_hz": ParameterValue(
                        LaunchConfiguration("yolo_inference_hz"), value_type=float
                    ),
                },
            ],
        ),
        Node(
            package="piper_probe",
            executable="probe_server",
            name="piper_probe_server",
            output="screen",
            parameters=[piper_config, {"use_sim_time": True}],
        ),
        Node(
            package="obstacle_traversal",
            executable="traversal_manager",
            name="obstacle_traversal_manager",
            output="screen",
            parameters=[traversal_config, {"use_sim_time": True}],
        ),
    ])
