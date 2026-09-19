"""Single production launch for Isaac, Module1-4, real providers, NeuPAN, and gates."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution,
    PythonExpression,
)
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
    obstacle = LaunchConfiguration("acceptance_obstacle")
    use_rviz = LaunchConfiguration("use_rviz")
    headless = LaunchConfiguration("headless")
    core_only = LaunchConfiguration("core_only")
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
        DeclareLaunchArgument("acceptance_obstacle", default_value="curtain"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("core_only", default_value="false"),
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
                "traversal_acceptance": "true",
                "acceptance_obstacle": obstacle,
                "scan_topic": "/scan_raw",
                "neupan_cmd_vel_topic": "/neupan_cmd_vel_raw",
                "use_semantic_mapping": "true",
                "use_diagnostics": "false",
                "use_rviz": use_rviz,
                # Keep the acceptance path centered through the 0.74 m doorway.
                # The former diagonal staging reached the curtain correctly but
                # converged on the real wall immediately behind its left edge.
                "start_x": "8.2", "start_y": "-3.2", "start_yaw": "0.0",
            }.items(),
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "launch", "llm_module4",
                "scout_semantic_navigation.launch.py",
                "use_sim_time:=true", "use_pushability_mapping:=false",
            ],
            name="semantic_navigation_launch",
            output="screen",
            additional_env={
                "PYTHONPATH": _semantic_navigation_pythonpath(),
                "PATH": _semantic_navigation_path(),
                "VIRTUAL_ENV": "",
                "PYTHONNOUSERSITE": "1",
            },
            condition=UnlessCondition(core_only),
        ),
        Node(
            package="obstacle_traversal", executable="core_acceptance_driver",
            name="core_acceptance_driver", output="screen",
            parameters=[{
                "scenario": obstacle,
                "goal_x": ParameterValue(
                    PythonExpression([
                        "4.20 if '", obstacle, "' == 'curtain' else 2.80"
                    ]),
                    value_type=float,
                ),
                "goal_y": -0.004,
                "goal_frame": "map",
                "use_sim_time": True,
            }],
            condition=IfCondition(core_only),
        ),
        Node(
            package="groundingdino_vlm", executable="verify_target_action_server",
            name="groundingdino_verify_target_server", output="screen",
            parameters=[{"config_path": dino_config, "use_sim_time": True}],
        ),
        Node(
            package="llmdecision", executable="pushability_action_server",
            name="llmdecision_pushability_server", output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="obstacle_traversal", executable="scan_filter",
            name="obstacle_traversal_scan_filter", output="screen",
            parameters=[traversal_config, {"use_sim_time": True}],
        ),
        Node(
            package="obstacle_traversal", executable="velocity_gate",
            name="obstacle_traversal_velocity_gate", output="screen",
            parameters=[traversal_config],
        ),
        Node(
            package="obstacle_traversal", executable="approach_controller",
            name="obstacle_approach_controller", output="screen",
            parameters=[traversal_config, {"use_sim_time": True}],
        ),
        Node(
            package="piper_probe", executable="yolo_obstacle_detector",
            name="piper_yolo_obstacle_detector", output="screen",
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
            package="piper_probe", executable="probe_server",
            name="piper_probe_server", output="screen",
            parameters=[piper_config, {"use_sim_time": True}],
        ),
        Node(
            package="obstacle_traversal", executable="traversal_manager",
            name="obstacle_traversal_manager", output="screen",
            parameters=[traversal_config, {"use_sim_time": True}],
        ),
        Node(
            package="obstacle_traversal", executable="system_diagnostics",
            name="traversal_system_diagnostics", output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ])
