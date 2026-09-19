"""Public Isaac living-room demo: local NeuPAN plus deterministic mock providers."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def _workspace_default() -> str:
    configured = os.getenv("SCOUT_WORKSPACE", "").strip()
    if configured:
        return str(Path(configured).expanduser().resolve())
    for candidate in Path(__file__).resolve().parents:
        if (candidate / "groundingdinoVLM").is_dir() and (candidate / "src").is_dir():
            return str(candidate)
    raise RuntimeError("Set SCOUT_WORKSPACE before launching the public demo")


def generate_launch_description():
    workspace = LaunchConfiguration("workspace_dir")
    isaac_python = LaunchConfiguration("isaac_python")
    return LaunchDescription([
        DeclareLaunchArgument("workspace_dir", default_value=_workspace_default()),
        DeclareLaunchArgument("isaac_python", default_value=EnvironmentVariable("ISAACSIM_PYTHON_EXE", default_value="")),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("start_x", default_value="8.2"),
        DeclareLaunchArgument("start_y", default_value="-3.2"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("neupan_ros2"), "launch", "isaac_sim.launch.py"
            ])),
            launch_arguments={
                "workspace_dir": workspace,
                "isaac_python": isaac_python,
                "full_room": "true",
                "headless": LaunchConfiguration("headless"),
                "traversal_acceptance": "true",
                "acceptance_obstacle": "curtain",
                "scan_topic": "/scan_raw",
                "neupan_cmd_vel_topic": "/neupan_cmd_vel_raw",
                 "use_semantic_mapping": "false",
                 "use_diagnostics": "false",
                "use_rviz": LaunchConfiguration("use_rviz"),
                "start_x": LaunchConfiguration("start_x"),
                "start_y": LaunchConfiguration("start_y"),
                 "start_yaw": LaunchConfiguration("start_yaw"),
            }.items(),
        ),
        Node(package="scout_public_mock", executable="mock_verify_target", name="groundingdino_verify_target_server", output="screen"),
        Node(package="scout_public_mock", executable="mock_assess_pushability", name="llmdecision_pushability_server", output="screen"),
        Node(package="scout_public_mock", executable="mock_probe_pushability", name="piper_probe_server", output="screen"),
        Node(package="scout_public_mock", executable="mock_approach_obstacle", name="mock_obstacle_approach_controller", output="screen"),
        Node(package="scout_public_mock", executable="mock_yolo_detector", name="piper_yolo_obstacle_detector", output="screen"),
        Node(package="scout_public_mock", executable="mock_piper_arm", name="mock_piper_arm", output="screen"),
        Node(package="scout_public_mock", executable="fixed_route_driver", name="fixed_route_driver", output="screen",
             parameters=[{
                 "goal_x": 4.20, "goal_y": -0.004,
                 "obstacle_x": 3.29, "obstacle_y": -0.004,
                 "goal_frame": "map", "require_authorization": False,
             }]),
        Node(package="obstacle_traversal", executable="scan_filter", name="obstacle_traversal_scan_filter", output="screen",
             parameters=[PathJoinSubstitution([FindPackageShare("obstacle_traversal"), "config", "isaac.yaml"]), {
                 "use_sim_time": True,
                 "maximum_authorization_duration_s": 120.0,
                 "initial_track_grace_s": 45.0,
             }]),
        Node(package="obstacle_traversal", executable="velocity_gate", name="obstacle_traversal_velocity_gate", output="screen",
             parameters=[PathJoinSubstitution([FindPackageShare("obstacle_traversal"), "config", "isaac.yaml"]), {
                 "maximum_traversal_linear_speed": 0.25,
             }]),
    ])
