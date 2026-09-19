"""Launch the complete Isaac Sim, NeuPAN, and RViz2 simulation."""

import os
from pathlib import Path
import site

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _is_true(value: str) -> bool:
    return value.lower() in ('1', 'true', 'yes', 'on')


def _create_isaac_actions(context):
    workspace = LaunchConfiguration('workspace_dir').perform(context)
    isaac_python = LaunchConfiguration('isaac_python').perform(context)
    requested_cores = int(
        LaunchConfiguration('cpu_cores').perform(context)
    )
    available_cores = len(os.sched_getaffinity(0))
    cpu_cores = min(max(requested_cores, 1), available_cores)
    script = os.path.join(workspace, 'isaac_sim', 'scout_avoidance.py')
    if not os.path.isfile(script):
        raise RuntimeError(f'Isaac Sim script not found: {script}')
    if not os.path.isfile(isaac_python):
        raise RuntimeError(f'Isaac Sim Python not found: {isaac_python}')

    command = [
        '/usr/bin/nice', '-n', '5',
        '/usr/bin/taskset', '-c', f'0-{cpu_cores - 1}',
        isaac_python, script,
        '--start-x', LaunchConfiguration('start_x').perform(context),
        '--start-y', LaunchConfiguration('start_y').perform(context),
        '--start-yaw', LaunchConfiguration('start_yaw').perform(context),
        '--arm-mount-x', LaunchConfiguration('arm_mount_x').perform(context),
        '--arm-mount-y', LaunchConfiguration('arm_mount_y').perform(context),
        '--arm-mount-z', LaunchConfiguration('arm_mount_z').perform(context),
        '--scan-topic', LaunchConfiguration('scan_topic').perform(context),
    ]
    command.append(
        '--full-room'
        if _is_true(LaunchConfiguration('full_room').perform(context))
        else '--navigation-room'
    )
    if _is_true(LaunchConfiguration('headless').perform(context)):
        command.append('--headless')
    if _is_true(LaunchConfiguration('debug_lidar').perform(context)):
        command.append('--debug-lidar')
    if _is_true(LaunchConfiguration('traversal_acceptance').perform(context)):
        command.extend([
            '--traversal-acceptance',
            '--acceptance-obstacle',
            LaunchConfiguration('acceptance_obstacle').perform(context),
        ])

    neupan_source = os.path.join(workspace, 'src', 'NeuPAN')
    python_path = neupan_source
    if os.environ.get('PYTHONPATH'):
        python_path += os.pathsep + os.environ['PYTHONPATH']

    process = ExecuteProcess(
        cmd=command,
        name='isaac_sim',
        output='screen',
        emulate_tty=True,
        additional_env={'PYTHONPATH': python_path},
    )
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=process,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason='Isaac Sim process exited')
                )
            ],
        )
    )
    return [shutdown_handler, process]


def _create_isaac_local_frame_bridge(_context):
    # Isaac's odometry graph already publishes base_link in a local frame whose
    # origin is the configured start pose. The room map is expressed in that
    # same start-local frame, so applying the inverse launch pose here would
    # transform the robot twice and displace it in RViz and Nav2.
    return [Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_isaac_world',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'odom', '--child-frame-id', 'isaac_world',
        ],
        parameters=[{'use_sim_time': True}],
    )]


def _create_map_to_odom(context):
    """Expose Isaac's start-local odometry as the public ``map`` frame.

    ``scout_avoidance.py`` publishes odometry with its origin at the configured
    start pose.  Applying the launch pose again here would translate the robot
    twice and put the fixed public route outside the robot's local map.
    """
    return [Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': True}],
    )]


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory('neupan_ros2')
    workspace_default = str(Path(share).parents[3])
    robot_dir = os.path.join(share, 'config', 'robots', 'scout')
    robot_config = os.path.join(robot_dir, 'robot.yaml')
    rviz_config = os.path.join(share, 'rviz', 'isaac_sim.rviz')
    use_rviz = LaunchConfiguration('use_rviz')
    use_diagnostics = LaunchConfiguration('use_diagnostics')
    use_semantic_mapping = LaunchConfiguration('use_semantic_mapping')
    semantic_config = os.path.join(share, 'config', 'semantic_mapping.yaml')
    diagnostics_rate = ParameterValue(
        LaunchConfiguration('diagnostics_rate'), value_type=float
    )
    diagnostics_points = ParameterValue(
        LaunchConfiguration('diagnostics_points'), value_type=int
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_diagnostics', default_value='true'),
        DeclareLaunchArgument('use_semantic_mapping', default_value='true'),
        DeclareLaunchArgument('diagnostics_rate', default_value='1.0'),
        DeclareLaunchArgument('diagnostics_points', default_value='12'),
        DeclareLaunchArgument('cpu_cores', default_value='12'),
        DeclareLaunchArgument('full_room', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('debug_lidar', default_value='false'),
        DeclareLaunchArgument('traversal_acceptance', default_value='false'),
        DeclareLaunchArgument('acceptance_obstacle', default_value='curtain'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('neupan_cmd_vel_topic', default_value='/neupan_cmd_vel'),
        DeclareLaunchArgument('start_x', default_value='7.7'),
        DeclareLaunchArgument('start_y', default_value='-4.0'),
        DeclareLaunchArgument('start_yaw', default_value='1.0'),
        DeclareLaunchArgument('arm_mount_x', default_value='-0.05'),
        DeclareLaunchArgument('arm_mount_y', default_value='0.0'),
        DeclareLaunchArgument('arm_mount_z', default_value='0.13'),
        DeclareLaunchArgument(
            'workspace_dir', default_value=workspace_default
        ),
        DeclareLaunchArgument(
            'isaac_python',
            default_value=EnvironmentVariable(
                'ISAACSIM_PYTHON_EXE',
                default_value='',
            ),
        ),
        SetEnvironmentVariable(
            'PYTHONPATH',
            [
                LaunchConfiguration('workspace_dir'),
                '/src/NeuPAN', os.pathsep,
                LaunchConfiguration('workspace_dir'),
                '/.deps/python', os.pathsep,
                EnvironmentVariable('PYTHONPATH', default_value=''),
                os.pathsep, site.getusersitepackages(),
            ],
        ),
        OpaqueFunction(function=_create_map_to_odom),
        OpaqueFunction(function=_create_isaac_local_frame_bridge),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar',
            arguments=[
                '--x', '0.21', '--y', '0', '--z', '0.17',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'lidar_link',
            ],
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='neupan_ros2',
            executable='neupan_node',
            name='neupan_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                robot_config,
                {
                    'robot_config_dir': robot_dir,
                    'use_sim_time': True,
                    'lidar_frame': 'lidar_link',
                    'require_scout_ready': False,
                    'enable_motion': True,
                    'filter_robot_footprint': True,
                    'robot_footprint_filter_padding': 0.04,
                    'scan_topic': '/scan',
                    'cmd_vel_topic': LaunchConfiguration('neupan_cmd_vel_topic'),
                },
            ],
        ),
        Node(
            package='neupan_ros2',
            executable='cmd_vel_mux',
            name='cmd_vel_mux',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='neupan_ros2',
            executable='semantic_mapper',
            name='semantic_mapper',
            output='screen',
            emulate_tty=True,
            parameters=[
                semantic_config,
                {
                    'use_sim_time': True,
                },
            ],
            condition=IfCondition(use_semantic_mapping),
        ),
        Node(
            package='neupan_ros2',
            executable='simulation_diagnostics',
            name='simulation_diagnostics',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'report_rate': diagnostics_rate,
                'nearest_points': diagnostics_points,
                'lidar_offset_x': 0.21,
            }],
            condition=IfCondition(use_diagnostics),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_rviz),
        ),
        OpaqueFunction(function=_create_isaac_actions),
    ])
