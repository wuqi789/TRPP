import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('neupan_ros2')
    livox_share = get_package_share_directory('livox_ros_driver2')
    slam_share = get_package_share_directory('slam_toolbox')

    robot_config_dir = os.path.join(pkg_share, 'config', 'robots', 'scout')
    robot_config = os.path.join(robot_config_dir, 'robot.yaml')
    livox_config = os.path.join(livox_share, 'config', 'MID360s_config.json')
    slam_config = os.path.join(slam_share, 'config', 'mapper_params_online_async.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'neupan_sim.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    use_octomap = LaunchConfiguration('use_octomap')
    enable_motion = LaunchConfiguration('enable_motion')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_octomap', default_value='true'),
        DeclareLaunchArgument(
            'enable_motion',
            default_value='true',
            description='Commands still require healthy Scout CAN-mode status'
        ),

        Node(
            package='scout_base',
            executable='scout_base_node',
            name='scout_base',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'port_name': 'can0',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'odom_topic_name': 'odom',
                'is_scout_mini': True,
                'is_omni_wheel': False,
                'simulated_robot': False,
            }]
        ),
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[{
                'xfer_format': 0,
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'point_filter_min_range': 0.15,
                'output_data_type': 0,
                'frame_id': 'livox_frame',
                'user_config_path': livox_config,
                'cmdline_input_bd_code': 'livox0000000001',
            }]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_livox',
            arguments=[
                '--x', '0.21', '--y', '0', '--z', '0.17',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'
            ]
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[('cloud_in', '/livox/lidar'), ('scan', '/scan')],
            parameters=[{
                'target_frame': 'livox_frame',
                'transform_tolerance': 0.05,
                'min_height': -0.12,
                'max_height': 1.33,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.00872665,
                'scan_time': 0.1,
                'range_min': 0.15,
                'range_max': 8.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }]
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config, {
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'min_laser_range': 0.15,
                'max_laser_range': 8.0,
                'map_update_interval': 2.0,
                'minimum_time_interval': 0.1,
                'minimum_travel_distance': 0.1,
                'minimum_travel_heading': 0.1,
            }]
        ),
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output={'both': 'log'},
            condition=IfCondition(use_octomap),
            remappings=[('cloud_in', '/livox/lidar')],
            parameters=[{
                'resolution': 0.08,
                'frame_id': 'map',
                'base_frame_id': 'base_link',
                'sensor_model.max_range': 8.0,
                'point_cloud_min_z': -0.2,
                'point_cloud_max_z': 2.0,
                'occupancy_min_z': -0.2,
                'occupancy_max_z': 2.0,
                'filter_ground': False,
            }]
        ),
        Node(
            package='neupan_ros2',
            executable='neupan_node',
            name='neupan_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                robot_config,
                {'robot_config_dir': robot_config_dir},
                {'enable_motion': ParameterValue(enable_motion, value_type=bool)},
            ],
            remappings=[('/neupan_cmd_vel', '/cmd_vel')]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            remappings=[
                ('/environment_grid', '/map'),
                ('/environment_grid_updates', '/map_updates'),
            ],
            condition=IfCondition(use_rviz)
        ),
    ])
