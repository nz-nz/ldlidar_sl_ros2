#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('ldlidar_sl_ros2')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # LiDAR parameters
    lidar_params = {
        'product_name': 'LDLiDAR_LD14P',
        'laser_scan_topic_name': 'scan',
        'point_cloud_2d_topic_name': 'pointcloud2d',
        'frame_id': 'base_laser',
        'port_name': '/dev/ttyUSB0',
        'serial_baudrate': 230400,
        'laser_scan_dir': True,
        'enable_angle_crop_func': False,
        'angle_crop_min': 135.0,
        'angle_crop_max': 225.0
    }

    # LiDAR node
    ldlidar_node = Node(
        package='ldlidar_sl_ros2',
        executable='ldlidar_sl_ros2_node',
        name='ldlidar_publisher_ld14',
        output='screen',
        parameters=[lidar_params, {'use_sim_time': use_sim_time}]
    )

    # Static transform publisher from base_link to base_laser
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser_ld14p',
        arguments=['0', '0', '0.18', '0', '0', '0', '1', 'base_link', 'base_laser']
    )

    # SLAM Toolbox configuration file
    slam_params_file = os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml')

    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(ldlidar_node)
    ld.add_action(static_transform_publisher)
    ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)

    return ld