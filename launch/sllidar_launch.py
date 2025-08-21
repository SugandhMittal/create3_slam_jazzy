#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch configurations
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='')
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('channel_type', default_value='serial', description='LIDAR channel type'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0', description='LIDAR serial port'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200', description='LIDAR serial baudrate'),
        DeclareLaunchArgument('frame_id', default_value='laser', description='LIDAR frame ID'),
        DeclareLaunchArgument('inverted', default_value='false', description='Invert scan data'),
        DeclareLaunchArgument('angle_compensate', default_value='true', description='Enable angle compensation'),
        DeclareLaunchArgument('scan_mode', default_value='Standard', description='LIDAR scan mode'),
        DeclareLaunchArgument('namespace', default_value='', description='Robot namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),

        # Static transform publisher (base_footprint -> laser)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['-0.012', '0', '0.144', '0', '0', '0', '1','base_footprint', 'laser'],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            namespace=namespace
        ),

        # I made this when I was not using teleop_twist_keyboard
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='odom_to_base',
        #     arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_footprint'],
        #     remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        #     namespace=namespace,
        #     output='screen'
        # ),



      # SLLIDAR Node
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='sllidar_ros2',
                    executable='sllidar_node',
                    name='sllidar_node',
                    namespace=namespace,
                    output='screen',
                    parameters=[{
                        'channel_type': channel_type,
                        'serial_port': serial_port,
                        'serial_baudrate': serial_baudrate,
                        'frame_id': frame_id,
                        'inverted': inverted,
                        'angle_compensate': angle_compensate,
                        'scan_mode': scan_mode,
                        'use_sim_time': use_sim_time
                    }]
                )
            ]
        ),
    ])

