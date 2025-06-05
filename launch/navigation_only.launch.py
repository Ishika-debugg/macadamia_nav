#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_name = 'macadamia_nav'
    
    return LaunchDescription([
        # LIDAR sensor (RPLidar)
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True
            }]
        ),
        
        # Static transform from base_link to laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0.1', '0', '0.12', '0', '0', '0', 'base_link', 'laser']
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher'
        ),
        
        # Main navigation node
        Node(
            package=pkg_name,
            executable='core_navigation.py',
            name='macadamia_navigator',
            output='screen',
            parameters=[{
                'linear_speed': 0.25,     # m/s
                'angular_speed': 0.3,     # rad/s
                'row_distance': 1.2,      # distance to maintain from tree line
                'safe_distance': 0.6      # obstacle avoidance distance
            }]
        ),
    ])

# Minimal test version
def generate_minimal_launch_description():
    pkg_name = 'macadamia_nav'
    
    return LaunchDescription([
        # LIDAR only
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser'
            }]
        ),
        
        # Fake odometry for testing
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),
        
        # Navigation node with conservative parameters
        Node(
            package=pkg_name,
            executable='macadamia_navigator.py',
            name='macadamia_navigator',
            output='screen',
            parameters=[{
                'linear_speed': 0.15,     # Slower for testing
                'angular_speed': 0.2,
                'row_distance': 1.0,
                'safe_distance': 0.8      # More conservative
            }]
        ),
    ])