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
            executable='macadamia_navigator.py',
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