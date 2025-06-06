from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_name = 'macadamia_nav'  # change this to your package name

    return LaunchDescription([

        # Start your custom row follower node
        Node(
            package='macadamia_nav',
            executable='row_follower',
            name='row_follower',
            output='screen'
        ),
    ])
