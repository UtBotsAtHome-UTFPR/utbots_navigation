import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

'''
export BASE_MODEL=hestia
'''

def generate_launch_description():

    utbots_nav_launch_file_dir = os.path.join(get_package_share_directory('utbots_nav'), 'launch')
    rplidar_launch_file_dir = os.path.join(get_package_share_directory('rplidar_ros'), 'launch')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([utbots_nav_launch_file_dir, '/hoverboard.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([utbots_nav_launch_file_dir, '/utbots_navigation.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rplidar_launch_file_dir, '/rplidar_a1_launch.py']),
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '-0.08', '0', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'laser', 'scan'],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.35', '0', '0.06', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),

        ])