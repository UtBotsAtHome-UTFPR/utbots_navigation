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

TODO: set this as default parameter
TODO: adjust which parameters should be available to chenge configs on the 3 pulled launch files
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
            PythonLaunchDescriptionSource([utbots_nav_launch_file_dir, '/rplidar.launch.py']),
        ),

        ])