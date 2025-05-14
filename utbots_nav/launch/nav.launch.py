import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

'''
export BASE_MODEL=hestia

TODO: set this as default parameter
TODO: adjust which parameters should be available to chenge configs on the 3 pulled launch files
'''

def generate_launch_description():
    # Arguments
    input_scan = LaunchConfiguration('input_scan_topic')
    output_scan = LaunchConfiguration('filtered_scan_topic')
    map_dir = LaunchConfiguration(
            'map',
            default=os.path.join(
                get_package_share_directory('utbots_nav'),
                'map',
                'corredor2.yaml'))

    utbots_nav_launch_file_dir = os.path.join(get_package_share_directory('utbots_nav'), 'launch')

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'input_scan_topic',
            default_value='scan',
            description='Input LaserScan topic name'
        ),
        DeclareLaunchArgument(
            'filtered_scan_topic',
            default_value='scan_filtered',
            description='Output (filtered) LaserScan topic name'
        ),
        # Load Map
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        # Hoverboard Diffbot Driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([utbots_nav_launch_file_dir, '/hoverboard.launch.py']),
        ),
        # LIDAR Driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([utbots_nav_launch_file_dir, '/rplidar.launch.py']),
        ),
        # Mapping launchfile
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([utbots_nav_launch_file_dir, '/utbots_navigation.launch.py']),
        ),
        # Laser Filter
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
                        parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("utbots_nav"),
                    "param", "box_filter.yaml",
                ])],
            remappings=[
                ('scan', input_scan),
                ('scan_filtered', output_scan)
            ]
        )
        ])