import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetRemap

'''
export BASE_MODEL=hestia
'''

def generate_launch_description():
    # Arguments
    input_scan_topic = LaunchConfiguration('input_scan_topic')
    filtered_scan_topic = LaunchConfiguration('filtered_scan_topic')
    map_dir = LaunchConfiguration('map')
    use_imu = LaunchConfiguration('use_imu')
    use_rviz = LaunchConfiguration('use_rviz')
    lidar_port = LaunchConfiguration('lidar_port')

    utbots_nav_launch_file_dir = os.path.join(get_package_share_directory('utbots_nav'), 'launch')

    return LaunchDescription([
        # Launch Arguments
        
        # Scan topics
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
            default_value=os.path.join(
                get_package_share_directory('utbots_nav'),
                'map',
                'corredor2.yaml'),
            description='Full path to map file to load'),

        # Use IMU
        DeclareLaunchArgument(
            'use_imu',
            default_value='true',
            description='Set to true to launch robot_localization'
        ),

        # Launch with RVIZ
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Use RVIZ2 for visualization'
        ),

        # Launch with RVIZ
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            description='Define LIDAR serial port'
        ),

        # Include Launches

        # Hoverboard Diffbot Driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([utbots_nav_launch_file_dir, '/hoverboard.launch.py']),
        ),
        
        # LIDAR Driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([utbots_nav_launch_file_dir, '/rplidar.launch.py']),
            launch_arguments={'lidar_port': lidar_port}.items()  # Pass map as an argument to the mapping launchfile
        ),
        
        # Mapping launchfile
        GroupAction([
            SetRemap('/odom', '/odometry/filtered'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([utbots_nav_launch_file_dir, '/utbots_navigation.launch.py']),
                launch_arguments={'map': map_dir}.items(),
            ),
        ], condition=IfCondition(use_imu)),

        GroupAction([
            SetRemap('/odom', '/odom'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([utbots_nav_launch_file_dir, '/utbots_navigation.launch.py']),
                launch_arguments={'map': map_dir}.items(),
            ),
        ], condition=UnlessCondition(use_imu)),

        # Run Nodes
        
        # Laser Filter
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("utbots_nav"),
                    "param", "box_filter.yaml",
                ])
            ],
            remappings=[
                ('scan', input_scan_topic),
                ('scan_filtered', filtered_scan_topic)
            ]
        ),
        
        # Kalman Filter for IMU integration to Odom
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("utbots_nav"),
                    "param", "ekf_filter.yaml",
                ])
            ],
            condition=IfCondition(use_imu)
        ),
        
        ])
