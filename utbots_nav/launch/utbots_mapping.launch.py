import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

'''
export BASE_MODEL=hestia
'''

BASE_MODEL = os.environ['BASE_MODEL']

def generate_launch_description():
    # Sim Time Argument
    use_sim_time = LaunchConfiguration('use_sim_time')

    # SLAM Parameters
    param_file_name = BASE_MODEL + '_map.yaml'
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(
            get_package_share_directory('utbots_nav'),
            'param',
            param_file_name))

    return LaunchDescription([
        # Sim Time Argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation/Gazebo clock'),

        # SLAM Parameters
        #NOTE: Esse arquivo deveria ser:
        #NOTE: slam_params_file:=hoverboard-driver/bringup/config/mapper_params_online_async.yaml
        #NOTE: O que seria get_package_share_directory('utbots_nav'), 'param'... mas se fizer certo não funciona e se tirar o caminho completo ele vai.
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join("utbots_nav",
                                    'param', 'hestia_map.yaml'),
            description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),

        # Async Slam Toolbox Node
        Node(
            parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen')
    ])
