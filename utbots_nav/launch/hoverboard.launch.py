from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    twist_mux_params = os.path.join(
        get_package_share_directory('utbots_nav'),
        'param',
        'twist_mux.yaml'
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[
            ('/cmd_vel_out', '/hoverboard_base_controller/cmd_vel_unstamped'),
        ]
    )

    nodes = [twist_mux]

    return LaunchDescription(nodes)