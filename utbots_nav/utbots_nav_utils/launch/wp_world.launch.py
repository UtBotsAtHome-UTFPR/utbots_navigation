from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction
import subprocess
import signal
# (.ros0) ubuntu@ubuntu:~$ colcon_cd  turtlebot3_navigation2 
# /opt/ros/humble/share/turtlebot3_navigation2

# Global variables to store subprocesses
rqt_action_proc = None
rqt_service_proc = None


def launch_rqt_tools(context, *args, **kwargs):
    global rqt_action_proc, rqt_service_proc
    # Launch RQT Action Caller
    # rqt_action_proc = subprocess.Popen(["ros2", "run", "rqt_action", "rqt_action"])

    # Launch RQT Service Caller
    rqt_service_proc= subprocess.Popen(["ros2", "run", "rqt_service_caller", "rqt_service_caller"])

    return ([
        
    ])


# def on_shutdown(context,*args, **kwargs):
#     global rqt_action_proc, rqt_service_proc

#     for proc in [rqt_action_proc, rqt_service_proc]:
#         if proc:
#             try:
#                 proc.send_signal(signal.SIGINT)
#                 proc.wait(timeout=5)
#             except Exception:
#                 proc.kill()

def launch_setup(context, *args, **kwargs):
    def set_world_map(map_file):
        world_tb3_nav = os.path.join(get_package_share_directory('turtlebot3_navigation2'),"map/") + "map"
        os.system(f"find {map_file} || cp {world_tb3_nav}.yaml {map_file}")
        # print(world_tb3_nav+".yaml")
        # print(map_file)
        os.system(f"find {map_file.rsplit('.yaml')[0]}.pgm || cp {world_tb3_nav}.pgm {map_file.rsplit('.yaml')[0]}.pgm \
                   && cat {map_file} | grep 'image: world.pgm' || sed -i '1s|.*|image: world.pgm|' {map_file}")

        # print(world_tb3_nav+".pgm")
        # print(map_file.rsplit('.yaml')[0]+".pgm")
        return ([])

    map_name = LaunchConfiguration('map_name').perform(context)

    map_file = os.path.join(
        get_package_share_directory('utbots_nav').rsplit('install/')[0],
        "src","utbots_navigation" ,"utbots_nav", "map" #src/utbots_navigation/utbots_nav/map/
    ) + "/" + map_name + ".yaml"
    
    if(map_name=="world"):
        set_world_map(map_file)

    launch_dir_tb3_gz = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch')
    
    launch_dir_tb3_nav= os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 'launch')

    return ([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir_tb3_nav, 'navigation2.launch.py')),
            launch_arguments={'map': map_file}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir_tb3_gz, 'turtlebot3_world.launch.py')),
            # launch_arguments={'map': 'true'}.items()
        ),       

            Node(
                package='utbots_nav_utils',
                executable='nav_to_wp',
                name='nav_to_wp',
                output='screen',
                emulate_tty=True,
                # parameters=[
                #     {
                #}
                # ],
                
            ),
        Node(
            package='utbots_nav_utils',
            executable='save_waypoint_service',
            name='save_waypoint_service',
            output='screen',
            emulate_tty=True,
            # parameters=[
            #     {
            #}
            # ],
        ),

    ])

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='world'),
        OpaqueFunction(function=launch_setup),
        OpaqueFunction(function=launch_rqt_tools),
        # OpaqueFunction(function=on_shutdown),
    ])