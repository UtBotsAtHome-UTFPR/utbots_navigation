# utbots_nav_utils

## Installation

```bash
cd <ros2_ws>/src
git clone https://github.com/UtBotsAtHome-UTFPR/utbots_navigation.git
git checkout ros2-dev
cd ../
```

### Dependencies

- Gazebo , Turtlebot3 dependencies and other nav2 dependencies are recomended

```bash
sudo apt install ros-humble-cartographer -y
sudo apt install ros-humble-cartographer-ros -y
sudo apt install ros-humble-rplidar-ros ros-humble-navigation2 -y
sudo apt install ros-humble-nav2-bringup -y
#for simulation with tb3:
sudo apt install ros-humble-turtlebot3* -y
sudo apt install ros-humble-gazebo-* -y
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc #In order to Gazebo to work
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc #In order to turtlebot3 to work
```

### Building

```bash
cd <ros2_ws>
# to build only the package and utbots_dependencies:
colcon build --packages-select utbots_nav_utils  utbots_msgs utbots_srvs utbots_actions \
# to rebuild utbots_actions utbots_msgs utbots_srvs (c++) ((will slow the process))
--allow-overriding utbots_actions utbots_msgs utbots_srvs \ 
# to build with all cpu threads: [for slow machines or machines under heavy load: use n/2(n<=8) or n - 4(n>8)]
--parallel-workers $(nproc)  \ 
&& source install/setup.bash # 

```bash
ros2 action send_goal /navigate_to_way_point utbots_actions/action/NavigateToWayPoint "{waypoint: {data: 'bathroom'}}"
```
### Setup

```bash
#recomended:
cat ~/.bashrc | grep 'source /opt/ros/$ROS_DISTRO/setup.bash' || \
#this will find something like 'source /opt/ros/humble/setup.bash'(sic.*)
grep 'source /opt/ros/'${ROS_DISTRO}'/setup.bash' || \
#if not, will register in the bashrc the source
echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc 
#necessary:
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc #In order to Gazebo to work
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc #In order to turtlebot3 to work
#or run on every new bash session:
#export TURTLEBOT3_MODEL=burger

```


## Running

```bash
#export TURTLEBOT3_MODEL=burger

ros2 run utbots_nav_utils nav_to_wp

ros2 run utbots_nav_utils save_waypoint_service

#or:

ros2 launch utbots_nav_utils wp_world.launch.py

```

### Changing parameters

Currently, there are no parameters for the simulation.
For testing with the real robot, need adjustments.

## Overview

### Navigate to Waypoint Action (and Server):

- The action server /navigate_to_way_point (NavigateToWayPoint action type) will recieve a string containing a waypoint and will send a NavigateToPose action goal to the respective waypoint.
- The server will read the waypoints file respetive to the map name present in the /map_server/yaml_filename/ parameter. The waypoints must be registered with the respective service.

#### Running
```bash
#to run only the server:
ros2 run utbots_nav_utils nav_to_wp
#to call the action goal:
ros2 action send_goal /navigate_to_way_point utbots_actions/action/NavigateToWayPoint \
 #change <waypoint_name> to the desired waypoint (like: bathroom, living_room, kitchen, etc. (sic.*))
"{waypoint: {data: '<waypoint_name>'}}" #like "{waypoint: {data: 'bathroom'}}"
```
### Save Waypoint Service (and Server):

- The server /utbots/utbots_nav/save_waypoint (utbots_srvs/srv/SetString service type) will recieve a string containing a waypoint and will register the robot pose with the respective waypoint (/amcl_pose).
- The server will create the waypoints file respetive to the map name present in the /map_server/yaml_filename/ parameter. The file will be created inside '<ros2-ws>/src/utbots_navigation/utbots_nav/map/' as '{file_name}_waypoints.yaml' (map.yaml -> map_waypoints.yaml)

#### Running
```bash
#to run only the server:
ros2 run utbots_nav_utils save_waypoint_service
#to call the service:
ros2 service call /utbots/utbots_nav/save_waypoint utbots_srvs/srv/SetString \
#change <waypoint_name> to the desired waypoint (like: bathroom, living_room, kitchen, etc. (sic.*))
"{waypoint: {data: '<waypoint_name>'}}" #like "{waypoint: {data: 'bathroom'}}"

#the service can be called via rqt_service_caller, its realy useful sometimes when the cli is buggy
```
---

## Simulating with the wp_world launch file

The launch file will run:
- To navigate the simulated robot over
    - turtlebot3_navigation2 navigation2.launch
- To simulate the physical robot inputs and outputs 
    - turtlebot3_gazebo turtlebot3_world.launch
- To save the waypoints
    - utbots_nav_utils save_waypoint_service
- To send the goal to the waypoints
    - utbots_nav_utils nav_to_wp
- To call the service:
    - rqt service caller
The launch file will also:
- make a copy of the world map (map.yaml) from the turtlebot source to the local maps folder
```bash
#(.ros0) ubuntu@ubuntu:~$ colcon_cd turtlebot3_gazebo && pwd
#/opt/ros/humble/share/turtlebot3_gazebo
```
The action nav_to_wp still need to be called externally


#### Running
```bash
#to launch all the nodes:
ros2 launch utbots_nav_utils wp_world.launch #map:='world' to load default world gazebo map
```
