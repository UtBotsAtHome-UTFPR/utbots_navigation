# utbots_voice

**This stack contains packages related to human-robot interface, such as:**

- rosaria
- ydlidar_ros_driver
- nav_main

## Getting started

### Installation

```bash
cd catkin_ws/src
git clone --recurse-submodules https://github.com/UtBotsAtHome-UTFPR/utbots_voice.git
cd ../
```

### Building

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### Running

See the usage explanation accessing each package in the repository or below

## Overview

### rosaria

- Enables communication of the Pioneer P3AT with ROS
- Run it from the launch file in nav_main (see below)

---

### ydlidar_ros_driver

- LIDAR driver for ROS

```bash
# Clone
git clone https://github.com/UtBotsAtHome-UTFPR/utbots_navigation.git

# Compile YDLidar SDK
roscd utbots_navigation ydlidar_ros_driver/YDLidar-SDK/build/
cmake ..
make
sudo make install

# Install Aria
sudo apt update && sudo apt install libaria-dev

# Compile workspace
cd ~/catkin_ws
catkin_make
```

---

### nav_main

- For programs and files related to get the navigation system working

```bash
# Lidar
roslaunch nav_main lidar.launch

# move_base (SLAM)
roslaunch nav_main move_base.launch

# RosAria
roslaunch nav_main rosaria.launch

# TF publishing between /map and /odom (and other TFs too)
roslaunch nav_main tf.launch
```
