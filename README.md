# utbots_navigation

**This stack contains packages related to environment mapping and autonomous navigation, such as:**

- hoverboard_driver_hestia
- utbots_nav

## Getting started

### Installation

```bash
cd <ros2_ws>/src
git clone https://github.com/UtBotsAtHome-UTFPR/utbots_navigation.git
git checkout ros2-dev
cd ../
```

### Dependencies

This stacks utilizes [rplidar_ros](https://github.com/Slamtec/rplidar_ros/tree/ros2) as the LIDAR driver and [navigation2](https://github.com/ros-navigation/navigation2) for navigation and mapping algorithms integration to ROS2. Install them with:

```bash
sudo apt install ros-humble-rplidar-ros ros-humble-navigation2
```

### Building

```bash
cd <ros2_ws>
colcon buiild --symlink-install
```

## Running

TODO: Document running the nav and map launches

## Overview

### rplidar_ros

- ROS2 Driver for RPLidar sensors

#### Installation
```bash
sudo apt install ros-humble-rplidar-ros
```

#### Running
We use RPLIDAR A1:
```bash
sudo chmod 777 /dev/ttyUSB0
ros2 launch rplidar_ros view_rplidar_a1_launch.py
```

---

### navigation2

- Navigation and mapping algorithms integration to ROS2

#### Installation
```bash
sudo apt install ros-humble-navigation2
```

#### Running
TODO: How to run basic nav2?
