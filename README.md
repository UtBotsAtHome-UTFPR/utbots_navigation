# utbots_navigation

![Static Badge](https://img.shields.io/badge/ROS_Foxy-Not_Tested-red)
![Static Badge](https://img.shields.io/badge/ROS_Humble-Tested-green)
![Static Badge](https://img.shields.io/badge/ROS_Jazzy-Not_Tested-red)

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

For the MPU6050 IMU integration, you must install and setup microROS:

```bash
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Download the micro-ROS tools
mkdir microros_ws
cd <ros2_ws>
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash

# Download and build micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

### Building

```bash
cd <ros2_ws>
colcon buiild --symlink-install
```

## Running

```bash
export BASE_MODEL=hestia
ros2 launch utbots_nav map.launch.py    ## Launch all mapping packages (Gmapping)
ros2 launch utbots_nav nav.launch.py    ## Launch all navigation packages (AMCL)
```

### Changing parameters

You can change the robot tf in `./hoverboard-driver-hestia/description/urdf/diffbot_description.urdf.xacro`, the navigation parameters in `./utbots_nav/param/hestia_nav.yaml` and mapping parameters in `./utbots_nav/param/hestia_map.yaml`.

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


---

### mpu6050driver

#### Installation
Refer to the [tutorial on microROS with ESP32](https://technologiehub.at/project-posts/micro-ros-on-esp32-tutorial/) and [PlatformIO IDE](https://docs.platformio.org/en/latest/integration/ide/vscode.html).

-  ESP32 board .cpp code for MPU6050 IMU read through I2C port and serial communication with ROS2 using the microROS package

#### Running
Connect the MPU6050 board pins like the following: VCC to 3.3V, GND to GND, SCL to G22 and SDA to G21. 

Flash the firmware in `mpu6050driver/src/main.cpp` to the board, e.g. using the PlatformIO interface in VSCode with the .ini file, selecting the the port and run the following:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev <port>
```

You should see a topic called `/imu_info_topic` in `ros2 topic list`.
