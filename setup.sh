#!/bin/bash

set -e  # Exit immediately on error
set -u  # Treat unset variables as errors

# Define variables
ROS_DISTRO=humble

echo "Installing required ROS 2 packages..."
sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO}-rplidar-ros \
  ros-${ROS_DISTRO}-navigation2 \
  ros-${ROS_DISTRO}-teleop-twist-keyboard \
  ros-${ROS_DISTRO}-twist-mux

echo "Sourcing ROS 2 setup..."
if [ "${AMENT_TRACE_SETUP_FILES:-}" ]; then
  source /opt/ros/${ROS_DISTRO}/setup.bash
fi

cd ../..

# micro-ROS (not currently used)
# Clone micro-ROS setup repository
# if [ ! -d "src/micro_ros_setup" ]; then
#   echo "Cloning micro-ROS setup repository..."
#   git clone -b ${ROS_DISTRO} https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
# else
#   echo "micro_ros_setup already exists, skipping clone."
# fi

# # Install dependencies
# echo "Installing dependencies..."
# sudo apt update
# rosdep update
# rosdep install --from-paths src --ignore-src -y

# # Build the workspace
# echo "Building workspace..."
# colcon build
# source install/local_setup.bash

# # Create and build the micro-ROS agent workspace
# echo "Setting up micro-ROS Agent workspace..."
# ros2 run micro_ros_setup create_agent_ws.sh
# ros2 run micro_ros_setup build_agent.sh

# # Source again after agent build
# source install/local_setup.bash

echo "Setup complete. micro-ROS is installed and ready (not currently in use)."
