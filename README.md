# utbots_navigation

- ## rosaria
    - Enables communication of the Pioneer P3AT with ROS
    - Run it from the launch file in nav_main (see below)

- ## ydlidar_ros_driver 
    - LIDAR driver for ROS
    ```bash
    # Clone
    git clone https://github.com/UtBotsAtHome-UTFPR/utbots_navigation.git

    # Compile YDLidar SDK
    roscd ydlidar_ros_driver/build
    cmake ..
    make
    sudo make install

    # Install Aria
    sudo apt update && sudo apt install libaria-dev

    # Compile workspace
    cd ~/catkin_ws
    catkin_make
    ```

- ## nav_main
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
