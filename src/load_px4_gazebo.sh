#!/bin/sh
# source /opt/ros/melodic/setup.zsh
# px4
source ~/catkin_ws/devel/setup.zsh   # mavros
source ~/Code/PX4-Autopilot/Tools/setup_gazebo.bash ~/Code/PX4-Autopilot/ ~/Code/PX4-Autopilot/build/px4_sitl_rtps
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Code/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Code/PX4-Autopilot/Tools/sitl_gazebo

