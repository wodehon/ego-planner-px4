#!/bin/sh
# source /opt/ros/melodic/setup.zsh
# px4
# source ~/catkin_ws/devel/setup.zsh   # mavros
source /home/dev/code/ego-planner-px4/devel/setup.zsh --extend
source /home/dev/code/PX4-Autopilot/dev-v1.14.0-fast_planner/Tools/simulation/gazebo-classic/setup_gazebo.bash /home/dev/code/PX4-Autopilot/dev-v1.14.0-fast_planner /home/dev/code/PX4-Autopilot/dev-v1.14.0-fast_planner/build/px4_sitl_devel
# run next two line in terminal
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/dev/code/PX4-Autopilot/dev-v1.14.0-fast_planner
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/dev/code/PX4-Autopilot/dev-v1.14.0-fast_planner/Tools/simulation/gazebo-classic/sitl_gazebo-classic


