#!/bin/bash
set -e

cd /home/developer/ros2_ws

vcs import --recursive < src/mpc_robot_controller/ros_deps.repos src
vcs import --recursive < src/rosbot_2r_demo/ros_deps.repos src
vcs import --recursive < src/vcs/rosbot_ros/rosbot/rosbot_hardware.repos src/vcs
vcs import --recursive < src/vcs/rosbot_ros/rosbot/rosbot_simulation.repos src/vcs

rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
