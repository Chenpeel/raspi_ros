#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build 
source install/setup.bash
ros2 launch websocket_bridge full_system.launch.py