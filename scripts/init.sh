#!/bin/bash

. /opt/ros/$ROS_DISTRO/setup.bash
colcon build 
. install/setup.bash
ros2 launch websocket_bridge full_system.launch.py