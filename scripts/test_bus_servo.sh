#!/bin/bash

for position in 0 45 90 135 180 135 90 45 0; do
    ros2 topic pub --once /servo/command std_msgs/String \
        "data: '{\"servo_type\":\"bus\",\"servo_id\":8,\"position\": ${position},\"speed\":50}'"
    sleep 0.1
done