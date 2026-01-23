#!/bin/bash
for id in {1..15}; do
    for position in 0 45 90 135 180 135 90 45 0; do
        ros2 topic pub --once /servo/command servo_msgs/msg/ServoCommand \
        "{servo_type: 'bus', servo_id: ${id}, position: ${position}, speed: 50, stamp: {sec: 0, nanosec: 0}}"
        sleep 0.1
    done
done