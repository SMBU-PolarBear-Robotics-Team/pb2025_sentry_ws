#!/bin/zsh

# Start mapping in real world
source install/setup.zsh

ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=none \
slam:=True \
use_robot_state_pub:=True
