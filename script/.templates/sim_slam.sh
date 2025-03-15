#!/bin/zsh

# Start mapping in simulation
source install/setup.zsh

ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
slam:=True
