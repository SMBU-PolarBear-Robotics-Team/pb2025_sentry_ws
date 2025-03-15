#!/bin/zsh

# Start navigation in simulation
source install/setup.zsh

ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
world:=rmul_2025 \
namespace:=blue_standard_robot1 \
slam:=False
