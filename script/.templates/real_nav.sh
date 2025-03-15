#!/bin/zsh

# Start navigation in real world
source install/setup.zsh

ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=smbu_exp2_20241212 \
slam:=False \
use_robot_state_pub:=True
