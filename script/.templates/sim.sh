#!/bin/zsh

# Start ign simulation
source install/setup.sh

ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
