#!/bin/zsh

# Record nav rosbag in simulation
source install/setup.zsh

ros2 bag record -o rmul_20250127 \
  /clock \
  /red_standard_robot1/livox/imu \
  /red_standard_robot1/livox/lidar \
  /red_standard_robot1/tf \
  /red_standard_robot1/tf_static \
  /red_standard_robot1/joint_states \
  /red_standard_robot1/robot_description
