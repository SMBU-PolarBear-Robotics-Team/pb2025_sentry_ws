#!/bin/zsh

# Record nav rosbag in real world
source install/setup.zsh

ros2 bag record -o smbu_exp2_$(date +%Y%m%d_%H%M%S) \
  /red_standard_robot1/livox/imu \
  /red_standard_robot1/livox/lidar \
  /red_standard_robot1/tf \
  /red_standard_robot1/tf_static \
  /red_standard_robot1/joint_states \
  /red_standard_robot1/robot_description
