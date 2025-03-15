#!/bin/zsh

# Record vision rosbag in real world 
source install/setup.zsh

ros2 bag record -o vision_spin_$(date +%Y%m%d_%H%M%S) \
  /red_standard_robot1/front_industrial_camera/image \
  /red_standard_robot1/front_industrial_camera/camera_info \
  /red_standard_robot1/tf \
  /red_standard_robot1/tf_static \
  /red_standard_robot1/joint_states \
  /red_standard_robot1/robot_description