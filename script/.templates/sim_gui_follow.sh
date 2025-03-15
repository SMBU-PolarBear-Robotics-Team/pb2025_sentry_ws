#!/bin/zsh

# Set user camera follow the robot in ignition-gazebo

x=-0.8
y=0.0
z=1.8

ign service -s /gui/follow \
  -r "data: 'blue_standard_robot1'" \
  --reqtype ignition.msgs.StringMsg \
  --reptype ignition.msgs.Boolean \
  --timeout 1000

ign service -s /gui/follow/offset \
  -r "x: $x, y: $y, z: $z" \
  --reqtype ignition.msgs.Vector3d \
  --reptype ignition.msgs.Boolean \
  --timeout 1000