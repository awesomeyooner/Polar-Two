#!/bin/bash

# Source MicroROS workspace
source ../uros_ws/install/setup.bash

# Launch MicroROS
ros2 launch uros_helper microros.launch.py
