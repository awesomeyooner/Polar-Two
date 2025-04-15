#!/bin/bash

#make workspace for packages
# echo 'Creating Workspace...'
# mkdir ros2_ws
# cd ros2_ws

#clone microros package
echo 'Cloning MicroROS Setup...'
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

#update
echo 'Updating Packages...'
sudo apt update && sudo apt upgrade -y && rosdep update
rosdep install --from-paths src --ignore-src -y
sudo apt-get install python3-pip

#build the setup package
echo 'Building Packages...'
colcon build
source install/local_setup.bash

#clone and build the agent package
echo 'Building Agent...'
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh

#source package
echo 'Sourcing...'
source install/local_setup.bash

echo 'Finished!!!'