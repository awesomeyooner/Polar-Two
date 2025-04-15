#!/bin/bash

# Clone microros package

build_workspace(){
    colcon build --symlink-install
}

source_workspace(){
    source install/setup.bash
}

install_uros(){

    # Clone the MicroROS Repo
    echo 'Cloning MicroROS Setup...'
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

    # Update Packages
    echo 'Updating Packages...'
    sudo apt update && sudo apt upgrade -y && rosdep update
    rosdep install --from-paths src --ignore-src -y
    sudo apt-get install python3-pip

    # Build the setup package
    build_workspace
    source_workspace

    # Run the setup script
    ros2 run micro_ros_setup create_agent_ws.sh

    # Build everything
    build_workspace

    # Finally, finish by sourcing
    source_workspace
}


install_uros