#!/bin/bash -i

install_ros(){

    source ~/.bashrc
    
    echo 'Installing ros2...'

    locale  # check for UTF-8

    sudo apt update && sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings

    sudo apt install -y software-properties-common
    sudo add-apt-repository universe -y

    sudo apt update && sudo apt install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt upgrade -y

    sudo apt install -y python3-colcon-common-extensions

    sudo apt install -y ros-humble-desktop

    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    
    source /opt/ros/humble/setup.bash
   
    sudo apt-get install -y ros-$ROS_DISTRO-ros-gz

    source ~/.bashrc

    echo 'Finished!'
}

install_ros