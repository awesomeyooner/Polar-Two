#!/bin/bash -i

# Reset
Color_Off='\033[0m'       # Text Reset

# Regular Colors
Black='\033[0;30m'        # Black
Red='\033[0;31m'          # Red
Green='\033[0;32m'        # Green
Yellow='\033[0;33m'       # Yellow
Blue='\033[0;34m'         # Blue
Purple='\033[0;35m'       # Purple
Cyan='\033[0;36m'         # Cyan
White='\033[0;37m'        # White

install_general_packages(){
    
    source ~/.bashrc

    echo -e "Running...${Green}Install APT Packages!${Color_Off}"

    sudo apt install -y \
    ros-humble-joint-state-publisher-gui \
    libserial-dev \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-ign-ros2-control \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-vision-opencv \
    joystick \
    jstest-gtk \
    evtest

    echo -e "Running...${Green}Installing Pip Packages!${Color_Off}"

    pip install catkin_pkg \
    empy==3.3.4 \
    lark \
    docutils \
    pyparsing

    pip install pyparsing

    pip install docutils

    echo -e "Running...${Green}Installing Conda Packages!${Color_Off}"

    conda install -y \
    nlohmann_json \
    numpy

    # Create symlink for numpy
    echo -e "Running...${Green}Creating Symlink for Numpy!${Color_Off}"
    sudo ln -s /home/dev/anaconda3/lib/python3.12/site-packages/numpy/_core/include/numpy /usr/include/numpy

    echo -e "Running...${Green}Uninstalling TIFF!${Color_Off}"
    conda uninstall -y libtiff
}

install_general_packages

source ~/.bashrc

pip install pyparsing

pip install docutils