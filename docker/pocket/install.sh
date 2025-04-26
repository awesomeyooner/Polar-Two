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

sudo chmod +x anaconda.sh packages.sh ros.sh

echo -e "Installing...${Green}Anaconda${Color_Off}"
./anaconda.sh

echo -e "Installing...${Green}ROS${Color_Off}"
./ros.sh

echo -e "Installing...${Green}General Packages${Color_Off}"
./packages.sh