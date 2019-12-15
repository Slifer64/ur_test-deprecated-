#!/bin/bash

# ==================================
# define some colors for output
# ==================================
COLOR_RED="\033[1;31m"
COLOR_GREEN="\033[1;32m"
COLOR_YELLOW="\033[1;33m"
COLOR_BLUE="\033[1;34m"
COLOR_CYAN="\033[1;36m"
COLOR_WHITE="\033[1;37m"
COLOR_RESET="\033[0m"

ROS_DISTRO=indigo

cd $INSTALL_SCRIPTS_DIR
UR_ERROR=0
rm log.txt > /dev/null

declare -a arr=("ros_deps" "eigen3" "armadillo" "optoforce" "ur_simulator")

echo -e $COLOR_BLUE"Installing ur_ws..."$COLOR_RESET

#echo -e $COLOR_BLUE"Installing main Dependencies: cmake, wget, xz-utils..."$COLOR_RESET
#sudo apt-get update > /dev/null
#sudo apt-get install -y build-essential cmake wget xz-utils unzip > /dev/null

mkdir -p deps

## now loop through the above array
for i in "${arr[@]}"
do
  cd $INSTALL_SCRIPTS_DIR
  source install_$i.sh
  if [ $UR_ERROR -ne 0 ]; then
    echo -e $COLOR_RED"Failed to install ur_ws Packages...."$COLOR_RESET
    exit 1
  fi
  sleep 4
done

cd $INSTALL_SCRIPTS_DIR
rm -rf deps/
cd ..
