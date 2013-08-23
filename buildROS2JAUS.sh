#!/bin/bash

#discover ip address
#ip=( $(sed -n -e 's/inet addr: //p' $D/data.txt) )
#exit()

cores=$1
if [[ -z "$cores" ]]; then
  echo "How many jobs should I use to make? "
  read cores
else
  echo "Using $cores jobs"
fi
# In order to fix the ROS package path, run the following
# REPLACE THE PATH BEFORE THE : WITH THE PATH TO SIMPLESIM
ROS_PACKAGE_PATH=$(pwd):${ROS_PACKAGE_PATH}

# create our clean build dir
rm -rf build/
mkdir build
cd build

# make everything
cmake ..
make -j$cores

