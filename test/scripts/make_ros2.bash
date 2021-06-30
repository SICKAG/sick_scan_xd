#!/bin/bash

# 
# Build sick_scan_xd for ROS2-Linux
# 

pushd ../../../.. 

# set build type (Debug or Release) and logfile
# BUILDTYPE=Debug
BUILDTYPE=Release

source /opt/ros/eloquent/setup.bash
colcon build --packages-select sick_scan --cmake-args " -DROS_VERSION=2" " -DCMAKE_BUILD_TYPE=$BUILDTYPE" --event-handlers console_direct+
source ./install/setup.bash

# Check sick_scan
#if [ ! -f ./build/sick_lidar3d/lidar3d_mrs100_recv ] ; then
#    echo -e "\n## ERROR building lidar3d_mrs100_recv, make lidar3d_mrs100_recv failed.\n"
#else
#    echo -e "\nbuild lidar3d_mrs100_recv finished.\n"
#fi
#ls -al ./build/sick_lidar3d/lidar3d_mrs100_recv
#echo -e "\n"

popd

