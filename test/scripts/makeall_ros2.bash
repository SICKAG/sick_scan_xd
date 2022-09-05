#!/bin/bash

#
# cleanup
#

printf "\033c"
pushd ../../../..
rm -rf ./log
rm -rf ./build
rm -rf ./install 
popd 

#
# Build sick_scan_xd shared api library
#
./makeall_linux.bash


#
# Build sick_scan_xd for ROS2
#

./make_ros2.bash
