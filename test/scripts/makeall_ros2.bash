#!/bin/bash

#
# cleanup
#

printf "\033c"
pushd ../../../..
rm -rf ./build ./devel ./install ./build_isolated ./devel_isolated ./install_isolated ./log
# if [ -d ./ros2_example_application ] ; then rm -rf ./ros2_example_application ; fi
popd 

#
# Build sick_scan_xd shared api library
#
./makeall_linux.bash


#
# Build sick_scan_xd for ROS2
#

# cp -rf ../../examples/ros2_example_application ../../../ros2_example_application
./make_ros2.bash

