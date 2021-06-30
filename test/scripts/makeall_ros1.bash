#!/bin/bash
pushd ../../../..
source /opt/ros/melodic/setup.bash

#
# cleanup
#

cp -f ./src/sick_scan_xd/package_ros1.xml ./src/sick_scan_xd/package.xml
rosclean purge -y
rm -rf ./build ./devel ./install
rm -rf ~/.ros/*
catkin clean --yes --all-profiles --verbose
catkin_make clean
popd 

#
# make
#

./make_ros1.bash

