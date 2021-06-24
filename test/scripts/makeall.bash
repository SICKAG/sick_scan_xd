#!/bin/bash
pushd ../../../..
source /opt/ros/melodic/setup.bash

#
# cleanup
#

rosclean purge -y
rm -rf ./build ./devel ./install
rm -rf ~/.ros/*
catkin clean --yes --all-profiles --verbose
catkin_make clean
popd 

#
# make
#

./make.bash

