#!/bin/bash
printf "\033c"
pushd ../../../..
source /opt/ros/melodic/setup.bash

#
# cleanup
#

rosclean purge -y
rm -rf ./build ./devel ./install ./build_isolated ./devel_isolated ./install_isolated
rm -rf ~/.ros/*
catkin clean --yes --all-profiles --verbose
# catkin_make clean
mkdir -p ./build_isolated ./devel_isolated ./install_isolated
ln -s ./build_isolated ./build
ln -s ./devel_isolated ./devel
ln -s ./install_isolated ./install
popd 

#
# make
#

./make_ros1_no_ldmrs.bash

