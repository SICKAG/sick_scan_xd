#!/bin/bash

sudo chmod a+x ./*.bash
sudo chmod a+x ../docker/*.bash
sudo dos2unix ./*.bash
sudo dos2unix ../docker/*.bash
printf "\033c"

#
# Build sick_scan_xd shared api library
#
./makeall_linux.bash

pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash ] ; then source /opt/ros/melodic/setup.bash ; fi
if [ -f /opt/ros/noetic/setup.bash  ] ; then source /opt/ros/noetic/setup.bash  ; fi

#
# cleanup
#

rosclean purge -y
rm -rf ./build ./devel ./install ./build_isolated ./devel_isolated ./install_isolated ./log
rm -rf ~/.ros/*
if [ -d ./ros2_example_application ] ; then rm -rf ./ros2_example_application ; fi
# catkin clean --yes --all-profiles --verbose
# catkin_make clean
mkdir -p ./build_isolated ./devel_isolated ./install_isolated
ln -s ./build_isolated ./build
ln -s ./devel_isolated ./devel
ln -s ./install_isolated ./install
popd 

#
# make
#

./make_ros1.bash

