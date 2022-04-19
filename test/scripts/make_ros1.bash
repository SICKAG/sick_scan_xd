#!/bin/bash
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash ] ; then source /opt/ros/melodic/setup.bash ; fi
if [ -f /opt/ros/noetic/setup.bash  ] ; then source /opt/ros/noetic/setup.bash  ; fi
rm -f ./build/catkin_make_install.log

#
# build and install
#

#catkin_make_isolated --install --cmake-args -DROS_VERSION=1 2>&1 | tee -a ./build/catkin_make_install.log
catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DCMAKE_ENABLE_EMULATOR=1 2>&1 | tee -a ./build/catkin_make_install.log
source ./install/setup.bash

#
# print warnings and errors
#

echo -e "\nmake.bash finished.\n"
echo -e "catkin_make warnings:"
cat build/catkin_make_install.log | grep -i "warning:"
echo -e "\ncatkin_make errors:"
cat build/catkin_make_install.log | grep -i "error:"

# print sick_scan binaries
echo -e "\ninstall/lib/sick_scan:"
ls -al ./install/lib/sick_scan
popd

