#!/bin/bash
pushd ../../../..
source /opt/ros/melodic/setup.bash
rm -f ./build/catkin_make_install.log

#
# build and install
#

#catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release

catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DLDMRS=0 -DCMAKE_ENABLE_EMULATOR=1 2>&1 | tee -a ./build/catkin_make_install.log
#catkin_make install --cmake-args -DROS_VERSION=1 2>&1 | tee -a ./build/catkin_make_install.log
source ./install/setup.bash

#
# print warnings and errors
#

echo -e "\nmake.bash finished.\n"
echo -e "catkin_make warnings:"
cat build/catkin_make_install.log | grep -i "warning:"
echo -e "\ncatkin_make errors:"
cat build/catkin_make_install.log | grep -i "error:"

# print sick_scan_xd binaries
echo -e "\ninstall/lib/sick_scan_xd:"
ls -al ./install/lib/sick_scan_xd
popd

