#!/bin/bash
pushd ../../../..
source /opt/ros/melodic/setup.bash
rm -f ./build/catkin_make_install.log

#
# build and install
#

catkin_make install 2>&1 | tee -a ./build/catkin_make_install.log
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

