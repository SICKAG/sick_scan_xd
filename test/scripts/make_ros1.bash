#!/bin/bash
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash ] ; then source /opt/ros/melodic/setup.bash ; fi
if [ -f /opt/ros/noetic/setup.bash  ] ; then source /opt/ros/noetic/setup.bash  ; fi
rm -f ./build/catkin_make_install.log

#
# Build and install msgpack11
#

mkdir -p ./build/msgpack11
pushd ./build/msgpack11
cmake -G "Unix Makefiles" -D CMAKE_CXX_FLAGS=-fPIC -D CMAKE_BUILD_TYPE=Release -D MSGPACK11_BUILD_TESTS=0 ../../src/msgpack11  2>&1 | tee -a ../catkin_make_install.log
make              2>&1 | tee -a ../catkin_make_install.log
sudo make install 2>&1 | tee -a ../catkin_make_install.log
popd

#
# Build and install sick_scan_xd
#

#catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DCMAKE_ENABLE_EMULATOR=1 2>&1 | tee -a ./build/catkin_make_install.log
catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DCMAKE_ENABLE_EMULATOR=1 -DSCANSEGMENT_XD=1 2>&1 | tee -a ./build/catkin_make_install.log
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

