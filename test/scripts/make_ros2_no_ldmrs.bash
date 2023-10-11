#!/bin/bash

# 
# Build sick_scan_xd for ROS2-Linux
# 

pushd ../../../.. 

# set build type (Debug or Release) and logfile
# BUILDTYPE=Debug
BUILDTYPE=Release

source /opt/ros/eloquent/setup.bash
# colcon build --cmake-args " -DROS_VERSION=2" " -DLDMRS=0" " -DCMAKE_BUILD_TYPE=$BUILDTYPE" --event-handlers console_direct+
colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" "-DCMAKE_ENABLE_EMULATOR=1" " -DLDMRS=0" " -DCMAKE_BUILD_TYPE=$BUILDTYPE" --event-handlers console_direct+
source ./install/setup.bash

# Check sick_scan_xd
if [ ! -f ./build/sick_scan_xd/sick_generic_caller                 ] ; then echo -e "\n## ERROR building sick_generic_caller\n"   ; else echo -e "\nbuild sick_generic_caller finished successfully.\n"   ; fi
if [ ! -f ./install/sick_scan_xd/lib/sick_scan_xd/sick_generic_caller ] ; then echo -e "\n## ERROR installing sick_generic_caller\n" ; else echo -e "\ninstall sick_generic_caller finished successfully.\n" ; fi
ls -al ./build/sick_scan_xd/sick_generic_caller
ls -al ./install/sick_scan_xd/lib/sick_scan_xd/sick_generic_caller
echo -e "\n"

popd

