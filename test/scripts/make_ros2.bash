#!/bin/bash

# 
# Build sick_scan_xd for ROS2-Linux
# 

pushd ../../../.. 

# set build type (Debug or Release) and logfile
# BUILDTYPE=Debug
BUILDTYPE=Release

if [ -f /opt/ros/eloquent/setup.bash ] ; then source /opt/ros/eloquent/setup.bash ; fi
if [ -f /opt/ros/foxy/setup.bash     ] ; then source /opt/ros/foxy/setup.bash     ; fi

# colcon build --cmake-args " -DROS_VERSION=2" " -DCMAKE_BUILD_TYPE=$BUILDTYPE" --event-handlers console_direct+
colcon build --packages-select libsick_ldmrs --cmake-args " -DCMAKE_BUILD_TYPE=$BUILDTYPE" --event-handlers console_direct+
source ./install/setup.bash
colcon build --packages-select msgpack11 --cmake-args " -DMSGPACK11_BUILD_TESTS=0" " -DCMAKE_BUILD_TYPE=$BUILDTYPE" --event-handlers console_direct+
# colcon build --packages-select sick_scan --cmake-args " -DROS_VERSION=2" " -DCMAKE_ENABLE_EMULATOR=1" " -DCMAKE_BUILD_TYPE=$BUILDTYPE" " -DSCANSEGMENT_XD=0" --event-handlers console_direct+
colcon build --packages-select sick_scan --cmake-args " -DROS_VERSION=2" " -DCMAKE_ENABLE_EMULATOR=1" " -DCMAKE_BUILD_TYPE=$BUILDTYPE" --event-handlers console_direct+
source ./install/setup.bash

# Check sick_scan
if [ ! -f ./build/sick_scan/sick_generic_caller                 ] ; then echo -e "\n## ERROR building sick_generic_caller\n"   ; else echo -e "\nbuild sick_generic_caller finished successfully.\n"   ; fi
if [ ! -f ./install/sick_scan/lib/sick_scan/sick_generic_caller ] ; then echo -e "\n## ERROR installing sick_generic_caller\n" ; else echo -e "\ninstall sick_generic_caller finished successfully.\n" ; fi
ls -al ./build/sick_scan/sick_generic_caller
ls -al ./install/sick_scan/lib/sick_scan/sick_generic_caller
echo -e "\n"

popd

