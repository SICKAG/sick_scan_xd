#!/bin/bash

# 
# Build sick_scan_xd for ROS2-Linux
# 

pushd ../../../.. 

# set build type (Debug or Release) and logfile
# BUILDTYPE=Debug
BUILDTYPE=Release

if   [ -f /opt/ros/humble/setup.bash   ] ; then source /opt/ros/humble/setup.bash ; ROS2_CMAKE_ARGS=" -DLDMRS=0"
elif [ -f /opt/ros/foxy/setup.bash     ] ; then source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/eloquent/setup.bash ] ; then source /opt/ros/eloquent/setup.bash
fi

# colcon build --cmake-args " -DROS_VERSION=2" " -DCMAKE_BUILD_TYPE=$BUILDTYPE" --event-handlers console_direct+
colcon build --packages-select libsick_ldmrs --cmake-args " -DCMAKE_BUILD_TYPE=$BUILDTYPE" --event-handlers console_direct+
source ./install/setup.bash
colcon build --packages-select msgpack11 --cmake-args " -DMSGPACK11_BUILD_TESTS=0" " -DCMAKE_BUILD_TYPE=$BUILDTYPE" --event-handlers console_direct+
colcon build --packages-select sick_scan --cmake-args " -DROS_VERSION=2" " -DCMAKE_ENABLE_EMULATOR=1" " -DCMAKE_BUILD_TYPE=$BUILDTYPE" $ROS2_CMAKE_ARGS --event-handlers console_direct+
source ./install/setup.bash

# Optional build ros2_example_application
if [ -d ./src/ros2_example_application ] ; then
  source /opt/ros/$ROS_DISTRO/setup.bash
  colcon build --packages-select sick_scan_ros2_example --event-handlers console_direct+
  source ./install/setup.bash
  ls -al ./build/sick_scan_ros2_example/sick_scan_ros2_example ./install/sick_scan_ros2_example/lib/sick_scan_ros2_example/sick_scan_ros2_example
  if [ ! -f ./build/sick_scan_ros2_example/sick_scan_ros2_example ] ; then echo -e "\n## ERROR building sick_scan_ros2_example\n"        ; else echo -e "build sick_scan_ros2_example finished successfully."     ; fi
fi

# print sick_scan binaries
echo -e "\n"
echo -e "src/sick_scan_xd/build_linux:"
ls -al ./src/sick_scan_xd/build_linux/sick_generic_caller
ls -al ./src/sick_scan_xd/build_linux/libsick_scan_shared_lib.so
ls -al ./src/sick_scan_xd/build_linux/sick_scan_xd_api_test
ldd -r ./src/sick_scan_xd/build_linux/libsick_scan_shared_lib.so # print undefined symbols in libsick_scan_shared_lib.so
echo -e "exported symbols in libsick_scan_shared_lib.so:"
nm -C -D ./src/sick_scan_xd/build_linux/libsick_scan_shared_lib.so | grep -i tixml # print exported symbos in libsick_scan_shared_lib.so
nm -C -D ./src/sick_scan_xd/build_linux/libsick_scan_shared_lib.so | grep -i tinyxml # print exported symbos in libsick_scan_shared_lib.so
nm -C -D ./src/sick_scan_xd/build_linux/libsick_scan_shared_lib.so | grep -i SickScanApi # print exported symbos in libsick_scan_shared_lib.so
echo -e "sick_scan_xd_api_test (ROS2):"
ls -al ./build/sick_scan/sick_scan_xd_api_test
ls -al ./install/sick_scan/lib/sick_scan/sick_scan_xd_api_test
echo -e "sick_generic_caller (ROS2):"
ls -al ./build/sick_scan/sick_generic_caller
ls -al ./install/sick_scan/lib/sick_scan/sick_generic_caller

# Check sick_scan
if [ ! -f ./src/sick_scan_xd/build_linux/libsick_scan_shared_lib.so ] ; then echo -e "\n## ERROR building libsick_scan_shared_lib.so\n" ; else echo -e "\nlibsick_scan_shared_lib.so successfully."           ; fi
if [ ! -f ./build/sick_scan/sick_scan_xd_api_test                   ] ; then echo -e "\n## ERROR building sick_scan_xd_api_test\n"      ; else echo -e "build sick_scan_xd_api_test finished successfully."   ; fi
if [ ! -f ./build/sick_scan/sick_generic_caller                     ] ; then echo -e "\n## ERROR building sick_generic_caller\n"        ; else echo -e "build sick_generic_caller finished successfully."     ; fi
if [ ! -f ./install/sick_scan/lib/sick_scan/sick_generic_caller     ] ; then echo -e "\n## ERROR installing sick_generic_caller\n"      ; else echo -e "install sick_generic_caller finished successfully."   ; fi
if [ ! -f ./install/sick_scan/lib/sick_scan/sick_scan_xd_api_test   ] ; then echo -e "\n## ERROR installing sick_scan_xd_api_test\n"    ; else echo -e "install sick_scan_xd_api_test finished successfully." ; fi

popd

