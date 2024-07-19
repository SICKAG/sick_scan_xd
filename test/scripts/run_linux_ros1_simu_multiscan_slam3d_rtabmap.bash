#!/bin/bash

# killall and cleanup after exit
function killall_cleanup()
{
  rosnode kill -a
  sleep 3
  killall sick_generic_caller
  killall static_transform_publisher
  killall rtabmap_viz
  pkill -f multiscan_sopas_test_server.py
  pkill -f multiscan_pcap_player.py
}

# Wait for a given amount of time in seconds, or until 'q' or 'Q' was pressed, or until rviz is closed
function waitUntilRvizClosed()
{
    duration_sec=$1
    sleep 1
    for ((cnt=1;cnt<=$duration_sec;cnt++)) ; do  
        echo -e "sick_scan_xd running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
        if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
        rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
        if [ $rviz_running -lt 1 ] ; then break ; fi
    done
}

# 
# Build rtabmap and sick_scan_xd
#
# rtabmap can be installed using a binary distribution:
# sudo apt install ros-$ROS_DISTRO-rtabmap-ros
# 

pushd ../../../..
source /opt/ros/noetic/setup.bash
killall_cleanup
printf "\033c"

REBUILD_ALL=0
BUILD_UPDATE=0
if [ ! $REBUILD_ALL -eq 0 ] ; then
  sudo apt-get install libboost-all-dev
  sudo apt-get install libeigen3-dev
  sudo apt-get install libsdl-image1.2-dev
  sudo apt-get install libsdl-dev
  sudo apt-get install ros-noetic-nav-msgs
  sudo apt-get install ros-noetic-tf2-sensor-msgs
  sudo apt-get install ros-noetic-imu-filter-madgwick
  sudo apt-get install python3-wstool
  sudo apt-get install ros-noetic-scan-tools
  pushd /tmp
  git clone https://github.com/introlab/rtabmap.git rtabmap
  git clone https://github.com/ethz-asl/libnabo.git libnabo
  git clone https://github.com/ethz-asl/libpointmatcher.git libpointmatcher  
  mkdir -p libnabo/build && pushd libnabo/build
  cmake ..
  cmake --build .
  sudo cmake --build . --target install
  popd  
  mkdir -p libpointmatcher/build && pushd libpointmatcher/build
  cmake ..
  make -j4
  sudo make install
  popd
  sudo ldconfig    
  mkdir -p rtabmap/build && pushd rtabmap/build
  cmake ..
  make -j4
  sudo make install
  popd  
  sudo ldconfig
  popd
  pushd src
  git clone https://github.com/ros-planning/navigation.git
  git clone https://github.com/ros-planning/navigation_msgs.git
  git clone https://github.com/introlab/rtabmap_ros.git
  if [ ! -d ./sick_scan_xd ] ; then git clone https://github.com/SICKAG/sick_scan_xd.git ; fi
  popd
  rm -rf ./build ./devel ./install ./build_isolated ./devel_isolated ./install_isolated ./log
  catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DCMAKE_ENABLE_EMULATOR=1 -Wno-dev
  sudo ldconfig
fi
if [ ! $BUILD_UPDATE -eq 0 ] ; then
  catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DCMAKE_ENABLE_EMULATOR=1 -Wno-dev
  sudo ldconfig
fi

# 
# Run sick_scan_xd multiScan simulation with rtabmap
# Note: run "rosrun rtabmap_slam rtabmap --params" to see all rtabmap parameters and their meaning
# 

source ./install_isolated/setup.bash
cp ./src/sick_scan_xd/launch/sick_multiscan*.launch ./install_isolated/share/sick_scan_xd/launch/ # optional update in case of launchfile changes after the last build
rm -f ~/.ros/rtabmap.db
sleep 1 ; python3 ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 &

# rviz: Add topics "/cloud_unstructured_fullframe", "/rtabmap/grid_map" and "/rtabmap/localization_pose" to view the navigational map and localization pose
sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_multiscan_rtabmap.rviz &

sleep 1 ; roslaunch sick_scan_xd sick_multiscan_rtabmap.launch hostname:="127.0.0.1" udp_receiver_ip:="127.0.0.1" &

# sleep 3 ; python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20240604_multiscan_01_motionless.pcapng --udp_port=-1 --repeat=10 --verbose=0 --send_rate=100 --filter=pcap_filter_multiscan_hildesheim &
sleep 3 ; python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20240604_multiscan_02_moving_planar_defaultsettings.pcapng --udp_port=-1 --repeat=10 --verbose=0 --send_rate=0 --filter=pcap_filter_multiscan_hildesheim &
# sleep 3 ; python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20240604_multiscan_03_moving_planar_anglefilter_-45_45deg.pcapng --udp_port=-1 --repeat=10 --verbose=0 --send_rate=100 --filter=pcap_filter_multiscan_hildesheim &
# Settings in sick_multiscan.launch for 20240604_multiscan_03_moving_planar_anglefilter_-45_45deg.pcapng with +/-45 deg angle filter:
# <arg name="host_LFPangleRangeFilter" default="1 -45.0 +45.0 -90.0 +90.0 1" />
# <arg name="host_set_LFPangleRangeFilter" default="True" />
# Default settings in sick_multiscan.launch for all other pcang-files:
# <arg name="host_LFPangleRangeFilter" default="0 -180.0 +179.0 -90.0 +90.0 1" />
# <arg name="host_set_LFPangleRangeFilter" default="False" />

#
# Switch to mapping and/or localization mode
#

# sleep 1 ; rosservice call /rtabmap/reset_odom_to_pose "{x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}"
sleep   3 ; rosservice call /sick_scan_xd/sick_multiscan/ColaMsg "{request: 'sMN IsSystemReady'}" # query sensor state
sleep   1 ; rosservice call /rtabmap/resume                # resume if previously paused
sleep   1 ; rosservice call /rtabmap/trigger_new_map       # start a new map
sleep   1 ; rosservice call /rtabmap/set_mode_mapping      # set mapping mode
sleep  60 ; rosservice call /rtabmap/set_mode_localization # set localization mode

#
# Shutdown after 240 seconds
#

waitUntilRvizClosed 240
echo -e "rtabmap simulation finished, killing all processes ..."
killall_cleanup
popd
