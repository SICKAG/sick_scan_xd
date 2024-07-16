#!/bin/bash

# killall and cleanup after exit
function killall_cleanup()
{
  rosnode kill -a
  sleep 3
  killall sick_generic_caller
  killall static_transform_publisher
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
# Build octomap server and sick_scan_xd
# 

pushd ../../../..
source /opt/ros/noetic/setup.bash
killall_cleanup
printf "\033c"

REBUILD_ALL=0
if [ ! $REBUILD_ALL -eq 0 ] ; then
  pushd src
  if [ ! -d ./sick_scan_xd ] ; then git clone https://github.com/SICKAG/sick_scan_xd.git ; fi
  git clone https://github.com/OctoMap/octomap_ros.git
  git clone https://github.com/OctoMap/octomap_msgs.git
  git clone https://github.com/OctoMap/octomap_mapping.git
  popd
  # Set configuration for multiScan in file "./src/octomap_mapping/octomap_server/launch/octomap_mapping.launch" :
  # <param name="frame_id" type="string" value="world" />
  # <remap from="cloud_in" to="/cloud_unstructured_fullframe" />
  rm -rf ./build ./devel ./install ./build_isolated ./devel_isolated ./install_isolated ./log
  catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DCMAKE_ENABLE_EMULATOR=1 -Wno-dev
fi

# 
# Run sick_scan_xd multiScan simulation
# 

source ./install_isolated/setup.bash
sleep 1 ; python3 ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 &
sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_multiscan_octomap.rviz &
sleep 1 ; roslaunch sick_scan_xd sick_multiscan.launch hostname:="127.0.0.1" udp_receiver_ip:="127.0.0.1" sw_pll_only_publish:=False &
sleep 3 ; python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20240604_multiscan_01_motionless.pcapng --udp_port=-1 --repeat=1 --verbose=0 --send_rate=100 --filter=pcap_filter_multiscan_hildesheim &
# sleep 3 ; python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20240604_multiscan_02_moving_planar_defaultsettings.pcapng --udp_port=-1 --repeat=1 --verbose=0 --send_rate=100 --filter=pcap_filter_multiscan_hildesheim &
# sleep 3 ; python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20240604_multiscan_03_moving_planar_anglefilter_-45_45deg.pcapng --udp_port=-1 --repeat=1 --verbose=0 --send_rate=100 --filter=pcap_filter_multiscan_hildesheim &
# Settings in sick_multiscan.launch for 20240604_multiscan_03_moving_planar_anglefilter_-45_45deg.pcapng with +/-45 deg angle filter:
# <arg name="host_LFPangleRangeFilter" default="1 -45.0 +45.0 -90.0 +90.0 1" />
# <arg name="host_set_LFPangleRangeFilter" default="True" />
# Default settings in sick_multiscan.launch for all other pcang-files:
# <arg name="host_LFPangleRangeFilter" default="0 -180.0 +179.0 -90.0 +90.0 1" />
# <arg name="host_set_LFPangleRangeFilter" default="False" />

# 
# Run octomap server
# To visualize octomap in rviz: add MarkerArray topic "/occupied_cells_vis_array" and Map topic "/projected_map"
# 

sleep 1 ; roslaunch octomap_server octomap_mapping.launch &

#
# Save octomap and shutdown
#

waitUntilRvizClosed 120
if [ -f ./octomap_multiscan.bt ] ; then rm -f ./octomap_multiscan.bt ; fi
rosrun octomap_server octomap_saver -f ./octomap_multiscan.bt ; sleep 5
echo -e "octomap simulation finished, killing all processes ..."
killall_cleanup

#
# Publish and display the saved octomap
#

roscore &
sleep 1 ; rosrun octomap_server octomap_server_node ./octomap_multiscan.bt &
sleep 5 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_multiscan_octomap.rviz &
waitUntilRvizClosed 15
killall_cleanup
popd
