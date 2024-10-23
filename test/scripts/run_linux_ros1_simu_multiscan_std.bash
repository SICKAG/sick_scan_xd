#!/bin/bash

# killall and cleanup after exit
function killall_cleanup()
{
  rosnode kill multiScan
  sleep 3
  killall sick_generic_caller
  killall rviz
  killall static_transform_publisher
  pkill -f multiscan_sopas_test_server.py
  pkill -f multiscan_pcap_player.py
  pkill -f polar_to_cartesian_pointcloud_ros1.py
  pkill -f sick_scan_xd_simu.py
  pkill -f sopas_json_test_server.py
  killall -9 sick_generic_caller
  sleep 3
}

# start static transforms for laserscan messages (all laserscan frame ids "world_1", "world_2", "world_3", ... "world_16"  for all layers are mapped to "world_6")
function run_laserscan_frame_transformers()
{
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world_6 world_1  100 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world_6 world_2  100 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world_6 world_3  100 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world_6 world_4  100 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world_6 world_5  100 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world_6 world_7  100 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world_6 world_8  100 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world_6 world_9  100 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world_6 world_10 100 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world_6 world_11 100 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world_6 world_12 100 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world_6 world_13 100 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world_6 world_14 100 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world_6 world_15 100 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world_6 world_16 100 &
}

# 
# Run multiscan simulation  with default configuration on ROS1-Linux
# 

pushd ../../../..
printf "\033c"
source /opt/ros/noetic/setup.bash
source ./devel_isolated/setup.bash
killall_cleanup
sleep 1

# Run multiscan emulator (sopas test server)
python3 ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 &
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_multiscan_emu.rviz & 
sleep 1
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_multiscan_laserscan_360.rviz & 
sleep 1
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_multiscan_emu_360.rviz & 
sleep 1
  
# Start sick_generic_caller with multiscan in compact format
echo -e "run_multiscan.bash: sick_scan_xd sick_multiscan.launch ..."
roslaunch sick_scan_xd sick_multiscan.launch hostname:="127.0.0.1" udp_receiver_ip:="127.0.0.1" laserscan_layer_filter:="1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1" &
sleep 3 
  
# Map all laserscan massages to frame id "world_6"
run_laserscan_frame_transformers
  
# Start polar to cartesian pointcloud converter
# polar pointcloud on topic "/cloud_polar_unstructured_fullframe" with fields "i", "range", "azimuth", "elevation" (input)
# cartesian pointcloud on topic "/cloud_polar_to_cartesian" with fields "x", "y", "z", "intensity" (output)
python3 ./src/sick_scan_xd/test/python/polar_to_cartesian_pointcloud_ros1.py --polar_topic="/cloud_polar_unstructured_fullframe" --cartesian_topic="/cloud_polar_to_cartesian" &
 
# Play compact pcapng-files to emulate multiscan output
echo -e "\nPlaying pcapng-files to emulate multiscan ...\n"
python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20231009-multiscan-compact-imu-01.pcapng --udp_port=-1 --repeat=1 --verbose=0 --filter=pcap_filter_multiscan_hildesheim --max_seconds=60

# Shutdown
echo -e "run_multiscan.bash finished, killing all processes ..."
killall_cleanup
popd

