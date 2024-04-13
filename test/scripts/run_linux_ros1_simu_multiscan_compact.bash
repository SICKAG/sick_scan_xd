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
  sleep 3 ; rosnode kill -a ; killall -9 sick_generic_caller
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

# Run multiscan simulation
function run_multiscan_simu()
{
  testcase=$1
  if [ $testcase == 0 ] ; then
    sick_scan_xd_args=(sick_multiscan.launch hostname:="127.0.0.1" udp_receiver_ip:="127.0.0.1" scandataformat:=2 laserscan_layer_filter:="1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1")
    multiscan_pcap_player=(--pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20231009-multiscan-compact-imu-01.pcapng --udp_port=-1 --repeat=1 --verbose=0 --filter=pcap_filter_multiscan_hildesheim --max_seconds=15)
  elif [ $testcase == 1 ] ; then
    sick_scan_xd_args=(sick_multiscan.launch hostname:="127.0.0.1" udp_receiver_ip:="127.0.0.1" scandataformat:=2 host_LFPangleRangeFilter:="1 -10.0 +30.0 -90.0 +90.0 1" host_set_LFPangleRangeFilter:="True")
    multiscan_pcap_player=(--pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20231019_multiscan_compact_azimuthrange-10+30.pcapng --udp_port=-1 --repeat=1 --verbose=0 --filter=pcap_filter_multiscan_hildesheim)
  elif [ $testcase == 2 ] ; then
    sick_scan_xd_args=(sick_multiscan.launch hostname:="127.0.0.1" udp_receiver_ip:="127.0.0.1" scandataformat:=1 host_LFPangleRangeFilter:="1 -10.0 +30.0 -90.0 +90.0 1" host_set_LFPangleRangeFilter:="True")
    multiscan_pcap_player=(--pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20231019_multiscan_msgpack_azimuthrange-10+30.pcapng --udp_port=-1 --repeat=1 --verbose=0 --filter=pcap_filter_multiscan_hildesheim)
  elif [ $testcase == 3 ] ; then
    sick_scan_xd_args=(sick_multiscan.launch hostname:="127.0.0.1" udp_receiver_ip:="127.0.0.1" scandataformat:=2 host_LFPangleRangeFilter:="1 -10.0 +30.0 -90.0 +90.0 1" host_set_LFPangleRangeFilter:="True")
    multiscan_pcap_player=(--pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20231018_multiscan_azimuthrange-10+30.pcapng --udp_port=-1 --repeat=1 --verbose=0 --filter=pcap_filter_multiscan_hildesheim)
  elif [ $testcase == 4 ] ; then
    sick_scan_xd_args=(sick_multiscan.launch hostname:="127.0.0.1" udp_receiver_ip:="127.0.0.1" scandataformat:=2 host_LFPangleRangeFilter:="1 -10.0 +60.0 -90.0 +90.0 1" host_set_LFPangleRangeFilter:="True")
    multiscan_pcap_player=(--pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20231018_multiscan_azimuthrange-10+60.pcapng --udp_port=-1 --repeat=1 --verbose=0 --filter=pcap_filter_multiscan_hildesheim)
  elif [ $testcase == 5 ] ; then
    sick_scan_xd_args=(sick_multiscan.launch hostname:="127.0.0.1" udp_receiver_ip:="127.0.0.1" scandataformat:=2 host_LFPangleRangeFilter:="1 -10.0 +90.0 -90.0 +90.0 1" host_set_LFPangleRangeFilter:="True")
    multiscan_pcap_player=(--pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20231018_multiscan_azimuthrange-10+90.pcapng --udp_port=-1 --repeat=1 --verbose=0 --filter=pcap_filter_multiscan_hildesheim)
  else
    echo -n "\n## ERROR run_multiscan_simu: invalid testcase\n"
    return 1 # return error
  fi
  
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
  roslaunch sick_scan_xd "${sick_scan_xd_args[@]}" &
  sleep 3 
  
  # Map all laserscan massages to frame id "world_6"
  run_laserscan_frame_transformers
  
  # Start polar to cartesian pointcloud converter
  # polar pointcloud on topic "/cloud_polar_unstructured_fullframe" with fields "i", "range", "azimuth", "elevation" (input)
  # cartesian pointcloud on topic "/cloud_polar_to_cartesian" with fields "x", "y", "z", "intensity" (output)
  python3 ./src/sick_scan_xd/test/python/polar_to_cartesian_pointcloud_ros1.py --polar_topic="/cloud_polar_unstructured_fullframe" --cartesian_topic="/cloud_polar_to_cartesian" &
  
  # Play compact pcapng-files to emulate multiscan output
  echo -e "\nPlaying pcapng-files to emulate multiscan ...\n"
  python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py "${multiscan_pcap_player[@]}"
  sleep 3
  return 0 # return success
}

# 
# Run multiscan on ROS1-Linux
# 

pushd ../../../..
printf "\033c"
source /opt/ros/noetic/setup.bash
source ./devel_isolated/setup.bash
killall_cleanup
sleep 1
rm -rf ~/.ros/log
sleep 1

# Run multiscan simulation with default configuration
run_multiscan_simu 0
# echo -e "\nrun_multiscan_simu: shutdown simu...\n\n"
# killall_cleanup
# exit

# Run testcases with angle range filter
killall_cleanup ; run_multiscan_simu 1
killall_cleanup ; run_multiscan_simu 2
killall_cleanup ; run_multiscan_simu 3
killall_cleanup ; run_multiscan_simu 4
killall_cleanup ; run_multiscan_simu 5

# Play compact pcapng-files to emulate multiScan output
rostopic echo -p /multiScan/imu &
echo -e "\nPlaying pcapng-files to emulate multiScan, using compact format ...\n"
# 20231009-multiscan-compact-imu-01.pcapng: compact, all layers, last echo, imu, max. 30 sec.
python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20231009-multiscan-compact-imu-01.pcapng --udp_port=-1 --repeat=1 --verbose=0 --max_seconds=15 --filter=pcap_filter_multiscan_hildesheim
# 20230607-multiscan-compact-v4-5layer.pcapng: compact, 5 layer, no imu
python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230607-multiscan-compact-v4-5layer.pcapng --udp_port=-1 --repeat=1 --verbose=0 --max_seconds=15 --filter=pcap_filter_multiscan_hildesheim
# pcapng files for measurement of imu latency (compact, hires0 layer, imu and lidar oscillating)
# rm -rf /tmp/imu_latency.csv ./*.csv
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20231009-multiscan-compact-hires0-imu-latency-01-10periods.pcapng --udp_port=-1 --repeat=1 --verbose=0 --filter=pcap_filter_multiscan_hildesheim
# mv /tmp/imu_latency.csv ./20231009_multiscan_timestamp_azimuth_imuacceleration.csv
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20231011-multiscan-compact-hires0-imu-latency-01-10periods.pcapng --udp_port=-1 --repeat=1 --verbose=0 --filter=pcap_filter_multiscan_hildesheim
# mv /tmp/imu_latency.csv ./20231011a_multiscan_timestamp_azimuth_imuacceleration.csv
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20231011-multiscan-compact-hires0-imu-latency-02-10periods.pcapng --udp_port=-1 --repeat=1 --verbose=0 --filter=pcap_filter_multiscan_hildesheim
# mv /tmp/imu_latency.csv ./20231011b_multiscan_timestamp_azimuth_imuacceleration.csv
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20231011-multiscan-compact-hires0-imu-latency-03-manuell.pcapng --udp_port=-1 --repeat=1 --verbose=0 --filter=pcap_filter_multiscan_hildesheim
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20231024-multiscan-imu-02.pcapng --udp_port=-1 --repeat=1 --verbose=0 --filter=pcap_filter_multiscan_hildesheim
# mv /tmp/imu_latency.csv ./20231024_multiscan_timestamp_azimuth_imuacceleration.csv
sleep 3

# Shutdown
echo -e "run_multiscan.bash finished, killing all processes ..."
killall_cleanup
rosnode kill -a
popd
