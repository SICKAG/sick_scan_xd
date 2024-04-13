#!/bin/bash

# killall and cleanup after exit
function killall_cleanup()
{
  rosnode kill -a
  killall sick_generic_caller
  pkill -f multiscan_sopas_test_server.py
  pkill -f multiscan_pcap_player.py
}

# 
# Run sick_scansegment_xd on ROS1-Linux
# 

pushd ../../../..
printf "\033c"
source /opt/ros/noetic/setup.bash
# source ./install_isolated/setup.bash
source ./devel_isolated/setup.bash
killall_cleanup
sleep 1
rm -rf ~/.ros/log
sleep 1

# Run sopas test server (emulate picoscan)
python3 ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 --FREchoFilter=1 &
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_picoscan_emu.rviz & 
sleep 1
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_picoscan_single_echo_laserscan_360.rviz & 
sleep 1
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_picoscan_emu_360.rviz & 
sleep 1

# Start sick_generic_caller with sick_picoscan with compact format
echo -e "run_lidar3d.bash: sick_scan_xd sick_picoscan.launch ..."
roslaunch sick_scan_xd sick_picoscan_single_echo.launch hostname:="127.0.0.1" udp_receiver_ip:="127.0.0.1" &
sleep 3 # read -p "Press ENTER to continue..."
(rostopic echo /sick_picoscan/scan_fullframe | grep frame_id) &

# Play picoscan pcapng-file with picoscan compact-data
python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230911-picoscan-compact.pcapng --udp_port=2115 --repeat=1
sleep 3

# Shutdown
echo -e "run_linux_ros1_simu_picoScan.bash finished, killing all processes ..."
killall_cleanup
popd
