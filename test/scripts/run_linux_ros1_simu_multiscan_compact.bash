#!/bin/bash

# killall and cleanup after exit
function killall_cleanup()
{
  rosnode kill -a
  killall sick_generic_caller
  pkill -f multiscan_sopas_test_server.py
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

# Run multiscan emulator (sopas test server)
python3 ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 &
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_multiscan_emu.rviz & 
sleep 1
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_multiscan_emu_360.rviz & 
sleep 1

# Start sick_generic_caller with multiscan in compact format
echo -e "run_multiscan.bash: sick_scan_xd sick_multiscan.launch ..."
roslaunch sick_scan_xd sick_multiscan.launch hostname:="127.0.0.1" udp_receiver_ip:="127.0.0.1" scandataformat:=2 &
sleep 3 

# Play compact pcapng-files to emulate MRS100 output
echo -e "\nPlaying pcapng-files to emulate MRS100, compact format V4, 5 layer ...\n"
python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230607-multiscan-compact-v4-5layer.pcapng --udp_port=2115 --repeat=1 --verbose=0
sleep 3

# Shutdown
echo -e "run_multiscan.bash finished, killing all processes ..."
killall_cleanup
popd
