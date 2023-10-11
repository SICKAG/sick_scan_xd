#!/bin/bash

# killall and cleanup after exit
function killall_cleanup()
{
  sleep 1 ; killall -SIGINT rviz2
  sleep 1 ; killall -SIGINT sick_generic_caller
  sleep 1 ; pkill -f multiscan_sopas_test_server.py
  sleep 1 ; killall -9 rviz2
  sleep 1 ; killall -9 sick_generic_caller
}

# 
# Run sick_scansegment_xd on ROS2-Linux
# 

pushd ../../../..
printf "\033c"
if [ -f /opt/ros/eloquent/setup.bash ] ; then source /opt/ros/eloquent/setup.bash ; fi
if [ -f /opt/ros/foxy/setup.bash     ] ; then source /opt/ros/foxy/setup.bash     ; fi
source ./install/setup.bash
killall_cleanup
sleep 1
rm -rf ~/.ros/log
sleep 1

# Run multiscan emulator (sopas test server)
python3 ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 &
sleep 1
ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_multiscan_emu.rviz & 
sleep 1
ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_multiscan_emu_360.rviz & 
sleep 1

# Start sick_generic_caller with sick_scansegment_xd
echo -e "run_lidar3d.bash: sick_scan_xd sick_multiscan.launch.py ..."
ros2 launch sick_scan_xd sick_multiscan.launch.py hostname:=127.0.0.1 udp_receiver_ip:="127.0.0.1" scandataformat:=2 &
sleep 3 

# Play pcapng-files to emulate multiScan compact V4 scandata
echo -e "\nPlaying pcapng-files to emulate multiScan\n"
python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230607-multiscan-compact-v4-5layer.pcapng --udp_port=2115 --repeat=1
sleep 3

# Shutdown
echo -e "run_lidar3d.bash finished, killing all processes ..."
killall_cleanup
popd
