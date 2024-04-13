#!/bin/bash

# killall and cleanup after exit
function killall_cleanup()
{
  rosnode kill -a
  killall sick_generic_caller
  pkill -f multiscan_sopas_test_server.py
  pkill -f multiscan_pcap_player.py
}

# Run example ros service calls
function call_service_examples()
{
  sleep 0.1 ; rosservice list
  sleep 0.1 ; rosservice call /sick_picoscan/ColaMsg "{request: 'sMN IsSystemReady'}"                             # response: "sAN IsSystemReady 1"
  sleep 0.1 ; rosservice call /sick_picoscan/ColaMsg "{request: 'sMN SetAccessMode 3 F4724744'}"                  # response: "sAN SetAccessMode 1"
  sleep 0.1 ; rosservice call /sick_picoscan/ColaMsg "{request: 'sWN ScanDataEthSettings 1 +127 +0 +0 +1 +2115'}" # response: "sWA ScanDataEthSettings"
  sleep 0.1 ; rosservice call /sick_picoscan/ColaMsg "{request: 'sWN ScanDataFormat 1'}"                          # response: "sWA ScanDataFormat"
  sleep 0.1 ; rosservice call /sick_picoscan/ColaMsg "{request: 'sWN ScanDataEnable 1'}"                          # response: "sWA ScanDataEnable"
  sleep 0.1 ; rosservice call /sick_picoscan/ColaMsg "{request: 'sMN LMCstartmeas'}"                              # response: "sAN LMCstartmeas"
  sleep 0.1 ; rosservice call /sick_picoscan/ColaMsg "{request: 'sMN Run'}"                                       # response: "sAN Run 1"
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

# Run sopas test server (picoscan emulator)
python3 ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 --FREchoFilter=1 &
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_picoscan_emu.rviz & 
sleep 1
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_picoscan_emu_360.rviz & 
sleep 1

# Start sick_generic_caller with sick_picoscan with msgpack format
echo -e "run_lidar3d.bash: sick_scan_xd sick_picoscan.launch ..."
roslaunch sick_scan_xd sick_picoscan.launch hostname:="127.0.0.1" udp_receiver_ip:="127.0.0.1" scandataformat:=1 &
sleep 3 # read -p "Press ENTER to continue..."

# Run example ros service calls
call_service_examples
sleep 3

# Play picoScan pcapng-files with sgpack-data, mix 30 and 60 degree segments
echo -e "\nPlaying pcapng-file to emulate picoscan...\n"
python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230911-picoscan-msgpack.pcapng --udp_port=2115 --repeat=1
# Old pcapng files, require old configuration (all_segments_min_deg=-134 und all_segments_max_deg=+135)
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230315-picoscan.pcapng --udp_port=2115 --repeat=1
# echo -e "\nPlaying pcapng-files to emulate picoScan. Note: Start of UDP msgpacks in 20221010_timtwo.pcapng takes a while...\n"
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230204_timtwo_v0.18.0_msgpack_profile4.pcapng --udp_port=2115 --repeat=1
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230204_timtwo_v0.18.0_msgpack_profile6.pcapng --udp_port=2115 --repeat=1
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230204_timtwo_v0.18.0_msgpack_profile3.pcapng --udp_port=2115 --repeat=1
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230204_timtwo_v0.18.0_msgpack_profile7.pcapng --udp_port=2115 --repeat=1
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230204_timtwo_v0.18.0_msgpack_profile2.pcapng --udp_port=2115 --repeat=1
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230204_timtwo_v0.18.0_msgpack_profile8.pcapng --udp_port=2115 --repeat=1
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230204_timtwo_v0.18.0_msgpack_profile5.pcapng --udp_port=2115 --repeat=1
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230204_timtwo_v0.18.0_msgpack_profile1.pcapng --udp_port=2115 --repeat=1
# echo -e "\nPlaying pcapng-files to emulate picoScan. Note: Start of UDP msgpacks in 20221010_timtwo.pcapng takes a while...\n"
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20221010_timtwo.pcapng --udp_port=2115 --repeat=1
sleep 3

# Shutdown
echo -e "run_linux_ros1_simu_picoScan.bash finished, killing all processes ..."
killall_cleanup
popd
