#!/bin/bash

# killall and cleanup after exit
function killall_cleanup()
{
  rosnode kill -a
  sleep 1
  killall sick_generic_caller
  sleep 1
  pkill -f multiscan_sopas_test_server.py
  pkill -f multiscan_pcap_player.py
  killall -9 sick_generic_caller
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
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_picoscan_laserscan_360.rviz & 
sleep 1
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_picoscan_emu_360.rviz & 
sleep 1

# Start sick_generic_caller with sick_picoscan with compact format
echo -e "run_lidar3d.bash: sick_scan_xd sick_picoscan.launch ..."
roslaunch sick_scan_xd sick_picoscan.launch hostname:="127.0.0.1" udp_receiver_ip:="127.0.0.1" scandataformat:=2 host_FREchoFilter:=0 &
sleep 3 # read -p "Press ENTER to continue..."

# Play picoscan pcapng-file with picoscan compact-data
python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230911-picoscan-compact.pcapng --udp_port=2115 --repeat=5
# Old pcapng files, require old configuration (all_segments_min_deg=-134 und all_segments_max_deg=+135), i.e.
# roslaunch sick_scan_xd sick_picoscan.launch hostname:="127.0.0.1" udp_receiver_ip:="127.0.0.1" scandataformat:=2 all_segments_min_deg:=-134 all_segments_max_deg:=135
# echo -e "\nPlaying pcapng-files to emulate picoscan with compact data format\n"
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230204_timtwo_v0.18.0_compact_profile1.pcapng --udp_port=2115 --repeat=100
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230204_timtwo_v0.18.0_compact_profile2.pcapng --udp_port=2115 --repeat=1
# # python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230204_timtwo_v0.18.0_compact_profile3.pcapng --udp_port=2115 --repeat=1
# # python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230204_timtwo_v0.18.0_compact_profile4.pcapng --udp_port=2115 --repeat=1
# # python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230204_timtwo_v0.18.0_compact_profile5.pcapng --udp_port=2115 --repeat=1
# # python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230204_timtwo_v0.18.0_compact_profile6.pcapng --udp_port=2115 --repeat=1
# # python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230204_timtwo_v0.18.0_compact_profile7.pcapng --udp_port=2115 --repeat=1
# # python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230204_timtwo_v0.18.0_compact_profile8.pcapng --udp_port=2115 --repeat=1
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile1_echofilter_off_fogfilter_off.pcapng --udp_port=2115 --repeat=1
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile2_echofilter_off_fogfilter_off.pcapng --udp_port=2115 --repeat=1
# #python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile3_echofilter_off_fogfilter_off.pcapng --udp_port=2115 --repeat=1
# #python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile4_echofilter_off_fogfilter_off.pcapng --udp_port=2115 --repeat=1
# #python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile5_echofilter_off_fogfilter_off.pcapng --udp_port=2115 --repeat=1
# #python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile6_echofilter_off_fogfilter_off.pcapng --udp_port=2115 --repeat=1
# #python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile7_echofilter_off_fogfilter_off.pcapng --udp_port=2115 --repeat=1
# #python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile8_echofilter_off_fogfilter_off.pcapng --udp_port=2115 --repeat=1
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile1_echofilter_lastecho_fogfilter_off.pcapng --udp_port=2115 --repeat=1
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile2_echofilter_lastecho_fogfilter_off.pcapng --udp_port=2115 --repeat=1
# #python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile3_echofilter_lastecho_fogfilter_off.pcapng --udp_port=2115 --repeat=1
# #python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile4_echofilter_lastecho_fogfilter_off.pcapng --udp_port=2115 --repeat=1
# #python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile5_echofilter_lastecho_fogfilter_off.pcapng --udp_port=2115 --repeat=1
# #python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile6_echofilter_lastecho_fogfilter_off.pcapng --udp_port=2115 --repeat=1
# #python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile7_echofilter_lastecho_fogfilter_off.pcapng --udp_port=2115 --repeat=1
# #python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile8_echofilter_lastecho_fogfilter_off.pcapng --udp_port=2115 --repeat=1
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile1_echofilter_lastecho_fogfilter_1.pcapng --udp_port=2115 --repeat=1
# #python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile8_echofilter_lastecho_fogfilter_1.pcapng --udp_port=2115 --repeat=1
# sleep 3

# Shutdown
echo -e "run_linux_ros1_simu_timtwo.bash finished, killing all processes ..."
killall_cleanup
killall_cleanup
popd
