#!/bin/bash

# killall and cleanup after exit
function killall_cleanup()
{
  sleep 1 ; killall -SIGINT rviz2
  sleep 1 ; killall -SIGINT sick_generic_caller
  sleep 1 ; pkill -f multiscan_sopas_test_server.py
  sleep 1 ; pkill -f multiscan_pcap_player.py
  sleep 1 ; killall -9 rviz2
  sleep 1 ; killall -9 sick_generic_caller
}

# 
# Run sick_picoScan on ROS2-Linux
# 

pushd ../../../..
printf "\033c"
if   [ -f /opt/ros/jazzy/setup.bash    ] ; then source /opt/ros/jazzy/setup.bash ; export QT_QPA_PLATFORM=xcb
elif [ -f /opt/ros/humble/setup.bash   ] ; then source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash     ] ; then source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/eloquent/setup.bash ] ; then source /opt/ros/eloquent/setup.bash
fi
source ./install/setup.bash
killall_cleanup
sleep 1
rm -rf ~/.ros/log
sleep 1

# Run multiScan/picoScan emulator (sopas test server)
python3 ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 --FREchoFilter=1 &
ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_picoscan_emu.rviz & 
sleep 1
ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_picoscan_emu_360.rviz & 
sleep 1

# Start sick_generic_caller with sick_picoscan with compact format
echo -e "run_lidar3d.bash: sick_scan_xd sick_picoscan.launch.py ..."
# ros2 launch sick_scan_xd sick_picoscan.launch.py hostname:=127.0.0.1 udp_receiver_ip:="127.0.0.1" scandataformat:=2 all_segments_min_deg:=-134 all_segments_max_deg:=135 &
# ros2 run --prefix 'gdb -ex run --args' sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_picoscan.launch hostname:=127.0.0.1 udp_receiver_ip:="127.0.0.1" scandataformat:=2
ros2 launch sick_scan_xd sick_picoscan.launch.py hostname:=127.0.0.1 udp_receiver_ip:="127.0.0.1" scandataformat:=2 &
sleep 3 # read -p "Press ENTER to continue..."

# Play picoScan pcapng-files with picoscan compact data
echo -e "\nPlaying pcapng-files to emulate picoscan with compact data format\n"
python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230911-picoscan-compact.pcapng --udp_port=2115 --repeat=1000
# Old pcapng files, require old configuration (all_segments_min_deg=-134 und all_segments_max_deg=+135)
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230204_timtwo_v0.18.0_compact_profile1.pcapng --udp_port=2115 --repeat=1
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
sleep 3

# Shutdown
echo -e "run_linux_ros2_simu_picoScan.bash finished, killing all processes ..."
killall_cleanup
popd
