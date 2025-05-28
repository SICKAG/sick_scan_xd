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

# Start sopas emulator
function start_sopas_emulator()
{
  python3 ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 --FREchoFilter=1 &
}

# Start pcapng player
function start_pcapng_player()
{
  python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230911-picoscan-compact.pcapng --udp_port=2115 --repeat=1000 &
}

# Start rviz
function start_rviz()
{
  killall -9 rviz2
  sleep 1
  ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_picoscan_emu.rviz & 
  sleep 1
  ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_picoscan_emu_360.rviz & 
  sleep 1
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

# Run multiScan/picoScan emulator (sopas test server), rviz2, sick_scan_xd and pcapng player
start_sopas_emulator
start_rviz
ros2 launch sick_scan_xd sick_picoscan.launch.py hostname:=127.0.0.1 udp_receiver_ip:="127.0.0.1" &
sleep 3
start_pcapng_player
sleep 20

# Kill and restart emulator, sick_scan_xd must reconnect
for i in {1..2}; do
  echo -e "\n##\n## killing picoscan emulator ...\n##\n\n"
  pkill -9 -f multiscan_sopas_test_server.py
  pkill -9 -f multiscan_pcap_player.py
  sleep 20
  echo -e "\n##\n## restarting picoscan emulator ...\n##\n\n"
  start_sopas_emulator
  sleep 3
  start_pcapng_player
  start_rviz
  sleep 20
done

# Shutdown
echo -e "\n##\n## run_linux_ros2_simu_picoscan_reconnect.bash finished, killing all processes ...\n##\n\n"
killall_cleanup
popd
