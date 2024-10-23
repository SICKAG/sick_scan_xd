#!/bin/bash

# killall and cleanup after exit
function killall_cleanup()
{
  sleep 3 ; killall sick_generic_caller
  sleep 3 ; killall rviz2
  sleep 3 ; pkill -f multiscan_sopas_test_server.py
  sleep 3 ; pkill -f multiscan_pcap_player.py
  sleep 3 ; killall -9 rviz2
  sleep 3 ; killall -9 sick_generic_caller
}

# 
# Run sick_scansegment_xd on ROS2-Linux
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

# Run multiscan emulator (sopas test server)
python3 ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 &
sleep 1
ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_multiscan_emu.rviz & 
sleep 1
ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_multiscan_emu_360.rviz & 
sleep 1

# Start sick_generic_caller with sick_scansegment_xd
echo -e "run_lidar3d.bash: sick_scan_xd sick_multiscan.launch.py ..."
# ros2 run --prefix 'gdb -ex run --args' sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_multiscan.launch hostname:=127.0.0.1 udp_receiver_ip:="127.0.0.1" scandataformat:=2
ros2 launch sick_scan_xd sick_multiscan.launch.py hostname:=127.0.0.1 udp_receiver_ip:="127.0.0.1" scandataformat:=2 &
sleep 3 

# Play pcapng-files to emulate multiScan compact V4 scandata
echo -e "\nPlaying pcapng-files to emulate multiScan\n"
# 20231009-multiscan-compact-imu-01.pcapng: compact, all layers, last echo, imu, max. 30 sec.
python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20231009-multiscan-compact-imu-01.pcapng --udp_port=-1 --repeat=1 --verbose=0 --max_seconds=15 --filter=pcap_filter_multiscan_hildesheim
# 20230607-multiscan-compact-v4-5layer.pcapng: compact, 5 layer, no imu
python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230607-multiscan-compact-v4-5layer.pcapng --udp_port=-1 --repeat=1 --verbose=0 --max_seconds=15 --filter=pcap_filter_multiscan_hildesheim


# Shutdown
echo -e "run sick_scansegment_xd finished, killing all processes ..."
killall_cleanup
popd
