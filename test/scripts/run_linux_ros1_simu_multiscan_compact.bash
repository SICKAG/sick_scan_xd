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
popd
