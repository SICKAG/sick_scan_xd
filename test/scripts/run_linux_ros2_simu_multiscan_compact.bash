#!/bin/bash

# killall and cleanup after exit
function killall_cleanup()
{
  sleep 3 ; killall sick_generic_caller 2>/dev/null
  sleep 3 ; killall rviz2 2>/dev/null
  sleep 3 ; pkill -f multiscan_sopas_test_server.py 2>/dev/null
  sleep 3 ; pkill -f multiscan_pcap_player.py 2>/dev/null
  sleep 3 ; killall -9 rviz2 2>/dev/null
  sleep 3 ; killall -9 sick_generic_caller 2>/dev/null
}
# 
# Run sick_scansegment_xd on ROS2-Linux
# 

pushd ../../../..
printf "\033c"
# Prefer ROSDISTRO if already defined
if [ -n "$ROSDISTRO" ] && [ -f "/opt/ros/$ROSDISTRO/setup.bash" ]; then
    source "/opt/ros/$ROSDISTRO/setup.bash"
else
    # Fallback: probe known distros
    for distro in jazzy humble foxy eloquent; do
        if [ -f "/opt/ros/$distro/setup.bash" ]; then
            source "/opt/ros/$distro/setup.bash"
            break
        fi
    done
fi

if [ -z "$ROSDISTRO" ]; then
    echo "ERROR: No supported ROS distribution found"
    exit 1
fi

# Optional: only for jazzy (Qt fix)
if [ "$ROSDISTRO" = "jazzy" ]; then
    export QT_QPA_PLATFORM=xcb
fi
echo -e "Running ./install/setup.bash"
echo -e "ROS distro: $ROSDISTRO"
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
echo -e "Start sick_scan_xd sick_multiscan.launch.py"
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
