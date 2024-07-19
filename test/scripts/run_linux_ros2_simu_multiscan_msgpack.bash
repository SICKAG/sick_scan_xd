#!/bin/bash

# killall and cleanup after exit
function killall_cleanup()
{
  sleep 1 ; killall -SIGINT rviz2
  sleep 1 ; killall -SIGINT sick_generic_caller
  sleep 1 ; pkill -f multiscan_sopas_test_server.py
  sleep 1 ; pkill -f multiscan_pcap_player.py
  sleep 1 ; pkill -f multiscan_laserscan_msg_to_pointcloud.py
  sleep 1 ; killall -9 rviz2
  sleep 1 ; killall -9 sick_generic_caller
}

# Run example ros service calls
function call_service_examples()
{
  sleep 0.1 ; ros2 service list
  sleep 0.1 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sMN IsSystemReady'}"                             # response: "sAN IsSystemReady 1"
  sleep 0.1 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sMN SetAccessMode 3 F4724744'}"                  # response: "sAN SetAccessMode 1"
  sleep 0.1 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sWN ScanDataEthSettings 1 +127 +0 +0 +1 +2115'}" # response: "sWA ScanDataEthSettings"
  sleep 0.1 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sWN ScanDataFormat 1'}"                          # response: "sWA ScanDataFormat"
  sleep 0.1 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sWN ScanDataPreformatting 1'}"                   # response: "sWA ScanDataPreformatting"
  sleep 0.1 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sWN ScanDataEnable 1'}"                          # response: "sWA ScanDataEnable"
  sleep 0.1 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sMN LMCstartmeas'}"                              # response: "sAN LMCstartmeas"
  sleep 0.1 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sMN Run'}"                                       # response: "sAN Run 1"
  sleep 0.1 ; ros2 service call /GetContaminationResult sick_scan_xd/srv/GetContaminationResultSrv "{}"                           # response: "sRA ContaminationResult 0 0"
}  

# Run example ros service calls for filter settings
function call_service_filter_examples()
{
  sleep 0.1 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sRN FREchoFilter'}"                                                # response: "sRA FREchoFilter 1"
  sleep 0.1 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sRN LFPangleRangeFilter'}"                                         # response: "sRA LFPangleRangeFilter 0 C0490FF9 40490FF9 BFC90FF9 3FC90FF9 1"
  sleep 0.1 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sRN LFPlayerFilter'}"                                              # response: "sRA LFPlayerFilter 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"
  sleep 0.1 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sMN SetAccessMode 3 F4724744'}"                                    # response: "sAN SetAccessMode 1"
  sleep 0.1 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sWN FREchoFilter 1'}"                                              # response: "sWA FREchoFilter"
  sleep 0.1 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sWN LFPangleRangeFilter 0 C0490FF9 40490FF9 BFC90FF9 3FC90FF9 1'}" # response: "sWA LFPangleRangeFilter"
  sleep 0.1 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sWN LFPlayerFilter 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1'}"            # response: "sWA LFPlayerFilter"
  sleep 0.1 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sMN Run'}"                                                         # response: "sAN Run 1"
  sleep 0.1 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sRN FREchoFilter'}"                                                # response: "sRA FREchoFilter 1"
  sleep 0.1 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sRN LFPangleRangeFilter'}"                                         # response: "sRA LFPangleRangeFilter 0 C0490FF9 40490FF9 BFC90FF9 3FC90FF9 1"
  sleep 0.1 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sRN LFPlayerFilter'}"                                              # response: "sRA LFPlayerFilter 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"
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
ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_multiscan_emu_laserscan.rviz & 
sleep 1
ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_multiscan_emu.rviz & 
sleep 1
ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_multiscan_emu_360.rviz & 
sleep 1

# Start sick_generic_caller with sick_scansegment_xd
# Note: To verify laserscan messages, we configure laserscan_layer_filter:="1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1", i.e. a laserscan message is published for each segment, each layer and each echo.
# By default, laserscan messages are only activated for layer 5 (elevation -0.07 degree, max number of scan points)
# All laserscan messages are converted to pointcloud by multiscan_laserscan_msg_to_pointcloud.py using a hardcoded elevation table.
# Note: Option laserscan_layer_filter:="1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1" should not be used for performance tests.
# ros2 launch sick_scan_xd sick_multiscan.launch.py hostname:=127.0.0.1 udp_receiver_ip:="127.0.0.1" laserscan_layer_filter:="1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1" &
echo -e "run_multiscan.bash: sick_scan_xd sick_multiscan.launch.py ..."
ros2 launch sick_scan_xd sick_multiscan.launch.py hostname:=127.0.0.1 udp_receiver_ip:="127.0.0.1" scandataformat:=1 &
sleep 3 
# python3 ./src/sick_scan_xd/test/python/multiscan_laserscan_msg_to_pointcloud.py &
# sleep 1

# Run example ros service calls
call_service_examples
call_service_filter_examples
sleep 3

# Play pcapng-files to emulate multiScan output
python3 ./src/sick_scan_xd/test/python/multiscan_perftest_player.py --udp_port=2115 --repeat=100 --send_rate=100 --verbose=0 --prompt=0
# echo -e "\nPlaying pcapng-files to emulate multiScan. Note: Start of UDP msgpacks in 20220915_multiscan_msgpack_output.pcapng takes a while...\n"
# python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20220915_mrs100_msgpack_output.pcapng --udp_port=2115 --repeat=1 --verbose=0 --prompt=0
sleep 3

# Shutdown
echo -e "run_multiscan.bash finished, killing all processes ..."
killall_cleanup
popd
