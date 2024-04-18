#!/bin/bash

# killall and cleanup after exit
function killall_cleanup()
{
  rosnode kill -a
  killall sick_generic_caller
  pkill -f multiscan_sopas_test_server.py
  pkill -f multiscan_pcap_player.py
  pkill -f multiscan_perftest_player.py
  pkill -f multiscan_laserscan_msg_to_pointcloud.py
}

# Run example ros service calls
function call_service_examples()
{
  sleep 0.1 ; rosservice list
  sleep 0.1 ; rosservice call /multiScan/ColaMsg "{request: 'sMN IsSystemReady'}"                             # response: "sAN IsSystemReady 1"
  sleep 0.1 ; rosservice call /multiScan/ColaMsg "{request: 'sMN SetAccessMode 3 F4724744'}"                  # response: "sAN SetAccessMode 1"
  sleep 0.1 ; rosservice call /multiScan/ColaMsg "{request: 'sWN ScanDataEthSettings 1 +127 +0 +0 +1 +2115'}" # response: "sWA ScanDataEthSettings"
  sleep 0.1 ; rosservice call /multiScan/ColaMsg "{request: 'sWN ScanDataFormat 1'}"                          # response: "sWA ScanDataFormat"
  sleep 0.1 ; rosservice call /multiScan/ColaMsg "{request: 'sWN ScanDataPreformatting 1'}"                   # response: "sWA ScanDataPreformatting"
  sleep 0.1 ; rosservice call /multiScan/ColaMsg "{request: 'sWN ScanDataEnable 1'}"                          # response: "sWA ScanDataEnable"
  sleep 0.1 ; rosservice call /multiScan/ColaMsg "{request: 'sMN LMCstartmeas'}"                              # response: "sAN LMCstartmeas"
  sleep 0.1 ; rosservice call /multiScan/ColaMsg "{request: 'sMN Run'}"                                       # response: "sAN Run 1"
  sleep 0.1 ; rosservice call /multiScan/GetContaminationResult "{}"                                          # response: "sRA GetContaminationResult 0 0"
}  

# Run example ros service calls for filter settings
function call_service_filter_examples()
{
  sleep 0.1 ; rosservice call /multiScan/ColaMsg "{request: 'sRN FREchoFilter'}"                                                # response: "sRA FREchoFilter 1"
  sleep 0.1 ; rosservice call /multiScan/ColaMsg "{request: 'sRN LFPangleRangeFilter'}"                                         # response: "sRA LFPangleRangeFilter 0 C0490FF9 40490FF9 BFC90FF9 3FC90FF9 1"
  sleep 0.1 ; rosservice call /multiScan/ColaMsg "{request: 'sRN LFPlayerFilter'}"                                              # response: "sRA LFPlayerFilter 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"
  sleep 0.1 ; rosservice call /multiScan/ColaMsg "{request: 'sMN SetAccessMode 3 F4724744'}"                                    # response: "sAN SetAccessMode 1"
  sleep 0.1 ; rosservice call /multiScan/ColaMsg "{request: 'sWN FREchoFilter 1'}"                                              # response: "sWA FREchoFilter"
  sleep 0.1 ; rosservice call /multiScan/ColaMsg "{request: 'sWN LFPangleRangeFilter 0 C0490FF9 40490FF9 BFC90FF9 3FC90FF9 1'}" # response: "sWA LFPangleRangeFilter"
  sleep 0.1 ; rosservice call /multiScan/ColaMsg "{request: 'sWN LFPlayerFilter 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1'}"            # response: "sWA LFPlayerFilter"
  sleep 0.1 ; rosservice call /multiScan/ColaMsg "{request: 'sMN Run'}"                                                         # response: "sAN Run 1"
  sleep 0.1 ; rosservice call /multiScan/ColaMsg "{request: 'sRN FREchoFilter'}"                                                # response: "sRA FREchoFilter 1"
  sleep 0.1 ; rosservice call /multiScan/ColaMsg "{request: 'sRN LFPangleRangeFilter'}"                                         # response: "sRA LFPangleRangeFilter 0 C0490FF9 40490FF9 BFC90FF9 3FC90FF9 1"
  sleep 0.1 ; rosservice call /multiScan/ColaMsg "{request: 'sRN LFPlayerFilter'}"                                              # response: "sRA LFPlayerFilter 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"
}  

# start static transforms for laserscan messages (16 laserscan frame ids "world_0", "world_1", "world_2", ... "world_15"  for 16 layers are mapped to "world")
function run_laserscan_frame_transformers()
{
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world world_0  20 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world world_1  20 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world world_2  20 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world world_3  20 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world world_4  20 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world world_5  20 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world world_6  20 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world world_7  20 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world world_8  20 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world world_9  20 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world world_10 20 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world world_11 20 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world world_12 20 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world world_13 20 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world world_14 20 &
  rosrun tf static_transform_publisher 0 0 0 0 0 0 world world_15 20 &
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

# Run Multiscan136 emulator (sopas test server)
python3 ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 &
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_multiscan_emu_laserscan.rviz & 
sleep 1
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_multiscan_emu.rviz & 
sleep 1
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_multiscan_emu_360.rviz & 
sleep 1

# Start sick_generic_caller with sick_scansegment_xd
# Note: To verify laserscan messages, we configure laserscan_layer_filter:="1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1", i.e. a laserscan message is published for each segment, each layer and each echo.
# By default, laserscan messages are only activated for layer 5 (elevation -0.07 degree, max number of scan points)
# All laserscan messages are converted to pointcloud by multiScan_laserscan_msg_to_pointcloud.py using a hardcoded elevation table.
# Note: Option laserscan_layer_filter:="1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1" should not be used for performance tests.
echo -e "run_linux_ros1_simu_multiscan.bash: sick_scan_xd sick_multiscan.launch ..."
roslaunch sick_scan_xd sick_multiscan.launch hostname:="127.0.0.1" udp_receiver_ip:="127.0.0.1" laserscan_layer_filter:="1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1" scandataformat:=1 &
sleep 3 
run_laserscan_frame_transformers
# python3 ./src/sick_scan_xd/test/python/multiscan_laserscan_msg_to_pointcloud.py &

# Run example ros service calls
call_service_examples
call_service_filter_examples
sleep 3

# Play pcapng-files to emulate multiscan output
python3 ./src/sick_scan_xd/test/python/multiscan_perftest_player.py --udp_port=2115 --repeat=100 --send_rate=100 --verbose=0 --prompt=0
#echo -e "\nPlaying pcapng-files to emulate multiscan. Note: Start of UDP msgpacks in 20220915_mrs100_msgpack_output.pcapng takes a while...\n"
#python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20220915_mrs100_msgpack_output.pcapng --udp_port=2115 --repeat=1
#sleep 3

# Shutdown
echo -e "run_linux_ros1_simu_multiscan.bash finished, killing all processes ..."
killall_cleanup
popd
