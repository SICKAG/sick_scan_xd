#!/bin/bash

# killall and cleanup after exit
function killall_cleanup()
{
  rosnode kill -a
  killall -9 lidar3d_mrs100_recv
  sudo pkill -f sopas_test_server.py 
}

# Run example ros service calls
function call_service_examples()
{
  sleep 1 ; rosservice list
  sleep 1 ; rosservice call /ColaMsg "{request: 'sMN IsSystemReady'}"                             # response: "sAN IsSystemReady 1"
  sleep 1 ; rosservice call /ColaMsg "{request: 'sMN SetAccessMode 3 F4724744'}"                  # response: "sAN SetAccessMode 1"
  sleep 1 ; rosservice call /ColaMsg "{request: 'sWN ScanDataEthSettings 1 +127 +0 +0 +1 +2115'}" # response: "sWA ScanDataEthSettings"
  sleep 1 ; rosservice call /ColaMsg "{request: 'sWN ScanDataFormatSettings 1 1'}"                # response: "sWA ScanDataFormatSettings"
  sleep 1 ; rosservice call /ColaMsg "{request: 'sWN ScanDataEnable 1'}"                          # response: "sWA ScanDataEnable"
  sleep 1 ; rosservice call /ColaMsg "{request: 'sMN LMCstartmeas'}"                              # response: "sAN LMCstartmeas"
  sleep 1 ; rosservice call /ColaMsg "{request: 'sMN Run'}"                                       # response: "sAN Run 1"
}  

# Run example ros service calls for filter settings
function call_service_filter_examples()
{
  sleep 1 ; rosservice call /ColaMsg "{request: 'sRN FREchoFilter'}"                                                # response: "sRA FREchoFilter 1"
  sleep 1 ; rosservice call /ColaMsg "{request: 'sRN LFPangleRangeFilter'}"                                         # response: "sRA LFPangleRangeFilter 0 C0490FF9 40490FF9 BFC90FF9 3FC90FF9 1"
  sleep 1 ; rosservice call /ColaMsg "{request: 'sRN LFPlayerFilter'}"                                              # response: "sRA LFPlayerFilter 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"
  sleep 1 ; rosservice call /ColaMsg "{request: 'sMN SetAccessMode 3 F4724744'}"                                    # response: "sAN SetAccessMode 1"
  sleep 1 ; rosservice call /ColaMsg "{request: 'sWN FREchoFilter 1'}"                                              # response: "sWA FREchoFilter"
  sleep 1 ; rosservice call /ColaMsg "{request: 'sWN LFPangleRangeFilter 0 C0490FF9 40490FF9 BFC90FF9 3FC90FF9 1'}" # response: "sWA LFPangleRangeFilter"
  sleep 1 ; rosservice call /ColaMsg "{request: 'sWN LFPlayerFilter 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1'}"            # response: "sWA LFPlayerFilter"
  sleep 1 ; rosservice call /ColaMsg "{request: 'sMN Run'}"                                                         # response: "sAN Run 1"
  sleep 1 ; rosservice call /ColaMsg "{request: 'sRN FREchoFilter'}"                                                # response: "sRA FREchoFilter 1"
  sleep 1 ; rosservice call /ColaMsg "{request: 'sRN LFPangleRangeFilter'}"                                         # response: "sRA LFPangleRangeFilter 0 C0490FF9 40490FF9 BFC90FF9 3FC90FF9 1"
  sleep 1 ; rosservice call /ColaMsg "{request: 'sRN LFPlayerFilter'}"                                              # response: "sRA LFPlayerFilter 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"
}  

# 
# Run sick_lidar3d on ROS1-Linux
# 

pushd ../../../..
printf "\033c"
source /opt/ros/noetic/setup.bash
source ./install/setup.bash
killall_cleanup
sleep 1
rm -rf ~/.ros/log
sleep 1

# Run mrs100 emulator (sopas test server)
python3 ./src/sick_scan_xd/test/python/mrs100_sopas_test_server.py --tcp_port=2111 --cola_binary=0 &
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_mrs100_emu.rviz & 
sleep 1
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_mrs100_emu_360.rviz & 
sleep 1

# Start lidar3d_mrs100_recv
echo -e "run_lidar3d.bash: sick_scan sick_lidar3d.launch ..."
roslaunch sick_scan sick_lidar3d.launch mrs100_ip:="127.0.0.1" mrs100_dst_ip:="127.0.0.1" &
sleep 3 # read -p "Press ENTER to continue..."

# Run example ros service calls
#call_service_examples
#sleep 3

# Play pcapng-files to emulate MRS100 output
python3 ./src/sick_scan_xd/test/python/mrs100_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20210929_mrs100_token_udp.pcapng --udp_port=2115
python3 ./src/sick_scan_xd/test/python/mrs100_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20210929_mrs100_cola-a-start-stop-scandata-output.pcapng --udp_port=2115

# Run example ros service calls
#call_service_examples
#call_service_filter_examples
#sleep 3

# Shutdown
echo -e "run_lidar3d.bash finished, killing all processes ..."
killall_cleanup
popd
