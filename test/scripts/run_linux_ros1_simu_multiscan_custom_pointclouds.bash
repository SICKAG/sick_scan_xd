#!/bin/bash

# Wait for a given amount of time in seconds, until 'q' or 'Q' pressed, or until rviz is closed (whichever comes first)
function waitUntilRvizClosed()
{
    duration_sec=$1
    sleep 1
    for ((cnt=1;cnt<=$duration_sec;cnt++)) ; do  
        echo -e "sick_scan_xd running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
        if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
        rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
        if [ $rviz_running -lt 1 ] ; then break ; fi
    done
}

# killall and cleanup after exit
function killall_cleanup()
{
  rosnode kill -a
  killall sick_generic_caller
  pkill -f multiscan_sopas_test_server.py  
  pkill -f multiscan_pcap_player.py
  pkill -f multiscan_perftest_player.py
  killall static_transform_publisher
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
source ./devel_isolated/setup.bash
killall_cleanup
sleep 1
rm -rf ~/.ros/log
sleep 1

# Run Multiscan136 emulator (sopas test server)
python3 ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 &
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_multiscan_custom_fullframe.rviz & 
sleep 1
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_cfg_multiscan_custom_segments.rviz & 
sleep 1

# Start sick_generic_caller with sick_multiscan
echo -e "run_linux_ros1_simu_multiscan_custom_pointclouds.bash: sick_scan_xd sick_multiscan.launch ..."
# Run multiscan with all 20 predefined pointclouds
roslaunch sick_scan_xd sick_multiscan.launch hostname:="127.0.0.1" udp_receiver_ip:="127.0.0.1" scandataformat:=1 custom_pointclouds:="cloud_unstructured_segments cloud_polar_unstructured_segments cloud_unstructured_fullframe cloud_unstructured_echo1 cloud_unstructured_echo1_segments cloud_unstructured_echo2 cloud_unstructured_echo2_segments cloud_unstructured_echo3 cloud_unstructured_echo3_segments cloud_unstructured_reflector cloud_unstructured_reflector_segments cloud_structured_hires0 cloud_structured_hires0_segments cloud_structured_hires1 cloud_structured_hires1_segments cloud_structured cloud_structured_segments cloud_all_fields_segments cloud_all_fields_fullframe" &
sleep 3
run_laserscan_frame_transformers

# Play pcapng-files to emulate multiScan output (use --echos=1 for performance tests with 1 echo, or --echos=3 for customized pointclouds)
echo -e "run_linux_ros1_simu_multiscan_custom_pointclouds.bash: multiscan_perftest_player.py ..."
# python3 ./src/sick_scan_xd/test/python/multiscan_perftest_player.py --udp_port=2115 --repeat=1000 --send_rate=100 --echos=1 --verbose=0 --prompt=0 &
python3 ./src/sick_scan_xd/test/python/multiscan_perftest_player.py --udp_port=2115 --repeat=1000 --send_rate=100 --echos=3 --verbose=0 --prompt=0 &
waitUntilRvizClosed 30

# Run multiscan with predefined pointclouds (fullframe and segment)
declare -a custom_pointcloud_configuration=(
  "cloud_unstructured_segments cloud_unstructured_fullframe cloud_all_fields_segments cloud_all_fields_fullframe"
  # "cloud_unstructured_segments cloud_unstructured_fullframe"
  # "cloud_unstructured_echo1 cloud_unstructured_echo1_segments"
  # "cloud_unstructured_echo2 cloud_unstructured_echo2_segments"
  # "cloud_unstructured_echo3 cloud_unstructured_echo3_segments"
  # "cloud_unstructured_reflector cloud_unstructured_reflector_segments"
  # "cloud_structured_hires0 cloud_structured_hires0_segments"
  # "cloud_structured_hires1 cloud_structured_hires1_segments"
  # "cloud_structured cloud_structured_segments"
  # "cloud_all_fields_segments cloud_all_fields_fullframe"
)
for custom_pointcloud_cfg in "${custom_pointcloud_configuration[@]}" ; do
   pkill -f multiscan_sopas_test_server.py ; killall sick_generic_caller ; sleep 3 ; killall -9 sick_generic_caller ; sleep 1
   echo -e "Run multiscan with predefined pointcloud: custom_pointclouds:=\"$custom_pointcloud_cfg\" ..."
   python3 ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 &
   roslaunch sick_scan_xd sick_multiscan.launch hostname:="127.0.0.1" udp_receiver_ip:="127.0.0.1" scandataformat:=1 custom_pointclouds:="$custom_pointcloud_cfg" &
   waitUntilRvizClosed 10
done

# Shutdown
echo -e "run_linux_ros1_simu_multiscan_custom_pointclouds.bash finished, killing all processes ..."
killall_cleanup
popd
