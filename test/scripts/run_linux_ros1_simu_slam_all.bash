#!/bin/bash

# Wait for a given amount of seconds, or until 'q' or 'Q' pressed, or until rviz is closed
function waitUntilRvizClosed()
{
    duration_sec=$1
    sleep 1
    for ((cnt=1;cnt<=$duration_sec;cnt++)) ; do  
        echo -e "sick_scan_xd emulation running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
        if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
        rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
        if [ $rviz_running -lt 1 ] ; then break ; fi
    done
}

# Shutdown simulation, kill sick_scan_emulator and sick_generic_caller
function kill_simu()
{
    rosnode kill -a ; sleep 1
    pkill -f sopas_json_test_server.py ; pkill -f multiscan_sopas_test_server.py ; pkill -f multiscan_pcap_player.py ; sleep 1
    killall sick_generic_caller ; sleep 1
    killall sick_scan_emulator ; sleep 1
}

# Run SLAM simulation
function run_simu()
{
    # Simulation parameter
    sick_scan_xd_launch_args=$1
    hector_slam_args=$2
    sopas_json_test_server_args=$3
    
    # Start rviz
    rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_slam_all.rviz --opengl 210 &
    
    # Start sick_scan_xd emulator
    echo -e "sick_scan_xd slam simulation: starting sopas_json_test_server.py ${sopas_json_test_server_args} --verbosity=0\n"
    python3 ./src/sick_scan_xd/test/python/sopas_json_test_server.py ${sopas_json_test_server_args} --verbosity=0 &
    sleep 1
    
    # Start sick_scan_xd driver
    echo -e "sick_scan_xd slam simulation: launching sick_scan_xd ${sick_scan_xd_launch_args} hostname:=127.0.0.1 tf_publish_rate:=0 sw_pll_only_publish:=False\n"
    roslaunch sick_scan_xd ${sick_scan_xd_launch_args} hostname:=127.0.0.1 tf_publish_rate:=0 sw_pll_only_publish:=False &
    sleep 1
    
    # Start SLAM
    echo -e "sick_scan_xd slam simulation: launching sick_scan_xd test_200_slam_ros1_hector.launch\n"
    roslaunch sick_scan_xd test_200_slam_ros1_hector.launch ${hector_slam_args} &
    sleep 1
    
    # Wait max. 30 sec or for 'q' or 'Q' or until rviz is closed
    echo -e "sick_scan_xd slam simulation running\n"
    waitUntilRvizClosed 30
    echo -e "sick_scan_xd slam simulation finished, shutdown all ros nodes\n"
    kill_simu
}

# Run SLAM simulation with picoScan lidar
function run_picoscan_simu()
{
    # Simulation parameter
    sick_scan_xd_launch_args=$1
    hector_slam_args=$2
    pcap_player_args=$3
    
    # Start rviz
    rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_slam_multiscan.rviz --opengl 210 &
    #/cloud_all_fields_fullframe
    
    # Start sopas emulator
    echo -e "sick_scan_xd slam simulation: starting multiscan_sopas_test_server.py\n"
    python3 ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 --FREchoFilter=1 &
    sleep 1
    
    # Start sick_scan_xd driver
    echo -e "sick_scan_xd slam simulation: launching sick_scan_xd ${sick_scan_xd_launch_args} hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 tf_publish_rate:=0 sw_pll_only_publish:=False\n"
    roslaunch sick_scan_xd ${sick_scan_xd_launch_args} hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 tf_publish_rate:=0 sw_pll_only_publish:=False scandataformat:=2 host_FREchoFilter:=0 &
    sleep 1
    
    # Start SLAM
    echo -e "sick_scan_xd slam simulation: launching sick_scan_xd test_200_slam_ros1_hector.launch\n"
    roslaunch sick_scan_xd test_200_slam_ros1_hector.launch ${hector_slam_args} &
    sleep 1
    
    # Play pcapng-file
    python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py ${pcap_player_args} --udp_port=2115 --repeat=10 &
    
    # Wait max. 30 sec or for 'q' or 'Q' or until rviz is closed
    echo -e "sick_scan_xd slam simulation running\n"
    waitUntilRvizClosed 30
    echo -e "sick_scan_xd slam simulation finished, shutdown all ros nodes\n"
    kill_simu
}

# Run SLAM simulation with multiScan lidar
function run_multiscan_simu()
{
    # Simulation parameter
    sick_scan_xd_launch_args=$1
    hector_slam_args=$2
    pcap_player_args=$3
    
    # Start rviz
    rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_slam_multiscan.rviz --opengl 210 &
    #/cloud_all_fields_fullframe
    
    # Start sopas emulator
    echo -e "sick_scan_xd slam simulation: starting multiscan_sopas_test_server.py\n"
    python3 ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 &
    sleep 1
    
    # Start sick_scan_xd driver
    echo -e "sick_scan_xd slam simulation: launching sick_scan_xd ${sick_scan_xd_launch_args} hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 tf_publish_rate:=0 sw_pll_only_publish:=False\n"
    roslaunch sick_scan_xd ${sick_scan_xd_launch_args} hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 tf_publish_rate:=0 sw_pll_only_publish:=False scandataformat:=2 laserscan_layer_filter:="1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1" &
    sleep 1
    
    # Start SLAM
    echo -e "sick_scan_xd slam simulation: launching sick_scan_xd test_200_slam_ros1_hector.launch\n"
    roslaunch sick_scan_xd test_200_slam_ros1_hector.launch ${hector_slam_args} &
    sleep 1
    
    # Play pcapng-file
    python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py ${pcap_player_args} --udp_port=-1 --repeat=1 --verbose=0 --filter=pcap_filter_multiscan_hildesheim --max_seconds=15 &
    
    # Wait max. 30 sec or for 'q' or 'Q' or until rviz is closed
    echo -e "sick_scan_xd slam simulation running\n"
    waitUntilRvizClosed 30
    echo -e "sick_scan_xd slam simulation finished, shutdown all ros nodes\n"
    kill_simu
}

# Setup
# To use sick_scan_xd and ROS1 hector slam, clone sick_scan_xd and hector_slam and rebuild:
# cd src
# git clone -b master https://github.com/SICKAG/sick_scan_xd.git
# git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
# cd ..
# catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DCMAKE_ENABLE_EMULATOR=1 -Wno-dev
printf "\033c"
pushd ../../../..
source /opt/ros/noetic/setup.bash
source ./devel_isolated/setup.bash

# Start roscore if not yet running
roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
if [ $roscore_running -lt 1 ] ; then 
  roscore &
  sleep 3
fi

# Run SLAM simulations
pcapng_folder=`(pwd)`/src/sick_scan_xd/test/emulator/scandata
run_simu "sick_nav_350.launch nav_do_initial_mapping:=True" "scan_topic:=/scan scan_layer_0_frame_id:=cloud_POS_000_DIST1 cloud_frame_id:=cloud" "--json_file=${pcapng_folder}/20230126_nav350_4reflectors_moving.pcapng.json --tcp_port=2112 --scandata_id=\"sAN?mNPOSGetData\" --send_rate=8"
run_picoscan_simu "sick_picoscan.launch" "scan_topic:=/sick_picoscan/scan_fullframe scan_layer_0_frame_id:=world_1 cloud_frame_id:=world" "--pcap_filename=${pcapng_folder}/20230911-picoscan-compact.pcapng"
# TODO: to be continued for other lidars
# run_multiscan_simu "sick_multiscan.launch" "scan_topic:=/multiScan/scan_fullframe scan_layer_0_frame_id:=world_6 cloud_frame_id:=world" "--pcap_filename=${pcapng_folder}/20231009-multiscan-compact-imu-01.pcapng"
# run_simu "sick_mrs_1xxx.launch" "--json_file=${pcapng_folder}/20210722_143600_ros2_mrs1104_sick_scan_xd.pcapng.json --tcp_port=2112"

popd

