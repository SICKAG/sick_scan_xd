#!/bin/bash


function killall_cleanup()
{
    rosnode kill multiScan
    sleep 3
    killall sick_generic_caller
    killall rviz
    killall static_transform_publisher
    pkill -f multiscan_sopas_test_server.py
    pkill -f multiscan_pcap_player.py
    pkill -f polar_to_cartesian_pointcloud_ros1.py
    pkill -f sick_scan_xd_simu.py
    pkill -f sopas_json_test_server.py
    sleep 3 ; rosnode kill -a ; killall -9 sick_generic_caller
    sleep 3
}

function prompt()
{
    read -n 1 -p "Press any key to continue..."
}

function run_sick_scan_xd_simu()
{
    cfg_args="$@"
    echo -e "Running sick_scan_xd simu.py $cfg_args"
    python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py $cfg_args
    simu_exit_status=$?
    killall_cleanup
    echo -e "simu exit_status for $cfg_args: $simu_exit_status"
    if [ $simu_exit_status -eq 0 ] ; then 
        echo -e "\nSUCCESS: sick_scan_xd test passed for configuration $cfg_args\n"
    else 
        echo -e "\n## ERROR: sick_scan_xd test FAILED for configuration $cfg_args\n"
        prompt
    fi    
}

# 
# Run sick_scan_xd simulation for Linux-ROS1
# 

printf "\033c"
pushd ../../../..
source /opt/ros/noetic/setup.bash
source ./devel_isolated/setup.bash

run_sick_scan_xd_simu --ros=noetic --cfg=./src/sick_scan_xd/test/docker/data/multiscan_compact_test01_cfg.json
run_sick_scan_xd_simu --ros=noetic --cfg=./src/sick_scan_xd/test/docker/data/picoscan_compact_test01_cfg.json
run_sick_scan_xd_simu --ros=noetic --cfg=./src/sick_scan_xd/test/docker/data/mrs1xxx_test01_cfg.json

# Use option --run_seconds=3600 (or any other value) for manual verification of all messages (pointcloud, laserscan, imu, etc)
# Use option --save_messages_jsonfile=received_messages.json (or any other filename) to save a reference json file after manual verification of all messages
# Example for MRS-1xxx:
# run_sick_scan_xd_simu --ros=noetic --cfg=./src/sick_scan_xd/test/docker/data/mrs1xxx_test01_cfg.json --save_messages_jsonfile=mrs1xxx_test01_ref_messages.json --run_seconds=5

popd

