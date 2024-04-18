#!/bin/bash

# Start tim7xx emulator and rviz
function start_tim7xx_emulator()
{
    echo -e "\nrun_linux_ros1_simu_add_transform: starting tim7xx emulation ...\n"
    roslaunch sick_scan_xd emulator_01_default.launch &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_tim7xx_add_transform.rviz --opengl 210 &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_tim7xx_add_transform_origin.rviz --opengl 210 &
    sleep 1
}

# Start ldmrs emulator and rviz
function start_ldmrs_emulator()
{
    echo -e "\nrun_linux_ros1_simu_add_transform: starting ldmrs emulation ...\n"
    roslaunch sick_scan_xd test_server_ldmrs.launch &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_ldmrs_add_transform.rviz --opengl 210 &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_ldmrs_add_transform_origin.rviz --opengl 210 &
    sleep 1
}

# Start multiScan emulator and rviz
function start_multiScan_emulator()
{
    echo -e "\nrun_linux_ros1_simu_add_transform: starting multiScan emulation ...\n"
    python3 ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_mrs100_add_transform.rviz --opengl 210 &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_mrs100_add_transform_origin.rviz --opengl 210 &
    sleep 1
}

# Wait for max 40 seconds, or until 'q' or 'Q' pressed, or until rviz is closed
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

# Shutdown simulation, kill all nodes and processes
function kill_simu()
{
    echo -e "Finishing sick_scan_xd emulation, shutdown ros nodes\n"
    rosnode kill -a ; sleep 1
    killall sick_generic_caller ; sleep 1
    killall sick_scan_emulator ; sleep 1
    pkill -f multiscan_sopas_test_server.py
    pkill -f multiscan_pcap_player.py
}

# Run sick_generic_caller with tim7xx and additional transform
function run_simu_tim7xx()
{
    tx=$1 ; ty=$2 ; tz=$3 ; roll=$4 ; pitch=$5 ; yaw=$6 ; duration_sec=$7
    echo -e "\nrun_linux_ros1_simu_add_transform.bash: starting sick_scan_xd sick_tim_7xx.launch with additional transform ($tx, $ty, $tz, $roll, $pitch, $yaw)\n"
    start_tim7xx_emulator
    roslaunch sick_scan_xd sick_tim_7xx.launch hostname:=127.0.0.1 add_transform_xyz_rpy:=$tx,$ty,$tz,$roll,$pitch,$yaw &
    rosrun tf static_transform_publisher $tx $ty $tz $yaw $pitch $roll cloud origin 100 &
    waitUntilRvizClosed $duration_sec
    kill_simu
}

# Run sick_generic_caller with multiScan and additional transform
function run_simu_multiScan()
{
    tx=$1 ; ty=$2 ; tz=$3 ; roll=$4 ; pitch=$5 ; yaw=$6 ; duration_sec=$7
    echo -e "\nrun_linux_ros1_simu_add_transform.bash: starting sick_scan_xd sick_multiscan.launch (multiScan) with additional transform ($tx, $ty, $tz, $roll, $pitch, $yaw)\n"
    start_multiScan_emulator
    roslaunch sick_scan_xd sick_multiscan.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 add_transform_xyz_rpy:=$tx,$ty,$tz,$roll,$pitch,$yaw &
    rosrun tf static_transform_publisher $tx $ty $tz $yaw $pitch $roll world origin 100 &
    python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20210929_mrs100_token_udp.pcapng --udp_port=2115 --repeat=1
    kill_simu
}

# Run sick_generic_caller with ldmrs and additional transform
function run_simu_ldmrs()
{
    tx=$1 ; ty=$2 ; tz=$3 ; roll=$4 ; pitch=$5 ; yaw=$6 ; duration_sec=$7
    echo -e "\nrun_linux_ros1_simu_add_transform.bash: starting sick_scan_xd sick_ldmrs.launch with additional transform ($tx, $ty, $tz, $roll, $pitch, $yaw)\n"
    start_ldmrs_emulator
    roslaunch sick_scan_xd sick_ldmrs.launch hostname:=127.0.0.1 add_transform_xyz_rpy:=$tx,$ty,$tz,$roll,$pitch,$yaw &
    rosrun tf static_transform_publisher $tx $ty $tz $yaw $pitch $roll cloud origin 100 &
    waitUntilRvizClosed $duration_sec
    kill_simu
}

printf "\033c"
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash ] ; then source /opt/ros/melodic/setup.bash   ; fi
if [ -f /opt/ros/noetic/setup.bash  ] ; then source /opt/ros/noetic/setup.bash    ; fi
if [ -f ./devel_isolated/setup.bash ] ; then source ./devel_isolated/setup.bash   ; fi

# Start roscore if not yet running
roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
if [ $roscore_running -lt 1 ] ; then 
  roscore &
  sleep 3
fi

# Run sick_generic_caller without additional transform (default)
echo -e "\nrun_linux_ros1_simu_add_transform.bash: starting sick_scan_xd sick_tim_7xx.launch, no transform\n"
start_tim7xx_emulator
roslaunch sick_scan_xd sick_tim_7xx.launch hostname:=127.0.0.1 &
rosrun tf static_transform_publisher 0 0 0 0 0 0 cloud origin 100 &
waitUntilRvizClosed 10
kill_simu

# Run sick_generic_caller with tim7xx and additional transforms
run_simu_tim7xx 0 0 0 0.0000000 0.0000000 0.7853982 40 # tim7xx, x=0, y=0, z=0, roll=0, pitch=0, yaw=45 deg, duration_sec=40
run_simu_tim7xx 0 0 0 0.0000000 0.7853982 0.0000000 15 # tim7xx, x=0, y=0, z=0, roll=0, pitch=45, yaw=0 deg, duration_sec=15
run_simu_tim7xx 0 0 0 0.7853982 0.0000000 0.0000000 15 # tim7xx, x=0, y=0, z=0, roll=45, pitch=0, yaw=0 deg, duration_sec=15
run_simu_tim7xx  0.5 -0.5 -0.5 -0.7853982 -0.7853982 -0.7853982 15 # tim7xx, x=+0.5, y=-0.5, z=-0.5, roll=-45, pitch=-45, yaw=-45 deg, duration_sec=15
run_simu_tim7xx  0.5 -0.5 -1.0 -1.0471976  1.0471976  1.0471976 15 # tim7xx, x=+0.5, y=-0.5, z=-1.0, roll=-60, pitch=+60, yaw=+60 deg, duration_sec=15
run_simu_tim7xx -0.5  0.5 -1.0  2.0943951 -2.0943951  2.0943951 15 # tim7xx, x=-0.5, y=+0.5, z=-1.0, roll=120, pitch=-120,yaw=120 deg, duration_sec=15

# Run sick_generic_caller with multiScan and additional transforms
run_simu_multiScan 0 0 0 0.0000000 0.0000000 0.0000000            # multiScan, x=0, y=0, z=0, roll=0, pitch=0, yaw=0 deg
run_simu_multiScan 0 0 0 0.0000000 0.0000000 0.7853982            # multiScan, x=0, y=0, z=0, roll=0, pitch=0, yaw=45 deg
run_simu_multiScan 0.5 -0.5 -0.5 -0.7853982 -0.7853982 -0.7853982 # multiScan, x=+0.5, y=-0.5, z=-0.5, roll=-45, pitch=-45, yaw=-45 deg

# Run sick_generic_caller with ldmrs and additional transforms
run_simu_ldmrs 0 0 0 0.0000000 0.0000000 0.0000000 10            # ldmrs, x=0, y=0, z=0, roll=0, pitch=0, yaw=0 deg, duration_sec=15
run_simu_ldmrs 0 0 0 0.0000000 0.0000000 0.7853982 15            # ldmrs, x=0, y=0, z=0, roll=0, pitch=0, yaw=45 deg, duration_sec=15
run_simu_ldmrs 0.5 -0.5 -0.5 -0.7853982 -0.7853982 -0.7853982 15 # ldmrs, x=+0.5, y=-0.5, z=-0.5, roll=-45, pitch=-45, yaw=-45 deg, duration_sec=15

echo -e "\nrun_linux_ros1_simu_add_transform.bash finished.\n"
popd

