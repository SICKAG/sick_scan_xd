#!/bin/bash

# Start tim7xx emulator and rviz
function start_tim7xx_emulator()
{
    echo -e "\nrun_linux_ros1_simu_add_transform: starting tim7xx emulation ...\n"
    roslaunch sick_scan_xd emulator_01_default.launch &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_tim7xx_add_transform.rviz --opengl 210 &
    sleep 1
}

# Start ldmrs emulator and rviz
function start_ldmrs_emulator()
{
    echo -e "\nrun_linux_ros1_simu_add_transform: starting ldmrs emulation ...\n"
    roslaunch sick_scan_xd test_server_ldmrs.launch &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_ldmrs_add_transform.rviz --opengl 210 &
    sleep 1
}

# Start multiScan emulator and rviz
function start_multiScan_emulator()
{
    echo -e "\nrun_linux_ros1_simu_add_transform: starting multiScan emulation ...\n"
    python3 ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_mrs100_add_transform.rviz --opengl 210 &
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

# Sets parameter add_transform_xyz_rpy dynamically, moves and rotates the pointcloud
function run_simu_transforms()
{
    nodename=$1
    delta=$2 # dx, dy in meter, e.g. 0.1
    echo -e "\nrun_linux_ros1_simu_add_transform.bash: setting transforms (translation in x,y)\n"
    for ((cnt=1;cnt<=1;cnt++)) ; do  # apply translation in x and y
        sleep 2 ; rosparam set $nodename "$delta,0.0,0,0,0,0"       # x=+0.3, y=+0.0, z=0, roll=0, pitch=0, yaw=0 deg
        sleep 2 ; rosparam set $nodename "$delta,$delta,0,0,0,0"    # x=+0.3, y=+0.3, z=0, roll=0, pitch=0, yaw=0 deg
        sleep 2 ; rosparam set $nodename "0.0,$delta,0,0,0,0"       # x=+0.3, y=+0.3, z=0, roll=0, pitch=0, yaw=0 deg
        sleep 2 ; rosparam set $nodename "-$delta,$delta,0,0,0,0"   # x=-0.3, y=+0.3, z=0, roll=0, pitch=0, yaw=0 deg
        sleep 2 ; rosparam set $nodename "-$delta,0.0,0,0,0,0"      # x=-0.3, y=+0.0, z=0, roll=0, pitch=0, yaw=0 deg
        sleep 2 ; rosparam set $nodename "-$delta,-$delta,0,0,0,0"  # x=-0.3, y=-0.3, z=0, roll=0, pitch=0, yaw=0 deg
        sleep 2 ; rosparam set $nodename "0.0,-$delta,0,0,0,0"      # x=+0.0, y=-0.3, z=0, roll=0, pitch=0, yaw=0 deg
        sleep 2 ; rosparam set $nodename "$delta,-$delta,0,0,0,0"   # x=+0.3, y=-0.3, z=0, roll=0, pitch=0, yaw=0 deg
        sleep 2 ; rosparam set $nodename "$delta,0.0,0,0,0,0"       # x=+0.3, y=+0.0, z=0, roll=0, pitch=0, yaw=0 deg
        sleep 2 ; rosparam set $nodename "0.0,0.0,0,0,0,0"          # x=+0.0, y=+0.0, z=0, roll=0, pitch=0, yaw=0 deg
    done
    echo -e "\nrun_linux_ros1_simu_add_transform.bash: setting transforms (rotation about yaw)\n"
    for ((cnt=1;cnt<=1;cnt++)) ; do  # apply rotation about yaw angle

        sleep 2 ; rosparam set $nodename "0,0,0,0.0000000,0.0000000,0.7853982"   # x=0, y=0, z=0, roll=0, pitch=0, yaw=45 deg
        sleep 2 ; rosparam set $nodename "0,0,0,0.0000000,0.0000000,1.5707963"   # x=0, y=0, z=0, roll=0, pitch=0, yaw=90 deg
        sleep 2 ; rosparam set $nodename "0,0,0,0.0000000,0.0000000,2.3561945"   # x=0, y=0, z=0, roll=0, pitch=0, yaw=135 deg
        sleep 2 ; rosparam set $nodename "0,0,0,0.0000000,0.0000000,3.1415926"   # x=0, y=0, z=0, roll=0, pitch=0, yaw=180 deg
        sleep 2 ; rosparam set $nodename "0,0,0,0.0000000,0.0000000,-2.3561945"  # x=0, y=0, z=0, roll=0, pitch=0, yaw=-135 deg
        sleep 2 ; rosparam set $nodename "0,0,0,0.0000000,0.0000000,-1.5707963"  # x=0, y=0, z=0, roll=0, pitch=0, yaw=-90 deg
        sleep 2 ; rosparam set $nodename "0,0,0,0.0000000,0.0000000,-0.7853982"  # x=0, y=0, z=0, roll=0, pitch=0, yaw=-45 deg
        sleep 2 ; rosparam set $nodename "0,0,0,0.0000000,0.0000000,0.0000000"   # x=0, y=0, z=0, roll=0, pitch=0, yaw=0 deg
    done
    waitUntilRvizClosed 2
    kill_simu
}


# Run sick_generic_caller with tim7xx and additional transform
function run_simu_tim7xx()
{
    start_tim7xx_emulator
    echo -e "\nrun_linux_ros1_simu_add_transform.bash: starting sick_scan_xd sick_tim_7xx.launch, no transform\n"
    roslaunch sick_scan_xd sick_tim_7xx.launch hostname:=127.0.0.1 add_transform_xyz_rpy:=0,0,0,0,0,0 add_transform_check_dynamic_updates:=true &
    sleep 5
    run_simu_transforms sick_tim_7xx/add_transform_xyz_rpy 0.3
    kill_simu
}

# Run sick_generic_caller with multiScan and additional transform
function run_simu_multiScan()
{
    start_multiScan_emulator
    echo -e "\nrun_linux_ros1_simu_add_transform.bash: starting sick_scan_xd sick_multiscan.launch, no transform\n"
    roslaunch sick_scan_xd sick_multiscan.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 add_transform_xyz_rpy:=0,0,0,0,0,0 add_transform_check_dynamic_updates:=true &
    sleep 5
    python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20210929_mrs100_token_udp.pcapng --udp_port=2115 --repeat=10 &
    run_simu_transforms sick_scansegment_xd/add_transform_xyz_rpy 2.0
    kill_simu
}

# Run sick_generic_caller with ldmrs and additional transform
function run_simu_ldmrs()
{
    start_ldmrs_emulator
    echo -e "\nrun_linux_ros1_simu_add_transform.bash: starting sick_scan_xd sick_ldmrs.launch, no transform\n"
    roslaunch sick_scan_xd sick_ldmrs.launch hostname:=127.0.0.1 add_transform_xyz_rpy:=0,0,0,0,0,0 add_transform_check_dynamic_updates:=true &
    sleep 5
    run_simu_transforms sick_ldmrs/add_transform_xyz_rpy 1.0 
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

# Run sick_generic_caller with additional transforms

run_simu_tim7xx
run_simu_multiScan
run_simu_ldmrs

echo -e "\nrun_linux_ros1_simu_add_transform.bash finished.\n"
popd

