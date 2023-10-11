#!/bin/bash

# Wait for a given amount of seconds, or until 'q' or 'Q' pressed, or until rviz is closed
function waitUntilRvizClosed()
{
    duration_sec=$1
    sleep 1
    for ((cnt=1;cnt<=$duration_sec;cnt++)) ; do  
        echo -e "mrs1104 emulation running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
        if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
        rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
        if [ $rviz_running -lt 1 ] ; then break ; fi
    done
}

# Shutdown simulation, kill sick_scan_emulator and sick_generic_caller
function kill_simu()
{
    rosservice call /sick_mrs_1xxx/SickScanExit "{}" ; sleep 1
    killall sick_generic_caller ; sleep 1
    killall sick_scan_emulator ; sleep 1
}

# Start rviz if not yet running
function start_rviz()
{
    # Note: Due to a bug in opengl 3 in combination with rviz and VMware, opengl 2 should be used by rviz option --opengl 210
    # See https://github.com/ros-visualization/rviz/issues/1444 and https://github.com/ros-visualization/rviz/issues/1508 for further details
    rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
    if [ $rviz_running -lt 1 ] ; then
        rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_mrs1104.rviz --opengl 210 &
        sleep 1
    fi
}

# Run sick_scan_emulator and sick_generic_caller for 30 seconds or until rviz is closed
function run_mrs1104_simu()
{
    duration_sec=$1
    pcapng_json_file=$2
    ang_res=$3 
    scan_freq=$4 
    scan_layer_filter=$5
    # Start rviz if not yet running
    start_rviz
    # Start sick_scan_xd emulator
    roslaunch sick_scan_xd emulator_mrs1104.launch scandatafiles:=$pcapng_json_file &
    sleep 1
    # Start sick_scan_xd driver for mrs1104
    echo -e "Launching sick_scan_xd sick_mrs_1xxx.launch\n"
    # roslaunch sick_scan_xd sick_mrs_1xxx.launch hostname:=192.168.0.151 &
    roslaunch sick_scan_xd sick_mrs_1xxx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False ang_res:=$ang_res scan_freq:=$scan_freq scan_layer_filter:="$scan_layer_filter" &
    sleep 1
    # Wait max. 30 sec or for 'q' or 'Q' or until rviz is closed
    waitUntilRvizClosed $duration_sec
    # Shutdown simulation
    kill_simu
}

# Setup
printf "\033c"
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash ] ; then source /opt/ros/melodic/setup.bash ; fi
if [ -f /opt/ros/noetic/setup.bash  ] ; then source /opt/ros/noetic/setup.bash  ; fi
if [ -f ./devel_isolated/setup.bash ] ; then source ./devel_isolated/setup.bash ; fi
pcapng_folder=`(pwd)`/src/sick_scan_xd/test/emulator/scandata
echo -e "run_simu_mrs1104.bash: starting mrs1104 emulation\n"

# Start roscore if not yet running
roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
if [ $roscore_running -lt 1 ] ; then 
  roscore &
  sleep 3
fi

# Run MRS-1104 simulation using recorded pcapng-files

run_mrs1104_simu 10 $pcapng_folder/20210722_143600_ros2_mrs1104_sick_scan_xd.pcapng.json 0 0 "4 1 1 1 1"
run_mrs1104_simu 10 $pcapng_folder/20230112_01_mrs1000_layer_1111_50hz_0.25deg.pcapng.json 0.25 50 "4 1 1 1 1"
run_mrs1104_simu 10 $pcapng_folder/20230112_02_mrs1000_layer_1000_50hz_0.25deg.pcapng.json 0.25 50 "4 1 0 0 0"
run_mrs1104_simu 10 $pcapng_folder/20230112_03_mrs1000_layer_0100_50hz_0.25deg.pcapng.json 0.25 50 "4 0 1 0 0"
run_mrs1104_simu 10 $pcapng_folder/20230112_04_mrs1000_layer_0010_50hz_0.25deg.pcapng.json 0.25 50 "4 0 0 1 0"
run_mrs1104_simu 10 $pcapng_folder/20230112_05_mrs1000_layer_0001_50hz_0.25deg.pcapng.json 0.25 50 "4 0 0 0 1"
run_mrs1104_simu 10 $pcapng_folder/20230112_06_mrs1000_layer_0101_50hz_0.25deg.pcapng.json 0.25 50 "4 0 1 0 1"
run_mrs1104_simu 10 $pcapng_folder/20230112_07_mrs1000_layer_1010_50hz_0.25deg.pcapng.json 0.25 50 "4 1 0 1 0"
run_mrs1104_simu 10 $pcapng_folder/20230112_08_mrs1000_layer_1111_25hz_0.125deg.pcapng.json 0.125 25 "4 1 1 1 1"
run_mrs1104_simu 10 $pcapng_folder/20230112_09_mrs1000_layer_0101_25hz_0.125deg.pcapng.json 0.125 25 "4 0 1 0 1"
run_mrs1104_simu 10 $pcapng_folder/20230112_10_mrs1000_layer_1010_25hz_0.125deg.pcapng.json 0.125 25 "4 1 0 1 0"
run_mrs1104_simu 10 $pcapng_folder/20230112_11_mrs1000_layer_1111_12.5hz_0.0625deg.pcapng.json 0.0625 12.5 "4 1 1 1 1"
run_mrs1104_simu 10 $pcapng_folder/20230112_12_mrs1000_layer_0101_12.5hz_0.0625deg.pcapng.json 0.0625 12.5 "4 0 1 0 1"
run_mrs1104_simu 10 $pcapng_folder/20230112_13_mrs1000_layer_1010_12.5hz_0.0625deg.pcapng.json 0.0625 12.5 "4 1 0 1 0"
run_mrs1104_simu 10 $pcapng_folder/20230112_14_mrs1000_layer_1000_12.5hz_0.0625deg.pcapng.json 0.0625 12.5 "4 1 0 0 0"
run_mrs1104_simu 10 $pcapng_folder/20230112_15_mrs1000_layer_0100_12.5hz_0.0625deg.pcapng.json 0.0625 12.5 "4 0 1 0 0"
run_mrs1104_simu 10 $pcapng_folder/20230112_16_mrs1000_layer_0010_12.5hz_0.0625deg.pcapng.json 0.0625 12.5 "4 0 0 1 0"
run_mrs1104_simu 10 $pcapng_folder/20230112_17_mrs1000_layer_0001_12.5hz_0.0625deg.pcapng.json 0.0625 12.5 "4 0 0 0 1"

# Shutdown
echo -e "Finishing mrs1104 emulation, shutdown all ros nodes\n"
rosnode kill -a ; sleep 1
popd

