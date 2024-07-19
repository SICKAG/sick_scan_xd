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
    killall sick_generic_caller ; sleep 1
    killall sick_scan_emulator ; sleep 1
}

# Setup
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

# Start rviz
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_slam_mrs1104.rviz --opengl 210 &

# Start sick_scan_xd emulator
pcapng_folder=`(pwd)`/src/sick_scan_xd/test/emulator/scandata
roslaunch sick_scan_xd emulator_mrs1104.launch scandatafiles:=$pcapng_folder/20210722_143600_ros2_mrs1104_sick_scan_xd.pcapng.json &
sleep 1

# Start sick_scan_xd driver for mrs1104
echo -e "Launching sick_scan_xd sick_mrs_1xxx.launch\n"
# roslaunch sick_scan_xd sick_mrs_1xxx.launch hostname:=192.168.0.151 &
roslaunch sick_scan_xd sick_mrs_1xxx.launch hostname:=127.0.0.1 tf_publish_rate:=0 sw_pll_only_publish:=False &
sleep 1

# Start SLAM
# To use sick_scan_xd and ROS1 hector slam, clone sick_scan_xd and hector_slam and rebuild:
# cd src
# git clone -b master https://github.com/SICKAG/sick_scan_xd.git
# git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
# cd ..
# catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DCMAKE_ENABLE_EMULATOR=1 -Wno-dev
roslaunch sick_scan_xd test_200_slam_ros1_hector.launch scan_topic:=/scan &
sleep 1

# Wait max. 60 sec or for 'q' or 'Q' or until rviz is closed
waitUntilRvizClosed 60
echo -e "sick_scan_xd simulation finished, shutdown all ros nodes\n"
kill_simu
popd

