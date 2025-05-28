#!/bin/bash

# 
# Run sick_scan_xd tests on Linux ROS1
# 

# Just prompt for key input
function prompt()
{
  read -n 1 -p "Press any key to continue..."
}

# Shutdown simulation, kill all nodes and processes
function kill_simu()
{
  rosnode kill -a ; sleep 1
  killall sick_generic_caller
  killall rviz
  pkill -f sopas_json_test_server.py
  pkill -f multiscan_sopas_test_server.py
  sleep 1 ; killall -9 sick_generic_caller
}

# Wait until a process has exited
function waitUntilProcessExit()
{
  duration_sec=$1
  procname=$2
  for ((cnt=1;cnt<=$duration_sec;cnt++)) ; do  
    proc_running=`(ps -elf | grep ${procname} | grep -v grep | wc -l)`
    if [ $proc_running -lt 1 ] ; then break ; fi
    sleep 1
  done
}

# Wait until sick_scan_xd and rviz are closed
function waitUntilSimuClosed()
{
  duration_sec=$1
  sleep 10
  waitUntilProcessExit $duration_sec sick_generic_caller
  sleep 5
  kill_simu
}

# 
# Init and cleanup
# 

printf "\033c"
pushd ../../../..
source /opt/ros/noetic/setup.bash
source ./devel_isolated/setup.bash
kill_simu
if [ -d ./log  ] ; then rm -rf ./log ; fi
mkdir -p ./log

# 
# Run sick_scan_xd tests on Linux ROS1
# 

for cfg in multiscan_compact_test01_cfg picoscan_compact_test01_cfg lms1xx_test01_cfg lms1xxx_test01_cfg lms5xx_test01_cfg mrs1xxx_test01_cfg mrs6xxx_test01_cfg nav350_test01_cfg rmsxxxx_test01_cfg tim240_test01_cfg tim7xx_test01_cfg tim7xxs_test01_cfg lms4xxx_test01_cfg lrs36x0_test01_cfg lrs36x1_test01_cfg lrs4xxx_test01_cfg oem15xx_test01_cfg tim4xx_test01_cfg tim5xx_test01_cfg ; do 
  echo -e "\n**\n** Start sick_scan_xd test:\n** python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --api=none --cfg=./src/sick_scan_xd/test/docker/data/${cfg}.json\n**\n"
  python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --api=none --cfg=./src/sick_scan_xd/test/docker/data/${cfg}.json
  waitUntilSimuClosed 60
  echo -e "\n**\n** Finished sick_scan_xd test:\n** python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --api=none --cfg=./src/sick_scan_xd/test/docker/data/${cfg}.json\n**\n"
  # prompt
done

