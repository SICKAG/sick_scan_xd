#!/bin/bash
printf "\033c"
pushd ../../../..
source /opt/ros/eloquent/setup.bash
source ./install/setup.bash

echo -e "run_simu_lms5xx.bash: starting lms5xx emulation\n"

# Start sick_scan emulator
python3 ./src/sick_scan_xd/test/emulator/test_server.py --scandata_file=./src/sick_scan_xd/test/emulator/scandata/20210302_lms511.pcapng.scandata.txt --scandata_frequency=20.0 --tcp_port=2112 & 
sleep 1

# Start rviz
rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_lms5xx.rviz &
sleep 1

# Start sick_scan driver for lms5xx
echo -e "Launching sick_scan sick_lms_1xx.launch\n"
# ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_5xx.launch hostname:=192.168.0.111 &
ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_5xx.launch hostname:=127.0.0.1 &
sleep 1

# Wait for 'q' or 'Q' to exit or until rviz is closed
while true ; do  
  echo -e "lms5xx emulation running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
  if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
  rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
  if [ $rviz_running -lt 1 ] ; then break ; fi
done

# Shutdown
pkill rviz2
pkill -f ./test/emulator/test_server.py
killall sick_generic_caller 

popd

