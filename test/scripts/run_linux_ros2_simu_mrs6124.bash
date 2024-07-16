#!/bin/bash

function simu_killall()
{
  sleep 1 ; killall -SIGINT rviz2
  sleep 1 ; killall -SIGINT sick_generic_caller
  sleep 1 ; killall -SIGINT sick_scan_emulator
  sleep 1 ; killall -9 rviz2
  sleep 1 ; killall -9 sick_generic_caller
  sleep 1 ; killall -9 sick_scan_emulator 
  sleep 1
}

simu_killall
printf "\033c"
pushd ../../../..
if   [ -f /opt/ros/jazzy/setup.bash    ] ; then source /opt/ros/jazzy/setup.bash ; export QT_QPA_PLATFORM=xcb
elif [ -f /opt/ros/humble/setup.bash   ] ; then source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash     ] ; then source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/eloquent/setup.bash ] ; then source /opt/ros/eloquent/setup.bash
fi
source ./install/setup.bash

# Replay pointcloud from rosbag file
# ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_ros2_mrs6124.rviz &
# ros2 bag info ./src/sick_scan_xd/test/emulator/scandata/rosbag2_2022_07_25-15_02_28_0.db3
# ros2 bag play ./src/sick_scan_xd/test/emulator/scandata/rosbag2_2022_07_25-15_02_28_0.db3

#
# Run simulation:
# 1. Start sick_scan_emulator
# 2. Start sick_scan_xd driver sick_generic_caller
# 3. Run rviz
#

echo -e "run_linux_ros2_simu_mrs6124.bash: starting MRS6124 emulation with sick_mrs_6xxx.launch\n"
cp -f ./src/sick_scan_xd/test/emulator/scandata/20220727_mrs6124.pcapng.json /tmp/lmd_scandata.pcapng.json
sleep 1 ; ros2 run sick_scan_xd sick_scan_emulator ./src/sick_scan_xd/test/emulator/launch/emulator_mrs6124.launch scanner_type:=sick_mrs_6xxx &
# sleep 1 ; ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_mrs_6xxx.launch hostname:=127.0.0.1 port:=2111 & 
sleep 1 ; ros2 launch sick_scan_xd sick_mrs_6xxx.launch.py hostname:=127.0.0.1 port:=2111 &
sleep 1 ; ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_ros2_mrs6124.rviz &

# Wait for 'q' or 'Q' to exit or until rviz is closed ...
while true ; do  
  echo -e "mrs6124 emulation running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
  if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
  rviz_running=`(ps -elf | grep rviz2 | grep -v grep | wc -l)`
  if [ $rviz_running -lt 1 ] ; then break ; fi
done

# ... or stop simulation after 30 seconds
# sleep 30

simu_killall  
popd

