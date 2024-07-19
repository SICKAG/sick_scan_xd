#!/bin/bash

#
# Set environment
#

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

#
# Run simulation:
# 1. Start sick_scan_emulator
# 2. Start sick_scan_xd driver sick_generic_caller
# 3. Run rviz
#

# Start sick_scan_xd emulator simulating 2 lidar devices
echo -e "run_linux_ros2_simu_tim7xx_twin.bash: starting tim7xx emulation with 2 sensors\n"
cp -f ./src/sick_scan_xd/test/emulator/scandata/sopas_et_field_test_1_2_both_010.pcapng_full.json /tmp/lmd_scandata.pcapng.json
ros2 run sick_scan_xd sick_scan_emulator ./src/sick_scan_xd/test/emulator/launch/emulator_01_default.launch result_telegrams_tcp_port:=2201 cola_telegrams_tcp_port:=2112 scanner_type:=sick_tim_7xx &
sleep 1
ros2 run sick_scan_xd sick_scan_emulator ./src/sick_scan_xd/test/emulator/launch/emulator_01_twin.launch result_telegrams_tcp_port:=2301 cola_telegrams_tcp_port:=2312 scanner_type:=sick_tim_7xx &
sleep 1

# Start rviz twice for tim_7xx and its twin
ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_emulator_cfg_twin_1.rviz &
sleep 1
ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_emulator_cfg_twin_2.rviz &
sleep 1

# Start sick_scan_xd driver twice for tim_7xx and its twin
ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xx.launch nodename:=sick_tim_7xx_1 hostname:=127.0.0.1 port:=2112 cloud_topic:=cloud_1 sw_pll_only_publish:=False &
sleep 1
ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xx.launch nodename:=sick_tim_7xx_2 hostname:=127.0.0.1 port:=2312 cloud_topic:=cloud_2 sw_pll_only_publish:=False &
sleep 1

# Wait for 'q' or 'Q' to exit or until rviz is closed ...
for n in $(seq 25 -1 1) ; do
  echo -e "mrs1104 emulation running. Up to $n seconds until scan data become available..."
  sleep 1
done
while true ; do  
  echo -e "mrs1104 emulation running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
  if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
  rviz_running=`(ps -elf | grep rviz2 | grep -v grep | wc -l)`
  if [ $rviz_running -lt 1 ] ; then break ; fi
done

# ... or stop simulation after 30 seconds
# sleep 30

simu_killall  
popd

