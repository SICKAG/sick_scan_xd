#!/bin/bash

function simu_killall()
{
  killall sick_generic_caller 
  killall sick_scan_emulator 
  killall rviz2 
}

simu_killall
printf "\033c"
pushd ../../../..
if [ -f /opt/ros/eloquent/setup.bash ] ; then source /opt/ros/eloquent/setup.bash ; fi
if [ -f /opt/ros/foxy/setup.bash     ] ; then source /opt/ros/foxy/setup.bash     ; fi
source ./install/setup.bash

# Start sick_scan emulator, rviz2 and sick_generic_caller
simu_killall
echo -e "run_linux_ros2_example_application_tim7xx\n"
cp -f ./src/sick_scan_xd/test/emulator/scandata/sopas_et_field_test_1_2_both_010.pcapng_full.json /tmp/lmd_scandata.pcapng.json
sleep  1 ; ros2 run sick_scan_xd sick_scan_emulator ./src/sick_scan_xd/test/emulator/launch/emulator_01_default.launch scanner_type:=sick_tim_7xx --ros-args --log-level ERROR &
sleep  1 ; ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False & 
sleep  1 ; ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_ros2.rviz &

# Run sick_scan_ros2_example
sleep  3
ros2 run sick_scan_ros2_example sick_scan_ros2_example --ros-args -p topics:=sick_tim_7xx &
sleep 30

# Shutdown
echo -e "Finished run_linux_ros2_example_application_tim7xx, shutdown ros nodes\n" 
simu_killall

popd

