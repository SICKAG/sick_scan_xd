#!/bin/bash

#
# Set environment
#

function simu_killall()
{
  sleep 1 ; pkill -f "ros2 topic echo"
  sleep 1 ; killall -SIGINT rviz2
  sleep 1 ; killall -SIGINT sick_generic_caller
  sleep 1 ; killall -SIGINT sick_scan_emulator
  sleep 1 ; killall -9 rviz2
  sleep 1 ; killall -9 sick_generic_caller
  sleep 1 ; killall -9 sick_scan_emulator 
  sleep 1
}

function run_simu()
{
    emulator_launch_cfg=$1
    sick_scan_launch_file=$2
    sleep  1 ; ros2 run sick_scan_xd sick_scan_emulator ./src/sick_scan_xd/test/emulator/launch/$emulator_launch_cfg  scanner_type:=sick_tim_7xx&
    # sleep  1 ; ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/$sick_scan_launch_file hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False & 
    sleep  1 ; ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/$sick_scan_launch_file hostname:=127.0.0.1 port:=2111 &
    sleep  1 ; ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_ros2.rviz &
    sleep  1 ; ros2 topic echo /sick_tim_7xx/lidinputstate &
    sleep 30 ; simu_killall
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
# 4. Stop simulation after 30 seconds
#

echo -e "run_linux_ros2_simu_tim7xx_tim7xxS.bash: starting TiM7xx/TiM7xxS emulation with  $sick_scan_launch_file\n"
cp -f ./src/sick_scan_xd/test/emulator/scandata/20241010-tim781-lidinputstate-toggle-1-9.pcapng.json /tmp/lmd_scandata.pcapng.json
run_simu emulator_tim781_lidinputstate.launch sick_tim_7xx.launch
  
popd

