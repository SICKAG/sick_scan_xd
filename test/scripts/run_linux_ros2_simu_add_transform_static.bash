#!/bin/bash

function simu_killall()
{
  sleep 1 ; killall -SIGINT rviz2
  sleep 1 ; killall -SIGINT tf2_ros
  sleep 1 ; pkill -f static_transform_publisher
  sleep 1 ; killall -SIGINT sick_generic_caller
  sleep 1 ; killall -SIGINT sick_scan_emulator
  sleep 1 ; killall -9 rviz2
  sleep 1 ; killall -9 tf2_ros
  sleep 1 ; killall -9 sick_generic_caller
  sleep 1 ; killall -9 sick_scan_emulator 
  sleep 1 ; pkill -9 -f static_transform_publisher
  sleep 1
}

# Start tim7xx emulator and rviz
function start_tim7xx_emulator()
{
    echo -e "\nrun_linux_ros2_simu_add_transform: starting tim7xx emulation ...\n"
    cp -f ./src/sick_scan_xd/test/emulator/scandata/sopas_et_field_test_1_2_both_010.pcapng_full.json /tmp/lmd_scandata.pcapng.json
    sleep  1 ; ros2 run sick_scan_xd sick_scan_emulator ./src/sick_scan_xd/test/emulator/launch/emulator_01_default.launch scanner_type:=sick_tim_7xx &
    sleep  1 ; ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_tim7xx_add_transform.rviz &
    sleep  1 ; ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_tim7xx_add_transform_origin.rviz &
    sleep 1
}

# Run sick_generic_caller with tim7xx and additional transform
function run_simu_tim7xx()
{
    tx=$1 ; ty=$2 ; tz=$3 ; roll=$4 ; pitch=$5 ; yaw=$6 ; duration_sec=$7
    echo -e "\nrun_linux_ros2_simu_add_transform.bash: starting sick_scan_xd sick_generic_caller sick_tim_7xx.launch with additional transform ($tx, $ty, $tz, $roll, $pitch, $yaw)\n"
    start_tim7xx_emulator
    ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 add_transform_xyz_rpy:=$tx,$ty,$tz,$roll,$pitch,$yaw &
    ros2 run tf2_ros static_transform_publisher $tx $ty $tz $yaw $pitch $roll cloud origin &
    # ros2 run tf2_ros tf2_echo cloud origin &
    sleep $duration_sec
    simu_killall
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

# Run sick_generic_caller with tim7xx and additional transforms
run_simu_tim7xx 0 0 0 0.0000000 0.0000000 0.0000000 10 # tim7xx, x=0, y=0, z=0, roll=0, pitch=0, yaw=0 deg, duration_sec=10
run_simu_tim7xx 0 0 0 0.0000000 0.0000000 0.7853982 40 # tim7xx, x=0, y=0, z=0, roll=0, pitch=0, yaw=45 deg, duration_sec=40
run_simu_tim7xx 0 0 0 0.0000000 0.7853982 0.0000000 15 # tim7xx, x=0, y=0, z=0, roll=0, pitch=45, yaw=0 deg, duration_sec=15
run_simu_tim7xx 0 0 0 0.7853982 0.0000000 0.0000000 15 # tim7xx, x=0, y=0, z=0, roll=45, pitch=0, yaw=0 deg, duration_sec=15
run_simu_tim7xx  0.5 -0.5 -0.5 -0.7853982 -0.7853982 -0.7853982 15 # tim7xx, x=+0.5, y=-0.5, z=-0.5, roll=-45, pitch=-45, yaw=-45 deg, duration_sec=15
run_simu_tim7xx  0.5 -0.5 -1.0 -1.0471976  1.0471976  1.0471976 15 # tim7xx, x=+0.5, y=-0.5, z=-1.0, roll=-60, pitch=+60, yaw=+60 deg, duration_sec=15
run_simu_tim7xx -0.5  0.5 -1.0  2.0943951 -2.0943951  2.0943951 15 # tim7xx, x=-0.5, y=+0.5, z=-1.0, roll=120, pitch=-120,yaw=120 deg, duration_sec=15

echo -e "\nrun_linux_ros2_simu_add_transform.bash finished.\n"
popd

