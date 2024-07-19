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

# Start tim7xx emulator and rviz
function start_tim7xx_emulator()
{
    echo -e "\nrun_linux_ros2_simu_add_transform: starting tim7xx emulation ...\n"
    cp -f ./src/sick_scan_xd/test/emulator/scandata/sopas_et_field_test_1_2_both_010.pcapng_full.json /tmp/lmd_scandata.pcapng.json
    sleep  1 ; ros2 run sick_scan_xd sick_scan_emulator ./src/sick_scan_xd/test/emulator/launch/emulator_01_default.launch scanner_type:=sick_tim_7xx &
    sleep  1 ; ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_tim7xx_add_transform.rviz &
    sleep 1
}

# Sets parameter add_transform_xyz_rpy dynamically, moves and rotates the pointcloud
function run_simu_transforms()
{
    nodename=$1
    paramname=$2
    delta=$3 # dx, dy in meter, e.g. 0.1
    echo -e "\nrun_linux_ros1_simu_add_transform.bash: setting transforms (translation in x,y)\n"
    for ((cnt=1;cnt<=1;cnt++)) ; do  # apply translation in x and y
        sleep 2 ; ros2 param set $nodename $paramname "$delta,0.0,0,0,0,0"       # x=+0.3, y=+0.0, z=0, roll=0, pitch=0, yaw=0 deg
        sleep 2 ; ros2 param set $nodename $paramname "$delta,$delta,0,0,0,0"    # x=+0.3, y=+0.3, z=0, roll=0, pitch=0, yaw=0 deg
        sleep 2 ; ros2 param set $nodename $paramname "0.0,$delta,0,0,0,0"       # x=+0.3, y=+0.3, z=0, roll=0, pitch=0, yaw=0 deg
        sleep 2 ; ros2 param set $nodename $paramname "\-$delta,$delta,0,0,0,0"   # x=-0.3, y=+0.3, z=0, roll=0, pitch=0, yaw=0 deg
        sleep 2 ; ros2 param set $nodename $paramname "\-$delta,0.0,0,0,0,0"      # x=-0.3, y=+0.0, z=0, roll=0, pitch=0, yaw=0 deg
        sleep 2 ; ros2 param set $nodename $paramname "\-$delta,\-$delta,0,0,0,0"  # x=-0.3, y=-0.3, z=0, roll=0, pitch=0, yaw=0 deg
        sleep 2 ; ros2 param set $nodename $paramname "0.0,\-$delta,0,0,0,0"      # x=+0.0, y=-0.3, z=0, roll=0, pitch=0, yaw=0 deg
        sleep 2 ; ros2 param set $nodename $paramname "$delta,\-$delta,0,0,0,0"   # x=+0.3, y=-0.3, z=0, roll=0, pitch=0, yaw=0 deg
        sleep 2 ; ros2 param set $nodename $paramname "$delta,0.0,0,0,0,0"       # x=+0.3, y=+0.0, z=0, roll=0, pitch=0, yaw=0 deg
        sleep 2 ; ros2 param set $nodename $paramname "0.0,0.0,0,0,0,0"          # x=+0.0, y=+0.0, z=0, roll=0, pitch=0, yaw=0 deg
    done
    echo -e "\nrun_linux_ros1_simu_add_transform.bash: setting transforms (rotation about yaw)\n"
    for ((cnt=1;cnt<=1;cnt++)) ; do  # apply rotation about yaw angle

        sleep 2 ; ros2 param set $nodename $paramname "0,0,0,0.0000000,0.0000000,0.7853982"    # x=0, y=0, z=0, roll=0, pitch=0, yaw=45 deg
        sleep 2 ; ros2 param set $nodename $paramname "0,0,0,0.0000000,0.0000000,1.5707963"    # x=0, y=0, z=0, roll=0, pitch=0, yaw=90 deg
        sleep 2 ; ros2 param set $nodename $paramname "0,0,0,0.0000000,0.0000000,2.3561945"    # x=0, y=0, z=0, roll=0, pitch=0, yaw=135 deg
        sleep 2 ; ros2 param set $nodename $paramname "0,0,0,0.0000000,0.0000000,3.1415926"    # x=0, y=0, z=0, roll=0, pitch=0, yaw=180 deg
        sleep 2 ; ros2 param set $nodename $paramname "0,0,0,0.0000000,0.0000000,\-2.3561945"  # x=0, y=0, z=0, roll=0, pitch=0, yaw=-135 deg
        sleep 2 ; ros2 param set $nodename $paramname "0,0,0,0.0000000,0.0000000,\-1.5707963"  # x=0, y=0, z=0, roll=0, pitch=0, yaw=-90 deg
        sleep 2 ; ros2 param set $nodename $paramname "0,0,0,0.0000000,0.0000000,\-0.7853982"  # x=0, y=0, z=0, roll=0, pitch=0, yaw=-45 deg
        sleep 2 ; ros2 param set $nodename $paramname "0,0,0,0.0000000,0.0000000,0.0000000"    # x=0, y=0, z=0, roll=0, pitch=0, yaw=0 deg
    done
    sleep 2
    kill_simu
}

# Run sick_generic_caller with tim7xx and additional transform
function run_simu_tim7xx()
{
    start_tim7xx_emulator
    echo -e "\nrun_linux_ros2_simu_add_transform.bash: starting sick_scan_xd sick_tim_7xx.launch, no transform\n"
    ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 add_transform_xyz_rpy:=0,0,0,0,0,0 add_transform_check_dynamic_updates:=true &
    sleep 5
    run_simu_transforms sick_scan_xd add_transform_xyz_rpy 0.3
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

run_simu_tim7xx

echo -e "\nrun_linux_ros2_simu_add_transform.bash finished.\n"
popd

