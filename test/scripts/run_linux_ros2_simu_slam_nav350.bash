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
    pkill -f ros2
    sleep 1
    pkill -f slam_toolbox
    pkill -f nav2
    pkill -f static_transform_publisher
    pkill -f sopas_json_test_server.py
    killall sick_generic_caller
    killall sick_scan_emulator
    killall rviz2
    sleep 1
}

# Setup
printf "\033c"
pushd ../../../..
if   [ -f /opt/ros/jazzy/setup.bash    ] ; then source /opt/ros/jazzy/setup.bash ; export QT_QPA_PLATFORM=xcb
elif [ -f /opt/ros/humble/setup.bash   ] ; then source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash     ] ; then source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/eloquent/setup.bash ] ; then source /opt/ros/eloquent/setup.bash
fi
source ./install/setup.bash

# Start rviz
ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_slam_nav350.rviz &

# Start sick_scan_xd emulator
python3 ./src/sick_scan_xd/test/python/sopas_json_test_server.py --tcp_port=2112 --json_file=./src/sick_scan_xd/test/emulator/scandata/20230126_nav350_4reflectors_moving.pcapng.json --scandata_id="sAN mNPOSGetData" --send_rate=8 --verbosity=0 &
sleep 1

# Start sick_scan_xd driver for NAV-350
echo -e "Launching sick_scan_xd sick_nav_350.launch\n"
ros2 launch sick_scan_xd sick_nav_350.launch.py hostname:=127.0.0.1 &
sleep 1

# Start SLAM
# Install:
# sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-slam-toolbox
# Run:
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link cloud  &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link  &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint  &
ros2 launch nav2_bringup navigation_launch.py &
ros2 launch slam_toolbox online_async_launch.py &
sleep 1

# Wait max. 60 sec or for 'q' or 'Q' or until rviz is closed
waitUntilRvizClosed 60
echo -e "sick_scan_xd simulation finished, shutdown all ros nodes\n"
kill_simu
kill_simu
popd

