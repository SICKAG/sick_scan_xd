#!/bin/bash
printf "\033c"
pushd ../../../..
if   [ -f /opt/ros/jazzy/setup.bash    ] ; then source /opt/ros/jazzy/setup.bash ; export QT_QPA_PLATFORM=xcb
elif [ -f /opt/ros/humble/setup.bash   ] ; then source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash     ] ; then source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/eloquent/setup.bash ] ; then source /opt/ros/eloquent/setup.bash
fi
source ./install/setup.bash

echo -e "run_simu_lms5xx.bash: starting lms5xx emulation\n"

# Start sick_scan_xd emulator
python3 ./src/sick_scan_xd/test/emulator/test_server.py --scandata_file=./src/sick_scan_xd/test/emulator/scandata/20210302_lms511.pcapng.scandata.txt --scandata_frequency=20.0 --tcp_port=2112 & 
# python3 ./src/sick_scan_xd/test/emulator/test_server.py --scandata_file=./src/sick_scan_xd/test/emulator/scandata/20220505_lms511_wireshark_issue49.pcapng.scandata.txt --scandata_frequency=20.0 --tcp_port=2112 & 
sleep 1

# Start rviz
rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_lms5xx.rviz &
sleep 1

# Start sick_scan_xd driver for lms5xx
echo -e "Launching sick_scan_xd sick_lms_1xx.launch\n"
# ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_5xx.launch hostname:=192.168.0.111 &
# ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
ros2 launch sick_scan_xd sick_lms_5xx.launch.py hostname:=127.0.0.1 sw_pll_only_publish:=False &

# Shutdown
sleep 30
pkill rviz2
pkill -f ./test/emulator/test_server.py
killall sick_generic_caller 

popd

