#!/bin/bash

#
# Set environment
#

function simu_killall()
{
  sleep 1 ; killall -SIGINT rviz2
  sleep 1 ; killall -SIGINT sick_generic_caller
  sleep 1 ; killall -SIGINT sick_scan_emulator
  sleep 1 ; pkill -f sopas_json_test_server.py ; pkill -f "ros2 topic echo"
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

# Start sick_scan_xd emulator
python3 ./src/sick_scan_xd/test/python/sopas_json_test_server.py --tcp_port=2112 --json_file=./src/sick_scan_xd/test/emulator/scandata/20240522-LMS4xxx.pcapng.json --verbosity=0 &
# python3 ./src/sick_scan_xd/test/python/sopas_json_test_server.py --tcp_port=2112 --json_file=./src/sick_scan_xd/test/emulator/scandata/20241021_lms4000_encoder_test.pcapng.json --verbosity=0 &
sleep 3 # Wait until pcapng.json file is loaded and parsed

# Start sick_scan_xd driver
ros2 launch sick_scan_xd sick_lms_4xxx.launch.py encoder_mode:=2 hostname:=127.0.0.1 sw_pll_only_publish:=False &
sleep 1

# Start rviz
ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_ros2.rviz &
(ros2 topic echo /sick_lms_4xxx/encoder | tr '\n' ' ') & # ros2 topic echo /sick_lms_4xxx/encoder
sleep 1

# Run for 30 seconds
sleep 30
simu_killall
popd

