#!/bin/bash
printf "\033c"
pushd ../../../..
if   [ -f /opt/ros/jazzy/setup.bash    ] ; then source /opt/ros/jazzy/setup.bash ; export QT_QPA_PLATFORM=xcb
elif [ -f /opt/ros/humble/setup.bash   ] ; then source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash     ] ; then source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/eloquent/setup.bash ] ; then source /opt/ros/eloquent/setup.bash
fi
source ./install/setup.bash

echo -e "run_simu_nav350.bash: starting NAV-350 emulation\n"

# Start sick_scan_xd emulator
python3 ./src/sick_scan_xd/test/python/sopas_json_test_server.py --tcp_port=2112 --json_file=./src/sick_scan_xd/test/emulator/scandata/20230126_nav350_4reflectors_moving.pcapng.json --scandata_id="sAN mNPOSGetData" --send_rate=8 --verbosity=0 &
sleep 1

# Start rviz
ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_emulator_cfg_nav350.rviz &
sleep 1

# Start sick_scan_xd driver for rms
echo -e "Launching sick_scan_xd sick_nav_350.launch\n"
ros2 launch sick_scan_xd sick_nav_350.launch.py hostname:=127.0.0.1 &
sleep 1

# Wait for 'q' or 'Q' to exit or until rviz is closed
while true ; do  
  echo -e "NAV-350 emulation running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
  if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
  rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
  if [ $rviz_running -lt 1 ] ; then break ; fi
done

# Shutdown
echo -e "Finishing NAV-350 emulation, shutdown ros nodes\n"
killall sick_generic_caller ; sleep 1
pkill -f sopas_json_test_server.py ; sleep 1
killall rviz2
popd

