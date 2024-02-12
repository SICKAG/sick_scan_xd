#!/bin/bash
printf "\033c"
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash     ] ; then source /opt/ros/melodic/setup.bash   ; fi
if [ -f /opt/ros/noetic/setup.bash      ] ; then source /opt/ros/noetic/setup.bash    ; fi
if [ -f ./devel_isolated/setup.bash     ] ; then source ./devel_isolated/setup.bash   ; fi

echo -e "run_simu_nav350.bash: starting NAV-350 emulation\n"

# Start roscore if not yet running
roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
if [ $roscore_running -lt 1 ] ; then 
  roscore &
  sleep 3
fi

# Start sick_scan_xd emulator
python3 ./src/sick_scan_xd/test/python/sopas_json_test_server.py --tcp_port=2112 --json_file=./src/sick_scan_xd/test/emulator/scandata/20230126_nav350_4reflectors_moving.pcapng.json --scandata_id="sAN mNPOSGetData" --send_rate=8 --verbosity=0 &
sleep 1

# Start rviz
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_nav350.rviz --opengl 210 &
sleep 1

# Start sick_scan_xd driver for rms
echo -e "Launching sick_scan_xd sick_nav_350.launch\n"
if false ; then
  nav_landmark_imk_file=`(pwd)`/src/sick_scan_xd/test/emulator/scandata/20230126_nav350_4reflectors_moving.imk
  nav_landmark_imk_file=`(pwd)`/src/sick_scan_xd/test/emulator/scandata/20231130_issue229.imk
  nav_landmark_imk_file=`(pwd)`/src/sick_scan_xd/test/emulator/scandata/20231130_issue229_tvz.imk
  roslaunch sick_scan_xd sick_nav_350.launch hostname:=127.0.0.1 nav_do_initial_mapping:=False nav_set_landmark_layout_by_imk_file:=$nav_landmark_imk_file sw_pll_only_publish:=False &
else
  # roslaunch sick_scan_xd sick_nav_350.launch hostname:=127.0.0.1 nav_do_initial_mapping:=False sw_pll_only_publish:=False &
  roslaunch sick_scan_xd sick_nav_350.launch hostname:=127.0.0.1 nav_do_initial_mapping:=True sw_pll_only_publish:=False &
fi
sleep 1

# Wait for 'q' or 'Q' to exit or until rviz is closed
while true ; do  
  echo -e "NAV-350 emulation running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
  if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
  rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
  if [ $rviz_running -lt 1 ] ; then break ; fi
done

# Service examples:
# rosservice call /sick_nav_350/ColaMsg "{request: 'sMN SetAccessMode 3 F4724744'}"
# rosservice call /sick_nav_350/ColaMsg "{request: 'sMN mNPOSGetData 1 2'}"

# Odometry examples (set velocity)
# rostopic pub --once /sick_nav_350/nav_odom_velocity sick_scan_xd/NAVOdomVelocity '{vel_x: 0.0, vel_y: 0.0, omega: 0.0, timestamp: 0, coordbase: 0}'
# rostopic pub --once /sick_nav_350/odom nav_msgs/Odometry '{twist: { twist: { linear: {x: 1.0, y: -1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}}'
# rostopic pub --rate 10 /sick_nav_350/nav_odom_velocity sick_scan_xd/NAVOdomVelocity '{vel_x: 0.0, vel_y: 0.0, omega: 0.0, timestamp: 0, coordbase: 0}'
# rostopic pub --rate 10 /sick_nav_350/nav_odom_velocity sick_scan_xd/NAVOdomVelocity '{vel_x: 1.0, vel_y: -1.0, omega: 0.5, timestamp: 123456789, coordbase: 0}'
# rostopic pub --rate 10 /sick_nav_350/odom nav_msgs/Odometry '{twist: { twist: { linear: {x: 1.0, y: -1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}}'

# Shutdown
echo -e "Finishing NAV-350 emulation, shutdown ros nodes\n"
rosnode kill -a ; sleep 1
killall sick_generic_caller ; sleep 1
pkill -f sopas_json_test_server.py
popd

