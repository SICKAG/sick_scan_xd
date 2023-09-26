#!/bin/bash
printf "\033c"
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash ] ; then source /opt/ros/melodic/setup.bash ; fi
if [ -f /opt/ros/noetic/setup.bash  ] ; then source /opt/ros/noetic/setup.bash  ; fi
if [ -f ./devel_isolated/setup.bash ] ; then source ./devel_isolated/setup.bash ; fi

echo -e "run_simu_mrs6124.bash: starting mrs6124 emulation\n"

# Start roscore if not yet running
roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
if [ $roscore_running -lt 1 ] ; then 
  roscore &
  sleep 3
fi

# Start sick_scan_xd emulator
roslaunch sick_scan_xd emulator_mrs6124.launch &
sleep 1

# Start rviz
# Note: Due to a bug in opengl 3 in combination with rviz and VMware, opengl 2 should be used by rviz option --opengl 210
# See https://github.com/ros-visualization/rviz/issues/1444 and https://github.com/ros-visualization/rviz/issues/1508 for further details

rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_mrs6124.rviz --opengl 210 &
sleep 1

# Start sick_scan_xd driver for mrs6124
echo -e "Launching sick_scan_xd sick_mrs_6xxx.launch\n"
# roslaunch sick_scan_xd sick_mrs_6xxx.launch hostname:=192.168.0.151 &
roslaunch sick_scan_xd sick_mrs_6xxx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
sleep 1

# Wait for 'q' or 'Q' to exit or until rviz is closed
while true ; do  
  echo -e "mrs1104 emulation running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
  if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
  rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
  if [ $rviz_running -lt 1 ] ; then break ; fi
done

# Shutdown
echo -e "Finishing mrs6124 emulation, shutdown ros nodes\n"
rosservice call /sick_mrs_6xxx/SickScanExit "{}"
sleep 1
rosnode kill -a ; sleep 1
killall sick_generic_caller ; sleep 1
killall sick_scan_emulator ; sleep 1
popd

