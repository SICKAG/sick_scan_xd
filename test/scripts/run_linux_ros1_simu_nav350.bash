#!/bin/bash
printf "\033c"
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash   ] ; then source /opt/ros/melodic/setup.bash   ; fi
if [ -f /opt/ros/noetic/setup.bash    ] ; then source /opt/ros/noetic/setup.bash    ; fi
if [ -f ./install_isolated/setup.bash ] ; then source ./install_isolated/setup.bash ; fi
if [ -f ./install/setup.bash          ] ; then source ./install/setup.bash          ; fi

echo -e "run_simu_nav350.bash: starting NAV-350 emulation\n"

# Start roscore if not yet running
roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
if [ $roscore_running -lt 1 ] ; then 
  roscore &
  sleep 3
fi

# Start sick_scan emulator
roslaunch sick_scan emulator_nav350.launch &
sleep 1

# Start rviz
# Note: Due to a bug in opengl 3 in combination with rviz and VMware, opengl 2 should be used by rviz option --opengl 210
# See https://github.com/ros-visualization/rviz/issues/1444 and https://github.com/ros-visualization/rviz/issues/1508 for further details

rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_nav350.rviz --opengl 210 &
sleep 1

# Start sick_scan driver for NAV-350
echo -e "Launching sick_scan sick_nav_350.launch\n"
# roslaunch sick_scan sick_nav_350.launch hostname:=192.168.0.151 &
roslaunch sick_scan sick_nav_350.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
sleep 5

# Send 'sMN LMCstartmeas' to start LMDscandata. This is not required for NAV-350 devices, which send LMDscandata immediately after establishing a tcp connection. In contrary, the emulator waits for the 'sMN LMCstartmeas' command.
rosservice call /sick_nav_350/ColaMsg "{request: 'sMN LMCstartmeas'}"
#rosservice call /sick_nav_350/ColaMsg "{request: 'sMN Run'}"

# Wait for 'q' or 'Q' to exit or until rviz is closed
while true ; do  
  echo -e "NAV-350 emulation running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
  if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
  rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
  if [ $rviz_running -lt 1 ] ; then break ; fi
done

# Shutdown
echo -e "Finishing NAV-350 emulation, shutdown ros nodes\n"
rosservice call /sick_nav_350/SickScanExit "{}"
killall sick_scan_emulator ; sleep 1
rosnode kill -a ; sleep 1
killall sick_generic_caller ; sleep 1
popd

