#!/bin/bash
printf "\033c"
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash   ] ; then source /opt/ros/melodic/setup.bash   ; fi
if [ -f /opt/ros/noetic/setup.bash    ] ; then source /opt/ros/noetic/setup.bash    ; fi
if [ -f ./install_isolated/setup.bash ] ; then source ./install_isolated/setup.bash ; fi
if [ -f ./install/setup.bash          ] ; then source ./install/setup.bash          ; fi

echo -e "run_simu_tim7xx_twin.bash: starting tim7xx twin emulation\n"

# Start roscore if not yet running
roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
if [ $roscore_running -lt 1 ] ; then 
  roscore &
  sleep 3
fi

# Start sick_scan_xd emulator simulating 2 lidar devices
roslaunch sick_scan_xd emulator_01_default.launch &
sleep 1
roslaunch sick_scan_xd emulator_01_twin.launch &
sleep 1

# Start rviz twice for tim_7xx and its twin
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_twin_1.rviz --opengl 210 &
sleep 1
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_twin_2.rviz --opengl 210 &
sleep 1

# Start sick_scan_xd driver twice for tim_7xx and its twin
roslaunch sick_scan_xd sick_tim_7xx.launch nodename:=sick_tim_7xx_1 hostname:=127.0.0.1 port:=2112 cloud_topic:=cloud_1 sw_pll_only_publish:=False &
sleep 1
roslaunch sick_scan_xd sick_tim_7xx.launch nodename:=sick_tim_7xx_2 hostname:=127.0.0.1 port:=2312 cloud_topic:=cloud_2 sw_pll_only_publish:=False &
sleep 1

# Wait for 'q' or 'Q' to exit or until rviz is closed
for n in $(seq 25 -1 1) ; do
  echo -e "tim7xx twin emulation running. Up to $n seconds until scan data become available..."
  sleep 1
done
while true ; do  
  echo -e "tim7xx twin emulation running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
  if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
  rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
  if [ $rviz_running -lt 1 ] ; then break ; fi
done

# Shutdown
echo -e "Finishing tim7xx twin emulation, shutdown ros nodes\n"
rosnode kill -a ; sleep 1
killall sick_generic_caller ; sleep 1
killall sick_scan_emulator ; sleep 1
popd

