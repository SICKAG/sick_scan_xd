#!/bin/bash
printf "\033c"
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash     ] ; then source /opt/ros/melodic/setup.bash   ; fi
if [ -f /opt/ros/noetic/setup.bash      ] ; then source /opt/ros/noetic/setup.bash    ; fi
if [ -f ./devel_isolated/setup.bash     ] ; then source ./devel_isolated/setup.bash   ; fi
# if [ -f ./install_isolated/setup.bash ] ; then source ./install_isolated/setup.bash ; fi
# if [ -f ./install/setup.bash          ] ; then source ./install/setup.bash          ; fi

# Start roscore if not yet running
roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
if [ $roscore_running -lt 1 ] ; then 
  roscore &
  sleep 3
fi

# Start rviz
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg.rviz --opengl 210 &
sleep 1

# Run TiM7xxS with initialization
echo -e "run_linux_ros1_simu_tim7xxS_without_init.bash: starting TiM7xxS emulation with sick_tim_7xxS.launch\n"
  
# Start sick_scan_xd emulator
roslaunch sick_scan_xd emulator_01_default.launch &
sleep 1
    
# Start sick_scan_xd driver for TiM7xxS
echo -e "Launching sick_scan_xd sick_tim_7xxS.launch\n"
roslaunch sick_scan_xd sick_tim_7xxS.launch hostname:=127.0.0.1 &

# Kill sick_generic_caller and restart without TiM7xxS initialization
sleep 5
killall -9 sick_generic_caller
killall -9 sick_scan_emulator
sleep 1
roslaunch sick_scan_xd emulator_01_default.launch start_scandata_immediately:=1 &
sleep 1
roslaunch sick_scan_xd sick_tim_7xxS.launch hostname:=127.0.0.1 initialize_scanner:=0 &
   
# Wait for 'q' or 'Q' to exit or until rviz is closed
for i in {1..60}; do # while true ; do  
  echo -e "TiM7xxS emulation running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
  if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
  rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
  if [ $rviz_running -lt 1 ] ; then break ; fi
done
    
# Shutdown
echo -e "Finishing TiM7xxS emulation, shutdown ros nodes\n"
pkill rviz
rosnode kill -a ; sleep 1
killall sick_generic_caller ; sleep 1
killall sick_scan_emulator ; sleep 1
killall rosmaster ; sleep 1
echo -e "run_linux_ros1_simu_tim7xxS_without_init.bash: TiM7xxS emulation finished.\n\n"

popd

