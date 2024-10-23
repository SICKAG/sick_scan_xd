#!/bin/bash
printf "\033c"
pushd ../../../..
if [ -f /opt/ros/noetic/setup.bash      ] ; then source /opt/ros/noetic/setup.bash    ; fi
if [ -f ./devel_isolated/setup.bash     ] ; then source ./devel_isolated/setup.bash   ; fi

# Start roscore if not yet running
# roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
# if [ $roscore_running -lt 1 ] ; then 
#   roscore &
#   sleep 3
# fi

# Start rviz
#rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg.rviz --opengl 210 &
sleep 1

# Start sick_scan_xd emulator
roslaunch --wait sick_scan_xd emulator_tim781_lidinputstate.launch &
sleep 1
    
# Start sick_scan_xd driver for TiM7xx
echo -e "Launching sick_scan_xd sick_tim_7xx.launch\n"
# rosrun --prefix 'gdb -ex run --args' sick_scan_xd sick_generic_caller _sick_scan_args:="./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False"
roslaunch --wait sick_scan_xd sick_tim_7xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &

# Wait for 'q' or 'Q' to exit or until rviz is closed
for i in {1..60}; do # while true ; do  
  # echo -e "TiM7xx emulation running. Close rviz or press 'q' to exit..."
  read -t 1.0 -n1 -s key
  if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
  # rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
  # if [ $rviz_running -lt 1 ] ; then break ; fi
done
    
# Shutdown
echo -e "Finishing TiM7xx emulation, shutdown ros nodes\n"
rosnode kill -a ; sleep 1
killall -9 sick_generic_caller ; sleep 1
killall -9 sick_scan_emulator ; sleep 1
echo -e "run_linux_ros1_simu_tim781_lidinputstate.bash finished.\n\n"

popd

