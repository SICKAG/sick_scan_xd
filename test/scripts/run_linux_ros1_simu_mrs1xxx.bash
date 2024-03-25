#!/bin/bash

function simu_killall()
{
  echo -e "Finishing mrs1xxx emulation, shutdown ros nodes\n"
  rosnode kill -a ; sleep 1
  killall sick_generic_caller ; sleep 1
  killall sick_scan_emulator ; sleep 1
}

# Wait for max 30 seconds, or until 'q' or 'Q' pressed, or until rviz is closed
function waitUntilRvizClosed()
{
  duration_sec=$1
  sleep 1
  for ((cnt=1;cnt<=$duration_sec;cnt++)) ; do  
    echo -e "sick_scan_xd running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
    if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
    rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
    if [ $rviz_running -lt 1 ] ; then break ; fi
  done
}

function run_mrs1xxx_simu()
{
  # Start sick_scan_xd emulator
  pcapng_json_file=$1
  duration_sec=$2
  roslaunch sick_scan_xd emulator_mrs1xxx_imu.launch scandatafiles:=$pcapng_json_file &
  sleep 1
  
  # Start rviz
  rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_mrs1104.rviz --opengl 210 &
  sleep 1
  
  # Start sick_scan_xd driver for mrs1104
  echo -e "Launching sick_scan_xd sick_mrs_1xxx.launch\n"
  roslaunch sick_scan_xd sick_mrs_1xxx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
  sleep 1
  
  # Wait for 'q' or 'Q' to exit or until rviz is closed
  rostopic echo -p /sick_mrs_1xxx/imu &
  waitUntilRvizClosed $duration_sec
  
  # Shutdown
  simu_killall
}

simu_killall
printf "\033c"
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash     ] ; then source /opt/ros/melodic/setup.bash   ; fi
if [ -f /opt/ros/noetic/setup.bash      ] ; then source /opt/ros/noetic/setup.bash    ; fi
if [ -f ./devel_isolated/setup.bash     ] ; then source ./devel_isolated/setup.bash ; fi

echo -e "run_simu_mrs1xxx.bash: starting mrs1xxx emulation\n"

# Start roscore if not yet running
roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
if [ $roscore_running -lt 1 ] ; then 
  roscore &
  sleep 3
fi

# Run sick_scan_xd with MRS1xxx emulator
pcapng_folder=`(pwd)`/src/sick_scan_xd/test/emulator/scandata
run_mrs1xxx_simu $pcapng_folder/20240304-MRS1xxx-default-settings-rssiflag-3.pcapng.json 10
run_mrs1xxx_simu $pcapng_folder/20240307-MRS1xxx-default-settings-rssiflag3-angres0.2500-scanfreq50.0.pcapng.json 10
run_mrs1xxx_simu $pcapng_folder/20240307-MRS1xxx-default-settings-rssiflag3-angres0.1250-scanfreq25.0.pcapng.json 10
run_mrs1xxx_simu $pcapng_folder/20240307-MRS1xxx-default-settings-rssiflag3-angres0.0625-scanfreq12.5.pcapng.json 10
run_mrs1xxx_simu $pcapng_folder/20240304-MRS1xxx-default-settings-rssiflag-1.pcapng.json 10

popd

