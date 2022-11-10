#!/bin/bash

#
# Set environment
#

function simu_killall()
{
  sleep 1 ; killall -SIGINT rviz2
  sleep 1 ; killall -SIGINT sick_generic_caller
  sleep 1 ; killall -9 rviz2
  sleep 1 ; killall -9 sick_generic_caller
  sleep 1
}

simu_killall
printf "\033c"
pushd ../../../..
if [ -f /opt/ros/eloquent/setup.bash ] ; then source /opt/ros/eloquent/setup.bash ; fi
if [ -f /opt/ros/foxy/setup.bash     ] ; then source /opt/ros/foxy/setup.bash     ; fi
source ./install/setup.bash

#
# Run simulation:
# 1. Start sick_scan_emulator
# 2. Start sick_scan driver sick_generic_caller
# 3. Run rviz
#

echo -e "run_linux_ros2_simu_rms.bash: starting rms emulation\n"

# Start sick_scan emulator
python3 ./src/sick_scan_xd/test/python/sopas_json_test_server.py --tcp_port=2112 --json_file=./src/sick_scan_xd/test/emulator/scandata/20221018_rms_1xxx_ascii_rms2_objects.pcapng.json --scandata_id="sSN LMDradardata" --send_rate=10 --verbosity=2 &
sleep  1 ; ros2 launch sick_scan sick_rms_2xxx.launch.py hostname:=127.0.0.1 sw_pll_only_publish:=False &
sleep  1 ; ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_ros2_rms2xxx.rviz &

# Wait for 'q' or 'Q' to exit or until rviz is closed ...
while true ; do  
  echo -e "rms emulation running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
  if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
  rviz_running=`(ps -elf | grep rviz2 | grep -v grep | wc -l)`
  if [ $rviz_running -lt 1 ] ; then break ; fi
done

# ... or stop simulation after 30 seconds
# sleep 30

simu_killall  
popd

