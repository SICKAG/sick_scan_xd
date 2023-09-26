#!/bin/bash

# runServiceCalls delay just runs example service calls
function runServiceCalls(){
  local delay=$1
  sleep $delay ; rosservice call /sick_lms_5xx/SCdevicestate "{}"                          # query device state
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sMN IsSystemReady'}"    # query system state
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sRN SCdevicestate'}"    # query device state
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sMN LMCstartmeas'}"     # start measurement
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sRN field000'}"         # query field 000
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sRN field001'}"         # query field 001
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sRN field002'}"         # query field 002
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sRN field003'}"         # query field 003
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sRN field004'}"         # query field 004
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sRN field005'}"         # query field 005
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sRN field006'}"         # query field 006
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sRN field007'}"         # query field 007
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sEN LFErec 1'}"         # activate LFErec messages
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sRN LFErec'}"           # query activation status of LFErec messages
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sEN LIDoutputstate 1'}" # activate LIDoutputstate messages
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sRN LIDoutputstate'}"   # query activation status of LIDoutputstate messages
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sEN LIDinputstate 1'}"  # activate LIDinputstate messages
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sRN LIDinputstate'}"    # query activation status of LIDinputstate messages
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sWN field000 400000000000000000000683ffff3cb0020100010003012cffff016201d2ffff01a301e6ffff00ce0000000000000001000b7365676d656e7465645f310000'}"                 # write field 000
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sWN field001 400000000000000000000683ffff3cb0010200000001000dbba0007d00000000000000c8000000c8000000000001001572656374616e676c655f6669656c645f305f6465670000'}" # write field 001
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sWN field002 400000000000000000000683ffff3cb0010300000001000f756b007f0006ddd0000000c8000000c80000000000010010726563746669656c645f34355f6465670000'}"           # write field 002
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sWN field003 400000000000000000000683ffff3cb003040000000000000001001b774000fa00000000000003e80000012c0960000005dc0001000d64796e616d69635f6669656c640000'}"     # write field 003
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sWN field004 400000000000000000001388ffff3cb002000000000000000000000100000000'}" # write field 004
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sWN field005 400000000000000000001388ffff3cb002000000000000000000000100000000'}" # write field 005
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sWN field006 400000000000000000001388ffff3cb002000000000000000000000100000000'}" # write field 006
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sWN field007 400000000000000000001388ffff3cb002000000000000000000000100000000'}" # write field 007
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sWN field008 400000000000000000001388ffff3cb002000000000000000000000100000000'}" # write field 008
  sleep $delay ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sWN field009 400000000000000000001388ffff3cb002000000000000000000000100000000'}" # write field 009
}

printf "\033c"
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash   ] ; then source /opt/ros/melodic/setup.bash   ; fi
if [ -f /opt/ros/noetic/setup.bash    ] ; then source /opt/ros/noetic/setup.bash    ; fi
if [ -f ./install_isolated/setup.bash ] ; then source ./install_isolated/setup.bash ; fi
if [ -f ./install/setup.bash          ] ; then source ./install/setup.bash          ; fi

#
# run_linux_ros1_simu_concurrent_scandata_and_service_calls.bash:
# * Start LMS5xx emulator and sick_scan_xd driver
# * Send LMDscandata and service calls concurrently
# * Ensure that concurrent sopas messages and scandata telegrams do not cause driver errors
#

echo -e "run_linux_ros1_simu_concurrent_scandata_and_service_calls.bash: starting lms5xx emulation\n"

# Start roscore if not yet running
roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
if [ $roscore_running -lt 1 ] ; then 
  roscore &
  sleep 3
fi

# Start sick_scan_xd emulator
roslaunch sick_scan_xd emulator_lms5xx.launch &
sleep 1

# Start rviz
# Note: Due to a bug in opengl 3 in combination with rviz and VMware, opengl 2 should be used by rviz option --opengl 210
# See https://github.com/ros-visualization/rviz/issues/1444 and https://github.com/ros-visualization/rviz/issues/1508 for further details
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_lms5xx.rviz --opengl 210 &
sleep 1

# Start sick_scan_xd driver for mrs1104
echo -e "run_linux_ros1_simu_concurrent_scandata_and_service_calls.bash: Launching sick_scan_xd sick_lms_5xx.launch\n"
roslaunch sick_scan_xd sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
sleep 5

# Run example ros service calls
sleep 2 ; rosservice list
runServiceCalls 2
for cnt in $(seq 1 1 10) ; do
  runServiceCalls 0
done

# Wait for 'q' or 'Q' to exit or until rviz is closed
while true ; do  
  echo -e "mrs1104 emulation running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
  if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
  rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
  if [ $rviz_running -lt 1 ] ; then break ; fi
done

# Shutdown
echo -e "run_linux_ros1_simu_concurrent_scandata_and_service_calls.bash finishing, shutdown ros nodes\n"
rosnode kill -a ; sleep 1
killall sick_generic_caller ; sleep 1
killall sick_scan_emulator ; sleep 1
popd
echo -e "\nrun_linux_ros1_simu_concurrent_scandata_and_service_calls.bash finished\n"

