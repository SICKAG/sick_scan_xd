#!/bin/bash
printf "\033c"
pushd ../../../..
source /opt/ros/melodic/setup.bash
source ./install/setup.bash

echo -e "run_simu_service_examples.bash\n"

# Start roscore if not yet running
roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
if [ $roscore_running -lt 1 ] ; then 
  roscore &
  sleep 3
fi

# Start sick_scan emulator
# roslaunch sick_scan emulator_lms5xx.launch &
roslaunch sick_scan emulator_lms5xx.launch > /dev/null &
sleep 1

# Start sick_scan driver for lms5xx
echo -e "Launching sick_scan sick_lms_5xx.launch\n"
roslaunch sick_scan sick_lms_5xx.launch hostname:=127.0.0.1 &
sleep 10

# Run example ros service calls
sleep 2 ; rosservice list
sleep 2 ; rosservice call /sick_lms_5xx/SCdevicestate "{}" # query device state
sleep 2 ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sMN IsSystemReady'}"
sleep 2 ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sRN SCdevicestate'}"
sleep 2 ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sEN LIDinputstate 1'}"
sleep 2 ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sEN LIDoutputstate 1'}"
sleep 2 ; rosservice call /sick_lms_5xx/ColaMsg "{request: 'sMN LMCstartmeas'}"

# Shutdown
echo -e "Finishing service examples, shutdown ros nodes\n"
rosnode kill -a ; sleep 1
killall sick_generic_caller ; sleep 1
killall sick_scan_emulator ; sleep 1
popd

