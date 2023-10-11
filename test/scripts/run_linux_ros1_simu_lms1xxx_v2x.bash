#!/bin/bash

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

# Shutdown simulation, kill all nodes and processes
function kill_simu()
{
    rosnode kill -a ; sleep 1
    killall sick_generic_caller ; sleep 1
    killall sick_scan_emulator ; sleep 1
}

# Start emulator and run sick_scan_xd
function run_simu()
{
    rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_lms1xxx.rviz --opengl 210 &
    emulator_launchfile=$1
    echo -e "Starting emulator $emulator_launchfile\n"
    sleep 1 ; roslaunch sick_scan_xd $emulator_launchfile scanner_type:=sick_lms_1xxx &
    sleep 1 ; echo -e "Launching sick_scan_xd sick_lms_1xxx.launch\n"
    sleep 1 ; roslaunch sick_scan_xd sick_lms_1xxx_v2.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
    sleep 1 ; waitUntilRvizClosed 30
    kill_simu
}

printf "\033c"
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash   ] ; then source /opt/ros/melodic/setup.bash   ; fi
if [ -f /opt/ros/noetic/setup.bash    ] ; then source /opt/ros/noetic/setup.bash    ; fi
if [ -f ./devel_isolated/setup.bash   ] ; then source ./devel_isolated/setup.bash   ; fi

echo -e "run_simu_lms1xxx.bash: starting lms1xxx emulation\n"

# Start roscore if not yet running
roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
if [ $roscore_running -lt 1 ] ; then 
  roscore &
  sleep 3
fi

# Run lms1xxx simulation (firmware version >= 2.0 with scan_freq and ang_res configuration, see https://github.com/SICKAG/sick_scan_xd/issues/122):
# 1. Start rviz
# 2. Start sick_scan_xd emulator with field settings for sick_lms_1xxx and play 
#    20221110-LMS1xxx-150hz-0.75deg.pcapng.json, 20221110-LMS1xxx-75hz-0.375deg.pcapng.json and 20221110-LMS1xxx-37.5hz-0.1875deg.pcapng.json
# 3. Start sick_scan_xd driver for lms1xxx
# 4. Wait for 'q' or 'Q' to exit, until rviz is closed, or max. 60 seconds
# 5. Shutdown
run_simu emulator_lms1xxx-150hz-0.75deg.launch
run_simu emulator_lms1xxx-75hz-0.375deg.launch
run_simu emulator_lms1xxx-37.5hz-0.1875deg.launch

# Shutdown
echo -e "Finishing lms1xxx emulation, shutdown ros nodes\n"
kill_simu

popd

