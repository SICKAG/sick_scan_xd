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

printf "\033c"
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash   ] ; then source /opt/ros/melodic/setup.bash   ; fi
if [ -f /opt/ros/noetic/setup.bash    ] ; then source /opt/ros/noetic/setup.bash    ; fi
if [ -f ./devel_isolated/setup.bash   ] ; then source ./devel_isolated/setup.bash   ; fi

echo -e "run_simu_lms1xx.bash: starting lms1xx emulation\n"

# Start roscore if not yet running
roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
if [ $roscore_running -lt 1 ] ; then 
  roscore &
  sleep 3
fi

# Start sick_scan_xd emulator with field settings for sick_lms_1xx, play 20210302_lms111.pcapng_full.json
roslaunch sick_scan_xd emulator_lms1xx.launch scanner_type:=sick_lms_1xx &
# Start rviz
sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_lms1xx.rviz --opengl 210 &
# Start sick_scan_xd driver for lms1xx
sleep 1 ; echo -e "Launching sick_scan_xd sick_lms_1xx.launch\n"
roslaunch sick_scan_xd sick_lms_1xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &

# Wait for 'q' or 'Q' to exit or until rviz is closed
waitUntilRvizClosed 60

# Shutdown and restart with 20220802_lms111.pcapng.json
kill_simu
echo -e "Restart sick_scan_xd lms111 simulation with 20220802_lms111.pcapng.json\n"
roslaunch sick_scan_xd emulator_lms111.launch scanner_type:=sick_lms_111 &
sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_lms1xx.rviz --opengl 210 &
sleep 1 ; roslaunch sick_scan_xd sick_lms_1xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
waitUntilRvizClosed 60

# Shutdown
echo -e "Finishing lms1xx emulation, shutdown ros nodes\n"
kill_simu
popd

