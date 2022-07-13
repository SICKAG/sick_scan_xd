#!/bin/bash

# Run example ros service calls

# Start sick_scan emulator and rviz
function start_emulator()
{
    # Start sick_scan emulator
    # roslaunch sick_scan emulator_01_default.launch &
    cp -f ./src/sick_scan_xd/test/emulator/scandata/sopas_et_field_test_1_2_both_010.pcapng.json /tmp/lmd_scandata.pcapng.json
    ./src/sick_scan_xd/build_linux/sick_scan_emulator ./src/sick_scan_xd/test/emulator/launch/emulator_01_default.launch &
    sleep 1
    # Start rviz
    rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_api_tim7xx.rviz --opengl 210 &
    sleep 1
}  

# Wait for 'q' or 'Q' to exit or until rviz is closed
function waitUntilRvizClosed()
{
    sleep 1
    while true ; do  
        echo -e "tim7xx  emulation running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
        if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
        rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
        if [ $rviz_running -lt 1 ] ; then break ; fi
    done
}

# Start sick_scan_xd_api_test (cpp example)
function start_api_test_cpp()
{
    # Run sick_scan_xd_api_test
    # ./install_isolated/lib/sick_scan/sick_scan_xd_api_test ./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False
    # rosrun --prefix 'gdb -ex run --args' sick_scan sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False"
    rosrun sick_scan sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False" &
}

# Run sick_scan_xd_api_test (python example)
function run_api_test_python()
{
    python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py ./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False
}

function kill_simu()
{
    echo -e "Finishing tim7xx emulation, shutdown ros nodes\n"
    rosnode kill -a ; sleep 1
    killall sick_scan_xd_api_test ; sleep 1
    killall sick_scan_emulator ; sleep 1
}

printf "\033c"
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash   ] ; then source /opt/ros/melodic/setup.bash   ; fi
if [ -f /opt/ros/noetic/setup.bash    ] ; then source /opt/ros/noetic/setup.bash    ; fi
if [ -f ./install_isolated/setup.bash ] ; then source ./install_isolated/setup.bash ; fi
if [ -f ./install/setup.bash          ] ; then source ./install/setup.bash          ; fi

echo -e "run_api_test_tim7xx.bash: starting tim7xx emulation\n"

export LD_LIBRARY_PATH=.:./build_linux:./src/sick_scan_xd/build_linux:$LD_LIBRARY_PATH

# Start roscore if not yet running
roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
if [ $roscore_running -lt 1 ] ; then 
  roscore &
  sleep 3
fi

# Start sick_scan emulator and rviz
start_emulator

# Run sick_scan_xd_api_test (python example)
run_api_test_python

# Wait until rviz is closed and shutdown
waitUntilRvizClosed
kill_simu

# Start sick_scan emulator and rviz
start_emulator

# Run sick_scan_xd_api_test (cpp example)
start_api_test_cpp

# Wait until rviz is closed and shutdown
waitUntilRvizClosed
kill_simu

popd

