#!/bin/bash

# Run example ros service calls

# Start tim7xx emulator and rviz
function start_tim7xx_emulator()
{
    echo -e "\n-------------------------------------------"
    echo -e "run_api_test: starting tim7xx emulation ...\n"
    # Start sick_scan emulator
    # roslaunch sick_scan emulator_01_default.launch &
    cp -f ./src/sick_scan_xd/test/emulator/scandata/sopas_et_field_test_1_2_both_010.pcapng.json /tmp/lmd_scandata.pcapng.json
    ./src/sick_scan_xd/build_linux/sick_scan_emulator ./src/sick_scan_xd/test/emulator/launch/emulator_01_default.launch &
    # Start rviz
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_api_tim7xx.rviz --opengl 210 &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_api_tim7xx_polar.rviz --opengl 210 &
    sleep 1
}

# Start mrs1xxx emulator and rviz
function start_mrs1xxx_emulator()
{
    echo -e "\n-------------------------------------------"
    echo -e "run_api_test: starting mrs1xxx emulation ...\n"
    roslaunch sick_scan emulator_mrs1xxx_imu.launch &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_api_mrs1104.rviz --opengl 210 &
    sleep 1
}

# Start ldmrs emulator and rviz
function start_ldmrs_emulator()
{
    echo -e "\n------------------------------------------"
    echo -e "run_api_test: starting ldmrs emulation ...\n"
    roslaunch sick_scan test_server_ldmrs.launch &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_api_ldmrs.rviz --opengl 210 &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_api_ldmrs_polar.rviz --opengl 210 &
    sleep 1
}
 
# Start mrs100 (multiscan136) emulator and rviz
function start_mrs100_emulator()
{
    echo -e "\n----------------------------------------------------------"
    echo -e "run_api_test: starting mrs100 (multiscan136) emulation ...\n"
    # roslaunch sick_scan test_server_ldmrs.launch &
    python3 ./src/sick_scan_xd/test/python/mrs100_sopas_test_server.py --tcp_port=2111 --cola_binary=0 &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_api_mrs100.rviz --opengl 210 &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_api_mrs100_polar.rviz --opengl 210 &
    sleep 1
}
 
# Wait for max 30 seconds, or until 'q' or 'Q' pressed, or until rviz is closed
function waitUntilRvizClosed()
{
    duration_sec=$1
    sleep 1
    for ((cnt=1;cnt<=$duration_sec;cnt++)) ; do  
        echo -e "sick_scan_xd api test running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
        if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
        rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
        if [ $rviz_running -lt 1 ] ; then break ; fi
    done
}

# Shutdown simulation, kill all nodes and processes
function kill_simu()
{
    echo -e "Finishing sick_scan emulation, shutdown ros nodes\n"
    rosnode kill sick_scan_api_test_py ; sleep 1
    rosnode kill sick_scan_api_test ; sleep 1
    rosnode kill sick_scan_emulator ; sleep 1
    rosnode kill -a ; sleep 1
    killall sick_scan_xd_api_test
    killall sick_scan_emulator
    pkill -f mrs100_sopas_test_server.py
    pkill -f sick_scan_xd_api_test.py
    sleep 1
    killall -9 sick_scan_xd_api_test
    killall -9 sick_scan_emulator
    pkill -9 -f mrs100_sopas_test_server.py
    pkill -9 -f sick_scan_xd_api_test.py
}

#
# API test against simulated TiM7xx, LDMRS and MRS100
#

kill_simu
printf "\033c"
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash     ] ; then source /opt/ros/melodic/setup.bash   ; fi
if [ -f /opt/ros/noetic/setup.bash      ] ; then source /opt/ros/noetic/setup.bash    ; fi
if [ -f ./devel_isolated/setup.bash     ] ; then source ./devel_isolated/setup.bash   ; fi
# if [ -f ./install_isolated/setup.bash ] ; then source ./install_isolated/setup.bash ; fi
# if [ -f ./install/setup.bash          ] ; then source ./install/setup.bash          ; fi

export LD_LIBRARY_PATH=.:./build_linux:./src/sick_scan_xd/build_linux:$LD_LIBRARY_PATH

# Start roscore if not yet running
roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
if [ $roscore_running -lt 1 ] ; then 
  roscore &
  sleep 3
fi

# Start tim7xx emulator and run sick_scan_xd_api_test (python example)
start_tim7xx_emulator
python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py ./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False &
waitUntilRvizClosed 10
kill_simu

# Start mrs100 (multiscan136) emulator and run sick_scan_xd_api_test (python example)
start_mrs100_emulator
python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py ./src/sick_scan_xd/launch/sick_scansegment_xd.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 publish_frame_id:=cloud &
# Play pcapng-files to emulate MRS100 output
python3 ./src/sick_scan_xd/test/python/mrs100_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20210929_mrs100_token_udp.pcapng --udp_port=2115 --repeat=2
python3 ./src/sick_scan_xd/test/python/mrs100_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20210929_mrs100_cola-a-start-stop-scandata-output.pcapng --udp_port=2115
waitUntilRvizClosed 1
kill_simu

# Start ldmrs emulator and run sick_scan_xd_api_test (python example)
start_ldmrs_emulator
python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py ./src/sick_scan_xd/launch/sick_ldmrs.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
waitUntilRvizClosed 15
kill_simu

# Start mrs1xxx emulator with imu messages and run sick_scan_xd_api_test (python example)
start_mrs1xxx_emulator
python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py ./src/sick_scan_xd/launch/sick_mrs_1xxx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
waitUntilRvizClosed 10
kill_simu

# Start tim7xx emulator and run sick_scan_xd_api_test (cpp example)
start_tim7xx_emulator
# ./install_isolated/lib/sick_scan/sick_scan_xd_api_test ./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False
# rosrun --prefix 'gdb -ex run --args' sick_scan sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False"
rosrun sick_scan sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False" &
waitUntilRvizClosed 10
kill_simu

# Start mrs1xxx emulator with imu messages and run sick_scan_xd_api_test (cpp example)
start_mrs1xxx_emulator
rosrun sick_scan sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_mrs_1xxx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False" &
waitUntilRvizClosed 10
kill_simu

echo -e "run_api_test finished\n"
popd

