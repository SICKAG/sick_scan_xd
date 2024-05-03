#!/bin/bash

# Start tim7xx emulator and rviz
function start_tim7xx_emulator()
{
    echo -e "\n-------------------------------------------"
    echo -e "run_api_test: starting tim7xx emulation ...\n"
    roslaunch sick_scan_xd emulator_01_default.launch &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_api_tim7xx.rviz --opengl 210 &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_api_tim7xx_polar.rviz --opengl 210 &
    sleep 1
}

# Start mrs1xxx emulator and rviz
function start_mrs1xxx_emulator()
{
    echo -e "\n-------------------------------------------"
    echo -e "run_api_test: starting mrs1xxx emulation ...\n"
    roslaunch sick_scan_xd emulator_mrs1xxx_imu.launch &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_api_mrs1104.rviz --opengl 210 &
    sleep 1
}

# Start ldmrs emulator and rviz
function start_ldmrs_emulator()
{
    echo -e "\n------------------------------------------"
    echo -e "run_api_test: starting ldmrs emulation ...\n"
    roslaunch sick_scan_xd test_server_ldmrs.launch &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_api_ldmrs.rviz --opengl 210 &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_api_ldmrs_polar.rviz --opengl 210 &
    sleep 1
}
 
# Start multiScan emulator and rviz
function start_multiScan_emulator()
{
    echo -e "\n----------------------------------------------------------"
    echo -e "run_api_test: starting multiScan emulation ...\n"
    python3 ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 &

    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_api_multiscan.rviz --opengl 210 &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_api_multiscan_polar.rviz --opengl 210 &
    sleep 1
}

# Start rms2xxx radar emulator and rviz
function start_rms2xxx_emulator()
{
    echo -e "\n-------------------------------------------"
    echo -e "run_api_test: starting rms2xxx radar emulation ...\n"
    python3 ./src/sick_scan_xd/test/python/sopas_json_test_server.py --tcp_port=2112 --json_file=./src/sick_scan_xd/test/emulator/scandata/20221018_rms_1xxx_ascii_rms2_objects.pcapng.json --scandata_id="sSN LMDradardata" --send_rate=10 --verbosity=1 &
    sleep 1
}

# Start nav350 emulator and rviz
function start_nav350_emulator()
{
    echo -e "\n----------------------------------------------------------"
    echo -e "run_api_test: starting nav350 emulation ...\n"
    python3 ./src/sick_scan_xd/test/python/sopas_json_test_server.py --tcp_port=2112 --json_file=./src/sick_scan_xd/test/emulator/scandata/20230126_nav350_4reflectors_moving.pcapng.json --scandata_id="sAN mNPOSGetData" --send_rate=8 --verbosity=0 &
    sleep 1 ; rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_nav350.rviz --opengl 210 &
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
    echo -e "Finishing sick_scan_xd emulation, shutdown ros nodes\n"
    rosnode kill sick_scan_api_test_py ; sleep 1
    rosnode kill sick_scan_api_test ; sleep 1
    rosnode kill sick_scan_emulator ; sleep 1
    rosnode kill -a ; sleep 1
    killall sick_scan_xd_api_test
    killall sick_scan_emulator
    pkill -f multiscan_sopas_test_server.py
    pkill -f sick_scan_xd_api_test.py
    pkill -f sopas_json_test_server.py
    sleep 1
    killall -9 sick_scan_xd_api_test
    killall -9 sick_scan_emulator
    pkill -9 -f multiscan_sopas_test_server.py
    pkill -9 -f sick_scan_xd_api_test.py
    pkill -9 -f sopas_json_test_server.py
}

kill_simu
printf "\033c"

# 
# Build and run minimalistic api usage examples (Python, C, C++)
# 
pushd ../../examples/scripts
# ./build_run_api_examples_linux.bash
popd

#
# API test against simulated TiM7xx, multiScan, LDMRS, MRS1xxx and RMSxxxx
#

pushd ../../../..
if [ -d ./log    ] ; then rm -rf ./log ; fi ; mkdir -p ./log
if [ -f /opt/ros/melodic/setup.bash     ] ; then source /opt/ros/melodic/setup.bash   ; fi
if [ -f /opt/ros/noetic/setup.bash      ] ; then source /opt/ros/noetic/setup.bash    ; fi
if [ -f ./devel_isolated/setup.bash     ] ; then source ./devel_isolated/setup.bash   ; fi
# if [ -f ./install_isolated/setup.bash ] ; then source ./install_isolated/setup.bash ; fi
# if [ -f ./install/setup.bash          ] ; then source ./install/setup.bash          ; fi

export LD_LIBRARY_PATH=.:./build_linux:./src/sick_scan_xd/build_linux:$LD_LIBRARY_PATH
export PYTHONPATH=.:./src/sick_scan_xd/python/api:$PYTHONPATH

# Start roscore if not yet running
roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
if [ $roscore_running -lt 1 ] ; then 
  roscore &
  sleep 3
fi


#
# Run python examples
#

# Start tim7xx emulator and run sick_scan_xd_api_test (python example)
start_tim7xx_emulator
python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py ./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
waitUntilRvizClosed 40
kill_simu

# Start multiScan emulator and run sick_scan_xd_api_test (python example)
start_multiScan_emulator
python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py ./src/sick_scan_xd/launch/sick_multiscan.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 publish_frame_id:=cloud scandataformat:=1 &
# Play pcapng-files to emulate multiScan output
python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20210929_mrs100_token_udp.pcapng --udp_port=2115 --repeat=2
python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20210929_mrs100_cola-a-start-stop-scandata-output.pcapng --udp_port=2115
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

# Start rms2xxx radar emulator and run sick_scan_xd_api_test (python example)
start_rms2xxx_emulator
python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py ./src/sick_scan_xd/launch/sick_rms_xxxx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
sleep 10
kill_simu

# Start nav350 emulator and run sick_scan_xd_api_test (python example)
start_nav350_emulator
python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py ./src/sick_scan_xd/launch/sick_nav_350.launch hostname:=127.0.0.1 nav_do_initial_mapping:=True sw_pll_only_publish:=False &
waitUntilRvizClosed 15
kill_simu

#
# Run cpp examples
#

# Start tim7xx emulator and run sick_scan_xd_api_test (cpp example)
start_tim7xx_emulator
# ./install_isolated/lib/sick_scan_xd/sick_scan_xd_api_test ./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False
# rosrun --prefix 'gdb -ex run --args' sick_scan_xd sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False"
rosrun sick_scan_xd sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False" &
waitUntilRvizClosed 40
kill_simu

# Start multiScan emulator and run sick_scan_xd_api_test (cpp example)
start_multiScan_emulator
rosrun sick_scan_xd sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_multiscan.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 publish_frame_id:=cloud" scandataformat:=1 &
# Play pcapng-files to emulate multiScan output
python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20210929_mrs100_token_udp.pcapng --udp_port=2115 --repeat=2
python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20210929_mrs100_cola-a-start-stop-scandata-output.pcapng --udp_port=2115
waitUntilRvizClosed 1
kill_simu

# Start ldmrs emulator and run sick_scan_xd_api_test (cpp example)
start_ldmrs_emulator
rosrun sick_scan_xd sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_ldmrs.launch hostname:=127.0.0.1 sw_pll_only_publish:=False" &
waitUntilRvizClosed 15
kill_simu

# Start mrs1xxx emulator with imu messages and run sick_scan_xd_api_test (cpp example)
start_mrs1xxx_emulator
rosrun sick_scan_xd sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_mrs_1xxx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False" &
waitUntilRvizClosed 10
kill_simu

# Start rms2xxx radar emulator and run sick_scan_xd_api_test (cpp example)
start_rms2xxx_emulator
rosrun sick_scan_xd sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_rms_xxxx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False" &
sleep 10
kill_simu

# Start nav350 emulator and run sick_scan_xd_api_test (cpp example)
start_nav350_emulator
rosrun sick_scan_xd sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_nav_350.launch hostname:=127.0.0.1 nav_do_initial_mapping:=True sw_pll_only_publish:=False" &
waitUntilRvizClosed 15
kill_simu

#
# Run python examples with polling
#

# Start tim7xx emulator and run sick_scan_xd_api_test (python example with polling)
start_tim7xx_emulator
python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py _polling:=1 ./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
waitUntilRvizClosed 40
kill_simu

# Start multiScan emulator and run sick_scan_xd_api_test (python example with polling)
start_multiScan_emulator
python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py _polling:=1 ./src/sick_scan_xd/launch/sick_multiscan.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 publish_frame_id:=cloud scandataformat:=1 &
# Play pcapng-files to emulate multiScan output
python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20210929_mrs100_token_udp.pcapng --udp_port=2115 --repeat=2
python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20210929_mrs100_cola-a-start-stop-scandata-output.pcapng --udp_port=2115
waitUntilRvizClosed 1
kill_simu

# Start ldmrs emulator and run sick_scan_xd_api_test (python example with polling)
start_ldmrs_emulator
python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py _polling:=1 ./src/sick_scan_xd/launch/sick_ldmrs.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
waitUntilRvizClosed 15
kill_simu

# Start mrs1xxx emulator with imu messages and run sick_scan_xd_api_test (python example with polling)
start_mrs1xxx_emulator
python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py _polling:=1 ./src/sick_scan_xd/launch/sick_mrs_1xxx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
waitUntilRvizClosed 10
kill_simu

# Start rms2xxx radar emulator and run sick_scan_xd_api_test (python example with polling)
start_rms2xxx_emulator
python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py _polling:=1 ./src/sick_scan_xd/launch/sick_rms_xxxx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
sleep 10
kill_simu

# Start nav350 emulator and run sick_scan_xd_api_test (python example with polling)
start_nav350_emulator
python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py _polling:=1 ./src/sick_scan_xd/launch/sick_nav_350.launch hostname:=127.0.0.1 nav_do_initial_mapping:=True sw_pll_only_publish:=False &
waitUntilRvizClosed 15
kill_simu

#
# Run cpp examples with polling
#

# Start tim7xx emulator and run sick_scan_xd_api_test (cpp example with polling)
start_tim7xx_emulator
# rosrun --prefix 'gdb -ex run --args' sick_scan_xd sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False" _polling:=1
rosrun sick_scan_xd sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False" _polling:=1 &
waitUntilRvizClosed 40
kill_simu

# Start multiScan emulator and run sick_scan_xd_api_test (cpp example with polling)
start_multiScan_emulator
rosrun sick_scan_xd sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_multiscan.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 scandataformat:=1 publish_frame_id:=cloud" _polling:=1 &
# Play pcapng-files to emulate multiScan output
python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20210929_mrs100_token_udp.pcapng --udp_port=2115 --repeat=2
python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20210929_mrs100_cola-a-start-stop-scandata-output.pcapng --udp_port=2115
waitUntilRvizClosed 1
kill_simu

# Start ldmrs emulator and run sick_scan_xd_api_test (cpp example with polling)
start_ldmrs_emulator
rosrun sick_scan_xd sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_ldmrs.launch hostname:=127.0.0.1 sw_pll_only_publish:=False" _polling:=1 &
waitUntilRvizClosed 15
kill_simu

# Start mrs1xxx emulator with imu messages and run sick_scan_xd_api_test (cpp example with polling)
start_mrs1xxx_emulator
rosrun sick_scan_xd sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_mrs_1xxx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False" _polling:=1 &
waitUntilRvizClosed 10
kill_simu

# Start rms2xxx radar emulator and run sick_scan_xd_api_test (cpp example with polling)
start_rms2xxx_emulator
rosrun sick_scan_xd sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_rms_xxxx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False" _polling:=1 &
sleep 10
kill_simu

# Start nav350 emulator and run sick_scan_xd_api_test (cpp example with polling)
start_nav350_emulator
rosrun sick_scan_xd sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_nav_350.launch hostname:=127.0.0.1 nav_do_initial_mapping:=True sw_pll_only_publish:=False" _polling:=1 &
waitUntilRvizClosed 15
kill_simu

echo -e "run_api_test finished\n"
popd

