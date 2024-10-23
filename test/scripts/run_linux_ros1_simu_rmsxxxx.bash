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
    echo -e "Finishing rms emulation, shutdown ros nodes\n"
    pkill -f sopas_json_test_server.py
    # rosnode kill -a ; sleep 3
    killall rviz
    killall sick_generic_caller ; sleep 3
    killall -9 sick_generic_caller
}

printf "\033c"
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash     ] ; then source /opt/ros/melodic/setup.bash   ; fi
if [ -f /opt/ros/noetic/setup.bash      ] ; then source /opt/ros/noetic/setup.bash    ; fi
if [ -f ./devel_isolated/setup.bash     ] ; then source ./devel_isolated/setup.bash   ; fi

echo -e "run_simu_rmsxxxx.bash: starting RMSxxxx emulation\n"

# Start roscore if not yet running
# roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
# if [ $roscore_running -lt 1 ] ; then 
#   roscore &
#   sleep 3
# fi

# Run rms2xxx emulator, rviz and launch sick_scan_xd sick_rms_xxxx.launch (ascii) with LMDscandata and LIDoutputstate telegrams
# Settings for 20221018_rms_1xxx_ascii_rms2_objects.pcapng.json: --scandata_id="sSN LMDradardata", activate_lidoutputstate:=false
# 20240909-rms2xxx-field-evaluation.pcapng.json: --scandata_id="sSN LMDradardata,sSN LIDoutputstate"
# 20240909-rms2xxx-field-evaluation-with-rotating-fan.pcapng.json

python3 ./src/sick_scan_xd/test/python/sopas_json_test_server.py --tcp_port=2112 --json_file=./src/sick_scan_xd/test/emulator/scandata/20240909-rms2xxx-field-evaluation.pcapng.json --scandata_id="sSN LMDradardata,sSN LIDoutputstate" --send_rate=10 --verbosity=1 &
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_rms2xxx.rviz --opengl 210 &
roslaunch sick_scan_xd sick_rms_xxxx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
waitUntilRvizClosed 60
kill_simu

# Run rms1xxx emulator, rviz and launch sick_scan_xd sick_rms_xxxx.launch (ascii)
# python3 ./src/sick_scan_xd/test/python/sopas_json_test_server.py --tcp_port=2112 --json_file=./src/sick_scan_xd/test/emulator/scandata/20220316-rms1000-ascii.pcapng.json --scandata_id="sSN LMDradardata" --send_rate=10 --verbosity=1 &
# rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_rms1xxx.rviz --opengl 210 &
# roslaunch sick_scan_xd sick_rms_xxxx.launch hostname:=127.0.0.1 activate_lidoutputstate:=false sw_pll_only_publish:=False &
# waitUntilRvizClosed 60
# kill_simu

# Run rms2xxx emulator, rviz and launch sick_scan_xd sick_rms_xxxx.launch (ascii)
# python3 ./src/sick_scan_xd/test/python/sopas_json_test_server.py --tcp_port=2112 --json_file=./src/sick_scan_xd/test/emulator/scandata/20221018_rms_1xxx_ascii_rms2_objects.pcapng.json --scandata_id="sSN LMDradardata" --send_rate=10 --verbosity=1 &
# rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_rms2xxx.rviz --opengl 210 &
# roslaunch sick_scan_xd sick_rms_xxxx.launch hostname:=127.0.0.1 activate_lidoutputstate:=false sw_pll_only_publish:=False &
# waitUntilRvizClosed 60
# kill_simu

popd
