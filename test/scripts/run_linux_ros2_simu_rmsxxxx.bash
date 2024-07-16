#!/bin/bash

# Wait for max 30 seconds, or until 'q' or 'Q' pressed, or until rviz is closed
function waitUntilRvizClosed()
{
    duration_sec=$1
    sleep 1
    for ((cnt=1;cnt<=$duration_sec;cnt++)) ; do  
        echo -e "sick_scan_xd running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
        if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
        rviz_running=`(ps -elf | grep rviz2 | grep -v grep | wc -l)`
        if [ $rviz_running -lt 1 ] ; then break ; fi
    done
}

# Shutdown simulation, kill all nodes and processes
function kill_simu()
{
    echo -e "Finishing rms emulation, shutdown ros nodes\n"
    pkill -f sopas_json_test_server.py
    killall -SIGINT sick_generic_caller ; sleep 1
    killall -SIGINT rviz2 ; sleep 1
    killall -9 sick_generic_caller ; sleep 1
    killall -9 rviz2 ; sleep 1
}

printf "\033c"
pushd ../../../..
if   [ -f /opt/ros/jazzy/setup.bash    ] ; then source /opt/ros/jazzy/setup.bash ; export QT_QPA_PLATFORM=xcb
elif [ -f /opt/ros/humble/setup.bash   ] ; then source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash     ] ; then source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/eloquent/setup.bash ] ; then source /opt/ros/eloquent/setup.bash
fi
source ./install/setup.bash
echo -e "run_simu_rmsxxxx.bash: starting RMSxxxx emulation\n"

# Run rms1xxx emulator, rviz and launch sick_scan_xd sick_rms_xxxx.launch (ascii)
python3 ./src/sick_scan_xd/test/python/sopas_json_test_server.py --tcp_port=2112 --json_file=./src/sick_scan_xd/test/emulator/scandata/20220316-rms1000-ascii.pcapng.json --scandata_id="sSN LMDradardata" --send_rate=10 --verbosity=1 &
ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_ros2_rms2xxx.rviz &
ros2 launch sick_scan_xd sick_rms_xxxx.launch.py hostname:=127.0.0.1 sw_pll_only_publish:=False &
waitUntilRvizClosed 15
kill_simu

# Run rms2xxx emulator, rviz and launch sick_scan_xd sick_rms_xxxx.launch (ascii)
python3 ./src/sick_scan_xd/test/python/sopas_json_test_server.py --tcp_port=2112 --json_file=./src/sick_scan_xd/test/emulator/scandata/20221018_rms_1xxx_ascii_rms2_objects.pcapng.json --scandata_id="sSN LMDradardata" --send_rate=10 --verbosity=1 &
ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_ros2_rms2xxx.rviz &
ros2 launch sick_scan_xd sick_rms_xxxx.launch.py hostname:=127.0.0.1 sw_pll_only_publish:=False &
waitUntilRvizClosed 15
kill_simu

popd
