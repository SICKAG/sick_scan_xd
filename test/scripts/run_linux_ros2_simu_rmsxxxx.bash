#!/bin/bash

# Wait until RViz is closed, 'q' is pressed, or the timeout expires.
function waitUntilRvizClosed()
{
    duration_sec=$1
    sleep 1

    for ((cnt=1; cnt<=$duration_sec; cnt++)); do
        echo -e "sick_scan_xd running. Close rviz or press 'q' to exit..."
        read -t 1.0 -n1 -s key

        # Allow manual termination by pressing 'q' or 'Q'.
        if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break; fi

        # Stop waiting if RViz has already been closed.
        rviz_running=$(ps -elf | grep rviz2 | grep -v grep | wc -l)
        if [ "$rviz_running" -lt 1 ]; then break; fi
    done
}

# Stop the emulator, ROS nodes, and RViz.
function kill_simu()
{
    echo -e "Finishing rms emulation, shutdown ros nodes\n"

    # Stop the SOPAS test server.
    pkill -f sopas_json_test_server.py

    # Try graceful shutdown first.
    killall -SIGINT sick_generic_caller
    sleep 1
    killall -SIGINT rviz2
    sleep 1

    # Force termination if processes are still running.
    killall -9 sick_generic_caller
    sleep 1
    killall -9 rviz2
    sleep 1
}

# Clear terminal output.
printf "\033c"

# Determine the repository root independent of the current working directory.
BASE="$(realpath "$(dirname "${BASH_SOURCE[0]}")/../../../..")"

# Example of using $BASE: python3 "$BASE/src/sick_scan_xd/test/python/sopas_json_test_server.py" ...
# Example of using $BASE: ros2 run rviz2 rviz2 -d "$BASE/src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_ros2_rms2xxx.rviz" &

# Source the installed ROS 2 distribution.
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    export QT_QPA_PLATFORM=xcb
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash ]; then
    source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/eloquent/setup.bash ]; then
    source /opt/ros/eloquent/setup.bash
fi

# Source the local sick_scan_xd workspace.
source "$BASE/install/setup.bash"

echo -e "run_simu_rmsxxxx.bash: starting RMSxxxx emulation\n"

# Start the SOPAS test server and replay recorded RMS radar data.
python3 "$BASE/src/sick_scan_xd/test/python/sopas_json_test_server.py" \
    --tcp_port=2111 \
    --json_file="$BASE/src/sick_scan_xd/test/emulator/scandata/20260319_rms_1xxx_ascii_rms2_objects.pcapng.json" \
    --scandata_id="sSN LMDradardata" \
    --send_rate=10 \
    --verbosity=1 &

# Start RViz with the RMS visualization configuration.
ros2 run rviz2 rviz2 \
    -d "$BASE/src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_ros2_rms2xxx.rviz" &

# Start the sick_scan_xd RMS driver connected to the local emulator.
ros2 launch sick_scan_xd sick_rms_xxxx.launch.py \
    hostname:=127.0.0.1 \
    sw_pll_only_publish:=False &

# Run the simulation for up to 15 seconds and clean up all processes afterwards.
waitUntilRvizClosed 15
kill_simu
