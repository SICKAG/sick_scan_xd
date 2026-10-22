#!/bin/bash

function simu_killall()
{
  sleep 1 ; pkill -f sick_scan_xd_api_test.py
  sleep 1 ; killall -SIGINT rviz2
  sleep 1 ; killall -SIGINT sick_generic_caller
  sleep 1 ; killall -SIGINT sick_scan_emulator
  sleep 1 ; pkill -9 -f sick_scan_xd_api_test.py
  sleep 1 ; killall -9 rviz2
  sleep 1 ; killall -9 sick_generic_caller
  sleep 1 ; killall -9 sick_scan_emulator 
  sleep 1
}

simu_killall
printf "\033c"

# 
# Build and run minimalistic api usage examples (Python, C, C++)
# 
pushd ../../examples/scripts
./build_run_api_examples_linux.bash
popd

#
#
# Run API test (python example) against simulated TiM7xx
#
pushd ../../../..
# Prefer ROSDISTRO if explicitly set and available.
if [ -n "${ROSDISTRO:-}" ] && [ -f "/opt/ros/${ROSDISTRO}/setup.bash" ]; then
    distro="${ROSDISTRO}"
else
    distro=""

    # Probe installed ROS2 distributions, newest first.
    #
    # lyrical   - supported
    # kilted    - supported
    # jazzy     - LTS, supported
    # iron      - EOL
    # humble    - LTS, supported
    # galactic  - EOL
    # foxy      - EOL
    # eloquent  - EOL
    # dashing   - EOL
    for candidate in \
        lyrical \
        kilted \
        jazzy \
        iron \
        humble \
        galactic \
        foxy \
        eloquent \
        dashing; do

        if [ -f "/opt/ros/${candidate}/setup.bash" ]; then
            distro="${candidate}"
            break
        fi
    done
fi

if [ -z "${distro}" ]; then
    echo "ERROR: No ROS2 distribution found in /opt/ros" >&2
    exit 1
fi

export ROSDISTRO="${distro}"
source "/opt/ros/${ROSDISTRO}/setup.bash"

# Force Qt to use X11/XCB for ROS2 Jazzy and newer.
if [[ "${ROSDISTRO}" == "jazzy" || "${ROSDISTRO}" > "jazzy" ]]; then
    export QT_QPA_PLATFORM=xcb
fi

echo "Using ROS2 ${ROSDISTRO}"
source ./install/setup.bash
export PYTHONPATH=.:./src/sick_scan_xd/python/api:$PYTHONPATH

echo -e "run_linux_ros2_api_test.bash: starting TiM7xx emulation ...\n"
cp -f ./src/sick_scan_xd/test/emulator/scandata/sopas_et_field_test_1_2_both_010.pcapng_full.json /tmp/lmd_scandata.pcapng.json
sleep  1 ; ros2 run sick_scan_xd sick_scan_emulator ./src/sick_scan_xd/test/emulator/launch/emulator_01_default.launch scanner_type:=sick_tim_7xx &

echo -e "run_linux_ros2_api_test.bash: starting sick_scan_xd_api_test (python example) ...\n"
# sleep  1 ; ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 &
sleep  1 ; python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py ./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False &
sleep  1 ; ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_api_tim7xx_ros2.rviz &
sleep 40

echo -e "run_linux_ros2_api_test.bash finished, killing api test and emulator ...\n"
simu_killall
echo -e "run_linux_ros2_api_test.bash finished.\n"
popd

