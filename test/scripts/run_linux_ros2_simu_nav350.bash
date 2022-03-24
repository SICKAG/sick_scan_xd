#!/bin/bash
printf "\033c"
pushd ../../../..
if [ -f /opt/ros/eloquent/setup.bash ] ; then source /opt/ros/eloquent/setup.bash ; fi
if [ -f /opt/ros/foxy/setup.bash     ] ; then source /opt/ros/foxy/setup.bash     ; fi
source ./install/setup.bash

echo -e "run_simu_nav350.bash: starting NAV-350 emulation\n"

# Start sick_scan emulator
cp -f ./src/sick_scan_xd/test/emulator/scandata/20220323_nav350_binary.pcapng.json /tmp/lmd_scandata.pcapng.json
sleep  1 ; ros2 run sick_scan sick_scan_emulator ./src/sick_scan_xd/test/emulator/launch/emulator_nav350.launch & 

# Start rviz
rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_nav350.rviz &
sleep 1

# Start sick_scan driver for nav350
echo -e "Launching sick_scan sick_nav_350.launch\n"
# ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_nav_350.launch hostname:=192.168.0.1 &
# ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_nav_350.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
ros2 launch sick_scan sick_nav_350.launch.py hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False &
sleep 5

# Send 'sMN LMCstartmeas' to start LMDscandata. This is not required for NAV-350 devices, which send LMDscandata immediately after establishing a tcp connection. In contrary, the emulator waits for the 'sMN LMCstartmeas' command.
ros2 service call /ColaMsg sick_scan/srv/ColaMsgSrv "{request: 'sMN LMCstartmeas'}"
# ros2 service call /ColaMsg sick_scan/srv/ColaMsgSrv "{request: 'sMN Run'}"

# Shutdown
sleep 30
ros2 service call /SickScanExit sick_scan/srv/SickScanExitSrv "{}"
sleep 1
pkill rviz2
killall sick_generic_caller 
killall sick_scan_emulator 
popd

