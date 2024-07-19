#!/bin/bash

function simu_killall()
{
  killall sick_generic_caller 
  killall sick_scan_emulator 
  killall test_server 
}

simu_killall
printf "\033c"
pushd ../../../..
if   [ -f /opt/ros/jazzy/setup.bash    ] ; then source /opt/ros/jazzy/setup.bash ; export QT_QPA_PLATFORM=xcb
elif [ -f /opt/ros/humble/setup.bash   ] ; then source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash     ] ; then source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/eloquent/setup.bash ] ; then source /opt/ros/eloquent/setup.bash
fi
source ./install/setup.bash

# Start sick_scan_xd emulator and sick_generic_caller
echo -e "run_linux_ros2_simu_service_examples\n"
sleep  1 ; ros2 run sick_scan_xd sick_scan_emulator ./src/sick_scan_xd/test/emulator/launch/emulator_01_default.launch scanner_type:=sick_tim_7xx > /dev/null &
sleep  1 ; ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False & 

# Run example ros service calls
sleep 2 ; ros2 service list
sleep 2 ; ros2 service call /SCdevicestate sick_scan_xd/srv/SCdevicestateSrv "{}" # query device state
sleep 2 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sMN IsSystemReady'}"
sleep 2 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sRN SCdevicestate'}"
sleep 2 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sEN LIDinputstate 1'}"
sleep 2 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sEN LIDoutputstate 1'}"
sleep 2 ; ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sMN LMCstartmeas'}"

# Shutdown
echo -e "Finishing service examples, shutdown ros nodes\n" 
simu_killall

popd

