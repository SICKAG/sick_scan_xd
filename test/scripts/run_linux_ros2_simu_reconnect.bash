#!/bin/bash

# randomInt minval maxval returns an uniform distributed random integer value in range [ minval , maxval )
function randomInt () {
  local minval=$1
  local maxval=$2
  echo $(((RANDOM % ($maxval - $minval)) + $minval))
}

# randomFloat minval maxval returns an uniform distributed random float value in range [ minval , maxval )
function randomFloat () {
  local minval=$1
  local maxval=$2
  echo "($maxval - $minval) * 0.000030518 * $RANDOM + $minval" | bc
}

# killSimulation kills all ros nodes, sick_generic_caller and sick_scan_emulator
function killSimulation () {
  sleep 1 ; killall -SIGINT rviz2
  sleep 1 ; killall -SIGINT sick_generic_caller
  sleep 1 ; killall -SIGINT sick_scan_emulator
  sleep 1 ; killall -9 rviz2
  sleep 1 ; killall -9 sick_generic_caller
  sleep 1 ; killall -9 sick_scan_emulator 
  sleep 1
}

# "rostopic echo -n 10" makes sure tha sick_scan_xd driver publishes the pointcloud (otherwise rostopic echo will stuck in an endless loop)
function rostopicEcho10Times () {
  # rostopic echo -n 10 /cloud > /dev/null # ROS1
  python3 ./src/sick_scan_xd/test/python/ros2_wait_for_cloud_message.py # ROS2 equivalent
}

# killRestartEmulator delay kills sick_scan_emulator and restarts sick_scan_emulator after a given delay in seconds
function killRestartEmulator () {
  local delay=$1
  echo -e "run_simu_reconnnect.bash: kill sick_scan_emulator, restart in $delay seconds...\n"
  killall -9 sick_scan_emulator
  sleep $delay
  echo -e "run_simu_reconnnect.bash: restarting sick_scan_emulator...\n"
  ros2 run sick_scan_xd sick_scan_emulator ./src/sick_scan_xd/test/emulator/launch/emulator_mrs1104.launch scanner_type:=sick_mrs_1xxx &
}

# killRestartEmulatorOnce delay kills sick_scan_emulator, restarts sick_scan_emulator after a given delay in seconds and waits for the pointcloud messages
function killRestartEmulatorOnce () {
  local delay=$1
  killRestartEmulator $delay
  sleep 20
  rostopicEcho10Times # make sure sick_scan_xd driver publishes the pointcloud (otherwise rostopic echo will stuck in an endless loop here)
}

# killRestartEmulatorTwice delay kills sick_scan_emulator and restarts sick_scan_emulator twice with a given delay and waits for the pointcloud messages
function killRestartEmulatorTwice () {
  local delay=$1
  killRestartEmulator $delay
  sleep $delay
  killRestartEmulator $delay
  sleep 20
  rostopicEcho10Times # make sure sick_scan_xd driver publishes the pointcloud (otherwise rostopic echo will stuck in an endless loop here)
}

# killRestartEmulatorTwice delay kills sick_scan_emulator and restarts sick_scan_emulator three times with a given delay and waits for the pointcloud messages
function killRestartEmulatorThreeTimes () {
  local delay=$1
  killRestartEmulator $delay
  sleep $delay
  killRestartEmulator $delay
  sleep $delay
  killRestartEmulator $delay
  sleep 20
  rostopicEcho10Times # make sure sick_scan_xd driver publishes the pointcloud (otherwise rostopic echo will stuck in an endless loop here)
}

# killRestartEmulatorTwice delay kills sick_scan_emulator and restarts sick_scan_emulator a random number of times with a random delay and finally waits for the pointcloud messages
function killRestartEmulatorRandomly () {
  local rand_repeat=$(randomInt 1 5) # random number in range 1 to 5
  for cnt in $(seq 1 1 $rand_repeat) ; do
    local rand_delay=$(randomFloat 1 10) # random delay in range 1 to 10 seconds
    echo -e "killRestartEmulatorRandomly: killRestartEmulator $cnt of $rand_repeat repetitions with rand_delay = $rand_delay\n"
    killRestartEmulator $rand_delay
    sleep $rand_delay
  done
  sleep 20
  rostopicEcho10Times # make sure sick_scan_xd driver publishes the pointcloud (otherwise rostopic echo will stuck in an endless loop here)
}

# Run ros2 simulation with reconnecting

killSimulation
printf "\033c"
pushd ../../../..
if   [ -f /opt/ros/jazzy/setup.bash    ] ; then source /opt/ros/jazzy/setup.bash ; export QT_QPA_PLATFORM=xcb
elif [ -f /opt/ros/humble/setup.bash   ] ; then source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash     ] ; then source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/eloquent/setup.bash ] ; then source /opt/ros/eloquent/setup.bash
fi
source ./install/setup.bash

echo -e "run_simu_reconnnect.bash: starting mrs1104 emulation\n"

# Start sick_scan_xd emulator
cp -f ./src/sick_scan_xd/test/emulator/scandata/20210722_143600_ros2_mrs1104_sick_scan_xd.pcapng.json /tmp/lmd_scandata.pcapng.json
ros2 run sick_scan_xd sick_scan_emulator ./src/sick_scan_xd/test/emulator/launch/emulator_mrs1104.launch scanner_type:=sick_mrs_1xxx &
sleep 1

# Start rviz
ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_ros2_mrs1104.rviz &
sleep 1

# Start sick_scan_xd driver for mrs1104
echo -e "run_simu_reconnnect.bash: Launching sick_scan_xd sick_mrs_1xxx.launch\n"
ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_mrs_1xxx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False & 
sleep 10

# Shutdown and restart emulator during both init and measurement, driver has to reconnect
for delay in 0.1 0.5 1.0 2.0 3.0 5.0 10.0 ; do
  killRestartEmulatorOnce $delay
done
for delay in 1.0 2.0 3.0 5.0 10.0 ; do
  killRestartEmulatorTwice $delay
done
for delay in 1.0 2.0 3.0 5.0 10.0 ; do
  killRestartEmulatorThreeTimes $delay
done
for n in {1..10} ; do
  killRestartEmulatorRandomly  
done

# Ensure that driver receives tcp messages after reconnect
rostopicEcho10Times # make sure sick_scan_xd driver publishes the pointcloud (otherwise rostopic echo will stuck in an endless loop here)
for n in {1..10} ; do  
  echo -e "mrs1104 emulation running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
  if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
  rviz_running=`(ps -elf | grep rviz2 | grep -v grep | wc -l)`
  if [ $rviz_running -lt 1 ] ; then break ; fi
done

# Shutdown
echo -e "run_simu_reconnnect.bash finished, shutdown ros nodes.\n"
killSimulation
echo -e "\nrun_simu_reconnnect.bash finished.\n"
popd

