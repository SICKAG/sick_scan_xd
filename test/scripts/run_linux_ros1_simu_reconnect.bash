#!/bin/bash

# randomInt minval maxval returns an uniform distributed random integer value in range [ minval , maxval )
function randomInt(){
  local minval=$1
  local maxval=$2
  echo $(((RANDOM % ($maxval - $minval)) + $minval))
}

# randomFloat minval maxval returns an uniform distributed random float value in range [ minval , maxval )
function randomFloat(){
  local minval=$1
  local maxval=$2
  echo "($maxval - $minval) * 0.000030518 * $RANDOM + $minval" | bc
}

# killSimulation kills all ros nodes, sick_generic_caller and sick_scan_emulator
function killSimulation () {
  rosnode kill -a ; sleep 1
  killall sick_generic_caller ; sleep 1
  killall sick_scan_emulator ; sleep 1
}

# killRestartEmulator delay kills sick_scan_emulator and restarts sick_scan_emulator after a given delay in seconds
function killRestartEmulator () {
  local delay=$1
  echo -e "run_simu_reconnnect.bash: kill sick_scan_emulator, restart in $delay seconds...\n"
  killall -9 sick_scan_emulator
  sleep $delay
  echo -e "run_simu_reconnnect.bash: restarting sick_scan_emulator...\n"
  roslaunch sick_scan_xd emulator_mrs1104.launch &
}

# killRestartEmulatorOnce delay kills sick_scan_emulator, restarts sick_scan_emulator after a given delay in seconds and waits for the pointcloud messages
function killRestartEmulatorOnce () {
  local delay=$1
  killRestartEmulator $delay
  sleep 20
  rostopic echo -n 10 /cloud > /dev/null # make sure sick_scan_xd driver publishes the pointcloud (otherwise rostopic echo will stuck in an endless loop here)
}

# killRestartEmulatorTwice delay kills sick_scan_emulator and restarts sick_scan_emulator twice with a given delay and waits for the pointcloud messages
function killRestartEmulatorTwice () {
  local delay=$1
  killRestartEmulator $delay
  sleep $delay
  killRestartEmulator $delay
  sleep 20
  rostopic echo -n 10 /cloud > /dev/null # make sure sick_scan_xd driver publishes the pointcloud (otherwise rostopic echo will stuck in an endless loop here)
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
  rostopic echo -n 10 /cloud > /dev/null # make sure sick_scan_xd driver publishes the pointcloud (otherwise rostopic echo will stuck in an endless loop here)
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
  rostopic echo -n 10 /cloud > /dev/null # make sure sick_scan_xd driver publishes the pointcloud (otherwise rostopic echo will stuck in an endless loop here)
}

# Run ros1 simulation with reconnecting

killSimulation
printf "\033c"
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash   ] ; then source /opt/ros/melodic/setup.bash   ; fi
if [ -f /opt/ros/noetic/setup.bash    ] ; then source /opt/ros/noetic/setup.bash    ; fi
if [ -f ./install_isolated/setup.bash ] ; then source ./install_isolated/setup.bash ; fi
if [ -f ./install/setup.bash          ] ; then source ./install/setup.bash          ; fi

echo -e "run_simu_reconnnect.bash: starting mrs1104 emulation\n"

# Start roscore if not yet running
roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
if [ $roscore_running -lt 1 ] ; then 
  roscore &
  sleep 3
fi

# Start sick_scan_xd emulator
roslaunch sick_scan_xd emulator_mrs1104.launch &
sleep 1

# Start rviz
# Note: Due to a bug in opengl 3 in combination with rviz and VMware, opengl 2 should be used by rviz option --opengl 210
# See https://github.com/ros-visualization/rviz/issues/1444 and https://github.com/ros-visualization/rviz/issues/1508 for further details

rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_mrs1104.rviz --opengl 210 &
sleep 1

# Start sick_scan_xd driver for mrs1104
echo -e "run_simu_reconnnect.bash: Launching sick_scan_xd sick_mrs_1xxx.launch\n"
roslaunch sick_scan_xd sick_mrs_1xxx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
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
rostopic echo -n 10 /cloud > /dev/null # make sure sick_scan_xd driver publishes the pointcloud (otherwise rostopic echo will stuck in an endless loop here)
for n in {1..10} ; do  
  echo -e "mrs1104 emulation running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
  if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
  rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
  if [ $rviz_running -lt 1 ] ; then break ; fi
done

# Shutdown
echo -e "run_simu_reconnnect.bash finished, shutdown ros nodes.\n"
killSimulation
echo -e "\nrun_simu_reconnnect.bash finished.\n"
popd

