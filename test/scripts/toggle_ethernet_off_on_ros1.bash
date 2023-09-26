#!/bin/bash

# toggle ethernet off and on for endurance test.
# this script must be run by sudo
#
# source ./install_isolated/setup.bash
# while(true) ; do roslaunch sick_scan_xd sick_mrs_6xxx.launch hostname:=192.168.0.25 ; done
#
# sudo -s
# source /opt/ros/noetic/setup.bash
# ./toggle_ethernet_off_on_ros1.bash

# switch ethernet connection on
function ethernet_on () {
  local eth_if=$1
  ip link set $eth_if up
}

# switch ethernet connection off
function ethernet_off () {
  local eth_if=$1
  ip link set $eth_if down
}

# randomFloat minval maxval returns an uniform distributed random float value in range [ minval , maxval )
function randomFloat () {
  local minval=$1
  local maxval=$2
  echo "($maxval - $minval) * 0.000030518 * $RANDOM + $minval" | bc
} 

# random delay
function delay () {
  local rand_delay=$(randomFloat 60 900) # random delay in range 1 to 15 minutes (60 to 900 seconds)
  #local rand_delay=$(randomFloat 5 20) # random delay in range 1 to 15 minutes (60 to 900 seconds)
  echo -e "\ntoggle_ethernet_off_on_ros1.bash: sleep $rand_delay ...\n"
  sleep $rand_delay
}

eth_if=enp0s31f6

rostopic echo -n 10 /cloud > /dev/null
while(true) ; do
  delay # Random wait (1 to 15 minutes)
  echo -e "\ntoggle_ethernet_off_on_ros1.bash: Switch ethernet off ...\n"
  ethernet_off $eth_if # Swith ethernet off
  delay # Random wait (1 to 15 minutes)
  echo -e "\ntoggle_ethernet_off_on_ros1.bash: Switch ethernet on ...\n"
  ethernet_on $eth_if # Swith ethernet off
  sleep 5 # make sure no point cloud any more
  echo -e "\ntoggle_ethernet_off_on_ros1.bash: waiting for point cloud message ...\n"
  rostopic echo -n 10 /cloud > /dev/null # make sure sick_scan_xd driver publishes the pointcloud (otherwise rostopic echo will stuck in an endless loop here)   
  echo -e "\ntoggle_ethernet_off_on_ros1.bash: point cloud message received.\n"
  sleep 5 # make sure rviz display the point cloud after reconnect
done
