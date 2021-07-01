#!/bin/bash


if [ -f /opt/ros/melodic/setup.bash ] ; then
  source /opt/ros/melodic/setup.bash
  gedit ./make_ros1.bash ./run_ros1_simu_tim7xx_tim7xxS.bash ./run_ros1_simu_lms5xx.bash ./run_ros1_simu_lms1xx.bash & 
fi
if [ -f /opt/ros/eloquent/setup.bash ] ; then
  source /opt/ros/eloquent/setup.bash
  gedit ./make_ros2.bash & 
fi

pushd ../../../..
source ./install/setup.bash

code ./sick_scan_xd_vscode.code-workspace
popd 
