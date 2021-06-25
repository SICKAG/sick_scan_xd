#!/bin/bash

gedit ./make_ros1.bash ./run_ros1_simu_tim7xx_tim7xxS.bash ./run_ros1_simu_lms5xx.bash ./run_ros1_simu_lms1xx.bash & 

pushd ../../../..
source /opt/ros/melodic/setup.bash
source ./install/setup.bash

code ./sick_scan_xd_vscode.code-workspace
popd 
