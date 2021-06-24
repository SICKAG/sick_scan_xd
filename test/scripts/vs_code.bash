#!/bin/bash

gedit ./make.bash ./run_simu_tim7xx_tim7xxS.bash ./run_simu_lms5xx.bash ./run_simu_lms1xx.bash & 

pushd ../../../..
source /opt/ros/melodic/setup.bash
source ./install/setup.bash

code ./sick_scan_xd_vscode.code-workspace
popd 
