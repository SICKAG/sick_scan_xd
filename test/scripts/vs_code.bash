#!/bin/bash


if [ -f /opt/ros/melodic/setup.bash ] ; then
  source /opt/ros/melodic/setup.bash
fi
if [ -f /opt/ros/eloquent/setup.bash ] ; then
  source /opt/ros/eloquent/setup.bash
fi

pushd ../../../..
source ./install/setup.bash

code ./sick_scan_xd_vscode.code-workspace
popd 
