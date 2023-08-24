#!/bin/bash


if [ -f /opt/ros/noetic/setup.bash ] ; then 
    source /opt/ros/noetic/setup.bash
elif [ -f /opt/ros/melodic/setup.bash ] ; then
    source /opt/ros/melodic/setup.bash
elif [ -f /opt/ros/foxy/setup.bash ] ; then 
    source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/eloquent/setup.bash ] ; then 
    source /opt/ros/eloquent/setup.bash
fi

pushd ../../../..
source ./devel_isolated/setup.bash

code ./sick_scan_xd_vscode.code-workspace
popd 
