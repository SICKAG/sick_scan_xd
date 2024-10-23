#!/bin/bash


if [ -f /opt/ros/noetic/setup.bash ] ; then 
    source /opt/ros/noetic/setup.bash
elif [ -f /opt/ros/humble/setup.bash ] ; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash ] ; then 
    source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/eloquent/setup.bash ] ; then 
    source /opt/ros/eloquent/setup.bash
fi

pushd ../../../..
if [ -f /devel_isolated/setup.bash ] ; then 
    source ./devel_isolated/setup.bash
elif [ -f ./install/setup.bash ] ; then
    source ./install/setup.bash
fi

code ./sick_scan_xd_vscode.code-workspace
popd 
