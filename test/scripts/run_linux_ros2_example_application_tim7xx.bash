#!/bin/bash

if [ -d ../../../../src/ros2_example_application ] ; then
    pushd ../../../../src/ros2_example_application/test/scripts
    ./run_linux_ros2_example_application_tim7xx.bash
    popd
fi
