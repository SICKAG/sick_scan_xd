#!/bin/bash

#
# cleanup
#

printf "\033c"
pushd ../../../..
cp -f ./src/sick_scan_xd/package_ros2.xml ./src/sick_scan_xd/package.xml
rm -rf ./log
rm -rf ./build
rm -rf ./install 
popd 

#
# make
#

./make_ros2.bash
