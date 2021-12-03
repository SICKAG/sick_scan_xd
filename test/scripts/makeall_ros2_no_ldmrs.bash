#!/bin/bash

#
# cleanup
#

printf "\033c"
pushd ../../../..
rm -rf ./log
rm -rf ./build
rm -rf ./install 
popd 

#
# make
#

./make_ros2_no_ldmrs.bash
