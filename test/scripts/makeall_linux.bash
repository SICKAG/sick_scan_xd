#!/bin/bash

# 
# Clean and rebuild sick_scan_xd on Linux
# 

pushd ../..
if [ -d ./build_linux ] ; then rm -rf ./build_linux ; fi
popd

./make_linux.bash

