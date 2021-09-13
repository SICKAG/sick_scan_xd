#!/bin/bash

# 
# Clean and rebuild sick_scan_xd on Linux
# 

printf "\033c"
if [ -d ../../build_linux ] ; then rm -rf ../../build_linux ; fi
if [ -d ../../../libsick_ldmrs/build ] ; then rm -rf ../../../libsick_ldmrs/build ; fi

./make_linux_no_ldmrs.bash

