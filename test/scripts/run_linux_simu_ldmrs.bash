#!/bin/bash

# 
# Run sick_generic_caller with sick_lms_5xx.launch (Linux native with test server)
# 

pushd ../..

# pushd ./demo
# rm -f ./scan.jpg ./scan.csv
# firefox ./image_viewer.html &
# sleep 1
# popd

./build_linux/test_server ./tools/test_server/config/test_server_ldmrs.launch &
sleep 1
./build_linux/sick_generic_caller ./launch/sick_ldmrs.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
sleep 30

killall test_server
killall sick_generic_caller
popd

