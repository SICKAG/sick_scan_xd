#!/bin/bash

printf "\033c"

# 
# Run sick_scan_xd_api_test with sick_lms_5xx.launch
# 

pushd ../..
export LD_LIBRARY_PATH=.:./build_linux:$LD_LIBRARY_PATH

# Start emulator
python3 ./test/emulator/test_server.py --scandata_file=./test/emulator/scandata/20210302_lms511.pcapng.scandata.txt --scandata_frequency=20.0 --tcp_port=2112 &
sleep 1

# Start image viewer (simple standalone pointcloud visualization)
firefox ./demo/image_viewer_api_test.html &
sleep 1

# Run sick_scan_xd api example
./build_linux/sick_scan_xd_api_test ./launch/sick_lms_5xx.launch hostname:=127.0.0.1 port:=2112 sw_pll_only_publish:=False
popd

# Finish
pkill -f ./test/emulator/test_server.py
killall sick_scan_xd_api_test
