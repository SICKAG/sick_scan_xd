#!/bin/bash

printf "\033c"

# 
# Build and run minimalistic api usage examples (Python, C, C++)
# 
pushd ../../examples/scripts
./build_run_api_examples_linux.bash
popd

# 
# Run sick_scan_xd_api_test with sick_tim_7xx.launch (Linux native with test server)
# 

pushd ../..
export LD_LIBRARY_PATH=.:./build_linux:$LD_LIBRARY_PATH

# Start emulator
cp -f ./test/emulator/scandata/sopas_et_field_test_1_2_both_010.pcapng.json /tmp/lmd_scandata.pcapng.json
./build_linux/sick_scan_emulator ./test/emulator/launch/emulator_01_default.launch &
sleep 1

# Start image viewer (simple standalone pointcloud visualization)
firefox ./demo/image_viewer_api_test.html &

# Start sick_scan_xd api example
./build_linux/sick_scan_xd_api_test ./launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False
popd

killall sick_scan_emulator
killall sick_scan_xd_api_test

