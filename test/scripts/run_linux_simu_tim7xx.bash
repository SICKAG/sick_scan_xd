#!/bin/bash

# 
# Run sick_generic_caller with sick_tim_7xx.launch (Linux native with test server)
# 

pushd ../..

# pushd ./demo
# rm -f ./scan.jpg ./scan.csv
# firefox ./image_viewer.html &
# sleep 1
# popd

cp -f ./test/emulator/scandata/sopas_et_field_test_1_2_both_010.pcapng.json /tmp/lmd_scandata.pcapng.json
./build_linux/sick_scan_emulator ./test/emulator/launch/emulator_01_default.launch &
sleep 1
./build_linux/sick_generic_caller ./launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False &
sleep 30

killall sick_scan_emulator
killall sick_generic_caller
popd

