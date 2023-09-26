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

python3 ./test/emulator/test_server.py --scandata_file=./test/emulator/scandata/20210302_lms511.pcapng.scandata.txt --scandata_frequency=20.0 --tcp_port=2112 &
# python3 ./test/emulator/test_server.py --scandata_file=./test/emulator/scandata/20220505_lms511_wireshark_issue49.pcapng.scandata.txt --scandata_frequency=20.0 --tcp_port=2112 &
sleep 1
./build_linux/sick_generic_caller ./launch/sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &

sleep 30

pkill -f ./test/emulator/test_server.py
killall sick_generic_caller
popd

