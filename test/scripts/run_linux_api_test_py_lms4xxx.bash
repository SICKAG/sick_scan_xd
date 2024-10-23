#!/bin/bash

printf "\033c"

# 
# Run sick_scan_xd_api_test.py with sick_lms_4xxx.launch (Linux native with test server)
# 

pushd ../..
export LD_LIBRARY_PATH=.:./build_linux:$LD_LIBRARY_PATH
export PYTHONPATH=.:./python/api:$PYTHONPATH

# Start emulator
python3 ./test/python/sopas_json_test_server.py --tcp_port=2112 --json_file=./test/emulator/scandata/20240522-LMS4xxx.pcapng.json --verbosity=0 &
sleep 1

# Start sick_scan_xd api example
python3 ./test/python/sick_scan_xd_api/sick_scan_xd_api_test.py ./launch/sick_lms_4xxx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
popd

# Run simulation for 30 seconds and exit
sleep 30
echo -e "\nFinishing simulation after 30 sec, killing sick_scan_xd_api_test...\n"
pkill -f sick_scan_xd_api_test.py
pkill -f sopas_json_test_server.py
echo -e "Finished simulation.\n"

