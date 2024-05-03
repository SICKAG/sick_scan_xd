#!/bin/bash

# killall and cleanup after exit
function killall_cleanup()
{
  killall sick_scan_xd_api_test
  pkill -f sick_scan_xd_api_test.py
  pkill -f multiscan_sopas_test_server.py
  pkill -f multiscan_pcap_player.py
}

# Play pcapng-files to emulate multiScan output
function multiscan_pcap_player()
{
  sleep 3
  python3 ./test/python/multiscan_perftest_player.py --udp_port=2115 --repeat=50 --send_rate=100 --verbose=0 --prompt=0
  killall_cleanup
}

# 
# Run sick_scan_xd_api_test.py with sick_multiscan.launch 
# 

printf "\033c"
pushd ../..
export LD_LIBRARY_PATH=.:./build_linux:$LD_LIBRARY_PATH
export PYTHONPATH=.:./python/api:$PYTHONPATH

# Start multiscan emulator (sopas test server)
python3 ./test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 &
sleep 1

# Start sick_scan_xd api example
python3 ./test/python/sick_scan_xd_api/sick_scan_xd_api_test.py ./launch/sick_multiscan.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 scandataformat:=1 &
sleep 3

# Play pcapng-files to emulate multiScan output
multiscan_pcap_player

popd

killall_cleanup
