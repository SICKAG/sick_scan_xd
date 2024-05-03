#!/bin/bash

# killall and cleanup after exit
function killall_cleanup()
{
  killall sick_scan_xd_api_test
  pkill -f multiscan_sopas_test_server.py
  pkill -f multiscan_pcap_player.py
}

# Play pcapng-files to emulate multiScan output
function multiscan_pcap_player()
{
  sleep 3
  python3 ./test/python/multiscan_perftest_player.py --udp_port=2115 --repeat=100 --send_rate=100 --verbose=0 --prompt=0
  # echo -e "\nPlaying pcapng-files to emulate multiScan. Note: Start of UDP msgpacks in 20220915_mrs100_msgpack_output.pcapng takes a while...\n"
  # python3 ./test/python/multiscan_pcap_player.py --pcap_filename=./test/emulator/scandata/20220915_mrs100_msgpack_output.pcapng --udp_port=2115
  # python3 ./test/python/multiscan_pcap_player.py --pcap_filename=./test/emulator/scandata/20210929_mrs100_token_udp.pcapng --udp_port=2115 --repeat=2
  # python3 ./test/python/multiscan_pcap_player.py --pcap_filename=./test/emulator/scandata/20210929_mrs100_cola-a-start-stop-scandata-output.pcapng --udp_port=2115
  killall_cleanup
}

# 
# Run sick_scan_xd_api_test with sick_multiscan.launch
# 

printf "\033c"
pushd ../..
export LD_LIBRARY_PATH=.:./build_linux:$LD_LIBRARY_PATH

# Start multiscan emulator (sopas test server)
python3 ./test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 &
sleep 1

# Start image viewer (simple standalone pointcloud visualization)
rm -f /tmp/sick_scan_api_demo.jpg
firefox ./demo/image_viewer_api_test.html &
sleep 1

# Play pcapng-files to emulate multiScan output
multiscan_pcap_player &

# Run sick_scan_xd api example with sick_multiscan.launch
# gdb --args ./build_linux/sick_scan_xd_api_test ./launch/sick_multiscan.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 scandataformat:=1
./build_linux/sick_scan_xd_api_test ./launch/sick_multiscan.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 scandataformat:=1

# Finish
killall_cleanup
popd

