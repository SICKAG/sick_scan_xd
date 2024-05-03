#!/bin/bash

# killall and cleanup after exit
function killall_cleanup()
{
  killall sick_generic_caller
  pkill -f multiscan_sopas_test_server.py
  pkill -f multiscan_pcap_player.py
}

# 
# Run sick_generic_caller with sick_multiscan.launch (Test multiScan on linux native with test server)
# 

pushd ../..
printf "\033c"
killall_cleanup

# Run multiScan emulator (sopas test server)
python3 ./test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 &
sleep 1

# Start sick_generic_caller with sick_scansegment_xd
./build_linux/sick_generic_caller ./launch/sick_multiscan.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 scandataformat:=1 &
sleep 3

# Play pcapng-files to emulate multiScan output
echo -e "\nPlaying pcapng-files to emulate multiScan. Note: Start of UDP msgpacks in 20220915_mrs100_msgpack_output.pcapng takes a while...\n"
python3 ./test/python/multiscan_pcap_player.py --pcap_filename=./test/emulator/scandata/20220915_mrs100_msgpack_output.pcapng --udp_port=2115
python3 ./test/python/multiscan_pcap_player.py --pcap_filename=./test/emulator/scandata/20210929_mrs100_token_udp.pcapng --udp_port=2115 --repeat=2
python3 ./test/python/multiscan_pcap_player.py --pcap_filename=./test/emulator/scandata/20210929_mrs100_cola-a-start-stop-scandata-output.pcapng --udp_port=2115
sleep 3

killall_cleanup
popd

