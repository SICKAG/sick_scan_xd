#!/bin/bash

# killall and cleanup after exit
function killall_cleanup()
{
  killall sick_generic_caller
  pkill -f mrs100_sopas_test_server.py
}

# 
# Run sick_generic_caller with sick_scansegment_xd.launch (Test MRS100 on linux native with test server)
# 

pushd ../..
printf "\033c"
killall_cleanup

# Display scandata
pushd ./demo
rm -f ./scan.jpg ./scan.csv
firefox ./image_viewer.html &
sleep 1
popd

# Run mrs100 emulator (sopas test server)
python3 ./test/python/mrs100_sopas_test_server.py --tcp_port=2111 --cola_binary=0 &
sleep 1

# Start sick_generic_caller with sick_scansegment_xd
./build_linux/sick_generic_caller ./launch/sick_scansegment_xd.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 &
sleep 3

# Play pcapng-files to emulate MRS100 output
python3 ./test/python/mrs100_pcap_player.py --pcap_filename=./test/emulator/scandata/20210929_mrs100_token_udp.pcapng --udp_port=2115 --repeat=2
python3 ./test/python/mrs100_pcap_player.py --pcap_filename=./test/emulator/scandata/20210929_mrs100_cola-a-start-stop-scandata-output.pcapng --udp_port=2115
sleep 3

pkill -f ./test/emulator/test_server.py
killall sick_generic_caller
popd

