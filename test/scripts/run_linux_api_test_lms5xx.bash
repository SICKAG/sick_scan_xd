#!/bin/bash

export RUN_SIMU=1

# Run lms 511 emulator in a loop
function run_emulator_lms511()
{
  for n in {1..9} ; do
    python3 ./test/emulator/test_server.py --scandata_file=./test/emulator/scandata/20210302_lms511.pcapng.scandata.txt --scandata_frequency=20.0 --tcp_port=2112
    if [ $RUN_SIMU -eq 0 ] ; then
      break
    fi
  done
}

# Kills test_server and sick_scan_xd_api_test processes
function kill_simu()
{
  pkill -f ./test/emulator/test_server.py
  killall sick_scan_xd_api_test
}

# 
# Run sick_scan_xd_api_test with sick_lms_5xx.launch
# 

kill_simu
printf "\033c"
pushd ../..
export LD_LIBRARY_PATH=.:./build_linux:$LD_LIBRARY_PATH

# Start emulator
# python3 ./test/emulator/test_server.py --scandata_file=./test/emulator/scandata/20210302_lms511.pcapng.scandata.txt --scandata_frequency=20.0 --tcp_port=2112 &
run_emulator_lms511 &
sleep 1

# Start image viewer (simple standalone pointcloud visualization)
firefox ./demo/image_viewer_api_test.html &
sleep 1

# Run sick_scan_xd api example
./build_linux/sick_scan_xd_api_test ./launch/sick_lms_5xx.launch hostname:=127.0.0.1 port:=2112 sw_pll_only_publish:=False
popd

# Finish
RUN_SIMU=0
sleep 1
for n in {1..9} ; do
  kill_simu
  sleep 0.1
done

