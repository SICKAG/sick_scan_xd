#!/bin/bash

# 
# Build and run minimalistic api usages examples on Linux
# Examples use sick_scan_emulator.
# Make sure that libsick_scan_xd_shared_lib.so has been build (see README.md for build instructions).
# 
printf "\033c"

# Start tim7xx emulator
function start_tim7xx_emulator()
{
  cp -f ./test/emulator/scandata/sopas_et_field_test_1_2_both_010.pcapng.json /tmp/lmd_scandata.pcapng.json
  sick_scan_emulator ./test/emulator/launch/emulator_01_default.launch &
  sleep 1
}

# Build the minimalistic C usage example
pushd ../c
mkdir -p ./build
cd ./build
cmake -G "Unix Makefiles" ..
make -j4
ls -al ./minimum_sick_scan_api_client
if [ -f ./minimum_sick_scan_api_client ] ; then 
  echo -e "\nbuild C minimum_sick_scan_api_client successful\n"
else
  echo -e "## ERROR build C minimum_sick_scan_api_client failed ..." ; read -n1 -s key
fi
popd

# Build the minimalistic C++ usage example
pushd ../cpp
mkdir -p ./build
cd ./build
cmake -G "Unix Makefiles" ..
make -j4
ls -al ./minimum_sick_scan_api_client
if [ -f ./minimum_sick_scan_api_client ] ; then 
  echo -e "\nbuild C++ minimum_sick_scan_api_client successful\n"
else
  echo -e "## ERROR build C++ minimum_sick_scan_api_client failed ..." ; read -n1 -s key
fi
popd

# Set environment: add build folder to LD_LIBRARY_PATH, add python/api to PYTHONPATH
pushd ../..
if [ -d ./build ] ; then export PATH=.:./build:$PATH ; fi
if [ -d ./build ] ; then export LD_LIBRARY_PATH=.:./build:$LD_LIBRARY_PATH ; fi
if [ -d ./build_linux ] ; then export PATH=.:./build_linux:$PATH ; fi
if [ -d ./build_linux ] ; then export LD_LIBRARY_PATH=.:./build_linux:$LD_LIBRARY_PATH ; fi
export PYTHONPATH=.:./python/api:$PYTHONPATH

# Run minimalistic python api example
start_tim7xx_emulator
echo -e "\nRun minimalistic python api example ...\n"
python3 ./examples/python/minimum_sick_scan_api_client.py ./launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False
killall sick_scan_emulator

# Run minimalistic C api example
start_tim7xx_emulator
echo -e "\nRun minimalistic C api example ...\n"
./examples/c/build/minimum_sick_scan_api_client ./launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False
killall sick_scan_emulator

# Run minimalistic C++ api example
start_tim7xx_emulator
echo -e "\nRun minimalistic C++ api example ...\n"
./examples/cpp/build/minimum_sick_scan_api_client ./launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False
killall sick_scan_emulator

# Cleanup
popd
rm -rf ../c/build
rm -rf ../cpp/build

