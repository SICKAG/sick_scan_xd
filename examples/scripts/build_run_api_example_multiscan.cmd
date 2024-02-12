REM 
REM Build and run minimalistic api usages examples on Windows with multiscan player
REM 

REM 
REM Build the minimalistic C and C++ usage example
REM 

call build_api_examples_windows.cmd

REM 
REM Set environment: add build folder to LD_LIBRARY_PATH, add python/api to PYTHONPATH
REM 

pushd ..\..
set PATH=.;.\build;.\build\Debug;.\build_win64;.\build_win64\Debug;%PATH%
set PYTHONPATH=.;.\python\api;%PATH%
python --version

REM 
REM Start sopas test server
REM 

start "python multiscan_sopas_test_server.py" cmd /k python ./test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0
@timeout /t 2

REM 
REM Start pcapng player
REM debug option: --send_rate=10
REM 

rem start "python multiscan_pcap_player.py" cmd /k python ./test/python/multiscan_pcap_player.py --pcap_filename=./test/emulator/scandata/20231009-multiscan-compact-imu-01.pcapng --udp_port=-1 --repeat=999 --filter=pcap_filter_multiscan_hildesheim
pushd .\examples\scripts
start "python multiscan_pcap_player.py" cmd /k run_multiscan_player.cmd
popd
@timeout /t 2

REM 
REM Run minimalistic C++ api example
REM 

@echo Run minimalistic C++ api example with multiscan player ...
.\examples\cpp\build\Debug\minimum_sick_scan_api_client.exe ./launch/sick_multiscan.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1
@timeout /t 2

popd
@pause
