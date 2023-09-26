REM 
REM Run a basic sick_generic_caller unittest on Windows 64 (standalone, no ROS required) with a test server emulating a basic multiScan device
REM 

rem set PATH=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64;%PATH%
rem set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%

REM 
REM Convert pcapng-files to msgpack and json
REM pip install msgpack
REM 
REM pushd ..\python
REM python --version
REM REM 
REM REM Convert 20220915_multiscan_msgpack_output.pcapng to msgpack/json (16-bit RSSI record)
REM REM 
REM del /f/q multiscan_dump*.msgpack
REM del /f/q multiscan_dump*.msgpack.hex
REM start python multiscan_receiver.py
REM python multiscan_pcap_player.py --pcap_filename=../emulator/scandata/20220915_multiscan_msgpack_output.pcapng --udp_port=2115 --verbose=1
REM move /y .\multiscan_dump_23644.msgpack     20220915_multiscan_msgpack_output.msgpack
REM move /y .\multiscan_dump_23644.msgpack.hex 20220915_multiscan_msgpack_output.msgpack.hex 
REM REM 
REM REM Convert 20210929_multiscan_token_udp.pcapng to msgpack/json (8-bit RSSI record)
REM REM 
REM del /f/q multiscan_dump*.msgpack
REM del /f/q multiscan_dump*.msgpack.hex
REM start python multiscan_receiver.py
REM python multiscan_pcap_player.py --pcap_filename=../emulator/scandata/20210929_multiscan_token_udp.pcapng --verbose=0
REM move /y .\multiscan_dump_12472.msgpack     20210929_multiscan_token_udp.msgpack
REM move /y .\multiscan_dump_12472.msgpack.hex 20210929_multiscan_token_udp.msgpack.hex 
REM del /f/q multiscan_dump*.msgpack
REM del /f/q multiscan_dump*.msgpack.hex
REM popd

REM 
REM Start sopas test server
REM 

pushd ..\..\build_win64
python --version
start "python multiscan_sopas_test_server.py" cmd /k python ../test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0
@timeout /t 3

REM 
REM Start sick_generic_caller
REM 

start "sick_generic_caller" cmd /k .\Debug\sick_generic_caller.exe ../launch/sick_multiscan.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 scandataformat:=1
@timeout /t 3

REM 
REM Run pcapng player:
REM 

@echo Playing pcapng-files to emulate multiScan...
python ../test/python/multiscan_perftest_player.py --dst_ip=127.0.0.1 --udp_port=2115 --repeat=100 --send_rate=100 --verbose=0 --prompt=0

REM @echo Playing pcapng-files to emulate multiScan. Note: Start of UDP msgpacks in 20220915_mrs100_msgpack_output.pcapng takes a while...
REM python ../test/python/multiscan_pcap_player.py --pcap_filename=../test/emulator/scandata/20220915_mrs100_msgpack_output.pcapng --udp_port=2115
REM For profiling and performance tests:
REM python ../test/python/multiscan_perftest_player.py --dst_ip=127.0.0.1 --udp_port=2115 --repeat=100 --send_rate=0 --force_delay=3.0e-3 --verbose=0 --prompt=0
REM For performance tests on the Raspberry (192.168.1.27):
REM python ../test/python/multiscan_perftest_player.py --dst_ip=192.168.1.27 --udp_port=2115 --repeat=100 --send_rate=0 --force_delay=3.0e-3 --verbose=0 --prompt=0
@timeout /t 3

popd
@pause
