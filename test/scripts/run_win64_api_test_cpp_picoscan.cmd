REM 
REM Run a basic sick_scan_xd api test on Windows 64 (standalone, no ROS required) with a test server emulating a picoScan device
REM 

pushd ..\..

REM 
REM Start sopas test server
REM 

python --version
start "python multiscan_sopas_test_server.py" cmd /k python ./test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0
@timeout /t 3

REM
REM Start image viewer (simple standalone pointcloud visualization)
REM

rem start "image_viewer" .\demo\image_viewer_api_test.html

REM 
REM Run sick_scan_xd_api_test (cpp)
REM 

start "sick_scan_xd_api_test" cmd /k .\build_win64\Debug\sick_scan_xd_api_test.exe ./launch/sick_picoscan.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1
@timeout /t 3

REM 
REM Run pcapng player
REM 

@echo Playing pcapng-files to emulate picoScan...
python ./test/python/multiscan_pcap_player.py --pcap_filename=./test/emulator/scandata/20240930_picoscan_120.pcapng --udp_port=2115 --repeat=5 --send_rate=10
python ./test/python/multiscan_pcap_player.py --pcap_filename=./test/emulator/scandata/20230911-picoscan-compact.pcapng --udp_port=2115 --repeat=5 --send_rate=10

popd
@pause
