REM 
REM Run a basic sick_scan_xd api test on Windows 64 (standalone, no ROS required) with a test server emulating a multiScan device
REM See run_win64_simu_multiscan_msgpack.cmd and https://github.com/SICKAG/sick_scan_xd/blob/master/doc/sick_scan_segment_xd.md
REM for details about multiScan emulation.
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

start "image_viewer" .\demo\image_viewer_api_test.html

REM 
REM Run sick_scan_xd_api_test (cpp)
REM 

start "sick_scan_xd_api_test" cmd /k .\build_win64\Debug\sick_scan_xd_api_test.exe ./launch/sick_multiscan.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1
@timeout /t 3

REM 
REM Run pcapng player
REM 

@echo Playing pcapng-files to emulate multiScan...
python ./test/python/multiscan_perftest_player.py --dst_ip=127.0.0.1 --udp_port=2115 --repeat=100 --send_rate=100 --verbose=0 --prompt=0

popd
@pause
