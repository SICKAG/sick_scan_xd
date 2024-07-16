REM 
REM Run sick_scan_xd on ROS-2 Windows with multiScan pcapng player
REM 

rem if exist "c:\dev\ros2_foxy\local_setup.bat" ( call C:\dev\ros2_foxy\local_setup.bat )
rem if exist "c:\opt\ros\foxy\x64\setup.bat" ( call c:\opt\ros\foxy\x64\setup.bat )
if exist "c:\opt\ros\humble\x64\setup.bat" ( call c:\opt\ros\humble\x64\setup.bat )
rem if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64" set PYTHON_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64
rem if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64" set PYTHON_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64
rem set PATH=%PYTHON_DIR%;%PYTHON_DIR%\DLLs;%PYTHON_DIR%\Lib;%PYTHON_DIR%\Scripts;%PATH%
rem set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%

pushd ..\..\..\..
call .\install\setup.bat
rem start "ros2 echo cloud" ros2 topic echo /cloud
start "rviz2 multiscan" ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_multiscan_windows_laserscan.rviz
start "rviz2 multiscan" ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_multiscan_windows.rviz
start "rviz2 multiscan 360" ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_multiscan_windows_360.rviz
@timeout /t 3

REM 
REM Start sopas test server
REM 

set PATH=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python39_64;%PATH%
python --version
where python
start "python multiscan_sopas_test_server.py" cmd /k python ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0
@timeout /t 3

REM Note: To verify laserscan messages, we configure laserscan_layer_filter:="1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1", i.e. a laserscan message is published for each segment, each layer and each echo.
REM By default, laserscan messages are only activated for layer 5 (elevation -0.07 degree, max number of scan points)
REM All laserscan messages are converted to pointcloud by multiscan_laserscan_msg_to_pointcloud.py using a hardcoded elevation table.
REM Note: Option laserscan_layer_filter:="1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1" should not be used for performance tests.
REM ros2 launch sick_scan_xd sick_multiscan.launch.py hostname:=127.0.0.1 udp_receiver_ip:="127.0.0.1" laserscan_layer_filter:="1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"
REM start "python multiscan_laserscan_msg_to_pointcloud.py" cmd /k python ./src/sick_scan_xd/test/python/multiscan_laserscan_msg_to_pointcloud.py
REM @timeout /t 1

REM 
REM Start sick_scan_xd on ROS-2 Windows
REM 

rem start "ros2 sick_generic_caller" ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_multiscan.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 publish_frame_id:=world add_transform_xyz_rpy:=0,0,0,0,0,0
start "ros2 sick_scan_xd" ros2 launch sick_scan_xd sick_multiscan.launch.py hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 scandataformat:=1

@timeout /t 3

REM 
REM Run pcapng player
REM 

rem set PATH=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python39_64;%PATH%
@echo.
@echo Playing pcapng-files to emulate multiScan...
@echo.
python ./src/sick_scan_xd/test/python/multiscan_perftest_player.py --dst_ip=127.0.0.1 --udp_port=2115 --repeat=100 --send_rate=100 --verbose=0 --prompt=0
rem @echo.
rem @echo Playing pcapng-files to emulate multiScan. Note: Start of UDP msgpacks in 20220915_multiscan_msgpack_output.pcapng takes a while...
rem @echo.
rem python ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20220915_mrs100_msgpack_output.pcapng --udp_port=2115
@timeout /t 3
popd

@pause

