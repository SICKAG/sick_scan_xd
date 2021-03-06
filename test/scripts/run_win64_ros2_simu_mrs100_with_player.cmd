REM 
REM Run sick_scan on ROS-2 Windows with mrs100 pcapng player
REM 

if exist "c:\dev\ros2_foxy\local_setup.bat" ( call C:\dev\ros2_foxy\local_setup.bat )
if exist "c:\opt\ros\foxy\x64\setup.bat" ( call c:\opt\ros\foxy\x64\setup.bat )
rem if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64" set PYTHON_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64
rem if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64" set PYTHON_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64
rem set PATH=%PYTHON_DIR%;%PYTHON_DIR%\DLLs;%PYTHON_DIR%\Lib;%PYTHON_DIR%\Scripts;%PATH%
rem set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%

pushd ..\..\..\..
call .\install\setup.bat
rem start "ros2 echo cloud" ros2 topic echo /cloud
start "rviz2 mrs100" ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_mrs100_windows.rviz
start "rviz2 mrs100 360" ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_mrs100_windows_360.rviz
@timeout /t 3

REM 
REM Start sopas test server
REM 

set PATH=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python39_64;%PATH%
python --version
where python
start "python mrs100_sopas_test_server.py" cmd /k python ./src/sick_scan_xd/test/python/mrs100_sopas_test_server.py --tcp_port=2111 --cola_binary=0
@timeout /t 3

REM 
REM Start sick_scan on ROS-2 Windows
REM 

start "ros2 sick_generic_caller" ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_scansegment_xd.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1
@timeout /t 3

REM 
REM Run pcapng player
REM 

rem set PATH=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python39_64;%PATH%
python ./src/sick_scan_xd/test/python/mrs100_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20210929_mrs100_token_udp.pcapng --udp_port=2115
python ./src/sick_scan_xd/test/python/mrs100_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20210929_mrs100_cola-a-start-stop-scandata-output.pcapng --udp_port=2115
@timeout /t 3
popd

@pause

