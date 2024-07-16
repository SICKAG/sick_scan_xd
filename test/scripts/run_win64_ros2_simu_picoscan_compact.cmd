REM 
REM Run sick_scan_xd on ROS-2 Windows with picoScan pcapng player
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
start "rviz2 picoScan" ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_picoscan_windows.rviz
start "rviz2 picoScan 360" ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_picoscan_windows_360.rviz
@timeout /t 3

REM 
REM Start sopas test server
REM 

set PATH=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python39_64;%PATH%
python --version
where python
start "python multiscan_sopas_test_server.py" cmd /k python ./src/sick_scan_xd/test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0 --FREchoFilter=1
@timeout /t 3

REM 
REM Start sick_scan_xd on ROS-2 Windows
REM 

start "ros2 sick_scan_xd" ros2 launch sick_scan_xd sick_picoscan.launch.py hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 scandataformat:=2 all_segments_min_deg:=-134 all_segments_max_deg:=135
@timeout /t 3

REM 
REM Run pcapng player with picoscan compact-data
REM 

rem set PATH=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python39_64;%PATH%
@echo.
@echo Playing pcapng-files to emulate picoscan150 with compact data
@echo.
python ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230204_timtwo_v0.18.0_compact_profile1.pcapng --udp_port=2115 --repeat=1
python ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230204_timtwo_v0.18.0_compact_profile2.pcapng --udp_port=2115 --repeat=1
python ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile1_echofilter_off_fogfilter_off.pcapng --udp_port=2115 --repeat=1
python ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile2_echofilter_off_fogfilter_off.pcapng --udp_port=2115 --repeat=1
python ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile1_echofilter_lastecho_fogfilter_off.pcapng --udp_port=2115 --repeat=1
python ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile2_echofilter_lastecho_fogfilter_off.pcapng --udp_port=2115 --repeat=1
python ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230509_picoscan_compact_profile1_echofilter_lastecho_fogfilter_1.pcapng --udp_port=2115 --repeat=1
@timeout /t 3
popd

@pause

