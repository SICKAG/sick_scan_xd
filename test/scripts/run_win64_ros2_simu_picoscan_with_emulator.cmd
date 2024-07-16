REM 
REM Run sick_scan_xd on ROS-2 Windows with picoscan emulator
REM 

rem if exist "c:\dev\ros2_foxy\local_setup.bat" ( call C:\dev\ros2_foxy\local_setup.bat )
rem if exist "c:\opt\ros\foxy\x64\setup.bat" ( call c:\opt\ros\foxy\x64\setup.bat )
if exist "c:\opt\ros\humble\x64\setup.bat" ( call c:\opt\ros\humble\x64\setup.bat )
rem set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%

pushd ..\..\..\..
call .\install\setup.bat
rem start "ros2 echo cloud" ros2 topic echo /cloud
start "rviz2 picoScan" ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_picoscan_windows.rviz
start "rviz2 picoScan 360" ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_picoscan_windows_360.rviz
@timeout /t 3

REM 
REM Start picoscan emulator
REM 

rem call run_win64_picoscan_emulator.cmd

REM 
REM Start sick_scan_xd on ROS-2 Windows
REM 

rem ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_picoscan.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 publish_frame_id:=world add_transform_xyz_rpy:=0,0,0,0,0,0
ros2 launch sick_scan_xd sick_picoscan.launch.py hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 scandataformat:=1 all_segments_min_deg:=-134 all_segments_max_deg:=135

popd
@pause

