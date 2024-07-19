REM 
REM Run sick_scan_xd on ROS-2 Windows with lms5xx emulator
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
start "rviz2" ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_lms5xx.rviz
@timeout /t 5

REM 
REM Run lms5xx emulator
REM 

python --version
start "python ../test/emulator/test_server.py" .\src\sick_scan_xd\test\emulator\test_server.cmd  ./src/sick_scan_xd/test/emulator/test_server.py --scandata_file=./src/sick_scan_xd/test/emulator/scandata/20210302_lms511.pcapng.scandata.txt --scandata_frequency=20.0 --tcp_port=2112
@timeout /t 1

REM 
REM Run sick_scan_xd on ROS-2 Windows
REM 

REM tim7xx, x=0, y=0, z=0, roll=0, pitch=0, yaw=0 deg
rem start "static_transform_publisher" ros2 run tf2_ros static_transform_publisher 0 0 0 0.0000000 0.0000000 0.0000000 cloud origin
rem ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False add_transform_xyz_rpy:=0,0,0,0,0,0
REM tim7xx, x=0, y=0, z=0, roll=0, pitch=0, yaw=45 deg
rem start "static_transform_publisher" ros2 run tf2_ros static_transform_publisher 0 0 0 0.7853982 0.0000000 0.0000000 cloud origin
rem ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False add_transform_xyz_rpy:=0,0,0,0.0000000,0.0000000,0.7853982
REM tim7xx, x=0, y=0, z=0, roll=0, pitch=45, yaw=0 deg
rem start "static_transform_publisher" ros2 run tf2_ros static_transform_publisher 0 0 0 0.0000000 0.7853982 0.0000000 cloud origin
rem ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False add_transform_xyz_rpy:=0,0,0,0.0000000,0.7853982,0.0000000
REM tim7xx, x=0, y=0, z=0, roll=45, pitch=0, yaw=0 deg
rem start "static_transform_publisher" ros2 run tf2_ros static_transform_publisher 0 0 0 0.0000000 0.0000000 0.7853982 cloud origin
rem ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False add_transform_xyz_rpy:=0,0,0,0.7853982,0.0000000,0.0000000
REM tim7xx, x=0, y=0, z=0, roll=45, pitch=45, yaw=45 deg
start "static_transform_publisher" ros2 run tf2_ros static_transform_publisher 0 0 0 0.7853982 0.7853982 0.7853982 cloud origin
ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False add_transform_xyz_rpy:=0,0,0,0.7853982,0.7853982,0.7853982

@pause
popd

