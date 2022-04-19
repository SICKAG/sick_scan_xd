REM 
REM Run sick_scan on ROS-1 Windows with lms5xx emulator
REM 

if exist c:\opt\ros\noetic\x64\setup.bat call c:\opt\ros\noetic\x64\setup.bat

pushd ..\..\..\..
call .\install_isolated\setup.bat

start "roscore" roscore
@timeout /t 5

start "rviz" rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_lms5xx.rviz
@timeout /t 5

REM 
REM Run lms5xx emulator
REM 

python --version
start "python ../test/emulator/test_server.py" .\src\sick_scan_xd\test\emulator\test_server.cmd  ./src/sick_scan_xd/test/emulator/test_server.py --scandata_file=./src/sick_scan_xd/test/emulator/scandata/20210302_lms511.pcapng.scandata.txt --scandata_frequency=20.0 --tcp_port=2112
@timeout /t 1

REM 
REM Run sick_scan on ROS-1 Windows
REM 

rem TODO:
rem rosrun sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False
rem roslaunch sick_scan ./install_isolated/share/sick_scan/launch/sick_lms_5xx.launch hostname:=127.0.0.1

@pause
popd

