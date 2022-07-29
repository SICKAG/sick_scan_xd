REM 
REM Run sick_scan on ROS-2 Windows with simple test server
REM 

if exist "c:\dev\ros2_foxy\local_setup.bat" ( call C:\dev\ros2_foxy\local_setup.bat )
if exist "c:\opt\ros\foxy\x64\setup.bat" ( call c:\opt\ros\foxy\x64\setup.bat )
rem set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%

pushd ..\..\..\..
call .\install\setup.bat
rem start "ros2 echo cloud" ros2 topic echo /cloud
start "rviz2" ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_ros2.rviz
@timeout /t 5

REM 
REM Run test server
REM 

copy /b/y .\src\sick_scan_xd\test\emulator\scandata\sopas_et_field_test_1_2_both_010.pcapng.json \tmp\lmd_scandata.pcapng.json 
start "test_server" ros2 run sick_scan sick_scan_emulator ./src/sick_scan_xd/test/emulator/launch/emulator_01_default.launch scanner_type:=sick_tim_7xx
@timeout /t 1

REM 
REM Run sick_scan on ROS-2 Windows
REM 

start "sick_generic_caller" ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False
@timeout /t 10

REM 
REM Call service examples
REM 

rem echo PYTHONPATH = %PYTHONPATH%
ros2 service list
@timeout /t 2

ros2 service type /ColaMsg
@timeout /t 2
ros2 service find sick_scan/srv/ColaMsgSrv
@timeout /t 2

ros2 service call /SCdevicestate sick_scan/srv/SCdevicestateSrv "{}"
@timeout /t 2
ros2 service call /ColaMsg sick_scan/srv/ColaMsgSrv "{request: 'sMN IsSystemReady'}"
@timeout /t 2
ros2 service call /ColaMsg sick_scan/srv/ColaMsgSrv "{request: 'sRN SCdevicestate'}"
@timeout /t 2
ros2 service call /ColaMsg sick_scan/srv/ColaMsgSrv "{request: 'sEN LIDinputstate 1'}"
@timeout /t 2
ros2 service call /ColaMsg sick_scan/srv/ColaMsgSrv "{request: 'sEN LIDoutputstate 1'}"
@timeout /t 2
ros2 service call /ColaMsg sick_scan/srv/ColaMsgSrv "{request: 'sMN LMCstartmeas'}" 

@pause
popd
