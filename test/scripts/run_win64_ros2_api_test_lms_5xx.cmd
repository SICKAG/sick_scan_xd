REM 
REM Build and run minimalistic api usage examples (Python, C, C++)
REM 

rem pushd ..\..\examples\scripts
rem call .\build_run_api_examples_windows.cmd
rem popd

REM 
REM Run sick_scan_xd API test on ROS-2 Windows with lms5xx emulator
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
start "rviz2" ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_api_test_lms5xx.rviz
@timeout /t 5

REM 
REM Run lms5xx emulator
REM 

python --version
start "python ../test/emulator/test_server.py" .\src\sick_scan_xd\test\emulator\test_server.cmd  ./src/sick_scan_xd/test/emulator/test_server.py --scandata_file=./src/sick_scan_xd/test/emulator/scandata/20210302_lms511.pcapng.scandata.txt --scandata_frequency=20.0 --tcp_port=2112
@timeout /t 1

REM 
REM Run sick_scan_xd API test on ROS-2 Windows
REM 

set PYTHONPATH=.;.\src\sick_scan_xd\python\api;%PYTHONPATH%

rem ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False
rem set PATH=%PATH%;.\src\sick_scan_xd\build_win64\Debug;.\src\sick_scan_xd\build\Debug
rem ros2 run sick_scan sick_scan_xd_api_test ./src/sick_scan_xd/launch/sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False
python ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py ./src/sick_scan_xd/launch/sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False

@pause
popd

