REM 
REM Run sick_scan on ROS-2 Windows with simple test server
REM 

if exist "c:\dev\ros2_foxy\local_setup.bat" ( call C:\dev\ros2_foxy\local_setup.bat )
if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64" set PYTHON_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64
if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64" set PYTHON_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64
set PATH=%PYTHON_DIR%;%PYTHON_DIR%\DLLs;%PYTHON_DIR%\Lib;%PYTHON_DIR%\Scripts;%PATH%
set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%

pushd ..\..\..\..
call .\install\setup.bat
start "ros2 echo cloud" ros2 topic echo /cloud
rem start "rviz2" rviz2

REM 
REM Run test server
REM 

start "test_server" ros2 run sick_scan test_server --ros-args --params-file ./src/sick_scan_xd/tools/test_server/config/test_server_cola.yaml
@timeout /t 1

REM 
REM Run sick_scan on ROS-2 Windows
REM 

ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_240.launch hostname:=127.0.0.1 port:=2112 sw_pll_only_publish:=false

@pause
popd
