REM 
REM Build sick_scan_xd API library (sick_scan_shared_lib.dll) on native Windows with Visual Studio 2019 and cmake
REM

call .\make_win64_vs2019.cmd

REM 
REM Build sick_scan_xd on Windows ROS-2 with Visual Studio 2019
REM Note: Visual Studio 2019 required, Visual Studio 2022 not supported by ROS
REM

pushd ..\..\..\..
rmdir /s/q .\log

if 1==1 (
    REM For a complete cleanup and build, remove .\build, .\install and the catkin/colcon generated file .\src\CMakeLists.txt
    rmdir /s/q .\build
    rmdir /s/q .\install
    rmdir /s/q .\log
    del /f/q .\src\CMakeLists.txt
)

for %%i in ( .\install\sick_scan\lib .\install\sick_scan\lib\sick_scan .\build\sick_scan\Debug .\build\sick_scan\Release ) do (
  if exist %%i\sick_scan_lib.lib       del /f/q %%i\sick_scan_lib.lib
  if exist %%i\sick_generic_caller.exe del /f/q %%i\sick_generic_caller.exe
)

call "%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64
if exist c:\dev\ros2_foxy\local_setup.bat call c:\dev\ros2_foxy\local_setup.bat
if exist c:\opt\ros\foxy\x64\setup.bat call c:\opt\ros\foxy\x64\setup.bat
rem set PATH=%ProgramFiles%\CMake\bin;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64;%PATH%
rem set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%

REM 
REM Build msgpack11 and sick_scan_xd on Windows with colcon for ROS2
REM 

colcon build --packages-select msgpack11 --cmake-args " -DMSGPACK11_BUILD_TESTS=0" --event-handlers console_direct+ 
colcon build --packages-select sick_scan --cmake-args " -DROS_VERSION=2" " -DCMAKE_ENABLE_EMULATOR=1" --event-handlers "console_direct+"
call .\install\setup.bat
start "sick_scan.sln" .\build\sick_scan\sick_scan.sln

@timeout /t 3
@echo.
if not exist .\build\sick_scan\Release\sick_scan_lib.lib                 ( @echo colcon build sick_scan_lib.lib failed         & @pause ) else ( @echo Successfully build sick_scan_lib.lib for ROS-2 Windows )
if not exist .\build\sick_scan\Release\sick_generic_caller.exe           ( @echo colcon build sick_generic_caller.exe failed   & @pause ) else ( @echo Successfully build sick_generic_caller.exe for ROS-2 Windows )
if not exist .\build\sick_scan\Release\sick_scan_xd_api_test.exe         ( @echo colcon build sick_scan_xd_api_test.exe failed & @pause ) else ( @echo Successfully build sick_scan_xd_api_test.exe for ROS-2 Windows )
if not exist .\install\sick_scan\lib\sick_scan\sick_generic_caller.exe   ( @echo colcon build sick_generic_caller.exe failed   & @pause ) else ( @echo Successfully build sick_generic_caller.exe for ROS-2 Windows )
if not exist .\install\sick_scan\lib\sick_scan\sick_scan_xd_api_test.exe ( @echo colcon build sick_scan_xd_api_test.exe failed & @pause ) else ( @echo Successfully build sick_scan_xd_api_test.exe for ROS-2 Windows )

popd
@pause
rem @timeout /t 10
