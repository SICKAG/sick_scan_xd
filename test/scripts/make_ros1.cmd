REM 
REM Build sick_scan_xd on Windows ROS-2 with Visual Studio 2019
REM Note: Visual Studio 2019 required, Visual Studio 2022 not supported by ROS
REM

pushd ..\..\..\..
rmdir /s/q .\log

if 1==1 (
    REM For a complete cleanup and build, remove .\build, .\install and the catkin/colcon generated file .\src\CMakeLists.txt
    rmdir /s/q .\build_isolated
    rmdir /s/q .\devel_isolated
    rmdir /s/q .\install_isolated
    del /f/q .\src\CMakeLists.txt
)

for %%i in ( .\install\sick_scan_xd\lib .\install\sick_scan_xd\lib\sick_scan_xd .\build\sick_scan_xd\Debug .\build\sick_scan_xd\Release ) do (
  if exist %%i\sick_scan_lib.lib       del /f/q %%i\sick_scan_lib.lib
  if exist %%i\sick_generic_caller.exe del /f/q %%i\sick_generic_caller.exe
)

call "%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64
if exist c:\opt\ros\noetic\x64\setup.bat call c:\opt\ros\noetic\x64\setup.bat

catkin_make_isolated --install --ignore-pkg libsick_ldmrs --cmake-args -DROS_VERSION=1 -DLDMRS=0 -DCMAKE_ENABLE_EMULATOR=1
rem catkin_make_isolated --install --ignore-pkg libsick_ldmrs --cmake-args -DROS_VERSION=1 -DCATKIN_ENABLE_TESTING=0

@timeout /t 3
@echo.
if not exist .\devel_isolated\sick_scan_xd\lib\sick_scan_lib.lib                 ( @echo colcon build sick_scan_lib.lib failed       & @pause ) else ( @echo Successfully build sick_scan_lib.lib for ROS-1 Windows )
if not exist .\devel_isolated\sick_scan_xd\lib\sick_scan_xd\sick_generic_caller.exe ( @echo colcon build sick_generic_caller.exe failed & @pause ) else ( @echo Successfully build sick_generic_caller.exe for ROS-1 Windows )

popd
@pause
rem @timeout /t 10
