REM 
REM Build sick_scan_xd for ROS2 on Windows with Visual Studio 2019
REM

set VS_DEV_CMD_PATH=VsDevCmd.bat
if exist "C:\BuildTools\Common7\Tools\VsDevCmd.bat" ( set VS_DEV_CMD_PATH="C:\BuildTools\Common7\Tools\VsDevCmd.bat" )
if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\2019\BuildTools\Common7\Tools\VsDevCmd.bat" ( set VS_DEV_CMD_PATH="%ProgramFiles(x86)%\Microsoft Visual Studio\2019\BuildTools\Common7\Tools\VsDevCmd.bat" )
if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" ( set VS_DEV_CMD_PATH="%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" )
call %VS_DEV_CMD_PATH% -arch=amd64 -host_arch=amd64

set ROS2_SETUP_BAT=c:\opt\ros\foxy\x64\setup.bat
if exist c:\dev\ros2_foxy\local_setup.bat   ( set ROS2_SETUP_BAT=c:\dev\ros2_foxy\local_setup.bat )
if exist c:\dev\ros2_humble\local_setup.bat ( set ROS2_SETUP_BAT=c:\dev\ros2_humble\local_setup.bat )
if exist c:\opt\ros\foxy\x64\setup.bat      ( set ROS2_SETUP_BAT=c:\opt\ros\foxy\x64\setup.bat )
if exist c:\opt\ros\humble\x64\setup.bat    ( set ROS2_SETUP_BAT=c:\opt\ros\humble\x64\setup.bat )
call %ROS2_SETUP_BAT%
rem set PATH=%ProgramFiles%\CMake\bin;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64;%PATH%
rem set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%
if exist C:\opt\ros\foxy\x64\python3.exe ( if not exist C:\opt\ros\foxy\x64\python3_d.exe ( copy /b/y C:\opt\ros\foxy\x64\python3.exe C:\opt\ros\foxy\x64\python3_d.exe ) )
if exist C:\opt\ros\humble\x64\python3.exe ( if not exist C:\opt\ros\humble\x64\python3_d.exe ( copy /b/y C:\opt\ros\humble\x64\python3.exe C:\opt\ros\humble\x64\python3_d.exe ) )

REM 
REM Build sick_scan_xd for ROS2 on Windows
REM 

colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" " -DCMAKE_ENABLE_DOCKERTESTS=1" --event-handlers "console_direct+"
call .\install\setup.bat

@echo.
if not exist .\build\sick_scan_xd\Release\sick_scan_xd_lib.lib ( @echo colcon build sick_scan_xd_lib.lib failed ) else ( @echo Successfully build sick_scan_xd_lib.lib for ROS-2 Windows )
if not exist .\build\sick_scan_xd\Release\sick_generic_caller.exe ( @echo colcon build sick_generic_caller.exe failed ) else ( @echo Successfully build sick_generic_caller.exe for ROS-2 Windows )
if not exist .\install\sick_scan_xd\lib\sick_scan_xd\sick_generic_caller.exe ( @echo colcon build sick_generic_caller.exe failed ) else ( @echo Successfully build sick_generic_caller.exe for ROS-2 Windows )
