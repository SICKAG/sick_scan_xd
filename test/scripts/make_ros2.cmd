REM 
REM Build sick_scan_xd on Windows ROS-2
REM

pushd ..\..\..\..
rmdir /s/q .\log
for %%i in ( .\install\sick_scan\lib .\install\sick_scan\lib\sick_scan .\build\sick_scan\Debug .\build\sick_scan\Release ) do (
  if exist %%i\sick_scan_lib.lib       del /f/q %%i\sick_scan_lib.lib
  if exist %%i\sick_generic_caller.exe del /f/q %%i\sick_generic_caller.exe
)

call "%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64
call c:\dev\ros2_foxy\local_setup.bat
set PATH=%ProgramFiles%\CMake\bin;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64;%PATH%
set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%

REM Boost with prebuild binaries:
REM if exist \boost_1_73_0 (
REM   set Boost_DIR=\boost_1_73_0
REM   set Boost_ROOT=\boost_1_73_0
REM   set Boost_INCLUDE_DIR=\boost_1_73_0
REM   set Boost_LIBRARY_DIR=\boost_1_73_0\lib64-msvc-14.2
REM )
REM @echo Boost_DIR=%Boost_DIR%
REM @echo Boost_INCLUDE_DIR=%Boost_INCLUDE_DIR%
REM @echo Boost_LIBRARY_DIR=%Boost_LIBRARY_DIR%

REM Boost support by vcpkg (recommended):
REM 1. Install vcpkg:
REM    Download vcpkg-master.zip from https://github.com/microsoft/vcpkg/archive/master.zip and unzip to c:/vcpkg. Alternatively, run "git clone https://github.com/microsoft/vcpkg"
REM    Install by running the following commands:
REM    cd c:/vcpkg
REM    .\bootstrap-vcpkg.bat
REM    .\vcpkg integrate install
REM 2. Install required packages:
REM    vcpkg.exe install pthread:x86-windows
REM    vcpkg.exe install pthread:x64-windows
REM    vcpkg.exe install boost:x64-windows
REM 3. Include vcpkg in your path:
REM    set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%

REM 
REM Build sick_scan_xd on Windows with colcon for ROS2
REM 

copy /b/y .\src\sick_scan_xd\package_ros2.xml .\src\sick_scan_xd\package.xml
rem colcon build --packages-select sick_scan --cmake-args " -DROS_VERSION=2" --event-handlers console_direct+
colcon build --cmake-args " -DROS_VERSION=2" --event-handlers console_direct+
call .\install\setup.bat
start "sick_scan.sln" .\build\sick_scan\sick_scan.sln

@timeout /t 3
@echo.
if not exist .\build\sick_scan\Release\sick_scan_lib.lib       ( @echo colcon build sick_scan_lib.lib failed       & @pause ) else ( @echo Successfully build sick_scan_lib.lib for ROS-2 Windows )
if not exist .\build\sick_scan\Release\sick_generic_caller.exe ( @echo colcon build sick_generic_caller.exe failed & @pause ) else ( @echo Successfully build sick_generic_caller.exe for ROS-2 Windows )

popd
@pause
rem @timeout /t 10
