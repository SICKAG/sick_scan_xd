REM 
REM Build sick_scan_xd on Windows native (no ROS) with Visual Studio 2019 and cmake
REM

pushd ..\..

rem call "%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64
rem if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64" set PYTHON_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64
rem if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64" set PYTHON_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64
rem set PATH=c:\opencv\x64\vc16\bin;%ProgramFiles%\CMake\bin;%ProgramFiles%\OpenSSL-Win64\bin;%ProgramFiles(x86)%\Graphviz2.38\bin;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64;%PATH%
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

set _os=x64
set _cmake_string=Visual Studio 16
set _msvc=Visual Studio 2019
set _cmake_build_dir=build_win64

REM 
REM Build sick_scan_xd on Windows native (no ROS) with Visual Studio 2019 and cmake
REM 

if not exist %_cmake_build_dir% mkdir %_cmake_build_dir%
pushd %_cmake_build_dir%
cmake -DROS_VERSION=0-DCMAKE_ENABLE_EMULATOR=1 -G "%_cmake_string%" -DCMAKE_TOOLCHAIN_FILE=c:/vcpkg/scripts/buildsystems/vcpkg.cmake ..
if %ERRORLEVEL% neq 0 ( @echo ERROR building %_cmake_string% sick_scan_xd with cmake & @pause )
start "sick_scan.sln" sick_scan.sln
popd
popd
@timeout /t 10
