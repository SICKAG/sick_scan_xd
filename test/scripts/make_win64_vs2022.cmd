REM 
REM Build sick_scan_xd on Windows native (no ROS) with Visual Studio 2022 and cmake
REM Note: ROS requires Visual Studio 2019, use make_win64_vs2019.cmd instead of make_win64_vs2022.cmd for release versions.
REM

pushd ..\..
set _os=x64
set _cmake_string=Visual Studio 17 2022
set _msvc=Visual Studio 2022
set _cmake_build_dir=build_win64

REM 
REM Build msgpack11 on Windows
REM 

call "%ProgramFiles(x86)%\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64
if not exist %_cmake_build_dir%\msgpack11 mkdir %_cmake_build_dir%\msgpack11
pushd %_cmake_build_dir%\msgpack11
cmake -DMSGPACK11_BUILD_TESTS=0 -G "%_cmake_string%" ../../../msgpack11
if %ERRORLEVEL% neq 0 ( @echo ERROR building %_cmake_string% msgpack11 with cmake & @pause )
rem start "msgpack11.sln" msgpack11.sln
devenv msgpack11.sln /clean     "Debug|x64"
devenv msgpack11.sln /rebuild   "Debug|x64"
devenv msgpack11.sln /clean     "Release|x64"
devenv msgpack11.sln /rebuild   "Release|x64"
popd

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
