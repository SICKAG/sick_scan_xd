REM 
REM Build sick_scan_xd on Windows native (no ROS) with Visual Studio 2022 and cmake
REM

pushd ..\..
set _os=x64
set _cmake_string=Visual Studio 17 2022
set _msvc=Visual Studio 2022
set _cmake_build_dir=build_win64

if not exist %_cmake_build_dir% mkdir %_cmake_build_dir%
pushd %_cmake_build_dir%
cmake -DROS_VERSION=0-DCMAKE_ENABLE_EMULATOR=1 -G "%_cmake_string%" -DCMAKE_TOOLCHAIN_FILE=c:/vcpkg/scripts/buildsystems/vcpkg.cmake ..
if %ERRORLEVEL% neq 0 ( @echo ERROR building %_cmake_string% sick_scan_xd with cmake & @pause )
start "sick_scan.sln" sick_scan.sln
popd

popd
@timeout /t 10
