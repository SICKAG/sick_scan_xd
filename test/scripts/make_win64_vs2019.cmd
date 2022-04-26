REM 
REM Build sick_scan_xd on Windows native (no ROS) with Visual Studio 2019 and cmake
REM

pushd ..\..

call "%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64
rem if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64" set PYTHON_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64
rem if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64" set PYTHON_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64
rem set PATH=c:\opencv\x64\vc16\bin;%ProgramFiles%\CMake\bin;%ProgramFiles%\OpenSSL-Win64\bin;%ProgramFiles(x86)%\Graphviz2.38\bin;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64;%PATH%
set PATH=%ProgramFiles%\CMake\bin;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64;%PATH%
set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%

set _os=x64
set _cmake_string=Visual Studio 16
set _msvc=Visual Studio 2019
set _cmake_build_dir=build_win64

REM 
REM Build msgpack11 on Windows
REM 

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
cmake -DROS_VERSION=0 -DCMAKE_ENABLE_EMULATOR=1 -G "%_cmake_string%" -DCMAKE_TOOLCHAIN_FILE=c:/vcpkg/scripts/buildsystems/vcpkg.cmake ..
if %ERRORLEVEL% neq 0 ( @echo ERROR building %_cmake_string% sick_scan_xd with cmake & @pause )
devenv sick_scan.sln /clean     "Debug|x64"
devenv sick_scan.sln /rebuild   "Debug|x64"
rem devenv sick_scan.sln /clean     "Release|x64"
rem devenv sick_scan.sln /rebuild   "Release|x64"
start "sick_scan.sln" sick_scan.sln
if exist .\Debug\sick_generic_caller.exe ( @echo sick_generic_caller debug successfully built ) else ( @echo ERROR building sick_generic_caller debug & @pause )
rem if exist .\Release\sick_generic_caller.exe ( @echo sick_generic_caller release successfully built ) else ( @echo ERROR building sick_generic_caller release & @pause )
popd
popd
@timeout /t 10
