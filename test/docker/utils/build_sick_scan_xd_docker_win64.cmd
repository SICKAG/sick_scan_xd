REM 
REM Build sick_scan_xd in a windows docker image with Visual Studio 2019 and cmake
REM

pushd .\src\sick_scan_xd
if exist "C:\BuildTools\Common7\Tools\VsDevCmd.bat" ( call "C:\BuildTools\Common7\Tools\VsDevCmd.bat"
) else (
  if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" ( call "%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat"
  ) else (
    @echo ERROR: File Common7\Tools\VsDevCmd.bat not found, Visual Studio 2019 must be installed
    exit
  )
)
set _cmake_string=Visual Studio 16
set _cmake_build_dir=build

if not exist %_cmake_build_dir% mkdir %_cmake_build_dir%
pushd %_cmake_build_dir%
cmake -DROS_VERSION=0 -DCMAKE_ENABLE_DOCKERTESTS=1 -G "%_cmake_string%" ..
if %ERRORLEVEL% neq 0 ( @echo ERROR building %_cmake_string% sick_scan_xd with cmake )
cmake --build . --clean-first --config Debug
if %ERRORLEVEL% neq 0 ( @echo ERROR building %_cmake_string% sick_scan_xd debug with cmake )
@echo.
if exist .\Debug\sick_scan_xd_shared_lib.dll     ( @echo sick_scan_xd_shared_lib.dll debug successfully built ) else ( @echo ERROR building sick_scan_xd_shared_lib.dll debug )
if exist .\Debug\sick_scan_xd_api_test.exe       ( @echo sick_scan_xd_api_test debug successfully built       ) else ( @echo ERROR building sick_scan_xd_api_test debug )
if exist .\Debug\sick_scan_xd_api_dockertest.exe ( @echo sick_scan_xd_api_dockertest debug successfully built ) else ( @echo ERROR building sick_scan_xd_api_dockertest debug )
if exist .\Debug\sick_generic_caller.exe         ( @echo sick_generic_caller debug successfully built         ) else ( @echo ERROR building sick_generic_caller debug )
popd
popd
