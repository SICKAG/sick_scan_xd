REM Set environment for Visual Studio 2019 (VS 16)
set _os=x64
set _cmake_string=Visual Studio 16
set _msvc=Visual Studio 2019
set _cmake_build_dir=build

REM Build the minimalistic C usage example
@echo Build minimalistic C api example ...
pushd ..\c
mkdir %_cmake_build_dir%
cd %_cmake_build_dir%
cmake -G "%_cmake_string%" ..
cmake --build . --clean-first --config Debug
if exist .\Debug\minimum_sick_scan_api_client.exe (
  @echo build C minimum_sick_scan_api_client successful
) else (
  @echo ## ERROR build C minimum_sick_scan_api_client failed ...
  @pause
)
popd

REM Build the minimalistic C++ usage example
@echo Build minimalistic C++ api example ...
pushd ..\cpp
mkdir %_cmake_build_dir%
cd %_cmake_build_dir%
cmake -G "%_cmake_string%" ..
cmake --build . --clean-first --config Debug
if exist .\Debug\minimum_sick_scan_api_client.exe (
  @echo build C++ minimum_sick_scan_api_client successful
) else (
  @echo ## ERROR build C++ minimum_sick_scan_api_client failed ...
  @pause
)
popd
