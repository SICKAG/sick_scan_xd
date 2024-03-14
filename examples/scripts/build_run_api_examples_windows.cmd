REM 
REM Build and run minimalistic api usages examples on Windows
REM Examples use sick_scan_emulator.
REM 

REM Build the minimalistic C and C++ usage example
call build_api_examples_windows.cmd

REM Set environment: add build folder to LD_LIBRARY_PATH, add python/api to PYTHONPATH
pushd ..\..
set PATH=.;.\build;.\build\Debug;.\build_win64;.\build_win64\Debug;%PATH%
set PYTHONPATH=.;.\python\api;%PATH%

REM Use python 3.9.13 installation from https://www.python.org/ftp/python/3.9.13/python-3.9.13-amd64.exe
if exist \Python39 set PATH=\Python39;%PATH%
where python
python --version

REM Run minimalistic python api example
start "test_server.py" cmd /k python ./test/emulator/test_server.py --scandata_file=./test/emulator/scandata/20210302_lms511.pcapng.scandata.txt --scandata_frequency=20.0 --tcp_port=2112
@timeout /t 2
@echo Run minimalistic python api example ...
python ./examples/python/minimum_sick_scan_api_client.py ./launch/sick_lms_5xx.launch hostname:=127.0.0.1

REM Run minimalistic C api example
start "test_server.py" cmd /k python ./test/emulator/test_server.py --scandata_file=./test/emulator/scandata/20210302_lms511.pcapng.scandata.txt --scandata_frequency=20.0 --tcp_port=2112
@timeout /t 2
@echo Run minimalistic C api example ...
.\examples\c\build\Debug\minimum_sick_scan_api_client.exe ./launch/sick_lms_5xx.launch hostname:=127.0.0.1

REM Run minimalistic C++ api example
start "test_server.py" cmd /k python ./test/emulator/test_server.py --scandata_file=./test/emulator/scandata/20210302_lms511.pcapng.scandata.txt --scandata_frequency=20.0 --tcp_port=2112
@timeout /t 2
@echo Run minimalistic C++ api example ...
.\examples\cpp\build\Debug\minimum_sick_scan_api_client.exe ./launch/sick_lms_5xx.launch hostname:=127.0.0.1

REM Cleanup
popd
rmdir /s/q ..\c\build
rmdir /s/q ..\cpp\build
@pause
