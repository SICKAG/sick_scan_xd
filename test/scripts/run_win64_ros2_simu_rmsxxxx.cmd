REM 
REM Run sick_scan_xd on ROS-2 Windows with rms xxxx emulator
REM 

rem if exist "c:\dev\ros2_foxy\local_setup.bat" ( call C:\dev\ros2_foxy\local_setup.bat )
rem if exist "c:\opt\ros\foxy\x64\setup.bat" ( call c:\opt\ros\foxy\x64\setup.bat )
if exist "c:\opt\ros\humble\x64\setup.bat" ( call c:\opt\ros\humble\x64\setup.bat )
rem if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64" set PYTHON_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64
rem if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64" set PYTHON_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64
rem set PATH=%PYTHON_DIR%;%PYTHON_DIR%\DLLs;%PYTHON_DIR%\Lib;%PYTHON_DIR%\Scripts;%PATH%
rem set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%

pushd ..\..\..\..
call .\install\setup.bat
set PATH=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python39_64;%PATH%
python --version
where python

REM
REM Run rms1, rms2 emulator and launch sick_scan_xd sick_rms_xxxx.launch (ascii)
REM

for %%f in ( 20220316-rms1000-ascii.pcapng.json 20221018_rms_1xxx_ascii_rms2_objects.pcapng.json ) do (

    start "rviz2 rmsxxxx" /min ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_cfg_rmsxxxx_windows.rviz
    @echo Running ros2 launch sick_scan_xd sick_rms_xxxx.launch.py with emulator, scandata file ./src/sick_scan_xd/test/emulator/scandata/%%f
    start "python sopas_json_test_server.py" /min cmd /c python ./src/sick_scan_xd/test/python/sopas_json_test_server.py --tcp_port=2112 --json_file=./src/sick_scan_xd/test/emulator/scandata/%%f --scandata_id="sSN LMDradardata" --send_rate=10 --verbosity=1
    start "python ros2 launch sick_rms_xxxx.launch.py" /min cmd /c ros2 launch sick_scan_xd sick_rms_xxxx.launch.py hostname:=127.0.0.1 sw_pll_only_publish:=False
    @timeout /t 15
    taskkill /im sick_generic_caller.exe /t /f
    taskkill /im python.exe /t /f
    taskkill /im rviz2.exe /t /f
)

@pause
popd

