REM 
REM Run a basic sick_generic_caller unittest on Windows 64 (standalone, no ROS required)
REM 

rem set PATH=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64;%PATH%
rem set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%

REM 
REM Start multiScan emulator 20230531_multiScan-Emulator-1.2.2-2
REM 
REM 
REM pushd ..\..\..\..\..\30_LieferantenDokumente\30_Datenemulator\20230531_multiScan-Emulator-1.2.2-2
REM cd subsysBase
REM start "subsysBase" subsysBase.Release.exe
REM @timeout /t 3
REM cd ..\subsysApp
REM start "subsysApp" subsysApp.Release.exe
REM @timeout /t 3
REM start http:\\127.0.0.1:80
REM popd
REM 
REM @echo.
REM @echo Login as Developer ?
REM @echo Start playback: load file 30_Datenemulator\realData_20220817_155932.sdr.msgpack ?
REM @echo.
REM @timeout /t 10
REM @pause

REM 
REM Start sopas test server
REM 

pushd ..\..\build_win64
python --version
start "python multiscan_sopas_test_server.py" cmd /k python ../test/python/multiscan_sopas_test_server.py --tcp_port=2111 --cola_binary=0
@timeout /t 3

REM 
REM Start sick_generic_caller with multiScan compact data format
REM 

start "sick_generic_caller" cmd /k .\Debug\sick_generic_caller.exe ../launch/sick_multiscan.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1 scandataformat:=2
@timeout /t 3

REM 
REM Run pcapng player:
REM 

@echo.
@echo Playing pcapng-files to emulate multiScan
@echo.
python ../test/python/multiscan_pcap_player.py --pcap_filename=../test/emulator/scandata/20230607-multiscan-compact-v4-5layer.pcapng --udp_port=2115 --send_rate=10
@timeout /t 3

popd
@pause
