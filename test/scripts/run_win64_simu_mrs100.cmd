REM 
REM Run a basic sick_generic_caller unittest on Windows 64 (standalone, no ROS required) with a test server emulating a basic MRS100 device
REM 

rem set PATH=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64;%PATH%
rem set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%

REM 
REM Start html view
REM 

pushd ..\..\demo
del /f/q scan.jpg scan.csv
start "ImageViewer" image_viewer.html
@timeout /t 1
popd

REM 
REM Start sopas test server
REM 

pushd ..\..\build_win64
python --version
start "python mrs100_sopas_test_server.py" cmd /k python ../test/python/mrs100_sopas_test_server.py --tcp_port=2111 --cola_binary=0
@timeout /t 3

REM 
REM Start sick_generic_caller
REM 

start "sick_generic_caller" cmd /k .\Debug\sick_generic_caller.exe ../launch/sick_scansegment_xd.launch hostname:=127.0.0.1 udp_receiver_ip:=127.0.0.1
@timeout /t 3

REM 
REM Run pcapng player
REM 

python ../test/python/mrs100_pcap_player.py --pcap_filename=../test/emulator/scandata/20210929_mrs100_token_udp.pcapng --udp_port=2115
python ../test/python/mrs100_pcap_player.py --pcap_filename=../test/emulator/scandata/20210929_mrs100_cola-a-start-stop-scandata-output.pcapng --udp_port=2115
@timeout /t 3

popd
@pause
