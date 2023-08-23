REM Emulator version vom 06.12.2021:
REM Start subsysApp.Win64.Release.exe only
REM Login as Authorized Client
REM For playback: load file 30_Datenemulator\navlayer_prototype.sdr.msgpack
REM pushd ..\..\..\..\..\30_LieferantenDokumente\30_Datenemulator\20211206_multiScan_Emulator
REM start subsysApp.Win64.Release.exe
REM @timeout /t 3
REM start http:\\127.0.0.1:80
REM popd

REM Emulator version vom 21.09.2022 (multiscan_0.10.2.18pre.rar):
REM Unpack multiscan_0.10.2.18pre.rar
REM Start subsysBase and then subsyApp
REM Login as Developer
REM For playback: load file 30_Datenemulator\realData_20220817_155932.sdr.msgpack

pushd ..\..\..\..\..\30_LieferantenDokumente\30_Datenemulator\20220921_multiscan_0.10.2.18pre
rem pushd ..\..\..\..\..\30_LieferantenDokumente\30_Datenemulator\20230531_multiScan-Emulator-1.2.2-2
cd subsysBase
start "subsysBase" subsysBase.Release.exe
@timeout /t 3
cd ..\subsysApp
start "subsysApp" subsysApp.Release.exe
@timeout /t 3
start http:\\127.0.0.1:80
popd

@echo.
@echo Login as Developer
@echo Start playback: load file 30_Datenemulator\realData_20220817_155932.sdr.msgpack
@echo.
@timeout /t 10
rem @pause
