REM Emulator version vom 06.12.2021:
REM Start subsysApp.Win64.Release.exe only
REM Login as Authorized Client, pw client
REM For playback: load file 30_Datenemulator\navlayer_prototype.sdr.msgpack

pushd ..\..\..\..\..\30_LieferantenDokumente\30_Datenemulator\20211206_MRS100_Emulator
start subsysApp.Win64.Release.exe
@timeout /t 3
start http:\\127.0.0.1:80
popd

@echo.
@echo Login as Authorized Client, pw client
@echo Start playback: load file 30_Datenemulator\navlayer_prototype.sdr.msgpack
@echo.
@timeout /t 10
rem @pause
