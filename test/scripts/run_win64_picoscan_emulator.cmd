REM TiMTwo Emulator vom 10.10.2022:

pushd ..\..\..\..\..\30_LieferantenDokumente\30_emulator_timtwo\emulator
cd housekeeper
start "housekeeper" housekeeper.Release.exe
@timeout /t 3
cd ..\application
start "application" application.Release.exe
@timeout /t 3
start http:\\127.0.0.1:80
popd

@echo.
@echo Login: Developer
@echo Start playback: load file 30_LieferantenDokumente\30_emulator_timtwo\timtwo_20220930_160720.sdr.msgpack
@echo.
@timeout /t 10
rem @pause
