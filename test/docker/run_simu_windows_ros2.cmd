REM
REM Run simulation and tests on Windows ROS2
REM 

if exist .\log\sick_scan_xd_simu ( rmdir /s/q .\log\sick_scan_xd_simu )
if exist .\log_sick_scan_xd_simu_win64_ros2 ( rmdir /s/q .\log_sick_scan_xd_simu_win64_ros2 )
call .\install\setup.bat
if _%ROS_DISTRO%_==__ (
    @echo ERROR in run_simu_windows_ros2.cmd: ROS_DISTRO not set
    @pause
    @exit
)
@echo run_simu_windows_ros2.cmd: ROS_VERSION = %ROS_VERSION%, ROS_DISTRO = %ROS_DISTRO%
python --version
set dockertest_exit_status=0

for %%c in ( multiscan_compact_test01_cfg.json picoscan_compact_test01_cfg.json lms1xx_test01_cfg.json lms1xxx_test01_cfg.json lms5xx_test01_cfg.json mrs6xxx_test01_cfg.json nav350_test01_cfg.json rmsxxxx_test01_cfg.json tim240_test01_cfg.json tim7xx_test01_cfg.json tim7xxs_test01_cfg.json lms4xxx_test01_cfg.json lrs36x0_test01_cfg.json lrs36x1_test01_cfg.json lrs4xxx_test01_cfg.json oem15xx_test01_cfg.json tim4xx_test01_cfg.json tim5xx_test01_cfg.json ) do ( 
    
    @echo.
    @echo python ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=%ROS_DISTRO% --api=none --cfg=./src/sick_scan_xd/test/docker/data/%%c
    @echo.
    @title sick_scan_xd_simu.py --ros=%ROS_DISTRO% --api=none --cfg=%%c
    
    python ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=%ROS_DISTRO% --api=none --cfg=./src/sick_scan_xd/test/docker/data/%%c
    if NOT %ERRORLEVEL%==0 ( set dockertest_exit_status=%ERRORLEVEL% )
    @timeout /t 5
    rem @pause
)

REM
REM Create summary and convert md-files into html
REM 

move /y .\log\sick_scan_xd_simu .\log_sick_scan_xd_simu_win64_ros2
pushd .\log_sick_scan_xd_simu_win64_ros2
call ..\src\sick_scan_xd\test\docker\utils\create_html_reports_win64.cmd
popd
