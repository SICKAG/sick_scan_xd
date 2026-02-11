REM
REM Build sick_scan_Xd with RelWithDebInfo
REM under Windows to allow Debugging
cd ..
cd ..
cd ..
@echo off
echo Cleanup temp. directories
rm -rf build install log     
echo BUILD RelWithDebInfo version for Windows Debugging
REM colcon build --packages-select diagnostic_updater sick_scan_xd --cmake-args "-DROS_VERSION=2" " -DLDMRS=0" "-DCMAKE_CXX_FLAGS=/bigobj" "-DCMAKE_BUILD_TYPE=RelWithDebInfo" --event-handlers console_direct+
colcon build --packages-select diagnostic_updater sick_scan_xd --cmake-args "-DROS_VERSION=2" " -DLDMRS=0" "-DCMAKE_BUILD_TYPE=RelWithDebInfo" --event-handlers console_direct+
REM Afterwards run devenv in the Pixi environment und open SLN folder in the RelWithDebInfo directory
REM ros2 launch sick_scan_xd sick_picoscan.launch.py hostname:=192.168.0.1 udp_receiver_ip:=192.168.0.100
REM for debugging set C:\pixi_ws\sick_scan_xd_ws\install\sick_scan_xd\share\sick_scan_xd\launch\sick_picoscan.launch hostname:=192.168.0.1 udp_receiver_ip:=192.168.0.100 --ros-args 
REM as command line argument.
REM Working directory:
REM C:\pixi_ws\sick_scan_xd_ws