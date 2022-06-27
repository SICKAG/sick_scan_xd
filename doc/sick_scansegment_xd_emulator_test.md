# MultiScan136 Test Emulator

On Windows with ROS-2, sick_lidar_3d can be tested against the SICK emulator. Run the following steps:

1. Checkout msgpack11 and sick_lidar3d:
	```
	git clone https://github.com/SICKAG/msgpack11.git
	git clone https://github.com/SICKAG/sick_lidar3d.git
	```

2. Build sick_lidar_3d on ROS2-Windows:
	```
	REM Set environment
	call "%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64
	call c:\dev\ros2_foxy\local_setup.bat
	set PATH=%ProgramFiles%\CMake\bin;%ProgramFiles%\OpenSSL-Win64\bin;%PATH%
	set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%
	set Boost_DIR=\boost_1_73_0
	set Boost_ROOT=\boost_1_73_0
	set Boost_INCLUDE_DIR=\boost_1_73_0
	set Boost_LIBRARY_DIR=\boost_1_73_0\lib64-msvc-14.2
	REM Build msgpack11 and sick_lidar3d on Windows with colcon for ROS2
	colcon build --packages-select msgpack11 --cmake-args " -DMSGPACK11_BUILD_TESTS=0" --event-handlers console_direct+
	colcon build --packages-select sick_lidar3d --cmake-args " -DROS_VERSION=2" " -DUSE_PYTHON=1" " -DPYTHON_DIR=%PYTHON_DIR%" --event-handlers console_direct+
	call .\install\setup.bat
	```
	Note, that the path `\boost_1_73_0` to boost needs to be modified in case of a different boost installation or version.

3. Start the MultiScan136 emulator (version 06.12.2021):
	```
	pushd <path_to>\30_Datenemulator\20211206_MRS100_Emulator
	start subsysApp.Win64.Release.exe
	popd
	```

4. Start rviz2:
	```
	call .\install\setup.bat
	start "rviz2" rviz2
	```

5. Run lidar3d_mrs100_recv:
	```
	call .\install\setup.bat
	start "lidar3d_mrs100_recv" ros2 run sick_lidar3d lidar3d_mrs100_recv --ros-args --params-file sick_lidar3d/config/sick_lidar3d.yaml -p "mrs100_ip:=127.0.0.1" -p "mrs100_dst_ip:=127.0.0.1"
	```

6. Optionally, run some sopas calls, e.g.:
	```
	ros2 service list
	ros2 service call /ColaMsg sick_lidar3d/srv/ColaMsgSrv "{request: 'sRN FREchoFilter'}"
	ros2 service call /ColaMsg sick_lidar3d/srv/ColaMsgSrv "{request: 'sRN LFPangleRangeFilter'}"
	ros2 service call /ColaMsg sick_lidar3d/srv/ColaMsgSrv "{request: 'sRN LFPlayerFilter'}"
	ros2 service call /ColaMsg sick_lidar3d/srv/ColaMsgSrv "{request: 'sMN SetAccessMode 3 F4724744'}"
	ros2 service call /ColaMsg sick_lidar3d/srv/ColaMsgSrv "{request: 'sWN FREchoFilter 0'}"
	ros2 service call /ColaMsg sick_lidar3d/srv/ColaMsgSrv "{request: 'sWN LFPangleRangeFilter 0 C0490FDB 4047F1E6 BFC90FDB 3FC90FDB 1'}"
	ros2 service call /ColaMsg sick_lidar3d/srv/ColaMsgSrv "{request: 'sWN LFPlayerFilter 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1'}"
	ros2 service call /ColaMsg sick_lidar3d/srv/ColaMsgSrv "{request: 'sMN Run'}"
	```
