# Getting started

Run the following steps for a quick start:

1. Create a workspace (e.g. folder `sick_scan_ws`), clone the sick_scan_xd repository and build sick_generic_caller and shared library:
 
   * For **Linux without ROS**: Follow the [build instructions for Linux generic without ROS](INSTALL-GENERIC.md#build-on-linux-generic-without-ros)

   * For **Linux with ROS-1**: Follow the [build instructions for Linux ROS1](INSTALL-ROS1.md#build-on-linux-ros1)
   
   * For **Linux with ROS-2**: Follow the [build instructions for Linux ROS2](INSTALL-ROS2.md#build-on-linux-ros2)
   
   * For **Windows without ROS**: Follow the [build instructions for Windows without ROS](INSTALL-GENERIC.md#build-on-windows)
   
   * For **Windows with ROS-2**: Follow the [build instructions for Windows with ROS2](INSTALL-ROS2.md#build-on-windows-ros2)

2. Connect your lidar. Check the network connection by `ping <lidar-ip-address>`.

3. Run the sick_scan_xd driver:
 
   * For **Linux without ROS**: Use the sick_scan_xd API and run `sick_scan_xd_api_test <launchfile> hostname:=<lidar-ip-address>`, e.g.:
      ```
      cd ./sick_scan_ws
      export LD_LIBRARY_PATH=.:`pwd`/build:$LD_LIBRARY_PATH  # append absolute path to the build folder
      ./build/sick_scan_xd_api_test ./sick_scan_xd/launch/sick_tim_7xx.launch hostname:=192.168.0.1
      ```

   * For **Linux with ROS-1**: Launch sick_scan_xd: `roslaunch sick_scan_xd <launchfile> hostname:=<lidar-ip-address>`, e.g.:
      ```
      cd ./sick_scan_ws
      source ./devel_isolated/setup.bash
      roslaunch sick_scan_xd sick_tim_7xx.launch hostname:=192.168.0.1
      ```
   
   * For **Linux with ROS-2**: Run `ros2 launch sick_scan_xd <launchfile> hostname:=<lidar-ip-address>`, e.g.:
      ```
      cd ./sick_scan_ws
      source ./install/setup.bash
      ros2 launch sick_scan_xd sick_tim_7xx.launch.py hostname:=192.168.0.1
      ```

   * For **Windows without ROS**: Use the sick_scan_xd API and run `sick_scan_xd_api_test <launchfile> hostname:=<lidar-ip-address>`, e.g.:
      ```
      cd .\sick_scan_ws\sick_scan_xd
      set PATH=.;.\build;..\build\Debug;%PATH%
      .\build\Debug\sick_scan_xd_api_test.exe launch/sick_tim_7xx.launch hostname:=192.168.0.1
      ```
   
   * For **Windows with ROS-2**: Run `ros2 launch sick_scan_xd <launchfile> hostname:=<lidar-ip-address>`, e.g.:
      ```
      cd .\sick_scan_ws
      call .\install\setup.bat
      ros2 launch sick_scan_xd sick_tim_7xx.launch.py hostname:=192.168.0.1
      ```

   See [Run sick_scan_xd driver](USAGE.md) and [sick_scan_xd API](doc/sick_scan_api/sick_scan_api.md) for configuration and further details.
