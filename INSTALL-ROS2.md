## Build on Linux ROS2

To build resp. install sick_scan_xd on Linux with ROS-2, you can build sick_scan_xd from sources or install prebuilt binaries.

## Install prebuilt binaries

Run the following steps to install sick_scan_xd on Linux with ROS 2 humble:

```
sudo apt update
sudo apt-get install ros-humble-sick-scan-xd
```

After successful installation, you can run sick_scan_xd using `ros2 launch sick_scan_xd <launchfile>.py`, e.g. `ros2 launch sick_scan_xd sick_multiscan.launch.py` for multiScan. sick_scan_xd can be removed by `sudo apt-get remove ros-humble-sick-scan-xd`.

## Build from sources

Run the following steps to build sick_scan_xd on Linux with ROS 2:

1. Create a workspace folder, e.g. `sick_scan_ws` (or any other name):
   ```
   mkdir -p ./sick_scan_ws
   cd ./sick_scan_ws
   ```

2. Clone repositories https://github.com/SICKAG/libsick_ldmrs and https://github.com/SICKAG/sick_scan_xd:
   ```
   mkdir ./src
   pushd ./src
   git clone https://github.com/SICKAG/libsick_ldmrs.git
   git clone -b master https://github.com/SICKAG/sick_scan_xd.git
   popd
   rm -rf ./build ./build_isolated/ ./devel ./devel_isolated/ ./install ./install_isolated/ ./log/ # remove any files from a previous build
   ```

3. Build sick_generic_caller:
   ```
   source /opt/ros/foxy/setup.bash # replace foxy by your ros distro
   colcon build --packages-select libsick_ldmrs --event-handlers console_direct+
   source ./install/setup.bash
   colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" --event-handlers console_direct+
   source ./install/setup.bash
   ```
   For ROS versions other than foxy, please replace `source /opt/ros/foxy/setup.bash` with your ros distribution.

Note: LDMRS sensors are currently not supported on Raspberry. Build with cmake flag `-DLDMRS=0 -DRASPBERRY=1` on Raspberry:
   ```
   colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" " -DLDMRS=0" " -DRASPBERRY=0" --event-handlers console_direct+
   ```

Note: libsick_ldmrs is only required to support LDMRS sensors. If you do not need or want to support LDMRS, you can skip building libsick_ldmrs. To build sick_generic_caller without LDMRS support, switch off option `BUILD_WITH_LDMRS_SUPPORT` in [CMakeLists.txt](./CMakeLists.txt) or call colcon with option `-DLDMRS=0`:
   ```
   colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" " -DLDMRS=0" --event-handlers console_direct+
   ```
Note: To build sick_generic_caller without multiScan136/sick_scansegment_xd/picoScan150 support, switch off option `BUILD_WITH_SCANSEGMENT_XD_SUPPORT` in [CMakeLists.txt](./CMakeLists.txt) or call cmake with option `-DSCANSEGMENT_XD=0`:
   ```
   colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" " -DSCANSEGMENT_XD=0" --event-handlers console_direct+
   ```

cmake flags can be combined. Use flags `-DLDMRS=0 -DSCANSEGMENT_XD=0` to build **without LDMRS** and **without multiScan100/picoScan100 support**:
   ```
   colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" " -DLDMRS=0" " -DSCANSEGMENT_XD=0" --event-handlers console_direct+
   ```

Note: Depending on the ROS-2 distribution, package diagnostic_updater might not be found (compiler error: `diagnostic_updater.hpp not found`). In this case package diagnostic_updater has to be installed by
```
sudo apt install ros-${ROS_DISTRO}-diagnostic-updater
sudo apt install ros-${ROS_DISTRO}-diagnostic-msgs
# E.g. to install diagnostic_updater on foxy, run
# sudo apt-get install ros-foxy-diagnostic-updater
# sudo apt install ros-foxy-diagnostic-msgs
```

### Summary for the different build options:

* **Without LDMRS-support** and **without multiScan100/picoScan100 support**

```
colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" " -DLDMRS=0" " -DSCANSEGMENT_XD=0" --event-handlers console_direct+
```
* **Without LDMRS-support** and **with multiScan100/picoScan100 support**
```
colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" " -DLDMRS=0" --event-handlers console_direct+
```
* **with LDMRS-support** and **without multiScan100/picoScan100 support**
```
colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" " -DSCANSEGMENT_XD=0" --event-handlers console_direct+
```
* **with LDMRS-support** and **with multiScan100/picoScan100 support**
```
 colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" " --event-handlers console_direct+
```


Note: To create source code documentation by doxygen, run
```
cd ./doxygen
doxygen ./docs/Doxyfile
```

## Build on Windows ROS2

To install sick_scan_xd on Windows with ROS-2, follow the steps below:

1. If not yet done, install Visual Studio. Visual Studio 2019 Community or Professional Edition is recommended.

2. Create a workspace folder, e.g. `sick_scan_ws` (or any other name):
   ```
   mkdir sick_scan_ws
   cd sick_scan_ws
   ```

3. Clone repository https://github.com/SICKAG/sick_scan_xd:
   ```
   mkdir .\src
   pushd .\src
   git clone -b master https://github.com/SICKAG/sick_scan_xd.git
   popd
   ```

4. Set the ROS-2 and Visual-Studio environment:
   ```
   call "%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64
   call C:\opt\ros\foxy\x64\setup.bat
   ```
   Note: This step depends on your local ROS-2 and Visual-Studio installation. Please replace `C:\opt\ros\foxy\x64\setup.bat` with your ROS-2 version and adapt the path to the Visual Studio folder if your installation is different.

5. Cleanup to insure a complete rebuild:
   ```
   rmdir /s/q .\build
   rmdir /s/q .\install
   rmdir /s/q .\log
   del /f/q .\src\CMakeLists.txt
   ```
   Note: This step is only required for a complete rebuild. A complete rebuild is recommended e.g. after an update of the sick_scan_xd sources.

6. Build sick_generic_caller:
   ```
   colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" --event-handlers console_direct+
   call .\install\setup.bat
   ```

Note: LDMRS sensors are currently not supported on Windows.

Note: To build sick_generic_caller without multiScan136/sick_scansegment_xd support, switch off option `BUILD_WITH_SCANSEGMENT_XD_SUPPORT` in [CMakeLists.txt](./CMakeLists.txt) or call cmake with option `-DSCANSEGMENT_XD=0`:
   ```
   colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" " -DSCANSEGMENT_XD=0" --event-handlers console_direct+
   ```
