# sick_scan_xd

This project provides the driver for the SICK lidar and radar sensors mentioned in the following list.

Based on the sick_scan drivers for ROS1, sick_scan_xd merges sick_scan, sick_scan2 and sick_scan_base repositories. The driver supports both Linux (native, ROS1, ROS2) and Windows (native and ROS2). See the [CHANGELOG.md](CHANGELOG.md) for the latest updates.


## Table of Contents

- [Executive Summary](#executive-summary)
- [Supported Hardware](#supported-hardware)
- [Supported Hardware](#supported-hardware)
- [Build on Linux generic without ROS](#build-on-linux-generic-without-ros)
- [Build on Linux ROS1](#build-on-linux-ros1)
- [Build on Linux ROS2](#build-on-linux-ros2)
- [Build on Windows](#build-on-windows)
- [Build on Windows ROS2](#build-on-windows-ros2)
- [Run sick_scan driver](#run-sick_scan-driver)
   - [Start Multiple Nodes](#start-multiple-nodes)
   - [Common parameters](#common-parameters)
   - [Starting Scanner with Specific Ip Address](#starting-scanner-with-specific-ip-address)
   - [Further useful parameters and features](#further-useful-parameters-and-features)
   - [ROS services](#ros-services)
   - [Driver states, timeouts](#driver-states-timeouts)
- [Sopas Mode](#sopas-mode)
- [Bugs and feature requests](#bugs-and-feature-requests)
- [Tools](#tools)
- [Troubleshooting](#troubleshooting)
- [SLAM-Support](doc/slam.md)
- [IMU-Support](#imu-Support)
- [Radar](doc/radar.md)
- [Profiling](doc/profiling.md)
- [Testing](#testing)
- [Creators](#creators)

## Executive Summary

* sick_scan_xd supports
    * generic use (Linux and Windows native)
    * Linux-ROS1
    * ROS2 (Linux and Windows)
  
  for the devices listed below.

* sick_scan_xd improves quality and consistency
* All features are available on all targets.
* The sick_scan_xd repository supports and maintains generic use, ROS1 and ROS2:
    * sick_scan_xd merges projects  sick_scan, sick_scan2 and sick_scan_base
    * Simplifies the use of devices, as there is only one github entry point for the supported devices
    * Simplifies integration of new devices
    * Avoids inconsistencies between different platforms
    * Simplifies and centralizes integration of new features, improvements, issues and bugfixes
* Identical sources for all targets are more transparent and clearer for developer and user
* sick_scan_xd has no dependencies to 3rd party libraries like boost or pthread

We recommend using sick_scan_xd instead of sick_scan, sick_scan2 or sick_scan_base for all new projects using one or multiple devices listed below. 
In the medium term, it's generally recommended to migrate to sick_scan_xd.

## Supported Hardware

This driver should work with all of the following products.

ROS Device Driver for SICK lidar and radar sensors - supported scanner types:

| **device name**    |  **part no.**   | **description**                                | **tested?**     |
|--------------------|------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------|:---------------:|
| MRS6124            | [6065086](https://www.sick.com/de/en/detection-and-ranging-solutions/3d-lidar-sensors/mrs6000/c/g448151)                         | 24 layer max. range: 200 m, ang. resol. 0.13 [deg] hor., 0.0625 [deg] ver. The SICK MRS6124 is a multi-layer, multi-echo 3D laser scanner that is geared towards rough outdoor environments. | ✔ [stable]|
|                                                                               | Scan-Rate: 10 Hz                       |                 |
| MRS1104            | [1081208](https://www.sick.com/sg/en/detection-and-ranging-solutions/3d-lidar-sensors/mrs1000/mrs1104c-111011/p/p495044)         | 4 layer max. range: 64 m, ang. resol. 0.25 [deg] hor., 2.50 [deg] ver.                                         | ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 50 Hz, 4x12.5 Hz            |                 |https://cdn.sick.com/media/docs/4/04/504/Operating_instructions_RMS3xx_en_IM0075504.PDF
|                    |                                                                                                                                  | Scan-Rate: 150 Hz, 4x37.5 Hz   |                 |
| TiM240             | [1104981](https://www.sick.com/ag/en/detection-and-ranging-solutions/2d-lidar-sensors/tim2xx/tim240-2050300/p/p654443)           | 1 layer max. range: 10 m, ang. resol. 1.00 [deg], 240 [deg]| ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 14.5 Hz   |                 |
| TiM433             | prototype  | 1 layer range: 0.05 m ... 15 m, ang. resol. 0.33 [deg], 240 [deg]| ✔ [prototype]|
|                    |                                                                                                                                  | Scan-Rate: 15.0 Hz   |                 |
| TiM443             | prototype  | 1 layer range: 0.05 m ... 15 m, ang. resol. 0.33 [deg], 240 [deg]| ✔ [prototype]|
|                    |                                                                                                                                  | Scan-Rate: 15.0 Hz   |                 |
| TiM551             | [1060445](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/tim5xx/tim551-2050001/p/p343045)                 | 1 layer max. range: 10 m, ang. resol. 1.00[deg] | ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| TiM561             | [1071419](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/tim5xx/tim561-2050101/p/p369446)                 | 1 layer max. range: 10 m, ang. resol. 0.33 [deg]| ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| TiM571             | [1079742](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/tim5xx/tim571-2050101/p/p412444)                 | 1 layer max. range: 25 m, ang. resol. 0.33 [deg]| ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| TiM771S            | [1105052](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/tim7xx/tim771s-2174104/p/p660929)                  | 1 layer max. range: 25 m, ang. resol. 0.33 [deg]| ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| TiM781             | [1096807](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/tim7xx/tim781-2174101/p/p594148)                   | 1 layer max. range: 25 m, ang. resol. 0.33 [deg]| ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| TiM781S            | [1096363](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/tim7xx/tim781s-2174104/p/p594149)                  | 1 layer max. range: 25 m, ang. resol. 0.33 [deg]| ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| LMS511-10100 PRO   | [e.g. 1046135](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/lms5xx/c/g179651)     | 1 layer max. range: 80 m, ang. resol. 0.167 [deg]| ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 100 Hz   |                 |
| LMS1104            | [1092445](https://www.sick.com/ag/en/detection-and-ranging-solutions/2d-lidar-sensors/lms1000/c/g387151)                         | 1 layer max. range: 64 m, ang. resol. 0.25 [deg] |  ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 150 Hz, 4x37.5 Hz   |
| LMS1xx-Family      | [e.g. 1041114](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/lms1xx/c/g91901) | 1 layer max. range: 28 m, ang. resol. 0.25 [deg]| ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| LMS4xxx-Family     | [e.g. 1091423](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/lms4000/lms4111r-13000/p/p578044?ff_data) | 1 layer max. range: 3 m, ang. resol. 0,0833 [deg], opening angle: +/- 50 [deg] | ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 600 Hz   |                 |
| LDMRS |   | 4 or 8 layer, max. range: 50/320 m, ang. resol. 0.025°/.../0.25 [deg] | ✔ [development]|
|       |   | Scan-Rate: 12.5-50 Hz | |
| LRS4000 |   | 1 layer, max. range: 130 m, ang. resol. 0.125/0.25/0.5 [deg]  | ✔ [development]|
|       |   | Scan-Rate: 12.5-25 Hz | |
| NAV310     | [e.g. 1052928](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/nav3xx/nav350-3232/p/p256041) | 1 layer max. range: 250 m, ang. resol. 0.125 [deg] | ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 8 Hz   |                 |
| NAV210+NAV245      | [e.g.    1074308](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/nav2xx/c/g356151) | 1 layer max. range: 100 m, ang. resol. 0.25 [deg]| ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 25 Hz   |                 |
| RMS3xx             | [8021530](https://cdn.sick.com/media/docs/4/04/504/Operating_instructions_RMS3xx_en_IM0075504.PDF)| Radar Sensor | ✔ [stable]|
| RMS1xxx             | [1107598](https://www.sick.com/de/en/detection-and-ranging-solutions/radar-sensors/rms1000/rms1731c-636111/p/p660833)| 1D Radar Sensor | ✔ [development]|

Note:
* LDMRS family is currently not supported on Windows.
* ROS services require installation of ROS-1 or ROS-2, i.e. services for Cola commands are currently not supported on native Linux or native Windows.
* ROS services are currently not available for LDMRS.
* dynamic reconfiguration of sick_scan parameter is supported on ROS-1 or ROS-2 only, neither under Linux nor under Windows.
* Publishing pointcloud data requires ROS-1 or ROS-2. On native Linux resp. native Windows, pointcloud data are currently saved to jpg- and csv-files for demonstration purposes.

## Build targets

sick_scan_xd can be build on Linux and Windows, with and without ROS, with and without LDMRS. The following table shows the allowed combinations and how to build.

| **target** | **cmake settings** | **build script** |
|------------|--------------------|------------------|
| Linux, native, LDMRS      | BUILD_WITH_LDMRS_SUPPORT ON  | cd test/scripts && makeall_linux.bash |
| Linux, native, no LDMRS   | BUILD_WITH_LDMRS_SUPPORT OFF | cd test/scripts && makeall_linux_no_ldmrs.bash |
| Linux, ROS-1, LDMRS       | BUILD_WITH_LDMRS_SUPPORT ON  | cd test/scripts && makeall_ros1.bash           |
| Linux, ROS-1, no LDMRS    | BUILD_WITH_LDMRS_SUPPORT OFF | cd test/scripts && makeall_ros1_no_ldmrs.bash  |
| Linux, ROS-2, LDMRS       | BUILD_WITH_LDMRS_SUPPORT ON  | cd test/scripts && makeall_ros2.bash           |
| Linux, ROS-2, no LDMRS    | BUILD_WITH_LDMRS_SUPPORT OFF | cd test/scripts && makeall_ros2_no_ldmrs.bash  |
| Windows, native, no LDMRS | BUILD_WITH_LDMRS_SUPPORT OFF | cd test\\scripts && make_win64.cmd             |
| Windows, ROS-2, no LDMRS  | BUILD_WITH_LDMRS_SUPPORT OFF | cd test\\scripts && make_ros2.cmd              |

If you're using ROS, set your ROS-environment before running one of these scripts, f.e.
* `source /opt/ros/noetic/setup.bash` for ROS-1 noetic, or
* `source /opt/ros/melodic/setup.bash` for ROS-1 melodic, or
* `source /opt/ros/eloquent/setup.bash` for ROS-2 eloquent, or
* `source /opt/ros/foxy/setup.bash` for ROS-2 fox.

See the build descriptions below for more details.

## Build on Linux generic without ROS

Run the following steps to build sick_scan_xd on Linux (no ROS required):

1. Clone repositories https://github.com/SICKAG/libsick_ldmrs and https://github.com/SICKAG/sick_scan_xd:
   ```
   git clone https://github.com/SICKAG/libsick_ldmrs.git
   git clone https://github.com/SICKAG/sick_scan_xd.git
   ```

2. Build libsick_ldmrs (only required for LDMRS sensors):
   ```
   pushd libsick_ldmrs
   mkdir -p ./build
   cd ./build
   cmake -G "Unix Makefiles" ..
   make -j4
   sudo make -j4 install    
   popd
   ```

3. Build sick_generic_caller:
   ```
   pushd sick_scan_xd
   mkdir -p ./build_linux
   cd ./build_linux
   export ROS_VERSION=0
   cmake -DROS_VERSION=0 -G "Unix Makefiles" ..
   make -j4
   popd
   ```

Note: libsick_ldmrs is only required to support LDMRS sensors. If you do not need or want to support LDMRS, you can skip building libsick_ldmrs. To build sick_generic_caller without LDMRS support, switch off option `BUILD_WITH_LDMRS_SUPPORT` in [CMakeLists.txt](./CMakeLists.txt) or call cmake with option `-DLDMRS=0`:
   ```
   cmake -DROS_VERSION=0 -DLDMRS=0 -G "Unix Makefiles" ..
   ```

## Build on Linux ROS1

Run the following steps to build sick_scan_xd on Linux with ROS 1:

1. Clone repositories https://github.com/SICKAG/libsick_ldmrs and https://github.com/SICKAG/sick_scan_xd:
   ```
   mkdir ./src
   pushd ./src
   git clone https://github.com/SICKAG/libsick_ldmrs.git # only required for LDMRS sensors
   git clone https://github.com/SICKAG/sick_scan_xd.git
   popd
   ```

2. Build sick_generic_caller:
   ```
   source /opt/ros/melodic/setup.bash
   catkin_make_isolated --install --cmake-args -DROS_VERSION=1
   source ./install_isolated/setup.bash
   ```
   For ROS versions other than melodic, please replace `source /opt/ros/melodic/setup.bash` with your ros distribution.

Note: libsick_ldmrs is only required to support LDMRS sensors. If you do not need or want to support LDMRS, you can skip building libsick_ldmrs. To build sick_generic_caller without LDMRS support, switch off option `BUILD_WITH_LDMRS_SUPPORT` in [CMakeLists.txt](./CMakeLists.txt) or call catkin_make_isolated with option `-DLDMRS=0`:
   ```
   catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DLDMRS=0
   ```

## Build on Linux ROS2

Run the following steps to build sick_scan_xd on Linux with ROS 2:

1. Clone repositories https://github.com/SICKAG/libsick_ldmrs and https://github.com/SICKAG/sick_scan_xd:
   ```
   mkdir ./src
   pushd ./src
   git clone https://github.com/SICKAG/libsick_ldmrs.git # only required for LDMRS sensors
   git clone https://github.com/SICKAG/sick_scan_xd.git
   popd
   ```

2. Build sick_generic_caller:
   ```
   source /opt/ros/eloquent/setup.bash
   colcon build --packages-select libsick_ldmrs --event-handlers console_direct+
   source ./install/setup.bash
   colcon build --packages-select sick_scan --cmake-args " -DROS_VERSION=2" --event-handlers console_direct+
   source ./install/setup.bash
   ```
   For ROS versions other than eloquent, please replace `source /opt/ros/eloquent/setup.bash` with your ros distribution.

Note: libsick_ldmrs is only required to support LDMRS sensors. If you do not need or want to support LDMRS, you can skip building libsick_ldmrs. To build sick_generic_caller without LDMRS support, switch off option `BUILD_WITH_LDMRS_SUPPORT` in [CMakeLists.txt](./CMakeLists.txt) or call colcon with option `-DLDMRS=0`:
   ```
   colcon build --packages-select sick_scan --cmake-args " -DROS_VERSION=2" " -DLDMRS=0" --event-handlers console_direct+
   ```

## Build on Windows

To install sick_scan_xd on Windows, follow the steps below:

1. If not yet done, install Visual Studio. Visual Studio 2019 Community or Professional Edition is recommended.

2. If not yet done, install Visual Studios package manager vcpkg:
   * Install vcpkg and libjson:
      * Download vcpkg-master.zip from https://github.com/microsoft/vcpkg/archive/master.zip and unzip to `c:\vcpkg`. Alternatively, run "git clone https://github.com/microsoft/vcpkg"
      * Install vcpkg by running the following commands:
         ```
        cd c:/vcpkg
        bootstrap-vcpkg.bat
        vcpkg integrate install
        ```
     * Install libjson with vcpkg
       ```
       vcpkg install jsoncpp:x64-windows
       ```
       maybe you need to install the english language package. Follow the instructions at https://agirlamonggeeks.com/2019/03/10/how-to-change-language-in-visual-studio-2019-after-installation/ 
   * Include vcpkg in your path:
      ```
     set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%
     ```

3. Clone repository https://github.com/SICKAG/sick_scan_xd:
   ```
   git clone https://github.com/SICKAG/sick_scan_xd.git
   ```

4. Build sick_generic_caller with cmake and Visual Studio 2019:
   ```
   cd sick_scan_xd
   set _os=x64
   set _cmake_string=Visual Studio 16 2019
   set _msvc=Visual Studio 2019
   set _cmake_build_dir=build
   if not exist %_cmake_build_dir% mkdir %_cmake_build_dir%
   pushd %_cmake_build_dir%
   cmake -DROS_VERSION=0 -G "%_cmake_string%" ..
   popd
   ```
   Open file `build\sick_scan_xd.sln` in Visual Studio and build all targets (shortcut F7).

Note: LDMRS sensors are currently not supported on Windows.

## Build on Windows ROS2

To install sick_scan_xd on Windows with ROS-2, follow the steps below:

1. If not yet done, install Visual Studio 2019 and vcpkg as described in [Build on Windows](#build-on-windows).

2. Clone repository https://github.com/SICKAG/sick_scan_xd:
   ```
   git clone https://github.com/SICKAG/sick_scan_xd.git
   ```

3. Build sick_generic_caller:
   ```
   colcon build --packages-select sick_scan --cmake-args " -DROS_VERSION=2" --event-handlers console_direct+
   call .\install\setup.bat
   ```

## IMU Support

Devices of the MRS6xxx and MRS1xxx series are available with an optionally built-in IMU.
Further information on the implementation and use of the experimental Imu support can be found on the [Imu page](doc/IMU.md).

## Run sick_scan driver

The sick_scan driver can be started on the command line by `sick_generic_caller <launchfile> [hostname:=<ip-address>]`. The start process varies slightly depending on the target OS:

- On native Linux without ROS, call

    ```sick_generic_caller <launchfile>```

- On Linux with ROS-1, call

    ```roslaunch sick_scan <launchfile>```

- On Linux with ROS-2, call

    ```ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/<launchfile>```

- On native Windows without ROS, call

    ```sick_generic_caller <launchfile>```

- On Windows with ROS-2, call

    ```ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/<launchfile>```

Use the following commands to run the sick_scan driver for a specific scanner type:

- For MRS6124:
    * Linux native:   `sick_generic_caller sick_mrs_6xxx.launch`
    * Linux ROS-1:    `roslaunch sick_scan sick_mrs_6xxx.launch`
    * Linux ROS-2:    `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_mrs_6xxx.launch`
    * Windows native: `sick_generic_caller sick_mrs_6xxx.launch`
    * Windows ROS-2:  `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_mrs_6xxx.launch`
- For MRS1104:
    * Linux native:   `sick_generic_caller sick_mrs_1xxx.launch`
    * Linux ROS-1:    `roslaunch sick_scan sick_mrs_1xxx.launch`
    * Linux ROS-2:    `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_mrs_1xxx.launch`
    * Windows native: `sick_generic_caller sick_mrs_1xxx.launch`
    * Windows ROS-2:  `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_mrs_1xxx.launch`
- For LMS1104:
    * Linux native:   `sick_generic_caller sick_lms_1xxx.launch`
    * Linux ROS-1:    `roslaunch sick_scan sick_lms_1xxx.launch`
    * Linux ROS-2:    `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_1xxx.launch`
    * Windows native: `sick_generic_caller sick_lms_1xxx.launch`
    * Windows ROS-2:  `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_1xxx.launch`
- For TiM240-prototype:
    * Linux native:   `sick_generic_caller sick_tim_240.launch`
    * Linux ROS-1:    `roslaunch sick_scan sick_tim_240.launch`
    * Linux ROS-2:    `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_240.launch`
    * Windows native: `sick_generic_caller sick_tim_240.launch`
    * Windows ROS-2:  `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_240.launch`
- For TiM4xx-family:
    * Linux native:   `sick_generic_caller sick_tim_4xx.launch`
    * Linux ROS-1:    `roslaunch sick_scan sick_tim_4xx.launch`
    * Linux ROS-2:    `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_4xx.launch`
    * Windows native: `sick_generic_caller sick_tim_4xx.launch`
    * Windows ROS-2:  `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_4xx.launch`
- For TiM5xx-family:
    * Linux native:   `sick_generic_caller sick_tim_5xx.launch`
    * Linux ROS-1:    `roslaunch sick_scan sick_tim_5xx.launch`
    * Linux ROS-2:    `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_5xx.launch`
    * Windows native: `sick_generic_caller sick_tim_5xx.launch`
    * Windows ROS-2:  `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_5xx.launch`
- For TiM7xx-family (no safety scanner):
    * Linux native:   `sick_generic_caller sick_tim_7xx.launch`
    * Linux ROS-1:    `roslaunch sick_scan sick_tim_7xx.launch`
    * Linux ROS-2:    `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xx.launch`
    * Windows native: `sick_generic_caller sick_tim_7xx.launch`
    * Windows ROS-2:  `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xx.launch`
- For TiM7xxS-family (safety scanner):
    * Linux native:   `sick_generic_caller sick_tim_7xxS.launch`
    * Linux ROS-1:    `roslaunch sick_scan sick_tim_7xxS.launch`
    * Linux ROS-2:    `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xxS.launch`
    * Windows native: `sick_generic_caller sick_tim_7xxS.launch`
    * Windows ROS-2:  `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xxS.launch`
- For LMS1xx-family:
    * Linux native:   `sick_generic_caller sick_lms_1xx.launch`
    * Linux ROS-1:    `roslaunch sick_scan sick_lms_1xx.launch`
    * Linux ROS-2:    `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_1xx.launch`
    * Windows native: `sick_generic_caller sick_lms_1xx.launch`
    * Windows ROS-2:  `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_1xx.launch`
- For LMS5xx-family:
    * Linux native:   `sick_generic_caller sick_lms_5xx.launch`
    * Linux ROS-1:    `roslaunch sick_scan sick_lms_5xx.launch`
    * Linux ROS-2:    `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_5xx.launch`
    * Windows native: `sick_generic_caller sick_lms_5xx.launch`
    * Windows ROS-2:  `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_5xx.launch`
- For LMS4xxx-family:
    * Linux native:   `sick_generic_caller sick_lms_4xxx.launch`
    * Linux ROS-1:    `roslaunch sick_scan sick_lms_4xxx.launch`
    * Linux ROS-2:    `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_4xxx.launch`
    * Windows native: `sick_generic_caller sick_lms_4xxx.launch`
    * Windows ROS-2:  `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_4xxx.launch`
- For LRS4000:
    * Linux native:   `sick_generic_caller sick_lrs_4xxx.launch`
    * Linux ROS-1:    `roslaunch sick_scan sick_lrs_4xxx.launch`
    * Linux ROS-2:    `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_lrs_4xxx.launch`
    * Windows native: `sick_generic_caller sick_lrs_4xxx.launch`
    * Windows ROS-2:  `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_lrs_4xxx.launch`
- For LDMRS-family:
    * Linux native:   `sick_generic_caller sick_ldmrs.launch`
    * Linux ROS-1:    `roslaunch sick_scan sick_ldmrs.launch`
    * Linux ROS-2:    `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_ldmrs.launch`
    * Note that LDMRS are currently not supported on Windows
- For NAV210 and NAV245:
    * Linux native:   `sick_generic_caller sick_nav_2xx.launch`
    * Linux ROS-1:    `roslaunch sick_scan sick_nav_2xx.launch`
    * Linux ROS-2:    `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_nav_2xx.launch`
    * Windows native: `sick_generic_caller sick_nav_2xx.launch`
    * Windows ROS-2:  `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_nav_2xx.launch`
- For NAV310:
    * Linux native:   `sick_generic_caller sick_nav_3xx.launch`
    * Linux ROS-1:    `roslaunch sick_scan sick_nav_3xx.launch`
    * Linux ROS-2:    `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_nav_3xx.launch`
    * Windows native: `sick_generic_caller sick_nav_3xx.launch`
    * Windows ROS-2:  `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_nav_3xx.launch`
- For RMS3xx-family:
    * Linux native:   `sick_generic_caller sick_rms_3xx.launch`
    * Linux ROS-1:    `roslaunch sick_scan sick_rms_3xx.launch`
    * Linux ROS-2:    `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_rms_3xx.launch`
    * Windows native: `sick_generic_caller sick_rms_3xx.launch`
    * Windows ROS-2:  `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_rms_3xx.launch`
- For RMS1xxx-family:
    * Linux native:   `sick_generic_caller sick_rms_1xxx.launch`
    * Linux ROS-1:    `roslaunch sick_scan sick_rms_1xxx.launch`
    * Linux ROS-2:    `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_rms_1xxx.launch`
    * Windows native: `sick_generic_caller sick_rms_1xxx.launch`
    * Windows ROS-2:  `ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_rms_1xxx.launch`

Common commandline options are

- `hostname:=<ip-address>` to connect to a sensor with a given IP address. Default value is always the factory default IP address of the scanner.

Further (common and scanner specific) options can be set via launchfile, see [Common parameters](#common-parameters) and configure the settings in the launchfile corresponding to the scanner type.

Note: After modifying a launch-file, it has to be installed by running `catkin_make_isolated --install --cmake-args -DROS_VERSION=1`
to be located and used by `roslaunch`.

On ROS-2 you can launch sick_generic_caller by python-launchfiles, too. Use
```
ros2 launch sick_scan <name>.launch.py <param>:=<value>
```
E.g. for LMS-5xx: `ros2 launch sick_scan sick_lms_5xx.launch.py hostname:=192.168.0.1`

The launch.py-files on ROS-2 passes the corresponding launch-file to the driver: [sick_lms_5xx.launch.py](launch/sick_lms_5xx.launch.py) gives an example for LMS-5xx. Parameter can be overwritten
* either by commandline, e.g. <br/> `ros2 launch sick_scan sick_lms_5xx.launch.py hostname:=192.168.0.1`, 
* or by passing additional arguments in the launch.py-file, e.g. <br/> `node = Node(package='sick_scan', executable='sick_generic_caller', arguments=[launch_file_path, 'hostname:=192.168.0.1'])`


### Start Multiple Nodes

Multiple nodes can be started to support multiple sensors. In this case, multiple instances of sick_scan have to be started, each node with different name and topic. ROS-1 example to run two TiM 7xx devices with ip address `192.168.0.1` and `192.168.0.2`:

```
roslaunch sick_scan sick_tim_7xx.launch nodename:=sick_tim_7xx_1 hostname:=192.168.0.1 cloud_topic:=cloud_1 &
roslaunch sick_scan sick_tim_7xx.launch nodename:=sick_tim_7xx_2 hostname:=192.168.0.2 cloud_topic:=cloud_2 &
```

On Linux with ROS-1, multiple nodes to support multiple sensors can be started by one launch file, too.
Take the launchfile [sick_tim_5xx_twin.launch](launch/sick_tim_5xx_twin.launch) as an example.
Remapping the scan and cloud topics is essential to distinguish the scandata and provide TF information.

ROS-2 example to run two TiM 7xx devices with ip address `192.168.0.1` and `192.168.0.2`:

```
ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xx.launch nodename:=sick_tim_7xx_1 hostname:=192.168.0.1 cloud_topic:=cloud_1 &
ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xx.launch nodename:=sick_tim_7xx_2 hostname:=192.168.0.2 cloud_topic:=cloud_2 &
```

### Common parameters

For the launch-file settings and the tag/values pairs the following keywords are supported:

| Keyword      |     Meaning     |  Default value |    Hint       |
|--------------|-----------------|----------------|---------------|
| scanner_type |  Scanner family |  ???           | see list above |
| min_ang      |  Start scan angle in [rad] |  -2.3998277           |  |
| max_ang      |  End scan angle in [rad] |  +2.3998277           |  |
| intensity_resolution_16bit | Switch between 8Bit/16Bit| "false" | do not change|
| hostname | Ip address of scanner  | 192.168.0.1 | change to scanner ip address in your network (see faq) |
| port | port number  | 2112 | do not change, check firewall rules if there is blocking traffic  |
| timelimit | Timelimit in [sec]   | 5 | do not change  |

- `scanner_type`
  Name of the used scanner. Usually this is also the name of the launch file. This entry is used to differentiate
  between the various scanner properties within the software code.

- `hostname`
  IP-address of the scanner (default: 192.168.0.1)

- `port`
  IP-port of the scanner (default: 2112)

- `min_ang`
  Start angle in [rad]

- `max_ang`
  End angle in [rad]

- `use_binary_protocol`
  Switch between SOPAS Binary and SOPAS ASCII protocol

- `intensity`
  Enable or disable transport of intensity values

- `intensity_resolution_16bit`
  If true, the intensity values is transferred as 16 bit value. If false, as 8 bit value.

- `min_intensity`
  If min_intensity > 0, all range values in a LaserScan message are set to infinity, if their intensity value is below min_intensity

- `cloud_topic`
  Topic name of the published pointcloud2 data

- `frame_id`
  Frame id used for the published data

Tag/value pairs of the commandline overwrite settings in the launch file.
The use of the parameters can be looked up in the launch files. This is also recommended as a starting point.

### Starting Scanner with Specific Ip Address

To start the scanner with a specific IP address, option `hostname:=<ip-address>` can be used.
The hostname is the ip-address of the scanner, e.g.
```
sick_generic_caller sick_tim_5xx.launch hostname:=192.168.0.71                      # Linux native
roslaunch sick_scan sick_tim_5xx.launch hostname:=192.168.0.71                      # Linux ROS-1
ros2 run sick_scan sick_generic_caller sick_tim_5xx.launch hostname:=192.168.0.71   # Linux ROS-2
sick_generic_caller sick_tim_5xx.launch hostname:=192.168.0.71                      # Windows native
ros2 run sick_scan sick_generic_caller sick_tim_5xx.launch hostname:=192.168.0.71   # Windows ROS-2
```

### Further useful parameters and features

- `timelimit`
  Timelimit in [sec] for max. wait time of incoming sensor reply

- `sw_pll_only_publish`
  If true, the internal Software PLL is fored to sync the scan generation time stamp to a system timestamp

- Angle compensation: For highest angle accuracy the NAV-Lidar series supports an [angle compensation mechanism](./doc/angular_compensation.md).

- **Field monitoring**: The **LMS1xx**, **LMS5xx**, **TiM7xx** and **TiM7xxS** families have [extended settings for field monitoring](./doc/field_monitoring_extensions.md).

- **Radar devices**: For radar devices (RMS-1xxx, RMS-3xx), radar raw targets or radar objects or both can be tracked and transmitted. You can activate parameter transmit_raw_targets, transmit_objects or both in the launchfile:
   ```
   <param name="transmit_raw_targets" type="bool" value="false"/>
   <param name="transmit_objects" type="bool" value="true"/>
   ```
   By default, radar objects are tracked.



### ROS services

On ROS-1 and ROS-2, services can be used to send COLA commands to the sensor. This can be very helpful for diagnosis, e.g. by querying the device status or its id.

Use the following examples to run a cola commond on ROS-1:
```
rosservice call /sick_lms_5xx/ColaMsg "{request: 'sMN IsSystemReady'}"
rosservice call /sick_lms_5xx/ColaMsg "{request: 'sRN SCdevicestate'}"
rosservice call /sick_lms_5xx/ColaMsg "{request: 'sEN LIDinputstate 1'}"
rosservice call /sick_lms_5xx/ColaMsg "{request: 'sEN LIDoutputstate 1'}"
rosservice call /sick_lms_5xx/ColaMsg "{request: 'sMN LMCstartmeas'}"
rosservice call /sick_lms_5xx/SCdevicestate "{}" # query device state
rosservice call /sick_lms_5xx/SCreboot "{}"      # execute a software reset on the device
rosservice call /sick_lms_5xx/SCsoftreset "{}"   # save current parameter and shut down device
```

Use the following examples to run a cola commond on ROS-2:
```
ros2 service call /ColaMsg sick_scan/srv/ColaMsgSrv "{request: 'sMN IsSystemReady'}"
ros2 service call /ColaMsg sick_scan/srv/ColaMsgSrv "{request: 'sRN SCdevicestate'}"
ros2 service call /ColaMsg sick_scan/srv/ColaMsgSrv "{request: 'sEN LIDinputstate 1'}"
ros2 service call /ColaMsg sick_scan/srv/ColaMsgSrv "{request: 'sEN LIDoutputstate 1'}"
ros2 service call /ColaMsg sick_scan/srv/ColaMsgSrv "{request: 'sMN LMCstartmeas'}"
ros2 service call /SCdevicestate sick_scan/srv/SCdevicestateSrv "{}" # query device state
ros2 service call /SCreboot sick_scan/srv/SCrebootSrv "{}"           # execute a software reset on the device
ros2 service call /SCsoftreset sick_scan/srv/SCsoftresetSrv "{}"     # save current parameter and shut down device
```

Use ros service `SickScanExit` to stop the scanner and driver:
```
rosservice call /sick_nav_3xx/SickScanExit "{}" # stop scanner and driver on ROS-1
ros2 service call /SickScanExit sick_scan/srv/SickScanExitSrv "{}" # stop scanner and driver on ROS-2
```

Note:
* The COLA commands are sensor specific. See the user manual and telegram listing for further details.
* ROS services require installation of ROS-1 or ROS-2, i.e. services for Cola commands are currently not supported on native Linux or native Windows.
* ROS services are currently not available for the LDMRS.
* Some SOPAS commands like `sMN SetAccessMode 3 F4724744` stop the current measurement. In this case, the driver restarts after a timeout (5 seconds by default). To process those SOPAS commands without restart, you can
   * send `sMN LMCstartmeas` and `sMN Run` to switch again into measurement mode within the timeout, or
   * increase the driver timeout `read_timeout_millisec_default` in the launch-file.

Example sequence with stop and start measurement to set a particle filter (TiM-7xxx on ROS-1):
```
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sMN SetAccessMode 3 F4724744'}"
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sRN LFPparticle'}" # response: "sRA LFPparticle \\x00\\x01\\xf4"
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sWN LFPparticle 0101F4'}" # response: "sWA LFPparticle"
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sMN LMCstartmeas'}"
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sMN Run'}"
```

### Driver states, timeouts

The driver runs in two different states:

1. Initialization: The scanner is initialized and configured by a list of sopas commands

2. Measurement: The scanner is operational, scandata are transmitted and the point cloud is published.
After start, the driver enters initialization mode. After successful initialization, the driver switches automatically into measurement mode.

The communication between driver and scanner is monitored. In case of communication timeouts, e.g. due to network problems, the TCP connection is reset and the scanner is re-initialized. The driver uses 3 different timeouts (i.e time since last message received from lidar):

1. In measurement mode: If no messages arrive for 5 seconds [timeout 0], the TCP/IP connection is closed. After a short delay, the tcp connection is reopened and the driver switches to initialisation mode and reinitialises the Lidar.

2. In initialisation mode: If no messages received after 120 sec [Timeout 1] the TCP/IP connection is closed. After a short delay, the tcp connection is reopened and the driver switches to initialisation mode and reinitialises the Lidar.

3. In any mode: If no messages received after 150 sec [Timeout 2] the driver terminates.

Note: The internal timer is reset on successful communication. i.e. the timeout refers to the time of the last message from the Lidar. If there was no message yet, then the time of programme start is used.

All timeouts can be configured in the launchfile:
```
<param name="message_monitoring_enabled" type="bool" value="True" />      <!-- Enable message monitoring with reconnect+reinit in case of timeouts, default: true -->
<param name="read_timeout_millisec_default" type="int" value="5000"/>     <!-- 5 sec read timeout in operational mode (measurement mode), default: 5000 milliseconds -->
<param name="read_timeout_millisec_startup" type="int" value="120000"/>   <!-- 120 sec read timeout during startup (sensor may be starting up, which can take up to 120 sec.), default: 120000 milliseconds -->
<param name="read_timeout_millisec_kill_node" type="int" value="150000"/> <!-- 150 sec pointcloud timeout, ros node will be killed if no point cloud published within the last 150 sec., default: 150000 milliseconds --> 
```

The following diagram shows the transition between the driver states:

![driverStatesDiagram](./doc/driverStatesDiagram1.png)

Note: Timeout 2 (i.e. no lidar message after 150 seconds) terminates the driver. By default, the driver does not restart automatically. It is therefor recommended to run the driver within an endless loop, e.g. in bash:

```
while(true) ; do roslaunch sick_scan <launchfile> [<arguments>] ; done
```


## Sopas Mode

This driver supports both COLA-B (binary) and COLA-A (ASCII) communication with the laser scanner. Binary mode is activated by default, since this mode generates less network traffic and enables more compatibility to all scanners.
If the communication mode set in the scanner memory is different from that used by the driver, the scanner's communication mode is changed. This requires a restart of the TCP-IP connection, which can extend the start time by up to 30 seconds.
There are two ways to prevent this:
1. Recommended:
   * Set the communication mode with the SOPAS ET software to binary and save this setting in the scanner's EEPROM.
   * Set "use_binary_protocol" to default value "true".
2. Use the parameter "use_binary_protocol" to overwrite the default settings of the driver.

## Bugs and Feature Requests

- Sopas protocol mapping:
-- All scanners: COLA-B (Binary)
- Software should be further tested, documented and beautified

## Tools

Various tools exist in the repository to improve the operation of the scanners. It is also recommended to read the following section "Troubleshooting".
Overview of the tools:

* Search for scanner in the network:
  Use the Python3 tool "sick_generic_device_finder.py" in the tools/sick_generic_device_finder directory.
  The tools will output the IP addresses of the connected scanners and some more information about the scanner.  
  Call it with python3, i.e.
  ``
  python3 sick_generic_device_finder.py
  ``
* Setting new IP address: With the help of the parameter "new_IP" a new IP address can be assigned when calling the node sick_scan.
  The launch file sick_new_ip.launch in the launch directory shows an example of how to use this parameter.
* Converting of pointclouds to images: With the tool pcl_converter.cpp one can convert pointcloud2-data
  to image. That is especial convenient for 24-layers scanners like the MRS6124.
* Setting up a brand new scanner: To set up a brand new scanner,
  it is recommended to use the two tools "sick_generic_device_finder.py" to find the scanner in the network
  and the launch file sick_new_ip.launch to set a new IP address. If further settings are to be saved that cannot be made via ROS   parameters, we recommend using the Windows tool "Sopas ET" from SICK.
* Unit tests: For a quick unit test after installation without the sensor hardware, a test server is provided to simulate a scanner. See [emulator](doc/emulator.md) for further details.
* Testing: The sick_scan_test program was developed for testing the driver. See [test/sick_scan_test.md](test/sick_scan_test.md) for details.

## Simulation

For unittests without sensor hardware, a simple test server is provided. To build the test server, activate cmake option `ENABLE_EMULATOR` in CMakeLists.txt and rebuild sick_scan. By default, option `ENABLE_EMULATOR` is switched off.

Please note that this just builds a simple test server for basic unittests of sick_scan drivers. Its purpose is to run basic tests and to help with diagnosis in case of issues. It does not emulate a real scanner!

Simulation requires jsoncpp. Install with `sudo apt-get install libjsoncpp-dev` on Linux and with `vcpkg install jsoncpp:x64-windows` on Windows.

You can find examples to test and run sick_scan in offline mode in folder `test/scripts`. Their purpose is to demonstrate the usage of the sick_scan driver. Please feel free to customize the scripts or use them as a starting point for own projects.

Note: Some larger scandata files for testing and development are provided in folder `test/emulator/scandata`. These files are versioned using "Git Large File Storage". Follow the description [doc/git_lfs.md](doc/git_lfs.md) to install and use git lfs extension.

### Simulation on Windows

Run script `run_simu_lms_5xx.cmd` in folder `test/scripts` or execute the following commands:

1. Start the test server:
    ```
    cd .\build
    start "testserver" cmd /k python ../test/emulator/test_server.py --scandata_file=../test/emulator/scandata/20210302_lms511.pcapng.scandata.txt --scandata_frequency=20.0 --tcp_port=2112
    @timeout /t 1
    ```

2. Run sick_generic_caller. On native Windows:
    ```
    .\Debug\sick_generic_caller.exe ../launch/sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False
    ```
    On Windows with ROS-2:
    ```
    ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False
    ```

3. Open file `image_viewer.html` in folder `demo` in your browser to view a jpg-image of the current scan.

Note, that python version 3 incl. runtime dlls must be accessable, f.e. by extending the PATH environment variable:
```
set PYTHON_DIR=%ProgramFiles(x86)%/Microsoft Visual Studio/Shared/Python37_64
set PATH=%PYTHON_DIR%;%PYTHON_DIR%/Scripts;c:\vcpkg\installed\x64-windows\bin;%PATH%
```

Further examples are provided in folder `test/scripts`.

### Simulation on Linux

Run script `run_simu_lms_5xx.bash` in folder `test/scripts` or execute the following commands:

1. Start the test server:
    ```
    python3 ./test/emulator/test_server.py --scandata_file=./test/emulator/scandata/20210302_lms511.pcapng.scandata.txt --scandata_frequency=20.0 --tcp_port=2112 &
    sleep 1
    ```

2. Run sick_generic_caller.
    - On native Linux:
         ```
        ./build/sick_generic_caller ./launch/sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
        ```
    - On Linux with ROS-1:
         ```
        roslaunch sick_scan sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
        ```
    - On Linux with ROS-2:
         ```
        ros2 run sick_scan sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
        ```

3. View the point cloud.
    - On native Linux:<br>
         Open file `image_viewer.html` in folder `demo` in a browser (f.e. firefox) to view a jpg-image of the current scan.
    - On Linux with ROS-1:
         ```
        rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_lms5xx.rviz &
        ```
    - On Linux with ROS-2:
         ```
        rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_lms5xx.rviz &
        ```

Further examples are provided in folder `test/scripts`.

## FAQ

* FAQ: [doc/faq.md](doc/faq.md)
* ROS installation: [doc/InstallROS2.md](doc/InstallROS2.md)

## Troubleshooting

The software is based on the ROS drivers sick_scan, sick_scan_base and sick_scan2. For FAQ and troubleshooting please also have a look at https://github.com/SICKAG/sick_scan , https://github.com/SICKAG/sick_scan_base and https://github.com/SICKAG/sick_scan2 .
Common problems might be solved in closed issues.

### General troubleshooting

1. Check Scanner IP in the launch file.
2. Check Ethernet connection to scanner with netcat e.g. ```nc -z -v -w5 $SCANNERIPADDRESS 2112```.
   For further details about setting up the correct ip settings see [IP configuration](doc/ipconfig/ipconfig.md)
3. View node startup output wether the IP connection could be established
4. Check the scanner status using the LEDs on the device. The LED codes are described in the above mentioned operation manuals.
5. Further testing and troubleshooting informations can found in the file test/readme_testplan.txt
6. If you stop the scanner in your debugging IDE or by other hard interruption (like Ctrl-C), you must wait until 60 sec. before
   the scanner is up and running again. During this time the MRS6124 reconnects twice.
   If you do not wait this waiting time you could see one of the following messages:
   * TCP connection error
   * Error-Message 0x0d
7. Amplitude values in rviz: If you see only one color in rviz try the following:
   Set the min/max-Range of intensity display in the range [0...200] and switch on the intensity flag in the launch file  
8. In case of network problems check your own ip address and the ip address of your laser scanner (by using SOPAS ET).
   * List of own IP-addresses: ifconfig|grep "inet addr"
   * Try to ping scanner ip address (used in launch file)
9. If the driver stops during init phase please stop the driver with ctrl-c and restart (could be caused due to protocol ASCII/Binary cola-dialect).

## Support

* In case of technical support please open a new issue. For optimal support, add the following information to your request:
 1. Scanner model name,
 2. Ros node startup log,
 3. Sopas file of your scanner configuration.
  The instructions at http://sickusablog.com/create-and-download-a-sopas-file/ show how to create the Sopas file.
* In case of application support please use [https://supportportal.sick.com ](https://supportportal.sick.com).
* Issue Handling: Issues, for which no reply was received from the questioner for more than 7 days,                     
  are closed by us because we assume that the user has solved the problem.

## Keywords

MRS1000
MRS1104
LMS1000
LMS1104
MRS6000
MRS6124
RMS1xxx
RMS1000
RMS3xx
RMS320
ROS LiDAR
SICK LiDAR
SICK Laser
SICK Laserscanner
SICK Radar
LMS1xx
MRS1xxx
LMS1xxx
MRS6xxx
TiM5xx
TiM551
TiM561
TiM571
TiM781
TiM781S
LMS5xx
LMS511
NAV210
NAV245
NAV310
LDMRS
LRS4000

## Creators

**Michael Lehning**

- <http://www.lehning.de>

on behalf of SICK AG

- <http://www.sick.com>

------------------------------------------------------------------------

<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/f/f1/Logo_SICK_AG_2009.svg/1200px-Logo_SICK_AG_2009.svg.png" width="420">

![Lehning Logo](http://www.lehning.de/style/banner.jpg "LEHNING Logo")
