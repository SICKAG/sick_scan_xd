# sick_scan_xd

This project provides a driver for the SICK LiDARs and Radar sensors mentioned [here](REQUIREMENTS.md). The driver supports both Linux (native, ROS1, ROS2) and Windows (native and ROS2). See the [CHANGELOG.md](CHANGELOG.md) for the latest updates.

## Table of Contents

- [Executive Summary](#executive-summary)
- [Supported SICK LIDAR and Radar sensors](REQUIREMENTS.md)
- [Getting started](GETTINGSTARTED.md)
- [Build targets](#build-targets)
- [Build on Linux generic without ROS](INSTALL-GENERIC.md#build-on-linux-generic-without-ros)
- [Build on Linux ROS1](INSTALL-ROS1.md#build-on-linux-ros1)
- [Build on Linux ROS2](INSTALL-ROS2.md#build-on-linux-ros2)
- [Build on Windows](INSTALL-GENERIC.md#build-on-windows)
- [Build on Windows ROS2](INSTALL-ROS2.md#build-on-windows-ros2)
- [USAGE](USAGE.md)
- [Driver API](doc/sick_scan_api/sick_scan_api.md)
- [IMU-Support](#imu-Support)
- [Radar](doc/radar.md)
- [picoScan100/multiScan100](doc/sick_scan_segment_xd.md)
- [NAV350](doc/nav350.md)
- [Software PLL](#software-pll)
- [Field Evaluation Information](#field-extensions)
- [SLAM-Support](doc/slam.md)
- [Software Overview](#software-overview)
- [Raspberry support](doc/raspberry.md)
- [Interlacing](doc/interlacing.md)
- [FAQ](FAQ.md)
- [Further support](SUPPORT.md)
- [CREDITS](CREDITS.md)

## Executive Summary

* sick_scan_xd supports
    * ROS1 (Linux)
    * ROS2 (Linux and Windows)
    * a Driver for generic use (Linux and Windows native)
    * a API for C/C++ or python applications
    * x64 and ARM-64 architecture
* sick_scan_xd provides a driver for the SICK LiDARs and Radar sensors mentioned [here](REQUIREMENTS.md).
* sick_scan_xd is designed to easily integrate new devices, features and improvements on all targets.
* sick_scan_xd has no dependencies to 3rd party libraries like boost or pthread.
* sick_scan_xd offers all features on all targets if the devices support the features.

## Repository organization

The repository supports two main branches.

The **"master"** branch is the branch that contains official releases that are tagged and versioned and also included in the ROS distribution. 

If you want to work with this official branch, you must explicitly specify this branch in the 'git clone' command by adding "-b master".

The "develop" branch is the default branch and contains the latest development status.

Example:

Checking out the latest revision (usually older than the develop version, but officially released):
```
git clone -b master https://github.com/SICKAG/sick_scan_xd.git
```

Checking out the latest development status:
```
git clone https://github.com/SICKAG/sick_scan_xd.git
```

## Build targets

sick_scan_xd can be build on 64-bit Linux and Windows, with and without ROS, with and without LDMRS. The following table shows the allowed combinations and how to build.

| **target** | **cmake settings** | **build script** |
|------------|--------------------|------------------|
| Linux, native, LDMRS      | BUILD_WITH_LDMRS_SUPPORT ON  | cd test/scripts && chmod a+x ./*.bash && ./makeall_linux.bash          |
| Linux, native, no LDMRS   | BUILD_WITH_LDMRS_SUPPORT OFF | cd test/scripts && chmod a+x ./*.bash && ./makeall_linux_no_ldmrs.bash |
| Linux, ROS-1, LDMRS       | BUILD_WITH_LDMRS_SUPPORT ON  | cd test/scripts && chmod a+x ./*.bash && ./makeall_ros1.bash           |
| Linux, ROS-1, no LDMRS    | BUILD_WITH_LDMRS_SUPPORT OFF | cd test/scripts && chmod a+x ./*.bash && ./makeall_ros1_no_ldmrs.bash  |
| Linux, ROS-2, LDMRS       | BUILD_WITH_LDMRS_SUPPORT ON  | cd test/scripts && chmod a+x ./*.bash && ./makeall_ros2.bash           |
| Linux, ROS-2, no LDMRS    | BUILD_WITH_LDMRS_SUPPORT OFF | cd test/scripts && chmod a+x ./*.bash && ./makeall_ros2_no_ldmrs.bash  |
| Windows, native, no LDMRS | BUILD_WITH_LDMRS_SUPPORT OFF | cd test\\scripts && make_win64.cmd |
| Windows, ROS-2, no LDMRS  | BUILD_WITH_LDMRS_SUPPORT OFF | cd test\\scripts && make_ros2.cmd  |

If you're using ROS, set your ROS-environment before running one of these scripts, f.e.
* `source /opt/ros/noetic/setup.bash` for ROS-1 noetic, or
* `source /opt/ros/foxy/setup.bash` for ROS-2 foxy, or
* `source /opt/ros/humble/setup.bash` for ROS-2 humble.

See the build descriptions for more details:
* [Build on Linux generic without ROS](INSTALL-GENERIC.md#build-on-linux-generic-without-ros)
* [Build on Linux ROS1](INSTALL-GENERIC.md#build-on-linux-ros1)
* [Build on Linux ROS2](INSTALL-ROS2.md#build-on-linux-ros2)
* [Build on Windows](INSTALL-GENERIC.md#build-on-windows)
* [Build on Windows ROS2](INSTALL-ROS2.md#build-on-windows-ros2)

sick_scan_xd supports 64 bit Linux and Windows, 32 bit systems are not supported.

## Driver API

sick_scan_xd provides a C API, which can be used by any programming language with C-bindings, e.g. in C/C++ or python applications. See [sick_scan_api.md](doc/sick_scan_api/sick_scan_api.md) for further details.

## IMU Support

Devices of the MRS6xxx and MRS1xxx series are available with an optionally built-in IMU.
Further information on the implementation and use of the experimental Imu support can be found on the [Imu page](doc/IMU.md).

## Radar support

See [radar documentation](doc/radar.md) for RMSxxxx support.

## multiScan100 support

See [sick_scan_segment_xd](doc/sick_scan_segment_xd.md) for multiScan100 support.

## Software PLL

A software pll is used to convert LiDAR timestamps in ticks to the ros system time. See [software_pll](doc/software_pll.md) for further details.

## Field Evaluation Information

The LMS1xx, LMS5xx, TiM7xx and TiM7xxS families support extensions for field monitoring. See [field_monitoring_extensions](doc/field_monitoring_extensions.md) for further details.

## Run sick_scan_xd driver

See [USAGE](USAGE.md) how to run and configure the sick_scan_xd driver.

## Software Overview

An overview over the software and its modules can be found in [software_overview](doc/software_overview.md).

## FAQ

* FAQ: [FAQ](FAQ.md)

## Keywords

MRS1000
MRS1104
LMS1000
LMS1104
MRS6000
MRS6124
RMS1xxx
RMS1000
RMSxxxx
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
LD-LRS3600
LD-LRS3601
LD-LRS3611
LD-OEM1500
LD-OEM1501
multiScan100
multiScan
picoScan100
picoScan