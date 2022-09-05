# sick_scan_xd

This project provides the driver for the SICK lidar and radar sensors mentioned in the following list.

Based on the sick_scan drivers for ROS1, sick_scan_xd merges sick_scan, sick_scan2 and sick_scan_base repositories. The driver supports both Linux (native, ROS1, ROS2) and Windows (native and ROS2). See the [CHANGELOG.md](CHANGELOG.md) for the latest updates.

## Table of Contents

- [Executive Summary](#executive-summary)
- [Supported hardware and platforms](REQUIREMENTS.md)
- [Build targets](#build-targets)
- [Build on Linux generic without ROS](INSTALL-GENERIC.md#build-on-linux-generic-without-ros)
- [Build on Linux ROS1](INSTALL-ROS1.md#build-on-linux-ros1)
- [Build on Linux ROS2](INSTALL-ROS2.md#build-on-linux-ros2)
- [Build on Windows](INSTALL-GENERIC.md#build-on-windows)
- [Build on Windows ROS2](INSTALL-ROS2.md#build-on-windows-ros2)
- [USAGE](USAGE.md)
- [API](#api)
- [IMU-Support](#imu-Support)
- [Radar](doc/radar.md)
- [Multiscan136](doc/sick_scan_segment_xd.md)
- [Software PLL](#software-pll)
- [Field extensions](#field-extensions)
- [SLAM-Support](doc/slam.md)
- [Profiling](doc/profiling.md)
- [Software Overview](#software-overview)
- [FAQ](FAQ.md)
- [Further support](SUPPORT.md)
- [CREDITS](CREDITS.md)

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

## Supported hardware and platforms

See [REQUIREMENTS](./REQUIREMENTS.md) for supported hardware and platforms.

## Build targets

sick_scan_xd can be build on Linux and Windows, with and without ROS, with and without LDMRS. The following table shows the allowed combinations and how to build.

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
* `source /opt/ros/melodic/setup.bash` for ROS-1 melodic, or
* `source /opt/ros/eloquent/setup.bash` for ROS-2 eloquent, or
* `source /opt/ros/foxy/setup.bash` for ROS-2 foxy.

See the build descriptions for more details:
* [Build on Linux generic without ROS](INSTALL-GENERIC.md#build-on-linux-generic-without-ros)
* [Build on Linux ROS1](INSTALL-GENERIC.md#build-on-linux-ros1)
* [Build on Linux ROS2](INSTALL-ROS2.md#build-on-linux-ros2)
* [Build on Windows](INSTALL-GENERIC.md#build-on-windows)
* [Build on Windows ROS2](INSTALL-ROS2.md#build-on-windows-ros2)

## API

sick_scan_xd provides a C API, which can be used by any programming language with C-bindings, e.g. in C/C++ or python applications. See [sick_scan_api.md](doc/sick_scan_api/sick_scan_api.md) for further details.

## IMU Support

Devices of the MRS6xxx and MRS1xxx series are available with an optionally built-in IMU.
Further information on the implementation and use of the experimental Imu support can be found on the [Imu page](doc/IMU.md).

## Radar support

See [radar documentation](doc/radar.md) for RMS1xxx and RMS3xx support.

## Multiscan136 support

See [sick_scan_segment_xd](doc/sick_scan_segment_xd.md) for Multiscan136 support.

## Software PLL

A software pll is used to convert lidar timestamps in ticks to the ros system time. See [software_pll](doc/software_pll.md) for further details.

## Field extensions

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
Multiscan136

