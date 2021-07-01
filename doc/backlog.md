# sick_scan_xd backlog

## Todo

* github: folder test/emulator/scandata/ ignored (scandata files too big) -> move to a public testdata drive?
* Merge sick_scan_xd and sick_scan2
* LDMRS support for ROS1 + ROS2 + Native OS (Windows + Linux)
* For Windows/Linux native and ROS-2: support / replacement for ros services, enable USE_ROSSERVICES in sick_generic_laser.cpp
* support diagnostic_updater for ROS2 + Native OS (Windows + Linux), enable USE_DIAGNOSTIC_UPDATER in sick_ros_wrapper.h
* support dynamic_reconfigure for ROS2 + Native OS (Windows + Linux), enable USE_DYNAMIC_RECONFIGURE in sick_ros_wrapper.h
* use USE_LAUNCHPARSER in sick_generic_laser.cpp for ROS2 (later: ROS2 standard parameter handling)
* For Windows/Linux native: visualize field monitoring using https://github.com/michael1309/pgmHandling
* Documentation (incl. table of supported sensors and features supported by Win/Linux/native/ROS1/ROS2)
* features for the future:
   * replace boost and pthread with std C++ 14
   * ros-like services on native Windows + Linux 

## Overview

### Targets

* Merge sick_scan, sick_scan_base and sick_scan2
* Support Linux (native, ROS1, ROS2) and Windows (native and ROS2)

### Modules

* Core: systemindependant core functions and communication with Lidar devices
* Core api: communication with core via C++ and event callbacks
* Subscriber: Systemdependant callback implementations, f.e. scandata-callback: publish on ROS, logging on non-ROS
* Configuration: yaml-files plus get/set-functions

## Known issues

* cmake error message "python_d.exe not found" when running rosidl generator: 
   * Workaround: Copy python.exe to python_d.exe in the python folder.

## Version history

* commit 5287911 on Jun 24, 2021: initial version based on https://github.com/SICKAG/sick_scan commit 5287911 on Jun 24, 2021 (sick_scan_pretest release 1.10.11)
* commit 21f5f60 on Jun 25, 2021: merge sick_scan and sick_scan_base, support for Windows, Linux and Linux-ROS1
* commit ffde775 on Jun 30, 2021: support for ROS 2
* commit  on Jul 01, 2021: test and minor fixes
