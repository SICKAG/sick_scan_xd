# sick_scan_xd backlog

## Todo

* github: folder test/emulator/scandata/ ignored (scandata files too big) -> move to a public testdata drive?
* For Non-ROS: replacement for ros services
* For Non-ROS: publish and visualization field monitoring

## Overview

### Targets

* Merge sick_scan, sick_scan_base and sick_scan2
* Support Linux (standard, ROS1, ROS2) and Windows (standard and ROS2)

### Modules

* Core: systemindependant core functions and communication with Lidar devices
* Core api: communication with core via C++ and event callbacks
* Subscriber: Systemdependant callback implementations, f.e. scandata-callback: publish on ROS, logging on non-ROS
* Configuration: yaml-files plus get/set-functions

## Version history

* commit 5287911 on Jun 24, 2021: initial version based on https://github.com/SICKAG/sick_scan commit 5287911 on Jun 24, 2021 (sick_scan_pretest release 1.10.11)
* merge sick_scan and sick_scan_base, support Windows, Linux and Linux-ROS1
