# sick_scan_xd backlog

## Todo

* Test ros services on Windows-ROS2
* Test with all supported targets
* features for future releases:
   * REST-API
   * adapt libsick_ldmrs and support LDMRS on Windows
   * support ros-like services for cola commands on native Windows and native Linux
   * [optional: replace launchparser for ROS2 (ROS2 uses USE_LAUNCHPARSER in sick_generic_laser.cpp, switch to ROS2 standard parameter handling?)]
   * replace boost and pthread with std C++ 14, remove pcl-dependencies for LDMRS support
   
## Overview

### Targets

* Merge sick_scan, sick_scan_base and sick_scan2
* Support Linux (native, ROS1, ROS2) and Windows (native and ROS2)

## Version history

* commit 5287911 on Jun 24, 2021: initial version based on https://github.com/SICKAG/sick_scan commit 5287911 on Jun 24, 2021 (sick_scan_pretest release 1.10.11)
* commit 21f5f60 on Jun 25, 2021: merge sick_scan and sick_scan_base, support for Windows, Linux and Linux-ROS1
* commit ffde775 on Jun 30, 2021: support for ROS 2
* commit f72f8cf on Jul 01, 2021: test and minor fixes
* commit aed7419 on Jul 06, 2021: merge with sick_scan2, ros services
* Release 0.1.0, commit 5568f73 on Jul 08, 2021: compatibility for targets Linux (native, ROS-1, ROS-2) and Windows (native, ROS2)
* Release 0.2.0, commit 343e88b on Jul 09, 2021: unittests
* Release 0.3.0, commit 267232a on Jul 13, 2021: Scandata files using Git Large File Storage
* Release 1.0.0, commit  on Jul 14, 2021: Offline tests, RC1
