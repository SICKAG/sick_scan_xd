# sick_scan_xd backlog

## Todo

* Test ros services on Windows-ROS2
* Documentation (incl. table of supported sensors and features supported by Windows/Linux/native/ROS1/ROS2)
* Test with all supported targets
* For Windows/Linux native: visualize field monitoring using https://github.com/michael1309/pgmHandling
* features for the future:
   * replace launchparser for ROS2 (ROS2 uses USE_LAUNCHPARSER in sick_generic_laser.cpp, switch to ROS2 standard parameter handling?)
   * adapt libsick_ldmrs and support LDMRS on Windows
   * support ros-like services for cola commands on native Windows and native Linux
   * REST-API
   * replace boost and pthread with std C++ 14
* github: folder test/emulator/scandata/ ignored (scandata files too big) -> move to Git Large File Storage
    * https://git-lfs.github.com/
    * https://docs.github.com/en/github/managing-large-files/versioning-large-files/configuring-git-large-file-storage
* sick_scan, sick_scan_xd: #131 https://github.com/SICKAG/sick_scan/issues/131 (intensity < min_intensity: range := inf, nachführen in sick_scan und sick_scan_xd, wie in sick_safetyscanners-master\src\SickSafetyscannersRos.cpp):
   ```
   if (m_min_intensities >= static_cast<double>(scan_point.getReflectivity())) { scan.ranges[i] = std::numeric_limits<double>::infinity(); } 
   ```
   
## Issues
* ROS1/ROS2 configuration min/max_ang for mrs_1xxx, tim_5xx:
    sick_scan2/config/sick_mrs_1xxx.yaml: 
    ```
    min_ang : -2.35619449 #in lidar cordinate frame  see ../doc/3d_coordinate_system_comp.png
    max_ang : 2.35619449 #in lidar cordinate frame  see ../doc/3d_coordinate_system_comp.png
    ```
    sick_scan/launch/sick_mrs_1xxx.launch:
    ```
    <param name="min_ang" type="double" value="-2.3998277"/>
    <param name="max_ang" type="double" value="+2.3998277"/>
    ```
    sick_scan2/config/sick_tim_5xx.yaml: 
    ```
    min_ang : 0.0 #in lidar cordinate frame  see ../doc/3d_coordinate_system_comp.png
    max_ang : 3.141 #in lidar cordinate frame  see ../doc/3d_coordinate_system_comp.png
    ```
    sick_scan/launch/sick_tim_5xx.launch:
    ```
    <param name="min_ang" type="double" value="-2.35619449"/> <!-- -135 deg -->
    <param name="max_ang" type="double" value="2.35619449"/>  <!-- +135 deg -->
    ```

## Overview

### Targets

* Merge sick_scan, sick_scan_base and sick_scan2
* Support Linux (native, ROS1, ROS2) and Windows (native and ROS2)

### Modules

* Core: systemindependant core functions and communication with Lidar devices
* Core api: communication with core via C++ and event callbacks
* Subscriber: Systemdependant callback implementations, f.e. scandata-callback: publish on ROS, logging on non-ROS
* Configuration: yaml-files plus get/set-functions

## Version history

* commit 5287911 on Jun 24, 2021: initial version based on https://github.com/SICKAG/sick_scan commit 5287911 on Jun 24, 2021 (sick_scan_pretest release 1.10.11)
* commit 21f5f60 on Jun 25, 2021: merge sick_scan and sick_scan_base, support for Windows, Linux and Linux-ROS1
* commit ffde775 on Jun 30, 2021: support for ROS 2
* commit f72f8cf on Jul 01, 2021: test and minor fixes
* commit aed7419 on Jul 06, 2021: merge with sick_scan2, ros services
* Release 0.1.0, commit 5568f73 on Jul 08, 2021: compatibility for targets Linux (native, ROS-1, ROS-2) and Windows (native, ROS2)
* Release 0.2.0, commit 343e88b on Jul 09, 2021: unittests
