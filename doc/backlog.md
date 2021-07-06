# sick_scan_xD backlog

## Todo

* For Windows/Linux native and ROS-2: support / replacement for ros services, enable USE_ROSSERVICES in sick_generic_laser.cpp
* Test server support for all targets in CMakeLists.txt
* support diagnostic_updater for ROS2 + native OS (Windows + Linux), enable USE_DIAGNOSTIC_UPDATER resp. USE_DIAGNOSTIC_UPDATER_LDMRS in sick_ros_wrapper.h
* support dynamic_reconfigure for ROS2 + native OS (Windows + Linux), enable USE_DYNAMIC_RECONFIGURE in sick_ros_wrapper.h
* support libsick_ldmrs for Native OS (Windows + Linux)
* For Windows/Linux native: visualize field monitoring using https://github.com/michael1309/pgmHandling
* Test (emualator and hardware)
* Documentation (incl. table of supported sensors and features supported by Win/Linux/native/ROS1/ROS2)
* features for the future:
   * replace launchparser for ROS2 (ROS2 uses USE_LAUNCHPARSER in sick_generic_laser.cpp, switch to ROS2 standard parameter handling?)
   * replace boost and pthread with std C++ 14
   * ros-like services on native Windows + Linux 
* Renaming from sick_scan_xd to sick_scan_xD
   
## Issues
* github: folder test/emulator/scandata/ ignored (scandata files too big) -> move to a public testdata drive?
    * Git Large File Storage
    * https://git-lfs.github.com/
    * https://docs.github.com/en/github/managing-large-files/versioning-large-files/configuring-git-large-file-storage
* Handling of scanner start/end angles different in ROS1 and ROS2 -> += 90.0 for all scanners except TiM240?
    sick_scan_common.cpp sick_scan (ROS1):
    ```
      // convert to 10000th degree
      double minAngSopas = rad2deg(this->config_.min_ang);
      double maxAngSopas = rad2deg(this->config_.max_ang);
      if (this->parser_->getCurrentParamPtr()->getScannerName().compare(SICK_SCANNER_TIM_240_NAME) == 0)
      {
        // the TiM240 operates directly in the ros coordinate system
      }
      else
      {
        minAngSopas += 90.0;
        maxAngSopas += 90.0;
      }
      angleStart10000th = (int) (boost::math::round(10000.0 * minAngSopas));
      angleEnd10000th = (int) (boost::math::round(10000.0 * maxAngSopas));
    ```
    sick_scan_common.cpp:1549 sick_scan2 (ROS2):
    ```
      // convert to 10000th degree
      double minAngSopas = rad2deg(this->config_.min_ang);
      double maxAngSopas = rad2deg(this->config_.max_ang);
      if (this->parser_->getCurrentParamPtr()->getScannerName().compare(SICK_SCANNER_TIM_240_NAME) == 0)
      {
        // the TiM240 operates directly in the ros coordinate system
      }
      else
      {
        //TODO change scanAngleShift for other scanners
        //minAngSopas += 90.0;
        //maxAngSopas += 90.0;
      }
      angleStart10000th = (int) (0.5 + 10000.0 * minAngSopas);
      angleEnd10000th = (int) (0.5 + 10000.0 * maxAngSopas);
    ```
    sick_scan_common.cpp:2010 sick_scan (ROS1):
    ```
        double askAngleRes = askAngleRes10000th / 10000.0;
        double askAngleStart = askAngleStart10000th / 10000.0;
        double askAngleEnd = askAngleEnd10000th / 10000.0;
        if (this->parser_->getCurrentParamPtr()->getScannerName().compare(SICK_SCANNER_TIM_240_NAME) == 0)
        {
          // the TiM240 operates directly in the ros coordinate system
        }
        else
        {
          askAngleStart -= 90; // angle in ROS relative to y-axis
          askAngleEnd -= 90; // angle in ROS relative to y-axis
        }
        this->config_.min_ang = askAngleStart / 180.0 * M_PI;
        this->config_.max_ang = askAngleEnd / 180.0 * M_PI;
    ```
    sick_scan_common.cpp:1714 sick_scan2 (ROS2):
    ```
        double askAngleRes = askAngleRes10000th / 10000.0;
        double askAngleStart = askAngleStart10000th / 10000.0;
        double askAngleEnd = askAngleEnd10000th / 10000.0;
        double angshift= this->parser_->getCurrentParamPtr()->getScanAngleShift();
        this->config_.min_ang = (askAngleStart / 180.0 * M_PI);
        this->config_.max_ang = (askAngleEnd / 180.0 * M_PI);
    ```
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
    <param name="min_ang" type="double" value="-2.35619449"/> <!-- -135° -->
    <param name="max_ang" type="double" value="2.35619449"/>  <!-- +135° -->
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

## Known issues

* cmake error message "python_d.exe not found" when running rosidl generator: 
   * Workaround: Copy python.exe to python_d.exe in the python folder.

## Version history

* commit 5287911 on Jun 24, 2021: initial version based on https://github.com/SICKAG/sick_scan commit 5287911 on Jun 24, 2021 (sick_scan_pretest release 1.10.11)
* commit 21f5f60 on Jun 25, 2021: merge sick_scan and sick_scan_base, support for Windows, Linux and Linux-ROS1
* commit ffde775 on Jun 30, 2021: support for ROS 2
* commit f72f8cf on Jul 01, 2021: test and minor fixes
