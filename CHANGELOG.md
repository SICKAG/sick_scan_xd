# Changelog #

Possible types are **Added** for new features. **Changed** for changes to the existing functionality. **Deprecated** for
features that will be removed in future versions **Removed** for deprecated features that were removed in this release.
**Fixed** for all bug fixes. **Security** to prompt users to update in case of closed vulnerabilities.

## Released ##

### Release 3.4.0

  - **add** azimut angle table for MRS-1xxx and LMS-1xxx with firmware 2.2.0 oder newer
  - **add** dockertests for MRS-1xxx, multiScan and picoScan with ROS-2
  - **add** API-funktion SickScanApiSendSOPAS to send SOPAS commands (e.g. "sRN SCdevicestate" or "sRN ContaminationResult")
  - **add** generation of TF messages
  - **add** Option to deactivate initialization sequence for TiM-7xxS devices
  - **add** Documented option "-b master"  to clone the release version
  - **fix** #316 API re-init nach close

### Release v3.3.0

  - **add** Option for MRS1xxx azimuth correction table
  - **add** Support for picoScan100 (single echo w/o addons)
  - **add** API logging functions and verbosity (#270)
  - **add** API documentation (multiple lidars not supported, #281)
  - **change** API extended (added topic in PointCloud messages, #271)

### Release v3.2.0

  - **add** IMU support for multiScan and picoScan
  - **add** support bloom releases for ROS-2 humble
  - **add** docker tests for ROS1 noetic incl. testcases for multiScan, picoScan, MRS1xxx
  - **add** PR #255, support picoScan performance profiles
  - **change** configuration time flag LMDscandatacfg switched off for the TiM240
  - **fix** #218 (API reinit)
  - **fix** #220 (Fullframe Laserscan messages multi- and picoScan)
  - **fix** #221 (No scandata while activated scan range filter)
  - **fix** #222 (rviz visualization of polar pointclouds)
  - **fix** #247 (Launchfile option for laserscan topic)
  - **fix** #256 (Clean API-exit picoScan and multiScan)
  - **fix** #260 (Provide API-functions to query lidar status, error codes and error messages)

### Release v3.1.0

  - **add** IMU support for multiScan
  - **add** LaserScan output for picoScan
  - **fix** API reinit
  - **fix** multiScan data output with range filter activated
  - **fix** adapt multiScan startup and shutdown sequence
  - **fix** adaptations for MRS-1000 v2 firmware

### Release v3.0.0

This release has a new major version as it breaks with the previously used ROS module name "sick_scan".
For consistency, the ROS module name has been changed to "sick_scan_xd" to match the name used everywhere else.

  - **add** New topics for additional data and properties
  - **add** Official ARM64 support
  - **add** Service for requesting contamination detection information
  - **change** Improvements from customer tickets and documentation enhancements
  - **fix** ROS module name consistency

### Release v2.10.3
  - **fix** avoid problems with min/max definition in the STL and preprocessor definitions

### Release v2.10.2
  - **fix** correct picoScan details in documentation and launch files

### Release v2.10.1
  - **update** make Compact format the default for picoScan and multiScan

### Release v2.10.0
  - **add** picoScan support
  - **add** Compact format support
  - **add** LMS4000 encoder setting support
  - **fix** multiScan angle range filter parameter unit
  - **fix** MRS1104 diagnostic message
  - **fix** TiM781S login

### v2.9.1 - Angle correction
  - **Fix** Angle correction (min/max angle settings), fix #166
  - **Fix** TiM240 initialization (start measurement)
  - **Added** Documentation for Interlacing mode

### v2.9.0 - RMSxxxx support and NAV350 support
  - **Added** RMSxxxx support, unification of RMS-1xxx and RMS-2xxx Note: RMSxxxx supports ASCII-communication mode only (Cola-A).
  - **Update** #159 (nav310 angle setting compability), merge with NAV310 angle settings branch https://github.com/SICKAG/sick_scan_xd/tree/159-nav310-angle-setting-compability
  - **Update** Documentation LD-LRS3600,LD-LRS3601,LD-LRS3611,LD-OEM1500,LD-OEM1501 support
  - **Update** Removed obsolete RMS-3xx
  - **Added** NAV350 support
  - **Update** Merge lms511_field_mon (fix lms511 field parsing and wait api), default_echo_setting (activate last echo by default), monitoring_ros2_qos (ROS2 QoS configuration), scansegment_xd_support (update build instructions), rename-mrs100-multiscan, rename-fullframe-topic

### v2.8.15 - Release Jan. 2023
  - **Update** Win64 build instructions
  - **Update** API documentation, driver states diagrams and typos
  - **Added**  LRS-36xx configuration for upside-down mounting
  - **Removed** Obsolete service commands and RMS3xx support
  - **Fix** LRS-36xx angle configuration
  - **Fix** catkin_lint warnings
  - **Fix** ROS-2 Humble build

### v2.8.14 - Laserscan messages for multiScan136
  - **Update** Laserscan messages for multiScan136 lidar, #96

### v2.8.13 - Dynamical pointcloud transform and QoS configuration
  - **Update** Configuration of ROS quality of service by launchfile, #101
  - **Update** Dynamical configuration of an additional pointcloud transform by rosparam, #104

### v2.8.11 - LMS 1xxx support
  - **Update** LMS 1xxx support with scan configuration (scan frequency and angular resolution for firmware 2.x)

### v2.8.10 - RMS ascii emulator and tests
  - **Update** RMS ascii emulator and tests
  - **Update** RMS2xxx support

### v2.8.9 - MRS-1000 layer angle conversion, improved MRS 1xxx support
  - **Fixed** MRS-1000 layer angle conversion for slam support

### v2.8.8 - RMS1xxx Cola-ASCII support
  - **Update** Update for RMS1xxx Cola-ASCII support

### v2.8.7 - Range filter
  - **Update** Range filter settings, #98 and #108
  - **Update** Preparation for RMS1xxx support (tutorial, preparation for RMS1/RMS2, not activated)

### v2.8.6 - multiScan136 update
  - **Update** multiScan136 update for 16-bit RSSI and modified SOPAS startup sequence

### v2.8.5 - LRS4000 update
  - **Update** LRS4000 extended configuration (glare detection sensitivity, echo-, mean-, median-filter)
  - **Added** ROS-2 usage example

### v2.8.4 - Generic API
  - **Update** Fix of version number
  - **Added**  GETTINGSTARTED.md

### v2.8.3 - Generic API
  - **Update** Update for ROS-2 Humble and docker container
  - **Update** Integrate API feedback, documentation and minimalistic usage examples
  - **Added** Minimalistic API usage examples (Python, C, C++)
  - **Update** Documentation
  - **Update** Doxygen and numpy.docstring support
  - **Fixed** Compiler warnings (Visual Studio)
  - **Update** README.md restructured
  - **Update** Collected update including previous v2.8.x changes

### v2.8.2 - development branch
  - **Fixed** Update build instructions in README.md, visualization in python API-example with low frequency to reduce cpu usage.

### v2.8.1 - development branch
  - **Added** Generic API implementation
  - **Fixed** LMS-111 field marker

### v2.8.0 - development branch
  - **Added** Generic interface, C-API
  - **Changed** merge pull request #85 (adapt to ROS2 Humble)
  - **Changed** Changed to namespace roswrap in ros wrapper classes
  - **Fixed** Library linker flag #91

### v2.7.5 -
  - **Fixed** MRS6124 pointcloud error #88

### v2.7.4 -
  - **Fixed** ROS2 compilation error #83

### v2.7.3 -
  - **Added** Supported for LFPmeanfilter and LFPmedianfilter (MRS1xxx, LMS1xxx, LMS4xxx, LRS4xxx)
  - **Added** Supported for LMDscandatascalefactor (LRS4xxx)

### v2.7.0 -
  - **Added** V2.7.0: Support for multiScan136 (sick_scansegment_xd)
  - **Fixed** Timestamp LaserScan-message corrected (identical timestamps in LaserScan- and PointCloud2-messages, both by Software-PLL)

### v2.6.8 -
  - **Fixed** Merge pull request #76

### v2.6.7 -
  - **Update** LMS511 configuration #67

### v2.6.6 -
  - **Update** NAV310 + LRS4xxx update, issues #58, #59, #60, #61

### v2.6.5 -
  - **Fixed** LRS4xxx scan configuration #52

### v2.6.4 -
  - **Fixed** LMS5xx echo filter settings corrected

### v2.6.3 -
  - **Fixed** Timestamp Laserscan message corrected

### v2.6.2 -
  - **Fixed** LDMRS spinning problem corrected

### v2.6.1 -
  - **Added** V2.6.1: Support for RMS-1xxx binary protocol

### v2.6.0 -
  - **Changed** V2.6.0: RMS configuration update, issue #7

### v2.5.2 -
  - **Fixed**  Fix LSR-4xxx laserscan angles #28
  - **Fixed**  Fix duplicated laserscan messages #28

### v2.5.1 -
  - **Fixed**  Error after SOPAS command SetAccessMode #27

### v2.5.0 -
  - **Fixed** Issue #24 (stop scanner at exit)
  - **Added** new ros service SickScanExit to stop scanner and exit

### v2.4.6 -
  - **Fixed** Corrected angle shift parameter for LMS-4xxx
  - **Changed** Typo corrected

### v2.4.5 -
  - **Changed** bugfix #158 (driver terminates), modified SOPAS-startup sequence.

### v2.4.4 - 2022-01-25
  - **Added** Support min and max angle configuration for LRS-3601
  - **Changed** Mirroring for NAV-3xx

### v2.4.3 - 2022-01-18
  - **Changed** Rename class sick_lidar::Util to namespace sick_lidar::util
  - **Fixed** FREchoFilter bug for LD-LRS36xx
  - **Added** Support of TiM240
  - **Added** Automatic switch to specified SOPAS mode (binary vs. ASCII) during startup

## Releases previously logged in CHANGELOG.rst ##

### v1.10.1 (2021-03-18)
* Update ipconfig.md
* Update ipconfig.md
* Update ipconfig.md
* Contributors: Michael Lehning

### v1.7.8 (2020-09-02)
* fixes `#100 <https://github.com/SICKAG/sick_scan/issues/100>`_
* Update software_pll.md
* software pll information added
* Update angular_compensation.md
* angle compensator
* compensation example plot updated
* angle compensation fixed for NAV2xx
* sizt_t warning reduced, bugfix for result flag by changing ip address
* network comp. to windows
* pcl dependency modified
* Contributors: Michael Lehning

### v1.6.0 (2020-05-14)
* NAV 210+NAV245 support added code reformated
* NAV310 added
* Contributors: Michael Lehning

### v1.4.2 (2019-11-14)
* fixed timing issues with MRS6124
* added launch info for lms4xxx
* added LMS 4xxx support
* tim_7xxS dependencys included
* Adding info for 7xxS-Launch-file
* safety scanner added
* added dependency for thrusty
* added information about TIM 7xx launch
* IMU Support, scan freq. and angle. resolution settings added
* TiM7xx integrated
* typical startup sequence
* added lms1xx hires mode
* added support for high ang. resolution for LMS 1xx
* added pointcloud chopping
* Issue resolve handling added
* Pointcloud splitting prepared
* added timing documentation
* cartographer support improved
* improved IMU support
* Update google_cartographer.md
* added Networktiming PLL
* improved performance, start of tim7xx integration
* Contributors: Michael Lehning

### v0.0.16 (2019-02-14)
* Update README.md
* Improved performance

### v0.0.15 (2019-02-05)
* Update README.md
* Support for Ubuntu Trusty `#001 <https://github.com/SICKAG/sick_scan/issues/001>`
* ip v4 parsing changed due to support of older linux version
* Contributors: Michael Lehning, Unknown

### v0.0.14 (2019-01-31)
* Merge branch 'devel'
* ip address setting support, improved Debug MSG
* Updated MRS6xxx launchfile
* getting diagrams otimized for MRS6124
* Warning option as comment added
* compilation fixes for uninitialized variables and no return functions
* writing ip address to eeprom prepared
* improved imu support
* added Python script to detect scanners
* Added first implementation of imu support
* IMU message handling prepared
* added Ip arg name
* Updated meshes
* Sample file for launching and rviz-config files
* Added lms1 and lms5 meshes and urdfs for them.  The gazebo sensors might still need work
* Lookup Table for multi echo fixed
* Test tool integrated into CMakeLists.txt
* Build receipt for sensor_alighment
* Fix for startup procedure to enable automatic  SOPAS ascii to SOPAS bin.
* stopScanData introduced, init flag introduced, signal handler introduced
  change start process to state machine
* radar_object_marker launch file updated
* Radar Simulation optimized
* Parsing of  PreHeader fixed and simulation optimized
  Raw target added for simulatoin
* RMS3xx documentation
* Preheading Parsing optimized
* Radar preheader parsing extended
* Radar datagram explanation
* Only first echo for MRS6124 as default to reduct data volume
* radar visualization optimized
* marker optimized
* clean of of radar_object_marker
* support hector slam
* SLAM-Support documentation
* hector slam support
* initial radar documentation added
* cleanup test program
* test launch file added to show pointcloud2 AND scans for the MRS1xxx
* timestamp of radar msg. improved, pointcloud2 debug messages for raw target and object targets added
* launch file for rosbag testing added
* Launch file for combination of laser scanner and radar added
* PCL converter ignores missing intensity values
* point cloud2image filter added, timestamping optimized
* Device Identiier handling opimized for MRS1xxx and LMS1xxx
* test files added
* omitting of laserscan frameid fixed
* debug messages removed from test script
* generation of test launch file without starting the test can be controlled by
  using setting flag entry launch_only to true.
* Switching of radar properties improved
* Tracking method and output selection for radar
* Test application for using min/max-interval checking
  and added more test parameter
* support for rms3xx prepared
* Copyright added
* licensed under apache 2.0
* file based simulation based on file name pattern added and evaluated.
* patches for ubuntu
* pointcloud2 prepared
* Parsing and test driven development optimized
* Simulation for objects added
* support of radar simulation
* Contributors: Dave Niewinski, Michael Lehning, Sai Kishor Kothakota, Unknown, unknown

### v0.0.13 (2018-05-02)
* moved some cpp files to ensure Debian compatibility
* Contributors: Unknown

### v0.0.12 (2018-04-25)
* Added script to start all test sequentially
* Added RSSi and Range Deviation Test to sick_scan_test
* channel handling for 8 bit rssi values corrected
* Defines for param keyword introduced
* added ros param for rssi data size 16 or 8 Bit
* added rssi resolution configswitch
* support for LMS_5xx and LMS_1xx added
* testprogramm can now handle comments;
* Test instructions added
* Generation of result file
* inital test revisited
* Initial version protocol tester
* Tiny XML Parser added
* added Sopas protocol param
* Added Tools and driver folder, removed unnecessary libusb dep.
* Added scanner_type to parameter set to allow the processing of parallel scanners
* timeout handling improved
* reading thread times after connection lost
  Timeout settings optimized
* protocol switching supported
* Protocol switching implemented
* added timeout and binary/ascii detection
* Support of LMS1104 debugged, skipping scan mgs. publish for MRS6124 (only pointcloud)
* Adding MRS6124 link to supported scanner table
  Edited trouble shooting
* Add documentation for network stack
* scandataCfg for binary commands prepared
* min_ang, max_ang adapted for MRS6xxx
* LMS1000 support continue, Bug fix for parsing distance value MRS6xxx, mrs6xxx.launch modified
* COLA_A and COLA_B prepared
* Package handling optimized (for asynchron tcp data transfer)
* Debug info added for receiving tcp packets
* Support of MRS1104
* Cleanup and supporting Tim571
* errorhandler added
* First version with 9413 bytes packet
* tcp handling optimized
* Queue introduced
* colaa+colab libs included
* Parsing of MRS6xxx-data packages integrated
* Timeout incremented due to startup wait phase for MRS6xxx
* Sleep duration between inital commands changed from 2.0 to 0.2
* Sleep of 10 Sec. introducted after start scandata to ensure that the scanner comes up.
