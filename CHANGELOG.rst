^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sick_scan
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.1 (2021-03-18)
-------------------
* Update ipconfig.md
* Update ipconfig.md
* Update ipconfig.md
* Contributors: Michael Lehning

1.7.8 (2020-09-02)
------------------
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

1.6.0 (2020-05-14)
------------------
* NAV 210+NAV245 support added code reformated
* NAV310 added
* Contributors: Michael Lehning

1.4.2 (2019-11-14)
------------------

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

0.0.16 (2019-02-14)
-------------------
* Update README.md
* Improved performance

0.0.15 (2019-02-05)
-------------------
* Update README.md
* Support for Ubuntu Trusty `#001 <https://github.com/SICKAG/sick_scan/issues/001>`
* ip v4 parsing changed due to support of older linux version
* Contributors: Michael Lehning, Unknown

0.0.14 (2019-01-31)
-------------------
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

0.0.13 (2018-05-02)
-------------------
* moved some cpp files to ensure Debian compatibility
* Contributors: Unknown

0.0.12 (2018-04-25)
-------------------
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
  Sleep of 10 Sec. introducted after start scandata to ensure that the scanner comes up.
0.0.11 (2018-01-24)
-------------------
