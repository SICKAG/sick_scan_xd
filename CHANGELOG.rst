^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sick_scan_xd
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.8.0 (2025-09-17)
------------------
* Release v3.8.0
  * add: Extra frameid for IMU message, #471
  * add: ROS2 kilted support
  * update: Fix code block indentation in README
  * update: README — add layer suffixes explanation and general improvements
  * update: Optimized CMakeLists.txt (ROS2 distros as list)
  * fix: Relaxed required field test
  * fix: ASCII parsing for TiM240, #478

3.7.0 (2025-05-28)
------------------
* Release v3.7.0
  * fix: TiM7xx fieldset settings and services, #394, #408
  * update: README.md and cmake requirements
  * change: Publish lferec messages latched, #420
  * fix: TCP connection timeout, #424
  * add: Support for LRS-4xxx IMU and contamination data, #418
  * update: Improved TiM-7xx field evaluation status after start, #420
  * update: Dockertests for ROS-1, ROS-2 and API
  * fix: Timestamp handling (corrected system time estimation from transmit vs. generation timestamps), #428
  * fix: Disable UDP-Parsing during reinitialisation
  * fix: ParameterAlreadyDeclaredException on reinitialisation
  * add: LaserScan messages encode reflector detection through high intensity values
  * update: user level and user password can be freely specified in the launch file. For picoScan, multiScan and LRS4xxx, the default user level is 4.

3.6.0 (2024-10-23)
------------------
* Release v3.6.0
  * add: Launchfiles and configuration for picoScan120
  * add: Optional AngleRangeFilter and IntervalFilter for picoScan
  * fix: Obsolete topic "/sick_multiscan/scan" removed
  * add: IMU automatically deactivated after receiving an error code for IMU activation from picoScan w/o addons
  * fix: Customization of hash values for authorization #366
  * fix: Replaced builtin_addressof expressions #370
  * add: Different UDP timeouts for state initial and running, improved UDP timeout handling
  * fix: Picoscan range_min value in laserScan message #382
  * add: Support for RMS2xxx LIDoutputstate telegrams
  * fix: sick_generic_caller debug assertion #385
  * add: Check of udp receiver ip at startup
  * add: cmake-option to overwrite optimization level
  * change: Documentation restructured
  * add: Improved field evaluation TiM7xx, Tim7xxS (publish LIDinputstate messages, configuration and services for options FieldSetSelectionMethod and ActiveFieldSet)
  * add: PicoScan parameter add_transform_xyz_rpy #399
  * fix: LMS4000 encoder settings #403
  * fix: CMake-flag for target sick_scan_xd_api_dockertest #404
  * change: Merge PR #405 (typo) and PR #406 (sick_scan_xd_api_test)

3.5.0 (2024-07-19)
------------------
* Release v3.5.0
  * add: Optional switch between "system timestamps" (default) and "tick timestamps"
  * add: Extract sick_scan_xd version from package.xml
  * add: Add git info to sick_scan_xd version
  * fix: #316 (blocking API-close in case of wrong ip-address)
  * add: #321, #310 (optional cmake-flag to set a calling convention for API calls)
  * add: Hector SLAM configuration and example for picoScan
  * add: Correction angle shift settings for LRS-36x0 and OEM-15xx
  * add: Update FAQ (Network settings etc.)
  * add: Integration PR #347 (NAV350 landmarks)
  * add: Support for 3D-SLAM (OctoMap, RTAB-Map, timestamps for each scanpoint for deskewing, configuration example for multiScan)
  * add: customisations for gcc 13 (Ubuntu 24)
  * add: Update for bloom releases ROS-2 iron and jazzy
  * add: Configuration multiScan interval filter via launchfile (#352)
  * add: SPDX licence text (PR #354)
  * fix: Fix #355 (API launchfile parsing error)
  * add: Example configurations for parallel operation multi and picoScan

3.4.0 (2024-04-18)
------------------
* Release v3.4.0
  * add: azimut angle table for MRS-1xxx and LMS-1xxx with firmware 2.2.0 oder newer
  * add: dockertests for MRS-1xxx, multiScan and picoScan with ROS-2
  * add: API-funktion SickScanApiSendSOPAS to send SOPAS commands (e.g. "sRN SCdevicestate" or "sRN ContaminationResult")
  * add: generation of TF messages
  * add: Option to deactivate initialization sequence for TiM-7xxS devices
  * add: Documented option "-b master"  to clone the release version
  * fix: #316 API re-init nach close

3.3.0 (2024-03-04)
------------------
* Release v3.3.0
  * add: Option for MRS1xxx azimuth correction table
  * add: Support for picoScan100 (single echo w/o addons)
  * add: API logging functions and verbosity (#270)
  * add: API documentation (multiple lidars not supported, #281)
  * changed: API extended (added topic in PointCloud messages, #271)

3.2.0 (2024-02-12)
------------------
* Release v3.2.0
  * add: IMU support for multiScan and picoScan
  * add: support bloom releases for ROS-2 humble
  * add: docker tests for ROS1 noetic incl. testcases for multiScan, picoScan, MRS1xxx
  * add: PR #255, support picoScan performance profiles
  * change: configuration time flag LMDscandatacfg switched off for the TiM240
  * fix: #218 (API reinit)
  * fix: #220 (Fullframe Laserscan messages multi- and picoScan)
  * fix: #221 (No scandata while activated scan range filter)
  * fix: #222 (rviz visualization of polar pointclouds)
  * fix: #247 (Launchfile option for laserscan topic)
  * fix: #256 (Clean API-exit picoScan and multiScan)
  * fix: #260 (Provide API-functions to query lidar status, error codes and error messages)
* Contributors: Manuel Aiple

3.1.0 (2023-11-21)
------------------
* Release v3.1.0
  * add: IMU support for multiScan
  * add: LaserScan output for picoScan
  * fix: API reinit
  * fix: multiScan data output with range filter activated
  * fix: adapt multiScan startup and shutdown sequence
  * fix: adaptations for MRS-1000 v2 firmware
* Contributors: Manuel Aiple

3.0.0 (2023-10-11)
------------------
* Release v3.0.0
  * add: New topics for additional data and properties
  * add: Official ARM64 support
  * add: ROS module name consistency
  * add: Service for requesting contamination detection information
  * add: Improvements from customer tickets and documentation enhancements
* Contributors: Manuel Aiple

2.10.2 (2023-09-01)
-------------------
* Release v2.10.2
  - correct picoScan details in documentation and launch files
* Release v2.10.1
  * make Compact format the default for picoScan and multiScan
* Release v2.10.0
  * add picoScan support
  * add Compact format support
  * add LMS4000 encoder setting support
  * fix multiScan angle range filter parameter unit
  * fix MRS1104 diagnostic message
  * fix TiM781S login
* chore: add note on supported interfaces
* Merge branch 'master' into develop
* Merge branch 'master' into develop
* Merge branch 'master' into develop
* Release v2.9.0 - RMSxxxx support and NAV350 support
* Merge feature/RMSxxxx into develop
* chore: revert picoScan references
* NAV350 support integrated
* chore: change mrs100 occurences into multiScan
* Correct cmake option handling and adapt documentation
* Merge branch 'master' into develop
  # Conflicts:
  # driver/src/sick_scan_common.cpp
* Feature/lrs3601 angle settings
* Documentation updated (regarding comments from Silas)
* fix: address points raised by catkin_lint
* Update README.md
* Update REQUIREMENTS.md
* Update README.md
* chore: Update driver state diagram
* Contributors: Manuel Aiple, Michael Lehning, Silas Gschwender

2.9.1 (2023-05-15)
------------------
* Release v2.9.1 - Angle correction
  * Angle correction (min/max angle settings), fix TIM551 points have bad coordinates. `#166 <https://github.com/SICKAG/sick_scan_xd/issues/166>`_
  * TiM240 initialization (start measurement)
* CHANGELOG.md updated
* Merge branch 'feature/interlace_doc' into feature/angle_correction
* Update angle correction `#166 <https://github.com/SICKAG/sick_scan_xd/issues/166>`_
* cone_section moved and interlacing mode explanation extended
* Remarks about interlacing and curved lines on a flat plane
* Contributors: Manuel Aiple, Michael Lehning, rostest

2.9.0 (2023-05-05)
------------------
* Release v2.9.0 - RMSxxxx support and NAV350 support
* Contributors: Manuel Aiple

2.8.15 (2023-02-10)
-------------------
* V.2.8.15 release:
  * **Update** Win64 build instructions
  * **Update** API documentation, driver states diagrams and typos
  * **Added** LRS-36xx configuration for upside-down mounting
  * **Removed** Obsolete service commands and RMS3xx support
  * **Fix** LRS-36xx angle configuration
  * **Fix** catkin_lint warnings
  * **Fix** ROS-2 Humble build
* Update REQUIREMENTS.md
* Contributors: Michael Lehning, Silas Gschwender

2.8.14 (2022-11-22 10:53)
-------------------------
* Merge branch 'feature/multiscan136_laserscan_msg'
* V2.8.14 Laserscan messages for Multiscan
  Update: Laserscan messages for Multiscan lidar, `#96 <https://github.com/SICKAG/sick_scan_xd/issues/96>`_
* Contributors: rostest

2.8.13 (2022-11-22 10:47)
-------------------------
* Merge branch 'feature/transform_update'
* V2.8.13 - Dynamical pointcloud transform and QoS configuration
  Update: Dynamical configuration of an additional pointcloud transform by rosparam, `#104 <https://github.com/SICKAG/sick_scan_xd/issues/104>`_
  Update: Configuration of ROS quality of service by launchfile, `#101 <https://github.com/SICKAG/sick_scan_xd/issues/101>`_
* Contributors: rostest

2.8.11 (2022-11-10)
-------------------
* Merge pull request `#125 <https://github.com/SICKAG/sick_scan_xd/issues/125>`_ from SICKAG/feature/lms1xxx-support
  LMS 1xxx support
* LMS 1xxx support
  LMS 1xxx support with scan configuration (scan frequency and angular resolution for firmware 2.x)
* CHANGELOG.md corrected
* Merge branch 'master' of https://github.com/SICKAG/sick_scan_xd
* radar ascii support and elevation layer fix
* Contributors: Michael Lehning, rostest

2.8.9 (2022-10-13)
------------------
* Merge pull request `#118 <https://github.com/SICKAG/sick_scan_xd/issues/118>`_ from SICKAG/feature/mrs_1xxx-layer-conversion
  mrs 1xxx slam support optimized
* mrs1xxx slam support optimized
* Contributors: Michael Lehning

2.8.8 (2022-10-06 13:47)
------------------------
* optimized ASCII rms1xxx support
* galatic support added
* Contributors: Michael Lehning

2.8.7 (2022-10-06 12:10)
------------------------
* v2.8.7 moved to release section
* Merge pull request `#116 <https://github.com/SICKAG/sick_scan_xd/issues/116>`_ from SICKAG/115-minmax-range-settings
  range filter settings optimized
* range filter settings optimized
* Update CMakeLists.txt
* Merge pull request `#113 <https://github.com/SICKAG/sick_scan_xd/issues/113>`_ from tomaszkapron/galactic
  fix: galactic build support
* fix: galactic build support
* Contributors: Michael Lehning, tomaszkapron

2.8.6 (2022-09-29)
------------------
* Release 2.8.6, Multiscan update for 16-bit RSSI and modified SOPAS startup sequence
* fix #pr111 merge
* Merge pull request `#111 <https://github.com/SICKAG/sick_scan_xd/issues/111>`_ from fmessmer/fix/cmake_if_conditions
  Fix/cmake if conditions
* consistently check for EQUAL ON
* fix message
* Additional description for cmake options
* Merge pull request `#110 <https://github.com/SICKAG/sick_scan_xd/issues/110>`_ from fmessmer/fix/cmake_messages
  fix message for options in CMakeLists
* fix message for options in CMakeLists
* Folder test/emulator/scandata added
  Empty folder test/emulator/scandata required for builds with emulator enabled
* Contributors: fmessmer, rostest

2.8.5 (2022-09-20)
------------------
* Merge pull request `#109 <https://github.com/SICKAG/sick_scan_xd/issues/109>`_ from SICKAG/feature/lrs4xxx-support-glare-detection
  V2.8.5 - LRS4xxx support glare detection filter
* V2.8.5 - LRS4xxx support glare detection filter
* Contributors: rostest

2.8.4 (2022-09-19)
------------------
* Update CHANGELOG.md
* Merge pull request `#107 <https://github.com/SICKAG/sick_scan_xd/issues/107>`_ from SICKAG/106-support-of-generic-api
  generic api support incl. test data and documentation
* generic api support incl. test data and documentation
* Additional API explanations
* Contributors: Michael Lehning, rostest

2.8.3 (2022-09-05)
------------------
* V2.8.3 - Generic API
  See CHANGELOG.md
* Contributors: rostest

2.7.5 (2022-08-01)
------------------
* Merge pull request `#93 <https://github.com/SICKAG/sick_scan_xd/issues/93>`_ from SICKAG/feature/mrs6124-fix
  Feature/mrs6124 fix
* Removed visibility linker flag
* angle offset MRS6124
* Merge pull request `#92 <https://github.com/SICKAG/sick_scan_xd/issues/92>`_ from SICKAG/feature/library-linking
  Library linker flag `#91 <https://github.com/SICKAG/sick_scan_xd/issues/91>`_
* Library linker flag `#91 <https://github.com/SICKAG/sick_scan_xd/issues/91>`_
* Bugfix MRS6124 pointcloud
* Contributors: rostest

2.7.4 (2022-06-30)
------------------
* Merge pull request `#84 <https://github.com/SICKAG/sick_scan_xd/issues/84>`_ from SICKAG/feature/ros2-compilation-error-83
  ROS2 compilation error `#83 <https://github.com/SICKAG/sick_scan_xd/issues/83>`_
* ROS2 compilation error `#83 <https://github.com/SICKAG/sick_scan_xd/issues/83>`_
* Contributors: rostest

2.7.3 (2022-06-29)
------------------
* Merge pull request `#82 <https://github.com/SICKAG/sick_scan_xd/issues/82>`_ from SICKAG/feature/scale_mean_median_filter
  Support for LFPmeanfilter and LFPmedianfilter (MRS1xxx, LMS1xxx, LMS4xxx, LRS4xxx) and LMDscandatascalefactor (LRS4xxx)
* Support for LFPmeanfilter, LFPmedianfilter, LMDscandatascalefactor
  Support for LFPmeanfilter and LFPmedianfilter (MRS1xxx, LMS1xxx, LMS4xxx, LRS4xxx) and LMDscandatascalefactor (LRS4xxx)
* Contributors: rostest

2.7.0 (2022-06-27)
------------------
* Merge pull request `#81 <https://github.com/SICKAG/sick_scan_xd/issues/81>`_ from SICKAG/feature/multiscan136-support
  V2.7.0 Integration of sick_scansegment_xd (multiscan136 support) `#80 <https://github.com/SICKAG/sick_scan_xd/issues/80>`_
* V2.7.0, Integration of sick_scan_segment_xd (multiscan136 support) `#80 <https://github.com/SICKAG/sick_scan_xd/issues/80>`_
* Merge pull request `#72 <https://github.com/SICKAG/sick_scan_xd/issues/72>`_ from fmessmer/feature/launch_args_lrs_4xxx
  add launch arguments for sick_lrs_4xxx.launch
* add launch arguments for sick_lrs_4xxx.launch
* Contributors: mojin@backpack-1, rostest

2.6.8 (2022-06-20)
------------------
* V2.6.8 Merge pull request `#76 <https://github.com/SICKAG/sick_scan_xd/issues/76>`_
* Merge pull request `#76 <https://github.com/SICKAG/sick_scan_xd/issues/76>`_ from youliangtan/master
  fix ros2 ros_info compilation
* fix ros2 ros_info compilation
* Merge pull request `#75 <https://github.com/SICKAG/sick_scan_xd/issues/75>`_ from SICKAG/70-lms511-problems-with-setting-the-start-and-end-angle-to-limit-the-data-output-during-scan-output
  Fixing problems of setting min-/max-angle for LMS511
* Fixing problems of setting min-/max-angle for LMS511
* Contributors: Michael Lehning, rostest, youliang

2.6.7 (2022-05-26)
------------------
* Merge pull request `#68 <https://github.com/SICKAG/sick_scan_xd/issues/68>`_ from SICKAG/feature/lms511-min_max_angles
* Merge pull request `#68 <https://github.com/SICKAG/sick_scan_xd/issues/68>`_ from SICKAG/feature/lms511-min_max_angles
  LMS511 configuration `#67 <https://github.com/SICKAG/sick_scan_xd/issues/67>`_
* LMS511 configuration `#67 <https://github.com/SICKAG/sick_scan_xd/issues/67>`_
* Merge pull request `#66 <https://github.com/SICKAG/sick_scan_xd/issues/66>`_ from SICKAG/feature/readme-update
  Update README.md
* Update README.md
* Merge pull request `#64 <https://github.com/SICKAG/sick_scan_xd/issues/64>`_ from SICKAG/feature/nav310_lrs4000_support
* Contributors: rostest

2.6.6 (2022-05-23)
------------------
* Release v2.6.6: NAV310 + LRS4xxx update, issues `#58 <https://github.com/SICKAG/sick_scan_xd/issues/58>`_, `#59 <https://github.com/SICKAG/sick_scan_xd/issues/59>`_, `#60 <https://github.com/SICKAG/sick_scan_xd/issues/60>`_, `#61 <https://github.com/SICKAG/sick_scan_xd/issues/61>`_
* Merge pull request `#64 <https://github.com/SICKAG/sick_scan_xd/issues/64>`_ from SICKAG/feature/nav310_lrs4000_support
  NAV310 + LRS4xxx update, issues `#58 <https://github.com/SICKAG/sick_scan_xd/issues/58>`_, `#59 <https://github.com/SICKAG/sick_scan_xd/issues/59>`_, `#60 <https://github.com/SICKAG/sick_scan_xd/issues/60>`_, `#61 <https://github.com/SICKAG/sick_scan_xd/issues/61>`_
  `#58 <https://github.com/SICKAG/sick_scan_xd/issues/58>`_ (NAV310): min/max angle removed from config
  `#59 <https://github.com/SICKAG/sick_scan_xd/issues/59>`_ (NAV310+LRS4xxx): laserscan and pointcloud identical
  `#60 <https://github.com/SICKAG/sick_scan_xd/issues/60>`_ (LRS4xxx): validated parameter scan_cfg_list_entry and skip
  `#61 <https://github.com/SICKAG/sick_scan_xd/issues/61>`_ (LRS4xxx): default value echo filter changed to "2" (last echo)
* NAV310 + LRS4xxx update, issues `#58 <https://github.com/SICKAG/sick_scan_xd/issues/58>`_, `#59 <https://github.com/SICKAG/sick_scan_xd/issues/59>`_, `#60 <https://github.com/SICKAG/sick_scan_xd/issues/60>`_, `#61 <https://github.com/SICKAG/sick_scan_xd/issues/61>`_
  `#58 <https://github.com/SICKAG/sick_scan_xd/issues/58>`_ (NAV310): min/max angle removed from config
  `#59 <https://github.com/SICKAG/sick_scan_xd/issues/59>`_ (NAV310+LRS4xxx): laserscan and pointcloud identical
  `#60 <https://github.com/SICKAG/sick_scan_xd/issues/60>`_ (LRS4xxx): validated parameter scan_cfg_list_entry and skip
  `#61 <https://github.com/SICKAG/sick_scan_xd/issues/61>`_ (LRS4xxx): default value echo filter changed to "2" (last echo)
* Merge pull request `#57 <https://github.com/SICKAG/sick_scan_xd/issues/57>`_ from SICKAG/feature/lrs-4xxx-support-parameter-skip
  Parameter skip added in lrs4xxx-launchfile `#56 <https://github.com/SICKAG/sick_scan_xd/issues/56>`_
* Parameter skip added in lrs4xxx-launchfile
* Contributors: rostest

2.6.5 (2022-05-10)
------------------
* Merge pull request `#53 <https://github.com/SICKAG/sick_scan_xd/issues/53>`_ from SICKAG/feature/lrs4xxx-configuration
  LRS4xxx scan configuration `#52 <https://github.com/SICKAG/sick_scan_xd/issues/52>`_
* LRS4xxx scan configuration `#52 <https://github.com/SICKAG/sick_scan_xd/issues/52>`_
* Contributors: rostest

2.6.4 (2022-05-09)
------------------
* Merge pull request `#51 <https://github.com/SICKAG/sick_scan_xd/issues/51>`_ from SICKAG/feature/lms511-echofilter
  Feature/lms511 echofilter
* LMS5xx echo filter settings corrected
* Remove emulator test sequences
* Contributors: rostest

2.6.3 (2022-05-04)
------------------
* Merge pull request `#48 <https://github.com/SICKAG/sick_scan_xd/issues/48>`_ from SICKAG/feature/timestamp-laserscan-message
  Timestamp Laserscan message corrected `#47 <https://github.com/SICKAG/sick_scan_xd/issues/47>`_
* Timestamp Laserscan message corrected `#47 <https://github.com/SICKAG/sick_scan_xd/issues/47>`_
  Timestamp of pointcloud and laserscan messages identical and computed from lidar ticks by software-pll
* Contributors: rostest

2.6.2 (2022-04-28)
------------------
* V2.6.2 LDMRS spinning problem
* Merge pull request `#45 <https://github.com/SICKAG/sick_scan_xd/issues/45>`_ from SICKAG/feature/ldmrs_spinning_problem
  LDMRS spinning problem corrected `#44 <https://github.com/SICKAG/sick_scan_xd/issues/44>`_
* LDMRS spinning problem corrected `#44 <https://github.com/SICKAG/sick_scan_xd/issues/44>`_
* Merge pull request `#43 <https://github.com/SICKAG/sick_scan_xd/issues/43>`_ from SICKAG/feature/status_update
  Update driver status `#42 <https://github.com/SICKAG/sick_scan_xd/issues/42>`_
* Update driver status
* Merge pull request `#41 <https://github.com/SICKAG/sick_scan_xd/issues/41>`_ from SICKAG/feature/LMS1xx_setscancfg
  Bugfix LMS1xx mLMPsetscancfg `#39 <https://github.com/SICKAG/sick_scan_xd/issues/39>`_
* Merge pull request `#40 <https://github.com/SICKAG/sick_scan_xd/issues/40>`_ from Pattern-Labs/feat/MinorImprovements
  feat/MinorImprovements
* Fixing logging for increments. Allowing nodename to be externally customized to allow multiple concurrent nodes.
* Bugfix LMS1xx mLMPsetscancfg `#39 <https://github.com/SICKAG/sick_scan_xd/issues/39>`_
  Bugfix for LMS1xx error at startup (settting mLMPsetscancfg, `#39 <https://github.com/SICKAG/sick_scan_xd/issues/39>`_)
* Merge pull request `#38 <https://github.com/SICKAG/sick_scan_xd/issues/38>`_ from SICKAG/feature/rms_support
  Update RMS support `#37 <https://github.com/SICKAG/sick_scan_xd/issues/37>`_ (configuration, documentation)
* Update RMS support `#37 <https://github.com/SICKAG/sick_scan_xd/issues/37>`_ (configuration, documentation)
* Contributors: John Pratt, rostest

2.6.1 (2022-04-04)
------------------
* V2.6.1: Support for RMS-1xxx binary protocol
* Merge pull request `#33 <https://github.com/SICKAG/sick_scan_xd/issues/33>`_ from scheunemann/master
  Inconsistent use of "MRS" and "TIM" in example urdf
* fix example urdf
* Contributors: Marcus Scheunemann, rostest

2.6.0 (2022-03-30)
------------------
* RMS configuration update,
* Contributors: rostest

2.5.2 (2022-03-22)
------------------
* Merge pull request `#30 <https://github.com/SICKAG/sick_scan_xd/issues/30>`_ from SICKAG/feature/lrs_4xxx_angles
  Feature/lrs 4xxx angles
  * Fix LSR-4xxx laserscan angles `#28 <https://github.com/SICKAG/sick_scan_xd/issues/28>`_
  * Fix duplicated laserscan messages `#28 <https://github.com/SICKAG/sick_scan_xd/issues/28>`_
* Fix `#28 <https://github.com/SICKAG/sick_scan_xd/issues/28>`_ (duplicated laserscan messages)
* Fix LSR-4xxx laserscan angles
* Contributors: rostest

2.5.1 (2022-03-16)
------------------
* Merge pull request `#29 <https://github.com/SICKAG/sick_scan_xd/issues/29>`_ from SICKAG/feature/lidar_concurrent_event_loops
  Error after SOPAS command SetAccessMode `#27 <https://github.com/SICKAG/sick_scan_xd/issues/27>`_
* Error after SOPAS command SetAccessMode `#27 <https://github.com/SICKAG/sick_scan_xd/issues/27>`_
* Contributors: rostest

2.5.0 (2022-03-09)
------------------
* Merge pull request `#25 <https://github.com/SICKAG/sick_scan_xd/issues/25>`_ from SICKAG/feature/lidar_stop_exit
* Fix issue `#24 <https://github.com/SICKAG/sick_scan_xd/issues/24>`_ (stop scanner at exit), new ros service SickScanExit to stop scanner and exit
* Contributors: rostest

2.4.6 (2022-03-03)
------------------
* Corrected angle shift parameter for LMS-4xxx
  Corrected angle shift parameter for LMS-4xxx, Typo corrected
* Contributors: rostest

2.4.5 (2022-02-28)
------------------
* Issues `#158 <https://github.com/SICKAG/sick_scan_xd/issues/158>`_ (driver terminates), `#22 <https://github.com/SICKAG/sick_scan_xd/issues/22>`_ (build error diagnostic_updater), `#21 <https://github.com/SICKAG/sick_scan_xd/issues/21>`_ (python launch files)
* Update field_monitoring_extensions.md
  Typo fixing for lidar name
* IMU enabled in MRS-1xxx launchfile
* Merge pull request `#19 <https://github.com/SICKAG/sick_scan_xd/issues/19>`_ from JWhitleyWork/fix-ros2-args-parsing
  Fix command-line parsing in ROS2. Thanks to @JWhitleyWork !
* Fix command-line parsing in ROS2.
* README and FAQ updated (link to changelog, launch-file customization)
* Contributors: Joshua Whitley, Michael Lehning, rostest

2.4.4 (2022-01-25)
------------------
* V2.4.4: configuration of start/stop angles for LRS-36x1
* CHANGELOG.md updated
* Contributors: Michael Lehning, rostest

2.4.3 (2022-01-18)
------------------
* V2.4.3: LMS111 support, switch Cola-A/Cola-B
  LMS111 support with 25+50 Hz `#13 <https://github.com/SICKAG/sick_scan_xd/issues/13>`_, optional switch Cola-A / Cola-B after startup `#11 <https://github.com/SICKAG/sick_scan_xd/issues/11>`_
* Added faq howto run muliple sensors concurrently
* Added faq hints about compiler errors
* Contributors: rostest

2.4.2 (2021-12-03)
------------------
* Release 2.4.2: Hardening
  Release 2.4.2: Hardening, Message and pointcloud monitoring, reconnect and -initialization after timeouts, Support for SOPAS-commands SCreboot and SCsoftreset
* Merge pull request `#10 <https://github.com/SICKAG/sick_scan_xd/issues/10>`_ from hatchbed/fix-build-type
  Fix ROS 1 build
* Fix ROS 1 build
  catkin_make was refusing to build this package because it couldn't
  identify the build type, so this explicitly sets the build_type to
  catkin when in a ROS 1 environment.
  Also, there was a header that was defining some values that should
  only be set in ROS 2, and the #if definition around it was accidentally
  checking if the ROS version was >0 rather than >1, so this also fixes
  that.
* ROS1/ROS2-compatibility
* Merge pull request `#2 <https://github.com/SICKAG/sick_scan_xd/issues/2>`_ from hatchbed/consolidate-package-manifests
  Consolidate ROS 1 & ROS 2 package manifests
* Merge pull request `#3 <https://github.com/SICKAG/sick_scan_xd/issues/3>`_ from hatchbed/1/fix-dynamic-reconfig-permissions
  Fix dynamic reconfig permissions
* adding lms_1xx_ros1.rviz file
* fixed mrs 6000 ang offset
* Merge remote-tracking branch 'origin/devel'
* radar info. updated
* added multi echo support for LMS 5xx
* Merge remote-tracking branch 'origin/master' into devel
* finshed LRS 36x0 and LRS 36x1 support
* Update README.md
  fixes `#7 <https://github.com/SICKAG/sick_scan_xd/issues/7>`_
* Fix dynamic reconfig permissions
  The .cfg files used to provide dynamic reconfigure support in ROS 1
  are executable Python files, and so they need to have the executable
  bit set in order to work properly.
  Fixes `#1 <https://github.com/SICKAG/sick_scan_xd/issues/1>`_
* Update README.md
* Consolidate ROS 1 & ROS 2 package manifests
  Previously, this package had separate package manifest files for ROS 1 and ROS 2, and it was necessary to run a script after cloning the repository to name the correct one package.xml.
  This consolidates both of them into a single package.xml file that works with both ROS 1 and ROS 2, so the package can now be cloned and built inside a standard colcon workspace without needing to run any additional scripts.  It also makes a few tweaks to other files to ensure compatibility.
  This has been tested on ROS Noetic and ROS Foxy in Ubuntu 20.04.
* initial support of LD_LRS3600 LD-LRS3601 LD-OEM1501
* starting with LRS_36xx and oem_15xx
* Contributors: Michael Lehning, P. J. Reed, rostest

2.3.0 (2021-10-25)
------------------
* Moved roswrap-headers of ros-generated messages
  Moved roswrap-headers of ros-generated messages to avoid include path dependencies
* RMS1000 info added/modified
* Contributors: Michael Lehning, rostest

2.2.0 (2021-10-18)
------------------
* Merge sick_scan, sick_scan2, sick_scan_base
  Merged repositories sick_scan, sick_scan2, sick_scan_base
* Initial commit
* Contributors: rostest
