# Changelog #

Possible types are **Added** for new features. **Changed** for changes to the existing functionality. **Deprecated** for
features that will be removed in future versions **Removed** for deprecated features that were removed in this release.
**Fixed** for all bug fixes. **Security** to prompt users to update in case of closed vulnerabilities.

## Unreleased ##

## Released ##

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
  - **Changed** Rename class sick_lidar3d::Util to namespace sick_lidar3d::util
  - **Fixed** FREchoFilter bug for LD-LRS36xx
  - **Added** Support of TiM240
  - **Added** Automatic switch to specified SOPAS mode (binary vs. ASCII) during startup

  
