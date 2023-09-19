## Supported SICK Devices

This driver works with all of the following products.

| Type                | Part no.                                                                                                                           |
| ------------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| MRS6124             | [6065086](https://www.sick.com/de/en/detection-and-ranging-solutions/3d-lidar-sensors/mrs6000/c/g448151)                           |
| MRS1000             | [e.g. 1081208](https://www.sick.com/sg/en/detection-and-ranging-solutions/3d-lidar-sensors/mrs1000/mrs1104c-111011/p/p495044)      |
| TiM2xx              | [1104981](https://www.sick.com/ag/en/detection-and-ranging-solutions/2d-lidar-sensors/tim2xx/tim240-2050300/p/p654443)             |
| TiM5xx              | [e.g. 1060445](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/tim5xx/tim551-2050001/p/p343045)        |
| TiM7xxS             | [e.g. 1105052](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/tim7xx/tim771s-2174104/p/p660929)         |
| TiM7xx              | [e.g 1096807](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/tim7xx/tim781-2174101/p/p594148)           |
| LMS5xx              | [e.g. 1046135](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/lms5xx/c/g179651)                       |
| LMS1000             | [1092445](https://www.sick.com/ag/en/detection-and-ranging-solutions/2d-lidar-sensors/lms1000/c/g387151)                           |
| LMS1xx              | [e.g. 1041114](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/lms1xx/c/g91901)                        |
| LMS4000             | [e.g. 1091423](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/lms4000/lms4111r-13000/p/p578044?ff_data) |
| LDMRS               | [e.g. 1115128](https://www.sick.com/de/de/p/p662073)                                                                               |
| LD-LRS3600          | [1060831](https://www.sick.com/no/en/lidar-sensors/2d-lidar-sensors/ld-lrs/ld-lrs3600/p/p362656)                                   |
| LD-LRS3601          | [1060832](https://www.sick.com/no/en/lidar-sensors/2d-lidar-sensors/ld-lrs/ld-lrs3601/p/p362657)                                   |
| LD-LRS3611          | [1067186](https://www.sick.com/no/en/lidar-sensors/2d-lidar-sensors/ld-lrs/ld-lrs3611/p/p362658)                                   |
| LD-OEM1500          | [1060828](https://www.sick.com/no/en/lidar-sensors/2d-lidar-sensors/ld-oem/ld-oem1500/p/p362654)                                   |
| LD-OEM1501          | [1060829](https://www.sick.com/no/en/lidar-sensors/2d-lidar-sensors/ld-oem/ld-oem1501/p/p362655)                                   |
| LRS4000             | [1098855](https://www.sick.com/no/en/detection-and-ranging-solutions/2d-lidar-sensors/lrs4000/c/g555594)                           |
| NAV310              | [e.g. 1060834](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/nav3xx/nav310-3211/p/p349345)             |
| NAV210+NAV245       | [e.g. 1074308](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/nav2xx/c/g356151)                         |
| RMS1000             | [1107598](https://www.sick.com/de/en/detection-and-ranging-solutions/radar-sensors/rms1000/rms1731c-636111/p/p660833)              |
| RMS2000             | BETA                                                                                                                               |
| multiScan100        | [e.g. 1131164](https://www.sick.com/1131164)                                                                                       |
| picoScan100         | [e.g. 1134610](https://www.sick.com/1134610)                                                                                       |

Note:
* LDMRS family is currently not supported on Windows.
* ROS services require installation of ROS-1 or ROS-2, i.e. services for Cola commands are currently not supported on native Linux or native Windows.
* ROS services are currently not available for LDMRS.
* dynamic reconfiguration of sick_scan_xd parameter is supported on ROS-1 or ROS-2 only, neither under Linux nor under Windows.
* Publishing pointcloud data requires ROS-1 or ROS-2. On native Linux resp. native Windows, pointcloud data are currently saved to jpg- and csv-files for demonstration purposes.

## Supported platforms

The driver is developed and tested for x86 architecture. Use for system with ARM architecture (e.g. Raspberry platform) is not officially supported.

## Supported interfaces

Only Ethernet-IPv4-based communication with the sensor is supported by this driver.