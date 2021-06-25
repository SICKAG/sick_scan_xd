# sick_scan_xd

This project provides a driver for the SICK lidar and radar sensors mentioned in the following list.

Based on sick_scan drivers for ROS1, sick_scan_xd merges sick_scan, sick_scan2 and sick_scan_base. It supports both Linux (standard, ROS1, ROS2) and Windows (standard and ROS2).

**CURRENTLY UNDER DEVELOPMENT**

## Table of Contents

- [Supported Hardware](#supported-hardware)
- [Start node](#start-node)
- [Bugs and feature requests](#bugs-and-feature-requests)
- [Tools](#tools)
- [Troubleshooting](#troubleshooting)
- [SLAM-Support](doc/slam.md)
- [Radar](doc/radar.md)
- [Profiling](doc/profiling.md)
- [Testing](#testing)
- [Creators](#creators)

## Supported Hardware

This driver should work with all of the following products.

ROS Device Driver for SICK lidar and radar sensors - supported scanner types:

| **device name**    |  **part no.**                                                                                                                | **description**                                | **tested?**     |
|--------------------|------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------|:---------------:|
| MRS6124            | [6065086](https://www.sick.com/de/en/detection-and-ranging-solutions/3d-lidar-sensors/mrs6000/c/g448151)                         | 24 layer max. range: 200 m, ang. resol. 0.13 [deg] hor., 0.0625 [deg] ver. The SICK MRS6124 is a multi-layer, multi-echo 3D laser scanner that is geared towards rough outdoor environments. | ROS1: ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 10 Hz                       |                 |
| MRS1104            | [1081208](https://www.sick.com/sg/en/detection-and-ranging-solutions/3d-lidar-sensors/mrs1000/mrs1104c-111011/p/p495044)         | 4 layer max. range: 64 m, ang. resol. 0.25 [deg] hor., 2.50 [deg] ver.                                         | ROS1: ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 50 Hz, 4x12.5 Hz            |                 |
| LMS1104            | [1092445](https://www.sick.com/ag/en/detection-and-ranging-solutions/2d-lidar-sensors/lms1000/c/g387151)                         | 1 layer max. range: 64 m, ang. resol. 0.25 [deg] |  ROS1: ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 150 Hz, 4x37.5 Hz   |                 |
| TiM240             | prototype [more info here](doc/tim240/tim240.md) | 1 layer max. range: 10 m, ang. resol. 1.00 [deg], 240 [deg]| ROS1: ✔ [prototype]|
|                    |                                                                                                                                  | Scan-Rate: 14.5 Hz   |                 |
| TiM433             | prototype  | 1 layer range: 0.05 m ... 15 m, ang. resol. 0.33 [deg], 240 [deg]| ROS1: ✔ [prototype]|
|                    |                                                                                                                                  | Scan-Rate: 15.0 Hz   |                 |
| TiM443             | prototype  | 1 layer range: 0.05 m ... 15 m, ang. resol. 0.33 [deg], 240 [deg]| ROS1: ✔ [prototype]|
|                    |                                                                                                                                  | Scan-Rate: 15.0 Hz   |                 |
| TiM551             | [1060445](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/tim5xx/tim551-2050001/p/p343045)                 | 1 layer max. range: 10 m, ang. resol. 1.00[deg] | ROS1: ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| TiM561             | [1071419](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/tim5xx/tim561-2050101/p/p369446)                 | 1 layer max. range: 10 m, ang. resol. 0.33 [deg]| ROS1: ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| TiM571             | [1079742](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/tim5xx/tim571-2050101/p/p412444)                 | 1 layer max. range: 25 m, ang. resol. 0.33 [deg]| ROS1: ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| TiM771S            | [1105052](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/tim7xx/tim771s-2174104/p/p660929)                  | 1 layer max. range: 25 m, ang. resol. 0.33 [deg]| ROS1: ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| TiM781             | [1096807](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/tim7xx/tim781-2174101/p/p594148)                   | 1 layer max. range: 25 m, ang. resol. 0.33 [deg]| ROS1: ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| TiM781S            | [1096363](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/tim7xx/tim781s-2174104/p/p594149)                  | 1 layer max. range: 25 m, ang. resol. 0.33 [deg]| ROS1: ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| LMS511-10100 PRO   | [e.g. 1046135](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/lms5xx/c/g179651)     | 1 layer max. range: 80 m, ang. resol. 0.167 [deg]| ROS1: ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 100 Hz   |                 |
| LMS1xx-Family      | [e.g. 1041114](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/lms1xx/c/g91901) | 1 layer max. range: 28 m, ang. resol. 0.25 [deg]| ROS1: ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| NAV310     | [e.g. 1052928](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/nav3xx/nav350-3232/p/p256041) | 1 layer max. range: 250 m, ang. resol. 0.125 [deg] | ROS1: ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 8 Hz   |                 |
| NAV210+NAV245      | [e.g. 	1074308](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/nav2xx/c/g356151) | 1 layer max. range: 100 m, ang. resol. 0.25 [deg]| ROS1: ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 25 Hz   |                 |
| LMS4xxx-Family      | [e.g. 1091423](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/lms4000/lms4111r-13000/p/p578044?ff_data) | 1 layer max. range: 3 m, ang. resol. 0,0833 [deg], opening angle: +/- 50 [deg] | ROS1: ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 600 Hz   |                 |
| RMS3xx             | [8021530](https://cdn.sick.com/media/docs/4/04/504/Operating_instructions_RMS3xx_en_IM0075504.PDF)| Radar Sensor | ROS1: ✔ [stable]|

## Build on Linux

Run the following steps to build sick_scan_xd on Linux (no ROS required):

1. Install jsoncpp:
   ```
   sudo apt-get install libjsoncpp-dev
   ```

2. Clone repository https://github.com/SICKAG/sick_scan_xd:
   ```
   git clone https://github.com/SICKAG/sick_scan_xd.git
   ```

3. Build sick_generic_caller:
   ```
   cd sick_scan_xd
   mkdir -p ./build_linux
   pushd ./build_linux
   cmake -DROS_VERSION=0 -G "Unix Makefiles" ..
   make -j4
   popd
   ```

## Build on Linux-ROS1

Run the following steps to build sick_scan_xd on Linux with ROS 1:

1. Install jsoncpp:
   ```
   sudo apt-get install libjsoncpp-dev
   ```

2. Clone repository https://github.com/SICKAG/sick_scan_xd:
   ```
   mkdir ./src
   pushd ./src
   git clone https://github.com/SICKAG/sick_scan_xd.git
   popd
   ```

3. Build sick_generic_caller:
   ```
   source /opt/ros/melodic/setup.bash
   catkin_make install --cmake-args -DROS_VERSION=1
   source ./install/setup.bash 
   ```

## Build on Windows

To install sick_scan_xd on Windows, follow the steps below:

1. If not yet done, install Visual Studio. Visual Studio 2019 Community or Professional Edition is recommended.

2. If not yet done, install boost and pthread using Visual Studios package manager vcpkg:
   * Install vcpkg:
      * Download vcpkg-master.zip from https://github.com/microsoft/vcpkg/archive/master.zip and unzip to `c:\vcpkg`. Alternatively, run "git clone https://github.com/microsoft/vcpkg"
      * Install vcpkg by running the following commands:
         ```
        cd c:/vcpkg
        bootstrap-vcpkg.bat
        vcpkg integrate install
        ```
   * Install required packages:
      ```
     vcpkg install pthread:x86-windows
     vcpkg install pthread:x64-windows
     vcpkg install boost:x64-windows
     vcpkg install jsoncpp
     vcpkg install jsoncpp:x64-windows
     ```
   * Include vcpkg in your path:
      ```
     set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%
     ```

3. Clone repository https://github.com/SICKAG/sick_scan_xd:
   ```
   git clone https://github.com/SICKAG/sick_scan_xd.git
   ```

4. Build sick_generic_caller with cmake and Visual Studio 2019:
   ```
   cd sick_scan_xd
   set _os=x64
   set _cmake_string=Visual Studio 16 2019
   set _msvc=Visual Studio 2019
   set _cmake_build_dir=build
   if not exist %_cmake_build_dir% mkdir %_cmake_build_dir%
   pushd %_cmake_build_dir%
   cmake -DROS_VERSION=0 -G "%_cmake_string%" ..
   popd
   ```
   Open file `build\sick_scan_xd.sln` in Visual Studio and build all targets (shortcut F7).

## IMU Support
Devices of the MRS6xxx and MRS1xxx series are available with an optionally built-in IMU.
Further information on the implementation and use of the experimental Imu support can be found on the [Imu page](doc/IMU.md).
##  Start Node

Use the following command to start ROS node:

- For MRS6124:
```bash
roslaunch sick_scan sick_mrs_6xxx.launch
```

- For MRS1104:
```bash
roslaunch sick_scan sick_mrs_1xxx.launch
```

- For LMS1104:
```bash
roslaunch sick_scan sick_lms_1xxx.launch
```

- For TiM240-prototype:
```bash
roslaunch sick_scan sick_tim_240.launch
```

- For TiM4xx-family:
```bash
roslaunch sick_scan sick_tim_4xx.launch
```

- For TiM5xx-family:
```bash
roslaunch sick_scan sick_tim_5xx.launch
```

- For TiM7xx-family (no safety scanner):
```bash
roslaunch sick_scan sick_tim_7xx.launch
```

- For TiM7xxS-family (safety scanner):
```bash
roslaunch sick_scan sick_tim_7xxS.launch
```

- For LMS1xx-family:
```bash
roslaunch sick_scan sick_lms_1xx.launch
```

- For LMS5xx-family:
```bash
roslaunch sick_scan sick_lms_5xx.launch
```

- For LMS4xxx-family:
```bash
roslaunch sick_scan sick_lms_4xxx.launch
```

- For NAV210:
```bash
roslaunch sick_scan sick_nav_2xx.launch
```

- For NAV245:
```bash
roslaunch sick_scan sick_nav_2xx.launch
```

- For NAV310:
```bash
roslaunch sick_scan sick_nav_3xx.launch
```

- For RMS3xx-family:
```bash
roslaunch sick_scan sick_rms_3xx.launch (under
opment)
```

### Starting Scanner with Specific Ip Address
To start the scanner with a specific IP address, the launch command can be used for most launch files as follows.
The hostname is the ip-address of the scanner:

```bash
roslaunch <launch-file> hostname:=<ip-address>
```
e.g.
```bash
roslaunch sick_scan sick_tim_5xx.launch hostname:=192.168.0.71
```


### Start Multiple Nodes

Take the launchfile "sick_tim_5xx_twin.launch" as an example.
Rempping the scan and cloud topics is essential to distinguish the scanndata and provide TF information.

## Parameter

The use of the parameters can be looked up in the launch files. This is also recommended as a starting point.
### Common parameters

- `scanner_type`
  Name of the used scanner. Usually this is also the name of the launch file. This entry is used to differentiate
  between the various scanner properties within the software code.

- `hostname`
  IP-address of the scanner (default: 192.168.0.1)

- `port`
  IP-port of the scanner (default: 2112)

- `min_ang`
  Start angle in [rad]

- `max_ang`
  End angle in [rad]

- `use_binary_protocol`
  Switch between SOPAS Binary and SOPAS ASCII protocol

- `intensity`
  Enable or disable transport of intensity values

- `intensity_resolution_16bit`
  If true, the intensity values is transferred as 16 bit value. If false, as 8 bit value.

- `cloud_topic`
  Topic name of the published pointcloud2 data

- `frame_id`
  Frame id used for the published data


### Further useful parameters and features

- `timelimit`
  Timelimit in [sec] for max. wait time of incoming sensor reply

- `sw_pll_only_publish`
  If true, the internal Software PLL is fored to sync the scan generation time stamp to a system timestamp

- Angle compensation: For highest angle accuracy the NAV-Lidar series supports an [angle compensation mechanism](./doc/angular_compensation.md).

- **Field monitoring**: The **LMS1xx**, **LMS5xx**, **TiM7xx** and **TiM7xxS** families have [extended settings for field monitoring](./doc/field_monitoring_extensions.md).

## Sopas Mode
This driver supports both COLA-B (binary) and COLA-A (ASCII) communication with the laser scanner. Binary mode is activated by default. Since this mode generates less network traffic.
If the communication mode set in the scanner memory is different from that used by the driver, the scanner's communication mode is changed. This requires a restart of the TCP-IP connection, which can extend the start time by up to 30 seconds.
There are two ways to prevent this:
1. [Recommended] Set the communication mode with the SOPAS ET software to binary and save this setting in the scanner's EEPROM.
2. Use the parameter "use_binary_protocol" to overwrite the default settings of the driver.
3. Setting "use_binary_protocol" to "False" activates COLA-A and disables COLA-B (default)


## Bugs and Feature Requests

- Stability issues: Driver is experimental for the RMS3xx
- Sopas protocol mapping:
-- All scanners: COLA-B (Binary)
- Software should be further tested, documented and beautified
- Setting of "topic" should not be hardcoded to /cloud in the future. This allows the simultaneous operation of several scanners. Each point cloud can then be converted using its own TF transformation.

## Tools

Various tools exist in the repository to improve the operation of the scanners. It is also recommended to read the following section "Troubleshooting".
Overview of the tools:

* Search for scanner in the network:
  Use the Python3 tool "sick_generic_device_finder.py" in the tools/sick_generic_device_finder directory.
  The tools will output the IP addresses of the connected scanners and some more information about the scanner.  
  Call it with python3, i.e.
  ``
  python3 sick_generic_device_finder.py
  ``
* Setting new IP address: With the help of the parameter "new_IP" a new IP address can be assigned when calling the node sick_scan.
  The launch file sick_new_ip.launch in the launch directory shows an example of how to use this parameter.
* Converting of pointclouds to images: With the tool pcl_converter.cpp one can convert pointcloud2-data
  to image. That is especial convenient for 24-layers scanners like the MRS6124.
* Setting up a brand new scanner: To set up a brand new scanner,
  it is recommended to use the two tools "sick_generic_device_finder.py" to find the scanner in the network
  and the launch file sick_new_ip.launch to set a new IP address. If further settings are to be saved that cannot be made via ROS   parameters, we recommend using the Windows tool "Sopas ET" from SICK.
* Unit tests: For a quick unit test after installation without the sensor hardware, a test server is provided to simulate a scanner. See [emulator](doc/emulator.md) for further details.


## Troubleshooting

1. Check Scanner IP in the launch file.
2. Check Ethernet connection to scanner with netcat e.g. ```nc -z -v -w5 $SCANNERIPADDRESS 2112```.
   For further details about setting up the correct ip settings see [IP configuration](doc/ipconfig/ipconfig.md)
3. View node startup output wether the IP connection could be established
4. Check the scanner status using the LEDs on the device. The LED codes are described in the above mentioned operation manuals.
5. Further testing and troubleshooting informations can found in the file test/readme_testplan.txt
6. If you stop the scanner in your debugging IDE or by other hard interruption (like Ctrl-C), you must wait until 60 sec. before
   the scanner is up and running again. During this time the MRS6124 reconnects twice.
   If you do not wait this waiting time you could see one of the following messages:
   * TCP connection error
   * Error-Message 0x0d
7. Amplitude values in rviz: If you see only one color in rviz try the following:
   Set the min/max-Range of intensity display in the range [0...200] and switch on the intensity flag in the launch file  
8. In case of network problems check your own ip address and the ip address of your laser scanner (by using SOPAS ET).
   * List of own IP-addresses: ifconfig|grep "inet addr"
   * Try to ping scanner ip address (used in launch file)
9. If the driver stops during init phase please stop the driver with ctrl-c and restart (could be caused due to protocol ASCII/Binary cola-dialect).

## FAQ

* FAQ: [doc/faq.md](doc/faq.md)

## Support

* In case of technical support please open a new issue. For optimal support, add the following information to your request:
 1. Scanner model name,
 2. Ros node startup log,
 3. Sopas file of your scanner configuration.
  The instructions at http://sickusablog.com/create-and-download-a-sopas-file/ show how to create the Sopas file.
* In case of application support please use [https://supportportal.sick.com ](https://supportportal.sick.com).
* Issue Handling: Issues, for which no reply was received from the questioner for more than 7 days,						
  are closed by us because we assume that the user has solved the problem.


## Installation

In the following instructions, replace `<rosdistro>` with the name of your ROS distro (e.g., `indigo`).

### From Binaries

The driver is released at longer intervals as a binary package and can therefore be installed via the package manager. To be able to use all new functions of the driver, the driver should be built from the sources published in this reposity:

`sudo apt-get install ros-<rosdistro>-sick-scan`

### From Source

```bash
source /opt/ros/<rosdistro>/setup.bash
mkdir -p ~/ros_catkin_ws/src/
cd ~/ros_catkin_ws/src/
git clone git://github.com/SICKAG/sick_scan.git
cd ..
catkin_make install
source ~/ros_catkin_ws/install/setup.bash
```

#### Development branch

A "devel" branch is also maintained for the very latest developments and tests. Add-ons and support for brand new scanners are usually first tested in this branch and can be checked out as needed as follows:
```bash
source /opt/ros/<rosdistro>/setup.bash
mkdir -p ~/ros_catkin_ws/src/
cd ~/ros_catkin_ws/src/
git clone -b devel --single-branch git://github.com/SICKAG/sick_scan.git
cd ..
catkin_make install
source ~/ros_catkin_ws/install/setup.bash
```

## Quick Start

```bash
roslaunch sick_scan sick_mrs6xxx.launch
rosrun rviz rviz
publish to point cloud
```
## Testing
The sick_scan_test program was developed for testing the driver. This program checks elementary properties of the scanner. In a first implementation stage, the shots per scan are checked. The test program works according to the following principle:
1. The parameters from an original launch file are read.
2. These parameters are modified according to the instructions in the test control file.
3. The modified parameters including all other parameter settings from the original launch file are copied to a test launch file.
4. The test launch file is started.
5. The parameters are checked.
6. The result of the check is transferred to a result file.
The basic procedure can be seen in the following figure:
![Alt text](./sick_scan_test.png?raw=true "princile of test program")
More information about the structure of the individual files in the test run can be found [here](test/sick_scan_test.md):


## Keywords

MRS1000
MRS1104
LMS1000
LMS1104
MRS6000
MRS6124
RMS3xx
RMS320
ROS LiDAR
SICK LiDAR
SICK Laser
SICK Laserscanner
SICK Radar
LMS1xx
MRS1xxx
LMS1xxx
MRS6xxx
TiM5xx
TiM551
TiM561
TiM571
TiM781
TiM781S
LMS5xx
LMS511
NAV210
NAV245
NAV310



## Creators

**Michael Lehning**

- <http://www.lehning.de>

on behalf of SICK AG

- <http://www.sick.com>

------------------------------------------------------------------------

<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/f/f1/Logo_SICK_AG_2009.svg/1200px-Logo_SICK_AG_2009.svg.png" width="420">

![Lehning Logo](http://www.lehning.de/style/banner.jpg "LEHNING Logo")
