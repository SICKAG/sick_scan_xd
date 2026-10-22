> [!WARNING]
>
> # ⚠ DEPRECATED — Archived Reference Only
>
> This document preserves information about **devices and platforms no longer actively maintained**:
>
> - **Deprecated devices (after driver version V3.9):** TiM2xx, TiM5xx, TiM7xx, TiM7xxS, LMS1xx, LD-LRS, LD-OEM, LD-MRS, NAV2xx, NAV3xx, MRS6124
> - **Deprecated platform:** ROS 1 (noetic, melodic, kinetic, …)
>
> For these products, **maintenance and support from SICK have been stopped**. No new features, bug fixes, or driver updates will be provided.
>
> **→ For current documentation, devices, and supported platforms see the main [README.md](../README.md).**
>
> The content below is archived for reference only and may be incomplete or outdated.

<details>
  <summary>Table of contents</summary>

* [Building the driver (legacy platforms and devices)](#building-the-driver-legacy-platforms-and-devices)
  * [ROS 1 on Linux](#ros-1-on-linux)
  * [LD-MRS build additions for Linux without ROS](#ld-mrs-build-additions-for-linux-without-ros)
* [Running the driver](#running-the-driver)
  * [Starting device with specific IP address](#starting-device-with-specific-ip-address)
  * [Start multiple devices / nodes](#start-multiple-devices--nodes)
  * [ROS services](#ros-services)
* [Getting started (legacy)](#getting-started-legacy)
  * [Detecting SICK devices in the network](#detecting-sick-devices-in-the-network)
  * [Change IP address](#change-ip-address)
  * [Test connection (Linux)](#test-connection-linux)
  * [Example startup sequence](#example-startup-sequence)
* [Driver features (legacy platforms and devices)](#driver-features-legacy-platforms-and-devices)
  * [Field evaluation information](#field-evaluation-information)
  * [Raspberry Pi: Build without internet or GitHub access (ROS 1 / LD-MRS)](#raspberry-pi-build-without-internet-or-github-access-ros-1--ld-mrs)
  * [Docker support](#docker-support)
  * [Hector SLAM support](#hector-slam-support)
  * [Google cartographer support](#google-cartographer-support)
  * [OctoMap support](#octomap-support)
  * [RTAB-Map support](#rtab-map-support)
* [Device specific information (deprecated devices)](#device-specific-information-deprecated-devices)
  * [TiMxxx](#timxxx)
  * [NAV350](#nav350)
  * [MRS6124](#mrs6124)
  * [Combination of devices (ROS 1)](#combination-of-devices-ros-1)
* [FAQ (deprecated-specific)](#faq-deprecated-specific)
  * [Why are my changes in launch files are ignored?](#why-are-my-changes-in-launch-files-are-ignored)
  * [How can I create a ROS 2 node in python to run sick\_generic\_caller from a launch.py file?](#how-can-i-create-a-ros-2-node-in-python-to-run-sick_generic_caller-from-a-launchpy-file)
  * [catkin reports "By not providing "FindSICKLDMRS.cmake" ..." . What can I do?](#catkin-reports-by-not-providing-findsickldmrscmake---what-can-i-do)
  * [How can I debug sick\_generic\_caller on ROS 1?](#how-can-i-debug-sick_generic_caller-on-ros-1)

</details>

---

## Building the driver (legacy platforms and devices)

Legacy build instructions for ROS 1 and LD-MRS. For current build instructions (ROS 2, native Linux/Windows), see [README.md](../README.md).

### ROS 1 on Linux

To build resp. install sick_scan_xd on Linux with ROS 1, you can build sick_scan_xd from sources or install prebuilt binaries.

#### ROS 1: Install prebuilt binaries

Run the following steps to install sick_scan_xd on Linux with ROS 1 noetic:

```sh
# Note: ROS1 is End of Life
sudo apt update
sudo apt-get install ros-noetic-sick-scan-xd
```

After successful installation, you can run sick_scan_xd using `roslaunch sick_scan_xd <launchfile>`, e.g. `roslaunch sick_scan_xd sick_picoscan.launch` for picoScan. sick_scan_xd can be removed by `sudo apt-get remove ros-noetic-sick-scan-xd`.

#### ROS 1: Build from sources

> [!NOTE]
> Although ROS1 is still mentioned below, active development for ROS1 will not be continued.

Run the following steps to build sick_scan_xd on Linux with ROS 1:

1. Create a workspace folder, e.g. `sick_scan_ws` (or any other name):

   ```sh
   mkdir -p ./sick_scan_ws
   cd ./sick_scan_ws
   ```

2. Clone repositories <https://github.com/SICKAG/libsick_ldmrs> and <https://github.com/SICKAG/sick_scan_xd>:

   ```sh
   mkdir ./src
   pushd ./src
   git clone https://github.com/SICKAG/libsick_ldmrs.git
   git clone -b master https://github.com/SICKAG/sick_scan_xd.git
   popd
   rm -rf ./build ./build_isolated/ ./devel ./devel_isolated/ ./install ./install_isolated/ ./log/ # remove any files from a previous build
   ```

3. Build sick_generic_caller:

   ```sh
   source /opt/ros/noetic/setup.bash # replace noetic by your ros distro
   catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -Wno-dev
   source ./devel_isolated/setup.bash
   # source ./install_isolated/setup.bash
   ```

   For ROS versions other than noetic, please replace `source /opt/ros/noetic/setup.bash` with your ROS distribution.

LD-MRS sensors are currently not supported on Raspberry. Build with cmake flag `-DLDMRS=0 -DRASPBERRY=1` on Raspberry:

```sh
catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DLDMRS=0 -DRASPBERRY=1 -Wno-dev
```

libsick_ldmrs is only required to support LD-MRS sensors. If you do not need or want to support LD-MRS, you can skip building libsick_ldmrs. To build sick_generic_caller without LD-MRS support, switch off option `BUILD_WITH_LDMRS_SUPPORT` in [CMakeLists.txt](./CMakeLists.txt) or call catkin_make_isolated with option `-DLDMRS=0`:

```sh
catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DLDMRS=0 -Wno-dev
```

To build sick_generic_caller without multiScan100/picoScan100/sick_scansegment_xd support, switch off option `BUILD_WITH_SCANSEGMENT_XD_SUPPORT` in [CMakeLists.txt](./CMakeLists.txt) or call cmake with option `-DSCANSEGMENT_XD=0`:

```sh
catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DSCANSEGMENT_XD=0 -Wno-dev
```

cmake flags can be combined. Use flags `-DLDMRS=0 -DSCANSEGMENT_XD=0` to build **without LD-MRS** and **without multiScan100/picoScan100 support**:

```sh
catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DLDMRS=0 -DSCANSEGMENT_XD=0 -Wno-dev
```

By default, sick_scan_xd builds with compiler flag `-O3` (optimization level for max speed). The optimization level can be overwritten (e.g. for debugging) by cmake flag `-DO=0` (compiler flags `-g -O0`), `-DO=1` (for compiler flags `-O1`) or `-DO=2` (for compiler flags `-O2`).

The sick_scan_xd driver requires C++14 and gcc version 5 or newer.

Build options:

| **LD-MRS support** | **multiScan100 / picoScan100 support** | **Build command**                                                                                         |
| ------------------ | -------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| no                 | no                                     | ``` catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DLDMRS=0 -DSCANSEGMENT_XD=0 -Wno-dev ``` |
| no                 | yes                                    | ``` catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DLDMRS=0 -Wno-dev ```                    |
| yes                | no                                     | ``` catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DSCANSEGMENT_XD=0 -Wno-dev ```           |
| yes                | yes                                    | ``` catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DSCANSEGMENT_XD=1 -Wno-dev ```           |

To create source code documentation by doxygen, run

```sh
cd ./doxygen
doxygen ./docs/Doxyfile
```

### LD-MRS build additions for Linux without ROS

Run the following steps to build sick_scan_xd on Linux (no ROS required):

1. Create a workspace folder, e.g. `sick_scan_ws` (or any other name):

   ```sh
   mkdir -p ./sick_scan_ws
   cd ./sick_scan_ws
   ```

2. Clone repositories <https://github.com/SICKAG/libsick_ldmrs> and <https://github.com/SICKAG/sick_scan_xd>:

   ```sh
   git clone https://github.com/SICKAG/libsick_ldmrs.git
   git clone -b master https://github.com/SICKAG/sick_scan_xd.git
   ```

3. Build libsick_ldmrs (required only once for LD-MRS sensors):

   ```sh
   pushd libsick_ldmrs
   mkdir -p ./build
   cd ./build
   cmake -G "Unix Makefiles" ..
   make -j4
   sudo make -j4 install
   popd
   ```

4. Build sick_generic_caller and libsick_scan_xd_shared_lib.so:

   ```sh
   mkdir -p ./build
   pushd ./build
   rm -rf ./*
   export ROS_VERSION=0
   cmake -DROS_VERSION=0 -G "Unix Makefiles" ../sick_scan_xd
   make -j4
   sudo make -j4 install
   popd
   ```

LD-MRS sensors are currently not supported on Raspberry. Build with cmake flag `-DLDMRS=0 -DRASPBERRY=1` on Raspberry:

   ```sh
   cmake -DROS_VERSION=0 -DLDMRS=0 -DRASPBERRY=1 -G "Unix Makefiles" ../sick_scan_xd
   ```

libsick_ldmrs is only required to support LD-MRS sensors. If you do not need or want to support LD-MRS, you can skip building libsick_ldmrs. To build sick_generic_caller without LD-MRS support, switch off option `BUILD_WITH_LDMRS_SUPPORT` in [CMakeLists.txt](./CMakeLists.txt) or call cmake with option `-DLDMRS=0`:

   ```sh
   cmake -DROS_VERSION=0 -DLDMRS=0 -G "Unix Makefiles" ../sick_scan_xd
   ```

To build sick_generic_caller without multiScan100/picoScan100 support, switch off option `BUILD_WITH_SCANSEGMENT_XD_SUPPORT` in [CMakeLists.txt](./CMakeLists.txt) or call cmake with option `-DSCANSEGMENT_XD=0`:

   ```sh
   cmake -DROS_VERSION=0 -DSCANSEGMENT_XD=0 -G "Unix Makefiles" ../sick_scan_xd
   ```

cmake flags can be combined. Use flags `-DLDMRS=0 -DSCANSEGMENT_XD=0` to build without LD-MRS and scansegment_xd support:

   ```sh
   cmake -DROS_VERSION=0 -DLDMRS=0 -DSCANSEGMENT_XD=0 -G "Unix Makefiles" ../sick_scan_xd
   ```

By default, sick_scan_xd builds with compiler flag `-O3` (optimization level for max speed). The optimization level can be overwritten (e.g. for debugging) by cmake flag `-DO=0` (compiler flags `-g -O0`), `-DO=1` (for compiler flags `-O1`) or `-DO=2` (for compiler flags `-O2`).

> **_NOTE:_** To create source code documentation by doxygen, run

```sh
cd ./doxygen
doxygen ./docs/Doxyfile
```

## Running the driver

The sick_scan_xd driver can be started on the command line by `sick_generic_caller <launchfile> [hostname:=<ip-address>]`. The start process varies slightly depending on the target OS:

On native Linux without ROS, call

```sh
sick_generic_caller <launchfile>
```

On Linux with ROS 1, call

```sh
./devel_isolated/setup.bash
roslaunch sick_scan_xd <launchfile>
```

On Linux with ROS 2, call

```sh
source ./install/setup.bash
ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/<launchfile>
```

On native Windows without ROS, call

```sh
sick_generic_caller <launchfile>
```

On Windows with ROS 2, call

```sh
call .\install\setup.bat
ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/<launchfile>
```

Use the following commands to run the sick_scan_xd driver for a specific device type:

- For MRS6124:
  - Linux native:   `sick_generic_caller sick_mrs_6xxx.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_mrs_6xxx.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_mrs_6xxx.launch`
  - Windows native: `sick_generic_caller sick_mrs_6xxx.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_mrs_6xxx.launch`
- For MRS1104:
  - Linux native:   `sick_generic_caller sick_mrs_1xxx.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_mrs_1xxx.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_mrs_1xxx.launch`
  - Windows native: `sick_generic_caller sick_mrs_1xxx.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_mrs_1xxx.launch`
- For LMS1104 with firmware 1.x:
  - Linux native:   `sick_generic_caller sick_lms_1xxx.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_lms_1xxx.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_1xxx.launch`
  - Windows native: `sick_generic_caller sick_lms_1xxx.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_1xxx.launch`
- For LMS1104 with firmware 2.x:
  - Linux native:   `sick_generic_caller sick_lms_1xxx_v2.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_lms_1xxx_v2.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_1xxx_v2.launch`
  - Windows native: `sick_generic_caller sick_lms_1xxx_v2.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_1xxx_v2.launch`
- For TiM240-prototype:
  - Linux native:   `sick_generic_caller sick_tim_240.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_tim_240.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_240.launch`
  - Windows native: `sick_generic_caller sick_tim_240.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_240.launch`
- For TiM5xx-family:
  - Linux native:   `sick_generic_caller sick_tim_5xx.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_tim_5xx.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_5xx.launch`
  - Windows native: `sick_generic_caller sick_tim_5xx.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_5xx.launch`
- For TiM7xx-family (no safety device):
  - Linux native:   `sick_generic_caller sick_tim_7xx.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_tim_7xx.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xx.launch`
  - Windows native: `sick_generic_caller sick_tim_7xx.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xx.launch`
- For TiM7xxS-family (safety device):
  - Linux native:   `sick_generic_caller sick_tim_7xxS.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_tim_7xxS.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xxS.launch`
  - Windows native: `sick_generic_caller sick_tim_7xxS.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xxS.launch`
- For LMS1xx-family:
  - Linux native:   `sick_generic_caller sick_lms_1xx.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_lms_1xx.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_1xx.launch`
  - Windows native: `sick_generic_caller sick_lms_1xx.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_1xx.launch`
- For LMS5xx-family:
  - Linux native:   `sick_generic_caller sick_lms_5xx.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_lms_5xx.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_5xx.launch`
  - Windows native: `sick_generic_caller sick_lms_5xx.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_5xx.launch`
- For LMS4000-family:
  - Linux native:   `sick_generic_caller sick_lms_4xxx.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_lms_4xxx.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_4xxx.launch`
  - Windows native: `sick_generic_caller sick_lms_4xxx.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_4xxx.launch`
- For LRS4000:
  - Linux native:   `sick_generic_caller sick_lrs_4xxx.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_lrs_4xxx.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lrs_4xxx.launch`
  - Windows native: `sick_generic_caller sick_lrs_4xxx.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lrs_4xxx.launch`
- For LD-MRS-family (Note that LD-MRS are currently not supported on Windows):
  - Linux native:   `sick_generic_caller sick_ldmrs.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_ldmrs.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_ldmrs.launch`
- For LRS36x0:
  - Linux native:   `sick_generic_caller sick_lrs_36x0.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_lrs_36x0.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lrs_36x0.launch`
  - Windows native: `sick_generic_caller sick_lrs_36x0.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lrs_36x0.launch`
- For LRS36x0 mounted upside down:
  - Note: For upside down mounted devices, the point cloud is rotated by 180 deg about the x axis (180 deg roll angle). This additional rotation is configured in the launch file using parameter `add_transform_xyz_rpy` with value `"0,0,0,3.141592,0,0"`
  - Linux native:   `sick_generic_caller sick_lrs_36x0_upside_down.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_lrs_36x0_upside_down.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lrs_36x0_upside_down.launch`
  - Windows native: `sick_generic_caller sick_lrs_36x0_upside_down.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lrs_36x0_upside_down.launch`
- For LRS36x1:
  - Linux native:   `sick_generic_caller sick_lrs_36x1.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_lrs_36x1.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lrs_36x1.launch`
  - Windows native: `sick_generic_caller sick_lrs_36x1.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lrs_36x1.launch`
- For LRS36x1 mounted upside down:
  - Note: For upside down mounted devices, the point cloud is rotated by 180 deg about the x axis (180 deg roll angle). This additional rotation is configured in the launch file using parameter `add_transform_xyz_rpy` with value `"0,0,0,3.141592,0,0"`.
  - Linux native:   `sick_generic_caller sick_lrs_36x1_upside_down.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_lrs_36x1_upside_down.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lrs_36x1_upside_down.launch`
  - Windows native: `sick_generic_caller sick_lrs_36x1_upside_down.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lrs_36x1_upside_down.launch`
- For LD-OEM15xx:
  - Linux native:   `sick_generic_caller sick_oem_15xx.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_oem_15xx.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_oem_15xx.launch`
  - Windows native: `sick_generic_caller sick_oem_15xx.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_oem_15xx.launch`
- For NAV210 and NAV245:
  - Linux native:   `sick_generic_caller sick_nav_2xx.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_nav_2xx.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_nav_2xx.launch`
  - Windows native: `sick_generic_caller sick_nav_2xx.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_nav_2xx.launch`
- For NAV310:
  - Linux native:   `sick_generic_caller sick_nav_31x.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_nav_31x.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_nav_31x.launch`
  - Windows native: `sick_generic_caller sick_nav_31x.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_nav_31x.launch`
- For NAV350:
  - Linux native:   `sick_generic_caller sick_nav_350.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_nav_350.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_nav_350.launch`
  - Windows native: `sick_generic_caller sick_nav_350.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_nav_350.launch`
- For RMS1009 and RMS2000:
  - Linux native:   `sick_generic_caller sick_rms_xxxx.launch`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_rms_xxxx.launch`
  - Linux ROS 2:    `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_rms_xxxx.launch`
  - Windows native: `sick_generic_caller sick_rms_xxxx.launch`
  - Windows ROS 2:  `ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_rms_xxxx.launch`
- For multiScan100:
  - Linux native:   `sick_generic_caller sick_multiscan.launch hostname:=<ip-address> udp_receiver_ip:=<ip-address>`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_multiscan.launch hostname:=<ip-address> udp_receiver_ip:=<ip-address>`
  - Linux ROS 2:    `ros2 launch sick_scan_xd sick_multiscan.launch.py hostname:=<ip-address> udp_receiver_ip:=<ip-address>`
  - Windows native: `sick_generic_caller sick_multiscan.launch hostname:=<ip-address> udp_receiver_ip:=<ip-address>`
  - Windows ROS 2:  `ros2 launch sick_scan_xd sick_multiscan.launch.py hostname:=<ip-address> udp_receiver_ip:=<ip-address>`
  - `hostname` is the IP address of the lidar, `udp_receiver_ip` is the IP address of the receiver (i.e. the IP of the computer running sick_generic_caller).
- For picoScan120:
  - Linux native:   `sick_generic_caller sick_picoscan_120.launch hostname:=<ip-address> udp_receiver_ip:=<ip-address>`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_picoscan_120.launch hostname:=<ip-address> udp_receiver_ip:=<ip-address>`
  - Linux ROS 2:    `ros2 launch sick_scan_xd sick_picoscan_120.launch.py hostname:=<ip-address> udp_receiver_ip:=<ip-address>`
  - Windows native: `sick_generic_caller sick_picoscan_120.launch hostname:=<ip-address> udp_receiver_ip:=<ip-address>`
  - Windows ROS 2:  `ros2 launch sick_scan_xd sick_picoscan_120.launch.py hostname:=<ip-address> udp_receiver_ip:=<ip-address>`
  - `hostname` is the IP address of the lidar, `udp_receiver_ip` is the IP address of the receiver (i.e. the IP of the computer running sick_generic_caller).
- For picoScan150:
  - Linux native:   `sick_generic_caller sick_picoscan.launch hostname:=<ip-address> udp_receiver_ip:=<ip-address>`
  - Linux ROS 1:    `roslaunch sick_scan_xd sick_picoscan.launch hostname:=<ip-address> udp_receiver_ip:=<ip-address>`
  - Linux ROS 2:    `ros2 launch sick_scan_xd sick_picoscan.launch.py hostname:=<ip-address> udp_receiver_ip:=<ip-address>`
  - Windows native: `sick_generic_caller sick_picoscan.launch hostname:=<ip-address> udp_receiver_ip:=<ip-address>`
  - Windows ROS 2:  `ros2 launch sick_scan_xd sick_picoscan.launch.py hostname:=<ip-address> udp_receiver_ip:=<ip-address>`
  - `hostname` is the IP address of the lidar, `udp_receiver_ip` is the IP address of the receiver (i.e. the IP of the computer running sick_generic_caller).

Common command is `hostname:=<ip-address>` to connect to a sensor with a given IP address. Default value is always the factory default IP address of the device. Further (common and device specific) options can be set via launch file, see [Parameters](../README.md#parameters) and configure the settings in the launch file corresponding to the device type.

> **_NOTE:_** After modifying a launch file, it has to be installed by running `catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -Wno-dev`
to be located and used by `roslaunch`.

On ROS 2 you can launch sick_generic_caller by python launch files, too. Use

```sh
ros2 launch sick_scan_xd <name>.launch.py <param>:=<value>
```

E.g. for LMS5xx: `ros2 launch sick_scan_xd sick_lms_5xx.launch.py hostname:=192.168.0.1`. The launch.py-files on ROS 2 passes the corresponding launch file to the driver: [sick_lms_5xx.launch.py](launch/sick_lms_5xx.launch.py) gives an example for LMS5xx. Parameter can be overwritten:

- either by command line, e.g. `ros2 launch sick_scan_xd sick_lms_5xx.launch.py hostname:=192.168.0.1`,
- or by passing additional arguments in the launch.py-file, e.g. `node = Node(package='sick_scan_xd', executable='sick_generic_caller', arguments=[launch_file_path, 'hostname:=192.168.0.1'])`

### Starting device with specific IP address

To start the device with a specific IP address, option `hostname:=<ip-address>` can be used.
The hostname is the IP address of the device, e.g.

```sh
sick_generic_caller sick_tim_5xx.launch hostname:=192.168.0.71                         # Linux native
roslaunch sick_scan_xd sick_tim_5xx.launch hostname:=192.168.0.71                      # Linux ROS 1
ros2 run sick_scan_xd sick_generic_caller sick_tim_5xx.launch hostname:=192.168.0.71   # Linux ROS 2
sick_generic_caller sick_tim_5xx.launch hostname:=192.168.0.71                         # Windows native
ros2 run sick_scan_xd sick_generic_caller sick_tim_5xx.launch hostname:=192.168.0.71   # Windows ROS 2
```

### Start multiple devices / nodes

Multiple nodes can be started to support multiple devices. In this case, multiple instances of sick_scan_xd have to be started, each node with different name and topic. ROS 1 example to run two TiM 7xx devices with IP address `192.168.0.1` and `192.168.0.2`:

```sh
roslaunch sick_scan_xd sick_tim_7xx.launch nodename:=sick_tim_7xx_1 hostname:=192.168.0.1 cloud_topic:=cloud_1 &
roslaunch sick_scan_xd sick_tim_7xx.launch nodename:=sick_tim_7xx_2 hostname:=192.168.0.2 cloud_topic:=cloud_2 &
```

On Linux with ROS 1, multiple nodes to support multiple sensors can be started by one launch file, too. Take the launch file [sick_tim_5xx_twin.launch](launch/sick_tim_5xx_twin.launch) as an example. Remapping the scan and cloud topics is essential to distinguish the scan data and provide TF information.

ROS 2 example to run two TiM7xx devices with IP address `192.168.0.1` and `192.168.0.2`:

```sh
ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xx.launch nodename:=sick_tim_7xx_1 hostname:=192.168.0.1 cloud_topic:=cloud_1 &
ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xx.launch nodename:=sick_tim_7xx_2 hostname:=192.168.0.2 cloud_topic:=cloud_2 &
```

To support multiple sensors, sick_scan_xd has to be started multiple times, with one sick_scan_xd-node for each sensor. By default, each sick_scan_xd-node connects to "192.168.0.1" and publishes its point cloud on topic "cloud". Therefore both the node name, the IP address of the sensor and the point cloud topic have to be configured differently for each node. Node name, IP address and point cloud topic can be configured in the launch file or by command line argument:
Topic, nodename and IP configuration in a launch file (example for TiM7xx):

```xml
    <launch>
        <arg name="nodename" default="sick_tim_7xx"/>
        <arg name="hostname" default="192.168.0.1"/>
        <arg name="cloud_topic" default="cloud"/>
        <node name="$(arg nodename)" pkg="sick_scan_xd" type="sick_generic_caller" respawn="false" output="screen">
            <param name="scanner_type" type="string" value="sick_tim_7xx"/>
            <param name="nodename" type="string" value="$(arg nodename)"/>
            <param name="hostname" type="string" value="$(arg hostname)"/>
            <param name="cloud_topic" type="string" value="$(arg cloud_topic)"/>
```

Topic, node name and IP configuration by command line (ROS 1 example for TiM7xx):

```sh
roslaunch sick_scan_xd sick_tim_7xx.launch nodename:=sick_tim_7xx_1 hostname:=192.168.0.1 cloud_topic:=cloud_1
roslaunch sick_scan_xd sick_tim_7xx.launch nodename:=sick_tim_7xx_2 hostname:=192.168.0.2 cloud_topic:=cloud_2
```

Topic, node name and IP configuration by command line (ROS 2 example for TiM7xx):

```sh
ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xx.launch nodename:=sick_tim_7xx_1 hostname:=192.168.0.1 cloud_topic:=cloud_1
ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_tim_7xx.launch nodename:=sick_tim_7xx_2 hostname:=192.168.0.2 cloud_topic:=cloud_2
```

The scripts [run_linux_ros1_simu_tim7xx_twin.bash](test/scripts/run_linux_ros1_simu_tim7xx_twin.bash) and [run_linux_ros2_simu_tim7xx_twin.bash](test/scripts/run_linux_ros2_simu_tim7xx_twin.bash) show a complete example with emulation of two TiM7xx sensors and two sick_scan_xd nodes running concurrently using different nodenames and topics.

To run two multiScan100 or picoScan100 devices simultaneously, each sick_scan_xd node must be configured with different lidar IP addresses and UDP ports, different node names, different ROS topics and frame ids for each point cloud. Therefore the following launch file parameter should be overwritten by individual settings for each lidar:

- `hostname`: e.g. "192.168.0.190" and "192.168.0.98"
- `nodename`: e.g. sick_picoscan0" and "sick_picoscan1"
- `publish_frame_id`: e.g. "world0" and "world1"
- `publish_laserscan_segment_topic`: e.g. "scan0_segment" and "scan1_segment"
- `publish_laserscan_fullframe_topic`: e.g. "scan0_fullframe" and "scan1_fullframe"
- `imu_topic`: e.g. "imu0" and "imu1"
- `udp_port`: e.g. "56661" and "56662"
- `imu_udp_port`: e.g. "7503" and "7504"
- individual topics and frame ids for each customized point cloud, e.g.

  - replace all "topic=/cloud_" by "topic=/cloud0_" resp. "topic=/cloud1_"
  - replace all "frameid=world" by "frameid=world0" resp. "frameid=world1"

It is recommended to first verify the launch file configurations separately for each picoScan100 before running them simultaneously.
For picoScan100 and multiScan100, parameter udp_receiver_ip must be set to the IP address of the PC running sick_scan_xd. It is recommend to use IP addresses in the same subnet.

> **_NOTE:_** The sick_scan_xd API does not support running multiple lidars simultaneously in a single process.** Currently the sick_scan_xd API does not support the single or multi-threaded use of 2 or more lidars in one process, since the sick_scan_xd library is not guaranteed to be thread-safe. To run multiple lidars simultaneously, we recommend using ROS or running sick_scan_xd in multiple and separate processes, so that each process serves one sensor.

### ROS services

On ROS 1 and ROS 2, services can be used to send COLA commands to the sensor. This can be very helpful for diagnosis, e.g. by querying the device status or its id.

Use the following examples to run a cola command on ROS 1:

```sh
rosservice call /sick_lms_5xx/ColaMsg "{request: 'sMN IsSystemReady'}"
rosservice call /sick_lms_5xx/ColaMsg "{request: 'sRN SCdevicestate'}"
rosservice call /sick_lms_5xx/ColaMsg "{request: 'sEN LIDinputstate 1'}"
rosservice call /sick_lms_5xx/ColaMsg "{request: 'sEN LIDoutputstate 1'}"
rosservice call /sick_lms_5xx/ColaMsg "{request: 'sMN LMCstartmeas'}"
rosservice call /sick_lms_5xx/SCdevicestate "{}" # query device state
rosservice call /sick_lms_5xx/SCreboot "{}"      # execute a software reset on the device
rosservice call /sick_lms_5xx/SCsoftreset "{}"   # save current parameter and shut down device
```

Use the following examples to run a cola command on ROS 2:

```sh
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sMN IsSystemReady'}"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sRN SCdevicestate'}"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sEN LIDinputstate 1'}"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sEN LIDoutputstate 1'}"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sMN LMCstartmeas'}"
ros2 service call /SCdevicestate sick_scan_xd/srv/SCdevicestateSrv "{}" # query device state
ros2 service call /SCreboot sick_scan_xd/srv/SCrebootSrv "{}"           # execute a software reset on the device
ros2 service call /SCsoftreset sick_scan_xd/srv/SCsoftresetSrv "{}"     # save current parameter and shut down device
```

Use ROS service `SickScanExit` to stop the device and driver:

```sh
rosservice call /sick_nav_31x/SickScanExit "{}" # stop device and driver on ROS 1
ros2 service call /SickScanExit sick_scan_xd/srv/SickScanExitSrv "{}" # stop device and driver on ROS 2
```

> **_NOTE:_**
>
> - The COLA commands are sensor specific. See the user manual and telegram listing for further details.
> - ROS services require installation of ROS 1 or ROS 2, i.e. services for Cola commands are currently not supported on native Linux or native Windows.
> - ROS services are currently not available for the LD-MRS.
> - ROS service "ColaMsg" should only be used for diagnosis. It is not recommended to change the lidar settings while the driver is running. Otherwise the driver settings can become different or inconsistent to the lidar settings. Restart the driver after changing lidar settings by SOAPS ET or SOPAS commands.
> - Some SOPAS commands like `sMN SetAccessMode 3 F4724744` stop the current measurement. In this case, the driver restarts after a timeout (5 seconds by default). To process those SOPAS commands without restart, you can
>   - send `sMN LMCstartmeas` and `sMN Run` to switch again into measurement mode within the timeout, or
>   - increase the driver timeout `read_timeout_millisec_default` in the launch file.

Additional services can be available for specific lidars. Service "GetContaminationResult" is e.g. available for MRS1000, LMS1000, multiScan100 and picoScan100:

```sh
# ROS 1 example for service GetContaminationResult (LMS1000)
rosservice call /sick_lms_1xxx/GetContaminationResult "{}"
# ROS 2 example for service GetContaminationResult (LMS1000)
ros2 service call /GetContaminationResult sick_scan_xd/srv/GetContaminationResultSrv "{}"
```

Service "GetContaminationData" is supported for LRS4000:

```sh
# ROS 1 example for service GetContaminationData (LRS4000 only)
rosservice call /sick_lrs_4xxx/GetContaminationData "{}"
# ROS 2 example for service GetContaminationData (LRS4000 only)
ros2 service call /GetContaminationData sick_scan_xd/srv/GetContaminationDataSrv "{}"
```

Example sequence with stop and start measurement to set a particle filter (TiM7xx on ROS 1):

```sh
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sMN SetAccessMode 3 F4724744'}"
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sRN LFPparticle'}" # response: "sRA LFPparticle \\x00\\x01\\xf4"
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sWN LFPparticle 0101F4'}" # response: "sWA LFPparticle"
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sMN LMCstartmeas'}"
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sMN Run'}"
```

## Getting started (legacy)

### Detecting SICK devices in the network

The [Python script](sick_scan_xd/tools/sick_generic_device_finder/sick_generic_device_finder.py) sends a UDP broadcast to which all available devices respond with a device description. The variable `UDP_IP = "192.168.0.255"` defines the broadcast address used by the script. If you are using a different IP address configuration on your host pc you have to change this variable according to the broadcast address of your network card. `ifconfig` shows the broadcast address for every network adapter. This method works for the device families: TiMxxx, LMS100 and LMS500.

### Change IP address

The IP address of the device can be changed with a customized launch file. The following launch sequence is an example:

```sh
roslaunch sick_scan_xd sick_new_ip.launch hostname:=192.168.0.1 new_IP:=192.168.0.100
```

The launch file restarts the lidar after the address change and stops the sick_scan_xd node. After a few seconds of boot time the device is reachable at the new IP address. The Python script is experimental. It is known that some Ethernet adapters are not fully supported. As a fallback you can always use the SOPAS ET software under Windows.

### Test connection (Linux)

To test the settings on Linux you can use netcat to check whether a TCP connection to the device can be established: `nc -z -v -w5 $SCANNERIPADDRESS 2112`. The connection should be successfully established.

```sh
@ubuntu:~S nc -z -v -w5 192.168.0.71 2112
Connection to 192.168.0.71 2112 port [tcp/*] succeeded!
```

Unlike a ping, the connection attempt to the host PC will fail.

```sh
@ubuntu: ~$ nc -z-v -w5 192.168.0.110 2112
nc: connect to 192.168.0.110 port 2112 (tcp) failed: Connection refused
```

### Example startup sequence

The following ROS boot protocol shows the typical start sequence when starting a SICK device. The MRS6124 is shown here as an example. However, the startup sequence is generally similar for all devices.

```sh
roslaunch sick_scan_xd sick_mrs_6xxx.launch hostname:=192.168.0.25
... logging to /home/rosuser/.ros/log/75631922-6109-11e9-b76f-54e1ad2921b6/roslaunch-ROS-NB-10680.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.
started roslaunch server http://ROS-NB:40757/
SUMMARY
========
PARAMETERS
 - /rosdistro: melodic
 - /rosversion: 1.14.3
 - /sick_mrs_6xxx/filter_echos: 0
 - /sick_mrs_6xxx/hostname: 192.168.0.25
 - /sick_mrs_6xxx/max_ang: 1.047197333
 - /sick_mrs_6xxx/min_ang: -1.040216
 - /sick_mrs_6xxx/port: 2112
 - /sick_mrs_6xxx/range_max: 250.0
 - /sick_mrs_6xxx/range_min: 0.1
 - /sick_mrs_6xxx/scanner_type: sick_mrs_6xxx
 - /sick_mrs_6xxx/timelimit: 5
 - /sick_mrs_6xxx/use_binary_protocol: True
NODES
  /
    sick_mrs_6xxx (sick_scan_xd/sick_generic_caller)
auto-starting new master
process[master]: started with pid [10690]
ROS_MASTER_URI=http://localhost:11311
setting /run_id to 75631922-6109-11e9-b76f-54e1ad2921b6
process[rosout-1]: started with pid [10701]
started core service [/rosout]
process[sick_mrs_6xxx-2]: started with pid [10708]
[ INFO] [1555502887.036684738]: sick_generic_caller V. 001.003.016
[ INFO] [1555502887.036717573]: Program arguments: /home/rosuser/ros_catkin_ws/devel/lib/sick_scan_xd/sick_generic_caller
[ INFO] [1555502887.036725741]: Program arguments: __name:=sick_mrs_6xxx
[ INFO] [1555502887.036731933]: Program arguments: __log:=/home/rosuser/.ros/log/75631922-6109-11e9-b76f-54e1ad2921b6/sick_mrs_6xxx-2.log
[ INFO] [1555502887.048425000]: Found sopas_protocol_type param overwriting default protocol:
[ INFO] [1555502887.048956468]: Binary protocol activated
[ INFO] [1555502887.048984179]: Start initializing scanner [Ip: 192.168.0.25] [Port: 2112]
[ INFO] [1555502887.067528995]: Publishing laserscan-pointcloud2 to cloud
[ INFO] [1555502887.071035827]: Parameter setting for <active_echo: 0>
[ INFO] [1555502887.271739084]: Sending  : <STX><STX><STX><STX><Len=0023>sMN SetAccessMode 0x03 0xf4 0x72 0x47 0x44 CRC:<0xb3>
[ INFO] [1555502887.273290840]: Receiving: <STX>sAN SetAccessMode \x01<ETX>
[ INFO] [1555502887.473927858]: Sending  : <STX><STX><STX><STX><Len=0015>sWN EIHstCola 0x01 CRC:<0x09>
[ INFO] [1555502887.475365983]: Receiving: <STX>sWA EIHstCola <ETX>
[ INFO] [1555502887.675864993]: Sending  : <STX><STX><STX><STX><Len=0015>sMN LMCstopmeas CRC:<0x10>
[ INFO] [1555502888.199590269]: Receiving: <STX>sAN LMCstopmeas \x00<ETX>
[ INFO] [1555502888.400030148]: Sending  : <STX><STX><STX><STX><Len=0015>sRN DeviceIdent CRC:<0x25>
[ INFO] [1555502888.401393378]: Receiving: <STX>sRA DeviceIdent \x00\x08\x4d\x52\x53\x36\x31\x32\x34\x52\x00\x0a\x31\x2e\x31\x2e\x30\x2e\x35\x36\x35\x43<ETX>
[ INFO] [1555502888.401653485]: Deviceinfo MRS6124R V1.1.0.565C found and supported by this driver.
[ INFO] [1555502888.602062286]: Sending  : <STX><STX><STX><STX><Len=0019>sRN FirmwareVersion CRC:<0x24>
[ INFO] [1555502888.603444526]: Receiving: <STX>sRA FirmwareVersion \x00\x0a\x31\x2e\x31\x2e\x30\x2e\x35\x36\x35\x43<ETX>
[ INFO] [1555502888.804094446]: Sending  : <STX><STX><STX><STX><Len=0017>sRN SCdevicestate CRC:<0x30>
[ INFO] [1555502888.805521867]: Receiving: <STX>sRA SCdevicestate \x01<ETX>
[ INFO] [1555502889.006161400]: Sending  : <STX><STX><STX><STX><Len=0010>sRN ODoprh CRC:<0x41>
[ INFO] [1555502889.007613972]: Receiving: <STX>sRA ODoprh \x00\x00\x19\xf1<ETX>
[ INFO] [1555502889.209949897]: Sending  : <STX><STX><STX><STX><Len=0010>sRN ODpwrc CRC:<0x52>
[ INFO] [1555502889.211413041]: Receiving: <STX>sRA ODpwrc \x00\x00\x02\x55<ETX>
[ INFO] [1555502889.413742132]: Sending  : <STX><STX><STX><STX><Len=0016>sRN LocationName CRC:<0x55>
[ INFO] [1555502889.415205992]: Receiving: <STX>sRA LocationName \x00\x0b\x6e\x6f\x74\x20\x64\x65\x66\x69\x6e\x65\x64<ETX>
[ INFO] [1555502889.417205292]: Sending  : <STX><STX><STX><STX><Len=0018>sRN LMPoutputRange CRC:<0x5e>
[ INFO] [1555502889.418631134]: Receiving: <STX>sRA LMPoutputRange \x00\x01\x00\x00\x05\x15\x00\x04\xa3\x80\x00\x16\xe3\x60<ETX>
[ INFO] [1555502889.418830949]: Angle resolution of scanner is 0.13010 [deg]  (in 1/10000th deg: 0x515)
[ INFO] [1555502889.418907556]: MIN_ANG:   -1.040 [rad]  -59.600 [deg]
[ INFO] [1555502889.418975818]: MAX_ANG:    1.047 [rad]   60.000 [deg]
[ INFO] [1555502889.419156102]: Sending  : <STX><STX><STX><STX><Len=0033>sWN LMPoutputRange 0x00 0x01 0x00 0x00 0x05 0x15 0x00 0x04 0xa3 0x80 0x00 0x16 0xe3 0x60 CRC:<0xd8>
[ INFO] [1555502889.420488646]: Receiving: <STX>sWA LMPoutputRange <ETX>
[ INFO] [1555502889.420719836]: Sending  : <STX><STX><STX><STX><Len=0018>sRN LMPoutputRange CRC:<0x5e>
[ INFO] [1555502889.421994443]: Receiving: <STX>sRA LMPoutputRange \x00\x01\x00\x00\x05\x15\x00\x04\xa3\x80\x00\x16\xe3\x60<ETX>
[ INFO] [1555502889.422165198]: Angle resolution of scanner is 0.13010 [deg]  (in 1/10000th deg: 0x515)
[ INFO] [1555502889.424815945]: MIN_ANG (after command verification):   -1.040 [rad]  -59.600 [deg]
[ INFO] [1555502889.424901901]: MAX_ANG (after command verification):    1.047 [rad]   60.000 [deg]
[ INFO] [1555502889.425102725]: Sending  : <STX><STX><STX><STX><Len=0032>sWN LMDscandatacfg 0x1f 0x00 0x01 0x01 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x01 CRC:<0x5c>
[ INFO] [1555502889.426373088]: Receiving: <STX>sWA LMDscandatacfg <ETX>
[ INFO] [1555502889.426606493]: Sending  : <STX><STX><STX><STX><Len=0018>sRN LMDscandatacfg CRC:<0x67>
[ INFO] [1555502889.427933309]: Receiving: <STX>sRA LMDscandatacfg \x1f\x00\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x01<ETX>
[ INFO] [1555502889.430654546]: Sending  : <STX><STX><STX><STX><Len=0018>sWN FREchoFilter 0x00 CRC:<0x7f>
[ INFO] [1555502889.431952374]: Receiving: <STX>sWA FREchoFilter <ETX>
[ INFO] [1555502889.432180430]: Sending  : <STX><STX><STX><STX><Len=0016>sMN LMCstartmeas CRC:<0x68>
[ INFO] [1555502889.963840302]: Receiving: <STX>sAN LMCstartmeas \x00<ETX>
[ INFO] [1555502889.964083670]: Sending  : <STX><STX><STX><STX><Len=0007>sMN Run CRC:<0x19>
[ INFO] [1555502889.965558914]: Receiving: <STX>sAN Run \x01<ETX>
[ INFO] [1555502889.965813465]: Sending  : <STX><STX><STX><STX><Len=0017>sEN LMDscandata 0x01 CRC:<0x33>
[ INFO] [1555502889.967297195]: Receiving: <STX>sEA LMDscandata \x01<ETX>
```

## Driver features (legacy platforms and devices)

### Field evaluation information

LMS1xx, LMS5xx, TiM7xx and TiM7xxS support field monitoring. Fields can be configured by SOPAS ET. Once they are configured, sick_scan_xd publishes ROS messages containing the monitoring information from the lidar.

By default, field monitoring is enabled in the launch files [sick_lms_1xx.launch](launch/sick_lms_1xx.launch),  [sick_lms_5xx.launch](launch/sick_lms_5xx.launch),
[sick_tim_7xx.launch](launch/sick_tim_7xx.launch) resp. [sick_tim_7xxS.launch](launch/sick_tim_7xxS.launch) by following settings:

```xml
<param name="activate_lferec" type="bool" value="True"/> <!-- activate field monitoring by lferec messages -->
<param name="activate_lidoutputstate" type="bool" value="True"/> <!-- activate field monitoring by lidoutputstate messages -->
<param name="activate_lidinputstate" type="bool" value="True"/> <!-- activate field monitoring by lidinputstate messages -->
```

The driver queries the field configuration from the lidar and activates field monitoring by sending cola commands `"sEN LFErec 1"` and `"sEN LIDoutputstate 1"` at startup. Field monitoring is deactivated when driver exits. During runtime, it's possible to query, activate or deactivate monitoring using ROS service ColaMsg with the following command (see section [Cola commands](#cola-commands)):

```sh
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sEN LFErec 1'}" # activate LFErec messages
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sEN LFErec 0'}" # deactivate LFErec messages
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sRN LFErec'}"   # query activation status of LFErec messages
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sEN LIDoutputstate 1'}" # activate LIDoutputstate messages
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sEN LIDoutputstate 0'}" # deactivate LIDoutputstate messages
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sRN LIDoutputstate'}"   # query activation status of LIDoutputstate messages
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sEN LIDinputstate 1'}"  # activate LIDinputstate messages
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sEN LIDinputstate 0'}"  # deactivate LIDinputstate messages
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sRN LIDinputstate'}"    # query activation status of LIDinputstate messages
```

LFErec and LIDoutputstate messages are defined in [LFErecMsg.msg](msg/LFErecMsg.msg) and [LFErecFieldMsg.msg](msg/LFErecFieldMsg.msg) resp. [LIDoutputstateMsg.msg](msg/LIDoutputstateMsg.msg) and published on the following topics: `"/sick_tim_7xxS/lferec"` resp. `"/sick_tim_7xxS/lidoutputstate"`.

| **Lidar** | **lferec topic**      | **lidoutputstate topic**      |
| --------- | --------------------- | ----------------------------- |
| lms_1xx   | /sick_lms_1xx/lferec  | /sick_lms_1xx/lidoutputstate  |
| lms_5xx   | /sick_lms_5xx/lferec  | /sick_lms_5xx/lidoutputstate  |
| lms_7xx   | /sick_tim_7xx/lferec  | /sick_tim_7xx/lidoutputstate  |
| lms_7xxS  | /sick_tim_7xxS/lferec | /sick_tim_7xxS/lidoutputstate |

To view the field monitoring messages, run

```sh
rostopic echo "/sick_lms_1xx/lferec"
rostopic echo "/sick_lms_1xx/lidoutputstate"
rostopic echo "/sick_lms_5xx/lferec"
rostopic echo "/sick_lms_5xx/lidoutputstate"
rostopic echo "/sick_tim_7xx/lferec"
rostopic echo "/sick_tim_7xx/lidoutputstate"
rostopic echo "/sick_tim_7xxS/lferec"
rostopic echo "/sick_tim_7xxS/lidoutputstate"
```

or use rviz to visualize monitored fields and their status (see section [Visualization with rviz](#visualization-with-rviz))

The most important values of the field monitoring messages are

- `field_index` (uint8) and `field_result_mrs` (uint8) for each field of a LFErec message with result status
  - 0: invalid / incorrect,
  - 1: free / clear, or
  - 2: infringed.

- `output_state` (uint8) for each LIDoutputstate message with status 0 (not active), 1 (active) or 2 (not used).

> **_NOTE:_** Field monitoring currently supports binary cola messages only, which is the default. If cola ascii is activated, please switch back to cola binary for field monitoring.

#### Visualization with rviz

The point cloud, the monitored fields and their status can be visualized using rviz. Use the [rviz configuration file](test/emulator/config/rviz_emulator_cfg.rviz) and run

```sh
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg.rviz
```

Otherwise you can just add visualizations of type `/cloud/PointCloud2` and `/sick_tim_7xxS/marker` (resp. `/sick_tim_1xx/marker` for lms_1xx,  `/sick_tim_5xx/marker` for lms_5xx and  `/sick_tim_7xx/marker` for tim_7xx):

![tim7xxs_screenshot01.png](doc/tim7xxs_screenshot01.png)

The following screenshot shows a TiM781S example with 2 fields (the 3. field is not configured), the first field with status "Clear", the second with status "Infringed":

![tim7xxs_screenshot02.png](doc/tim7xxs_screenshot02.png)

The following screenshot shows a LMS511 example with a segmented field, two rectangular fields and a dynamic fields:

![lms511_screenshot01.png](doc/lms511_screenshot01.png)

> **_NOTE:_** Some combinations of rviz, OpenGL 3, VMware and graphic card drivers may cause visualization issues. In case of missing markers, try rviz with Open GL 2 using the command

```sh
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg.rviz --opengl 210
```

#### Cola commands

Cola commands can be sent for diagnosis and development using the ROS service ColaMsg. This service is implemented in sick_scan_xd and started by

```xml
<param name="start_services" type="bool" value="True"/>
```

in the launch file [sick_tim_7xxS.launch](launch/sick_tim_7xxS.launch) (resp. [sick_lms_1xx.launch](launch/sick_lms_1xx.launch), [sick_lms_5xx.launch](launch/sick_lms_5xx.launch), [sick_tim_7xx.launch](launch/sick_tim_7xx.launch) ). The ROS service sends the given cola command to the lidar and returns its response.

Example for cola command `"sRN SCdevicestate"` and response `"sRA SCdevicestate \\x00"` with error status 0 (no error):

```xml
rosservice call /sick_tim_7xx/ColaMsg "{request: 'sRN SCdevicestate'}"
response: "sRA SCdevicestate \\x00"
```

#### Emulation

sick_scan_emulator implements a simple test server for cola commands. It receives Cola-commands, returns TiM781S-like responses and sends scan data from a json-file. Run

```xml
roslaunch sick_scan_xd emulator_01_default.launch
```

to emulate a local Tim781S device. Then start and connect the sick_scan_xd driver by

```sh
roslaunch sick_scan_xd sick_tim_7xxS.launch hostname:=127.0.0.1
```

Note that sick_scan_emulator just implements a simple server for offline tests. It does not emulate a lidar device completely and should only be used for development and testing.

Scandata messages are parsed from json-file(s). These json-files are configured in the launch file [emulator.launch](test/emulator/launch/emulator_01_default.launch) and converted form wireshark-records (pcapng-files) using pcap_json_converter.py (see section Pcapng converter tool](#pcapng-converter-tool)).

A LMS111 device can be emulated by running

```sh
roslaunch sick_scan_xd emulator_lms1xx.launch &
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_lms1xx.rviz &
roslaunch sick_scan_xd sick_lms_1xx.launch hostname:=127.0.0.1
```

A LMS511 device can be emulated by running

```sh
roslaunch sick_scan_xd emulator_lms5xx.launch &
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_lms5xx.rviz &
roslaunch sick_scan_xd sick_lms_5xx.launch hostname:=127.0.0.1
```

#### Unit tests

Folder `test/emulator/scandata` contains scan data examples for unit tests. To run an offline unit test for LMS111, LMS511, TiM781, TiM781S enter the following commands:

```sh
cd test/scripts
./run_simu_lms1xx.bash
./run_simu_lms5xx.bash
./run_simu_tim7xx_tim7xxS.bash
```

or start emulator, driver and rviz by running

```sh
source ./install/setup.bash
# Start sick_scan_xd emulator
roslaunch sick_scan_xd emulator_01_default.launch &
sleep 1
# Start rviz
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg.rviz --opengl 210 &
sleep 1
# Start sick_scan_xd driver for TiM871S
roslaunch sick_scan_xd sick_tim_7xxS.launch hostname:=127.0.0.1
```

#### Pcapng converter tool

The pcapng converter tool [pcap_json_converter.py](test/pcap_json_converter/pcap_json_converter.py) converts pcapng-files to json-files. Run the following steps to create a json-file with scan data for the emulator:

1. Start wireshark and filter the TCP traffic on port 2112 with the filter expression `tcp and tcp.port==2112`.
2. Start TiM781S and run the sick_scan_xd driver.
3. Capture the network traffic for some time.
4. Stop capturing and save the network traffic in a pcapng-file.
5. Convert the pcapng-file to json by `python pcap_json_converter.py --pcap_filename=<filepath>.pcapng`. Result is a jsonfile `<filepath>.pcapng.json`
6. Set the resulting json-file in the emulator configuration [emulator.launch](test/emulator/launch/emulator_01_default.launch) by `<arg name="scandatafiles" default="<filepath>.pcapng.json"/>`

### Raspberry Pi: Build without internet or GitHub access (ROS 1 / LD-MRS)

Checkout sick_scan_xd and use `scp -rp` to copy files and directories recursively from local host to a Raspberry, e.g.:

On your local Linux PC (Raspberry IP-address is 192.168.178.52 in this example):

```sh
mkdir -p ./sick_scan_xd_raspberry_pi_pretest/src
pushd ./sick_scan_xd_raspberry_pi_pretest/src
git clone https://github.com/SICKAG/libsick_ldmrs.git
git clone -b master https://github.com/SICKAG/sick_scan_xd.git
popd
scp -rp ./sick_scan_xd_raspberry_pi_pretest 192.168.178.52:/home/rostest/sick_scan_xd_raspberry_pi_pretest
```

On your Raspberry Pi (ROS 1):

```sh
cd /home/rostest/sick_scan_xd_raspberry_pi_pretest
pushd ./src/sick_scan_xd/test/scripts
chmod a+x ./*.bash
./makeall_ros1.bash
popd
source ./devel_isolated/setup.bash
```

To view the point cloud on your local Linux PC (with Raspberry IP-address is 192.168.178.52 in this example):

```sh
export ROS_MASTER_URI=http://192.168.178.52:11311/
rviz
```

### Docker support

This file describes how to build and run sick_scan_xd in a docker container on Linux.

Run the following steps to install and run docker on Linux:

1. Install Docker:
   - Follow the instructions on <https://docs.docker.com/desktop/install/ubuntu/>,
   - or (recommended) install Docker without Docker Desktop by running

    ```sh
          pushd /tmp
          curl -fsSL https://get.docker.com -o get-docker.sh
          sudo sh get-docker.sh
          sudo usermod -aG docker $USER
          popd
    ```

2. Reboot
3. Quick test: Run

    ```sh
        docker --version
        docker info
        docker run hello-world
    ```

4. Optionally install pandoc to generate html reports:

    ```sh
      sudo apt-get install pandoc
    ```

5. Optionally start "Docker Desktop" if installed (not required). Note:
   - "Docker Desktop" is not required to build and run sick_scan_xd in docker container, and
   - depending on your system, it runs qemu-system-x86 with high cpu and memory load.

#### Build and run on Linux ROS 1 (short cut)

> [!NOTE]
> Although ROS1 is still mentioned below, active development for ROS1 will not be continued.

Shortcut to build and run sick_scan_xd in a docker container for ROS1 noetic on Linux:

```sh
# Create a workspace folder (e.g. sick_scan_ws or any other name) and clone the sick_scan_xd repository:
mkdir -p ./sick_scan_ws/src
cd ./sick_scan_ws/src
git clone -b master https://github.com/SICKAG/sick_scan_xd.git
# Build and run sick_scan_xd in a linux noetic docker container:
cd sick_scan_xd/test/docker
sudo chmod a+x ./*.bash
./run_dockerfile_linux_ros1_noetic_sick_scan_xd.bash
echo -e "docker test status = $?"
```

After successful build and run, the message **SUCCESS: sick_scan_xd docker test passed** will be displayed.
Otherwise an error message **ERROR: sick_scan_xd docker test FAILED** is printed.

The following chapter gives a more detailed description of the build and run process.

This section describes how to

- build a docker image with ROS 1 noetic on Linux
- build sick_scan_xd in the docker image
- run and test sick_scan_xd in the docker container

sick_scan_xd can be build from local sources, from git sources or it can be installed from prebuilt binaries.

#### Build and run from local sources

The following instructions

- build a docker image with ROS 1 noetic on Linux,
- build sick_scan_xd in the docker image from local sources, and
- run and test sick_scan_xd in the docker container against a multiScan100 emulator.

This is the recommended way for

- testing locally modified sick_scan_xd sources, or
- testing sick_scan_xd with sources cloned from a public github repository, or
- testing sick_scan_xd with sources cloned from a git repository requiring authentication.

Run the following steps to build and run sick_scan_xd for ROS 1 noetic in a docker container on Linux:

1. Create a workspace folder, e.g. `sick_scan_ws` (or any other name):

    ```sh
      mkdir -p ./sick_scan_ws
      cd ./sick_scan_ws
    ```

2. Clone repository <https://github.com/SICKAG/sick_scan_xd>:

    ```sh
      mkdir ./src
      pushd ./src
      git clone -b master https://github.com/SICKAG/sick_scan_xd.git
      popd
    ```

   If you want to test sources from a different branch or repository, just replace the git call resp. the git url.

3. Create a docker image named `sick_scan_xd/ros1_noetic` from dockerfile [src/sick_scan_xd/test/docker/dockerfile_linux_ros1_noetic_sick_scan_xd](dockerfile_linux_ros1_noetic_sick_scan_xd) with local sources in folder `./src/sick_scan_xd`:

    ```sh
      docker build --progress=plain -t sick_scan_xd/ros1_noetic -f ./src/sick_scan_xd/test/docker/dockerfile_linux_ros1_noetic_sick_scan_xd .
      docker images -a # list all docker images
    ```

4. Run docker image `sick_scan_xd/ros1_noetic` and test sick_scan_xd with a simulated multiScan100 lidar:

    ```sh
      # Allow docker to display rviz
      xhost +local:docker
      # Run sick_scan_xd simulation in docker container sick_scan_xd/ros1_noetic
      docker run -it --name sick_scan_xd_container -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY -w /workspace sick_scan_xd/ros1_noetic python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --cfg=./src/sick_scan_xd/test/docker/data/multiscan_compact_test01_cfg.json
      # Check docker exit status (0: success, otherwise error)
      docker_exit_status=$?
      echo -e "docker_exit_status = $docker_exit_status"
      if [ $docker_exit_status -eq 0 ] ; then echo -e "\nSUCCESS: sick_scan_xd docker test passed\n" ; else echo -e "\n## ERROR: sick_scan_xd docker test FAILED\n" ; fi
    ```

    If all tests were passed, i.e. all expected point cloud-, Laserscan- and IMU-messages have been verified, docker returns with exit status 0 and the message `SUCCESS: sick_scan_xd docker test passed` is displayed. Otherwise docker returns with an error code and the message `## ERROR: sick_scan_xd docker test FAILED` is displayed.

5. To optionally cleanup and uninstall all containers and images, run the following commands:

```sh
   docker ps -a -q # list all docker container
   docker stop $(docker ps -a -q)
   docker rm $(docker ps -a -q)
   docker system prune -a -f
   docker volume prune -f
   docker images -a # list all docker images
   # docker rmi -f $(docker images -a) # remove all docker images
```

   This will remove **all** docker logfiles, images, containers and caches.

#### Build and run from a git repository

By default, sick_scan_xd sources are provided in the local folder `./src/sick_scan_xd`. This source folder is just copied into the docker image and used to build sick_scan_xd. This step is executed by the COPY command in the [dockerfile](dockerfile_linux_ros1_noetic_sick_scan_xd):

```dockerfile
COPY ./src/sick_scan_xd /workspace/src/sick_scan_xd
```

This docker command copies the local folder `./src/sick_scan_xd` into the docker image (destination folder in the docker image is `/workspace/src/sick_scan_xd`). This default option is useful to test new sick_scan_xd versions or to test a local copy of sick_scan_xd from sources requiring authorization.

Alternatively, docker can build sick_scan_xd from a git repository like <https://github.com/SICKAG/sick_scan_xd>. A git repository can be set by docker build option `--build-arg SICK_SCAN_XD_GIT_URL=<sick_scan_xd-git-url>`, e.g. `--build-arg SICK_SCAN_XD_GIT_URL=https://github.com/SICKAG/sick_scan_xd`. Replace the `docker build ....` command in step 3 by:

```sh
# Create a docker image named "sick_scan_xd/ros1_noetic" from dockerfile "./sick_scan_xd/test/docker/dockerfile_linux_ros1_noetic_sick_scan_xd" with github sources from https://github.com/SICKAG/sick_scan_xd
docker build --build-arg SICK_SCAN_XD_GIT_URL=https://github.com/SICKAG/sick_scan_xd --progress=plain -t sick_scan_xd/ros1_noetic -f ./src/sick_scan_xd/test/docker/dockerfile_linux_ros1_noetic_sick_scan_xd .
```

If option `SICK_SCAN_XD_GIT_URL` is set, the docker build command in the [dockerfile](dockerfile_linux_ros1_noetic_sick_scan_xd) clones the given repository:

```dockerfile
RUN /bin/bash -c "if [ $SICK_SCAN_XD_GIT_URL != $NONE ] ; then ( pushd /workspace/src ; rm -rf ./sick_scan_xd ; git clone -b master $SICK_SCAN_XD_GIT_URL ; popd ) ; fi"
```

After a successful build, run and test sick_scan_xd in the docker container as described above in step 4:

```sh
xhost +local:docker
docker run -it --name sick_scan_xd_container -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY -w /workspace sick_scan_xd/ros1_noetic python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --cfg=./src/sick_scan_xd/test/docker/data/multiscan_compact_test01_cfg.json
```

#### Build and run from prebuilt binaries

Alternatively, docker can install a prebuilt sick_scan_xd binary using apt-get, e.g. `apt-get install -y ros-noetic-sick-scan-xd`. Use docker build options `--build-arg SICK_SCAN_XD_APT_PKG=ros-noetic-sick-scan-xd --build-arg SICK_SCAN_XD_BUILD_FROM_SOURCES=0` to install a prebuilt sick_scan_xd binary in the docker image, e.g.

```sh
# Create a docker image named "sick_scan_xd/ros1_noetic" from dockerfile "./sick_scan_xd/test/docker/dockerfile_linux_ros1_noetic_sick_scan_xd" with "apt-get install ros-noetic-sick-scan-xd"
docker build --build-arg SICK_SCAN_XD_APT_PKG=ros-noetic-sick-scan-xd --build-arg SICK_SCAN_XD_BUILD_FROM_SOURCES=0 --progress=plain -t sick_scan_xd/ros1_noetic -f ./src/sick_scan_xd/test/docker/dockerfile_linux_ros1_noetic_sick_scan_xd .
```

After a successful build, run and test sick_scan_xd in the docker container as described above in step 4:

```sh
xhost +local:docker
docker run -it --name sick_scan_xd_container -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY -w /workspace sick_scan_xd/ros1_noetic python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --cfg=./src/sick_scan_xd/test/docker/data/multiscan_compact_test01_cfg.json
```

To install other prebuilt sick_scan_xd binaries, replace command `apt-get install -y $SICK_SCAN_XD_APT_PKG` in the [dockerfile](dockerfile_linux_ros1_noetic_sick_scan_xd) by a customized procedure.

### Hector SLAM support

In robotic mapping and navigation, simultaneous localization and mapping (SLAM) is the computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it. For further details please refer to <https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping> .
The following assumes that the SLAM algorithm works with a lidar  mounted on a mobile base. The mobile base (e.g. a robot) records the environment while driving and creates the map from it. The mobile base usually has a so-called inertial measurement unit (IMU). In principle, however, it is also possible to estimate the direction of movement from the chronological sequence of the laser scans by means of correlation observations. The lidar then virtually takes over the task of the IMU and other components (e.g. counting the wheel revolutions). The method of estimating the position and orientation (position estimation) of a mobile system based on data from its driving system is called odometry (cf. <https://en.wikipedia.org/wiki/Odometry>).
The SLAM algorithm hector_slam (<http://wiki.ros.org/hector_slam>) supports odometry estimation directly from the laser scans and is therefore used as a reference implementation in the following.
Other widely used SLAM algorithms such as gmapping (cf. <http://wiki.ros.org/gmapping>) do not have this option. They depend on the data of an IMU. One possibility to use Gmapping nevertheless is the integration of the project laser_scan_matcher (<https://answers.ros.org/question/63457/gmapping-without-odom/> and <http://wiki.ros.org/laser_scan_matcher>).  Here, however, the pose must still be converted into an odometry message (see <https://answers.ros.org/question/12489/obtaining-nav_msgsodometry-from-a-laser_scan-eg-with-laser_scan_matcher/>).

#### NAV350 ROS 1 SLAM example

Build hector_slam and sick_scan_xd:

```sh
cd src
git clone -b master https://github.com/SICKAG/sick_scan_xd.git
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
cd ..
catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DCMAKE_ENABLE_EMULATOR=1 -Wno-dev
```

Run rviz, sick_scan_xd with NAV350 and hector_slam:

```sh
source ./devel_isolated/setup.bash
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_slam_nav350.rviz &
roslaunch sick_scan_xd sick_nav_350.launch hostname:=192.168.0.1 tf_publish_rate:=0 &
roslaunch sick_scan_xd test_200_slam_ros1_hector.launch scan_topic:=/scan scan_layer_0_frame_id:=cloud_POS_000_DIST1 cloud_frame_id:=cloud &
```

> **_NOTE:_**  By default, sick_scan_xd publishes transform (TF) messages, which map frame id "map" to the point cloud frame id. To avoid conflicts with hector SLAM, it is recommended to disable these TF messages by commandline parameter **`tf_publish_rate:=0`** or by setting `<param name="tf_publish_rate" type="double" value="0"/>` in the launch file.

The following rviz screenshot shows an example of a NAV350 point cloud created by sick_scan_xd and its map generated by hector_slam on ROS 1:

![slam_example_ros1_nav350.png](doc/screenshots/slam_example_ros1_nav350.png)

#### NAV350 ROS 2 SLAM example

Install ths ROS 2 slam-toolbox with `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox` (replace `humble` by your ROS distribution).

Build sick_scan_xd for ROS 2 as described in [ROS 2 on Linux](../README.md#ros-2-on-linux).

Run rviz2, sick_scan_xd, slam_toolbox and static transforms:

```sh
source ./install/setup.bash
ros2 run rviz2 rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_slam_nav350.rviz &
ros2 launch sick_scan_xd sick_nav_350.launch.py hostname:=192.168.0.1 tf_publish_rate:=0 &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link cloud  &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link  &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint  &
ros2 launch nav2_bringup navigation_launch.py &
ros2 launch slam_toolbox online_async_launch.py &
```

> **_NOTE:_** Laserscan messages need to be remapped to topic `/scan` (default is `/sick_nav_350/scan`). Use `remappings=[ ('/sick_nav_350/scan', '/scan'), ]` in the launch file, e.g.:

```python
node = Node(
    package='sick_scan',
    executable='sick_generic_caller',
    output='screen',
    remappings=[ ('/sick_nav_350/scan', '/scan'), ], # remap laserscan messages to topic /scan
)
```

The following rviz2 screenshot shows an example of a NAV350 laserscan created by sick_scan_xd and its map generated by slam_toolbox on ROS 2:

![slam_example_ros2_nav350.png](doc/screenshots/slam_example_ros2_nav350.png)

#### picoScan100 ROS 1 SLAM example

Run rviz, sick_scan_xd with picoScan100 and hector_slam:

```sh
source ./devel_isolated/setup.bash
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_slam_multiscan.rviz &
roslaunch sick_scan_xd sick_nav_350.launch hostname:=192.168.0.1 hostname:=127.0.0.1 udp_receiver_ip:=192.168.0.100 tf_publish_rate:=0 &
roslaunch sick_scan_xd test_200_slam_ros1_hector.launch scan_topic:=/sick_picoscan/scan_fullframe scan_layer_0_frame_id:=world_1 cloud_frame_id:=world &
```

Replace IP address `192.168.0.100` with the IP address of your local machine running sick_scan_xd. The following rviz screenshot shows an example of a picoScan100 point cloud created by sick_scan_xd and its map generated by hector_slam on ROS 1:

![slam_example_ros1_picoscan.png](doc/screenshots/slam_example_ros1_picoscan.png)

#### MRS1000 SLAM support

MRS1104 provides 4 layers covering elevation angles at -2.5°, 0.0°, 2.5° and 5.0°. The layer with 0.0° is used for SLAM by default. The following rviz screenshot shows an example of a MRS1104 point cloud created by sick_scan_xd and its map generated by hector_slam on ROS 1:

![slam_example_ros1_mrs1104.png](doc/screenshots/slam_example_ros1_mrs1104.png)

Since Hector-Slam expects only one laser scan frame with a unique identifier for the laser scans, the following parameters were added to the driver.

slam_echo: The name of the echo is entered here, which is filtered out of all possible 12 echoes. This should be "laser_POS_000_DIST1". This exports the first hit in the position with an elevation angle of 0°. If you want to use the layers with elevation angles -2.5°, 2.5° and 5.0°, you can set another flag with the name slam_bundle to True. If this flag is set, the oblique distances are multiplied by the cosine in this direction to obtain the projection onto the XY plane. This quadruples the number of points and increases the scan rate from 12.5 Hz to 50 Hz. However, for oblique impact surfaces (i.e. no vertical walls) this method can lead to larger estimation errors. In this case slam_bundle should be set to false.

### Google cartographer support

The support of Google Cartographer was made possible by a number of extensions to the driver. On the driver side, the MRS1104 is prepared to support the Google Cartographer. The Google Cartographer expects data packets at a high recording density (several hundred packets per second) to perform the SLAM algorithm. For this reason, an option has been introduced that allows the scans to be chopped into small angular ranges. The time stamps for these small ranges were converted accordingly.

Setup Google Cartographer (these steps are for illustration only, you must adapt these lines to your local directory names)

1. Login to Ubuntu.
2. Open multiple terminals.
3. Terminal 1:
   . ros1_start.sh
   roscore
4. Terminal 2:
   . ros1_start.sh
   cd ~/ros_catkin_ws
   source ./devel/setup.bash
5. Terminal 3:
   roslaunch sick_scan_xd sick_mrs_1xxx_cartographer.launch cloud_topic:=horizontal_laser_3d frame_id:=horizontal_vlp16_link
6. Terminal 4:
   roslaunch sick_scan_xd sick_tim_5xx.launch cloud_topic:=vertical_laser_3d frame_id:=vertical_vlp16_link hostname:=192.168.0.71
7. Terminal 5:
   - . ros1_start.sh
   - cd ~/ros_cartographer_ws
   - source ./install_isolated/setup.bash
   - catkin_make_isolated
   - roslaunch cartographer_ros live_demo_backpack_3d.launch

Example output:

The following figure shows an example of an outdoor slam result using a MRS1104:

![slam_example](doc/slam_example.png)

### OctoMap support

[OctoMap](https://github.com/OctoMap) models a 3D occupancy map. The octomap_server builds and distributes volumetric 3D occupancy maps from a 3D point cloud. Tutorials and examples can be found e.g. in [3D Mapping with OctoMap](https://www.arminhornung.de/Research/pub/hornung13roscon.pdf), [octomap_tutorial](https://github.com/tejalbarnwal/octomap_tutorial) and [Basic usage of octomap_mapping](https://www.youtube.com/watch?v=dF2mlKJqkUg). Note that OctoMap is not a fully SLAM algorithm, but it can create 2D and 3D maps from point clouds.

Run the following steps to build and run OctoMap and sick_scan_xd with a multiScan100 lidar on ROS 1:

1. Clone OctoMap + sick_scan_xd:

    ```sh
        pushd src
        git clone https://github.com/SICKAG/sick_scan_xd.git
        git clone https://github.com/OctoMap/octomap_ros.git
        git clone https://github.com/OctoMap/octomap_msgs.git
        git clone https://github.com/OctoMap/octomap_mapping.git
        popd
    ```

2. Set topic and frame_id for multiScan100 in octomap_mapping.launch:

    ```xml
        <param name="frame_id" type="string" value="world" />
        <remap from="cloud_in" to="/cloud_unstructured_fullframe" />
    ```

3. Build:

    ```sh
        rm -rf ./build ./devel ./install ./build_isolated ./devel_isolated ./install_isolated ./log
        catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -Wno-dev
    ```

4. Run OctoMap + sick_scan_xd:

    ```sh
        # Run sick_scan_xd + multiScan100
        roslaunch sick_scan_xd sick_multiscan.launch hostname:="192.168.0.1" udp_receiver_ip:=" 192.168.0.100"
        # Run octomap_server
        roslaunch octomap_server octomap_mapping.launch
    ```

    Replace parameter "hostname" with the IP address of the multiScan100 lidar and "udp_receiver_ip" with the IP address of the PC running sick_scan_xd.

5. Visualize OctoMap with rviz:
  
    - Add MarkerArray topic "/occupied_cells_vis_array“ (colored voxels)
    - Add Map topic "/projected_map“ (gray 2D Projection)

6. Save the OctoMap:

    ```sh
        rosrun octomap_server octomap_saver -f ./octomap_multiscan.bt
    ```

7. Publish the saved OctoMap:

    ```sh
        rosrun octomap_server octomap_server_node ./octomap_multiscan.bt
    ```

    The following screenshot shows an example of an octomap created from a multiScan100 point cloud:

    ![octomap_example_ros1_multiscan](doc/screenshots/octomap_example_ros1_multiscan.png)

### RTAB-Map support

[RTAB-Map](https://introlab.github.io/rtabmap/) (Real-Time Appearance-Based Mapping) is a RGB-D, Stereo and Lidar Graph-Based SLAM approach, which can be used for 3D-SLAM in combination with multiScan100 or other SICK lidars. sick_scan_xd provides a 3D-SLAM example using RTAB-Map with the multiScan100 lidar. The following section describes how to install and run RTAB-Map with sick_scan_xd and a multiScan.

#### Install RTAB-Map on ROS 1

Run the following steps to build rtabmap and sick_scan_xd with on ROS 1:

1. Build the prerequisites for RTAB-Map:

    ```sh
        sudo apt-get install libboost-all-dev
        sudo apt-get install libeigen3-dev
        sudo apt-get install libsdl-image1.2-dev
        sudo apt-get install libsdl-dev
        sudo apt-get install ros-noetic-nav-msgs
        sudo apt-get install ros-noetic-tf2-sensor-msgs
        sudo apt-get install ros-noetic-imu-filter-madgwick
        sudo apt-get install python3-wstool
        sudo apt-get install ros-noetic-scan-tools
        pushd /tmp
        mkdir -p libnabo/build && pushd libnabo/build
        cmake ..
        cmake --build .
        sudo cmake --build . --target install
        popd
        mkdir -p libpointmatcher/build && pushd libpointmatcher/build
        cmake ..
        make -j4
        sudo make install
        popd
        sudo ldconfig
        mkdir -p rtabmap/build && pushd rtabmap/build
        cmake ..
        make -j4
        sudo make install
        popd
        sudo ldconfig
        popd
    ```

2. Build RTAB-Map and sick_scan_xd in your workspace:

    ```sh
        pushd src
        git clone https://github.com/ros-planning/navigation.git
        git clone https://github.com/ros-planning/navigation_msgs.git
        git clone https://github.com/introlab/rtabmap_ros.git
        git clone https://github.com/SICKAG/sick_scan_xd.git
        popd
        rm -rf ./build ./devel ./install ./build_isolated ./devel_isolated ./install_isolated ./log
        catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -Wno-dev
        sudo ldconfig
    ```

    Run `sudo ldconfig` if you encounter errors while loading shared libraries. Note that building rtabmap with libpointermatch is highly recommended.

#### Run RTAB-MAP and multiScan100 on ROS 1

[sick_multiscan_rtabmap.launch](launch/sick_multiscan_rtabmap.launch) provides a launch file to run 3D-SLAM using rtabmap and sick_scan_xd with a multiScan100 lidar. The SLAM parameters are just examples taken from [rtabmap_examples/launch/test_ouster.launch](https://github.com/introlab/rtabmap_ros/blob/master/rtabmap_examples/launch/test_ouster.launch). Run `rosrun rtabmap_slam rtabmap --params` to see all RTAB-Map options, parameters and their meaning and adopt launch file [sick_multiscan_rtabmap.launch](launch/sick_multiscan_rtabmap.launch) if required.

Run the following command to start rtabmap and sick_scan_xd with a multiScan100 lidar:

```sh
source ./install_isolated/setup.bash
roslaunch sick_scan_xd sick_multiscan_rtabmap.launch hostname:="191.168.0.1" udp_receiver_ip:=" 191.168.0.100"
```

Replace parameter "hostname" with the IP address of the multiScan100 lidar and "udp_receiver_ip" with the IP address of the PC running sick_scan_xd. The following screenshot shows an example of RTAB-MAP and a multiScan100 point cloud:

![rtabmap_example_ros1_multiscan](doc/screenshots/rtabmap_example_ros1_multiscan.png)

To visualize SLAM results, add e.g. topics `/rtabmap/grid_map`, `/rtabmap/localization_pose` and `/rtabmap/odom` in rviz.

rtabmap provides services to switch to mapping or localization mode:

```sh
rosservice call /rtabmap/resume                # resume after pause
rosservice call /rtabmap/trigger_new_map       # start a new map
rosservice call /rtabmap/set_mode_mapping      # set mapping mode
rosservice call /rtabmap/set_mode_localization # set localization mode
```

Alternatively, you can use the options in rtabmap-viz:

![rtabmap_viz_options](doc/screenshots/rtabmap_viz_options.png)

> **_NOTE:_** RTAB-Map support for ROS 2 is documented in the active [README.md](../README.md).

## Device specific information (deprecated devices)

### TiMxxx

The **TiM240** has an opening angle of 240 degrees. In contrast to the previous devices from SICK, the coordinate system used corresponds directly to the ROS convention. For this reason, this device does not require a coordinate conversion of 90 degrees around the Z-axis. However, this is taken into account in the driver code, so that the user will not notice any difference in the setting of the angular ranges during use.
The angular position according to the data sheet can be taken from the drawings below. The following figures show the difference between the TiM5xx family and the TiM240 device.

![TiM5xx scan area](doc/tim240/tim_5xx_scanarea.jpg)

**TiM5xx** scanning area

![TiM240 scan area](doc/tim240/tim_240_scanarea.jpg)

**TiM240** scanning area

For the **TiM7x1S** lidars, the initial lidar configuration can be deactivated using optional argument initialize_scanner:=0. Note that this mode does not initialize the lidar. The mode assumes that the device is in an appropriate state corresponding to the properties configured in the launch file. It is not recommended to use this option unless for specific tasks in a controlled environment. **Do not use this mode except the lidar has been configured properly and initialized successfully and is in the same state as after initialization by the launch file! This option is for advanced users only!**
Example: `roslaunch sick_scan_xd sick_tim_7xxS.launch hostname:=192.168.0.1 initialize_scanner:=0`

The field evaluation for **TiM7xx** lidars support two additional options to configure the active field set: FieldSetSelectionMethod and ActiveFieldSet. These options allow to set the active field set during runtime, see the operation manual for details.
Options FieldSetSelectionMethod and ActiveFieldSet can be configured in the launch file and by ROS services "FieldSetRead" and "FieldSetWrite".
Initial values for FieldSetSelectionMethod and ActiveFieldSet can be configured in the launch file, e.g.:

```xml
<param name="active_field_set" type="int" value="-1"/> <!-- set ActiveFieldSet at startup: -1 = do not set (default), index of active field otherwise -->
<param name="field_set_selection_method" type="int" value="-1"/> <!-- set FieldSetSelectionMethod at startup: -1 = do not set (default), 0 = active field selection by digital inputs, 1 = active field selection by telegram -->
```

By default, options FieldSetSelectionMethod and ActiveFieldSet are not written by the driver, i.e. the default values apply (factory defaults or settings by SOPAS ET).

Example service call for ROS 1 to read resp. write options FieldSetSelectionMethod and ActiveFieldSet:

```sh
rosservice call /sick_tim_7xx/FieldSetRead "{}" # returns field_set_selection_method and active_field_set
rosservice call /sick_tim_7xx/FieldSetWrite "{field_set_selection_method_in: -1, active_field_set_in: 1}" # write active_field_set
```

Example service call for ROS 2 to read resp. write options FieldSetSelectionMethod and ActiveFieldSet:

```sh
ros2 service call /FieldSetRead sick_scan_xd/srv/FieldSetReadSrv "{}" # returns field_set_selection_method and active_field_set
ros2 service call /FieldSetWrite sick_scan_xd/srv/FieldSetWriteSrv "{field_set_selection_method_in: -1, active_field_set_in: 1}" # write active_field_set
```

Parameter active_field_set < 0: do not set (default), active_field_set > 0: index of active field otherwise (see operation manual for details about ActiveFieldSet telegram)
Parameter field_set_selection_method < 0: do not set (default), field_set_selection_method = 0: active field selection by digital inputs, field_set_selection_method = 1: active field selection by telegram
Note that FieldSetSelectionMethod (parameter field_set_selection_method) requires a higher authorization level and should be configured in the launch file. It is therefore recommended to set `field_set_selection_method_in: -1` when using ROS service FieldSetWrite.

### NAV350

NAV350 devices are supported by sick_scan_xd since 2023. Since they support navigation and use a different communication mode, this chapter gives an overview of the NAV350 support in sick_scan_xd. Please refer to the manuals for further information.

#### Process loop

Scan data, landmarks and poses of NAV350 devices are queried by SOPAS commands with polling. Therefore the sick_scan_xd process loop runs as followed:

1. Initialization and setup
2. Main loop (polling):
   1. Send data request "sMN mNPOSGetData 1 2"
   2. Receive and parse response
   3. Convert and publish pointcloud, laserscan, landmarks, pose and transform
   4. API: notify listeners and run their callback functions
   5. Repeat from step 1
3. In case of incoming odometry messages (asynchronous):
   1. Convert to SOPAS command
   2. Send "sMN mNPOSSetSpeed <odom_data>" to NAV350

#### Initialization and setup

After initialization, sick_scan_xd switches to navigation mode by default. Navigation requires mapping (i.e. a valid landmark layout), which can be done by

- SOPAS ET (recommended), or
- optional mapping with parameter `nav_do_initial_mapping:=True` using the landmarks detected at start, or
- using an optional imk-file.

Configuration and setup using SOPAS ET is most powerful and recommended. The default SOPAS initialization sequence runs as followed:

```python
"sMN SetAccessMode 3 F4724744"     # switch to access level authorized client
"sMN mNEVAChangeState 1"           # switch to operation mode standby
"sWN NEVACurrLayer <layer>"        # set layer configured by launchfile
"sWN NLMDLandmarkDataFormat 0 1 1" # set result format (landmark data)
"sWN NAVScanDataFormat 1 1"        # set result format (scan data)
"sWN NPOSPoseDataFormat 1 1"       # set result format (pose data)
"sMN mNEVAChangeState 4"           # switch to navigation mode
"sMN mNPOSGetData 1 2"             # query pose, landmark and scan data
```

If optional parameter `nav_do_initial_mapping` is true, a landmark layout is initialized using the reflectors detected at startup (sopas command "sMN mNMAPDoMapping"). The SOPAS initialization sequence for an initial mapping runs as followed:

```python
"sMN SetAccessMode 3 F4724744"     # switch to access level authorized client
"sMN mNEVAChangeState 1"           # switch to operation mode standby
"sWN NEVACurrLayer <layer>"        # set layer configured by launchfile
"sWN NLMDLandmarkDataFormat 0 1 1" # set result format (landmark data)
"sWN NAVScanDataFormat 1 1"        # set result format (scan data)
"sWN NPOSPoseDataFormat 1 1"       # set result format (pose data)
"sMN mNEVAChangeState 2"           # switch to mapping mode
"sMN mNLAYEraseLayout 1"           # clear landmark layout
"sWN NMAPMapCfg ..."               # configure mapping parameter
"sWN NLMDReflSize <size>"          # set reflector size configured by launchfile
"sMN mNMAPDoMapping"               # detect landmarks and run mapping
"sMN mNLAYAddLandmark ..."         # add all detected landmarks to the layout
"sMN mNLAYStoreLayout"             # store landmark layout
"sMN mNEVAChangeState 4"           # switch to navigation mode
"sMN mNPOSGetData 1 2"             # query pose, landmark and scan data
```

The landmark layout stored in an imk-file can optionally loaded at startup with optional parameter. See the NAV350 manual for details about imk-files. The settings are configured in launch file [sick_nav_350.launch](launch/sick_nav_350.launch).

#### Messages

sick_scan_xd polls the NAV350 scan data, reflectors and poses in its main loop. Scan data are published by point cloud messages (in topic "cloud" by default). Reflectors are published by type `sick_scan_xd/NAVLandmarkData` on topic "/sick_nav_350/nav_landmark" and as MarkerArray on topic "/sick_nav_350/nav_reflectors" for easy visualization using rviz. Poses are published by type `sick_scan_xd/NAVPoseData` on topic "/sick_nav_350/nav_pose" and as ROS transform on topic "/tf".
The following rviz-screenshot shows the pointcloud, landmarks and pose of a NAV350:

![nav350_ros1_screenshot2.jpg](doc/nav350_ros1_screenshot2.jpg)

Odometry messages can be sent to the NAV350 device using ROS messages `nav_msgs/Odometry` on topic "/sick_nav_350/odom" or `sick_scan_xd/NAVOdomVelocity` on topic "/sick_nav_350/nav_odom_velocity". Odometry messages `sick_scan_xd/NAVOdomVelocity` specify the velocity (vx, vy) in m/s in lidar coordinates. Odometry messages `nav_msgs/Odometry` specify the velocity (vx, vy) in m/s in ROS coordinates. The angular velocity is expected in radians/s.

Example odometry messages with vx = 1 m/s, vy = -1 m/s and omega: 0.5 rad/s:

```sh
rostopic pub --rate 10 /sick_nav_350/nav_odom_velocity sick_scan_xd/NAVOdomVelocity '{vel_x: 1.0, vel_y: -1.0, omega: 0.5, timestamp: 123456789, coordbase: 0}'
rostopic pub --rate 10 /sick_nav_350/odom nav_msgs/Odometry '{twist: { twist: { linear: {x: 1.0, y: -1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}}'
```

#### Angle compensation

For measurements with the highest demands on the accuracy of the angle measurement, the devices of the NAV series allow compensation of slight angle deviations during a rotation. The compensation is determined by the three parameters.

- Additive compensation by an angle offset
- Sinusoidal correction by specifying the amplitude and phase of compensation

The three parameters are then used to calculate the compensation as follows:

![Formula for angle compensation](doc/angle_compensation/angle_compensation_000.png)

Offset and phase are given in [deg]

##### Example

The information is read from lidar by using the command `sRN MCAngleCompSin`.
The answer gives one amplitude, phase and offset compensation in tens of thousands.

The function reads

- Amplitude-Parameter: +1893
- Phase-Parameter: -210503
- Offset-Parameter: -245

These corresponds to:

- Amplitude-compensation: +0.1893
- Phase-Compensation: -21.0503 [deg]
- Offset-Compensation: -0.0245 [deg]

Compensation formula for example for NAV210/NAV245

```math
Angle[comp.] = Angle[Raw] - 0.189300 * sin(Angle[Raw] + -21.050300 [deg]) -  -0.024500
```

Compensation formula for example for NAV310

```math
Angle[comp.] = Angle[Raw] + 0.189300 * sin(Angle[Raw] - -21.050300 [deg]) +  -0.024500
```

Example lookup values for NAV310 for this example (first entries)

| **IN [Deg]** | **Out [Deg]** | **Correction [Deg]** |
| ------------ | ------------- | -------------------- |
| 0.000000     | 0.043494      | 0.043494             |
| 1.000000     | 1.046567      | 0.046567             |
| 2.000000     | 2.049618      | 0.049618             |
| 3.000000     | 3.052647      | 0.052647             |
| 4.000000     | 4.055652      | 0.055652             |
| 5.000000     | 5.058633      | 0.058633             |
| 6.000000     | 6.061588      | 0.061588             |
| 7.000000     | 7.064518      | 0.064518             |
| 8.000000     | 8.067420      | 0.067420             |
| 9.000000     | 9.070294      | 0.070294             |
| 10.000000    | 10.073139     | 0.073139             |

##### Comparing compensated vs. raw values

For the example the compensation function looks like this (X-Axis: measured angle [deg], Y-Axis: compensation in [deg])

 ![Plot of compensation function (example)](doc/angle_compensation/angle_compensation_001.png)

##### Coordinate systems

For a better understanding of the data sheets of the different lidar systems the following drawings compare the different coordinate systems. Usually the devices rotate counter-clockwise. The NAV3xx series rotate clockwise. All coordinate systems following the right-hand rule, if the axis definition as shown in the picture is used.

![Used coordinate systems](doc/angle_compensation/3d_coordinate_system_comp.png)

By means of simple matrix operations all axis orientations can be transformed into each other. But since we are only interested in the angle around the Z-axis, the conversions can be done as follows (CS = Coordinate System):

| IN From | Out To | Operation    | Additional info                    |
| ------- | ------ | ------------ | ---------------------------------- |
| ROS     | NAV3xx | Out=-In+180° | maps [-180°...180°] to [360°...0°] |
| NAV3xx  | ROS    | Out=-In+180° | maps [0°...360°] to [180°...-180°] |
| ROS     | NAV2XX | Out=In+90°   |                                    |
| NAV2xx  | ROS    | Out=In-90°   |                                    |

##### Check compensation function

By using `Octave` ones can check the compensation function against the given values by exporting the value via a testbed function.

<!--
   40 a = dlmread("angle_compensation_debug.csv",';',1,0);
   41 size(a)
   42 a
   43 mean(a(:,3))
   44 a3 = a(:,3)-mean(a(:,3))
   45 mean(a3)
   46 S3 = fft(a3)
   47 plot(abs(S3))
   48 abs(S3)
   49 S3(1)
   50 S3(2)
   51 abs(S3(2))/360
   52 abs(S3(2))/360*2
   53 angle(S3(2))
   54 angle(S3(2))/pi*180
   55 angle(S3(2))/pi*180 - 90
   56 angle(S3(2))/pi*180 + 90
   57 plot(a(:,1),a(:,3))
   58 title "compensation example"
-->

### MRS6124

The layers are taken up by the device in packs of 6. It delivers at an output data rate of 10 Hz and 24 layers 24/6*10=40 scan packets of 6 layers per second. The following table shows an example of the timing for a complete 24 layer recording

| **Raw Time [µs]** | **Delta Time [µs]** | **Elevation Angle [Deg]** |
| ----------------- | ------------------- | ------------------------- |
| 2551706348        | 0                   | 13.19                     |
| 2551706348        | 0                   | 12.565                    |
| 2551706348        | 0                   | 11.940                    |
| 2551706348        | 0                   | 11.315                    |
| 2551706348        | 0                   | 10.690                    |
| 2551706348        | 0                   | 10.065                    |
| 2551731348        | 25000               | 9.440                     |
| 2551731348        | 25000               | 8.815                     |
| 2551731348        | 25000               | 8.190                     |
| 2551731348        | 25000               | 7.565                     |
| 2551731348        | 25000               | 6.940                     |
| 2551731348        | 25000               | 6.315                     |
| 2551756348        | 50000               | 5.690                     |
| 2551756348        | 50000               | 5.065                     |
| 2551756348        | 50000               | 4.440                     |
| 2551756348        | 50000               | 3.815                     |
| 2551756348        | 50000               | 3.190                     |
| 2551756348        | 50000               | 2.565                     |
| 2551781348        | 75000               | 1.940                     |
| 2551781348        | 75000               | 1.315                     |
| 2551781348        | 75000               | 0.690                     |
| 2551781348        | 75000               | 0.065                     |
| 2551781348        | 75000               | -0.560                    |
| 2551781348        | 75000               | -1.185                    |
| NEW SCAN          |                     |                           |
| 2551807862        | 101514              | 13.190                    |
| 2551807862        | 101514              | 12.565                    |
| 2551807862        | 101514              | 11.940                    |
| 2551807862        | 101514              | 11.315                    |
| 2551807862        | 101514              | 10.690                    |
| 2551807862        | 101514              | 10.065                    |
| 2551832862        | 126514              | 9.440                     |

The time stamps between the layers are interpolated by the scanner. The time stamps of the first layer (Ang.=13.19°) are measured and show jitter accordingly.

### Combination of devices (ROS 1)

#### multiScan100 and picoScan100

The following example shows a multiScan100 and a picoScan100 device running simultaneously on ROS 1. The IP address of the multiScan100 is `192.168.0.1` (default), the IP address of the picoScan100 has been set to `192.168.0.2`. The Linux-PC running sick_scan_xd uses IP address `192.168.0.100`.
`fping -a -q -g 192.168.0.0/24` shows all available devices in subnet `192.168.0.x`:

| **device**    | **IP**       |
| ------------- | ------------ |
| 192.168.0.1   | multiScan100 |
| 192.168.0.2   | picoScan100  |
| 192.168.0.100 | Linux-PC     |

Open 192.168.0.1 and 192.168.0.2 in a browser to view the network settings with SOPASair:

![multiple_lidars_02.png](doc/screenshots/multiple_lidars_02.png)

The frame ids and ROS topics of both lidars should be configured differently. Copy both launch files (sick_multiscan.launch and sick_picoscan.launch in this example) e.g. to lidar1.launch and lidar2.launch and replace ROS topics and frame ids, e.g.

- replace all "topic=/cloud_" by "topic=/cloud1_" in lidar1.launch
- replace all "topic=/cloud_" by "topic=/cloud2_" in lidar2.launch
- replace all "frameid=world" by "frameid=world1" in lidar1.launch
- replace all "frameid=world" by "frameid=world2" in lidar2.launch

![multiple_lidars_03.png](doc/screenshots/multiple_lidars_03.png)

![multiple_lidars_04.png](doc/screenshots/multiple_lidars_04.png)

Provide the launch files with `catkin_make_isolated --install --cmake-args -DROS_VERSION=1`.

Then launch sick_scan_xd twice with two different launch files, IP addresses, node names, UDP ports, topic and frame ids.

Example:

```sh
roslaunch sick_scan_xd lidar1.launch hostname:=192.168.0.1 udp_receiver_ip:=192.168.0.100 nodename:=lidar1 udp_port:=2115 imu_udp_port:=7503 publish_frame_id:=world1 publish_laserscan_segment_topic:=scan1_segment publish_laserscan_fullframe_topic:=scan1_fullframe imu_topic:=imu1 &
```

```sh
roslaunch sick_scan_xd lidar2.launch hostname:=192.168.0.2 udp_receiver_ip:=192.168.0.100 nodename:=lidar2 udp_port:=2116 imu_udp_port:=7504 publish_frame_id:=world2 publish_laserscan_segment_topic:=scan2_segment publish_laserscan_fullframe_topic:=scan2_fullframe imu_topic:=imu2 &
```

Rviz shows the point clouds of both lidars running simultaneously, with frame id `world1` for lidar1 (multiScan) and frame id `world2` for lidar2 (picoScan100):

![multiple_lidars_05.png](doc/screenshots/multiple_lidars_05.png)

![multiple_lidars_06.png](doc/screenshots/multiple_lidars_06.png)

If the 6D poses of the lidars are known, their coordinates can be transformed to a common frame by a static_transform_publisher. Example:

```sh
rosrun tf static_transform_publisher 0 0 0 0 0 0 world world1 100 &
rosrun tf static_transform_publisher 0 0 0 0 0 0 world world2 100 &
```

![multiple_lidars_07.png](doc/screenshots/multiple_lidars_07.png)

The big purple dots show the picoScan100 pointcloud, the other points are the multiScan100 point clouds. Both are transformed to the common frame id `world`. Note that both point clouds do not match exactly, because the 6D poses are just assumed to be (x=0, y=0, z=0, yaw=0, pitch=0, roll=0) in this example.

The multiScan100 and picoScan100 scans can be visualized by rviz. The following screenshots show two examples of a multiScan100 pointcloud:

![msgpacks-emulator-rviz](doc/20210929-tokenized-msgpacks-emulator-rviz.png)

![msgpacks-multiscan-rviz.png](doc/20210929-tokenized-msgpacks-multiscan-rviz.png)

Note that sick_scan_xd publishes 2 pointclouds:

- The point cloud on topic `/cloud` is published for each scan segment.
- The point cloud on topic `/cloud_fullframe` collects all segments for a complete 360 degree full scan (360 degree for multiScan100, 276 degree for picoscan100).

Pointcloud callbacks defined in the [API](../README.md#generic-driver-api) are called the same way: A callback registered with SickScanApiRegisterPolarPointCloudMsg is called

- with a segment_idx >= 0 for each scan segment, and
- with segment_idx := -1 for the complete 360 degree full scan.

#### RMS1000 and MRS6000

1. Setup environment and power supply
2. roslaunch sick_scan_xd test_0002_combi_live.launch
3. Check setup using rviz
4. Close all applications, which are not necessary (like IDE, browser, git client)
5. Setup Tracking algorithm

    ```sh
      top
    ```

6. Record data

```sh
   rosrun rosbag record record -o combi -a
```

#### RMS1000 and LMS1000

This tutorial shows how to combine a RMS1000 radar with a LMS1000 lidar.
To demonstrate the lidar/radar combination, a RMS1000 and a LMS1000 device were put into operation. The sick_scan_xd driver and rviz were started on ROS 1 Linux. Bagfiles have been recorded to demonstrate the required transform (rms_1xxx_lms_1xx_movement_off.bag and rms_1xxx_lms_1xx_movement_on.bag).
Run the following steps:

1. Connect RMS1000 and LMS1000 and start sick_scan_xd with launch files sick_lms_1xxx.launch and sick_rms_xxxx.launch:

    ```sh
      roslaunch sick_scan_xd sick_lms_1xxx.launch
      roslaunch sick_scan_xd sick_rms_xxxx.launch
    ```

   Make sure, that different ROS node names and different IP-addresses are used.
   The following rviz screenshot shows both pointclouds:

   ![rms_1xxx_lms_1xx_combi_screenshot01.png](doc/combination_rms_1xxx_lms_1xx/rms_1xxx_lms_1xx_combi_screenshot01.png)

   Note that each sensor has its own frame id and coordinate system. The RMS1000uses the frame id "radar", the LMS1000 uses the frame id "cloud". To combine both sensor, we have to transform the radar frame and coordinates to the lidar frame and coordinates.
   Radar targets have multiple echos due to reflection.

2. Start a ROS static_transform_publisher to convert radar frames (frame id `/radar`) to lidar frames (frame id `/cloud`):

    ```sh
    rosrun tf static_transform_publisher 0 0 0 0 0 0 /cloud /radar 100
    ```

    Using this transform, rviz displays both the radar and lidar pointcloud:

    ![rms_1xxx_lms_1xx_combi.png](doc/combination_rms_1xxx_lms_1xx/rms_1xxx_lms_1xx_combi.png)

    > [!NOTE]
    > If you use this example with a playback of bagfiles (e.g. `rosbag play --loop ./rms_1xxx_lms_1xx_movement_off.bag`), you might encounter errors due to different timestamps (the recorded timestamps in the bagfiles are different from the timestamps by the static_transform_publisher).

    Alternatively, the radar frame id and an optional transform can be configured in the radar launch file (parameter "frame_id" and "add_transform_xyz_rpy").

## FAQ (deprecated-specific)

Also see closed issues: <https://github.com/SICKAG/sick_scan_xd/issues>

### Why are my changes in launch files are ignored?

roslaunch still uses an old version after modifying the launch file.
After modifying a launch file, it has to be installed by running `catkin_make_isolated --install --cmake-args -DROS_VERSION=1` to be located and used by `roslaunch`.

### How can I create a ROS 2 node in python to run sick_generic_caller from a launch.py file?

Example to launch a TiM7xx node in ROS 2:

```python
    sick_scan_pkg_prefix = get_package_share_directory('sick_scan_xd')
    tim_launch_file_path = os.path.join(sick_scan_pkg_prefix, 'launch/sick_tim_7xx.launch')
    tim_top_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        output='screen',
        arguments=[
            tim_launch_file_path,
            'nodename:=/lidars/tim_top',
            'hostname:=192.168.0.110',
            'cloud_topic:=/lidars/tim_top/cloud',
            'frame_id:=tim_top'
        ]
    )
```

Thanks to user JWhitleyWork.

### catkin reports "By not providing "FindSICKLDMRS.cmake" ..." . What can I do?

Full error: `By not providing "FindSICKLDMRS.cmake" in CMAKE_MODULE_PATH this project ..., but CMake did not find one."`

ROS doesn't automatically rebuild everything if you just append "-DLMRRS=0". If `catkin_make_isolated --install --cmake-args -DROS_VERSION=1` was applied,
remove the build/devel/install-directories created by the ROS build process. For this, run the following commands to remove the directories, which holds the previous build results:

```sh
cd ~/ros_catkin_ws
rm -rf build_isolated
rm -rf devel_isolated
rm -rf install_isolated
rm -rf devel
```

It is possible that not all directories are present in this list. The only subdirectory left should be "src".
You can check this with the following command:

```sh
ls */ -d
```

The output should be:

```sh
src/
```

After doing this please rerun the command

```sh
catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DLDMRS=0
```

### How can I debug sick_generic_caller on ROS 1?

Build with compiler option `-g` and run sick_generic_caller as described using a launch file. Stop sick_generic_caller (Ctrl-C or kill) and save the current ROS parameter using `rosparam dump <dumpfile>.yaml`. Load these parameter with `rosparam load <dumpfile>.yaml` and restart sick_generic_caller in gdb or in your IDE.
