<img align=right width="200" src="https://upload.wikimedia.org/wikipedia/commons/thumb/f/f1/Logo_SICK_AG_2009.svg/1200px-Logo_SICK_AG_2009.svg.png"/>

Executive Summary
---

This documentation is intended to provide background information on the maintenance and extension of the repository.

Table of Contents
---

- [Adding a new device to the driver](#adding-a-new-device-to-the-driver)
  - [Naming Convention](#naming-convention)
  - [Launch Files](#launch-files)
  - [Code Modification](#code-modification)
- [Bloom release](#bloom-release)
  - [First time installation of toolchain](#first-time-installation-of-toolchain)
  - [Release build for ROS 1](#release-build-for-ros-1)
  - [Release build for ROS 2](#release-build-for-ros-2)
  - [Check status](#check-status)
  - [Useful links and information](#useful-links-and-information)
- [Testing](#testing)
  - [Unit tests](#unit-tests)
  - [Examples](#examples)
- [Simulation](#simulation)
  - [Windows](#windows)
  - [Linux](#linux)
- [Profiling](#profiling)
  - [Installation](#installation)
  - [Usage](#usage)

# Adding a new device to the driver

This driver is designed to support several different scanner types (including radar) from Sick. This documentation describes how to add additional devices to the driver.

## Naming Convention

For each device type a name pattern is assigned as follows:
``
sick_<device family>_<identifier>
``

The name type is used in the code to decide which scanner-specific parameters are set.
The name type is passed as a parameter as follows:
```
<param name="scanner_type" type="string" value="sick_lms_5xx" />
```

## Launch Files

A launch file is created for each device type,
which usually has the same naming convention as the scanner type.
To create a new device, it is recommended to copy, rename and edit an existing launch file.

## Code Modification

1. Hint: Construction of parser:
    ```
    sick_scan_xd::SickGenericParser *parser = new sick_scan_xd::SickGenericParser(scannerName);
    ```
2. Add string constant like the constant SICK_SCANNER_RMS_XXXX_NAME

3. Append this constant to allowedScannerNames
   like allowedScannerNames.push_back(SICK_SCANNER_RMS_XXXX_NAME);
   in the file sick_generic_parser.cpp

4. Add new parameter block like
	```
	if (basicParams[i].getScannerName().compare(SICK_SCANNER_MRS_1XXX_NAME) == 0)
	{...
	} in the file sick_generic_parser.cpp
	```

5. Copy the file sick_generic_radar.cpp and add a new class following the structure
of this file.

# Bloom release

## First time installation of toolchain

1. Install on Linux:
    * Install bloom:
        ```
        sudo apt-get update
        sudo apt-get install python3-bloom python3-catkin-pkg
        ```
    * Install docker:
        ```
        pushd /tmp
        curl -fsSL https://get.docker.com -o get-docker.sh
        sudo sh get-docker.sh
        sudo usermod -aG docker $USER
        popd
        shutdown -r now # reboot
        # short quicktest
        docker --version
        docker info
        docker run hello-world
        ```
    * Install ros-buildfarm:
        ```
        # sudo apt-get install python3-ros-buildfarm # not successfully, unable to locate
        pip3 install ros-buildfarm # installs ros-buildfarm 3.0 successfully
        ```

2. Build the prerelease:
    * Short version to build a prerelase:
        * Run the following commands:
            ```
            mkdir -p ./ws_sick_scan_xd_bloom/src
            cd ./ws_sick_scan_xd_bloom/src
            git clone -b master https://github.com/SICKAG/sick_scan_xd.git
            cd ./sick_scan_xd/test/scripts
            ./run_linux_ros1_bloom.bash
            ```
        * Fix any errors during the prerelease build and check in
        * Repeat `./run_linux_ros1_bloom.bash` until the the prerelease build finishes without errors
    * Alternative version:
        * Open http://prerelease.ros.org/noetic in the brower
        * Add a custom repository: `sick_scan_xd` , `https://github.com/SICKAG/sick_scan_xd` , `master` (or `feature/bloom_pretest` or any other branch to test)
        * Add a custom repository: `msgpack11` , `https://github.com/SICKAG/msgpack11` , `master`
        * Add a custom repository: `libsick_ldmrs` , `https://github.com/SICKAG/libsick_ldmrs` , `master`
        * Confirm next steps (i.e. URL of build farm: https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config/production/index.yaml, Ubuntu focal)
        * Click on `Generate command`
        * Run the generated command, i.e.:
            ```
            source /opt/ros/noetic/setup.bash
            mkdir -p /tmp/prerelease_job
            cd /tmp/prerelease_job
            generate_prerelease_script.py \
              https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config/production/index.yaml \
              noetic default ubuntu focal amd64 \
              --custom-repo \
                sick_scan_xd:git:https://github.com/SICKAG/sick_scan_xd:master \
                msgpack11:git:https://github.com/SICKAG/msgpack11:master \
                libsick_ldmrs:git:https://github.com/SICKAG/libsick_ldmrs:master \
              --level 1 \
              --output-dir ./
            ```
        * Run `printf "\033c" ; rm -rf ~/.ccache ; mkdir -p ~/.ccache ; ./prerelease.sh` in folder `/tmp/prerelease_job`
        * In case of error message `/usr/lib/ccache/cc is not able to compile a simple test program`:
            * Remove folder `~/.ccache` before running `./prerelease.sh`
            * See https://answers.ros.org/question/347063/error-pre-release-melodic/
        * Fix any errors during the prerelease build and check in
        * Remove the temporary build folder by `rm -rf /tmp/prerelease_job`
        * Repeat until `prerelease.sh` finishes without errors.

3. Submit package sick_scan_xd for indexing (noetic)
    * Fork `https://github.com/ros/rosdistro` -> `https://github.com/<username>/rosdistro.git`
    * `git clone https://github.com/<username>/rosdistro.git`
    * Edit file `rosdistro/noetic/distribution.yaml` and add after `sick_scan`:
        ```
        sick_scan_xd:
          doc:
            type: git
            url: https://github.com/SICKAG/sick_scan_xd.git
            version: master
        ```
    * `cd rosdistro ; source /opt/ros/noetic/setup.bash ; rosdistro_reformat file://"$(pwd)"/index.yaml`
    * git commit: `git commit -m "Adding sick_scan_xd to documentation index for distro noetic" distribution.yaml`
    * git push: `git push origin master`
    * Submit a pull request on `https://github.com/<username>/rosdistro`

4. For ROS 2 humble: Follow instructions on https://docs.ros.org/en/humble/How-To-Guides/Releasing/Releasing-a-Package.html

**_NOTE:_** Bloom releases for ROS 2 foxy are not longer supported (Pull request failed, "This pull request changes files for a ROS distribution that is no longer supported (End Of Life)")

    * Submit package sick_scan_xd for indexing (ROS 2 humble)
        * Reset fork `https://github.com/<username>/rosdistro.git` to origin/master or delete the fork and create a new one -> `https://github.com/<username>/rosdistro.git`
        * `git clone https://github.com/<username>/rosdistro.git`
        * Edit file `rosdistro/humble/distribution.yaml` and add after `sick_safevisionary_ros2`:
            ```
            sick_scan_xd:
              doc:
                type: git
                url: https://github.com/SICKAG/sick_scan_xd.git
                version: develop
              status: developed
            ```
        * git commit and push ("Adding sick_scan_xd to documentation index for distro humble")
        * Submit a pull request on `https://github.com/<username>/rosdistro`
        * Do the same for any new ROS 2 version, e.g. iron and jazzy (`rosdistro/iron/distribution.yaml`, `rosdistro/jazzy/distribution.yaml`)
    * [Start a new release team](https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new?assignees=&labels=&template=new_release_team.md&title=Add+release+team)
        * ROS 2 sick_scan_xd team: https://github.com/orgs/ros2-gbp/teams/sick_scan_xd
        * ROS 2 sick_scan_xd release repository: https://github.com/ros2-gbp/sick_scan_xd-release

## Release build for ROS 1

* Build a prerelease (dry run in a docker container):
    * Run the following commands:
        ```
        git clone -b master https://github.com/SICKAG/sick_scan_xd.git
        cd ./sick_scan_xd/test/scripts
        sudo dos2unix ./*.bash ; sudo chmod a+x ./*.bash
        ./run_linux_ros1_bloom.bash
        ```
    * Fix any errors during the prerelease build and check in
    * Repeat `./run_linux_ros1_bloom.bash` until the the prerelease build finishes without errors

* Build a binary release: follow https://wiki.ros.org/bloom/Tutorials/FirstTimeRelease
    * Update version number in package.xml, minor version number should be incremented at least
    * Create resp. update CHANGELOG.rst:
        ```
        source /opt/ros/noetic/setup.bash
        cd ./src/sick_scan_xd
        rm ./CHANGELOG.rst
        catkin_generate_changelog --all # create CHANGELOG.rst
        ```
    * Commit and pull all changes incl. CHANGELOG.rst and package.xml:
        ```
        git add CHANGELOG.rst package.xml
        git commit -m "Update CHANGELOG.rst and package version"
        git push
        ```
    * Run `catkin_prepare_release` and `bloom-release` in folder `src/sick_scan_xd`:
        ```
        source /opt/ros/noetic/setup.bash
        catkin_prepare_release -y
        bloom-release --rosdistro noetic --track noetic sick_scan_xd # at first time: call with option --edit for configuration
        ```
    * For the initial release (first time): Run `bloom-release` in folder `src/sick_scan_xd` with option `--edit`:
        ```
        source /opt/ros/noetic/setup.bash
        catkin_prepare_release -y
        bloom-release --rosdistro noetic --track noetic sick_scan_xd --edit
        Release repository url: https://github.com/SICKAG/sick_scan_xd-release.git
        upstream: <default, i.e. press ENTER>
        Upstream Repository URI: https://github.com/SICKAG/sick_scan_xd.git
        Upstream VCS Type: <default: git, i.e. press ENTER>
        Version: <default: auto, i.e. press ENTER>
        Release Tag: <default: version, i.e. press ENTER>
        Upstream Devel Branch: feature/bloom_pretest
        ROS Distro: noetic
        Patches Directory: <default: none, i.e. press ENTER>
        Release Repository Push URL:  <default: none, i.e. press ENTER>
        ```
    * Check status: https://index.ros.org/p/sick_scan_xd/#noetic
    * Install binary release: `sudo apt update ; sudo apt-get install ros-noetic-sick-scan-xd`. Note from https://wiki.ros.org/bloom/Tutorials/FirstTimeRelease : Packages built are periodically synchronized over to the shadow-fixed and public repositories, so it might take as long as a month before your package is available on the public ROS debian repositories (i.e. available via apt-get).

## Release build for ROS 2

For ROS 2 follow the instructions on https://docs.ros.org/en/humble/How-To-Guides/Releasing/Releasing-a-Package.html :
* Checkout the sick_scan_xd version to be released and run:
    ```
    git clone -b master https://github.com/SICKAG/sick_scan_xd.git
    cd ./sick_scan_xd
    rm ./CHANGELOG.rst
    catkin_generate_changelog --all # create CHANGELOG.rst
    ```
* Commit CHANGELOG.rst and optional modifications:
    ```
    git add CHANGELOG.rst
    git commit -m "Update CHANGELOG.rst"
    git push
    ```
* Run `catkin_prepare_release` and `bloom-release`:
    ```
    bloom-release --rosdistro humble --track humble sick_scan_xd # at first time: call with option --new-track
    ```
    For the initial release (i.e. at the first time): Run bloom-relase configuration with option --new-track:
    `bloom-release --new-track --rosdistro humble --track humble sick_scan_xd`
    * Release repository url: https://github.com/ros2-gbp/sick_scan_xd-release.git
    * Upstream: <default>
    * Upstream Repository URI: https://github.com/SICKAG/sick_scan_xd.git
    * Upstream Devel Branch: develop
    * ROS Distro: humble
    After the initial release has been approved: Run
    ```
    sudo rosdep init
    rosdep update
    ```

## Check status
Jenkins build status:
* ROS 1 noetic jenkins build status: https://build.ros.org/job/Ndev__sick_scan_xd__ubuntu_focal_amd64/lastBuild/
* ROS 2 humble jenkins build status: https://build.ros2.org/job/Hdev__sick_scan_xd__ubuntu_jammy_amd64/lastBuild/
* ROS 2 iron   jenkins build status: https://build.ros2.org/job/Idev__sick_scan_xd__ubuntu_jammy_amd64/lastBuild/
* ROS 2 jazzy  jenkins build status: https://build.ros2.org/job/Jdev__sick_scan_xd__ubuntu_noble_amd64/lastBuild/
* ROS 1 jenkins: https://build.ros.org/search/?q=sick_scan_xd
* ROS 2 jenkins: https://build.ros2.org/search/?q=sick_scan_xd

Release repositories:
* ROS 1 release repository: https://github.com/SICKAG/sick_scan_xd-release
* ROS 2 release repository: https://github.com/ros2-gbp/sick_scan_xd-release.git


Show version and list information about prebuilt binaries:
```
sudo apt update
sudo apt show ros-noetic-sick-scan-xd
sudo apt show ros-humble-sick-scan-xd
sudo apt show ros-iron-sick-scan-xd
sudo apt show ros-jazzy-sick-scan-xd
```

Installation of prebuilt binaries:
```
sudo apt update
sudo apt-get install ros-noetic-sick-scan-xd
sudo apt-get install ros-humble-sick-scan-xd
sudo apt-get install ros-iron-sick-scan-xd
sudo apt-get install ros-jazzy-sick-scan-xd
sudo apt-get remove ros-noetic-sick-scan-xd
sudo apt-get remove ros-humble-sick-scan-xd
sudo apt-get remove ros-iron-sick-scan-xd
sudo apt-get remove ros-jazzy-sick-scan-xd
```


## Useful links and information

* http://wiki.ros.org/bloom
* https://wiki.ros.org/bloom/Tutorials/FirstTimeRelease
* https://docs.ros.org/en/humble/How-To-Guides/Releasing/Releasing-a-Package.html


**Bloom builds an old sick_scan_xd version (ROS 1)**

Check `devel_branch` in https://github.com/SICKAG/sick_scan_xd-release/blob/master/tracks.yaml . If devel_branch is an old branch, replace it with e.g. `develop` or `master`, or update the `<devel_branch>` to a new version.

**Bloom builds an old sick_scan_xd version (ROS 2)**
Check `devel_branch` in https://github.com/ros2-gbp/sick_scan_xd-release/blob/master/tracks.yaml . If devel_branch is an old branch, replace it with e.g. `develop` or `master`, or update the `<devel_branch>` to a new version.

**Bloom builds a new sick_scan_xd version, but apt still installs an old version**

  * Check the sick_scan_xd version in the release repositories https://github.com/SICKAG/sick_scan_xd-release.git (ROS 1) and https://github.com/ros2-gbp/sick_scan_xd-release.git (ROS 2)
  * Install bloom (if not yet done) using `sudo apt-get install python-bloom` on Linux or `pip install -U bloom` on Windows
  * Run
        ```
        bloom-release --rosdistro noetic -d sick_scan_xd # release repository: https://github.com/SICKAG/sick_scan_xd-release.git, argument -d enables debug infos
        bloom-release --rosdistro humble -d sick_scan_xd # release repository: https://github.com/ros2-gbp/sick_scan_xd-release.git, argument -d enables debug infos
        bloom-release --rosdistro iron   -d sick_scan_xd # release repository: https://github.com/ros2-gbp/sick_scan_xd-release.git, argument -d enables debug infos
        bloom-release --rosdistro jazzy  -d sick_scan_xd # release repository: https://github.com/ros2-gbp/sick_scan_xd-release.git, argument -d enables debug infos
        ```
  * In case of github 2FA errors: Follow http://wiki.ros.org/bloom/Tutorials/GithubManualAuthorization to create a 2FA token and configure the token in file `~/.config/bloom`.
  * Note: Updates of release repository https://github.com/SICKAG/sick_scan_xd-release.git require github authentification via ssh. See https://docs.github.com/en/authentication/connecting-to-github-with-ssh and https://wiki.ros.org/bloom/Tutorials/GithubManualAuthorization for details.

# Testing

## Unit tests

For a quick unit test after installation without the sensor hardware, a test server is provided to simulate a scanner. It implements a simple tcp server, which responds to binary cola messages and sends predefined LMDscandata to a tcp-client. The sick_scan_xd driver can connect to the local test server instead of the lidar device for offline-tests. Please note, that this test server does not emulate a Lidar sensor. It just sends some simple scan data and response messages to a tcp client. It can be used for a quick unit test after build and install.

To build the test server, activate cmake option `ENABLE_EMULATOR` in CMakeLists.txt and rebuild sick_scan_xd. By default, option `ENABLE_EMULATOR` is switched off.

For a unit test of LMS1xx, run the following commands in different terminals:

```
cd sick_scan_xd
source ./install/setup.bash

# Start sick_scan_xd emulator
roslaunch sick_scan_xd emulator_lms1xx.launch &
sleep 1

# Start rviz
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_lms1xx.rviz &
sleep 1

# Start sick_scan_xd driver
roslaunch sick_scan_xd sick_lms_1xx.launch hostname:=127.0.0.1
```

For a unit test of LMS5xx, run the following commands in different terminals:

```
cd sick_scan_xd
source ./install/setup.bash

# Start sick_scan_xd emulator
roslaunch sick_scan_xd emulator_lms5xx.launch &
sleep 1

# Start rviz
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_lms5xx.rviz &
sleep 1

# Start sick_scan_xd driver
roslaunch sick_scan_xd sick_lms_5xx.launch hostname:=127.0.0.1
```

For a unit test of LMS7xx, run the following commands in different terminals:

```
cd sick_scan_xd
source ./install/setup.bash

# Start sick_scan_xd emulator
roslaunch sick_scan_xd emulator_01_default.launch &
sleep 1

# Start rviz
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg.rviz &
sleep 1

# Start sick_scan_xd driver
roslaunch sick_scan_xd sick_tim_7xx.launch hostname:=127.0.0.1
```

For a unit test of LMS7xxS, run the following commands in different terminals:

```
cd sick_scan_xd
source ./install/setup.bash

# Start sick_scan_xd emulator
roslaunch sick_scan_xd emulator_01_default.launch &
sleep 1

# Start rviz
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg.rviz &
sleep 1

# Start sick_scan_xd driver
roslaunch sick_scan_xd sick_tim_7xxS.launch hostname:=127.0.0.1
```

Alternatively, you can just run the test scripts provided in folder `sick_scan_xd/test/scripts`:

```
cd sick_scan_xd/test/scripts
./makeall.bash
./run_simu_lms1xx.bash
./run_simu_lms5xx.bash
./run_simu_tim7xx_tim7xxS.bash
```

Make sure to finish all sick_scan_xd nodes after a test. All nodes can be killed by
```
rosnode kill -a ; sleep 1
killall sick_generic_caller ; sleep 1
killall sick_scan_emulator ; sleep 1
```

## Examples

rviz example screenshots using sick_scan_xd with LMS1xx and LMS5xx test server:

![emulator_lms1xx_screenshot.png](doc/emulator_lms1xx_screenshot.png)

rviz example screenshots using sick_scan_xd with LMS7xx and LMS7xxS test server:

![emulator_lms1xx_screenshot.png](doc/emulator_lms7xx_screenshot.png)

Further examples are provided in folder `test/scripts`.

# Simulation

For unittests without sensor hardware, a simple test server is provided. To build the test server, call either cmake with option `-DCMAKE_ENABLE_EMULATOR=1`, or activate cmake option `ENABLE_EMULATOR` in CMakeLists.txt. Then rebuild sick_scan_xd. By default, option `ENABLE_EMULATOR` is switched off.

Please note that this just builds a simple test server for basic unittests of sick_scan_xd drivers. Its purpose is to run basic tests and to help with diagnosis in case of issues. It does not emulate a real scanner!

Simulation requires jsoncpp. Install with `sudo apt-get install libjsoncpp-dev` on Linux and with `vcpkg install jsoncpp:x64-windows` on Windows.

You can find examples to test and run sick_scan_xd in offline mode in folder `test/scripts`. Their purpose is to demonstrate the usage of the sick_scan_xd driver. Please feel free to customize the scripts or use them as a starting point for own projects.

## Windows

Run script `run_simu_lms_5xx.cmd` in folder `test/scripts` or execute the following commands:

1. Start the test server:
    ```
    cd .\build
    start "testserver" cmd /k python ../test/emulator/test_server.py --scandata_file=../test/emulator/scandata/20210302_lms511.pcapng.scandata.txt --scandata_frequency=20.0 --tcp_port=2112
    @timeout /t 1
    ```

2. Run sick_generic_caller. On native Windows:
    ```
    .\Debug\sick_generic_caller.exe ../launch/sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False
    ```
    On Windows with ROS 2:
    ```
    ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False
    ```

3. Open file `image_viewer.html` in folder `demo` in your browser to view a jpg-image of the current scan.

Note, that python version 3 incl. runtime dlls must be accessable, f.e. by extending the PATH environment variable:
```
set PYTHON_DIR=%ProgramFiles(x86)%/Microsoft Visual Studio/Shared/Python37_64
set PATH=%PYTHON_DIR%;%PYTHON_DIR%/Scripts;c:\vcpkg\installed\x64-windows\bin;%PATH%
```

Further examples are provided in folder `test/scripts`.

## Linux

Run script `run_simu_lms_5xx.bash` in folder `test/scripts` or execute the following commands:

1. Start the test server:
    ```
    python3 ./test/emulator/test_server.py --scandata_file=./test/emulator/scandata/20210302_lms511.pcapng.scandata.txt --scandata_frequency=20.0 --tcp_port=2112 &
    sleep 1
    ```

2. Run sick_generic_caller.
    - On native Linux:
         ```
        ./build/sick_generic_caller ./launch/sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
        ```
    - On Linux with ROS 1:
         ```
        roslaunch sick_scan_xd sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
        ```
    - On Linux with ROS 2:
         ```
        ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/sick_lms_5xx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
        ```

3. View the point cloud.
    - On native Linux:<br>
         Open file `image_viewer.html` in folder `demo` in a browser (f.e. firefox) to view a jpg-image of the current scan.
    - On Linux with ROS 1:
         ```
        rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_lms5xx.rviz &
        ```
    - On Linux with ROS 2:
         ```
        rviz2 -d ./src/sick_scan_xd/test/emulator/config/rviz2_lms5xx.rviz &
        ```

Further examples are provided in folder `test/scripts`.

# Profiling

Since the existing node can basically be used on different platforms, bottlenecks can occur with weak hardware. To better analyze these bottlenecks, software profiling can be performed.
The following example shows how to perform profiling.
For further details on profiling, please refer to https://baptiste-wicht.com/posts/2011/09/profile-c-application-with-callgrind-kcachegrind.html, for example.

## Installation

First of all, you need to install Callgrind and KCachegrind.
You also need to install graphviz in order to view the call graph in KCachegrind. The applications are already packaged for the most important Linux distributions. You can just use apt-get to install them:
```
sudo apt-get install valgrind kcachegrind graphviz
```
## Usage
We have to start by profiling the application with Callgrind. To profile an application with Callgrind, you just have to prepend the Callgrind invocation in front of your normal program invocation:
```
valgrind --tool=callgrind program [program_options]
```
In order to establish a reference to the source code during profiling, the program must be compiled with debug symbols, this can be done with catkin_make
```
catkin_make install -DCMAKE_BUILD_TYPE=Debug
```
It is necessary to create a rosmaster so that the sick_scan_xd node can connect to it because we can't use roslaunch during profiling.
```
roscore
```
To set the parameters we start a node as usual with roslaunch.
```
roslaunch sick_scan_xd sick_lms_5xx.launch hostname:=192.168.0.151
```
While this node is running we can use ```ps -aef| grep sick_scan_xd``` to determine the program path and the call parameters.
```
rosuser@ROS-NB:~$ ps -aef|grep sick_scan_xd
rosuser   4839  2443  0 14:43 pts/1    00:00:00 /usr/bin/python /opt/ros/melodic/bin/roslaunch sick_scan_xd sick_lms_5xx.launch hostname:=192.168.0.151
rosuser   4854  4839  1 14:43 ?        00:00:03 /home/rosuser/ros_catkin_ws/devel/lib/sick_scan_xd/sick_generic_caller __name:=sick_lms_5xx __log:=/home/rosuser/.ros/log/f9861670-304c-11e9-9839-54e1ad2921b6/sick_lms_5xx-1.log
rosuser   4910  4875  0 14:46 pts/6    00:00:00 grep --color=auto sick_scan_xd
```
now we can close the node and restart with callgrid
```
valgrind --tool=callgrind program /home/rosuser/ros_catkin_ws/devel/lib/sick_scan_xd/sick_generic_caller __name:=sick_lms_5xx
```
The result will be stored in a callgrind.out.XXX file where XXX will be the process identifier.
You can read this file using a text editor, but it won't be very useful because it's very cryptic.
That's here that KCacheGrind will be useful. You can launch KCacheGrind using command line
or in the program menu if your system installed it here. Then, you have to open your profile file.

The first view present a list of all the profiled functions. You can see the inclusive
and the self cost of each function and the location of each one.

![src_view.png](doc/src_view.png)

Once you click on a function, the other views are filled with information. The view in uppper right part of the window gives some information about the selected function.

![profile_002](doc/profile_002.png)

The view have several tabs presenting different information:

* Types : Present the types of events that have been recorded. In our case, it's not really interesting, it's just the number of instructions fetch
* Callers : List of the direct callers.
* All Callers : List of all the callers, it seems the callers and the callers of the callers.
* Callee Map : A map of the callee, personally, I do not really understand this view, but it's a kind of call graph representing the cost of the functions.
* Source code : The source code of the function if the application has been compiled with the debug symbol.

And finally, you have another view with data about the selected function.

![profile_003](doc/profile_003.png)

Again, several tabs:

* Callees : The direct callees of the function
* Call Graph : The call graph from the function to the end
* All Callees : All the callees and the callees of the callees
* Caller Map : All functions are represented as blocks the size corresponds to their CPU time. Callees are stacked on the callers.
* Machine Code : The machine code of the function if the application has been profiled with --dump-instr=yes option

You have also several display options and filter features to find exactly what you want and display it the way you want.

The information provided by KCacheGrind can be very useful to find which functions takes too much time or which functions are called too much.
This text is an adopted version of https://baptiste-wicht.com/posts/2011/09/profile-c-application-with-callgrind-kcachegrind.html . Thanks to Baptiste Wicht.