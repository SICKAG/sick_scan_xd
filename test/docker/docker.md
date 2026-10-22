# Build and run sick_scan_xd in a docker container

This file describes how to build and run sick_scan_xd in a docker container on Linux.

## Table of Contents

- [Install docker on Linux](#install-docker-on-linux)
- [Build and run all sick_scan_xd dockertests on Linux](#build-and-run-all-sick_scan_xd-dockertests-on-linux)
- [Testing](#testing)
- [Install docker on Windows](#install-docker-on-windows)
- [Build and run sick_scan_xd dockertests on Windows](#build-and-run-sick_scan_xd-dockertests-on-windows)

## Install docker on Linux

Run the following steps to install and run docker on Linux:

1. Install Docker: 
   * Follow the instructions on https://docs.docker.com/desktop/install/ubuntu/,
   * or (more recommended) install Docker without Docker Desktop by running
      ```
      pushd /tmp
      curl -fsSL https://get.docker.com -o get-docker.sh
      sudo sh get-docker.sh
      sudo usermod -aG docker $USER
      popd
      ```

2. Reboot
    
3. Quicktest: Run 
    ```
    docker --version
    docker info
    docker run hello-world
    ```

4. Optionally install pandoc to generate html reports:
    ```
   sudo apt-get install pandoc
    ```

5. Optionally start "Docker Desktop" if installed (not required). Note:
   * "Docker Desktop" is not required to build and run sick_scan_xd in docker container, and
   * depending on your system, it runs qemu-system-x86 with high cpu and memory load.


## Build and run all sick_scan_xd dockertests on Linux 

Script `run_all_dockertests_linux.bash` in folder `sick_scan_xd/test/docker` builds docker images for x64, ROS1 and ROS2 on Linux und runs all dockertests:

```
# Create a workspace folder (e.g. sick_scan_ws or any other name) and clone the sick_scan_xd repository:
mkdir -p ./sick_scan_ws/src
cd ./sick_scan_ws/src
git clone -b develop https://github.com/SICKAG/sick_scan_xd.git
# Build and run all sick_scan_xd docker images and tests:
cd sick_scan_xd/test/docker
sudo chmod a+x ./*.bash
./run_all_dockertests_linux.bash
echo -e "docker test status = $?"
```

After successful build and run, the message **SUCCESS: sick_scan_xd docker test passed** will be displayed. 
Otherwise an error message **ERROR: sick_scan_xd docker test FAILED** is printed.

Dockerfiles to create linux docker images are provided in folder sick_scan_xd/test/docker/dockerfiles:

|Dockerfile|System|
|----------|------|
| dockerfile_linux_x64_develop | ubuntu 22.04 + cmake + python modules |
| dockerfile_linux_ros1_noetic_develop | ubuntu 20.04 + ROS1 noetic + python modules |
| dockerfile_linux_ros2_humble_develop | ubuntu 22.04 + ROS2 humble + python modules |
| dockerfile_sick_scan_xd/linux_x64 | linux_x64_develop + sick_scan_xd |
| dockerfile_sick_scan_xd/ros1_noetic | linux_ros1_noetic_develop + sick_scan_xd |
| dockerfile_sick_scan_xd/ros2_humble | linux_ros2_humble_develop + sick_scan_xd |

The following chapter gives a more detailed description of the build and run process for ROS1/Linux. The same process applies accordingly for dockertests of the C++ API, the Python API and ROS-2.

### How to build and run sick_scan_xd for ROS1 noetic in a docker container on Linux 

This section describes how to
* build a docker image with ROS1 noetic on Linux
* build sick_scan_xd in the docker image
* run and test sick_scan_xd in the docker container

sick_scan_xd can be build from local sources, from git sources or it can be installed from prebuilt binaries.

#### Build and run with sick_scan_xd from local sources

The following instructions
* build a docker image with ROS1 noetic on Linux,
* build sick_scan_xd in the docker image from local sources, and
* run and test sick_scan_xd in the docker container against a multiScan emulator.

This is the recommended way for
* testing locally modified sick_scan_xd sources (e.g. pre-release tests), or 
* testing sick_scan_xd with sources cloned from a public gitlab or github repository (e.g. branch tests), or
* testing sick_scan_xd with sources cloned from a git repository requiring authentification.

Run the following steps to build and run sick_scan_xd for ROS1 noetic in a docker container on Linux:

1. Create a workspace folder, e.g. `sick_scan_ws` (or any other name):
   ```
   mkdir -p ./sick_scan_ws
   cd ./sick_scan_ws
   ```

2. Clone repository https://github.com/SICKAG/sick_scan_xd:
   ```
   mkdir ./src
   pushd ./src
   git clone -b master https://github.com/SICKAG/sick_scan_xd.git
   popd
   ```
   If you want to test sources from a different branch or repository, just replace the git call resp. the git url. If you want to test sources not yet released, just provide the modified sources in the src-folder.

3. Create a docker image named `sick_scan_xd/ros1_noetic` from dockerfile [src/sick_scan_xd/test/docker/dockerfiles/dockerfile_linux_ros1_noetic_sick_scan_xd](dockerfiles/dockerfile_linux_ros1_noetic_sick_scan_xd) with local sources in folder `./src/sick_scan_xd`:
   ```
   docker build --progress=plain -t sick_scan_xd/ros1_noetic -f ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_linux_ros1_noetic_sick_scan_xd .
   docker images -a # list all docker images
   ```

4. Run docker image `sick_scan_xd/ros1_noetic` and test sick_scan_xd with a simulated multiScan lidar:
   ```
   # Allow docker to display rviz
   xhost +local:docker
   # Run sick_scan_xd simulation in docker container sick_scan_xd/ros1_noetic
   docker run -it --name sick_scan_xd_container -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY -w /workspace sick_scan_xd/ros1_noetic python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --cfg=./src/sick_scan_xd/test/docker/data/multiscan_compact_test01_cfg.json
   # Check docker exit status (0: success, otherwise error)
   docker_exit_status=$?
   echo -e "docker_exit_status = $docker_exit_status"
   if [ $docker_exit_status -eq 0 ] ; then echo -e "\nSUCCESS: sick_scan_xd docker test passed\n" ; else echo -e "\n## ERROR: sick_scan_xd docker test FAILED\n" ; fi
   ```
    If all tests were passed, i.e. all expected Pointcloud-, Laserscan- and IMU-messages have been verified, docker returns with exit status 0 and the message `SUCCESS: sick_scan_xd docker test passed` is displayed. Otherwise docker returns with an error code and the message `## ERROR: sick_scan_xd docker test FAILED` is displayed.

5. To optionally cleanup and uninstall all containers and images, run the following commands:
   ```
   docker ps -a -q # list all docker container
   docker stop $(docker ps -a -q)
   docker rm $(docker ps -a -q)
   docker system prune -a -f
   docker volume prune -f
   docker images -a # list all docker images
   # docker rmi -f $(docker images -a) # remove all docker images
   ```
   This will remove **all** docker logfiles, images, containers and caches.

#### Build and run with sick_scan_xd sources from a git repository

By default, sick_scan_xd sources are provided in the local folder `./src/sick_scan_xd`. This source folder is just copied into the docker image and used to build sick_scan_xd. This step is executed by the COPY command in the [dockerfile](dockerfiles/dockerfile_linux_ros1_noetic_sick_scan_xd):
```
COPY ./src/sick_scan_xd /workspace/src/sick_scan_xd
```
This docker command copies the local folder `./src/sick_scan_xd` into the docker image (destination folder in the docker image is `/workspace/src/sick_scan_xd`). This default option is useful to test new sick_scan_xd versions or release candidates before check in, or to test a local copy of sick_scan_xd from sources requiring authorization.

Alternatively, docker can build sick_scan_xd from a public git repository like https://github.com/SICKAG/sick_scan_xd. A git repository can be set by docker build option `--build-arg SICK_SCAN_XD_GIT_URL=<sick_scan_xd-git-url>`, e.g. `--build-arg SICK_SCAN_XD_GIT_URL=https://github.com/SICKAG/sick_scan_xd`. Replace the `docker build ....` command in step 3 by:

```
# Create a docker image named "sick_scan_xd/ros1_noetic" from dockerfile "./sick_scan_xd/test/docker/dockerfiles/dockerfile_linux_ros1_noetic_sick_scan_xd" with github sources from https://github.com/SICKAG/sick_scan_xd
docker build --build-arg SICK_SCAN_XD_GIT_URL=https://github.com/SICKAG/sick_scan_xd --progress=plain -t sick_scan_xd/ros1_noetic -f ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_linux_ros1_noetic_sick_scan_xd .
```

If option `SICK_SCAN_XD_GIT_URL` is set, the docker build command in the [dockerfile](dockerfiles/dockerfile_linux_ros1_noetic_sick_scan_xd) clones the given repository:
```
RUN /bin/bash -c "if [ $SICK_SCAN_XD_GIT_URL != $NONE ] ; then ( pushd /workspace/src ; rm -rf ./sick_scan_xd ; git clone -b master $SICK_SCAN_XD_GIT_URL ; popd ) ; fi"
```

After a successful build, run and test sick_scan_xd in the docker container as described above in step 4:
```
xhost +local:docker
docker run -it --name sick_scan_xd_container -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY -w /workspace sick_scan_xd/ros1_noetic python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --cfg=./src/sick_scan_xd/test/docker/data/multiscan_compact_test01_cfg.json
```

#### Build and run with prebuilt sick_scan_xd binaries

Alternatively, docker can install a prebuild sick_scan_xd binary using apt-get, e.g. `apt-get install -y ros-noetic-sick-scan-xd`. Use docker build options `--build-arg SICK_SCAN_XD_APT_PKG=ros-noetic-sick-scan-xd --build-arg SICK_SCAN_XD_BUILD_FROM_SOURCES=0` to install a prebuild sick_scan_xd binary in the docker image, e.g.
```
# Create a docker image named "sick_scan_xd/ros1_noetic" from dockerfile "./sick_scan_xd/test/docker/dockerfiles/dockerfile_linux_ros1_noetic_sick_scan_xd" with "apt-get install ros-noetic-sick-scan-xd"
docker build --build-arg SICK_SCAN_XD_APT_PKG=ros-noetic-sick-scan-xd --build-arg SICK_SCAN_XD_BUILD_FROM_SOURCES=0 --progress=plain -t sick_scan_xd/ros1_noetic -f ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_linux_ros1_noetic_sick_scan_xd .
```

After a successful build, run and test sick_scan_xd in the docker container as described above in step 4:
```
xhost +local:docker
docker run -it --name sick_scan_xd_container -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY -w /workspace sick_scan_xd/ros1_noetic python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --cfg=./src/sick_scan_xd/test/docker/data/multiscan_compact_test01_cfg.json
```

To install other prebuilt sick_scan_xd binaries, replace command `apt-get install -y $SICK_SCAN_XD_APT_PKG` in the [dockerfile](dockerfiles/dockerfile_linux_ros1_noetic_sick_scan_xd) by a customized procedure.

## Testing

The docker container currently supports sick_scan_xd testing with ROS-1 noetic on Linux with a multiScan, picoScan and MRS-1xxx emulators. Python script [sick_scan_xd_simu.py](python/sick_scan_xd_simu.py) runs the following steps to verify sick_scan_xd:
It
* starts a tiny sopas test server to emulate sopas responses of a multiScan, picoScan or MRS-1xxx,
* launches sick_scan_xd,
* starts rviz to display pointclouds and laserscan messages,
* replays UDP packets with scan data, which previously have been recorded and converted to json-file,
* receives the pointcloud-, laserscan- and IMU-messages published by sick_scan_xd,
* compares the received messages to predefined reference messages,
* checks against errors and verifies complete and correct messages, and
* returns status 0 for success or an error code in case of any failures.

The following screenshot shows an example of a successful sick_scan_xd multiScan test in a docker container:

![dockertest_noetic_sick_scan_xd_multiscan.png](screenshots/dockertest_noetic_sick_scan_xd_multiscan.png)

#### Test configuration

Test cases are configured by jsonfile. Currently sick_scan_xd provides configuration files for docker tests for the following lidars:
* multiScan docker test: [multiscan_compact_test01_cfg.json](data/multiscan_compact_test01_cfg.json)
* picoScan docker test: [picoscan_compact_test01_cfg.json](data/picoscan_compact_test01_cfg.json)
* LMS-1xx docker test: [lms1xx_test01_cfg.json](data/lms1xx_test01_cfg.json)
* LMS-1xxx docker test: [lms1xxx_test01_cfg.json](data/lms1xxx_test01_cfg.json)
* LMS-4xxx docker test: [lms4xxx_test01_cfg.json](data/lms4xxx_test01_cfg.json)
* LMS-5xx docker test: [lms5xx_test01_cfg.json](data/lms5xx_test01_cfg.json)
* LRS-36x0 docker test: [lrs36x0_test01_cfg.json](data/lrs36x0_test01_cfg.json)
* LRS-36x1 docker test: [lrs36x1_test01_cfg.json](data/lrs36x1_test01_cfg.json)
* LRS-4xxx docker test: [lrs4xxx_test01_cfg.json](data/lrs4xxx_test01_cfg.json)
* MRS-1xxx docker test: [mrs1xxx_test01_cfg.json](data/mrs1xxx_test01_cfg.json)
* MRS-6xxx docker test: [mrs6xxx_test01_cfg.json](data/mrs6xxx_test01_cfg.json)
* NAV-1xxx docker test: [nav350_test01_cfg.json](data/nav350_test01_cfg.json)
* OEM-15xx docker test: [oem15xx_test01_cfg.json](data/oem15xx_test01_cfg.json)
* RMS-xxxx docker test: [rmsxxxx_test01_cfg.json](data/rmsxxxx_test01_cfg.json)
* TiM-240 docker test: [tim240_test01_cfg.json](data/tim240_test01_cfg.json)
* TiM-4xx docker test: [tim4xx_test01_cfg.json](data/tim4xx_test01_cfg.json)
* TiM-5xx docker test: [tim5xx_test01_cfg.json](data/tim5xx_test01_cfg.json)
* TiM-7xx docker test: [tim7xx_test01_cfg.json](data/tim7xx_test01_cfg.json)
* TiM-7xxS docker test: [tim7xxs_test01_cfg.json](data/tim7xxs_test01_cfg.json)

To execute these test cases in a linux noetic docker container, run the following example commands:
```
# Test sick_scan_xd against multiScan emulator
docker run -it --name sick_scan_xd_container_multiscan_compact_test01 -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY -w /workspace sick_scan_xd/ros1_noetic python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --cfg=./src/sick_scan_xd/test/docker/data/multiscan_compact_test01_cfg.json
# Test sick_scan_xd against multiScan emulator
docker run -it --name sick_scan_xd_container_picoscan_compact_test01 -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY -w /workspace sick_scan_xd/ros1_noetic python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --cfg=./src/sick_scan_xd/test/docker/data/picoscan_compact_test01_cfg.json
# Test sick_scan_xd against MRS-1xxx emulator
docker run -it --name sick_scan_xd_container_mrs1xxx_test01 -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY -w /workspace sick_scan_xd/ros1_noetic python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --cfg=./src/sick_scan_xd/test/docker/data/mrs1xxx_test01_cfg.json
```

Use the configuration files as a template for new test cases resp. different lidars if required.

Visualization with rviz is deactivated by default. Use sick_scan_xd_simu.py option `--run_rviz=1` to activate rviz visualization for development or debugging.

#### Configuration of error testcases

In addition to the test cases above, some exemplary error test cases are provided for docker tests:
* multiScan error test: [multiscan_compact_errortest01_cfg.json](data/multiscan_compact_errortest01_cfg.json) - this test case simulates a multiScan error (lidar does not send scan data)
* picoScan error test: [picoscan_compact_errortest01_cfg.json](data/picoscan_compact_errortest01_cfg.json) - this test case simulates a network error (picoScan not reachable, no tcp connection and lidar does not respond to sopas requests)

Note that these configurations causes the docker test to fail and result in the status **TEST FAILED**.

#### Data export from docker container

Use command `docker cp <container_id>:<src_path> <local_dst_path>` to export all sick_scan_xd logfiles from the docker container:
```
# Export sick_scan_xd output folder from docker container to local folder ./log/sick_scan_xd_simu
docker ps -a # list all container
mkdir -p ./log
docker cp $(docker ps -aqf "name=sick_scan_xd_container_multiscan_compact_test01"):/workspace/log/sick_scan_xd_simu ./log
docker cp $(docker ps -aqf "name=sick_scan_xd_container_picoscan_compact_test01"):/workspace/log/sick_scan_xd_simu ./log
docker cp $(docker ps -aqf "name=sick_scan_xd_container_mrs1xxx_test01"):/workspace/log/sick_scan_xd_simu ./log
```

After successfull test and data export, the sick_scan_xd container can be deleted by
```
docker rm $(docker ps -aqf "name=sick_scan_xd_container_multiscan_compact_test01")
docker rm $(docker ps -aqf "name=sick_scan_xd_container_picoscan_compact_test01")
docker rm $(docker ps -aqf "name=sick_scan_xd_container_mrs1xxx_test01")
```

Alternatively, use command `docker rm $(docker ps -aq)` to delete all docker container.

#### Test reports

Each docker test saves logfiles and jsonfiles and generates a test report in logfolder `/workspace/log/sick_scan_xd_simu/<date_time>` with `<date_time>` in format `<YYYYMMDD_hhmmss>`. Script `run_dockerfile_linux_ros1_noetic_sick_scan_xd.bash` runs all docker tests and converts the test reports to html with pandoc. Use the commands above to copy the logfolder from the docker container to your local host and open file `log/sick_scan_xd_simu/sick_scan_xd_testreport.md.html` in a browser. The following screenshot shows a test summary and a test report of a successful multiscan docker test:

![docker_test_summary_example01.png](screenshots/docker_test_summary_example01.png)
![docker_test_report_example01.png](screenshots/docker_test_report_example01.png)

#### Data preparation

Both scan data and reference messages must be prepared and provided for testing sick_scan_xd. 

Scan data are normally recorded by wireshark and converted to json. They are replayed by [multiscan_pcap_player.py](../python/multiscan_pcap_player.py) resp.[sopas_json_test_server.py](../python/sopas_json_test_server.py) during testing to emulate a lidar.

Reference messages are used to verify the pointcloud-, laserscan- and optional IMU-messages published by sick_scan_xd. They can be generated by saving the messages of a successful test. In this case the messages must be manually checked e.g. by using rviz. Make sure that the messages saved as a reference are correct, i.e. rviz displays the pointclouds and laserscans correctly and consistent with the real scene observed by the lidar.

This section describes how to record and convert these data.

##### Scan data preparation

Run the following steps to prepare scan data for sick_scan_xd testing:

1. Record scan data using wireshark:
   * Install sick_scan_xd
   * Connect the lidar, e.g. multiscan
   * Start wireshark
   * Run sick_scan_xd, e.g. on Linux with ROS-1 by `roslaunch sick_multiscan.launch hostname:="192.168.0.1" udp_receiver_ip:="192.168.0.100"`
   * Check pointclouds and laserscans with `rviz`
   * Save the network traffic in a pcapng-file by wireshark

2. For multiScan and picoScan: Play the pcapng-file for a short time (e.g. 1 second) using [multiscan_pcap_player.py](../python/multiscan_pcap_player.py) and save udp packets in a json file. 
   
   Example to convert the recorded pcapng-file `20231009-multiscan-compact-imu-01.pcapng` to json-file `multiscan_compact_test01_udp_scandata.json`:
   ```
   python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20231009-multiscan-compact-imu-01.pcapng --udp_port=-1 --repeat=1 --verbose=0 --filter=pcap_filter_multiscan_hildesheim --max_seconds=1 --save_udp_jsonfile=./src/sick_scan_xd/test/docker/data/multiscan_compact_test01_udp_scandata.json
   ```

   Example to convert the recorded pcapng-file `20230911-picoscan-compact.pcapng` to json-file `picoscan_compact_test01_udp_scandata.json`:
   ```
   python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20230911-picoscan-compact.pcapng --udp_port=-1 --repeat=1 --verbose=0 --filter=pcap_filter_multiscan_hildesheim --max_seconds=1 --save_udp_jsonfile=./src/sick_scan_xd/test/docker/data/picoscan_compact_test01_udp_scandata.json
   ```

3. For lidars using SOPAS LMDscandata over TCP (e.g. MRS-1xxx): Convert the the pcapng-file to json file with [pcap_json_converter.py](../pcap_json_converter/pcap_json_converter.py).
   Example for MRS-1xxx:
   ```
   python3 ./src/sick_scan_xd/test/pcap_json_converter/pcap_json_converter.py --pcap_filename=./src/sick_scan_xd/test/emulator/scandata/20211201_MRS_1xxx_IMU_with_movement.pcapng
   mv ./src/sick_scan_xd/test/emulator/scandata/20211201_MRS_1xxx_IMU_with_movement.pcapng.json ./src/sick_scan_xd/test/docker/data/mrs1xxx_test01_tcp_data.json
   ``` 

##### Generation of reference messages

Run the following steps to generate reference messages for sick_scan_xd testing:

1. Build sick_scan_xd for ROS1 as described in [Build on Linux ROS1](../../INSTALL-ROS1.md#build-on-linux-ros1) and start environment:
   ```
   source /opt/ros/noetic/setup.bash
   source ./devel_isolated/setup.bash
   roscore &
   rviz &
   ```

2. Run [sick_scan_xd_simu.py](./python/sick_scan_xd_simu.py) and save messages in a json-file.

   Example for a multiScan testcase:
   ```
   python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --save_messages_jsonfile=received_messages.json --cfg=./src/sick_scan_xd/test/docker/data/multiscan_compact_test01_cfg.json
   ```
   Example for a picoScan testcase:
   ```
   python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --save_messages_jsonfile=received_messages.json --cfg=./src/sick_scan_xd/test/docker/data/picoscan_compact_test01_cfg.json
   ```
   Example for a MRS-1xxx testcase:
   ```
   python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --save_messages_jsonfile=received_messages.json --cfg=./src/sick_scan_xd/test/docker/data/mrs1xxx_test01_cfg.json --run_seconds=5
   ```

3. Make sure that the messages saved as a reference are correct, i.e. rviz displays the pointclouds and laserscans correctly and consistent with the real scene observed by the lidar. For lidars with IMU data support, use `rostopic echo <imu topic>` to verify plausible IMU data. Use parameter `--run_seconds=<sec>` to run the simulation as long as needed for the manual verification. For multiScan and picoScan, increase parameter `./src/sick_scan_xd/test/python/multiscan_pcap_player.py --repeat=<number of repetitions> ...` in the configuration file to run longer simulations.

4. After manual verification, rename the generated json-file `received_messages.json` and use it as reference messages in your test configuration, e.g. [multiscan_compact_test01_ref_messages.json](data/multiscan_compact_test01_ref_messages.json) for the multiScan testcase.

## Install docker on Windows

Download and install the Docker Desktop https://desktop.docker.com/win/main/amd64/Docker%20Desktop%20Installer.exe and run "Docker Desktop" as admin.

Note:
* The windows version of the docker container must match the windows version on the docker host.
* Hyper-V must be enabled, see https://learn.microsoft.com/en-us/virtualization/hyper-v-on-windows/quick-start/enable-hyper-v
* Docker must be set to use Windows containers, not the default Linux containers:
   * Open Settings in Docker Desktop and disable WSL ("Use the WSL 2 based engine": OFF)
   * Run `"%ProgramFiles%\Docker\Docker\DockerCli.exe" -SwitchDaemon .`
* If you encounter errors during windows or package installation, check or disable anti-virus software and try again.
* Docker commands `docker system prune -a` and `docker volume prune -f` remove all container and might help in case of problems.
* Building a Windows docker image incl. Visual Studio Compiler may take several hours. Take care of your newly created docker image, do not delete it if you do not have to!
* Use `docker image save -o images.tar image1 [image2 ...]` to save any images you want to keep to a local tar file.
* Use `use docker image load -i images.tar` to restore previously saved images.
* Docker uploads a complete copy of the working directory to the docker daemon when running `docker build`. Exclude large folders using a `.dockerignore` file. The `.dockerignore` file must be in the current working directory when running `docker build`. Copy `sick_scan_xd\.dockerignore` if you are executing `docker build` in another folder.

Dockerfiles to create a windows docker image are provided in folder sick_scan_xd/test/docker/dockerfiles:

|Dockerfile|System|
|----------|------|
| dockerfile_windows_x64_buildtools | Windows core (mcr.microsoft.com/windows) + build tools incl. Visual Studio compiler (vs_buildtools) |
| dockerfile_windows_x64_develop | windows_x64_buildtools + vcpkg + jsoncpp + cmake + python |
| dockerfile_windows_x64_sick_scan_xd | windows_x64_develop + sick_scan_xd |
| dockerfile_windows_dotnet48_buildtools | Windows core with .NET 4.8 (required by chocolatey and ROS2) + build tools incl. Visual Studio compiler (vs_buildtools) |
| dockerfile_windows_dotnet48_develop | windows_dotnet48_buildtools + chocolatey + cmake + python |
| dockerfile_windows_dotnet48_ros2_foxy | windows_dotnet48_develop + ROS2 foxy |
| dockerfile_windows_dotnet48_ros2_humble | windows_dotnet48_develop + ROS2 humble |
| dockerfile_windows_dotnet48_ros2_sick_scan_xd | windows_dotnet48_ros2_humble + sick_scan_xd |

## Build and run sick_scan_xd dockertests on Windows

Start docker desktop and load docker images windows_x64_develop.tar and windows_dotnet48_ros2_humble.tar:
```
docker image load -i windows_x64_develop.tar
docker image load -i windows_dotnet48_ros2_humble.tar
```
If you do not have these docker images, run `build_dockerimage_windows_x64_sick_scan_xd.cmd` in folder `sick_scan_xd/test/docker` to build them. Note that the Windows docker build process is very timeconsuming; using existing docker images is therefore recommended.

Script `run_all_dockertests_windows.cmd` in folder `sick_scan_xd/test/docker` builds the docker image for Windows und runs all dockertests:

```
REM Create a workspace folder (e.g. sick_scan_ws or any other name) and clone the sick_scan_xd repository:
mkdir sick_scan_ws\src
cd sick_scan_ws\src
git clone -b develop https://github.com/SICKAG/sick_scan_xd.git
REM Build and run all sick_scan_xd docker images and tests:
cd sick_scan_xd\test\docker
run_all_dockertests_windows.cmd
```

After successful build and run, the message **SUCCESS: sick_scan_xd docker test passed** will be displayed. 
Otherwise an error message **ERROR: sick_scan_xd docker test FAILED** is printed.
