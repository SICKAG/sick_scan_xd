# Build and run sick_scan_xd in a docker container

This file describes how to build and run sick_scan_xd in a docker container on Linux.

## Install docker on Linux

Run the following steps to install and run docker on Linux:

1. Install Docker: 
   * Follow the instructions on https://docs.docker.com/desktop/install/ubuntu/,
   * or (recommended) install Docker without Docker Desktop by running
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

## Build and run sick_scan_xd for ROS1 noetic in a docker container on Linux 

Shortcut to build and run sick_scan_xd in a docker container for ROS1 noetic on Linux:
```
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

3. Create a docker image named `sick_scan_xd/ros1_noetic` from dockerfile [src/sick_scan_xd/test/docker/dockerfile_linux_ros1_noetic_sick_scan_xd](dockerfile_linux_ros1_noetic_sick_scan_xd) with local sources in folder `./src/sick_scan_xd`:
   ```
   docker build --progress=plain -t sick_scan_xd/ros1_noetic -f ./src/sick_scan_xd/test/docker/dockerfile_linux_ros1_noetic_sick_scan_xd .
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

By default, sick_scan_xd sources are provided in the local folder `./src/sick_scan_xd`. This source folder is just copied into the docker image and used to build sick_scan_xd. This step is executed by the COPY command in the [dockerfile](dockerfile_linux_ros1_noetic_sick_scan_xd):
```
COPY ./src/sick_scan_xd /workspace/src/sick_scan_xd
```
This docker command copies the local folder `./src/sick_scan_xd` into the docker image (destination folder in the docker image is `/workspace/src/sick_scan_xd`). This default option is useful to test new sick_scan_xd versions or release candidates before check in, or to test a local copy of sick_scan_xd from sources requiring authorization.

Alternatively, docker can build sick_scan_xd from a public git repository like https://github.com/SICKAG/sick_scan_xd. A git repository can be set by docker build option `--build-arg SICK_SCAN_XD_GIT_URL=<sick_scan_xd-git-url>`, e.g. `--build-arg SICK_SCAN_XD_GIT_URL=https://github.com/SICKAG/sick_scan_xd`. Replace the `docker build ....` command in step 3 by:

```
# Create a docker image named "sick_scan_xd/ros1_noetic" from dockerfile "./sick_scan_xd/test/docker/dockerfile_linux_ros1_noetic_sick_scan_xd" with github sources from https://github.com/SICKAG/sick_scan_xd
docker build --build-arg SICK_SCAN_XD_GIT_URL=https://github.com/SICKAG/sick_scan_xd --progress=plain -t sick_scan_xd/ros1_noetic -f ./src/sick_scan_xd/test/docker/dockerfile_linux_ros1_noetic_sick_scan_xd .
```

If option `SICK_SCAN_XD_GIT_URL` is set, the docker build command in the [dockerfile](dockerfile_linux_ros1_noetic_sick_scan_xd) clones the given repository:
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
# Create a docker image named "sick_scan_xd/ros1_noetic" from dockerfile "./sick_scan_xd/test/docker/dockerfile_linux_ros1_noetic_sick_scan_xd" with "apt-get install ros-noetic-sick-scan-xd"
docker build --build-arg SICK_SCAN_XD_APT_PKG=ros-noetic-sick-scan-xd --build-arg SICK_SCAN_XD_BUILD_FROM_SOURCES=0 --progress=plain -t sick_scan_xd/ros1_noetic -f ./src/sick_scan_xd/test/docker/dockerfile_linux_ros1_noetic_sick_scan_xd .
```

After a successful build, run and test sick_scan_xd in the docker container as described above in step 4:
```
xhost +local:docker
docker run -it --name sick_scan_xd_container -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY -w /workspace sick_scan_xd/ros1_noetic python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --cfg=./src/sick_scan_xd/test/docker/data/multiscan_compact_test01_cfg.json
```

To install other prebuilt sick_scan_xd binaries, replace command `apt-get install -y $SICK_SCAN_XD_APT_PKG` in the [dockerfile](dockerfile_linux_ros1_noetic_sick_scan_xd) by a customized procedure.

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
* MRS-1xxx docker test: [mrs1xxx_test01_cfg.json](data/mrs1xxx_test01_cfg.json)

To execute these test cases in a linux noetic docker container, run the following commands:
```
# Test sick_scan_xd against multiScan emulator
docker run -it --name sick_scan_xd_container_multiscan_compact_test01 -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY -w /workspace sick_scan_xd/ros1_noetic python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --cfg=./src/sick_scan_xd/test/docker/data/multiscan_compact_test01_cfg.json
# Test sick_scan_xd against multiScan emulator
docker run -it --name sick_scan_xd_container_picoscan_compact_test01 -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY -w /workspace sick_scan_xd/ros1_noetic python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --cfg=./src/sick_scan_xd/test/docker/data/picoscan_compact_test01_cfg.json
# Test sick_scan_xd against MRS-1xxx emulator
docker run -it --name sick_scan_xd_container_mrs1xxx_test01 -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY -w /workspace sick_scan_xd/ros1_noetic python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --cfg=./src/sick_scan_xd/test/docker/data/mrs1xxx_test01_cfg.json
```

Use the configuration files as a template for new test cases resp. different lidars if required.

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

1. Run [sick_scan_xd_simu.py](./python/sick_scan_xd_simu.py) and save messages in a json-file.

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

2. Make sure that the messages saved as a reference are correct, i.e. rviz displays the pointclouds and laserscans correctly and consistent with the real scene observed by the lidar. For lidars with IMU data support, use `rostopic echo <imu topic>` to verify plausible IMU data. Use parameter `--run_seconds=<sec>` to run the simulation as long as needed for the manual verification. For multiScan and picoScan, increase parameter `./src/sick_scan_xd/test/python/multiscan_pcap_player.py --repeat=<number of repetitions> ...` in the configuration file to run longer simulations.

3. After manual verification, rename the generated json-file `received_messages.json` and use it as reference messages in your test configuration, e.g. [multiscan_compact_test01_ref_messages.json](data/multiscan_compact_test01_ref_messages.json) for the multiScan testcase.
