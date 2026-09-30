# Docker Test multiScan100

Step-by-step instructions for creating a new Docker test for multiScan100 LiDARs:

## Preparation

* Rebuild sick_scan_xd:
    ```
    . 01_clean_up.sh
    . 02_build.sh
    ```

* Create the data directory for Docker test configurations, scan data, and reference data:
    ```
    mkdir ./src/sick_scan_xd/test/docker/data
    pushd ./src/sick_scan_xd/test/docker/data
    unzip ../data.zip .
    popd
    ```

## Data Recording

* Power on the multiScan100, start Wireshark, and launch sick_scan_xd using `. 08_launch_multiscan.sh`

* Verify in rviz that point clouds and laser scans are correct

* Record a short Wireshark capture, 1 second is sufficient (new scanner data will later be committed, therefore keep the sequence as short as possible)

* Save the recording as a pcapng file, for example:
    `./src/sick_scan_xd/test/docker/data/20260518_multiscan_01.pcapng`

## Converting the pcapng Recording to a JSON File

* **Switch off and disconnect the LiDAR:** No additional UDP data sources may be active on UDP ports 2115 (scanner data) and 7503 (IMU data) during the following steps.

* **Terminate all sick processes:** for example with
    `pkill -f sick`

* Convert the pcapng recording into JSON:
   `python3 ./src/sick_scan_xd/test/python/multiscan_pcap_player.py --pcap_filename=./src/sick_scan_xd/test/docker/data/20260518_multiscan_01.pcapng --udp_port=-1 --repeat=1 --verbose=0 --filter=pcap_filter_multiscan_data --max_seconds=1 --save_udp_jsonfile=./src/sick_scan_xd/test/docker/data/20260518_multiscan_01.json`

   Explanation:
   `--repeat=1`: play once and exit afterwards
   `--filter=pcap_filter_multiscan_data`: pcapng filter for Multiscan data, filters src_ip=192.168.0.1, dst_ip=192.168.0.100, ports 2115 (scan data) and 7503 (imu)
   `--max_seconds=1`: optional limitation of file size

* Verify that the file
    `./src/sick_scan_xd/test/docker/data/20260518_multiscan_01.json`
    exists afterwards and contains JSON data (blocks containing timestamp, port, and payload data).


## Generating Reference Data

* Adjust the configuration in
    `./src/sick_scan_xd/test/docker/data/multiscan_compact_test01_cfg.json`:
    ```
    "args_udp_scandata_sender": [
      "./src/sick_scan_xd/test/python/multiscan_pcap_player.py --json_filename=./src/sick_scan_xd/test/docker/data/20260518_multiscan_01.json --udp_port=-1 --repeat=1 --verbose=0 --send_rate=10"
    ],
    ```

* Generate reference data for subsequent Docker tests:
   `python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=humble --save_messages_jsonfile=20260518_multiscan_01_ref.json --cfg=./src/sick_scan_xd/test/docker/data/multiscan_compact_test01_cfg.json`

   Replace the option `--ros=humble` with `--ros=foxy` if required.

   At the end, warnings will be printed (test failed). This is expected and can be ignored because the test cannot yet verify the new recording against the newly generated reference data.

   Since UDP scanner data is sent very slowly using the option `--send_rate=10` during reference generation, execution takes slightly longer.

   The generated reference data will afterwards be located in
   `./log/sick_scan_xd_simu/<timestamp>`,
   for example:
   `./log/sick_scan_xd_simu/20260521_080121/20260518_multiscan_01_ref.json`

   The reference data file `20260518_multiscan_01_ref.json` must contain the structure:
   `{ "RefLaserscanMsg": { ... }, "RefPointcloudMsg": { ... }, "RefImuMsg": { ... } }`

* Copy the new reference data file into the Docker data directory:
    ```
    cp -f ./log/sick_scan_xd_simu/20260521_080121/20260518_multiscan_01_ref.json ./src/sick_scan_xd/test/docker/data
    ```

* Adjust the configuration in
    `./src/sick_scan_xd/test/docker/data/multiscan_compact_test01_cfg.json`:
    ```
    "info.reference_messages_jsonfile": "# Json file with reference messages. Test passed if all messages were published by sick_scan_xd. Test failed otherwise.",
    "reference_messages_jsonfile": "./src/sick_scan_xd/test/docker/data/20260518_multiscan_01_ref.json",

    "args_udp_scandata_sender": [
      "./src/sick_scan_xd/test/python/multiscan_pcap_player.py --json_filename=./src/sick_scan_xd/test/docker/data/20260518_multiscan_01.json --udp_port=-1 --repeat=3 --verbose=0 --send_rate=20"
    ],
    ```

    The parameter `reference_messages_jsonfile` configures the newly created reference data.

    The parameter `--send_rate=20` in `args_udp_scandata_sender` configures the transmission speed of UDP packets during the Docker test. If packet loss occurs on weaker hardware or virtual machines, this parameter can be reduced further.

    With parameter `--repeat=3`, the data recording is replayed three times during the Docker test.

    Background: the UDP sender and the sick_scan_xd driver run in separate processes. If the UDP sender already starts transmitting while sick_scan_xd is still initializing, verification may detect missing data. Replaying the recording multiple times ensures that sick_scan_xd publishes all reference data during a successful test.

    Every entry in the reference data file must be published at least once by sick_scan_xd during the test, otherwise the test fails.

## Test Without Docker

* First verify the new test outside a Docker container:
    ```
    pkill -f sick
    python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=humble --cfg=./src/sick_scan_xd/test/docker/data/multiscan_compact_test01_cfg.json
    pkill -f sick
    ```

    Replace the option `--ros=humble` with `--ros=foxy` if required.

    This is the same command as before, now using the updated configuration (including the new reference data) and without option `--save_messages_jsonfile`.

    At the end, the following output should appear:
    ```
    sick_scan_xd_simu finished, messages successfully verified, TEST PASSED
    sick_scan_xd_monitor.verify_messages sucessful, TEST PASSED
    sick_scan_xd_simu exit status: 0, success
    ```

    Additionally, the file `sick_scan_xd_testreport.md` should be generated in the log folder, for example:
    `./log/sick_scan_xd_simu/20260521_084617/sick_scan_xd_testreport.md`

## Build Docker Image

* Quick Docker installation test:
    ```
    docker --version
    docker info
    docker run hello-world
    docker images -a # list all docker images
    docker ps -a # list all containers
    ```

* Build the Docker image with the latest sources and tests:
    ```
    dockerimage=sick_scan_xd/ros2_humble
    docker image ls linux_ros2_humble_develop # required to build dockerimage sick_scan_xd/ros2_humble
    dockerfile=./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_linux_ros2_humble_sick_scan_xd
    docker build --progress=plain -t ${dockerimage} -f ${dockerfile} .
    docker image ls ${dockerimage}
    ```

    Replace `humble` with `foxy` if required (optional, humble inside the Docker container also works if only foxy is installed locally).

## Run Docker Test

* Run the same test (same simulation) inside the Docker container:
    ```
    dockerimage=sick_scan_xd/ros2_humble
    export_folder=sick_scan_xd_simu_humble
    simu_args="--ros=humble --api=none"
    cfg=multiscan_compact_test01_cfg
    container_name=sick_scan_xd_$cfg
    container_id=$(docker ps -aqf "name=$container_name")
    docker rm $container_id # remove previous docker container

    docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY -w /workspace --name $container_name ${dockerimage} python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py ${simu_args} --cfg=./src/sick_scan_xd/test/docker/data/$cfg.json

    docker_exit_status=$?
    ```

    At the end, the following output should appear again:
    ```
    sick_scan_xd_simu finished, messages successfully verified, TEST PASSED
    sick_scan_xd_monitor.verify_messages sucessful, TEST PASSED
    sick_scan_xd_simu exit status: 0, success
    ```

    Additionally, `$docker_exit_status` must be 0 (successful test).

* If required, reports and log files can be exported from the Docker container:
    ```
    container_id=$(docker ps -aqf "name=$container_name")
    docker cp $container_id:/workspace/log/sick_scan_xd_simu ./log # export log files from container workspace to local folder ./log/sick_scan_xd_simu
    ```

    Similar to the non-Docker test, the Docker test files will afterwards be available in
    `./log/sick_scan_xd_simu`,
    for example:
    `./log/sick_scan_xd_simu/20260521_100119/sick_scan_xd_testreport.md`

## FAQ

* No reference data is generated:
    * No LiDAR may be connected
    * Terminate all sick processes before conversion: `pkill -f sick`
    * Activate the virtual Python environment for scapy, pcapng, etc.

* Update to a new ROS version:
    * In sources under `docker/python`, replace `"humble"` with `"humble, jazzy or lyrical"` (`ros2_supported_versions=["humble", "jazzy","lyrical"]`)
    * Create a Docker image with jazzy installation:
        * Replace `humble` with `jazzy` in `dockerfile_linux_ros2_humble_sick_scan_xd` and `dockerfile_linux_ros2_humble_develop` and rename accordingly
    * For "lyrical" replace "jazzy" with "lyrical"

* Test data has been updated. To update the repository, zip folder
    `./src/sick_scan_xd/test/docker/data`
    into
    `./src/sick_scan_xd/test/docker/data.zip`:
    ```
    pushd ./src/sick_scan_xd/test/docker/data
    zip ../data_new.zip ./*
    mv -f ../data_new.zip ../data.zip
    popd
    ```
