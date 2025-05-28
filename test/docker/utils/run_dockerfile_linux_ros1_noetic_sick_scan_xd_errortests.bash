#!/bin/bash

# 
# Build sick_scan_xd for Linux-ROS1 in Docker and run error testcases
# 
printf "\033c"
pushd ../../../../..

docker build --progress=plain -t sick_scan_xd/ros1_noetic -f ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_linux_ros1_noetic_sick_scan_xd .
xhost +local:docker
for cfg in multiscan_compact_errortest01_cfg picoscan_compact_errortest01_cfg ; do 
    echo -e "Running sick_scan_xd docker test with configuration file $cfg"
    container_name=sick_scan_xd_$cfg
    container_id=$(docker ps -aqf "name=$container_name") 
    docker rm $container_id # remove previous docker container
    docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY -w /workspace --name $container_name sick_scan_xd/ros1_noetic python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --api=none --cfg=./src/sick_scan_xd/test/docker/data/$cfg.json
    docker_exit_status=$?
    echo -e "docker_exit_status for $cfg: $docker_exit_status"
    if [ $docker_exit_status -eq 0 ] ; then 
        echo -e "\nSUCCESS: sick_scan_xd docker test passed for configuration $cfg\n"  # unexpected test result, error testcases should result in status "test FAILED"
        read -n 1 -p "Press any key to continue..."
    else 
        echo -e "\n## ERROR: sick_scan_xd docker test FAILED for configuration $cfg\n" # expected test result for error testcases
        docker_status_final=$docker_exit_status
    fi
done

