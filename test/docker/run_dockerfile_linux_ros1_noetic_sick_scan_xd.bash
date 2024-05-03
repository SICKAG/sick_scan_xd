#!/bin/bash


function prompt()
{
    read -n 1 -p "Press any key to continue..."
}

# 
# Build sick_scan_xd for Linux-ROS1 in Docker
# 
printf "\033c"
pushd ../../../..

# Default: Create a docker-image named "sick_scan_xd/ros1_noetic" from dockerfile "./sick_scan_xd/test/docker/dockerfile_linux_ros1_noetic_sick_scan_xd" with local sources in folder "./src/sick_scan_xd"
docker build --progress=plain -t sick_scan_xd/ros1_noetic -f ./src/sick_scan_xd/test/docker/dockerfile_linux_ros1_noetic_sick_scan_xd .
# Alternative: Create a docker-image named "sick_scan_xd/ros1_noetic" from dockerfile "./sick_scan_xd/test/docker/dockerfile_linux_ros1_noetic_sick_scan_xd" with github sources from https://github.com/SICKAG/sick_scan_xd
# docker build --build-arg SICK_SCAN_XD_GIT_URL=https://github.com/SICKAG/sick_scan_xd --progress=plain -t sick_scan_xd/ros1_noetic -f ./src/sick_scan_xd/test/docker/dockerfile_linux_ros1_noetic_sick_scan_xd .
# Alternative: Create a docker-image named "sick_scan_xd/ros1_noetic" from dockerfile "./sick_scan_xd/test/docker/dockerfile_linux_ros1_noetic_sick_scan_xd" with "apt-get install ros-noetic-sick-scan-xd"
# docker build --build-arg SICK_SCAN_XD_APT_PKG=ros-noetic-sick-scan-xd --build-arg SICK_SCAN_XD_BUILD_FROM_SOURCES=0 --progress=plain -t sick_scan_xd/ros1_noetic -f ./src/sick_scan_xd/test/docker/dockerfile_linux_ros1_noetic_sick_scan_xd .
docker image ls sick_scan_xd/ros1_noetic
# docker rm $(docker ps -a -q) # optional cleanup: remove all docker container

# Run the docker-image named "sick_scan_xd/ros1_noetic" and execute a multiscan test within the docker container
# docker run option --rm automatically removes the container when it exits
xhost +local:docker # see https://medium.com/intro-to-artificial-intelligence/rviz-on-docker-bdf4d0fca5b to run X11-gui-applications like rviz in docker
docker_status_final=0 # success
for cfg in multiscan_compact_test01_cfg picoscan_compact_test01_cfg mrs1xxx_test01_cfg ; do 
    echo -e "Running sick_scan_xd docker test with configuration file $cfg"
    container_name=sick_scan_xd_$cfg
    container_id=$(docker ps -aqf "name=$container_name") 
    docker rm $container_id # remove previous docker container
    docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY -w /workspace --name $container_name sick_scan_xd/ros1_noetic python3 ./src/sick_scan_xd/test/docker/python/sick_scan_xd_simu.py --ros=noetic --cfg=./src/sick_scan_xd/test/docker/data/$cfg.json
    docker_exit_status=$?
    echo -e "docker_exit_status for $cfg: $docker_exit_status"
    if [ $docker_exit_status -eq 0 ] ; then 
        echo -e "\nSUCCESS: sick_scan_xd docker test passed for configuration $cfg\n"
    else 
        echo -e "\n## ERROR: sick_scan_xd docker test FAILED for configuration $cfg\n"
        docker_status_final=$docker_exit_status
        # prompt
    fi
    # docker ps -a # list all container
done

# Export sick_scan_xd output folder from docker container to local folder ./log/sick_scan_xd_simu
docker ps -a # list all container
if [ -d ./log/sick_scan_xd_simu ] ; then rm -rf ./log/sick_scan_xd_simu ; fi
mkdir -p ./log
for cfg in multiscan_compact_test01_cfg picoscan_compact_test01_cfg mrs1xxx_test01_cfg ; do 
    container_name=sick_scan_xd_$cfg
    container_id=$(docker ps -aqf "name=$container_name")
    docker cp $container_id:/workspace/log/sick_scan_xd_simu ./log
done
pushd ./log/sick_scan_xd_simu
echo -e "# sick_scan_xd test report summary\n" > sick_scan_xd_testreport.md
if [ $docker_status_final -eq 0 ] ; then 
    echo -e "**SUCCESS: all sick_scan_xd docker tests passed**\n" >> sick_scan_xd_testreport.md
else 
    echo -e "**ERROR: sick_scan_xd docker tests with status FAILED**\n" >> sick_scan_xd_testreport.md
fi
for summary in ./*/sick_scan_xd_summary.md ; do (cat $summary >> sick_scan_xd_testreport.md) ; done
for mdfile in ./*.md ; do pandoc -f markdown -t html -s $mdfile -o $mdfile.html ; done
for mdfile in ./*/*.md ; do pandoc -f markdown -t html -s $mdfile -o $mdfile.html ; done
popd
echo -e "\nlogfiles exported from docker container to local folder ./log/sick_scan_xd_simu:"
ls -al ./log/sick_scan_xd_simu/*
echo -e "\n"
cat ./log/sick_scan_xd_simu/sick_scan_xd_testreport.md
firefox ./log/sick_scan_xd_simu/sick_scan_xd_testreport.md.html &

popd
if [ $docker_status_final -eq 0 ] ; then 
    echo -e "\nSUCCESS: all sick_scan_xd docker tests passed\n"
else 
    echo -e "\n## ERROR: sick_scan_xd docker tests with status FAILED\n"
    prompt
fi
exit $docker_status_final

