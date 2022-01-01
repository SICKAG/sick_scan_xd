#!/bin/bash

# 
# Build sick_scan_xd for Linux-ROS1 in Docker
# 
printf "\033c"
pushd ../../../..

# Create a docker-image named "rostest/ros1_sick_scan_xd" from dockerfile "./sick_scan_xd/test/docker/Dockerfile-ros1-sick_scan_xd" with content folder "."
docker build -t rostest/ros1_sick_scan_xd -f ./src/sick_scan_xd/test/docker/Dockerfile-ros1-sick_scan_xd .

# Run the docker-image named "rostest/ros1_sick_scan_xd" and execute the MRS1104 example "run_linux_ros1_simu_mrs1104.bash" within the docker container
xhost +local:docker # see https://medium.com/intro-to-artificial-intelligence/rviz-on-docker-bdf4d0fca5b to run X11-gui-applications like rviz in docker
docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY rostest/ros1_sick_scan_xd

popd
