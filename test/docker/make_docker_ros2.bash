#!/bin/bash

# 
# Build sick_scan_xd for Linux-ROS2 in Docker
# 
printf "\033c"
pushd ../../../..

# Create a docker-image named "rostest/sick_scan_xd" from dockerfile "./sick_scan_xd/test/docker/Dockerfile-ros2-sick_scan_xd" with content folder "."
docker build -t rostest/sick_scan_xd -f ./src/sick_scan_xd/test/docker/Dockerfile-ros2-sick_scan_xd .

# Run the docker-image named "rostest/sick_scan_xd" and execute the TiM7xx example "run_linux_ros2_simu_tim7xx_tim7xxS.bash" within the docker container
xhost +local:docker # see https://medium.com/intro-to-artificial-intelligence/rviz-on-docker-bdf4d0fca5b to run X11-gui-applications like rviz2 in docker
docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY rostest/sick_scan_xd

popd
