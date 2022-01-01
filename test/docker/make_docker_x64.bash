#!/bin/bash

# 
# Build sick_scan_xd for native linux-x64 in Docker
# 
printf "\033c"
pushd ../../../..

# Create a docker-image named "rostest/x64_sick_scan_xd" from dockerfile "./sick_scan_xd/test/docker/Dockerfile-x64-sick_scan_xd" with content folder "."
docker build -t rostest/x64_sick_scan_xd -f ./src/sick_scan_xd/test/docker/Dockerfile-x64-sick_scan_xd .

# Run the docker-image named "rostest/x64_sick_scan_xd" and execute the ldmrs example "run_linux_simu_ldmrs.bash" within the docker container
xhost +local:docker
docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY rostest/x64_sick_scan_xd

popd
