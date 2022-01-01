REM 
REM Build sick_scan_xd for ROS-2 with Docker and Windows Subsystem for Linux (WSL)
REM 

REM Test rviz2
REM docker build -t rostest/rviz2 -f Dockerfile-rviz2 .
REM docker run -it --rm -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 rostest/rviz2

pushd ..\..\..\..

REM Create a docker-image named "rostest/sick_scan_xd" from dockerfile "./sick_scan_xd/test/docker/Dockerfile-ros2-sick_scan_xd" with content folder "."
docker build -t rostest/sick_scan_xd -f ./src/sick_scan_xd/test/docker/Dockerfile-ros2-sick_scan_xd .

REM Run the docker-image named "rostest/sick_scan_xd" and execute the TiM7xx example "run_linux_ros2_simu_tim7xx_tim7xxS.bash" within the docker container
docker run -it --rm -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 rostest/sick_scan_xd

popd
@pause
