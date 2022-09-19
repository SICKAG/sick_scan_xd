REM 
REM Build and run sick_scan_xd for ROS-2 foxy with Docker and Windows Subsystem for Linux (WSL)
REM 
REM Start Docker desktop to run the docker demon (required once if docker demon not started automatically at login).
REM Start vc_xsrv_config.xlaunch to run VcXsrv X-server (required once if X-server not started automatically at login).
REM 

REM Test rviz2
REM docker build -t rostest/rviz2 -f Dockerfile-rviz2 .
REM docker run -it --rm -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 rostest/rviz2

pushd ..\..\..\..

REM Create a docker-image named "rostest/sick_scan_xd" from dockerfile "./sick_scan_xd/test/docker/Dockerfile-ros2-foxy-sick_scan_xd" with content folder "."
docker build --progress=plain -t rostest/sick_scan_xd -f ./src/sick_scan_xd/test/docker/Dockerfile-ros2-foxy-sick_scan_xd .

REM Run the docker-image named "rostest/sick_scan_xd" and execute the TiM7xx example "run_linux_ros2_simu_tim7xx_tim7xxS.bash" within the docker container
docker run -it --rm -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 rostest/sick_scan_xd

REM Cleanup
rem docker container stop rostest/ros1_sick_scan_xd
rem docker container kill rostest/ros1_sick_scan_xd

popd
@pause
