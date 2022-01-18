REM 
REM Build sick_scan_xd for native x64 with Docker and Windows Subsystem for Linux (WSL)
REM 

pushd ..\..\..\..

REM Create a docker-image named "rostest/x64_sick_scan_xd" from dockerfile "./sick_scan_xd/test/docker/Dockerfile-x64-sick_scan_xd" with content folder "."
docker build -t rostest/x64_sick_scan_xd -f ./src/sick_scan_xd/test/docker/Dockerfile-x64-sick_scan_xd .

REM Run the docker-image named "rostest/x64_sick_scan_xd" and execute the LDMRS example "run_linux_simu_ldmrs.bash" within the docker container
docker run -it --rm -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 rostest/x64_sick_scan_xd

popd
@pause
