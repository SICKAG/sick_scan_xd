REM 
REM Build and run sick_scan_xd for native x64 with Docker and Windows Subsystem for Linux (WSL)
REM 
REM Start Docker desktop to run the docker demon (required once if docker demon not started automatically at login).
REM Start vc_xsrv_config.xlaunch to run VcXsrv X-server (required once if X-server not started automatically at login).
REM

pushd ..\..\..\..

REM Create a docker-image named "rostest/x64_sick_scan_xd" from dockerfile "./sick_scan_xd/test/docker/Dockerfile-x64-sick_scan_xd" with content folder "."
rem docker build --no-cache --progress=plain -t rostest/x64_sick_scan_xd -f ./src/sick_scan_xd/test/docker/Dockerfile-x64-sick_scan_xd .
docker build --progress=plain -t rostest/x64_sick_scan_xd -f ./src/sick_scan_xd/test/docker/Dockerfile-x64-sick_scan_xd .

REM Run the docker-image named "rostest/x64_sick_scan_xd" and execute the sick_scan_xd example "run_linux_api_test_lms5xx.bash" within the docker container
docker run -it --rm -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 rostest/x64_sick_scan_xd

REM Cleanup
rem docker container stop rostest/x64_sick_scan_xd
rem docker container kill rostest/x64_sick_scan_xd

popd
@pause
