REM
REM Run dockertests on windows without envolving docker container
REM 

REM
REM Build sick_scan_xd for Windows 64 (native and ROS2, dry run without docker)
REM and run sick_scan_xd simulation (C++ API, Python API and ROS2,
REM optional dry run without docker for debugging and development)
REM 

REM 
REM Start docker desktop to run sick_scan_xd docker tests
REM 

docker version
if NOT %ERRORLEVEL%==0 (
    @echo.
    @echo Error: Docker engine is not running.
    @echo Start docker desktop to run sick_scan_xd docker tests
    @echo and load docker images windows_x64_develop.tar and windows_dotnet48_ros2_humble.tar:
    @echo docker image load -i windows_x64_develop.tar
    @echo docker image load -i windows_dotnet48_ros2_humble.tar
    @echo.
    @pause
    @exit
)

set BUILD_SIMU_TESTS_WINDOWS_X64=0
set BUILD_SIMU_TESTS_WINDOWS_ROS2=0
set RUN_ALL_SIMU_TESTS_WINDOWS_X64=0
set RUN_ALL_SIMU_TESTS_WINDOWS_ROS2=0
set PATH=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python39_64;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python39_64\Scripts;%PATH%

pushd ..\..\..\..
if exist .\log ( rmdir /s/q .\log )
if %BUILD_SIMU_TESTS_WINDOWS_X64%==1 ( call .\src\sick_scan_xd\test\docker\utils\build_sick_scan_xd_docker_win64.cmd )
if %BUILD_SIMU_TESTS_WINDOWS_ROS2%==1 ( call .\src\sick_scan_xd\test\docker\utils\build_sick_scan_xd_win64_ros2.cmd )
if %RUN_ALL_SIMU_TESTS_WINDOWS_X64%==1 ( call .\src\sick_scan_xd\test\docker\run_simu_windows_x64.cmd )
if %RUN_ALL_SIMU_TESTS_WINDOWS_ROS2%==1 ( call .\src\sick_scan_xd\test\docker\run_simu_windows_ros2.cmd )
popd

REM
REM Build sick_scan_xd in windows docker image
REM and run dockertest for sick_scan_xd.
REM docker image windows_x64_sick_scan_xd used for C++ and Python API,
REM docker image windows_dotnet48_ros2_sick_scan_xd used for ROS2.
REM 

set RUN_ALL_DOCKERTESTS_WINDOWS_X64=1
if %RUN_ALL_DOCKERTESTS_WINDOWS_X64%==1 (
    REM Rebuild all docker images for windows
    REM call .\build_dockerimage_windows_x64_sick_scan_xd.cmd
    pushd ..\..\..\..
    REM Rebuild docker images windows_x64_sick_scan_xd and windows_dotnet48_ros2_sick_scan_xd with a copy of local folder ./src/sick_scan_xd
    @echo "Building docker image windows_x64_sick_scan_xd ..."
    docker build -t windows_x64_sick_scan_xd -f ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_windows_x64_sick_scan_xd .
    docker build -t windows_dotnet48_ros2_sick_scan_xd -f ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_windows_dotnet48_ros2_sick_scan_xd .
    docker image ls & docker ps -a
    @timeout /t 10
    REM Run docker tests windows api     
    start "docker_run_windows_x64_sick_scan_xd" docker run -it windows_x64_sick_scan_xd
    timeout /t 5
    @pause
    REM Run docker tests windows ros2     
    start "docker_run_windows_dotnet48_ros2_sick_scan_xd" docker run -it windows_dotnet48_ros2_sick_scan_xd
    timeout /t 5
    @pause
    popd
)
rem @pause
