REM 
REM Build sick_scan_xd for Windows in Docker
REM
REM Important notes (do not ignore!):
REM * Docker Desktop must be updated and running.
REM * Hyper-V must be enabled, see https://learn.microsoft.com/en-us/virtualization/hyper-v-on-windows/quick-start/enable-hyper-v
REM * Docker must be set to use Windows containers, not the default Linux containers.
REM   - Open Settings in Docker Desktop and disable WSL ("Use the WSL 2 based engine": OFF)
REM   - Run `"%ProgramFiles%\Docker\Docker\DockerCli.exe" -SwitchDaemon .`
REM * The windows version of the docker container must match the windows version on the docker host.
REM * If you encounter errors during windows installation, try again.
REM * Docker commands "docker system prune -a" and "docker volume prune -f" remove all container and might help in case of problems.
REM * Visual Studio installation requires a user registration with 2FA, e.g. with your github account. 
REM * Building a Windows docker image incl. Visual Studio Compiler may take several hours. Take care of your newly created docker image, do not delete it if you do not have to!
REM * Use `docker image save -o images.tar image1 [image2 ...]` to save any images you want to keep to a local tar file.
REM * Use `use docker image load -i images.tar` to restore previously saved images.
REM * Docker uploads a complete copy of the working directory to the docker daemon when running `docker build`. Exclude large folders using a `.dockerignore` file.
REM   The `.dockerignore` file must be in the current working directory when running `docker build`. Copy `sick_scan_xd\.dockerignore` if you are executing `docker build` in another folder.
REM   Otherwise docker container may require an unnecessary huge amount of RAM when running.
REM * If installing software in the docker image failed, check or disable anti-virus software.
REM 
REM Basic test to build and run a windows docker container:
REM   cd /d c:\tmp
REM   %ProgramFiles%\Docker\Docker\DockerCli.exe -SwitchDaemon .
REM   docker --version
REM   docker images -a
REM   docker ps -a
REM   docker pull mcr.microsoft.com/windows:ltsc2019
REM   docker run mcr.microsoft.com/windows:ltsc2019 cmd echo "cmd running in windows docker container" & pause
REM 
REM Create and save a windows docker image(s) with buildtools, cmake and python (required once, takes several hours to complete)
REM   windows_x64_buildtools:   Windows core (mcr.microsoft.com/windows) + build tools incl. Visual Studio compiler (vs_buildtools) 
REM   windows_x64_develop:      windows_x64_buildtools + vcpkg + jsoncpp + cmake + python
REM   windows_x64_sick_scan_xd: windows_x64_develop + sick_scan_xd
REM   windows_dotnet48_buildtools: Windows core with .NET 4.8 (required by chocolatey and ROS2) + build tools incl. Visual Studio compiler (vs_buildtools)
REM   windows_dotnet48_develop: windows_dotnet48_buildtools + chocolatey + cmake + python
REM   windows_dotnet48_ros2_foxy: windows_dotnet48_develop + ROS2 foxy
REM   windows_dotnet48_ros2_humble: windows_dotnet48_develop + ROS2 humble
REM   windows_dotnet48_ros2_sick_scan_xd: windows_dotnet48_ros2_humble + sick_scan_xd
REM
REM Load and test docker images:
REM   docker image load -i ./docker_images/windows_x64_buildtools.tar
REM   docker run -it windows_x64_buildtools
REM   docker image load -i ./docker_images/windows_x64_develop.tar
REM   docker run -it windows_x64_develop
REM   docker run -it sick_scan_xd_windows_x64
REM 

pushd ..\..\..\..
copy /b/y .\src\sick_scan_xd\.dockerignore .
docker --version
docker image ls & docker ps -a

REM 
REM Build Windows x64 docker image
REM

REM Build resp. load docker image windows_x64_buildtools, if not yet done
if not exist .\docker_images\windows_x64_buildtools.tar ( 
    @echo "Building docker image windows_x64_buildtools ..."
    docker build -t windows_x64_buildtools -f ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_windows_x64_buildtools .
    docker image save -o ./docker_images/windows_x64_buildtools.tar windows_x64_buildtools
    docker image save -o ./docker_images/mcr_microsoft_com_windows.tar mcr.microsoft.com/windows
)
docker inspect windows_x64_buildtools
if not %ERRORLEVEL%==0 ( echo "Load docker image windows_x64_buildtools..." & docker image load -i ./docker_images/windows_x64_buildtools.tar )

REM Build resp. load docker image windows_x64_develop, if not yet done
if not exist .\docker_images\windows_x64_develop.tar ( 
    @echo "Building docker image windows_x64_develop ..."
    docker build -t windows_x64_develop -f ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_windows_x64_develop .
    docker image save -o ./docker_images/windows_x64_develop.tar windows_x64_develop
)
docker inspect windows_x64_develop
if not %ERRORLEVEL%==0 ( echo "Load docker image windows_x64_develop..." & docker image load -i ./docker_images/windows_x64_develop.tar )

REM 
REM Build ROS2-Windows docker image
REM

REM Build resp. load docker image windows_dotnet48_buildtools, if not yet done
if not exist .\docker_images\windows_dotnet48_buildtools.tar ( 
    @echo "Building docker image windows_dotnet48_buildtools ..."
    docker build -t windows_dotnet48_buildtools -f ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_windows_dotnet48_buildtools .
    docker image save -o ./docker_images/mcr_microsoft_com_dotnet_48.tar mcr.microsoft.com/dotnet/framework/sdk
    docker image save -o ./docker_images/windows_dotnet48_buildtools.tar windows_dotnet48_buildtools
)
docker inspect windows_dotnet48_buildtools
if not %ERRORLEVEL%==0 ( echo "Load docker image windows_dotnet48_buildtools..." & docker image load -i ./docker_images/windows_dotnet48_buildtools.tar )

REM Build resp. load docker image windows_dotnet48_develop, if not yet done
if not exist .\docker_images\windows_dotnet48_develop.tar ( 
    @echo "Building docker image windows_dotnet48_develop ..."
    docker build -t windows_dotnet48_develop -f ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_windows_dotnet48_develop .
    docker image save -o ./docker_images/windows_dotnet48_develop.tar windows_dotnet48_develop
)
docker inspect windows_dotnet48_develop
if not %ERRORLEVEL%==0 ( echo "Load docker image windows_dotnet48_develop..." & docker image load -i ./docker_images/windows_dotnet48_develop.tar )

REM Build resp. load docker image windows_dotnet48_ros2_develop (i.e. foxy), if not yet done
REM if not exist .\docker_images\windows_dotnet48_ros2_develop.tar ( 
REM     @echo "Building docker image windows_dotnet48_ros2_develop ..."
REM     docker build -t windows_dotnet48_ros2_develop -f ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_windows_dotnet48_ros2_develop .
REM     docker image save -o ./docker_images/windows_dotnet48_ros2_develop.tar windows_dotnet48_ros2_develop
REM )
REM docker inspect windows_dotnet48_ros2_develop
REM if not %ERRORLEVEL%==0 ( echo "Load docker image windows_dotnet48_ros2_develop..." & docker image load -i ./docker_images/windows_dotnet48_ros2_develop.tar )

REM Build resp. load docker image windows_dotnet48_ros2_foxy, if not yet done
if not exist .\docker_images\windows_dotnet48_ros2_foxy.tar ( 
    @echo "Building docker image windows_dotnet48_ros2_foxy ..."
    docker build -t windows_dotnet48_ros2_foxy -f ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_windows_dotnet48_ros2_foxy .
    docker image save -o ./docker_images/windows_dotnet48_ros2_foxy.tar windows_dotnet48_ros2_foxy
)
docker inspect windows_dotnet48_ros2_foxy
if not %ERRORLEVEL%==0 ( echo "Load docker image windows_dotnet48_ros2_foxy..." & docker image load -i ./docker_images/windows_dotnet48_ros2_foxy.tar )

REM Build resp. load docker image windows_dotnet48_ros2_humble, if not yet done
if not exist .\docker_images\windows_dotnet48_ros2_humble.tar ( 
    @echo "Building docker image windows_dotnet48_ros2_humble ..."
    docker build -t windows_dotnet48_ros2_humble -f ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_windows_dotnet48_ros2_humble .
    docker image save -o ./docker_images/windows_dotnet48_ros2_humble.tar windows_dotnet48_ros2_humble
)
docker inspect windows_dotnet48_ros2_humble
if not %ERRORLEVEL%==0 ( echo "Load docker image windows_dotnet48_ros2_humble..." & docker image load -i ./docker_images/windows_dotnet48_ros2_humble.tar )

REM 
REM Build sick_scan_xd docker images (Windows x64 and Windows ROS-2)
REM

REM Rebuild docker images windows_x64_sick_scan_xd and windows_dotnet48_ros2_sick_scan_xd with a copy of local folder ./src/sick_scan_xd
@echo "Building docker image windows_x64_sick_scan_xd ..."
docker build -t windows_x64_sick_scan_xd -f ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_windows_x64_sick_scan_xd .
docker build -t windows_dotnet48_ros2_sick_scan_xd -f ./src/sick_scan_xd/test/docker/dockerfiles/dockerfile_windows_dotnet48_ros2_sick_scan_xd .
docker image ls & docker ps -a
@timeout /t 10

REM
REM Run the docker container
REM
REM @echo "Running docker container windows_x64_sick_scan_xd ..."
REM docker run -it windows_x64_sick_scan_xd
REM @echo "Running docker container windows_dotnet48_ros2_sick_scan_xd ..."
REM docker run -it windows_dotnet48_ros2_sick_scan_xd
REM @pause

popd
