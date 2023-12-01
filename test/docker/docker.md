# Build and run sick_scan_xd in a docker container

This file describes how to build and run sick_scan_xd in a docker container on Windows and Linux.

## Install docker on Windows

1. Install Windows Subsystem for Linux (WSL2):
   1. Run `wsl --install` as admin and reboot.
      
       In case of error message "WslRegisterDistribution failed with error: 0x80370102": Enable Hyper-V in the bios. If Windows runs in a VM:
         VMWare-Player/Workstation -> Virtual Machine Settings -> Processors -> Virtualize Intel VT-x/EPT or AMD-V/RVI
   
   2. Download + install the linux kernel update package https://wslstorestorage.blob.core.windows.net/wslblob/wsl_update_x64.msi and reboot

   3. Check the WSL version with `wsl --status`, it should be version 2
   
   4. Run `wsl --install -d Ubuntu` as admin
   
   5. Check the WSL linux distribution with `wsl --list`, it should be Ubuntu. Otherwise install via Microsoft Store -> Search for Ubuntu -> Install -> Run

   6. Upgrade to latest version: Run `wsl` -> `sudo apt-get -y update`

2. Download + install the Docker Desktop https://desktop.docker.com/win/main/amd64/Docker%20Desktop%20Installer.exe and run "Docker Desktop" as admin

3. Quicktest: Run as admin
    ```
    docker --version
    docker info
    docker run hello-world
    ```
    After running `docker run hello-world`, the Docker desktop shows the hello-world docker container and image.

4. Add your windows account to group docker-users: PC -> Computer Management (as admin) -> Users -> Add <name> to docker-users and reboot

5. Download and install X11 server `VcXsrv` to display the GUI for your Linux apps: https://sourceforge.net/projects/vcxsrv/  

## Install docker on Linux

1. Install Docker:
    ```
    pushd /tmp
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    sudo usermod -aG docker $USER
    popd
    ```
    
2. Quicktest: Run 
    ```
    docker --version
    docker info
    docker run hello-world
    ```

3. To remove all docker logfiles, images, container, caches, etc.pp, run:
    ```
    docker stop $(docker ps -a -q)
    docker rm $(docker ps -a -q)
    docker system prune -a -f
    docker volume prune -f
    docker images -a # list all docker image
    ```

## Build and run docker sick_scan_xd in a docker container on Windows

Docker on Windows uses the Windows Subsystem for Linux (WSL). Therefore, the Linux version of sick_scan_xd runs in the docker image.

It is recommended to start the Docker desktop to run the docker demon (required once if docker demon not started automatically at login). 
Start src\sick_scan_xd\test\docker\vc_xsrv_config.xlaunch to run the VcXsrv X-server (required once if X-server not started automatically at login).

1. Create a workspace folder, e.g. `sick_scan_ws` (or any other name):
   ```
   mkdir sick_scan_ws
   cd sick_scan_ws
   ```

2. Clone repositories https://github.com/SICKAG/libsick_ldmrs and https://github.com/SICKAG/sick_scan_xd:
   ```
   mkdir src
   pushd src
   git clone https://github.com/SICKAG/libsick_ldmrs.git
   git clone https://github.com/SICKAG/sick_scan_xd.git
   popd
   ```

3. Build and run sick_scan_xd in a docker container (generic version without ROS):
   ```
   cd src\sick_scan_xd\test\docker
   make_docker_x64.cmd
   ```
   Alternatively, build and run sick_scan_xd in the docker container using
   ```
   REM Create a docker-image named "rostest/x64_sick_scan_xd" from dockerfile "./sick_scan_xd/test/docker/Dockerfile-x64-sick_scan_xd" with content folder "."
   docker build --progress=plain -t rostest/x64_sick_scan_xd -f ./src/sick_scan_xd/test/docker/Dockerfile-x64-sick_scan_xd .
   REM Run the docker-image named "rostest/x64_sick_scan_xd" and execute the sick_scan_xd example "run_linux_api_test_lms5xx.bash" within the docker container
   docker run -it --rm -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 rostest/x64_sick_scan_xd
   ```

4. Build and run sick_scan_xd in a docker container using ROS-1 noetic:
   ```
   cd src\sick_scan_xd\test\docker
   make_docker_ros1.cmd
   ```
   Alternatively, build and run sick_scan_xd in the docker container using
   ```
   REM Create a docker-image named "rostest/ros1_sick_scan_xd" from dockerfile "./sick_scan_xd/test/docker/Dockerfile-ros1-sick_scan_xd" with content folder "."
   docker build --progress=plain -t rostest/ros1_sick_scan_xd -f ./src/sick_scan_xd/test/docker/Dockerfile-ros1-sick_scan_xd .
   REM Run the docker-image named "rostest/ros1_sick_scan_xd" and execute the MRS1104 example "run_linux_ros1_simu_mrs1104.bash" within the docker container
   docker run -it --rm -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 rostest/ros1_sick_scan_xd
   ```

5. Build and run sick_scan_xd in a docker container using ROS-2 foxy:
   ```
   cd src\sick_scan_xd\test\docker
   make_docker_ros2-foxy.cmd
   ```
   Alternatively, build and run sick_scan_xd in the docker container using
   ```
   REM Create a docker-image named "rostest/sick_scan_xd" from dockerfile "./sick_scan_xd/test/docker/Dockerfile-ros2-foxy-sick_scan_xd" with content folder "."
   docker build --progress=plain -t rostest/sick_scan_xd -f ./src/sick_scan_xd/test/docker/Dockerfile-ros2-foxy-sick_scan_xd .
   REM Run the docker-image named "rostest/sick_scan_xd" and execute the TiM7xx example "run_linux_ros2_simu_tim7xx_tim7xxS.bash" within the docker container
   docker run -it --rm -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 rostest/sick_scan_xd
   ```

6. Build and run sick_scan_xd in a docker container using ROS-2 humble:
   ```
   cd src\sick_scan_xd\test\docker
   make_docker_ros2-humble.cmd
   ```
   Alternatively, build and run sick_scan_xd in the docker container using
   ```
   REM Create a docker-image named "rostest/sick_scan_xd" from dockerfile "./sick_scan_xd/test/docker/Dockerfile-ros2-humble-sick_scan_xd" with content folder "."
   docker build --progress=plain -t rostest/sick_scan_xd -f ./src/sick_scan_xd/test/docker/Dockerfile-ros2-humble-sick_scan_xd .
   REM Run the docker-image named "rostest/sick_scan_xd" and execute the TiM7xx example "run_linux_ros2_simu_tim7xx_tim7xxS.bash" within the docker container
   docker run -it --rm -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 rostest/sick_scan_xd
   ```
