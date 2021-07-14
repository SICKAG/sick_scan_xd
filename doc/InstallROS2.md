## Install ROS2 on Ubuntu

1. If not yet done, install common development packages, such as 
```console
sudo apt-get update
sudo apt-get install net-tools
sudo apt-get install samba
sudo apt-get install cmake
sudo apt-get install libjpeg-dev
sudo apt-get install libboost-all-dev
sudo apt-get install libopencv-dev
sudo apt-get install libopencv-contrib-dev
sudo apt-get install libssl-dev
sudo apt-get install gcc-multilib
sudo apt-get install valgrind
sudo apt-get install doxygen
sudo apt-get install libncurses5-dev
sudo apt-get install mc
sudo apt-get install openssh-server
sudo apt-get install git
```

2. On Ubuntu 20, you can install the (currently latest) ROS2 version Foxy Fitzroy. On Ubuntu 18, you have to install the previous ROS2 version Eloquent Elusor.

2.1. For Ubuntu 18, follow the instructions on https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/:
```console
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt install ros-eloquent-desktop
source /opt/ros/eloquent/setup.bash
sudo apt install python3-argcomplete
sudo apt install python3-colcon-common-extensions
sudo apt install python3-pip
sudo pip3 install jupyter
```

2.2. For Ubuntu 20, follow the instructions on https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/:
```console
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt install ros-foxy-desktop
source /opt/ros/foxy/setup.bash
sudo apt install python3-argcomplete
sudo apt install python3-colcon-common-extensions
```

3. Test installation by running some basic examples:
```console
source /opt/ros/eloquent/setup.bash
ros2 topic list
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
ros2 topic echo /chatter
rviz2
ros2 run rviz2 rviz2
```

If `ros2 topic list` crashes with messages like:
```
 ** On entry to DGEBAL parameter number  3 had an illegal value
 ** On entry to DGEHRD  parameter number  2 had an illegal value
 ** On entry to DORGHR DORGQR parameter number  2 had an illegal value
 ** On entry to DHSEQR parameter number  4 had an illegal value
Failed to load entry point 'list': The current Numpy installation ('c:\\python38\\lib\\site-packages\\numpy\\__init__.py') fails to pass a sanity check due to a bug in the windows runtime. See this issue for more information: https://tinyurl.com/y3dm3h86
Traceback (most recent call last):
```
downgrade numpy from V. 1.19.4 to V. 1.19.3 by using the following command:
`pip install numpy==1.19.3`
(see  https://tinyurl.com/y3dm3h86 for more details)

### C++ Development

For C++ development, install an IDE like Visual Studio Code or Clion.

For Clion: see https://github.com/SICKAG/sick_scan2/blob/master/README.md#developing-with-clion-ide

For Visual Studio Code: Download the debian package code_1.47.3-1595520028_amd64.deb (or any later version) from https://code.visualstudio.com and install by
```console
sudo apt install ./code_1.47.3-1595520028_amd64.deb
```

Open Visual Studio Code by running `code` in the console, select `Customize`, `Tools and languages` and install Python, C/C++, ROS, Colcon Tasks and Markdown (and any other usefull extensions you might need).

Checkout and build sick_lidar3d (see README.md):
```
# Checkout msgpack11 and sick_lidar3d
mkdir ./sick_lidar3d_ws
cd ./sick_lidar3d_ws
git clone https://github.com/ar90n/msgpack11.git
git clone https://github.com/SICKAG/sick_lidar3d.git
# Build msgpack11 and sick_lidar3d with BUILDTYPE=Debug or BUILDTYPE=Release
pushd ./sick_lidar3d/test/scripts
./makeall.bash
popd
```
Start Visual Studio Code: 
```
source ./install/setup.bash
code
```
and open folder `sick_lidar3d_ws` via `File` menu and save a new workspace with `Save Workspace As...`. Open file `launch.json` in the Visual Studio Code Editor and insert the program to debug:
```
"program": "${workspaceFolder}/build_x64/sick_lidar3d/lidar3d_mrs100_recv",
"args": [],
"stopAtEntry": true,
"cwd": "${workspaceFolder}/build_x64/sick_lidar3d ",
```
Press F5 to run lidar3d_mrs100_recv in the debugger.

To build and run lidar3d_mrs100_recv with ROS2-support, open file `c_cpp_properties.json` in the Visual Studio Code Editor and insert compiler settings:
```
"defines": [ "__ROS_VERSION=2" ],
"includePath": [
"${workspaceFolder}/sick_lidar3d/include/**",
"/opt/ros/eloquent/include",
"/usr/include/**"
],
```
Open file `launch.json` in the Visual Studio Code Editor and insert the program to debug:
```
"program": "${workspaceFolder}/build/sick_lidar3d/lidar3d_mrs100_recv",
"args": [],
"stopAtEntry": true,
"cwd": "${workspaceFolder}/build/sick_lidar3d ",
```
Press F5 to run lidar3d_mrs100_recv for ROS2 in the debugger.

You can run a new colcon build via `Terminal`, `Configure default build task`, `colcon: build` and `Run build task`

## Install ROS2 on Windows 10

Follow the instructions on https://index.ros.org/doc/ros2/Installation/Foxy/Windows-Install-Binary/:

1. Run powershell as admin and enter
```console
Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1'))
```

2. Run cmd as admin and enter
```console
choco install -y python --version 3.8.3
choco install -y vcredist2013 vcredist140
```

3. Download and run https://slproweb.com/download/Win64OpenSSL-1_1_1g.exe to install openssl and run
```console
setx OPENSSL_CONF "%ProgramFiles%\OpenSSL-Win64\bin\openssl.cfg" /m
set PATH=%ProgramFiles%\OpenSSL-Win64\bin;%PATH%
```

4. Download and install Visual Studio 2019 Community Edition from https://visualstudio.microsoft.com/downloads. Select `Python development`, `Desktop development with C++` and `unselect C++ CMake tools`.

5. Download and unzip https://s3.amazonaws.com/RTI/Bundles/5.3.1/Evaluation/rti_connext_dds_secure-5.3.1-eval-x64Win64VS2017.zip from https://www.rti.com/free-trial/dds-files-5.3.1, run `rti_connext_dds-5.3.1-eval-x64Win64VS2017.exe` to install RTI Connext DDS and run
```console
set NDDSHOME=%ProgramFiles%\rti_connext_dds-5.3.1
```

6. Download https://github.com/ros2/ros2/releases/download/opencv-archives/opencv-3.4.6-vc16.VS2019.zip, unzip to folder `c:\opencv` and run
```console
setx OpenCV_DIR C:\opencv /m
set PATH=C:\opencv\x64\vc16\bin;%PATH%
```

7. Install cmake by
```console
choco install -y cmake
set PATH=%ProgramFiles%\CMake\bin;%PATH%
```

8. Download asio.1.12.1.nupkg, bullet.2.89.0.nupkg, cunit.2.1.3.nupkg, eigen.3.3.4.nupkg, log4cxx.0.10.0.nupkg, tinyxml-usestl.2.6.2.nupkg, tinyxml2.6.0.0.nupkg from 
https://github.com/ros2/choco-packages/releases/download/2020-02-24 and run
```console
choco install -y -s <PATH\TO\DOWNLOADS\> asio cunit eigen tinyxml-usestl tinyxml2 log4cxx bullet
```
Note: `<PATH\TO\DOWNLOADS\>` with downloaded files *.nupkg must not be a network drive.

9. Install python packages by
```console
python -m pip install -U catkin_pkg cryptography empy ifcfg lark-parser lxml netifaces numpy opencv-python pyparsing pyyaml setuptools
python -m pip install -U pydot PyQt5
python -m pip install -U colcon-common-extensions
python -m pip install -U msgpack
python -m pip install -U matplotlib
```

10. Install graphviz by
```console
choco install graphviz
set PATH=%ProgramFiles(x86)%\Graphviz2.38\bin;%PATH%
```

11. Download ROS2 release version from https://github.com/ros2/ros2/releases/download/release-foxy-20200710/ros2-foxy-20200710-windows-release-amd64.zip (or a later release) and unzip to folder `C:\dev\ros2_foxy`. Run
```console
call C:\dev\ros2_foxy\local_setup.bat
```

12. Test installation by running some basic examples:
```console
ros2 topic list
start "ros2 talker"   ros2 run demo_nodes_cpp talker
start "ros2 listener" ros2 run demo_nodes_py listener
start "ros2 topic"    ros2 topic echo /chatter
rviz2
ros2 run rviz2 rviz2
```

if `ros2 topic list` crashes, downgrade numpy from 1.19.4 to 1.19.3 by using the following command:
```
pip install numpy==1.19.3
```

13. Download https://github.com/git-for-windows/git/releases/download/v2.27.0.windows.1/Git-2.27.0-64-bit.exe (or any later release) and install git. Optionally install a git gui client like gitkraken.

14. Build and test a basic ros2 example using colcon:
```console
cd \dev
md .\ros2_example_ws\src
cd .\ros2_example_ws
git clone https://github.com/ros2/examples src/examples
call "%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64
call C:\dev\ros2_foxy\local_setup.bat
set PATH=C:\opencv\x64\vc16\bin;%ProgramFiles%\CMake\bin;%ProgramFiles%\OpenSSL-Win64\bin;%ProgramFiles(x86)%\Graphviz2.38\bin;%PATH%
colcon build
call .\install\setup.bat
colcon test
start "ros2 example subscriber" ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
start "ros2 example publisher"  ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```

15. Use `ros2 pkg create <package_name>` to create a new ROS2 test package:
```console
cd \dev
mkdir .\ros2_tmp_test_ws
cd .\ros2_tmp_test_ws
ros2 pkg create ros2_tmp_test
```console

16. Download prebuild boost libraries from https://dl.bintray.com/boostorg/release/1.73.0/source/boost_1_73_0.zip (or any later release) and unzip to folder `\boost_1_73_0`

17. Checkout, build and run sick_lidar3d:
```console
cd \dev
mkdir .\sick_lidar3d_ws
cd .\sick_lidar3d_ws

rem get msgpack library
git clone https://github.com/ar90n/msgpack11.git

rem get sick_lidar3d
rem git clone https://github.com/michael1309/sick_lidar3d_pretest.git ./sick_lidar3d # sick_lidar3d pretest (development)
rem git clone https://github.com/michael1309/sick_lidar3d.git                        # sick_lidar3d prerelease
git clone https://github.com/SICKAG/sick_lidar3d.git

call "%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64
call C:\dev\ros2_foxy\local_setup.bat
set PATH=C:\opencv\x64\vc16\bin;%ProgramFiles%\CMake\bin;%ProgramFiles%\OpenSSL-Win64\bin;%ProgramFiles(x86)%\Graphviz2.38\bin;%PATH%

rem build all packages
rem colcon build
colcon build --packages-select msgpack
colcon build --packages-select msgpack11 --cmake-args " -DMSGPACK11_BUILD_TESTS=0" " -DCMAKE_C_FLAGS_DEBUG=/MT" " -DCMAKE_CXX_FLAGS_DEBUG=/MT" " -DCMAKE_C_FLAGS_RELEASE=/MT" " -DCMAKE_CXX_FLAGS_RELEASE=/MT"
colcon build --packages-select sick_lidar3d --cmake-args " -DROS_VERSION=2"

rem Open sick_lidar3d projectfile in Visual Studio
call .\install\setup.bat
start "sick_lidar3d.sln" .\build\sick_lidar3d\sick_lidar3d.sln
```console
