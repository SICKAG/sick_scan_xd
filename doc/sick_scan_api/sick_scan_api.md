# Generic API for sick_scan_xd

## Overview

A generic API for sick_scan_xd has the following goals:
* Easy integration of sick_scan_xd into customer systems with and without ROS
* Integrate SICK lidars with one API, independent of lidar types or underlying operating system
* Provide the same sick_scan_xd functionality on systems without ROS
* In particular: make the sick_scan_xd functionality available on non-ROS-systems without need to customize sources or configuration files.

The generic sick_scan_xd API provides an interface to all lidars supported by sick_scan_xd. This API can be used in C, C++, Python, or any other language with support of C-bindings.

The generic sick_scan_xd API ships with the API-header, the library (binary or sources) and usage examples for C, C++ and Python. The following component diagram shows the relationship between API, library, lidar and a customized application:

![apiComponentsDiagram1.png](apiComponentsDiagram1.png)

## Build and test shared library

The shared library, which implements the C-API, is built native on Linux or Windows (i.e. without ROS). Follow the instructions on [Build on Linux generic without ROS](../../README.md/#build-on-linux-generic-without-ros) for Linux resp. [Build on Windows](../../README.md/#build-on-windows) for Windows.

### Build the shared library on Linux

Run the following commands to build the shared library `libsick_scan_shared_lib.so` on Linux:
```
# Clone repositories
git clone https://github.com/SICKAG/libsick_ldmrs.git
git clone https://github.com/SICKAG/msgpack11.git
git clone https://github.com/SICKAG/sick_scan_xd.git
# Build libsick_ldmrs library
mkdir -p ./build
mkdir -p ./libsick_ldmrs/build
pushd libsick_ldmrs/build
cmake -G "Unix Makefiles" ..
make -j4
sudo make -j4 install    
popd
# Build msgpack library
mkdir -p ./msgpack11/build
pushd msgpack11/build
cmake -DMSGPACK11_BUILD_TESTS=0 -DCMAKE_POSITION_INDEPENDENT_CODE=ON -G "Unix Makefiles" ..
make -j4
sudo make -j4 install    
popd
# Build libsick_scan_shared_lib.so
pushd ./build
export ROS_VERSION=0
cmake -DROS_VERSION=0 -G "Unix Makefiles" ../sick_scan_xd
make -j4
sudo make -j4 install    
# Check build and library dependencies
ls -al ./sick_generic_caller
ls -al ./libsick_scan_shared_lib.so
ls -al ./sick_scan_xd_api_test
ldd -r ./libsick_scan_shared_lib.so
popd
```
After successfull build, the shared library `libsick_scan_shared_lib.so` and a tiny test executable `sick_scan_xd_api_test` are created. 

### Build the shared library on Windows

Run the following commands to build the shared library `sick_scan_shared_lib.dll` with Visual Studio 2019 on Windows:
```
# Clone repositories
git clone https://github.com/SICKAG/msgpack11.git
git clone https://github.com/SICKAG/sick_scan_xd.git
# Build libraries msgpack and sick_scan_shared_lib.dll
call "%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64
set _os=x64
set _cmake_string=Visual Studio 16
set _msvc=Visual Studio 2019
set _cmake_build_dir=build
if not exist %_cmake_build_dir% mkdir %_cmake_build_dir%
if not exist %_cmake_build_dir%\msgpack11 mkdir %_cmake_build_dir%\msgpack11
pushd %_cmake_build_dir%\msgpack11
cmake -DMSGPACK11_BUILD_TESTS=0 -G "%_cmake_string%" ../../../msgpack11
if %ERRORLEVEL% neq 0 ( @echo ERROR building %_cmake_string% msgpack11 with cmake & @pause )
cmake --build . --clean-first --config Debug
cmake --build . --clean-first --config Release
popd
pushd %_cmake_build_dir%
cmake -DROS_VERSION=0 -DCMAKE_ENABLE_EMULATOR=1 -G "%_cmake_string%" ..
if %ERRORLEVEL% neq 0 ( @echo ERROR building %_cmake_string% sick_scan_xd with cmake & @pause )
cmake --build . --clean-first --config Debug
```
After successfull build, the shared library `sick_scan_shared_lib.dll` and a tiny test executable `sick_scan_xd_api_test.exe` are created. To install the library and header in the system folder, run `cmake --build . --target install` with admin priviledges. Note that LDMRS is not supported on Windows.

### Test the shared library

The executable file `sick_scan_xd_api_test` provides a minimalistic API test. Run `sick_scan_xd_api_test <launchfile> hostname:=<ip-address>` to test the API against a lidar, e.g. on Linux:
```
# export LD_LIBRARY_PATH=.:./build:$LD_LIBRARY_PATH # append relative path to build folder
export LD_LIBRARY_PATH=.:`pwd`/build:$LD_LIBRARY_PATH  # append absolute path to build folder
./build/sick_scan_xd_api_test ./sick_scan_xd/launch/sick_tim_7xx.launch hostname:=192.168.0.1
```
On Windows, run e.g.
```
set PATH=.;.\build;..\build\Debug;%PATH%
.\build\Debug\sick_scan_xd_api_test.exe launch/sick_lms_5xx.launch hostname:=192.168.0.1
```

The executable binary `sick_scan_xd_api_test` will just load library `libsick_scan_shared_lib.so` resp. `sick_scan_shared_lib.dll`, start the lidar and print a message when receiving lidar messages, e.g. `sick_scan_xd_api_test: pointcloud callback`. Replace `sick_lms_1xx.launch` in the example by the launchfile corresponding to your type of lidar.

To load the library, the build folder has to be included in `LD_LIBRARY_PATH` (Linux) resp. `PATH` (Windows). Set this environment variable to your build folder, e.g. on Linux using
```
# export LD_LIBRARY_PATH=.:./build:$LD_LIBRARY_PATH # append relative path to build folder
export LD_LIBRARY_PATH=.:`pwd`/build:$LD_LIBRARY_PATH  # append absolute path to build folder
```
resp. on Windows
```
set PATH=.;.\build;.\build\Debug;%PATH%
```

## Usage example

The sick_scan_xd API can be used on Linux or Windows in any language with support of C-bindings. There are 3 steps required to use the API:

1. API- and lidar-initialization by
    * SickScanApiLoadLibrary
    * SickScanApiCreate
    * SickScanApiInitByLaunchfile or SickScanApiInitByCli

2. Receive messages by registration of callbacks using `SickScanApiRegister<MsgType>Msg`-functions (recommended) or by polling using `SickScanApiWaitNext<MsgType>Msg`-functions

3. Close lidar and API by
    * `SickScanApiDeregister<MsgType>Msg`-functions
    * SickScanApiClose
    * SickScanApiRelease

All functions named `SickScanApi` are implemented within the library file ("sick_scan_shared_lib.dll" on Windows resp. "libsick_scan_shared_lib.so" on Linux). A small wrapper is provided ([sick_scan_xd_api_wrapper.c](../../test/src/sick_scan_xd_api/sick_scan_xd_api_wrapper.c) for C/C++, [sick_scan_api.py](../../python/api/sick_scan_api.py) for python), which loads and unloads the library (functions `SickScanApiLoadLibrary` and `SickScanApiUnloadLibrary`) and delegates the function calls to the binary.

### Minimalistic usage example in C

File [minimum_sick_scan_api_client.c](../../examples/c/minimum_sick_scan_api_client.c) shows a minimalistic example of a C client using the sick_scan_xd API. To build and run this example, open a command shell in folder `examples/scripts` and run `.\build_run_api_examples_linux.bash` on Linux resp. `build_run_api_examples_windows.cmd` on Windows. Make sure, that the shared library `libsick_scan_shared_lib.so` resp. `sick_scan_shared_lib.dll` has been successfully built in the build-folder.

Alternatively, follow the build and run instructions on Linux:
```
cd examples/c
mkdir -p ./build
cd ./build
cmake -G "Unix Makefiles" ..
make -j4
cd ../..
export LD_LIBRARY_PATH=.:./build:$LD_LIBRARY_PATH
./examples/c/build/minimum_sick_scan_api_client <launchfile> hostname:=<lidar-ip-address>
```

Alternatively, follow the build and run instructions on Windows:
```
REM Set environment for Visual Studio 2019 (VS 16)
set _os=x64
set _cmake_string=Visual Studio 16
set _msvc=Visual Studio 2019
set _cmake_build_dir=build
REM Build the minimalistic C usage example
cd examples\c
mkdir %_cmake_build_dir%
cd %_cmake_build_dir%
cmake -G "%_cmake_string%" ..
cmake --build . --clean-first --config Debug
REM Set environment: add build folder to LD_LIBRARY_PATH, add python/api to PYTHONPATH
cd ..\..
set PATH=.;.\build;.\build\Debug;.\build_win64;.\build_win64\Debug;%PATH%
REM Run minimalistic C api example
.\examples\c\build\Debug\minimum_sick_scan_api_client.exe <launchfile> hostname:=<lidar-ip-address>
```

### Minimalistic usage example in C++

File [minimum_sick_scan_api_client.cpp](../../examples/cpp/minimum_sick_scan_api_client.cpp) shows a minimalistic example of a C++ client using the sick_scan_xd API. To build and run this example, open a command shell in folder `examples/scripts` and run `.\build_run_api_examples_linux.bash` on Linux resp. `build_run_api_examples_windows.cmd` on Windows. Make sure, that the shared library `libsick_scan_shared_lib.so` resp. `sick_scan_shared_lib.dll` has been successfully built in the build-folder.

Alternatively, follow the build and run instructions on Linux:
```
cd examples/cpp
mkdir -p ./build
cd ./build
cmake -G "Unix Makefiles" ..
make -j4
cd ../..
export LD_LIBRARY_PATH=.:./build:$LD_LIBRARY_PATH
./examples/cpp/build/minimum_sick_scan_api_client <launchfile> hostname:=<lidar-ip-address>
```

Alternatively, follow the build and run instructions on Windows:
```
REM Set environment for Visual Studio 2019 (VS 16)
set _os=x64
set _cmake_string=Visual Studio 16
set _msvc=Visual Studio 2019
set _cmake_build_dir=build
REM Build the minimalistic C++ usage example
cd examples\cpp
mkdir %_cmake_build_dir%
cd %_cmake_build_dir%
cmake -G "%_cmake_string%" ..
cmake --build . --clean-first --config Debug
REM Set environment: add build folder to LD_LIBRARY_PATH, add python/api to PYTHONPATH
cd ..\..
set PATH=.;.\build;.\build\Debug;.\build_win64;.\build_win64\Debug;%PATH%
REM Run minimalistic C++ api example
.\examples\cpp\build\Debug\minimum_sick_scan_api_client.exe <launchfile> hostname:=<lidar-ip-address>
```

### Minimalistic usage example in Python

File [minimum_sick_scan_api_client.py](../../examples/python/minimum_sick_scan_api_client.py) shows a minimalistic example of a python client using the sick_scan_xd API. To build and run this example, open a command shell in folder `examples/scripts` and run `.\build_run_api_examples_linux.bash` on Linux resp. `build_run_api_examples_windows.cmd` on Windows. Make sure, that the shared library `libsick_scan_shared_lib.so` resp. `sick_scan_shared_lib.dll` has been successfully built in the build-folder.

Alternatively, follow the run instructions on Linux:
```
export LD_LIBRARY_PATH=.:./build:$LD_LIBRARY_PATH
export PYTHONPATH=.:./python/api:$PYTHONPATH
python3 ./examples/python/minimum_sick_scan_api_client.py <launchfile> hostname:=<lidar-ip-address>
```

Alternatively, follow the run instructions on Windows:
```
set PATH=.;.\build;.\build\Debug;.\build_win64;.\build_win64\Debug;%PATH%
set PYTHONPATH=.;.\python\api;%PATH%
python ./examples/python/minimum_sick_scan_api_client.py <launchfile> hostname:=<lidar-ip-address>
```

### Complete usage example in C++

A complete C/C++ usage example is implemented in [sick_scan_xd_api_test.cpp](../../test/src/sick_scan_xd_api/sick_scan_xd_api_test.cpp). Note that the shared library ("sick_scan_shared_lib.dll" on Windows resp. "libsick_scan_shared_lib.so" on Linux) has no dependencies to ROS. The usage example on the other hand supports both ROS-1, ROS-2 and native Linux or Windows. When build on ROS, it converts the SickScanApi-messages into ROS-messages. On ROS, they can be visualized by rviz. The following screenshot shows a pointcloud published by `rosrun sick_scan sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_tim_7xx.launch"`:

![api_test_linux_ros1_tim7xx.png](api_test_linux_ros1_tim7xx.png)

Without ROS, sick_scan_xd_api_test plots a jpeg-file to enable a simple visualization of a pointcloud. E.g.:
```
firefox ./demo/image_viewer_api_test.html &
./build_linux/sick_scan_xd_api_test ./launch/sick_tim_7xx.launch
```
![api_test_linux_tim7xx.png](api_test_linux_tim7xx.png)

### Complete usage example in Python

A complete python usage example is implemented in [sick_scan_xd_api_test.py](../../test/python/sick_scan_xd_api/sick_scan_xd_api_test.py). It is handy to test the sick_scan_xd library. Like its C++ counterpart [sick_scan_xd_api_test.cpp](../../test/src/sick_scan_xd_api/sick_scan_xd_api_test.cpp), it just loads library `libsick_scan_shared_lib.so` resp. `sick_scan_shared_lib.dll`, starts a lidar and receives the lidar pointcloud and messages via API. On ROS-1, the lidar pointcloud and messages are converted to ROS and published. The lidar pointcloud can be visualized by rviz using topic "/sick_scan_xd_api_test/api_cloud".

Run `python3 sick_scan_xd_api_test.py <launchfile> hostname:=<ip-address>` to test the API against a lidar, e.g.:

```
export PYTHONPATH=.:./src/sick_scan_xd/python/api:$PYTHONPATH
source /opt/ros/noetic/setup.bash # replace by noetic by your ros version
python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py ./src/sick_scan_xd/launch/sick_lms_1xx.launch hostname:=192.168.0.1
```

The pthon usage example [sick_scan_xd_api_test.py](../../test/python/sick_scan_xd_api/sick_scan_xd_api_test.py) imports [sick_scan_api.py](../../python/api/sick_scan_api.py), which contains the python definitions of the sick_scan_xd API. Make sure that sick_scan_api.py can be imported, e.g. by including folder `python/api` in PYTHONPATH by:

`export PYTHONPATH=.:./src/sick_scan_xd/python/api:$PYTHONPATH` on Linux, resp. <br/>
`set PYTHONPATH=.;.\src\sick_scan_xd\python\api;%PYTHONPATH%` on Windows

Note: The shared library ("sick_scan_shared_lib.dll" on Windows resp. "libsick_scan_shared_lib.so" on Linux) works standalone and does not have any ROS dependancies. The usage example [sick_scan_xd_api_test.py](../../test/python/sick_scan_xd_api/sick_scan_xd_api_test.py) converts API- to ROS-messages for visualization and is therefore dependent on ROS, if ROS is installed. 

If ROS is not installed, [sick_scan_xd_api_test.py](../../test/python/sick_scan_xd_api/sick_scan_xd_api_test.py) uses matplotlib to visualize the pointcloud. The following screenshot shows a TiM-7xx pointcloud on Linux without ROS:

![api_test_python_tim7xx.png](api_test_python_tim7xx.png)

### Simulation and unittest

sick_scan_xd provides a tiny server for offline tests which simulates a basic lidar. It just accepts TCP connections, responds to sopas requests with predefined responses and sends lidar data from file. See [Simulation](../../README.md/#simulation) for further details. Note that the simulation does not emulate or replace a lidar, it just supports basic unittests.

Open a new terminal and run the following steps to test the api against a TiM7xx simulation using the python example mentioned above:

1. Build library `libsick_scan_shared_lib.so` incl. emulator with option `-DCMAKE_ENABLE_EMULATOR=1`:
   ```
   mkdir -p ./src/build
   pushd ./src/build
   rm -rf ./*
   cmake -DROS_VERSION=0 -DCMAKE_ENABLE_EMULATOR=1 -G "Unix Makefiles" ../sick_scan_xd
   make -j4
   ls -al libsick_scan_shared_lib.so sick_scan_xd_api_test sick_generic_caller sick_scan_emulator # list size and date of the binaries
   popd
   ```

2. Build sick_scan_xd for ROS-1 on Linux, see [Build on Linux ROS1](../../README.md/#build-on-linux-ros1)

3. Start the TiM7xx simulator:
   ```
   cp -f ./src/sick_scan_xd/test/emulator/scandata/sopas_et_field_test_1_2_both_010.pcapng.json /tmp/lmd_scandata.pcapng.json
   ./src/build/sick_scan_emulator ./src/sick_scan_xd/test/emulator/launch/emulator_01_default.launch &
   sleep 1
   ```

4. Run sick_scan_xd_api_test.py against the TiM7xx simulator on localhost:
   ```
   export PYTHONPATH=.:./src/sick_scan_xd/python/api:$PYTHONPATH
   python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py ./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False
   ```

5. Start rviz and visualize the pointcloud on topic "/sick_scan_xd_api_test/api_cloud".

Note: The shared library ("sick_scan_shared_lib.dll" on Windows resp. "libsick_scan_shared_lib.so" on Linux) works standalone and does not have any ROS dependancies. The usage example [sick_scan_xd_api_test.py](../../test/python/sick_scan_xd_api/sick_scan_xd_api_test.py) uses ROS for visualization.

## C-API

The header file [sick_scan_api.h](../../include/sick_scan_xd_api/sick_scan_api.h) defines the C-interface. It defines all datatypes, messages and functions of the generic sick_scan_xd API. To allow equal operations on all systems, the definition of datatypes and messages is as close as possible to their equivalents currently used on ROS.

Python file [sick_scan_api.py](../../python/api/sick_scan_api.py) defines the same interface in python.

## Useful links

[ctypes](https://docs.python.org/3/library/ctypes.html) is used for data exchange and function calls between Python and C-libraries:
* https://docs.python.org/3/library/ctypes.html
* https://docs.python.org/3/library/ctypes.html#structures-and-unions
* https://docs.python.org/3/library/ctypes.html#callback-functions
