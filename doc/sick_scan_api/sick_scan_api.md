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
export LD_LIBRARY_PATH=.:./build:$LD_LIBRARY_PATH
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

The following code snippet shows a minimalistic example of a C/C++ application using the sick_scan_xd API:

```
#include "sick_scan_api.h"

// Implement a callback to process pointcloud messages
void customizedPointCloudMsgCb(SickScanApiHandle apiHandle, const SickScanPointCloudMsg* msg)
{
    printf("pointcloud message received\n"); // data processing to be done
}

// Create a sick_scan instance and initialize a LMS-511
SickScanApiLoadLibrary("libsick_scan_shared_lib.so");
SickScanApiHandle apiHandle = SickScanApiCreate(argc, argv);
SickScanApiInitByLaunchfile(apiHandle, "sick_lms_5xx.launch");

// Register for pointcloud messages
SickScanApiRegisterPointCloudMsg(apiHandle, &customizedPointCloudMsgCb);

// Register for pointcloud messages
SickScanApiRegisterCartesianPointCloudMsg(apiHandle, customizedPointCloudMsgCallback);

// Run application or main loop
getchar();

// Close lidar and release sick_scan api
SickScanApiDeregisterPointCloudMsg(apiHandle, customizedPointCloudMsgCallback);
SickScanApiClose(apiHandle);
SickScanApiRelease(apiHandle);
SickScanApiUnloadLibrary();
```

Note: All functions named `SickScanApi` are implemented within the library file ("sick_scan_shared_lib.dll" on Windows resp. "libsick_scan_shared_lib.so" on Linux). A small wrapper is included in the examples, which loads and unloads the library (functions `SickScanApiLoadLibrary` and `SickScanApiUnloadLibrary`) and delegates the function calls to the binary.

A complete C/C++ usage example is implemented in [sick_scan_xd_api_test.cpp](../../test/src/sick_scan_xd_api/sick_scan_xd_api_test.cpp). Note that the shared library ("sick_scan_shared_lib.dll" on Windows resp. "libsick_scan_shared_lib.so" on Linux) has no dependencies to ROS. The usage example on the other hand supports both ROS-1, ROS-2 and native Linux or Windows. When build on ROS, it converts the SickScanApi-messages into ROS-messages. On ROS, they can be visualized by rviz. The following screenshot shows a pointcloud published by `rosrun sick_scan sick_scan_xd_api_test _sick_scan_args:="./src/sick_scan_xd/launch/sick_tim_7xx.launch"`:

![api_test_linux_ros1_tim7xx.png](api_test_linux_ros1_tim7xx.png)

Without ROS, sick_scan_xd_api_test plots a jpeg-file to enable a simple visualization of a pointcloud. E.g.:
```
firefox ./demo/image_viewer_api_test.html &
./build_linux/sick_scan_xd_api_test ./launch/sick_tim_7xx.launch
```
![api_test_linux_tim7xx.png](api_test_linux_tim7xx.png)

A complete python usage example is implemented in [sick_scan_xd_api_test.py](../../test/python/sick_scan_xd_api/sick_scan_xd_api_test.py).

### Python example

The following code snippet shows a minimalistic example of a python application using the sick_scan_xd API:

```
import sick_scan_api.py

# Implement a callback to process pointcloud messages
def pyCustomizedPointCloudMsgCb(api_handle, msg):
    print("pointcloud message received") # data processing to be done

# Create a sick_scan instance and initialize a LMS-511
sick_scan_library = SickScanApiLoadLibrary(["build/", "./"], "libsick_scan_shared_lib.so")
api_handle = SickScanApiCreate(sick_scan_library)
SickScanApiInitByLaunchfile(sick_scan_library, api_handle, "sick_lms_5xx.launch")

# Register for pointcloud messages
cartesian_pointcloud_callback = SickScanPointCloudMsgCallback(pyCustomizedPointCloudMsgCb)
SickScanApiRegisterCartesianPointCloudMsg(sick_scan_library, api_handle, cartesian_pointcloud_callback)

# Run application or main loop
time.sleep(60)

# Close lidar and release sick_scan api
SickScanApiDeregisterCartesianPointCloudMsg(sick_scan_library, api_handle, cartesian_pointcloud_callback)
SickScanApiClose(sick_scan_library, api_handle)
SickScanApiRelease(sick_scan_library, api_handle)
SickScanApiUnloadLibrary(sick_scan_library)
```

Python example [sick_scan_xd_api_test.py](../../test/python/sick_scan_xd_api/sick_scan_xd_api_test.py) is handy to test the sick_scan_xd library. Like its C++ counterpart [sick_scan_xd_api_test.cpp](../../test/src/sick_scan_xd_api/sick_scan_xd_api_test.cpp), it just loads library `libsick_scan_shared_lib.so` resp. `sick_scan_shared_lib.dll`, starts a lidar and receives the lidar pointcloud and messages via API. On ROS-1, the lidar pointcloud and messages are converted to ROS and published. The lidar pointcloud can be visualized by rviz using topic "/sick_scan_xd_api_test/api_cloud".

Run `python3 sick_scan_xd_api_test.py <launchfile> hostname:=<ip-address>` to test the API against a lidar, e.g.:

```
source /opt/ros/noetic/setup.bash # replace by noetic by your ros version
python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py ./src/sick_scan_xd/launch/sick_lms_1xx.launch hostname:=192.168.0.1
```

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
   python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py ./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False
   ```

5. Start rviz and visualize the pointcloud on topic "/sick_scan_xd_api_test/api_cloud".

Note: The shared library ("sick_scan_shared_lib.dll" on Windows resp. "libsick_scan_shared_lib.so" on Linux) works standalone and does not have any ROS dependancies. The usage example [sick_scan_xd_api_test.py](../../test/python/sick_scan_xd_api/sick_scan_xd_api_test.py) uses ROS for visualization.

## C-API

The header file [sick_scan_api.h](../../include/sick_scan_xd_api/sick_scan_api.h) defines the C-interface. It defines all datatypes, messages and functions of the generic sick_scan_xd API. To allow equal operations on all systems, the definition of datatypes and messages is as close as possible to their equivalents currently used on ROS.

Python file [sick_scan_api.py](../../test/python/sick_scan_xd_api/sick_scan_api.py) defines the same interface in python.

## Useful links

[ctypes](https://docs.python.org/3/library/ctypes.html) is used for data exchange and function calls between Python and C-libraries:
* https://docs.python.org/3/library/ctypes.html
* https://docs.python.org/3/library/ctypes.html#structures-and-unions
* https://docs.python.org/3/library/ctypes.html#callback-functions
