## Build on Linux generic without ROS

Run the following steps to build sick_scan_xd on Linux (no ROS required):

1. Create a workspace folder, e.g. `sick_scan_ws` (or any other name):
   ```
   mkdir -p ./sick_scan_ws
   cd ./sick_scan_ws
   ```

2. Clone repositories https://github.com/SICKAG/libsick_ldmrs and https://github.com/SICKAG/sick_scan_xd:
   ```
   git clone https://github.com/SICKAG/libsick_ldmrs.git
   git clone -b master https://github.com/SICKAG/sick_scan_xd.git
   ```

3. Build libsick_ldmrs (required only once for LDMRS sensors):
   ```
   pushd libsick_ldmrs
   mkdir -p ./build
   cd ./build
   cmake -G "Unix Makefiles" ..
   make -j4
   sudo make -j4 install    
   popd
   ```

4. Build sick_generic_caller and libsick_scan_xd_shared_lib.so:
   ```
   mkdir -p ./build
   pushd ./build
   rm -rf ./*
   export ROS_VERSION=0
   cmake -DROS_VERSION=0 -G "Unix Makefiles" ../sick_scan_xd
   make -j4
   sudo make -j4 install    
   popd
   ```

Note: LDMRS sensors are currently not supported on Raspberry. Build with cmake flag `-DLDMRS=0 -DRASPBERRY=1` on Raspberry:
   ```
   cmake -DROS_VERSION=0 -DLDMRS=0 -DRASPBERRY=1 -G "Unix Makefiles" ../sick_scan_xd
   ```

Note: libsick_ldmrs is only required to support LDMRS sensors. If you do not need or want to support LDMRS, you can skip building libsick_ldmrs. To build sick_generic_caller without LDMRS support, switch off option `BUILD_WITH_LDMRS_SUPPORT` in [CMakeLists.txt](./CMakeLists.txt) or call cmake with option `-DLDMRS=0`:
   ```
   cmake -DROS_VERSION=0 -DLDMRS=0 -G "Unix Makefiles" ../sick_scan_xd
   ```

Note: To build sick_generic_caller without multiScan100/picoScan100 support, switch off option `BUILD_WITH_SCANSEGMENT_XD_SUPPORT` in [CMakeLists.txt](./CMakeLists.txt) or call cmake with option `-DSCANSEGMENT_XD=0`:
   ```
   cmake -DROS_VERSION=0 -DSCANSEGMENT_XD=0 -G "Unix Makefiles" ../sick_scan_xd
   ```

cmake flags can be combined. Use flags `-DLDMRS=0 -DSCANSEGMENT_XD=0` to build without LDMRS and scansegment_xd support:
   ```
   cmake -DROS_VERSION=0 -DLDMRS=0 -DSCANSEGMENT_XD=0 -G "Unix Makefiles" ../sick_scan_xd
   ```

Note: To create source code documentation by doxygen, run
```
cd ./doxygen
doxygen ./docs/Doxyfile
```

## Build on Windows

To install sick_scan_xd on Windows, follow the steps below:

1. If not yet done, install Visual Studio. Visual Studio 2019 Community or Professional Edition is recommended.

2. Create a workspace folder, e.g. `sick_scan_ws` (or any other name):
   ```
   mkdir sick_scan_ws
   cd sick_scan_ws
   ```

3. Clone repository https://github.com/SICKAG/sick_scan_xd:
   ```
   git clone -b master https://github.com/SICKAG/sick_scan_xd.git
   ```

4. Build sick_generic_caller and sick_scan_xd_shared_lib.dll with cmake and Visual Studio 2019:
   ```
   cd sick_scan_xd
   set _os=x64
   set _cmake_string=Visual Studio 16 2019
   set _msvc=Visual Studio 2019
   set _cmake_build_dir=build
   if not exist %_cmake_build_dir% mkdir %_cmake_build_dir%
   pushd %_cmake_build_dir%
   cmake -DROS_VERSION=0 -G "%_cmake_string%" ..
   cmake --build . --clean-first --config Debug
   cmake --build . --clean-first --config Release
   REM open sick_scan_xd.sln in Visual Studio 2019 for development and debugging
   popd
   ```
   For development or debugging, open file `sick_scan_xd\build\sick_scan_xd.sln` in Visual Studio. To install the library and header in the system folder, run `cmake --build . --target install` with admin priviledges. 

After successful build, binary files `sick_generic_caller.exe` and `sick_scan_xd_shared_lib.dll` are created in folders `sick_scan_xd\build\Debug` and `sick_scan_xd\build\Release`.

Note: LDMRS sensors are currently not supported on Windows.

Note: To build sick_generic_caller without multiScan100/picoScan100 support, switch off option `BUILD_WITH_SCANSEGMENT_XD_SUPPORT` in [CMakeLists.txt](./CMakeLists.txt) or call cmake with option `-DSCANSEGMENT_XD=0`:
   ```
   cmake -DROS_VERSION=0 -DSCANSEGMENT_XD=0 -G "%_cmake_string%" ..
   ```
