## Build on Linux ROS2

Run the following steps to build sick_scan_xd on Linux with ROS 2:

1. Create a workspace folder, e.g. `sick_scan_ws` (or any other name):
   ```
   mkdir -p ./sick_scan_ws
   cd ./sick_scan_ws
   ```

2. Clone repositories https://github.com/SICKAG/libsick_ldmrs, https://github.com/SICKAG/msgpack11.git and https://github.com/SICKAG/sick_scan_xd:
   ```
   mkdir ./src
   pushd ./src
   git clone https://github.com/SICKAG/libsick_ldmrs.git # only required for LDMRS sensors
   git clone https://github.com/SICKAG/msgpack11.git     # only required for Multiscan136 (sick_scansegment_xd)
   git clone https://github.com/SICKAG/sick_scan_xd.git
   popd
   ```

3. Build sick_generic_caller:
   ```
   source /opt/ros/foxy/setup.bash # replace foxy by your ros distro
   colcon build --packages-select libsick_ldmrs --event-handlers console_direct+
   source ./install/setup.bash
   colcon build --packages-select msgpack11 --cmake-args " -DMSGPACK11_BUILD_TESTS=0" --event-handlers console_direct+
   colcon build --packages-select sick_scan --cmake-args " -DROS_VERSION=2" --event-handlers console_direct+
   source ./install/setup.bash
   ```
   For ROS versions other than foxy, please replace `source /opt/ros/foxy/setup.bash` with your ros distribution.

Note: libsick_ldmrs is only required to support LDMRS sensors. If you do not need or want to support LDMRS, you can skip building libsick_ldmrs. To build sick_generic_caller without LDMRS support, switch off option `BUILD_WITH_LDMRS_SUPPORT` in [CMakeLists.txt](./CMakeLists.txt) or call colcon with option `-DLDMRS=0`:
   ```
   colcon build --packages-select sick_scan --cmake-args " -DROS_VERSION=2" " -DLDMRS=0" --event-handlers console_direct+
   ```
Note: msgpack is only required to support Multiscan136/sick_scansegment_xd. If you do not need or want to support Multiscan136/sick_scansegment_xd, you can skip building msgpack. To build sick_generic_caller without Multiscan136/sick_scansegment_xd support, switch off option `BUILD_WITH_SCANSEGMENT_XD_SUPPORT` in [CMakeLists.txt](./CMakeLists.txt) or call cmake with option `-DSCANSEGMENT_XD=0`:
   ```
   colcon build --packages-select sick_scan --cmake-args " -DROS_VERSION=2" " -DSCANSEGMENT_XD=0" --event-handlers console_direct+
   ```

cmake flags can be combined. Use flags `-DLDMRS=0 -DSCANSEGMENT_XD=0` to build without LDMRS and scansegment_xd support:
   ```
   colcon build --packages-select sick_scan --cmake-args " -DROS_VERSION=2" " -DLDMRS=0" " -DSCANSEGMENT_XD=0" --event-handlers console_direct+
   ```

Note: To create source code documentation by doxygen, run
```
cd ./doxygen
doxygen ./docs/Doxyfile
```

## Build on Windows ROS2

To install sick_scan_xd on Windows with ROS-2, follow the steps below:

1. If not yet done, install Visual Studio 2019 and vcpkg as described in [Build on Windows](INSTALL-GENERIC.md#build-on-windows).

2. Create a workspace folder, e.g. `sick_scan_ws` (or any other name):
   ```
   mkdir sick_scan_ws
   cd sick_scan_ws
   ```

3. Clone repositories https://github.com/SICKAG/msgpack11.git and https://github.com/SICKAG/sick_scan_xd:
   ```
   mkdir ./src
   pushd ./src
   git clone https://github.com/SICKAG/msgpack11.git     # only required for Multiscan136 (sick_scansegment_xd)
   git clone https://github.com/SICKAG/sick_scan_xd.git
   popd
   ```

4. Build sick_generic_caller:
   ```
   colcon build --packages-select msgpack11 --cmake-args " -DMSGPACK11_BUILD_TESTS=0" --event-handlers console_direct+ 
   colcon build --packages-select sick_scan --cmake-args " -DROS_VERSION=2" --event-handlers console_direct+
   call .\install\setup.bat
   ```

Note: LDMRS sensors are currently not supported on Windows.

Note: msgpack is only required to support Multiscan136/sick_scansegment_xd. If you do not need or want to support Multiscan136/sick_scansegment_xd, you can skip building msgpack. To build sick_generic_caller without Multiscan136/sick_scansegment_xd support, switch off option `BUILD_WITH_SCANSEGMENT_XD_SUPPORT` in [CMakeLists.txt](./CMakeLists.txt) or call cmake with option `-DSCANSEGMENT_XD=0`:
   ```
   colcon build --packages-select sick_scan --cmake-args " -DROS_VERSION=2" " -DSCANSEGMENT_XD=0" --event-handlers console_direct+
   ```
