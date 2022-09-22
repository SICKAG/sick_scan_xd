## Build on Linux ROS1

Run the following steps to build sick_scan_xd on Linux with ROS 1:

1. Create a workspace folder, e.g. `sick_scan_ws` (or any other name):
   ```
   mkdir -p ./sick_scan_ws
   cd ./sick_scan_ws
   ```

2. Clone repositories https://github.com/SICKAG/libsick_ldmrs, https://github.com/SICKAG/msgpack11.git and https://github.com/SICKAG/sick_scan_xd:
   ```
   mkdir ./src
   pushd ./src
   git clone https://github.com/SICKAG/libsick_ldmrs.git
   git clone https://github.com/SICKAG/msgpack11.git
   git clone https://github.com/SICKAG/sick_scan_xd.git
   popd
   ```

3. Build msgpack11 library (required only once for Multiscan136/sick_scansegment_xd):
   ```
   mkdir -p ./build/msgpack11
   pushd ./build/msgpack11
   cmake -G "Unix Makefiles" -D CMAKE_CXX_FLAGS=-fPIC -D CMAKE_BUILD_TYPE=Release -D MSGPACK11_BUILD_TESTS=0 ../../src/msgpack11
   make
   sudo make install
   popd
   ```

4. Build sick_generic_caller:
   ```
   source /opt/ros/noetic/setup.bash # replace noetic by your ros distro
   catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -Wno-dev
   source ./devel_isolated/setup.bash
   # source ./install_isolated/setup.bash
   ```
   For ROS versions other than noetic, please replace `source /opt/ros/noetic/setup.bash` with your ros distribution.

Note: libsick_ldmrs is only required to support LDMRS sensors. If you do not need or want to support LDMRS, you can skip building libsick_ldmrs. To build sick_generic_caller without LDMRS support, switch off option `BUILD_WITH_LDMRS_SUPPORT` in [CMakeLists.txt](./CMakeLists.txt) or call catkin_make_isolated with option `-DLDMRS=0`:
   ```
   catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DLDMRS=0 -Wno-dev
   ```

Note: msgpack11 is only required to support Multiscan136/sick_scansegment_xd. If you do not need or want to support Multiscan136/sick_scansegment_xd, you can skip building msgpack. To build sick_generic_caller without Multiscan136/sick_scansegment_xd support, switch off option `BUILD_WITH_SCANSEGMENT_XD_SUPPORT` in [CMakeLists.txt](./CMakeLists.txt) or call cmake with option `-DSCANSEGMENT_XD=0`:
   ```
   catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DSCANSEGMENT_XD=0 -Wno-dev
   ```

cmake flags can be combined. Use flags `-DLDMRS=0 -DSCANSEGMENT_XD=0` to build without LDMRS and scansegment_xd support:
   ```
   catkin_make_isolated --install --cmake-args -DROS_VERSION=1 -DLDMRS=0 -DSCANSEGMENT_XD=0 -Wno-dev
   ```

Note: To create source code documentation by doxygen, run
```
cd ./doxygen
doxygen ./docs/Doxyfile
```
