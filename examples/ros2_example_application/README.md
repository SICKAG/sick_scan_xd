# sick_scan_xd ROS-2 example

This project contains a tiny ROS-2 example. It shows how to use sick_scan_xd messages and services in a ROS-2 application on Linux. 

## Build

Run the following steps to build sick_scan_xd:

1. Clone repositories https://github.com/SICKAG/libsick_ldmrs and https://github.com/SICKAG/sick_scan_xd parrallel to your application folder. For this example we assume that a folder `workspace/src` exist, incl. a ROS-2 application which will be using sick_scan_xd services and messages.

   ```
   pushd ./workspace/src
   git clone https://github.com/SICKAG/libsick_ldmrs.git
   git clone -b master https://github.com/SICKAG/sick_scan_xd.git
   popd
   ```

2. As an example, we copy folder `workspace/src/sick_scan_xd/examples/ros2_example_application` to `workspace/src/ros2_example_application`. Folder `workspace/src/ros2_example_application` contains our ROS-2 example application. It is just a tiny example, which integrates and uses some sick_scan_xd messages and services. Please feel free to use this example as a starting point for a sick_scan_xd integration into your own ROS2 project.

   ```
   cp -rf workspace/src/sick_scan_xd/examples/ros2_example_application workspace/src/ros2_example_application
   ```
   After this step, folder `workspace/src` should have the following 3 subfolders:
   ```
   workspace/src/libsick_ldmrs
   workspace/src/sick_scan_xd
   workspace/src/ros2_example_application
   ```

3. Build sick_scan_xd as described in [Build on Linux ROS2](../../INSTALL-ROS2.md):

   ```
   cd workspace
   source /opt/ros/$ROS_DISTRO/setup.bash # replace $ROS_DISTRO by your ros distro
   colcon build --packages-select libsick_ldmrs --event-handlers console_direct+
   source ./install/setup.bash
   colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" --event-handlers console_direct+
   source ./install/setup.bash
   ```

   Note: libsick_ldmrs is only required to support LDMRS sensors. If you do not need or want to support LDMRS, you can skip building libsick_ldmrs. To build sick_generic_caller without LDMRS support, run colcon with cmake option `-DLDMRS=0`:
   ```
   colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" " -DLDMRS=0" --event-handlers console_direct+
   ```

   Note: To build sick_generic_caller without multiScan136/sick_scansegment_xd/picoScan150 support, run colcon with cmake option `-DSCANSEGMENT_XD=0`:
   ```
   colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" " -DSCANSEGMENT_XD=0" --event-handlers console_direct+
   ```

   cmake flags can be combined. Use flags `-DLDMRS=0 -DSCANSEGMENT_XD=0` to build without LDMRS and scansegment_xd support:
   ```
   colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" " -DLDMRS=0" " -DSCANSEGMENT_XD=0" --event-handlers console_direct+
   ```

4. Build the ROS-2 example application:

   ```
   colcon build --packages-select sick_scan_ros2_example --event-handlers console_direct+
   ```

   Note: ROS-2 package `sick_scan_ros2_example` uses sick_scan_xd messages and services. File [package.xml](package.xml) contains the following dependency to package sick_scan_xd:
   ```
   <depend>sick_scan_xd</depend>
   ```
   File [CMakeLists.txt](CMakeLists.txt) contains the following dependencies to package sick_scan_xd:
   ```
   find_package(sick_scan_xd REQUIRED)
   ament_target_dependencies(
        sick_scan_ros2_example
        "rclcpp"
        "sensor_msgs"
        "std_msgs"
        "sick_scan_xd"
   )
   ```
   Please include these dependencies in your own ROS-2 project.
   
   File [sick_scan_ros2_example.cpp](src/sick_scan_ros2_example.cpp) shows how to include and use sick_scan_xd messages and services. Please feel free to use this example as a starting point for a sick_scan_xd integration into your own ROS2 project.

## Run the ROS-2 example

After successful build, the executable `sick_scan_ros2_example` has been generated in the install folder `workspace/install/sick_scan_ros2_example/lib/sick_scan_ros2_example`. Run the following steps to test `sick_scan_ros2_example`:

1. Connect a lidar and run the sick_scan_xd driver `sick_generic_caller`: 

    ```
   cd workspace
   source ./install/setup.bash
   ros2 run sick_scan_xd sick_generic_caller ./src/sick_scan_xd/launch/<launchfile>
   ```
   See [Run sick_scan_xd driver](../../USAGE.md) for details.

2. Run `sick_scan_ros2_example`:
    ```
   ros2 run sick_scan_ros2_example sick_scan_ros2_example --ros-args -p topics:=sick_tim_7xx
   ```
   Note: Depending on your lidar, sick_scan_xd messages are published on different topics. Use parameter topics, e.g. `topics:=sick_tim_7xx` for a TiM-7xx lidar. If in doubt, list all sick_scan_xd topics and services by `ros2 topic list` and `ros2 service list`.
