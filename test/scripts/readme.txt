A simple lidar emulator for development and test purposes can be found in folder test/emulator. It implements a simple tcp server, which responds to binary cola messages and sends predefined LMDscandata to a tcp-client. The sick_scan_xd driver can connect to the local test server instead of the lidar device for offline-tests.

To build the test server, activate cmake option `ENABLE_EMULATOR` in CMakeLists.txt and rebuild sick_scan_xd. By default, option `ENABLE_EMULATOR` is switched off.

To run the test server, start sick_scan_xd with one of the emulator launchfiles in test/emulator/launch, f.e.
```
roslaunch sick_scan_xd emulator_01_default.launch # emulate TiM 7xx
roslaunch sick_scan_xd emulator_lms1xx.launch # emulate LMS 1xx
roslaunch sick_scan_xd emulator_lms5xx.launch # emulate LMS 5xx
```

Then start the driver with option hostname:=127.0.0.1, f.e.
```
roslaunch sick_scan_xd sick_tim_7xx.launch hostname:=127.0.0.1
```

To visualize the emulation in rviz, run
```
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg.rviz
```

Please note, that the test server does not emulate a lidar device in details. It simply replies predefined cola responses to the driver and sends LMDscandata from the json-files in test/emulator/scandata for diagnosis and offline-tests. Examples to build and run offline-tests can be found in folder test/scripts.
