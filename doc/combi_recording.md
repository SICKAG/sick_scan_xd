# Checking driver in combination of MRS6xxx and RMS3xx

1. Setup environment
   * 230 V Voltage converter
   * Power supply notebook
   * Power bank
   * Tripod
   * Measurement Unit
   * Switch
   * Permit for measurement in public area
2. roslaunch sick_scan test_0002_combi_live.launch
3. Check setup using rviz
4. Close all applications, which are not necessary (like IDE, browser, git client)
5. Setup Tracking algorithm

```
top
```
6. Record data
```
rosrun rosbag record record -o combi -a
```


