# Startup Sequence

## Table of contents

- [Introduction](#introduction)
- [Quick Check of Firmware](#quick-check-of-firmware)
- [Startup Sequence](#startup-sequence)

## Introduction

The following ROS boot protocol shows the typical start sequence when starting a SICK laser scanner. The MRS6124 is shown here as an example. However, the startup sequence is generally similar for all scanners.

## Quick Check of firmware

After a firmware update, the following Quickcheck is performed:

1. Is the device accessible via ping?
2. Can the device be started with the corresponding generic launch file?
3. At the end of the launch process, the device switches to receive mode
    for scan data? Typically the last command sent is ```<STX>sEA LMDscandata \x01<ETX>```.
4. Check with rviz: Is it possible to see the Pointcloud2 data or similar? Is the shown data reasonable?
5. Check the scan rate with the command
```
rostopic hz /cloud
```

6. Further inspection, if any, by dumping Pointcloud2 data.
The header is of particular interest here. A typical call can therefore look as follows:
```
rostopic echo /cloud|grep frame -B 7 -A 26
```


## Startup Sequence
```

roslaunch sick_scan sick_mrs_6xxx.launch hostname:=192.168.0.25
... logging to /home/rosuser/.ros/log/75631922-6109-11e9-b76f-54e1ad2921b6/roslaunch-ROS-NB-10680.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ROS-NB:40757/

SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.3
 * /sick_mrs_6xxx/filter_echos: 0
 * /sick_mrs_6xxx/hostname: 192.168.0.25
 * /sick_mrs_6xxx/max_ang: 1.047197333
 * /sick_mrs_6xxx/min_ang: -1.040216
 * /sick_mrs_6xxx/port: 2112
 * /sick_mrs_6xxx/range_max: 250.0
 * /sick_mrs_6xxx/range_min: 0.1
 * /sick_mrs_6xxx/scanner_type: sick_mrs_6xxx
 * /sick_mrs_6xxx/timelimit: 5
 * /sick_mrs_6xxx/use_binary_protocol: True

NODES
  /
    sick_mrs_6xxx (sick_scan/sick_generic_caller)

auto-starting new master
process[master]: started with pid [10690]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 75631922-6109-11e9-b76f-54e1ad2921b6
process[rosout-1]: started with pid [10701]
started core service [/rosout]
process[sick_mrs_6xxx-2]: started with pid [10708]
[ INFO] [1555502887.036684738]: sick_generic_caller V. 001.003.016
[ INFO] [1555502887.036717573]: Program arguments: /home/rosuser/ros_catkin_ws/devel/lib/sick_scan/sick_generic_caller
[ INFO] [1555502887.036725741]: Program arguments: __name:=sick_mrs_6xxx
[ INFO] [1555502887.036731933]: Program arguments: __log:=/home/rosuser/.ros/log/75631922-6109-11e9-b76f-54e1ad2921b6/sick_mrs_6xxx-2.log
[ INFO] [1555502887.048425000]: Found sopas_protocol_type param overwriting default protocol:
[ INFO] [1555502887.048956468]: Binary protocol activated
[ INFO] [1555502887.048984179]: Start initialising scanner [Ip: 192.168.0.25] [Port: 2112]
[ INFO] [1555502887.067528995]: Publishing laserscan-pointcloud2 to cloud
[ INFO] [1555502887.071035827]: Parameter setting for <active_echo: 0>
[ INFO] [1555502887.271739084]: Sending  : <STX><STX><STX><STX><Len=0023>sMN SetAccessMode 0x03 0xf4 0x72 0x47 0x44 CRC:<0xb3>
[ INFO] [1555502887.273290840]: Receiving: <STX>sAN SetAccessMode \x01<ETX>
[ INFO] [1555502887.473927858]: Sending  : <STX><STX><STX><STX><Len=0015>sWN EIHstCola 0x01 CRC:<0x09>
[ INFO] [1555502887.475365983]: Receiving: <STX>sWA EIHstCola <ETX>
[ INFO] [1555502887.675864993]: Sending  : <STX><STX><STX><STX><Len=0015>sMN LMCstopmeas CRC:<0x10>
[ INFO] [1555502888.199590269]: Receiving: <STX>sAN LMCstopmeas \x00<ETX>
[ INFO] [1555502888.400030148]: Sending  : <STX><STX><STX><STX><Len=0015>sRN DeviceIdent CRC:<0x25>
[ INFO] [1555502888.401393378]: Receiving: <STX>sRA DeviceIdent \x00\x08\x4d\x52\x53\x36\x31\x32\x34\x52\x00\x0a\x31\x2e\x31\x2e\x30\x2e\x35\x36\x35\x43<ETX>
[ INFO] [1555502888.401653485]: Deviceinfo MRS6124R V1.1.0.565C found and supported by this driver.
[ INFO] [1555502888.602062286]: Sending  : <STX><STX><STX><STX><Len=0019>sRN FirmwareVersion CRC:<0x24>
[ INFO] [1555502888.603444526]: Receiving: <STX>sRA FirmwareVersion \x00\x0a\x31\x2e\x31\x2e\x30\x2e\x35\x36\x35\x43<ETX>
[ INFO] [1555502888.804094446]: Sending  : <STX><STX><STX><STX><Len=0017>sRN SCdevicestate CRC:<0x30>
[ INFO] [1555502888.805521867]: Receiving: <STX>sRA SCdevicestate \x01<ETX>
[ INFO] [1555502889.006161400]: Sending  : <STX><STX><STX><STX><Len=0010>sRN ODoprh CRC:<0x41>
[ INFO] [1555502889.007613972]: Receiving: <STX>sRA ODoprh \x00\x00\x19\xf1<ETX>
[ INFO] [1555502889.209949897]: Sending  : <STX><STX><STX><STX><Len=0010>sRN ODpwrc CRC:<0x52>
[ INFO] [1555502889.211413041]: Receiving: <STX>sRA ODpwrc \x00\x00\x02\x55<ETX>
[ INFO] [1555502889.413742132]: Sending  : <STX><STX><STX><STX><Len=0016>sRN LocationName CRC:<0x55>
[ INFO] [1555502889.415205992]: Receiving: <STX>sRA LocationName \x00\x0b\x6e\x6f\x74\x20\x64\x65\x66\x69\x6e\x65\x64<ETX>
[ INFO] [1555502889.417205292]: Sending  : <STX><STX><STX><STX><Len=0018>sRN LMPoutputRange CRC:<0x5e>
[ INFO] [1555502889.418631134]: Receiving: <STX>sRA LMPoutputRange \x00\x01\x00\x00\x05\x15\x00\x04\xa3\x80\x00\x16\xe3\x60<ETX>
[ INFO] [1555502889.418830949]: Angle resolution of scanner is 0.13010 [deg]  (in 1/10000th deg: 0x515)
[ INFO] [1555502889.418907556]: MIN_ANG:   -1.040 [rad]  -59.600 [deg]
[ INFO] [1555502889.418975818]: MAX_ANG:    1.047 [rad]   60.000 [deg]
[ INFO] [1555502889.419156102]: Sending  : <STX><STX><STX><STX><Len=0033>sWN LMPoutputRange 0x00 0x01 0x00 0x00 0x05 0x15 0x00 0x04 0xa3 0x80 0x00 0x16 0xe3 0x60 CRC:<0xd8>
[ INFO] [1555502889.420488646]: Receiving: <STX>sWA LMPoutputRange <ETX>
[ INFO] [1555502889.420719836]: Sending  : <STX><STX><STX><STX><Len=0018>sRN LMPoutputRange CRC:<0x5e>
[ INFO] [1555502889.421994443]: Receiving: <STX>sRA LMPoutputRange \x00\x01\x00\x00\x05\x15\x00\x04\xa3\x80\x00\x16\xe3\x60<ETX>
[ INFO] [1555502889.422165198]: Angle resolution of scanner is 0.13010 [deg]  (in 1/10000th deg: 0x515)
[ INFO] [1555502889.424815945]: MIN_ANG (after command verification):   -1.040 [rad]  -59.600 [deg]
[ INFO] [1555502889.424901901]: MAX_ANG (after command verification):    1.047 [rad]   60.000 [deg]
[ INFO] [1555502889.425102725]: Sending  : <STX><STX><STX><STX><Len=0032>sWN LMDscandatacfg 0x1f 0x00 0x01 0x01 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x01 CRC:<0x5c>
[ INFO] [1555502889.426373088]: Receiving: <STX>sWA LMDscandatacfg <ETX>
[ INFO] [1555502889.426606493]: Sending  : <STX><STX><STX><STX><Len=0018>sRN LMDscandatacfg CRC:<0x67>
[ INFO] [1555502889.427933309]: Receiving: <STX>sRA LMDscandatacfg \x1f\x00\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x01<ETX>
[ INFO] [1555502889.430654546]: Sending  : <STX><STX><STX><STX><Len=0018>sWN FREchoFilter 0x00 CRC:<0x7f>
[ INFO] [1555502889.431952374]: Receiving: <STX>sWA FREchoFilter <ETX>
[ INFO] [1555502889.432180430]: Sending  : <STX><STX><STX><STX><Len=0016>sMN LMCstartmeas CRC:<0x68>
[ INFO] [1555502889.963840302]: Receiving: <STX>sAN LMCstartmeas \x00<ETX>
[ INFO] [1555502889.964083670]: Sending  : <STX><STX><STX><STX><Len=0007>sMN Run CRC:<0x19>
[ INFO] [1555502889.965558914]: Receiving: <STX>sAN Run \x01<ETX>
[ INFO] [1555502889.965813465]: Sending  : <STX><STX><STX><STX><Len=0017>sEN LMDscandata 0x01 CRC:<0x33>
[ INFO] [1555502889.967297195]: Receiving: <STX>sEA LMDscandata \x01<ETX>
```
