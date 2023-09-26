# SOPAS support for sick_scan_segment_xd

On ROS-1 and ROS-2, sick_scan_segment_xd provides ros service `ColaMsg` to send CoLa commands to the lidar. Using this service, filters can be applied to multiScan136 and picoScan150 lidars during runtime. 

Examples on ROS-1:
```
rosservice list
rosservice call /sick_scansegment_xd/ColaMsg "{request: 'sMN IsSystemReady'}"                             # response: "sAN IsSystemReady 1"
rosservice call /sick_scansegment_xd/ColaMsg "{request: 'sMN SetAccessMode 3 F4724744'}"                  # response: "sAN SetAccessMode 1"
rosservice call /sick_scansegment_xd/ColaMsg "{request: 'sWN ScanDataEthSettings 1 +127 +0 +0 +1 +2115'}" # response: "sWA ScanDataEthSettings"
rosservice call /sick_scansegment_xd/ColaMsg "{request: 'sWN ScanDataFormat 1'}"                          # response: "sWA ScanDataFormat"
rosservice call /sick_scansegment_xd/ColaMsg "{request: 'sWN ScanDataPreformatting 1'}"                   # response: "sWA ScanDataPreformatting"
rosservice call /sick_scansegment_xd/ColaMsg "{request: 'sWN ScanDataEnable 1'}"                          # response: "sWA ScanDataEnable"
rosservice call /sick_scansegment_xd/ColaMsg "{request: 'sMN LMCstartmeas'}"                              # response: "sAN LMCstartmeas"
rosservice call /sick_scansegment_xd/ColaMsg "{request: 'sMN Run'}"                                       # response: "sAN Run 1"
rosservice call /sick_scansegment_xd/ColaMsg "{request: 'sRN FREchoFilter'}"                                                # response: "sRA FREchoFilter 1"
rosservice call /sick_scansegment_xd/ColaMsg "{request: 'sRN LFPangleRangeFilter'}"                                         # response: "sRA LFPangleRangeFilter 0 C0490FF9 40490FF9 BFC90FF9 3FC90FF9 1"
rosservice call /sick_scansegment_xd/ColaMsg "{request: 'sRN LFPlayerFilter'}"                                              # response: "sRA LFPlayerFilter 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"
rosservice call /sick_scansegment_xd/ColaMsg "{request: 'sMN SetAccessMode 3 F4724744'}"                                    # response: "sAN SetAccessMode 1"
rosservice call /sick_scansegment_xd/ColaMsg "{request: 'sWN FREchoFilter 1'}"                                              # response: "sWA FREchoFilter"
rosservice call /sick_scansegment_xd/ColaMsg "{request: 'sWN LFPangleRangeFilter 0 C0490FF9 40490FF9 BFC90FF9 3FC90FF9 1'}" # response: "sWA LFPangleRangeFilter"
rosservice call /sick_scansegment_xd/ColaMsg "{request: 'sWN LFPlayerFilter 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1'}"            # response: "sWA LFPlayerFilter"
rosservice call /sick_scansegment_xd/ColaMsg "{request: 'sMN Run'}"                                                         # response: "sAN Run 1"
rosservice call /sick_scansegment_xd/ColaMsg "{request: 'sRN FREchoFilter'}"                                                # response: "sRA FREchoFilter 1"
rosservice call /sick_scansegment_xd/ColaMsg "{request: 'sRN LFPangleRangeFilter'}"                                         # response: "sRA LFPangleRangeFilter 0 C0490FF9 40490FF9 BFC90FF9 3FC90FF9 1"
rosservice call /sick_scansegment_xd/ColaMsg "{request: 'sRN LFPlayerFilter'}"                                              # response: "sRA LFPlayerFilter 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"
```

Note: LFPlayerFilter and LFPangleRangeFilter are not supported by picoscan150.

Examples on ROS-2:
```
ros2 service list
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sMN IsSystemReady'}"                             # response: "sAN IsSystemReady 1"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sMN SetAccessMode 3 F4724744'}"                  # response: "sAN SetAccessMode 1"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sWN ScanDataEthSettings 1 +127 +0 +0 +1 +2115'}" # response: "sWA ScanDataEthSettings"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sWN ScanDataFormat 1'}"                          # response: "sWA ScanDataFormat"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sWN ScanDataPreformatting 1'}"                   # response: "sWA ScanDataPreformatting"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sWN ScanDataEnable 1'}"                          # response: "sWA ScanDataEnable"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sMN LMCstartmeas'}"                              # response: "sAN LMCstartmeas"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sMN Run'}"                                       # response: "sAN Run 1"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sRN FREchoFilter'}"                                                # response: "sRA FREchoFilter 1"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sRN LFPangleRangeFilter'}"                                         # response: "sRA LFPangleRangeFilter 0 C0490FF9 40490FF9 BFC90FF9 3FC90FF9 1"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sRN LFPlayerFilter'}"                                              # response: "sRA LFPlayerFilter 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sMN SetAccessMode 3 F4724744'}"                                    # response: "sAN SetAccessMode 1"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sWN FREchoFilter 1'}"                                              # response: "sWA FREchoFilter"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sWN LFPangleRangeFilter 0 C0490FF9 40490FF9 BFC90FF9 3FC90FF9 1'}" # response: "sWA LFPangleRangeFilter"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sWN LFPlayerFilter 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1'}"            # response: "sWA LFPlayerFilter"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sMN Run'}"                                                         # response: "sAN Run 1"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sRN FREchoFilter'}"                                                # response: "sRA FREchoFilter 1"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sRN LFPangleRangeFilter'}"                                         # response: "sRA LFPangleRangeFilter 0 C0490FF9 40490FF9 BFC90FF9 3FC90FF9 1"
ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: 'sRN LFPlayerFilter'}"                                              # response: "sRA LFPlayerFilter 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"
```

Note: LFPlayerFilter and LFPangleRangeFilter are not supported by picoscan150.
