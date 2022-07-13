#!/bin/bash

source /opt/ros/noetic/setup.bash
# static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms
# tf_echo <source_frame> <target_frame>
# x = 5 deg: 0.0872665, y = -10 deg: -0.1745329, z = 15 deg: 0.2617994
rosrun tf static_transform_publisher 0 0 0 0.2617994 -0.1745329 0.0872665 parent_frame child_frame 100
rosrun tf tf_echo parent_frame child_frame

