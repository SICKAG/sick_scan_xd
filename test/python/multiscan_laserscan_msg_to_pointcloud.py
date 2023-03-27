# Receive LaserScan messages and convert to pointcloud

import math
import numpy as np
import os
import sys
import threading
import time

# set __ROS_VERSION to 1 (ROS-1), or 2 (ROS-2)
__ROS_VERSION = os.getenv("ROS_VERSION")
if __ROS_VERSION is not None:
    __ROS_VERSION = int(__ROS_VERSION)
if __ROS_VERSION == 2:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from rclpy.qos import QoSReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from sensor_msgs.msg import PointCloud2, PointField
else:
    import rospy
    from sensor_msgs.msg import LaserScan
    from sensor_msgs.msg import PointCloud2, PointField
    class Node:
        pass

class LaserScanSubscriber(Node):

    def __init__(self, __ROS_VERSION, num_messages_decay):
        self.__ROS_VERSION = __ROS_VERSION
        if self.__ROS_VERSION == 2: # ROS-2
            super().__init__("multiscan_laserscan_msg_to_pointcloud")
        self.num_messages_decay = num_messages_decay
        self.scan_msg_array = []
        if self.__ROS_VERSION == 2: # ROS-2
            self.scan_subscription = self.create_subscription(LaserScan, "/sick_scansegment_xd/scan_segment", self.listener_callback, 10) # QoSProfile(depth=16*12*3,reliability=QoSReliabilityPolicy.RELIABLE))
            self.cloud_publisher = self.create_publisher(PointCloud2, "/laserscan_msg_cloud", 10)
        else: # ROS-1
            self.scan_subscription = rospy.Subscriber("/sick_scansegment_xd/scan_segment", LaserScan, self.listener_callback, queue_size=16*12*3) # multiScan: 16 layer, 12 segments, 3 echos
            self.cloud_publisher = rospy.Publisher("/laserscan_msg_cloud", PointCloud2, queue_size=10)
        self.frame_id_elevation_table = { # Elevation table for Multiscan 136 (16 layer):
            "world_0":  -22.71 * math.pi / 180, # layer:0  elevation: -22.71 deg
            "world_1":  -17.56 * math.pi / 180, # layer:1  elevation: -17.56 deg
            "world_2":  -12.48 * math.pi / 180, # layer:2  elevation: -12.48 deg
            "world_3":   -7.51 * math.pi / 180, # layer:3  elevation:  -7.51 deg
            "world_4":   -2.49 * math.pi / 180, # layer:4  elevation:  -2.49 deg
            "world_5":   -0.07 * math.pi / 180, # layer:5  elevation:  -0.07 deg
            "world_6":   +2.43 * math.pi / 180, # layer:6  elevation:  +2.43 deg 
            "world_7":   +7.29 * math.pi / 180, # layer:7  elevation:  +7.29 deg 
            "world_8":  +12.79 * math.pi / 180, # layer:8  elevation: +12.79 deg 
            "world_9":  +17.28 * math.pi / 180, # layer:9  elevation: +17.28 deg 
            "world_10": +21.94 * math.pi / 180, # layer:10 elevation: +21.94 deg 
            "world_11": +26.73 * math.pi / 180, # layer:11 elevation: +26.73 deg 
            "world_12": +31.86 * math.pi / 180, # layer:12 elevation: +31.86 deg 
            "world_13": +33.42 * math.pi / 180, # layer:13 elevation: +33.42 deg 
            "world_14": +37.18 * math.pi / 180, # layer:14 elevation: +37.18 deg 
            "world_15": +42.79 * math.pi / 180  # layer:15 elevation: +42.79 deg
        }

    def listener_callback(self, msg):
        # print("subscriber: {}".format(msg))
        # print("subscriber: {}".format(msg.header.frame_id))
        self.scan_msg_array.append(msg)
        if len(self.scan_msg_array) >= self.num_messages_decay:
            # Create pointcloud header and field description
            ros_pointcloud = PointCloud2()
            ros_pointcloud.header.seq = msg.header.seq
            ros_pointcloud.header.stamp = msg.header.stamp
            ros_pointcloud.header.frame_id = "world"
            ros_pointcloud.width = 0
            for scan_msg in self.scan_msg_array:
                ros_pointcloud.width = ros_pointcloud.width + len(scan_msg.ranges)
            ros_pointcloud.height = 1
            ros_pointcloud.is_bigendian = False
            ros_pointcloud.is_dense = True
            ros_pointcloud.fields =  [ PointField("x", 0, PointField.FLOAT32, 1), PointField("y", 4, PointField.FLOAT32, 1), PointField("z", 8, PointField.FLOAT32, 1), PointField("intensity", 12, PointField.FLOAT32, 1) ]
            ros_pointcloud.point_step = 16
            ros_pointcloud.row_step = ros_pointcloud.point_step * ros_pointcloud.width
            # Convert laserscan message to pointcloud
            cloud_data = np.zeros(4 * ros_pointcloud.width * ros_pointcloud.height, dtype=np.float32)
            cloud_point_idx = 0
            for scan_msg in self.scan_msg_array:
                azimuth = scan_msg.angle_min
                azimuth_inc = scan_msg.angle_increment
                elevation = self.frame_id_elevation_table[scan_msg.header.frame_id]
                elevation_cos = math.cos(elevation)
                elevation_sin = math.sin(elevation)
                for point_idx, range in enumerate(scan_msg.ranges):
                    cloud_data[4 * cloud_point_idx + 0] = range * elevation_cos * math.cos(azimuth) # x
                    cloud_data[4 * cloud_point_idx + 1] = range * elevation_cos * math.sin(azimuth) # y
                    cloud_data[4 * cloud_point_idx + 2] = range * elevation_sin                     # z
                    cloud_data[4 * cloud_point_idx + 3] = scan_msg.intensities[point_idx]           # intensity
                    cloud_point_idx = cloud_point_idx + 1
                    azimuth = azimuth + azimuth_inc
            ros_pointcloud.data = cloud_data.tostring()
            self.cloud_publisher.publish(ros_pointcloud)
            self.scan_msg_array = []

def main(args=None):
    global __ROS_VERSION
    num_messages_decay = 3 * 12 * 16 # decay over one complete fullscan, multiScan: 16 layer, 12 segments, 3 echos
    if __ROS_VERSION == 2: # ROS-2
        rclpy.init()
        subscriber = LaserScanSubscriber(__ROS_VERSION, num_messages_decay)
        rclpy.spin(subscriber)
        rclpy.shutdown()
        
    else: # ROS-1
        node = rospy.init_node("multiscan_laserscan_msg_to_pointcloud")
        subscriber = LaserScanSubscriber(__ROS_VERSION, num_messages_decay)
        # rospy.spin()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

if __name__ == '__main__':
    main()
