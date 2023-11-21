# Receive polar pointcloud messages (fields: "i", "range", "azimuth", "elevation")
# and convert to cartesian pointcloud messages  (fields: "x", "y", "z", "i")

import argparse
import math
import numpy as np

import rospy
from sensor_msgs.msg import PointCloud2, PointField
class Node:
    pass

# Convert a polar pointcloud to ros cartesian pointcloud
def convertPolarToCartesianPointCloud(polar_pointcloud):
    # Copy pointcloud header
    ros_pointcloud = PointCloud2()
    ros_pointcloud.header = polar_pointcloud.header
    ros_pointcloud.width = polar_pointcloud.width
    ros_pointcloud.height = polar_pointcloud.height
    ros_pointcloud.is_bigendian = polar_pointcloud.is_bigendian
    ros_pointcloud.is_dense = polar_pointcloud.is_dense
    # Create pointcloud fields
    field_offset_range = -1
    field_offset_azimuth = -1
    field_offset_elevation = -1
    field_offset_intensity = -1
    for polar_field in polar_pointcloud.fields:
        if polar_field.name == "range" and polar_field.datatype == PointField.FLOAT32:
            field_offset_range = polar_field.offset
        elif polar_field.name == "azimuth" and polar_field.datatype == PointField.FLOAT32:
            field_offset_azimuth = polar_field.offset
        elif polar_field.name == "elevation" and polar_field.datatype == PointField.FLOAT32:
            field_offset_elevation = polar_field.offset
        elif (polar_field.name == "intensity" or polar_field.name == "i") and polar_field.datatype == PointField.FLOAT32:
            field_offset_intensity = polar_field.offset
    ros_pointcloud.fields =  [ PointField("x", 0, PointField.FLOAT32, 1), PointField("y", 4, PointField.FLOAT32, 1), PointField("z", 8, PointField.FLOAT32, 1), PointField("intensity", 12, PointField.FLOAT32, 1) ]
    ros_pointcloud.point_step = 16
    ros_pointcloud.row_step = ros_pointcloud.point_step * ros_pointcloud.width
    # Convert pointcloud data
    cartesian_point_cloud_buffer = np.zeros(4 * ros_pointcloud.width * ros_pointcloud.height, dtype = np.float32)
    cartesian_point_cloud_offset = 0
    for row_idx in range(polar_pointcloud.height):
        for col_idx in range(polar_pointcloud.width):
            # Get scan point in polar coordinates (range, azimuth and elevation)
            polar_point_offset = row_idx * polar_pointcloud.row_step + col_idx * polar_pointcloud.point_step
            point_range = np.frombuffer(polar_pointcloud.data, dtype = np.float32, count = 1, offset = polar_point_offset + field_offset_range)[0]
            point_azimuth = np.frombuffer(polar_pointcloud.data, dtype = np.float32, count = 1, offset = polar_point_offset + field_offset_azimuth)[0]
            point_elevation = np.frombuffer(polar_pointcloud.data, dtype = np.float32, count = 1, offset = polar_point_offset + field_offset_elevation)[0]
            point_intensity = 0
            if field_offset_intensity >= 0:
                point_intensity = np.frombuffer(polar_pointcloud.data, dtype = np.float32, count = 1, offset = polar_point_offset + field_offset_intensity)[0]
            # Convert from polar to cartesian coordinates
            point_x = point_range * math.cos(point_elevation) * math.cos(point_azimuth)
            point_y = point_range * math.cos(point_elevation) * math.sin(point_azimuth)
            point_z = point_range * math.sin(point_elevation)
            cartesian_point_cloud_buffer[cartesian_point_cloud_offset + 0] = point_x
            cartesian_point_cloud_buffer[cartesian_point_cloud_offset + 1] = point_y
            cartesian_point_cloud_buffer[cartesian_point_cloud_offset + 2] = point_z
            cartesian_point_cloud_buffer[cartesian_point_cloud_offset + 3] = point_intensity
            cartesian_point_cloud_offset = cartesian_point_cloud_offset + 4
    ros_pointcloud.data = cartesian_point_cloud_buffer.tostring()
    return ros_pointcloud

class PointCloudSubscriber(Node):
    def __init__(self, subscriber_topic, publisher_topic):
        self.cloud_subscriber = rospy.Subscriber(subscriber_topic, PointCloud2, self.listener_callback, queue_size=16*12*3) # multiScan: 16 layer, 12 segments, 3 echos
        self.cloud_publisher = rospy.Publisher(publisher_topic, PointCloud2, queue_size=16*12*3)
    def listener_callback(self, msg):
        ros_pointcloud = convertPolarToCartesianPointCloud(msg)
        self.cloud_publisher.publish(ros_pointcloud)

if __name__ == '__main__':
    subscriber_topic = "/cloud_polar_unstructured_fullframe"
    publisher_topic = "/cloud_polar_to_cartesian"
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--polar_topic", help="ros topic of polar pointcloud (subscriber)", default=subscriber_topic, type=str)
    arg_parser.add_argument("--cartesian_topic", help="ros topic of cartesian pointcloud (publisher)", default=publisher_topic, type=str)
    cli_args = arg_parser.parse_args()
    subscriber_topic = cli_args.polar_topic
    publisher_topic = cli_args.cartesian_topic
    node = rospy.init_node("polar_to_cartesian_pointcloud")
    subscriber = PointCloudSubscriber(subscriber_topic, publisher_topic)
    # rospy.spin()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
