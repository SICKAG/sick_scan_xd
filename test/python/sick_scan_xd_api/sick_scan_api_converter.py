#
# Data conversion utilities for sick_scan_api
#
import ctypes
import numpy as np
import os
import sys
import sick_scan_api
from sick_scan_api import *

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField

def SickScanApiConvertPointCloudToROS1(api_pointcloud):
    
    # Copy pointcloud header and dimensions
    ros_pointcloud = PointCloud2()
    ros_pointcloud.header.seq = api_pointcloud.header.seq
    ros_pointcloud.header.stamp.secs = api_pointcloud.header.timestamp_sec
    ros_pointcloud.header.stamp.nsecs = api_pointcloud.header.timestamp_nsec
    ros_pointcloud.header.frame_id = ctypesCharArrayToString(api_pointcloud.header.frame_id)
    ros_pointcloud.width = api_pointcloud.width
    ros_pointcloud.height = api_pointcloud.height
    ros_pointcloud.is_bigendian = api_pointcloud.is_bigendian
    ros_pointcloud.is_dense = api_pointcloud.is_dense
    ros_pointcloud.point_step = api_pointcloud.point_step
    ros_pointcloud.row_step = api_pointcloud.row_step
    
    # Copy pointcloud fields
    num_fields = api_pointcloud.fields.size
    msg_fields_buffer = api_pointcloud.fields.buffer
    ros_pointcloud.fields =  [PointField()] * num_fields
    for n in range(num_fields):
        ros_pointcloud.fields[n] = PointField(ctypesCharArrayToString(msg_fields_buffer[n].name), msg_fields_buffer[n].offset, msg_fields_buffer[n].datatype, msg_fields_buffer[n].count)
    
    # Copy pointcloud data
    cloud_data_buffer_len = (ros_pointcloud.row_step * ros_pointcloud.height) # length of cloud data in byte
    cloud_data_buffer = bytearray(cloud_data_buffer_len)
    assert(api_pointcloud.data.size == cloud_data_buffer_len)
    for n in range(cloud_data_buffer_len):
        cloud_data_buffer[n] = api_pointcloud.data.buffer[n]
    cloud_data = np.frombuffer(cloud_data_buffer, dtype = np.float32, count = num_fields * ros_pointcloud.height * ros_pointcloud.width)
    ros_pointcloud.data = cloud_data.tostring()

    return ros_pointcloud

