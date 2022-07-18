#
# Data conversion utilities for sick_scan_api
#
import ctypes
import math
import numpy as np
import os
import sys
import sick_scan_api
from sick_scan_api import *

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField

# Convert a cartesian SickScanPointCloudMsg to ros sensor_msgs.msg.PointCloud2
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
    assert(api_pointcloud.data.size == cloud_data_buffer_len)
    cloud_data_buffer = bytearray(cloud_data_buffer_len)
    for n in range(cloud_data_buffer_len):
        cloud_data_buffer[n] = api_pointcloud.data.buffer[n]
    cloud_data = np.frombuffer(cloud_data_buffer, dtype = np.uint8, count = cloud_data_buffer_len)
    
    ros_pointcloud.data = cloud_data.tostring()
    return ros_pointcloud

# Convert a polar SickScanPointCloudMsg to ros sensor_msgs.msg.PointCloud2
def SickScanApiConvertPolarPointCloudToROS1(api_pointcloud):
    
    # Copy pointcloud header
    ros_pointcloud = PointCloud2()
    ros_pointcloud.header.seq = api_pointcloud.header.seq
    ros_pointcloud.header.stamp.secs = api_pointcloud.header.timestamp_sec
    ros_pointcloud.header.stamp.nsecs = api_pointcloud.header.timestamp_nsec
    ros_pointcloud.header.frame_id = ctypesCharArrayToString(api_pointcloud.header.frame_id)
    ros_pointcloud.width = api_pointcloud.width
    ros_pointcloud.height = api_pointcloud.height
    ros_pointcloud.is_bigendian = api_pointcloud.is_bigendian
    ros_pointcloud.is_dense = api_pointcloud.is_dense
    
    # Create pointcloud fields
    num_fields = api_pointcloud.fields.size
    msg_fields_buffer = api_pointcloud.fields.buffer
    field_offset_range = -1
    field_offset_azimuth = -1
    field_offset_elevation = -1
    field_offset_intensity = -1
    for n in range(num_fields):
        point_field = PointField(ctypesCharArrayToString(msg_fields_buffer[n].name), msg_fields_buffer[n].offset, msg_fields_buffer[n].datatype, msg_fields_buffer[n].count)
        if point_field.name == "range" and point_field.datatype == PointField.FLOAT32:
            field_offset_range = msg_fields_buffer[n].offset
        elif point_field.name == "azimuth" and point_field.datatype == PointField.FLOAT32:
            field_offset_azimuth = msg_fields_buffer[n].offset
        elif point_field.name == "elevation" and point_field.datatype == PointField.FLOAT32:
            field_offset_elevation = msg_fields_buffer[n].offset
        elif point_field.name == "intensity" and point_field.datatype == PointField.FLOAT32:
            field_offset_intensity = msg_fields_buffer[n].offset
    ros_pointcloud.fields =  [ PointField("x", 0, PointField.FLOAT32, 1), PointField("y", 4, PointField.FLOAT32, 1), PointField("z", 8, PointField.FLOAT32, 1), PointField("intensity", 12, PointField.FLOAT32, 1) ]
    ros_pointcloud.point_step = 16
    ros_pointcloud.row_step = ros_pointcloud.point_step * ros_pointcloud.width
    
    # Copy pointcloud data
    polar_cloud_data_buffer_len = (api_pointcloud.row_step * api_pointcloud.height) # length of polar cloud data in byte
    assert(api_pointcloud.data.size == polar_cloud_data_buffer_len and field_offset_range >= 0 and field_offset_azimuth >= 0 and field_offset_elevation >= 0)
    polar_cloud_data_buffer = bytearray(polar_cloud_data_buffer_len)
    for n in range(polar_cloud_data_buffer_len):
        polar_cloud_data_buffer[n] = api_pointcloud.data.buffer[n]
    cartesian_point_cloud_buffer = np.zeros(4 * ros_pointcloud.width * ros_pointcloud.height, dtype = np.float32)
    cartesian_point_cloud_offset = 0
    for row_idx in range(api_pointcloud.height):
        for col_idx in range(api_pointcloud.width):
            # Get lidar point in polar coordinates (range, azimuth and elevation)
            polar_point_offset = row_idx * api_pointcloud.row_step + col_idx * api_pointcloud.point_step
            point_range = np.frombuffer(polar_cloud_data_buffer, dtype = np.float32, count = 1, offset = polar_point_offset + field_offset_range)[0]
            point_azimuth = np.frombuffer(polar_cloud_data_buffer, dtype = np.float32, count = 1, offset = polar_point_offset + field_offset_azimuth)[0]
            point_elevation = np.frombuffer(polar_cloud_data_buffer, dtype = np.float32, count = 1, offset = polar_point_offset + field_offset_elevation)[0]
            point_intensity = 0
            if field_offset_intensity >= 0:
                point_intensity = np.frombuffer(polar_cloud_data_buffer, dtype = np.float32, count = 1, offset = polar_point_offset + field_offset_intensity)[0]
            # Convert from polar to cartesian coordinates
            point_x = point_range * math.cos(point_elevation) * math.cos(point_azimuth)
            point_y = point_range * math.cos(point_elevation) * math.sin(point_azimuth)
            point_z = point_range * math.sin(point_elevation)
            # print("point {},{}: offset={}, range={}, azimuth={}, elevation={}, intensity={}, x={}, y={}, z={}".format(col_idx, row_idx, polar_point_offset, 
            #     point_range, point_azimuth * 180 / math.pi, point_elevation * 180 / math.pi, point_intensity, point_x, point_y, point_z))
            cartesian_point_cloud_buffer[cartesian_point_cloud_offset + 0] = point_x
            cartesian_point_cloud_buffer[cartesian_point_cloud_offset + 1] = point_y
            cartesian_point_cloud_buffer[cartesian_point_cloud_offset + 2] = point_z
            cartesian_point_cloud_buffer[cartesian_point_cloud_offset + 3] = point_intensity
            cartesian_point_cloud_offset = cartesian_point_cloud_offset + 4

    ros_pointcloud.data = cartesian_point_cloud_buffer.tostring()
    return ros_pointcloud

