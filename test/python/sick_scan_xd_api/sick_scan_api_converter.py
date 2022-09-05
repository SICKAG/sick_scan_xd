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

# set __ROS_VERSION to 0 (no ROS), 1 (publish ROS-1 pointclouds), or 2 (ROS-2)
__ROS_VERSION = os.getenv("ROS_VERSION")
if __ROS_VERSION is None:
    __ROS_VERSION = 0
else:
    __ROS_VERSION = int(__ROS_VERSION)
if __ROS_VERSION == 1:
    import rospy
    from rospy import Duration
    from sensor_msgs import point_cloud2
elif __ROS_VERSION == 2:
    from rclpy.duration import Duration
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray

# Convert a cartesian SickScanPointCloudMsg to ros sensor_msgs.msg.PointCloud2
def SickScanApiConvertPointCloudToROS(api_pointcloud):
    
    if __ROS_VERSION == 0:
        print("## ERROR: SickScanApiConvertPointCloudToROS not implemented (ROS_VERSION = {})".format(__ROS_VERSION))
        return None
    # Copy pointcloud header and dimensions
    ros_pointcloud = PointCloud2()
    if __ROS_VERSION == 1:
        ros_pointcloud.header.seq = api_pointcloud.header.seq
        ros_pointcloud.header.stamp.secs = api_pointcloud.header.timestamp_sec
        ros_pointcloud.header.stamp.nsecs = api_pointcloud.header.timestamp_nsec
    elif __ROS_VERSION == 2:
        ros_pointcloud.header.stamp.sec = api_pointcloud.header.timestamp_sec
        ros_pointcloud.header.stamp.nanosec = api_pointcloud.header.timestamp_nsec
    ros_pointcloud.header.frame_id = ctypesCharArrayToString(api_pointcloud.header.frame_id)
    ros_pointcloud.width = api_pointcloud.width
    ros_pointcloud.height = api_pointcloud.height
    ros_pointcloud.is_bigendian = (api_pointcloud.is_bigendian > 0)
    ros_pointcloud.is_dense = (api_pointcloud.is_dense > 0)
    ros_pointcloud.point_step = api_pointcloud.point_step
    ros_pointcloud.row_step = api_pointcloud.row_step
    
    # Copy pointcloud fields
    num_fields = api_pointcloud.fields.size
    msg_fields_buffer = api_pointcloud.fields.buffer
    ros_pointcloud.fields =  [PointField()] * num_fields
    for n in range(num_fields):
        ros_pointcloud.fields[n] = PointField(name = ctypesCharArrayToString(msg_fields_buffer[n].name), offset = msg_fields_buffer[n].offset, datatype = msg_fields_buffer[n].datatype, count = msg_fields_buffer[n].count)
    
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
def SickScanApiConvertPolarPointCloudToROS(api_pointcloud):
    
    if __ROS_VERSION != 1:
        print("## ERROR: SickScanApiConvertPointCloudToROS not implemented (ROS_VERSION = {})".format(__ROS_VERSION))
        return None
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

# Convert radar objects to ros sensor_msgs.msg.PointCloud2
def SickScanApiConvertRadarObjectsToROS(header, radar_objects):

    if __ROS_VERSION != 1:
        print("## ERROR: SickScanApiConvertPointCloudToROS not implemented (ROS_VERSION = {})".format(__ROS_VERSION))
        return None
    # Copy pointcloud header
    ros_pointcloud = PointCloud2()
    ros_pointcloud.header.seq = header.seq
    ros_pointcloud.header.stamp.secs = header.timestamp_sec
    ros_pointcloud.header.stamp.nsecs = header.timestamp_nsec
    ros_pointcloud.header.frame_id = ctypesCharArrayToString(header.frame_id)
    ros_pointcloud.width = radar_objects.size
    ros_pointcloud.height = 1
    ros_pointcloud.is_bigendian = False
    ros_pointcloud.is_dense = True

    # Set field description
    ros_pointcloud.fields =  [ 
        PointField("x", 0, PointField.FLOAT32, 1), PointField("y", 4, PointField.FLOAT32, 1), PointField("z", 8, PointField.FLOAT32, 1), 
        PointField("vx", 12, PointField.FLOAT32, 1), PointField("vy", 16, PointField.FLOAT32, 1), PointField("vz", 20, PointField.FLOAT32, 1) ]
    ros_pointcloud.point_step = 24
    ros_pointcloud.row_step = ros_pointcloud.point_step * ros_pointcloud.width
    
    # Copy radar object data
    ros_point_cloud_buffer = np.zeros(6 * ros_pointcloud.width * ros_pointcloud.height, dtype = np.float32)
    ros_point_cloud_offset = 0
    for n in range(radar_objects.size):
        ros_point_cloud_buffer[ros_point_cloud_offset + 0] = radar_objects.buffer[n].object_box_center_position.x
        ros_point_cloud_buffer[ros_point_cloud_offset + 1] = radar_objects.buffer[n].object_box_center_position.y
        ros_point_cloud_buffer[ros_point_cloud_offset + 2] = radar_objects.buffer[n].object_box_center_position.z
        ros_point_cloud_buffer[ros_point_cloud_offset + 3] = radar_objects.buffer[n].velocity_linear.x
        ros_point_cloud_buffer[ros_point_cloud_offset + 4] = radar_objects.buffer[n].velocity_linear.y
        ros_point_cloud_buffer[ros_point_cloud_offset + 5] = radar_objects.buffer[n].velocity_linear.z

    ros_pointcloud.data = ros_point_cloud_buffer.tostring()
    return ros_pointcloud

# Convert SickScanVisualizationMarkerBuffer to ros visualization_msgs.msg.MarkerArray
def SickScanApiConvertMarkerArrayToROS(sick_markers):

    if __ROS_VERSION == 0:
        print("## ERROR: SickScanApiConvertPointCloudToROS not implemented (ROS_VERSION = {})".format(__ROS_VERSION))
        return None
    ros_marker = MarkerArray()
    for n in range(sick_markers.size):
        sick_marker = sick_markers.buffer[n]
        marker = Marker()
        # Copy marker
        if __ROS_VERSION == 1:
            marker.header.seq = sick_marker.header.seq
            marker.header.stamp.secs = sick_marker.header.timestamp_sec
            marker.header.stamp.nsecs = sick_marker.header.timestamp_nsec
            marker.lifetime = rospy.Duration(sick_marker.lifetime_sec, sick_marker.lifetime_nsec)
        elif __ROS_VERSION == 2:
            marker.header.stamp.sec = sick_marker.header.timestamp_sec
            marker.header.stamp.nanosec = sick_marker.header.timestamp_nsec
            # marker.lifetime = Duration(seconds = sick_marker.lifetime_sec, nanoseconds = sick_marker.lifetime_nsec)
        marker.header.frame_id = ctypesCharArrayToString(sick_marker.header.frame_id)
        marker.ns= ctypesCharArrayToString(sick_marker.ns)
        marker.id = sick_marker.id
        marker.type = sick_marker.type
        marker.action = sick_marker.action
        marker.pose.position.x = sick_marker.pose_position.x
        marker.pose.position.y = sick_marker.pose_position.y
        marker.pose.position.z = sick_marker.pose_position.z
        marker.pose.orientation.x = sick_marker.pose_orientation.x
        marker.pose.orientation.y = sick_marker.pose_orientation.y
        marker.pose.orientation.z = sick_marker.pose_orientation.z
        marker.pose.orientation.w = sick_marker.pose_orientation.w
        marker.scale.x = sick_marker.scale.x
        marker.scale.y = sick_marker.scale.y
        marker.scale.z = sick_marker.scale.z
        marker.color.r = sick_marker.color.r
        marker.color.g = sick_marker.color.g
        marker.color.b = sick_marker.color.b
        marker.color.a = sick_marker.color.a
        marker.frame_locked = (sick_marker.frame_locked > 0)
        marker.text = ctypesCharArrayToString(sick_marker.text)
        marker.mesh_resource = ctypesCharArrayToString(sick_marker.mesh_resource)
        marker.mesh_use_embedded_materials = (sick_marker.mesh_use_embedded_materials > 0)
        for m in range(sick_marker.points.size):
            sick_point = sick_marker.points.buffer[m]
            ros_point = Point(x = sick_point.x, y = sick_point.y, z = sick_point.z)
            marker.points.append(ros_point)
        for m in range(sick_marker.colors.size):
            sick_color = sick_marker.colors.buffer[m]
            ros_color = ColorRGBA(r = sick_color.r, g = sick_color.g, b = sick_color.b, a = sick_color.a)
            marker.colors.append(ros_color)
        ros_marker.markers.append(marker)
    return ros_marker
