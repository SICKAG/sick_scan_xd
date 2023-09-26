/*
* Copyright (C) 2022, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2022, SICK AG, Waldkirch
* All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*       http://www.apache.org/licenses/LICENSE-2.0
*
*   Unless required by applicable law or agreed to in writing, software
*   distributed under the License is distributed on an "AS IS" BASIS,
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*   See the License for the specific language governing permissions and
*   limitations under the License.
*
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Osnabrueck University nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*     * Neither the name of SICK AG nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission
*     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*      Authors:
*         Michael Lehning <michael.lehning@lehning.de>
*
*/
#include <sick_scan_api_converter.h>

#if __ROS_VERSION == 1

/* Convert a cartesian SickScanPointCloudMsg to sensor_msgs::PointCloud2 (ROS-1 only) */
sensor_msgs::PointCloud2 SickScanApiConverter::convertPointCloudMsg(const SickScanPointCloudMsg & msg)
{
    sensor_msgs::PointCloud2 pointcloud;
    // Copy header and pointcloud dimension
    pointcloud.header.seq = msg.header.seq;
    pointcloud.header.stamp.sec = msg.header.timestamp_sec;
    pointcloud.header.stamp.nsec = msg.header.timestamp_nsec;
    pointcloud.header.frame_id = msg.header.frame_id;
    pointcloud.width = msg.width;
    pointcloud.height = msg.height;
    pointcloud.is_bigendian = msg.is_bigendian;
    pointcloud.is_dense = msg.is_dense;
    pointcloud.point_step = msg.point_step;
    pointcloud.row_step = msg.row_step;
    // Copy field descriptions
    int num_fields = msg.fields.size;
    SickScanPointFieldMsg* msg_fields_buffer = (SickScanPointFieldMsg*)msg.fields.buffer;
    pointcloud.fields.resize(num_fields);
    for(int n = 0; n < num_fields; n++)
    {
        pointcloud.fields[n].name = msg_fields_buffer[n].name;
        pointcloud.fields[n].offset = msg_fields_buffer[n].offset;
        pointcloud.fields[n].count = msg_fields_buffer[n].count;
        pointcloud.fields[n].datatype = msg_fields_buffer[n].datatype;
    }
    // Copy pointcloud data
    pointcloud.data.resize(msg.row_step * msg.height, 0);
    memcpy(&pointcloud.data[0], msg.data.buffer, msg.row_step * msg.height);
    // Return converted pointcloud
    return pointcloud;
}

/* Convert a polar SickScanPointCloudMsg to sensor_msgs::PointCloud2 (ROS-1 only) */
sensor_msgs::PointCloud2 SickScanApiConverter::convertPolarPointCloudMsg(const SickScanPointCloudMsg & msg)
{
    sensor_msgs::PointCloud2 pointcloud;
    // Copy header and pointcloud dimension
    pointcloud.header.seq = msg.header.seq;
    pointcloud.header.stamp.sec = msg.header.timestamp_sec;
    pointcloud.header.stamp.nsec = msg.header.timestamp_nsec;
    pointcloud.header.frame_id = msg.header.frame_id;
    pointcloud.width = msg.width;
    pointcloud.height = msg.height;
    pointcloud.is_bigendian = msg.is_bigendian;
    pointcloud.is_dense = msg.is_dense;
    // Create field descriptions
    int num_fields = msg.fields.size;
    SickScanPointFieldMsg* msg_fields_buffer = (SickScanPointFieldMsg*)msg.fields.buffer;
    int field_offset_range = -1, field_offset_azimuth = -1, field_offset_elevation = -1, field_offset_intensity = -1;
    for(int n = 0; n < num_fields; n++)
    {
        if (strcmp(msg_fields_buffer[n].name, "range") == 0 && msg_fields_buffer[n].datatype == SICK_SCAN_POINTFIELD_DATATYPE_FLOAT32)
            field_offset_range = msg_fields_buffer[n].offset;
        else if (strcmp(msg_fields_buffer[n].name, "azimuth") == 0 && msg_fields_buffer[n].datatype == SICK_SCAN_POINTFIELD_DATATYPE_FLOAT32)
            field_offset_azimuth = msg_fields_buffer[n].offset;
        else if (strcmp(msg_fields_buffer[n].name, "elevation") == 0 && msg_fields_buffer[n].datatype == SICK_SCAN_POINTFIELD_DATATYPE_FLOAT32)
            field_offset_elevation = msg_fields_buffer[n].offset;
        else if (strcmp(msg_fields_buffer[n].name, "intensity") == 0 && msg_fields_buffer[n].datatype == SICK_SCAN_POINTFIELD_DATATYPE_FLOAT32)
            field_offset_intensity = msg_fields_buffer[n].offset;
    }
    pointcloud.fields.resize(4);
    for (int i = 0; i < pointcloud.fields.size(); i++)
    {
        std::string channelId[] = {"x", "y", "z", "intensity"};
        pointcloud.fields[i].name = channelId[i];
        pointcloud.fields[i].offset = i * sizeof(float);
        pointcloud.fields[i].count = 1;
        pointcloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }
    pointcloud.point_step = pointcloud.fields.size() * sizeof(float); // i.e. point_step := 16 byte
    pointcloud.row_step = pointcloud.point_step * pointcloud.width;
    // Convert pointcloud data
    pointcloud.data.resize(pointcloud.row_step * pointcloud.height, 0);
    int cartesian_point_cloud_offset = 0;
    float* cartesian_point_cloud_buffer = (float*)pointcloud.data.data();
    for (int row_idx = 0; row_idx < msg.height; row_idx++)
    {
        for (int col_idx = 0; col_idx < msg.width; col_idx++, cartesian_point_cloud_offset+=4)
        {
            // Get lidar point in polar coordinates (range, azimuth and elevation)
            int polar_point_offset = row_idx * msg.row_step + col_idx * msg.point_step;
            float point_range = ((float*)(msg.data.buffer + polar_point_offset + field_offset_range))[0];
            float point_azimuth = ((float*)(msg.data.buffer + polar_point_offset + field_offset_azimuth))[0];
            float point_elevation = ((float*)(msg.data.buffer + polar_point_offset + field_offset_elevation))[0];
            float point_intensity = 0;
            if (field_offset_intensity >= 0)
                point_intensity = ((float*)(msg.data.buffer + polar_point_offset + field_offset_intensity))[0];
            // Convert from polar to cartesian coordinates
            float point_x = point_range * cosf(point_elevation) * cosf(point_azimuth);
            float point_y = point_range * cosf(point_elevation) * sinf(point_azimuth);
            float point_z = point_range * sinf(point_elevation);
            // printf("point %d,%d: offset=%d, range=%f, azimuth=%f, elevation=%f, intensity=%f, x=%f, y=%f, z=%f\n", col_idx, row_idx, polar_point_offset, 
            //     point_range, point_azimuth * 180 / M_PI, point_elevation * 180 / M_PI, point_intensity, point_x, point_y, point_z);
            cartesian_point_cloud_buffer[cartesian_point_cloud_offset + 0] = point_x;
            cartesian_point_cloud_buffer[cartesian_point_cloud_offset + 1] = point_y;
            cartesian_point_cloud_buffer[cartesian_point_cloud_offset + 2] = point_z;
            cartesian_point_cloud_buffer[cartesian_point_cloud_offset + 3] = point_intensity;         
        }
    }
    return pointcloud;
}

/* Convert a SickScanImuMsg to sensor_msgs::Imu (ROS-1 only) */
sensor_msgs::Imu SickScanApiConverter::convertImuMsg(const SickScanImuMsg & src_msg)
{
    sensor_msgs::Imu dst_msg;
    // Copy header
    dst_msg.header.seq = src_msg.header.seq;
    dst_msg.header.stamp.sec = src_msg.header.timestamp_sec;
    dst_msg.header.stamp.nsec = src_msg.header.timestamp_nsec;
    dst_msg.header.frame_id = src_msg.header.frame_id;
    // Copy imu data
    dst_msg.orientation.x = src_msg.orientation.x;
    dst_msg.orientation.y = src_msg.orientation.y;
    dst_msg.orientation.z = src_msg.orientation.z;
    dst_msg.orientation.w = src_msg.orientation.w;
    for(int n = 0; n < 9; n++)
        dst_msg.orientation_covariance[n] = src_msg.orientation_covariance[n];
    dst_msg.angular_velocity.x = src_msg.angular_velocity.x;
    dst_msg.angular_velocity.y = src_msg.angular_velocity.y;
    dst_msg.angular_velocity.z = src_msg.angular_velocity.z;
    for(int n = 0; n < 9; n++)
        dst_msg.angular_velocity_covariance[n] = src_msg.angular_velocity_covariance[n];
    dst_msg.linear_acceleration.x = src_msg.linear_acceleration.x;
    dst_msg.linear_acceleration.y = src_msg.linear_acceleration.y;
    dst_msg.linear_acceleration.z = src_msg.linear_acceleration.z;
    for(int n = 0; n < 9; n++)
        dst_msg.linear_acceleration_covariance[n] = src_msg.linear_acceleration_covariance[n];
    return dst_msg;
}

/* Convert a SickScanLFErecMsg to sick_scan_msg::LFErecMsg (ROS-1 only) */
sick_scan_xd::LFErecMsg SickScanApiConverter::convertLFErecMsg(const SickScanLFErecMsg& src_msg)
{
    sick_scan_xd::LFErecMsg dst_msg;
    // Copy header
    dst_msg.header.seq = src_msg.header.seq;
    dst_msg.header.stamp.sec = src_msg.header.timestamp_sec;
    dst_msg.header.stamp.nsec = src_msg.header.timestamp_nsec;
    dst_msg.header.frame_id = src_msg.header.frame_id;
    // Copy LFErec data
    dst_msg.fields_number = src_msg.fields_number;
    dst_msg.fields.resize(dst_msg.fields_number);
    for(int n = 0; n < dst_msg.fields_number; n++)
    {
        dst_msg.fields[n].version_number = src_msg.fields[n].version_number;
        dst_msg.fields[n].field_index = src_msg.fields[n].field_index;
        dst_msg.fields[n].sys_count = src_msg.fields[n].sys_count;
        dst_msg.fields[n].dist_scale_factor = src_msg.fields[n].dist_scale_factor;
        dst_msg.fields[n].dist_scale_offset = src_msg.fields[n].dist_scale_offset;
        dst_msg.fields[n].angle_scale_factor = src_msg.fields[n].angle_scale_factor;
        dst_msg.fields[n].angle_scale_offset = src_msg.fields[n].angle_scale_offset;
        dst_msg.fields[n].field_result_mrs = src_msg.fields[n].field_result_mrs;
        dst_msg.fields[n].time_state = src_msg.fields[n].time_state;
        dst_msg.fields[n].year = src_msg.fields[n].year;
        dst_msg.fields[n].month = src_msg.fields[n].month;
        dst_msg.fields[n].day = src_msg.fields[n].day;
        dst_msg.fields[n].hour = src_msg.fields[n].hour;
        dst_msg.fields[n].minute = src_msg.fields[n].minute;
        dst_msg.fields[n].second = src_msg.fields[n].second;
        dst_msg.fields[n].microsecond = src_msg.fields[n].microsecond;
    }
    return dst_msg;
}

/* Convert a SickScanLIDoutputstateMsg to sick_scan_xd::LIDoutputstateMsg (ROS-1 only) */
sick_scan_xd::LIDoutputstateMsg SickScanApiConverter::convertLIDoutputstateMsg(const SickScanLIDoutputstateMsg& src_msg)
{
    sick_scan_xd::LIDoutputstateMsg dst_msg;
    // Copy header
    dst_msg.header.seq = src_msg.header.seq;
    dst_msg.header.stamp.sec = src_msg.header.timestamp_sec;
    dst_msg.header.stamp.nsec = src_msg.header.timestamp_nsec;
    dst_msg.header.frame_id = src_msg.header.frame_id;
    // Copy LIDoutputstate data
    dst_msg.version_number = src_msg.version_number;
    dst_msg.system_counter = src_msg.system_counter;
    int max_states = (int)(sizeof(src_msg.output_state) / sizeof(src_msg.output_state[0]));
    int max_counts = (int)(sizeof(src_msg.output_count) / sizeof(src_msg.output_count[0]));
    dst_msg.output_state.resize(max_states);
    dst_msg.output_count.resize(max_counts);
    for(int n = 0; n < max_states; n++)
        dst_msg.output_state[n] = src_msg.output_state[n];
    for(int n = 0; n < max_counts; n++)
        dst_msg.output_count[n] = src_msg.output_count[n];
    dst_msg.time_state = src_msg.time_state;
    dst_msg.year = src_msg.year;
    dst_msg.month = src_msg.month;
    dst_msg.day = src_msg.day;
    dst_msg.hour = src_msg.hour;
    dst_msg.minute = src_msg.minute;
    dst_msg.second = src_msg.second;
    dst_msg.microsecond = src_msg.microsecond;
    return dst_msg;
}

/* Convert a SickScanLIDoutputstateMsg to sick_scan_xd::RadarScan (ROS-1 only) */
sick_scan_xd::RadarScan SickScanApiConverter::convertRadarScanMsg(const SickScanRadarScan& src_msg)
{
    sick_scan_xd::RadarScan dst_msg;
    // Copy header
    dst_msg.header.seq = src_msg.header.seq;
    dst_msg.header.stamp.sec = src_msg.header.timestamp_sec;
    dst_msg.header.stamp.nsec = src_msg.header.timestamp_nsec;
    dst_msg.header.frame_id = src_msg.header.frame_id;
    // Copy radarpreheader data
    dst_msg.radarpreheader.uiversionno = src_msg.radarpreheader.uiversionno;
    dst_msg.radarpreheader.radarpreheaderdeviceblock.uiident = src_msg.radarpreheader.uiident;
    dst_msg.radarpreheader.radarpreheaderdeviceblock.udiserialno = src_msg.radarpreheader.udiserialno;
    dst_msg.radarpreheader.radarpreheaderdeviceblock.bdeviceerror = src_msg.radarpreheader.bdeviceerror;
    dst_msg.radarpreheader.radarpreheaderdeviceblock.bcontaminationwarning = src_msg.radarpreheader.bcontaminationwarning;
    dst_msg.radarpreheader.radarpreheaderdeviceblock.bcontaminationerror = src_msg.radarpreheader.bcontaminationerror;
    dst_msg.radarpreheader.radarpreheaderstatusblock.uitelegramcount = src_msg.radarpreheader.uitelegramcount;
    dst_msg.radarpreheader.radarpreheaderstatusblock.uicyclecount = src_msg.radarpreheader.uicyclecount;
    dst_msg.radarpreheader.radarpreheaderstatusblock.udisystemcountscan = src_msg.radarpreheader.udisystemcountscan;
    dst_msg.radarpreheader.radarpreheaderstatusblock.udisystemcounttransmit = src_msg.radarpreheader.udisystemcounttransmit;
    dst_msg.radarpreheader.radarpreheaderstatusblock.uiinputs = src_msg.radarpreheader.uiinputs;
    dst_msg.radarpreheader.radarpreheaderstatusblock.uioutputs = src_msg.radarpreheader.uioutputs;
    dst_msg.radarpreheader.radarpreheadermeasurementparam1block.uicycleduration = src_msg.radarpreheader.uicycleduration;
    dst_msg.radarpreheader.radarpreheadermeasurementparam1block.uinoiselevel = src_msg.radarpreheader.uinoiselevel;
    dst_msg.radarpreheader.radarpreheaderarrayencoderblock.resize(src_msg.radarpreheader.numencoder);
    for(int n = 0; n < src_msg.radarpreheader.numencoder; n++)
    {
        dst_msg.radarpreheader.radarpreheaderarrayencoderblock[n].udiencoderpos = src_msg.radarpreheader.udiencoderpos[n];
        dst_msg.radarpreheader.radarpreheaderarrayencoderblock[n].iencoderspeed = src_msg.radarpreheader.iencoderspeed[n];
    }
    // Copy radar target pointcloud data
    dst_msg.targets = convertPointCloudMsg(src_msg.targets);
    // Copy radar object data
    dst_msg.objects.resize(src_msg.objects.size);
    for(int n = 0; n < src_msg.objects.size; n++)
    {
        const SickScanRadarObject& src_object = src_msg.objects.buffer[n];
        dst_msg.objects[n].id = src_object.id;
        dst_msg.objects[n].tracking_time.sec = src_object.tracking_time_sec;
        dst_msg.objects[n].tracking_time.nsec = src_object.tracking_time_nsec;
        dst_msg.objects[n].last_seen.sec = src_object.last_seen_sec;
        dst_msg.objects[n].last_seen.nsec = src_object.last_seen_nsec;
        dst_msg.objects[n].velocity.twist.linear.x = src_object.velocity_linear.x;
        dst_msg.objects[n].velocity.twist.linear.y = src_object.velocity_linear.y;
        dst_msg.objects[n].velocity.twist.linear.y = src_object.velocity_linear.z;
        dst_msg.objects[n].velocity.twist.angular.x = src_object.velocity_angular.x;
        dst_msg.objects[n].velocity.twist.angular.y = src_object.velocity_angular.y;
        dst_msg.objects[n].velocity.twist.angular.y = src_object.velocity_angular.z;
        for(int m = 0; m < 36; m++)
            dst_msg.objects[n].velocity.covariance[m] = src_object.velocity_covariance[m];
        dst_msg.objects[n].bounding_box_center.position.x = src_object.bounding_box_center_position.x;
        dst_msg.objects[n].bounding_box_center.position.y = src_object.bounding_box_center_position.y;
        dst_msg.objects[n].bounding_box_center.position.z = src_object.bounding_box_center_position.z;
        dst_msg.objects[n].bounding_box_center.orientation.x = src_object.bounding_box_center_orientation.x;
        dst_msg.objects[n].bounding_box_center.orientation.y = src_object.bounding_box_center_orientation.y;
        dst_msg.objects[n].bounding_box_center.orientation.z = src_object.bounding_box_center_orientation.z;
        dst_msg.objects[n].bounding_box_center.orientation.w = src_object.bounding_box_center_orientation.w;
        dst_msg.objects[n].bounding_box_size.x = src_object.bounding_box_size.x;
        dst_msg.objects[n].bounding_box_size.y = src_object.bounding_box_size.y;
        dst_msg.objects[n].bounding_box_size.z = src_object.bounding_box_size.z;
        dst_msg.objects[n].object_box_center.pose.position.x = src_object.object_box_center_position.x;
        dst_msg.objects[n].object_box_center.pose.position.y = src_object.object_box_center_position.y;
        dst_msg.objects[n].object_box_center.pose.position.z = src_object.object_box_center_position.z;
        dst_msg.objects[n].object_box_center.pose.orientation.x = src_object.object_box_center_orientation.x;
        dst_msg.objects[n].object_box_center.pose.orientation.y = src_object.object_box_center_orientation.y;
        dst_msg.objects[n].object_box_center.pose.orientation.z = src_object.object_box_center_orientation.z;
        dst_msg.objects[n].object_box_center.pose.orientation.w = src_object.object_box_center_orientation.w;
        for(int m = 0; m < 36; m++)
            dst_msg.objects[n].object_box_center.covariance[m] = src_object.object_box_center_covariance[m];
        dst_msg.objects[n].object_box_size.x = src_object.object_box_size.x;
        dst_msg.objects[n].object_box_size.y = src_object.object_box_size.y;
        dst_msg.objects[n].object_box_size.z = src_object.object_box_size.z;
        dst_msg.objects[n].contour_points.resize(src_object.contour_points.size);
        for(int m = 0; m < src_object.contour_points.size; m++)
        {
            dst_msg.objects[n].contour_points[m].x = src_object.contour_points.buffer[m].x;
            dst_msg.objects[n].contour_points[m].y = src_object.contour_points.buffer[m].y;
            dst_msg.objects[n].contour_points[m].z = src_object.contour_points.buffer[m].z;
        }
    }
    return dst_msg;
}

/* Convert a RadarScan objects to sensor_msgs::PointCloud2 (ROS-1 only) */
sensor_msgs::PointCloud2 SickScanApiConverter::convertRadarObjectsToPointCloud(const SickScanHeader& header, const sick_scan_xd::RadarObject* radar_objects, int num_objects)
{
    sensor_msgs::PointCloud2 ros_pointcloud;
    // Copy pointcloud header
    ros_pointcloud.header.seq = header.seq;
    ros_pointcloud.header.stamp.sec = header.timestamp_sec;
    ros_pointcloud.header.stamp.nsec = header.timestamp_nsec;
    ros_pointcloud.header.frame_id = header.frame_id;
    ros_pointcloud.width = num_objects;
    ros_pointcloud.height = 1;
    ros_pointcloud.is_bigendian = false;
    ros_pointcloud.is_dense = true;
    // Set field description
    std::vector<std::string> field_names = {"x", "y", "z", "vx", "vy", "vz"};
    ros_pointcloud.fields.resize(field_names.size());
    for(int n = 0; n < ros_pointcloud.fields.size(); n++)
    {
        ros_pointcloud.fields[n].name = field_names[n];
        ros_pointcloud.fields[n].offset = n * sizeof(float);
        ros_pointcloud.fields[n].datatype = sensor_msgs::PointField::FLOAT32;
        ros_pointcloud.fields[n].count = 1;
    }
    ros_pointcloud.point_step = ros_pointcloud.fields.size() * sizeof(float);
    ros_pointcloud.row_step = ros_pointcloud.point_step * ros_pointcloud.width;
    // Copy radar object data
    ros_pointcloud.data.resize(ros_pointcloud.row_step * ros_pointcloud.height, 0);
    float* dst_data_p = (float*)ros_pointcloud.data.data();
    for(int n = 0; n < num_objects; n++, dst_data_p+=6)
    {
        dst_data_p[0] = radar_objects[n].object_box_center.pose.position.x;
        dst_data_p[1] = radar_objects[n].object_box_center.pose.position.y;
        dst_data_p[2] = radar_objects[n].object_box_center.pose.position.z;
        dst_data_p[3] = radar_objects[n].velocity.twist.linear.x;
        dst_data_p[4] = radar_objects[n].velocity.twist.linear.y;
        dst_data_p[5] = radar_objects[n].velocity.twist.linear.z;
    }
    return ros_pointcloud;
}

/* Convert a SickScanLdmrsObjectArray to sensor_msgs::SickLdmrsObjectArray (ROS-1 only) */
sick_scan_xd::SickLdmrsObjectArray SickScanApiConverter::convertLdmrsObjectArray(const SickScanLdmrsObjectArray& src_msg)
{
    sick_scan_xd::SickLdmrsObjectArray dst_msg;
    // Copy header
    dst_msg.header.seq = src_msg.header.seq;
    dst_msg.header.stamp.sec = src_msg.header.timestamp_sec;
    dst_msg.header.stamp.nsec = src_msg.header.timestamp_nsec;
    dst_msg.header.frame_id = src_msg.header.frame_id;
    // Copy ldmrs objects
    dst_msg.objects.resize(src_msg.objects.size);
    for(int n = 0; n < src_msg.objects.size; n++)
    {
        const SickScanLdmrsObject& src_object = src_msg.objects.buffer[n];
        dst_msg.objects[n].id = src_object.id;
        dst_msg.objects[n].tracking_time.sec = src_object.tracking_time_sec;
        dst_msg.objects[n].tracking_time.nsec = src_object.tracking_time_nsec;
        dst_msg.objects[n].last_seen.sec = src_object.last_seen_sec;
        dst_msg.objects[n].last_seen.nsec = src_object.last_seen_nsec;
        dst_msg.objects[n].velocity.twist.linear.x = src_object.velocity_linear.x;
        dst_msg.objects[n].velocity.twist.linear.y = src_object.velocity_linear.y;
        dst_msg.objects[n].velocity.twist.linear.y = src_object.velocity_linear.z;
        dst_msg.objects[n].velocity.twist.angular.x = src_object.velocity_angular.x;
        dst_msg.objects[n].velocity.twist.angular.y = src_object.velocity_angular.y;
        dst_msg.objects[n].velocity.twist.angular.y = src_object.velocity_angular.z;
        for(int m = 0; m < 36; m++)
            dst_msg.objects[n].velocity.covariance[m] = src_object.velocity_covariance[m];
        dst_msg.objects[n].bounding_box_center.position.x = src_object.bounding_box_center_position.x;
        dst_msg.objects[n].bounding_box_center.position.y = src_object.bounding_box_center_position.y;
        dst_msg.objects[n].bounding_box_center.position.z = src_object.bounding_box_center_position.z;
        dst_msg.objects[n].bounding_box_center.orientation.x = src_object.bounding_box_center_orientation.x;
        dst_msg.objects[n].bounding_box_center.orientation.y = src_object.bounding_box_center_orientation.y;
        dst_msg.objects[n].bounding_box_center.orientation.z = src_object.bounding_box_center_orientation.z;
        dst_msg.objects[n].bounding_box_center.orientation.w = src_object.bounding_box_center_orientation.w;
        dst_msg.objects[n].bounding_box_size.x = src_object.bounding_box_size.x;
        dst_msg.objects[n].bounding_box_size.y = src_object.bounding_box_size.y;
        dst_msg.objects[n].bounding_box_size.z = src_object.bounding_box_size.z;
        dst_msg.objects[n].object_box_center.pose.position.x = src_object.object_box_center_position.x;
        dst_msg.objects[n].object_box_center.pose.position.y = src_object.object_box_center_position.y;
        dst_msg.objects[n].object_box_center.pose.position.z = src_object.object_box_center_position.z;
        dst_msg.objects[n].object_box_center.pose.orientation.x = src_object.object_box_center_orientation.x;
        dst_msg.objects[n].object_box_center.pose.orientation.y = src_object.object_box_center_orientation.y;
        dst_msg.objects[n].object_box_center.pose.orientation.z = src_object.object_box_center_orientation.z;
        dst_msg.objects[n].object_box_center.pose.orientation.w = src_object.object_box_center_orientation.w;
        for(int m = 0; m < 36; m++)
            dst_msg.objects[n].object_box_center.covariance[m] = src_object.object_box_center_covariance[m];
        dst_msg.objects[n].object_box_size.x = src_object.object_box_size.x;
        dst_msg.objects[n].object_box_size.y = src_object.object_box_size.y;
        dst_msg.objects[n].object_box_size.z = src_object.object_box_size.z;
        dst_msg.objects[n].contour_points.resize(src_object.contour_points.size);
        for(int m = 0; m < src_object.contour_points.size; m++)
        {
            dst_msg.objects[n].contour_points[m].x = src_object.contour_points.buffer[m].x;
            dst_msg.objects[n].contour_points[m].y = src_object.contour_points.buffer[m].y;
            dst_msg.objects[n].contour_points[m].z = src_object.contour_points.buffer[m].z;
        }
    }
    return dst_msg;
}

/* Convert a SickScanVisualizationMarkerMsg to visualization_msgs::MarkerArray (ROS-1 only) */
visualization_msgs::MarkerArray SickScanApiConverter::convertVisualizationMarkerMsg(const SickScanVisualizationMarkerMsg& src_msg)
{
    visualization_msgs::MarkerArray dst_msg;
    if (src_msg.markers.size > 0)
    {
        // Copy markers
        dst_msg.markers.resize(src_msg.markers.size);
        for(int n = 0; n < src_msg.markers.size; n++)
        {
            const SickScanVisualizationMarker& src_marker = src_msg.markers.buffer[n];
            visualization_msgs::Marker& dst_marker = dst_msg.markers[n];
            // Copy header
            dst_marker.header.seq = src_marker.header.seq;
            dst_marker.header.stamp.sec = src_marker.header.timestamp_sec;
            dst_marker.header.stamp.nsec = src_marker.header.timestamp_nsec;
            dst_marker.header.frame_id = src_marker.header.frame_id;
            // Copy data
            dst_marker.ns = src_marker.ns;
            dst_marker.id = src_marker.id;
            dst_marker.type = src_marker.type;
            dst_marker.action = src_marker.action;
            dst_marker.pose.position.x = src_marker.pose_position.x;
            dst_marker.pose.position.y = src_marker.pose_position.y;
            dst_marker.pose.position.z = src_marker.pose_position.z;
            dst_marker.pose.orientation.x = src_marker.pose_orientation.x;
            dst_marker.pose.orientation.y = src_marker.pose_orientation.y;
            dst_marker.pose.orientation.z = src_marker.pose_orientation.z;
            dst_marker.pose.orientation.w = src_marker.pose_orientation.w;
            dst_marker.scale.x = src_marker.scale.x;
            dst_marker.scale.y = src_marker.scale.y;
            dst_marker.scale.z = src_marker.scale.z;
            dst_marker.color.r = src_marker.color.r;
            dst_marker.color.g = src_marker.color.g;
            dst_marker.color.b = src_marker.color.b;
            dst_marker.color.a = src_marker.color.a;
            dst_marker.lifetime.sec = src_marker.lifetime_sec;
            dst_marker.lifetime.nsec = src_marker.lifetime_nsec;
            dst_marker.frame_locked = src_marker.frame_locked;
            dst_marker.text = src_marker.text;
            dst_marker.mesh_resource = src_marker.mesh_resource;
            dst_marker.mesh_use_embedded_materials = src_marker.mesh_use_embedded_materials;
            dst_marker.points.resize(src_marker.points.size);
            for(int m = 0; m < src_marker.points.size; m++)
            {
                dst_marker.points[m].x = src_marker.points.buffer[m].x;
                dst_marker.points[m].y = src_marker.points.buffer[m].y;
                dst_marker.points[m].z = src_marker.points.buffer[m].z;
            }
            dst_marker.colors.resize(src_marker.colors.size);
            for(int m = 0; m < src_marker.colors.size; m++)
            {
                dst_marker.colors[m].r = src_marker.colors.buffer[m].r;
                dst_marker.colors[m].g = src_marker.colors.buffer[m].g;
                dst_marker.colors[m].b = src_marker.colors.buffer[m].b;
                dst_marker.colors[m].a = src_marker.colors.buffer[m].a;
            }
        }
    }
    return dst_msg;
}

#endif // __ROS_VERSION == 1
