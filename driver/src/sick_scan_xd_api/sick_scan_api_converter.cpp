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
    pointcloud.data.resize(msg.row_step * msg.height);
    memcpy(&pointcloud.data[0], msg.data.buffer, msg.row_step * msg.height);
    // Return converted pointcloud
    return pointcloud;
}
#endif // __ROS_VERSION == 1

#if __ROS_VERSION == 1
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
    pointcloud.data.resize(pointcloud.row_step * pointcloud.height);
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
#endif // __ROS_VERSION == 1
