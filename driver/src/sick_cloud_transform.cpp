/*
* Copyright (C) 2017, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2017, SICK AG, Waldkirch
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
 *  Created on: 12.08.2022
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 */
#include <float.h>
#include <sick_scan/sick_cloud_transform.h>
#include <sick_scan/sick_scan_parse_util.h>

/*
* class SickCloudTransform applies an additional transform to the cartesian pointcloud, default: "0,0,0,0,0,0" (i.e. no transform)
* Note: add_transform_xyz_rpy is specified by 6D pose x,y,z,roll,pitch,yaw in [m] resp. [rad]
* It transforms a 3D point in cloud coordinates to 3D point in user defined world coordinates:
* add_transform_xyz_rpy := T[world,cloud] with parent "world" and child "cloud", i.e. P_world = T[world,cloud] * P_cloud
* The additional transform applies to cartesian lidar pointclouds and visualization marker (fields)
* It is NOT applied to polar pointclouds, radarscans, ldmrs objects or other messages
*/

sick_scan_xd::SickCloudTransform::SickCloudTransform()
{
}

sick_scan_xd::SickCloudTransform::SickCloudTransform(rosNodePtr nh, bool cartesian_input_only)
{
    m_nh = nh;
    std::string add_transform_xyz_rpy = "0,0,0,0,0,0";
    rosDeclareParam(nh, "add_transform_xyz_rpy", add_transform_xyz_rpy);
    rosGetParam(nh, "add_transform_xyz_rpy", add_transform_xyz_rpy);
    bool add_transform_check_dynamic_updates = false;
    rosDeclareParam(nh, "add_transform_check_dynamic_updates", add_transform_check_dynamic_updates);
    rosGetParam(nh, "add_transform_check_dynamic_updates", add_transform_check_dynamic_updates);
    if (!init(add_transform_xyz_rpy, cartesian_input_only, add_transform_check_dynamic_updates))
    {
        ROS_ERROR_STREAM("## ERROR SickCloudTransform(): Initialization by \"" << add_transform_xyz_rpy << "\" failed, use 6D pose \"x,y,z,roll,pitch,yaw\" in [m] resp. [rad]");
    }
}

sick_scan_xd::SickCloudTransform::SickCloudTransform(rosNodePtr nh, const std::string& add_transform_xyz_rpy, bool cartesian_input_only, bool add_transform_check_dynamic_updates)
{
    m_nh = nh;
    if (!init(add_transform_xyz_rpy, cartesian_input_only, add_transform_check_dynamic_updates))
    {
        ROS_ERROR_STREAM("## ERROR SickCloudTransform(): Initialization by \"" << add_transform_xyz_rpy << "\" failed, use 6D pose \"x,y,z,roll,pitch,yaw\" in [m] resp. [rad]");
    }
}

// Initializes rotation matrix and translation vector from a 6D pose configuration (x,y,z,roll,pitch,yaw) in [m] resp. [rad]
bool sick_scan_xd::SickCloudTransform::init(const std::string& add_transform_xyz_rpy, bool cartesian_input_only, bool add_transform_check_dynamic_updates)
{
    // Split string add_transform_xyz_rpy to 6D pose x,y,z,roll,pitch,yaw in [m] resp. [rad]
    std::vector<float> config_values = sick_scan_xd::parsePose(add_transform_xyz_rpy);
    if(config_values.size() != 6)
    {
        ROS_ERROR_STREAM("## ERROR SickCloudTransform(): Can't parse config string \"" << add_transform_xyz_rpy << "\", use 6D pose \"x,y,z,roll,pitch,yaw\" in [m] resp. [rad]");
        return false;
    }
    m_translation_vector[0] = config_values[0];
    m_translation_vector[1] = config_values[1];
    m_translation_vector[2] = config_values[2];
    m_apply_3x3_rotation = false;
    m_rotation_matrix = { 1, 0, 0, 0, 1, 0, 0, 0, 1 }; // 3x3 identity
    m_azimuth_offset = 0;
    if (cartesian_input_only)
    {
        if (fabs(config_values[3]) > FLT_EPSILON || fabs(config_values[4]) > FLT_EPSILON || fabs(config_values[5]) > FLT_EPSILON)
        {
            m_apply_3x3_rotation = true; // i.e. we have to apply the 3x3 rotation matrix to the cartesian pointclouds
            m_rotation_matrix = eulerToRot3x3(config_values[3], config_values[4], config_values[5]);
        }
    }
    else
    {
        if (fabs(config_values[3]) < FLT_EPSILON && fabs(config_values[4]) < FLT_EPSILON)
        {
            // roll == 0, pitch == 0, i.e. only yaw (rotation about z-axis) is configured. In this case an offset can be added to the lidar azimuth
            // before conversion to cartesian pointcloud, which is faster than a 3x3 matrix multiplication.
            m_apply_3x3_rotation = false; // roll == 0, pitch == 0, offset for azimuth is sufficient
            m_rotation_matrix = { 1, 0, 0, 0, 1, 0, 0, 0, 1 }; // 3x3 identity
            m_azimuth_offset = config_values[5];   // azimuth offset := yaw in [rad]
        }
        else
        {
            m_apply_3x3_rotation = true; // i.e. we have to apply the 3x3 rotation matrix to the cartesian pointclouds
            m_rotation_matrix = eulerToRot3x3(config_values[3], config_values[4], config_values[5]);
            m_azimuth_offset = 0;
        }
    }
    // Initialization successful
    m_add_transform_xyz_rpy = add_transform_xyz_rpy;
    m_cartesian_input_only = cartesian_input_only;
    m_add_transform_check_dynamic_updates = add_transform_check_dynamic_updates;
    ROS_INFO_STREAM("SickCloudTransform: add_transform_xyz_rpy = (" << add_transform_xyz_rpy << ")");
    ROS_INFO_STREAM("SickCloudTransform: azimuth_offset = " << (m_azimuth_offset * 180.0 / M_PI) << " [deg]");
    ROS_INFO_STREAM("SickCloudTransform: additional 3x3 rotation matrix = { (" 
        << m_rotation_matrix[0][0] << "," << m_rotation_matrix[0][1] << "," << m_rotation_matrix[0][2] << "), (" 
        << m_rotation_matrix[1][0] << "," << m_rotation_matrix[1][1] << "," << m_rotation_matrix[1][2] << "), (" 
        << m_rotation_matrix[2][0] << "," << m_rotation_matrix[2][1] << "," << m_rotation_matrix[2][2] << ") }");
    ROS_INFO_STREAM("SickCloudTransform: apply 3x3 rotation = " << (m_apply_3x3_rotation ? "true" : "false"));
    ROS_INFO_STREAM("SickCloudTransform: additional translation = (" << m_translation_vector[0] << "," << m_translation_vector[1] << "," << m_translation_vector[2] << ")");
    ROS_INFO_STREAM("SickCloudTransform: check_dynamic_updates = " << (m_add_transform_check_dynamic_updates ? "true" : "false"));
    return true;
}

// converts roll (rotation about X), pitch (rotation about Y), yaw (rotation about Z) to 3x3 rotation matrix
sick_scan_xd::SickCloudTransform::Matrix3x3 sick_scan_xd::SickCloudTransform::eulerToRot3x3(float roll, float pitch, float yaw)
{
    // roll: rotation about x axis
    Matrix3x3 rot_x = {
        1, 0, 0,
        0, cosf(roll), -sinf(roll),
        0, sinf(roll), cosf(roll)
    };
    // pitch: rotation about y axis
    Matrix3x3 rot_y = {
        cosf(pitch), 0, sinf(pitch),
        0, 1, 0,
        -sinf(pitch), 0, cosf(pitch)
    };
    // yaw: rotation about z axis
    Matrix3x3 rot_z = {
        cosf(yaw), -sinf(yaw), 0,
        sinf(yaw), cosf(yaw), 0,
        0, 0, 1
    };
    // 3x3 rotation matrix
    Matrix3x3 rot_3x3 = multiply3x3(multiply3x3(rot_z, rot_y), rot_x); // rot_3x3 = rot_z * rot_y * rot_x
    return rot_3x3;
}

// Multiply two 3x3 matrices, return a * b
sick_scan_xd::SickCloudTransform::Matrix3x3 sick_scan_xd::SickCloudTransform::multiply3x3(const sick_scan_xd::SickCloudTransform::Matrix3x3& a, const sick_scan_xd::SickCloudTransform::Matrix3x3& b)
{
    Matrix3x3 result3x3 = {

        a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
        a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
        a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],

        a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
        a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
        a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],

        a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
        a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
        a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2]
    };
    return result3x3;
}
