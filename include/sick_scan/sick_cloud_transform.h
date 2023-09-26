#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
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
 * class SickCloudTransform applies an additional transform to the cartesian pointcloud, default: "0,0,0,0,0,0" (i.e. no transform)
 * Note: add_transform_xyz_rpy is specified by 6D pose x,y,z,roll,pitch,yaw in [m] resp. [rad]
 * It transforms a 3D point in cloud coordinates to 3D point in user defined world coordinates:
 * add_transform_xyz_rpy := T[world,cloud] with parent "world" and child "cloud", i.e. P_world = T[world,cloud] * P_cloud
 * The additional transform applies to cartesian lidar pointclouds and visualization marker (fields)
 * It is NOT applied to polar pointclouds, radarscans, ldmrs objects or other messages
 *
 */

#ifndef SICK_CLOUD_TRANSFORM_H_
#define SICK_CLOUD_TRANSFORM_H_

#include <array>
#include <string>
#include <vector>

#include <sick_scan/sick_ros_wrapper.h>

namespace sick_scan_xd
{
    /*
    * class SickCloudTransform applies an additional transform to the cartesian pointcloud, default: "0,0,0,0,0,0" (i.e. no transform)
    * Note: add_transform_xyz_rpy is specified by 6D pose x,y,z,roll,pitch,yaw in [m] resp. [rad]
    * It transforms a 3D point in cloud coordinates to 3D point in user defined world coordinates:
    * add_transform_xyz_rpy := T[world,cloud] with parent "world" and child "cloud", i.e. P_world = T[world,cloud] * P_cloud
    * The additional transform applies to cartesian lidar pointclouds and visualization marker (fields)
    * It is NOT applied to polar pointclouds, radarscans, ldmrs objects or other messages
    */
    class SickCloudTransform
    {
    public:

        SickCloudTransform();
        SickCloudTransform(rosNodePtr nh, bool cartesian_input_only /* = false */);
        SickCloudTransform(rosNodePtr nh, const std::string& add_transform_xyz_rpy, bool cartesian_input_only /* = false */, bool add_transform_check_dynamic_updates /* = false */);

        /*
        * Apply an optional transform to point (x, y, z).
        * @param[in] float_type: float or double
        * @param[in+out] x input x in child coordinates, output x in parent coordinates
        * @param[in+out] y input y in child coordinates, output y in parent coordinates
        * @param[in+out] z input z in child coordinates, output z in parent coordinates
        */
        template<typename float_type> inline void applyTransform(float_type& x, float_type& y, float_type& z)
        {
            // Check parameter and re-init if parameter "add_transform_xyz_rpy" changed
            if (m_add_transform_check_dynamic_updates && m_nh)
            {
                std::string add_transform_xyz_rpy = m_add_transform_xyz_rpy;
                rosGetParam(m_nh, "add_transform_xyz_rpy", add_transform_xyz_rpy);
                if (m_add_transform_xyz_rpy != add_transform_xyz_rpy)
                {
                    if (!init(add_transform_xyz_rpy, m_cartesian_input_only, m_add_transform_check_dynamic_updates))
                    {
                        ROS_ERROR_STREAM("## ERROR SickCloudTransform(): Re-Initialization by \"" << add_transform_xyz_rpy << "\" failed, use 6D pose \"x,y,z,roll,pitch,yaw\" in [m] resp. [rad]");
                    }
                }
            }
            // Apply transform
            if (m_apply_3x3_rotation)
            {
                float_type u = x * m_rotation_matrix[0][0] + y * m_rotation_matrix[0][1] + z * m_rotation_matrix[0][2];
                float_type v = x * m_rotation_matrix[1][0] + y * m_rotation_matrix[1][1] + z * m_rotation_matrix[1][2];
                float_type w = x * m_rotation_matrix[2][0] + y * m_rotation_matrix[2][1] + z * m_rotation_matrix[2][2];
                x = u;
                y = v;
                z = w;
            }
            x += m_translation_vector[0];
            y += m_translation_vector[1];
            z += m_translation_vector[2];
        }

        /*
        * Return the azimuth offset, i.e. yaw in [rad] if only yaw is configured, or 0 otherwise (default)
        */
        inline float azimuthOffset(void) const
        {
            return m_azimuth_offset;
        }

    protected:

        typedef std::array<float, 3> Vector3D; // 3D translation vector
        typedef std::array<std::array<float, 3>, 3> Matrix3x3; // 3x3 rotation matrix

        // Initializes rotation matrix and translation vector from a 6D pose configuration (x,y,z,roll,pitch,yaw) in [m] resp. [rad]
        bool init(const std::string& add_transform_xyz_rpy, bool cartesian_input_only, bool add_transform_check_dynamic_updates);

        // Converts roll (rotation about X), pitch (rotation about Y), yaw (rotation about Z) to 3x3 rotation matrix
        static Matrix3x3 eulerToRot3x3(float roll, float pitch, float yaw);

        // Multiply two 3x3 matrices, return a * b
        static Matrix3x3 multiply3x3(const Matrix3x3& a, const Matrix3x3& b);

        rosNodePtr m_nh = 0; // ros node handle
        std::string m_add_transform_xyz_rpy = "";                     // currently configured ros parameter "add_transform_xyz_rpy"
        bool m_add_transform_check_dynamic_updates = false;           // True: ros parameter "add_transform_xyz_rpy" can be updated during runtime by "rosparam set", False: parameter "add_transform_xyz_rpy" configured by launchfile only
        bool m_cartesian_input_only = false;                          // currently configured parameter cartesian_input_only
        bool m_apply_3x3_rotation = false;                            // true, if the 3x3 rotation_matrix has to be applied, otherwise false (default)
        Vector3D m_translation_vector = { 0, 0, 0 };                  // translational part x,y,z of the 6D pose, default: 0
        Matrix3x3 m_rotation_matrix = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };  // rotational part roll,pitch,yaw by 3x3 rotation matrix, default: 3x3 identity
        float m_azimuth_offset = 0; // azimuth offset, i.e. yaw in [rad], if only yaw is configured (in this case an offset can be added to the lidar azimuth before conversion to cartesian pointcloud, which is faster than a 3x3 matrix multiplication)

    }; // class SickCloudTransform
} // namespace sick_scan_xd
#endif // SICK_CLOUD_TRANSFORM_H_
