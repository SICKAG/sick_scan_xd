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
 */

#ifndef SICK_RANGE_FILTER_H_
#define SICK_RANGE_FILTER_H_

#include <cfloat>
#include <iomanip>
#include <sick_scan/sick_ros_wrapper.h>

namespace sick_scan_xd
{
    /*
    * enum RangeFilterResultHandling configures the range filter handling
    * Note: Range filter applies only to Pointcloud messages, not to LaserScan messages.
    */
    typedef enum RangeFilterResultHandlingEnum
    {
        RANGE_FILTER_DEACTIVATED = 0,  // do not apply range filter
        RANGE_FILTER_DROP = 1,         // drop point, if range is not within [range_min, range_max]
        RANGE_FILTER_TO_ZERO = 2,      // set range = 0, if range is not within [range_min, range_max]
        RANGE_FILTER_TO_RANGE_MAX = 3, // set range = range_max, if range is not within [range_min, range_max]
        RANGE_FILTER_TO_FLT_MAX = 4,   // set range = FLT_MAX, if range is not within [range_min, range_max]
        RANGE_FILTER_TO_NAN = 5        // set range = NAN, if range is not within [range_min, range_max]
    } RangeFilterResultHandling;

    /*
    * class SickRangeFilter filters a pointcloud by range.
    */
    class SickRangeFilter
    {
    public:

        /*
        * Initializing constructor
        * @param[in] range_min min range in meter, default: 0
        * @param[in] range_max max range in meter, default: FLT_MAX
        * @param[in] settings filter configuration, default: RANGE_FILTER_DEACTIVATED (do not apply range filter)
        */
        SickRangeFilter(float range_min = 0, float range_max = FLT_MAX, RangeFilterResultHandling settings = RangeFilterResultHandling::RANGE_FILTER_DEACTIVATED)
        : m_range_min(range_min), m_range_max(range_max), m_settings(settings)
        {
		}

        /*
        * Apply an optional range filter.
        * Note: Range filter applies only to Pointcloud messages, not to LaserScan messages.
        * @param[in+out] range range in meter (input: range from scan message, output: range after filter)
        * @param[out] range_modified true, if range has been overwritten, otherwise false
        * @return false if point dropped, otherwise true (i.e. append a point to the pointcloud, if apply returns true)
        */
        bool apply(float& range, bool& range_modified) const
        {
            bool ret_val = true;
            range_modified = false;
            if ((m_settings != RANGE_FILTER_DEACTIVATED) && (range < m_range_min || range > m_range_max)) // range not in [range_min, range_max], apply filter
            {
                switch(m_settings)
                {
                    case RANGE_FILTER_DEACTIVATED:  // do not apply range filter
                        break;
                    case RANGE_FILTER_DROP:         // drop point, if range is not within [range_min, range_max]
                        ret_val = false;
                        break;
                    case RANGE_FILTER_TO_ZERO:      // set range = 0, if range is not within [range_min, range_max]
                        range = 0;
                        range_modified = true;
                        break;
                    case RANGE_FILTER_TO_RANGE_MAX: // set range = range_max, if range is not within [range_min, range_max]
                        range = m_range_max;
                        range_modified = true;
                        break;
                    case RANGE_FILTER_TO_FLT_MAX:   // set range = FLT_MAX, if range is not within [range_min, range_max]
                        range = FLT_MAX;
                        range_modified = true;
                        break;
                    case RANGE_FILTER_TO_NAN:       // set range = NAN, if range is not within [range_min, range_max]
                        range = std::nanf("");
                        range_modified = true;
                        break;
                    default:
                        ROS_ERROR_STREAM("## ERROR SickRangeFilter::apply(): invalid setting " << m_settings << ", please check parameter \"range_filter_handling\" in the configuration and/or launch-file.");
                        break;
                }
            }
			return ret_val;
        }

        /*
        * Overwrites x,y,z of a lidar_point if range has been set to 0, max, FLT_MAX or NaN by apply
        * @return true if x,y,z have been set, false otherwise
        */
        bool applyXYZ(float& x, float& y, float& z, float azimuth, float elevation)
        {
            bool ret_val = false;
            switch(m_settings) 
            {
                case RANGE_FILTER_TO_ZERO:
                    x = y = z = 0.0f;
                    ret_val = true;
                    break;
                case RANGE_FILTER_TO_RANGE_MAX:
                    x = m_range_max * cos(elevation) * cos(azimuth);
                    y = m_range_max * cos(elevation) * sin(azimuth);
                    z = m_range_max * sin(elevation);
                    ret_val = true;
                    break;
                case RANGE_FILTER_TO_FLT_MAX: 
                    x = y = z = FLT_MAX;
                    ret_val = true;
                    break;
                case RANGE_FILTER_TO_NAN:
                    x = y = z = std::nanf("");;
                    ret_val = true;
                    break;
                default:
                    break;
            }
            return ret_val;
        }

        /*
        * Copy and resize a pointcloud after applying an optional range filter.
        * @param[in] rangeNum number of points after range filtering
        * @param[in+out] pointcloud pointcloud with resized data fields
        */
        void resizePointCloud(size_t rangeNum, ros_sensor_msgs::PointCloud2& cloud)
        {
            ros_sensor_msgs::PointCloud2 resized_pointcloud;
            resized_pointcloud.header = cloud.header;
            resized_pointcloud.height = cloud.height;
            resized_pointcloud.width = rangeNum;
            resized_pointcloud.is_bigendian = cloud.is_bigendian;
            resized_pointcloud.is_dense = cloud.is_dense;
            resized_pointcloud.point_step = cloud.point_step;
            resized_pointcloud.fields = cloud.fields;
            resized_pointcloud.row_step = resized_pointcloud.point_step * resized_pointcloud.width;
            resized_pointcloud.data.resize(resized_pointcloud.row_step * resized_pointcloud.height, 0);
            for(size_t n = 0; n < resized_pointcloud.height; n++) // copy pointcloud data to flat array
            {
                memcpy(&resized_pointcloud.data[n * resized_pointcloud.row_step], &cloud.data[n * cloud.row_step], resized_pointcloud.row_step);
            }
            cloud = resized_pointcloud;
		}

        /*
        * Returns a human readable string of the filter configuration
        */
        std::string print(void) const
        {
            std::stringstream s;
            s << "(" << std::fixed << std::setprecision(3) << m_range_min << "," << m_range_max << "," << (int)(m_settings) << ")";
            return s.str();
        }

        /*
        * Returns the filter configuration
        */
        float rangeMin(void) const { return m_range_min; }
        float rangeMax(void) const { return m_range_max; }
        RangeFilterResultHandling setting(void) const { return m_settings; }

    protected:

        float m_range_min = 0;
        float m_range_max = FLT_MAX;
        RangeFilterResultHandling m_settings = RANGE_FILTER_DEACTIVATED;

    }; // class SickRangeFilter
} // namespace sick_scan_xd
#endif // SICK_RANGE_FILTER_H_
