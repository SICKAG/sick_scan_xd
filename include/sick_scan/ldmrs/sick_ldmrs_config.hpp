#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * Copyright (C) 2020, Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2020, SICK AG, Waldkirch
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
 *  Created on: 2nd Oct 2020
 *
 *      Authors:
 *       Michael Lehning <michael.lehning@lehning.de>
 *
 */

#ifndef __SICK_LDMRS_CONFIG_H__
#define __SICK_LDMRS_CONFIG_H__

#include <cmath> 
#include <string>
#include <sick_scan/sick_ros_wrapper.h>

namespace sick_ldmrs_driver
{
    /*
     * @brief Utility function to declare and get a ros parameter
     */
    template<typename T> bool param(rosNodePtr node, const std::string & param_name, T& value, const T& default_value)
    {
        ROS_DEBUG_STREAM("ROS::param(" << param_name << "," << default_value << ")");
        try
        {
            rosDeclareParam<T>(node, param_name, default_value);
        }
        catch(const std::exception& exc)
        {
            ROS_WARN_STREAM("## ERROR ROS::param: declare_parameter(" << param_name << "," << default_value << ") failed, exception " << exc.what());
        }
        try
        {
            bool ret = rosGetParam(node, param_name, value);
            ROS_INFO_STREAM("ROS::param(" << param_name << "," << default_value << "): " << param_name << "=" << value);
            return ret;
        }
        catch(const std::exception& exc)
        {
            ROS_WARN_STREAM("## ERROR ROS::param: get_parameter(" << param_name << "," << default_value << ") failed, exception " << exc.what());
        }
        return false;
    }

    /*
     * @brief Configuration of SICK LDMRS driver
     */
    class SickLDMRSDriverConfig
    {
    public:

        static const double deg2rad(double angle){ return (angle * M_PI / 180.0); }
        static const double tics2rad(double tics){ return (deg2rad(tics / 32.0)); }

        enum scan_freq_enum // "Available scan frequencies"
        {
            ScanFreq1250 = 0,  // "Scan frequency 12.5 Hz"
            ScanFreq2500 = 1,  // "Scan frequency 25.0 Hz"
            ScanFreq5000 = 2   // "Scan frequency 25.0 Hz"
        };

        enum contour_enum // "Contour point density"
        {
            ClosestPointOnly = 0,  // "Closest point only"
            LowDensity = 1,        // "Low density"
            HighDensity = 2        // "High density"
        };

        enum angular_res_enum // "Angular resolution type"
        {
            FocusedRes = 0,   // "Focused resolution"
            ConstantRes = 1,  // "Constant resolution"
            FlexRes = 2       // "Flexible resolution"
        };

        enum range_reduction_enum // "Layer range reduction"
        {
            RangeFull = 0,  // "All layers full range"
            RangeLowerReduced = 1,  // "Lower 4 layers reduced range"
            RangeUpperReduced = 2,  // "Upper 4 layers reduced range"
            RangeAllReduced = 3     // "All 8 layers reduced range"
        };

        enum resolution_enum // "FlexRes angular resolution"
        {
            Res0125 = 4,   // "Angular resolution 0.125 degrees"
            Res0250 = 8,   // "Angular resolution 0.25 degrees"
            Res0500 = 16,  // "Angular resolution 0.5 degrees"
            Res1000 = 32   // "Angular resolution 1.0 degrees"
        };

        /*
         * @brief Constructor of SICK LDMRS driver configuration.
         *        Initializes the configuration with default values.
         */
        SickLDMRSDriverConfig(rosNodePtr nh = 0);

        bool set_parameter(const std::string & name, const bool & value);
        bool set_parameter(const std::string & name, const int64_t & value);
        bool set_parameter(const std::string & name, const double & value);
        bool set_parameter(const std::string & name, const std::string & value);

        // ROS parameters
        std::string frame_id;        // gen.add("frame_id",              str_t,    0, "The TF frame in which point clouds will be returned.",                         "cloud")
        // Measurement parameters
        double start_angle;          // gen.add("start_angle",           double_t, 0, "The angle of the first range measurement [rad].",                              1600 * tics2rad, -1918 * tics2rad, 1600 * tics2rad)   # 50  deg
        double end_angle;            // gen.add("end_angle",             double_t, 0, "The angle of the last range measurement [rad].",                              -1920 * tics2rad, -1920 * tics2rad, 1598 * tics2rad)   # -60 deg
        int scan_frequency;          // gen.add("scan_frequency",        int_t,    0, "Scan frequency, 0 = 12.5Hz, 1 = 25 Hz, 2 = 50 Hz",                             0,                0,               2, edit_method=scan_freq_enum)
        double sync_angle_offset;    // gen.add("sync_angle_offset",     double_t, 0, "Angle under which the LD-MRS measures at the time of the sync pulse [rad].",   0,               -5760 * tics2rad, 5759 * tics2rad)   # -180...179.96 deg
        int angular_resolution_type; // gen.add("angular_resolution_type", int_t,  0, "Angular resolution type: 0 = focused, 1 = constant, 2 = flexible",             1,                0,               2, edit_method=angular_res_enum)
        int layer_range_reduction;   // gen.add("layer_range_reduction", int_t,    0, "0: Full range, 1: lower 4 reduced, 2: upper 4 reduced, 3: all reduced",        0,                0,               3, edit_method=range_reduction_enum)
        bool ignore_near_range;      // gen.add("ignore_near_range",     bool_t,   0, "Ignore scan points up to 15m. Requires layer_range_reduction = lower 4 reduced.", False)
        bool sensitivity_control;    // gen.add("sensitivity_control",   bool_t,   0, "Reduce the sensitivity automatically in case of extraneous light.",            False)

        // FlexRes parameters
        double flexres_start_angle1; // gen.add("flexres_start_angle1",  double_t, 0, "FlexRes: start angle of sector 1.",                                            1600 * tics2rad, -1918 * tics2rad, 1600 * tics2rad)
        double flexres_start_angle2; // gen.add("flexres_start_angle2",  double_t, 0, "FlexRes: start angle of sector 2.",                                            1120 * tics2rad, -1918 * tics2rad, 1600 * tics2rad)
        double flexres_start_angle3; // gen.add("flexres_start_angle3",  double_t, 0, "FlexRes: start angle of sector 3.",                                             960 * tics2rad, -1918 * tics2rad, 1600 * tics2rad)
        double flexres_start_angle4; // gen.add("flexres_start_angle4",  double_t, 0, "FlexRes: start angle of sector 4.",                                             640 * tics2rad, -1918 * tics2rad, 1600 * tics2rad)
        double flexres_start_angle5; // gen.add("flexres_start_angle5",  double_t, 0, "FlexRes: start angle of sector 5.",                                               0 * tics2rad, -1918 * tics2rad, 1600 * tics2rad)
        double flexres_start_angle6; // gen.add("flexres_start_angle6",  double_t, 0, "FlexRes: start angle of sector 6.",                                            -640 * tics2rad, -1918 * tics2rad, 1600 * tics2rad)
        double flexres_start_angle7; // gen.add("flexres_start_angle7",  double_t, 0, "FlexRes: start angle of sector 7.",                                            -960 * tics2rad, -1918 * tics2rad, 1600 * tics2rad)
        double flexres_start_angle8; // gen.add("flexres_start_angle8",  double_t, 0, "FlexRes: start angle of sector 8.",                                           -1280 * tics2rad, -1918 * tics2rad, 1600 * tics2rad)
        int flexres_resolution1;     // gen.add("flexres_resolution1",   int_t,    0, "FlexRes: angular resolution of sector 1.",                                     32,               4,               32, edit_method=resolution_enum)
        int flexres_resolution2;     // gen.add("flexres_resolution2",   int_t,    0, "FlexRes: angular resolution of sector 2.",                                     16,               4,               32, edit_method=resolution_enum)
        int flexres_resolution3;     // gen.add("flexres_resolution3",   int_t,    0, "FlexRes: angular resolution of sector 3.",                                      8,               4,               32, edit_method=resolution_enum)
        int flexres_resolution4;     // gen.add("flexres_resolution4",   int_t,    0, "FlexRes: angular resolution of sector 4.",                                      4,               4,               32, edit_method=resolution_enum)
        int flexres_resolution5;     // gen.add("flexres_resolution5",   int_t,    0, "FlexRes: angular resolution of sector 5.",                                      8,               4,               32, edit_method=resolution_enum)
        int flexres_resolution6;     // gen.add("flexres_resolution6",   int_t,    0, "FlexRes: angular resolution of sector 6.",                                     16,               4,               32, edit_method=resolution_enum)
        int flexres_resolution7;     // gen.add("flexres_resolution7",   int_t,    0, "FlexRes: angular resolution of sector 7.",                                     32,               4,               32, edit_method=resolution_enum)
        int flexres_resolution8;     // gen.add("flexres_resolution8",   int_t,    0, "FlexRes: angular resolution of sector 8.",                                     16,               4,               32, edit_method=resolution_enum)
        // Object tracking parameters
        int contour_point_density;   // gen.add("contour_point_density", int_t,    0, "Contour point density, 0: closest point only, 1: low density, 2: high density", 2,               0,               2, edit_method=contour_enum)
        int min_object_age;          // gen.add("min_object_age",        int_t,    0, "Minimum tracking age (number of scans) of an object to be transmitted.",        0,               0,               65535)
        int max_prediction_age;      // gen.add("max_prediction_age",    int_t,    0, "Maximum prediction age (number of scans) of an object to be transmitted.",      0,               0,               65535)

    }; // class SickLDMRSDriverConfig

} // namespace sick_ldmrs_driver

#endif // __SICK_LDMRS_CONFIG_H__
