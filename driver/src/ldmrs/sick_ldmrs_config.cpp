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

#include "sick_scan/ldmrs/sick_ldmrs_config.hpp"

/*
 * @brief Constructor of SICK LDMRS driver configuration.
 *        Initializes the configuration with default values.
 */
sick_ldmrs_driver::SickLDMRSDriverConfig::SickLDMRSDriverConfig(rosNodePtr nh)
{
    if(nh)
    {
        // ROS parameters
        sick_ldmrs_driver::param<std::string>(nh, "frame_id", frame_id, "cloud");                            // gen.add("frame_id",              str_t,    0, "The TF frame in which point clouds will be returned.",                         "cloud")
        // Measurement parameters
        sick_ldmrs_driver::param<double>(nh, "start_angle",             start_angle,              0.872664); // gen.add("start_angle",           double_t, 0, "The angle of the first range measurement [rad].",                              1600 * tics2rad, -1918 * tics2rad, 1600 * tics2rad)   # 50  deg
        sick_ldmrs_driver::param<double>(nh, "end_angle",               end_angle,               -1.047197); // gen.add("end_angle",             double_t, 0, "The angle of the last range measurement [rad].",                              -1920 * tics2rad, -1920 * tics2rad, 1598 * tics2rad)   # -60 deg
        sick_ldmrs_driver::param<int>   (nh, "scan_frequency",          scan_frequency,           0       ); // gen.add("scan_frequency",        int_t,    0, "Scan frequency, 0 = 12.5Hz, 1 = 25 Hz, 2 = 50 Hz",                             0,                0,               2, edit_method=scan_freq_enum)
        sick_ldmrs_driver::param<double>(nh, "sync_angle_offset",       sync_angle_offset,        0       ); // gen.add("sync_angle_offset",     double_t, 0, "Angle under which the LD-MRS measures at the time of the sync pulse [rad].",   0,               -5760 * tics2rad, 5759 * tics2rad)   # -180...179.96 deg
        sick_ldmrs_driver::param<int>   (nh, "angular_resolution_type", angular_resolution_type,  1       ); // gen.add("angular_resolution_type", int_t,  0, "Angular resolution type: 0 = focused, 1 = constant, 2 = flexible",             1,                0,               2, edit_method=angular_res_enum)
        sick_ldmrs_driver::param<int>   (nh, "layer_range_reduction",   layer_range_reduction,    0       ); // gen.add("layer_range_reduction", int_t,    0, "0: Full range, 1: lower 4 reduced, 2: upper 4 reduced, 3: all reduced",        0,                0,               3, edit_method=range_reduction_enum)
        sick_ldmrs_driver::param<bool>  (nh, "ignore_near_range",       ignore_near_range,       false    ); // gen.add("ignore_near_range",     bool_t,   0, "Ignore scan points up to 15m. Requires layer_range_reduction = lower 4 reduced.", False)
        sick_ldmrs_driver::param<bool>  (nh, "sensitivity_control",     sensitivity_control,     false    ); // gen.add("sensitivity_control",   bool_t,   0, "Reduce the sensitivity automatically in case of extraneous light.",            False)
        // FlexRes parameters
        sick_ldmrs_driver::param<double>(nh, "flexres_start_angle1", flexres_start_angle1,  0.872664);       // gen.add("flexres_start_angle1",  double_t, 0, "FlexRes: start angle of sector 1.",                                            1600 * tics2rad, -1918 * tics2rad, 1600 * tics2rad)
        sick_ldmrs_driver::param<double>(nh, "flexres_start_angle2", flexres_start_angle2,  0.610865);       // gen.add("flexres_start_angle2",  double_t, 0, "FlexRes: start angle of sector 2.",                                            1120 * tics2rad, -1918 * tics2rad, 1600 * tics2rad)
        sick_ldmrs_driver::param<double>(nh, "flexres_start_angle3", flexres_start_angle3,  0.523598);       // gen.add("flexres_start_angle3",  double_t, 0, "FlexRes: start angle of sector 3.",                                             960 * tics2rad, -1918 * tics2rad, 1600 * tics2rad)
        sick_ldmrs_driver::param<double>(nh, "flexres_start_angle4", flexres_start_angle4,  0.349065);       // gen.add("flexres_start_angle4",  double_t, 0, "FlexRes: start angle of sector 4.",                                             640 * tics2rad, -1918 * tics2rad, 1600 * tics2rad)
        sick_ldmrs_driver::param<double>(nh, "flexres_start_angle5", flexres_start_angle5,  0.000000);       // gen.add("flexres_start_angle5",  double_t, 0, "FlexRes: start angle of sector 5.",                                               0 * tics2rad, -1918 * tics2rad, 1600 * tics2rad)
        sick_ldmrs_driver::param<double>(nh, "flexres_start_angle6", flexres_start_angle6, -0.349065);       // gen.add("flexres_start_angle6",  double_t, 0, "FlexRes: start angle of sector 6.",                                            -640 * tics2rad, -1918 * tics2rad, 1600 * tics2rad)
        sick_ldmrs_driver::param<double>(nh, "flexres_start_angle7", flexres_start_angle7, -0.523598);       // gen.add("flexres_start_angle7",  double_t, 0, "FlexRes: start angle of sector 7.",                                            -960 * tics2rad, -1918 * tics2rad, 1600 * tics2rad)
        sick_ldmrs_driver::param<double>(nh, "flexres_start_angle8", flexres_start_angle8, -0.698131);       // gen.add("flexres_start_angle8",  double_t, 0, "FlexRes: start angle of sector 8.",                                           -1280 * tics2rad, -1918 * tics2rad, 1600 * tics2rad)
        sick_ldmrs_driver::param<int>   (nh, "flexres_resolution1",  flexres_resolution1,  32       );       // gen.add("flexres_resolution1",   int_t,    0, "FlexRes: angular resolution of sector 1.",                                     32,               4,               32, edit_method=resolution_enum)
        sick_ldmrs_driver::param<int>   (nh, "flexres_resolution2",  flexres_resolution2,  16       );       // gen.add("flexres_resolution2",   int_t,    0, "FlexRes: angular resolution of sector 2.",                                     16,               4,               32, edit_method=resolution_enum)
        sick_ldmrs_driver::param<int>   (nh, "flexres_resolution3",  flexres_resolution3,   8       );       // gen.add("flexres_resolution3",   int_t,    0, "FlexRes: angular resolution of sector 3.",                                      8,               4,               32, edit_method=resolution_enum)
        sick_ldmrs_driver::param<int>   (nh, "flexres_resolution4",  flexres_resolution4,   4       );       // gen.add("flexres_resolution4",   int_t,    0, "FlexRes: angular resolution of sector 4.",                                      4,               4,               32, edit_method=resolution_enum)
        sick_ldmrs_driver::param<int>   (nh, "flexres_resolution5",  flexres_resolution5,   8       );       // gen.add("flexres_resolution5",   int_t,    0, "FlexRes: angular resolution of sector 5.",                                      8,               4,               32, edit_method=resolution_enum)
        sick_ldmrs_driver::param<int>   (nh, "flexres_resolution6",  flexres_resolution6,  16       );       // gen.add("flexres_resolution6",   int_t,    0, "FlexRes: angular resolution of sector 6.",                                     16,               4,               32, edit_method=resolution_enum)
        sick_ldmrs_driver::param<int>   (nh, "flexres_resolution7",  flexres_resolution7,  32       );       // gen.add("flexres_resolution7",   int_t,    0, "FlexRes: angular resolution of sector 7.",                                     32,               4,               32, edit_method=resolution_enum)
        sick_ldmrs_driver::param<int>   (nh, "flexres_resolution8",  flexres_resolution8,  16       );       // gen.add("flexres_resolution8",   int_t,    0, "FlexRes: angular resolution of sector 8.",                                     16,               4,               32, edit_method=resolution_enum)
        // Object tracking parameters
        sick_ldmrs_driver::param<int>(nh, "contour_point_density", contour_point_density, 2);                // gen.add("contour_point_density", int_t,    0, "Contour point density, 0: closest point only, 1: low density, 2: high density", 2,               0,               2, edit_method=contour_enum)
        sick_ldmrs_driver::param<int>(nh, "min_object_age",        min_object_age,        0);                // gen.add("min_object_age",        int_t,    0, "Minimum tracking age (number of scans) of an object to be transmitted.",        0,               0,               65535)
        sick_ldmrs_driver::param<int>(nh, "max_prediction_age",    max_prediction_age,    0);                // gen.add("max_prediction_age",    int_t,    0, "Maximum prediction age (number of scans) of an object to be transmitted.",      0,               0,               65535)
    }
}

#define SET_PARAMETER(param_name,parameter,name,value) if(name==param_name){ parameter = value; return true; }

bool sick_ldmrs_driver::SickLDMRSDriverConfig::set_parameter(const std::string & name, const bool & value)
{
    SET_PARAMETER("ignore_near_range", ignore_near_range, name, value);
    SET_PARAMETER("sensitivity_control", sensitivity_control, name, value);
    return false;
}

bool sick_ldmrs_driver::SickLDMRSDriverConfig::set_parameter(const std::string & name, const int64_t & value)
{
    SET_PARAMETER("angular_resolution_type", angular_resolution_type, name, value);
    SET_PARAMETER("layer_range_reduction", layer_range_reduction, name, value);
    SET_PARAMETER("flexres_resolution1", flexres_resolution1, name, value);
    SET_PARAMETER("flexres_resolution2", flexres_resolution2, name, value);
    SET_PARAMETER("flexres_resolution3", flexres_resolution3, name, value);
    SET_PARAMETER("flexres_resolution4", flexres_resolution4, name, value);
    SET_PARAMETER("flexres_resolution5", flexres_resolution5, name, value);
    SET_PARAMETER("flexres_resolution6", flexres_resolution6, name, value);
    SET_PARAMETER("flexres_resolution7", flexres_resolution7, name, value);
    SET_PARAMETER("flexres_resolution8", flexres_resolution8, name, value);
    SET_PARAMETER("contour_point_density", contour_point_density, name, value);
    SET_PARAMETER("min_object_age", min_object_age, name, value);
    SET_PARAMETER("max_prediction_age", max_prediction_age, name, value);
    return false;
}

bool sick_ldmrs_driver::SickLDMRSDriverConfig::set_parameter(const std::string & name, const double & value)
{
    SET_PARAMETER("start_angle", start_angle, name, value);
    SET_PARAMETER("end_angle", end_angle, name, value);
    SET_PARAMETER("sync_angle_offset", sync_angle_offset, name, value);
    SET_PARAMETER("flexres_start_angle1", flexres_start_angle1, name, value);
    SET_PARAMETER("flexres_start_angle2", flexres_start_angle2, name, value);
    SET_PARAMETER("flexres_start_angle3", flexres_start_angle3, name, value);
    SET_PARAMETER("flexres_start_angle4", flexres_start_angle4, name, value);
    SET_PARAMETER("flexres_start_angle5", flexres_start_angle5, name, value);
    SET_PARAMETER("flexres_start_angle6", flexres_start_angle6, name, value);
    SET_PARAMETER("flexres_start_angle7", flexres_start_angle7, name, value);
    SET_PARAMETER("flexres_start_angle8", flexres_start_angle8, name, value);
    return false;
}

bool sick_ldmrs_driver::SickLDMRSDriverConfig::set_parameter(const std::string & name, const std::string & value)
{
    SET_PARAMETER("frame_id", frame_id, name, value);
    return false;
}
