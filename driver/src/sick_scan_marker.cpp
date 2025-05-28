/*
 * @brief Implementation of object markers for sick_scan
 *
 * Copyright (C) 2021, Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2021, SICK AG, Waldkirch
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
 *  Created on: 13.01.2021
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 */
#include <sick_scan/sick_scan_common_nw.h>
#include <sick_scan/sick_scan_common.h>
#include <sick_scan/sick_generic_parser.h>

#include "sick_scan/sick_scan_marker.h"

#ifdef ROSSIMU
#include <sick_scan/pointcloud_utils.h>
#endif

static ros_std_msgs::ColorRGBA color(float r, float g, float b, float a = 0.5f)
{
    ros_std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

static ros_std_msgs::ColorRGBA red(void)
{
    return color(1.0f, 0.0f, 0.0f);
}

static ros_std_msgs::ColorRGBA green(void) // free fields
{
    return color(0.0f, 1.0f, 0.0f);
}

static ros_std_msgs::ColorRGBA blue(void)
{
    return color(0.0f, 0.0f, 1.0f);
}

static ros_std_msgs::ColorRGBA yellow(void) // infringed fields
{
    return color(1.0f, 1.0f, 0.0f);
}

static ros_std_msgs::ColorRGBA gray(void) // invalid fields (default)
{
    return color(0.5f, 0.5f, 0.5f);
}

sick_scan_xd::SickScanMarker::SickScanMarker(rosNodePtr nh, const std::string & marker_topic, const std::string & marker_frame_id)
: m_nh(nh), m_scan_mon_fieldset(1), m_marker_output_legend_offset_x(-0.5)
{
    if(nh)
    {
        m_frame_id = marker_frame_id.empty() ? "/cloud" : marker_frame_id;
        std::string vis_marker_topic = marker_topic.empty() ? "sick_scan/marker" : marker_topic;
        m_marker_publisher = rosAdvertise<ros_visualization_msgs::MarkerArray>(nh, vis_marker_topic, 1);
        sick_scan_xd::setVisualizationMarkerTopic(vis_marker_topic);
        m_add_transform_xyz_rpy = sick_scan_xd::SickCloudTransform(nh, true);
    }
}

sick_scan_xd::SickScanMarker::~SickScanMarker()
{
}

void sick_scan_xd::SickScanMarker::updateMarker(const std::vector<SickScanMonField>& fields, int fieldset, int _eval_field_logic)
{
    sick_scan_xd::EVAL_FIELD_SUPPORT eval_field_logic = (sick_scan_xd::EVAL_FIELD_SUPPORT)_eval_field_logic;
    m_scan_mon_fields = fields;
    if(eval_field_logic == USE_EVAL_FIELD_TIM7XX_LOGIC)
    {
        m_scan_mon_fieldset = fieldset;
        std::vector<FieldInfo> default_fields = {FieldInfo(0,0,"-","3",gray()), FieldInfo(1,0,"-","2",gray()), FieldInfo(2,0,"-","1",gray())};
        m_scan_mon_field_marker = createMonFieldMarker(default_fields);
        m_scan_mon_field_legend = createMonFieldLegend(default_fields);
    }
    // if(eval_field_logic != USE_EVAL_FIELD_LMS5XX_LOGIC)
    //     m_scan_fieldset_legend = createMonFieldsetLegend(0);
    // m_scan_outputstate_legend = createOutputStateLegend({"0", "0", "0"}, {"-", "-", "-"}, {gray(), gray(), gray()}); // only if outputstates active, i.e. after updateMarker(LIDoutputstateMsg)
    publishMarker();
}

void sick_scan_xd::SickScanMarker::updateMarker(sick_scan_msg::LIDinputstateMsg& msg, int _eval_field_logic)
{
    sick_scan_xd::EVAL_FIELD_SUPPORT eval_field_logic = (sick_scan_xd::EVAL_FIELD_SUPPORT)_eval_field_logic;
    SickScanFieldMonSingleton *fieldMon = SickScanFieldMonSingleton::getInstance();
    if(fieldMon && eval_field_logic == USE_EVAL_FIELD_TIM7XX_LOGIC)
    {
        ROS_DEBUG_STREAM("SickScanMarker: active_fieldset = " << fieldMon->getActiveFieldset());
        m_scan_mon_fieldset = fieldMon->getActiveFieldset();
        m_scan_fieldset_legend = createMonFieldsetLegend(m_scan_mon_fieldset);
        publishMarker();
    }
}

void sick_scan_xd::SickScanMarker::updateMarker(sick_scan_msg::LIDoutputstateMsg& msg, int _eval_field_logic)
{
    sick_scan_xd::EVAL_FIELD_SUPPORT eval_field_logic = (sick_scan_xd::EVAL_FIELD_SUPPORT)_eval_field_logic;
    SickScanFieldMonSingleton *fieldMon = SickScanFieldMonSingleton::getInstance();
    if(fieldMon && eval_field_logic == USE_EVAL_FIELD_TIM7XX_LOGIC)
    {
        m_scan_mon_fieldset = fieldMon->getActiveFieldset();
        ROS_DEBUG_STREAM("SickScanMarker: active_fieldset = " << fieldMon->getActiveFieldset());
    }
    int num_devices = (int)std::min<size_t>(msg.output_count.size(), msg.output_state.size());
    std::vector<std::string> output_state(num_devices);
    std::vector<std::string> output_count(num_devices);
    std::vector<ros_std_msgs::ColorRGBA> output_colors(num_devices);
    for(int field_idx = 0; field_idx < num_devices; field_idx++)
    {
        int count = msg.output_count[field_idx];
        int state = msg.output_state[field_idx];
        output_state[field_idx] = std::to_string(state);
        output_count[field_idx] = std::to_string(count);
        if(state == 1) // 1 = active = yellow
        {
            output_state[field_idx] = "[ON]";
            output_colors[field_idx] = yellow();
        }
        else // 0 = not active = gray or 2 = not used = gray
        {
            output_state[field_idx] = "[OFF]";
            output_colors[field_idx] = gray();
        }
    }
    std::stringstream dbg_info;
    dbg_info << "SickScanMarker::updateMarker(): LIDoutputstateMsg (state,count) = { ";
    for(int field_idx = 0; field_idx < num_devices; field_idx++)
        dbg_info << ((field_idx > 0) ? ", (" : "(") << output_state[field_idx] << "," << output_count[field_idx] << ")";
    dbg_info << " }";
    ROS_DEBUG_STREAM(dbg_info.str());
    if(eval_field_logic == USE_EVAL_FIELD_TIM7XX_LOGIC)
        m_scan_fieldset_legend = createMonFieldsetLegend(m_scan_mon_fieldset);
    m_scan_outputstate_legend = createOutputStateLegend(output_state, output_count, output_colors);
    publishMarker();
}

void sick_scan_xd::SickScanMarker::updateMarker(sick_scan_msg::LFErecMsg& msg, int _eval_field_logic)
{
    sick_scan_xd::EVAL_FIELD_SUPPORT eval_field_logic = (sick_scan_xd::EVAL_FIELD_SUPPORT)_eval_field_logic;
    SickScanFieldMonSingleton *fieldMon = SickScanFieldMonSingleton::getInstance();
    if(fieldMon && eval_field_logic == USE_EVAL_FIELD_TIM7XX_LOGIC)
    {
        m_scan_mon_fieldset = fieldMon->getActiveFieldset();
        ROS_DEBUG_STREAM("SickScanMarker: active_fieldset = " << fieldMon->getActiveFieldset());
    }
    std::vector<FieldInfo> field_info(msg.fields.size());
    for(int field_idx = 0; field_idx < msg.fields.size(); field_idx++)
    {
        // LFErec: field_index runs from 1 to 3, field_info: field_index_scan_mon runs from 0 to 47 (field_index of m_scan_mon_fields)
        assert(m_scan_mon_fieldset >= 1); // active fieldset assuming m_scan_mon_fieldset >= 1 (default: 1)
        field_info[field_idx].field_index_scan_mon = (int)(msg.fields[field_idx].field_index - 1 + msg.fields.size() * (m_scan_mon_fieldset - 1));
        field_info[field_idx].field_result = msg.fields[field_idx].field_result_mrs;
        if(field_info[field_idx].field_result == 1) // 1 = free/clear = green
        {
            field_info[field_idx].field_status = "Clear";
            field_info[field_idx].field_color = green();
        }
        else if(field_info[field_idx].field_result == 2) // 2 = infringed = yellow
        {
            field_info[field_idx].field_status = "Infringed";
            field_info[field_idx].field_color = yellow();
        }
        else // 0 = invalid = gray
        {
            field_info[field_idx].field_status = "Incorrect";
            field_info[field_idx].field_color = gray();
        }
        if(eval_field_logic == USE_EVAL_FIELD_TIM7XX_LOGIC)
            field_info[field_idx].field_name = std::to_string(field_info.size() - field_idx); // field_info[field_info_idx].field_index;
        else
            field_info[field_idx].field_name = std::to_string(msg.fields[field_idx].field_index);
    }
    std::stringstream dbg_info;
    dbg_info << "SickScanMarker::updateMarker(): LFErec states={";
    for(int field_idx = 0; field_idx < msg.fields.size(); field_idx++)
        dbg_info << ((field_idx > 0) ? "," : "") << (int)msg.fields[field_idx].field_index << ":" << (int)msg.fields[field_idx].field_result_mrs;
    dbg_info << "}, mon_field_point_cnt={";
    for(int field_idx = 0; field_idx < m_scan_mon_fields.size(); field_idx++)
        dbg_info << ((field_idx > 0) ? "," : "") << m_scan_mon_fields[field_idx].getPointCount();
    dbg_info << "}, mon_field_set = " << m_scan_mon_fieldset;
    ROS_DEBUG_STREAM(dbg_info.str());
    m_scan_mon_field_marker = createMonFieldMarker(field_info);
    m_scan_mon_field_legend = createMonFieldLegend(field_info);
    if(eval_field_logic == USE_EVAL_FIELD_TIM7XX_LOGIC)
        m_scan_fieldset_legend = createMonFieldsetLegend(m_scan_mon_fieldset);
    publishMarker();
}

void sick_scan_xd::SickScanMarker::publishMarker(void)
{
    ros_visualization_msgs::MarkerArray marker_array;
    marker_array.markers.reserve(m_scan_mon_field_marker.size() + m_scan_mon_field_legend.size() + m_scan_outputstate_legend.size());
    for(int n = 0; n < m_scan_mon_field_marker.size(); n++)
        marker_array.markers.push_back(m_scan_mon_field_marker[n]);
    for(int n = 0; n < m_scan_mon_field_legend.size(); n++)
        marker_array.markers.push_back(m_scan_mon_field_legend[n]);
    for(int n = 0; n < m_scan_outputstate_legend.size(); n++)
        marker_array.markers.push_back(m_scan_outputstate_legend[n]);
    for(int n = 0; n < m_scan_fieldset_legend.size(); n++)
        marker_array.markers.push_back(m_scan_fieldset_legend[n]);
    notifyVisualizationMarkerListener(m_nh, &marker_array);
    rosPublish(m_marker_publisher, marker_array);
#ifdef ROSSIMU
    setVisualizationMarkerArray(marker_array.markers); // update ros simu output image
#endif
}

static void appendTrianglePoints(int point_count, const std::vector<float>& points_x, const std::vector<float>& points_y,
    ros_visualization_msgs::Marker& marker_point, int& triangle_idx, int nr_triangles, ros_std_msgs::ColorRGBA field_color)
{
    for(int point_idx = 2; point_idx < point_count && triangle_idx < nr_triangles; point_idx++, triangle_idx++)
    {

        marker_point.points[3 * triangle_idx + 0].x = points_x[0];
        marker_point.points[3 * triangle_idx + 0].y = points_y[0];
        marker_point.points[3 * triangle_idx + 0].z = 0;

        marker_point.points[3 * triangle_idx + 1].x = points_x[point_idx - 1];
        marker_point.points[3 * triangle_idx + 1].y = points_y[point_idx - 1];
        marker_point.points[3 * triangle_idx + 1].z = 0;

        marker_point.points[3 * triangle_idx + 2].x = points_x[point_idx];
        marker_point.points[3 * triangle_idx + 2].y = points_y[point_idx];
        marker_point.points[3 * triangle_idx + 2].z = 0;

        marker_point.colors[3 * triangle_idx + 0] = field_color;
        marker_point.colors[3 * triangle_idx + 1] = field_color;
        marker_point.colors[3 * triangle_idx + 2] = field_color;
    }
}

std::vector<ros_visualization_msgs::Marker> sick_scan_xd::SickScanMarker::createMonFieldMarker(const std::vector<FieldInfo>& field_info)
{
    int nr_triangles = 0;
    for(int field_info_idx = 0; field_info_idx < field_info.size(); field_info_idx++)
    {
        int field_idx = field_info[field_info_idx].field_index_scan_mon;
        assert(field_idx >= 0 && field_idx < m_scan_mon_fields.size());
        const sick_scan_xd::SickScanMonField& mon_field = m_scan_mon_fields[field_idx];
        SickScanMonFieldType field_typ = mon_field.fieldType();
        if(field_typ == MON_FIELD_DYNAMIC) // dynamic fields have two rectangle (first rectangle for v = max, second rectangle for v = 0)
            nr_triangles += 2 * std::max<int>(0, mon_field.getPointCount()/2 - 2); // 3 points: 1 triangle, 4 points: 3 triangles, and so on
        else
            nr_triangles += std::max<int>(0, mon_field.getPointCount() - 2); // 3 points: 1 triangle, 4 points: 3 triangles, and so on
        // std::map<SickScanMonFieldType,std::string> field_type_str = { {MON_FIELD_RADIAL, "MON_FIELD_RADIAL"}, {MON_FIELD_RECTANGLE, "MON_FIELD_RECTANGLE"}, {MON_FIELD_SEGMENTED, "MON_FIELD_SEGMENTED"}, {MON_FIELD_DYNAMIC, "MON_FIELD_DYNAMIC"} };
        // ROS_INFO_STREAM("sick_scan_xd::SickScanMarker::createMonFieldMarker(): field[" << field_info_idx << "]: type=" << field_type_str[field_typ] << ", " << (mon_field.getPointCount()) << " points");
    }

    // Draw fields using marker triangles
    ros_visualization_msgs::Marker marker_point;
    marker_point.header.stamp = rosTimeNow();
    marker_point.header.frame_id = m_frame_id;
    marker_point.ns = "sick_scan";
    marker_point.id = 1;
    marker_point.type = ros_visualization_msgs::Marker::TRIANGLE_LIST;
    marker_point.scale.x = 1;
    marker_point.scale.y = 1;
    marker_point.scale.z = 1;
    marker_point.pose.position.x = 0.0;
    marker_point.pose.position.y = 0.0;
    marker_point.pose.position.z = 0.0;
    marker_point.pose.orientation.x = 0.0;
    marker_point.pose.orientation.y = 0.0;
    marker_point.pose.orientation.z = 0.0;
    marker_point.pose.orientation.w = 1.0;
    marker_point.action = ros_visualization_msgs::Marker::ADD; // note: ADD == MODIFY
    marker_point.color = gray();
    marker_point.lifetime = rosDurationFromSec(0); // lifetime 0 indicates forever

    marker_point.points.resize(3 * nr_triangles);
    marker_point.colors.resize(3 * nr_triangles);
    std::vector<ros_geometry_msgs::Point> triangle_centroids;
    triangle_centroids.reserve(field_info.size());
    for(int field_info_idx = 0, triangle_idx = 0; field_info_idx < field_info.size() && triangle_idx < nr_triangles; field_info_idx++)
    {
        int field_idx = field_info[field_info_idx].field_index_scan_mon;
        assert(field_idx >= 0 && field_idx < m_scan_mon_fields.size());
        ros_std_msgs::ColorRGBA field_color = field_info[field_info_idx].field_color;
        const sick_scan_xd::SickScanMonField& mon_field = m_scan_mon_fields[field_idx];
        int point_count = mon_field.getPointCount();
        const std::vector<float>& points_x = mon_field.getFieldPointsX();
        const std::vector<float>& points_y = mon_field.getFieldPointsY();
        SickScanMonFieldType field_typ = mon_field.fieldType();
        if(field_typ == MON_FIELD_DYNAMIC) // dynamic fields have two rectangle (first rectangle for v = max, second rectangle for v = 0)
        {
            std::vector<float> field1_points_x(point_count/2), field1_points_y(point_count/2), field2_points_x(point_count/2), field2_points_y(point_count/2);
            for(int n = 0; n < point_count/2; n++)
            {
                field1_points_x[n] = points_x[n];
                field1_points_y[n] = points_y[n];
                field2_points_x[n] = points_x[n + point_count/2];
                field2_points_y[n] = points_y[n + point_count/2];
            }
            ros_std_msgs::ColorRGBA field1_color = field_color, field2_color = field_color;
            field1_color.r *= 0.5;
            field1_color.g *= 0.5;
            field1_color.b *= 0.5;
            appendTrianglePoints(point_count/2, field1_points_x, field1_points_y, marker_point, triangle_idx, nr_triangles, field1_color);
            appendTrianglePoints(point_count/2, field2_points_x, field2_points_y, marker_point, triangle_idx, nr_triangles, field2_color);
        }
        else
        {
            appendTrianglePoints(point_count, points_x, points_y, marker_point, triangle_idx, nr_triangles, field_color);
        }
    }
    // Apply an additional transform to the cartesian pointcloud, default: "0,0,0,0,0,0" (i.e. no transform)
    for(int n = 0; n < marker_point.points.size(); n++)
    {
		m_add_transform_xyz_rpy.applyTransform(marker_point.points[n].x, marker_point.points[n].y, marker_point.points[n].z);
    }

    std::vector<ros_visualization_msgs::Marker> marker_array;
    marker_array.reserve(1 + field_info.size());
    marker_array.push_back(marker_point);

    // Draw field names
    for(int field_info_idx = 0; field_info_idx < field_info.size(); field_info_idx++)
    {
        int field_idx = field_info[field_info_idx].field_index_scan_mon;
        assert(field_idx >= 0 && field_idx < m_scan_mon_fields.size());
        const sick_scan_xd::SickScanMonField& mon_field = m_scan_mon_fields[field_idx];
        if(mon_field.getPointCount() >= 3)
        {
            ros_geometry_msgs::Point triangle_centroid;
            triangle_centroid.x = 0;
            triangle_centroid.y = 0;
            triangle_centroid.z = 0;
            const std::vector<float>& points_x = mon_field.getFieldPointsX();
            const std::vector<float>& points_y = mon_field.getFieldPointsY();
            for(int point_idx = 0; point_idx < mon_field.getPointCount(); point_idx++)
            {
                triangle_centroid.x += points_x[point_idx];
                triangle_centroid.y += points_y[point_idx];
            }
            triangle_centroid.x /= (float)(mon_field.getPointCount());
            triangle_centroid.y /= (float)(mon_field.getPointCount());
            ros_visualization_msgs::Marker marker_field_name;
            marker_field_name.header.stamp = rosTimeNow();
            marker_field_name.header.frame_id = m_frame_id;
            marker_field_name.ns = "sick_scan";
            marker_field_name.id = 2 + field_info_idx;
            marker_field_name.type = ros_visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker_field_name.scale.z = 0.1;
            marker_field_name.pose.position.x = triangle_centroid.x;
            marker_field_name.pose.position.y = triangle_centroid.y;
            marker_field_name.pose.position.z = triangle_centroid.z;
            marker_field_name.pose.orientation.x = 0.0;
            marker_field_name.pose.orientation.y = 0.0;
            marker_field_name.pose.orientation.z = 0.0;
            marker_field_name.pose.orientation.w = 1.0;
            marker_field_name.action = ros_visualization_msgs::Marker::ADD; // note: ADD == MODIFY
            marker_field_name.color = field_info[field_info_idx].field_color;
            marker_field_name.color.a = 1;
            marker_field_name.lifetime = rosDurationFromSec(0); // lifetime 0 indicates forever
            marker_field_name.text = field_info[field_info_idx].field_name;
            marker_array.push_back(marker_field_name);
        }
        else
        {
            ros_visualization_msgs::Marker marker_field_name;
            marker_field_name.header.stamp = rosTimeNow();
            marker_field_name.header.frame_id = m_frame_id;
            marker_field_name.ns = "sick_scan";
            marker_field_name.id = 2 + field_info_idx;
#if defined(_WIN32) && defined(DELETE)
#           pragma push_macro("DELETE")
#           undef DELETE
            marker_field_name.action = ros_visualization_msgs::Marker::DELETE;
#           pragma pop_macro("DELETE")
#else
            marker_field_name.action = ros_visualization_msgs::Marker::DELETE;
#endif
            marker_field_name.lifetime = rosDurationFromSec(0); // lifetime 0 indicates forever
            marker_array.push_back(marker_field_name);
        }

    }

    return marker_array;
}

std::vector<ros_visualization_msgs::Marker> sick_scan_xd::SickScanMarker::createMonFieldLegend(const std::vector<FieldInfo>& field_info)
{
    std::vector<ros_visualization_msgs::Marker> marker_array;
    marker_array.reserve(2 * field_info.size());
    for(int loop_cnt = 0; loop_cnt < 2; loop_cnt++)
    {
        for(int field_info_idx = 0, triangle_idx = 0; field_info_idx < field_info.size(); field_info_idx++)
        {
            ros_visualization_msgs::Marker marker_point;
            marker_point.header.stamp = rosTimeNow();
            marker_point.header.frame_id = m_frame_id;
            marker_point.ns = "sick_scan";
            marker_point.id = 100 + loop_cnt * (int)m_scan_mon_fields.size() + field_info_idx;
            marker_point.type = ros_visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker_point.scale.z = 0.1;
            marker_point.pose.position.x = -0.1 * field_info_idx - 0.1;
            marker_point.pose.position.y = ((loop_cnt == 0) ? 0.3 : -0.2);
            marker_point.pose.position.z = 0.0;
            marker_point.pose.orientation.x = 0.0;
            marker_point.pose.orientation.y = 0.0;
            marker_point.pose.orientation.z = 0.0;
            marker_point.pose.orientation.w = 1.0;
            marker_point.action = ros_visualization_msgs::Marker::ADD; // note: ADD == MODIFY
            marker_point.color = field_info[field_info_idx].field_color;
            marker_point.color.a = 1;
            marker_point.lifetime = rosDurationFromSec(0); // lifetime 0 indicates forever
            std::stringstream marker_text;
            // int detection_field_number = field_info.size() - field_info_idx; // field_info[field_info_idx].field_index;
            if (loop_cnt == 0)
                marker_text << "Detection field " << (field_info[field_info_idx].field_name) << " : ";
            else
                marker_text << field_info[field_info_idx].field_status;
            marker_point.text = marker_text.str();
            marker_array.push_back(marker_point);
            if(m_marker_output_legend_offset_x > marker_point.pose.position.x - 0.1)
                m_marker_output_legend_offset_x = marker_point.pose.position.x - 0.1;
        }
    }
    return marker_array;
}

std::vector<ros_visualization_msgs::Marker> sick_scan_xd::SickScanMarker::createMonFieldsetLegend(int fieldset)
{
    std::vector<ros_visualization_msgs::Marker> marker_array;
    marker_array.reserve(2);
    for(int loop_cnt = 0; loop_cnt < 2; loop_cnt++)
    {
        ros_visualization_msgs::Marker marker_point;
        marker_point.header.stamp = rosTimeNow();
        marker_point.header.frame_id = m_frame_id;
        marker_point.ns = "sick_scan";
        marker_point.id = 500 + loop_cnt;
        marker_point.type = ros_visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_point.scale.z = 0.1;
        marker_point.pose.position.x = -0.4;
        marker_point.pose.position.y = ((loop_cnt == 0) ? 0.16 : -0.2);
        marker_point.pose.position.z = 0.0;
        marker_point.pose.orientation.x = 0.0;
        marker_point.pose.orientation.y = 0.0;
        marker_point.pose.orientation.z = 0.0;
        marker_point.pose.orientation.w = 1.0;
        marker_point.action = ros_visualization_msgs::Marker::ADD; // note: ADD == MODIFY
        marker_point.color = green();
        marker_point.color.a = 1;
        marker_point.lifetime = rosDurationFromSec(0); // lifetime 0 indicates forever
        std::stringstream marker_text;
        if (loop_cnt == 0)
            marker_text << "Fieldset :";
        else
            marker_text << std::to_string(fieldset);
        marker_point.text = marker_text.str();
        marker_array.push_back(marker_point);
    }
   return marker_array;
}


std::vector<ros_visualization_msgs::Marker> sick_scan_xd::SickScanMarker::createOutputStateLegend(const std::vector<std::string>& output_state, const std::vector<std::string>& output_count, const std::vector<ros_std_msgs::ColorRGBA>& output_colors)
{
    std::vector<ros_visualization_msgs::Marker> marker_array;
    marker_array.reserve(2 * output_count.size());
    for(int loop_cnt = 0; loop_cnt < 2; loop_cnt++)
    {
        for(int field_idx = 0; field_idx < output_count.size(); field_idx++)
        {
            ros_visualization_msgs::Marker marker_point;
            marker_point.header.stamp = rosTimeNow();
            marker_point.header.frame_id = m_frame_id;
            marker_point.ns = "sick_scan";
            marker_point.id = 400 + loop_cnt * (int)output_count.size() + field_idx;
            marker_point.type = ros_visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker_point.scale.z = 0.1;
            marker_point.pose.position.x = -0.1 * field_idx + m_marker_output_legend_offset_x;
            marker_point.pose.position.y = ((loop_cnt == 0) ? 0.16 : -0.3);
            marker_point.pose.position.z = 0.0;
            marker_point.pose.orientation.x = 0.0;
            marker_point.pose.orientation.y = 0.0;
            marker_point.pose.orientation.z = 0.0;
            marker_point.pose.orientation.w = 1.0;
            marker_point.action = ros_visualization_msgs::Marker::ADD; // note: ADD == MODIFY
            marker_point.color = output_colors[field_idx];
            marker_point.color.a = 1;
            marker_point.lifetime = rosDurationFromSec(0); // lifetime 0 indicates forever
            std::stringstream marker_text;
            int output_device = field_idx + 1;
            if (loop_cnt == 0)
                marker_text << "Output " << output_device << " : ";
            else
                marker_text << (field_idx < output_state.size() ? (output_state[field_idx]) : "") << " Count:" << output_count[field_idx];
            marker_point.text = marker_text.str();
            marker_array.push_back(marker_point);
        }
    }
    return marker_array;
}
