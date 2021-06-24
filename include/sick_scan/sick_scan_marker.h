/*
 * @brief Implementation of object markers for sick_scan
 *
 * Copyright (C) 2021, Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2021, SICK AG, Waldkirch
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
 *     * Neither the name of Osnabrück University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

#ifndef SICK_SCAN_MARKER_H_
#define SICK_SCAN_MARKER_H_

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

#include "sick_scan/LFErecMsg.h"
#include "sick_scan/LIDoutputstateMsg.h"
#include "sick_scan/sick_generic_field_mon.h"


namespace sick_scan
{
  class SickScanMarker
  {
  public:

    SickScanMarker(ros::NodeHandle* nh = 0, const std::string & marker_topic = "", const std::string & marker_frame_id = "");

    virtual ~SickScanMarker();

    void updateMarker(const std::vector<SickScanMonField>& fields, int fieldset, int eval_field_logic);

    void updateMarker(sick_scan::LIDoutputstateMsg& msg, int eval_field_logic);

    void updateMarker(sick_scan::LFErecMsg& msg, int eval_field_logic);

  protected:

    class FieldInfo
    {
    public:
      FieldInfo(int idx=0, int result=0, const std::string& status="", const std::string& name="", const std_msgs::ColorRGBA& color=std_msgs::ColorRGBA())
      : field_index_scan_mon(idx), field_result(result), field_status(status), field_name(name), field_color(color) {}
      int field_index_scan_mon; // 0 to 47
      int field_result;// 0 = invalid = gray, 1 = free/clear = green, 2 = infringed = yellow
      std::string field_status; // field_result as string
      std::string field_name; // name within the field set ("1", "2" or "3")
      std_msgs::ColorRGBA field_color; // field_result as color
    };

    void publishMarker(void);
    std::vector<visualization_msgs::Marker> createMonFieldMarker(const std::vector<FieldInfo>& field_info);
    std::vector<visualization_msgs::Marker> createMonFieldLegend(const std::vector<FieldInfo>& field_info);
    std::vector<visualization_msgs::Marker> createMonFieldsetLegend(int fieldset);
    std::vector<visualization_msgs::Marker> createOutputStateLegend(const std::vector<std::string>& output_state, const std::vector<std::string>& output_count, const std::vector<std_msgs::ColorRGBA>& output_colors);

    std::string m_frame_id;
    ros::Publisher m_marker_publisher;
    int m_scan_mon_fieldset;
    std::vector<sick_scan::SickScanMonField> m_scan_mon_fields;
    std::vector<visualization_msgs::Marker> m_scan_mon_field_marker;
    std::vector<visualization_msgs::Marker> m_scan_mon_field_legend;
    std::vector<visualization_msgs::Marker> m_scan_fieldset_legend;
    std::vector<visualization_msgs::Marker> m_scan_outputstate_legend;
    float m_marker_output_legend_offset_x;

  }; /* class SickScanMarker */

} /* namespace sick_scan */
#endif /* SICK_SCAN_MARKER_H_ */
