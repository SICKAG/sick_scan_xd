#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
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

#ifndef SICK_SCAN_MARKER_H_
#define SICK_SCAN_MARKER_H_

#include <sick_scan/sick_ros_wrapper.h>
#include <sick_scan/sick_cloud_transform.h>
#include "sick_scan/sick_range_filter.h"
#include "sick_scan/sick_generic_field_mon.h"


namespace sick_scan_xd
{
  class SickScanMarker
  {
  public:

    SickScanMarker(rosNodePtr nh = 0, const std::string & marker_topic = "", const std::string & marker_frame_id = "");

    virtual ~SickScanMarker();

    void updateMarker(const std::vector<SickScanMonField>& fields, int fieldset, int eval_field_logic);

    void updateMarker(sick_scan_msg::LIDinputstateMsg& msg, int eval_field_logic);

    void updateMarker(sick_scan_msg::LIDoutputstateMsg& msg, int eval_field_logic);

    void updateMarker(sick_scan_msg::LFErecMsg& msg, int eval_field_logic);

  protected:

    class FieldInfo
    {
    public:
      FieldInfo(int idx=0, int result=0, const std::string& status="", const std::string& name="", const ros_std_msgs::ColorRGBA& color= ros_std_msgs::ColorRGBA())
      : field_index_scan_mon(idx), field_result(result), field_status(status), field_name(name), field_color(color) {}
      int field_index_scan_mon; // 0 to 47
      int field_result;// 0 = invalid = gray, 1 = free/clear = green, 2 = infringed = yellow
      std::string field_status; // field_result as string
      std::string field_name; // name within the field set ("1", "2" or "3")
      ros_std_msgs::ColorRGBA field_color; // field_result as color
    };

    void publishMarker(void);
    std::vector<ros_visualization_msgs::Marker> createMonFieldMarker(const std::vector<FieldInfo>& field_info);
    std::vector<ros_visualization_msgs::Marker> createMonFieldLegend(const std::vector<FieldInfo>& field_info);
    std::vector<ros_visualization_msgs::Marker> createMonFieldsetLegend(int fieldset);
    std::vector<ros_visualization_msgs::Marker> createOutputStateLegend(const std::vector<std::string>& output_state, const std::vector<std::string>& output_count, const std::vector<ros_std_msgs::ColorRGBA>& output_colors);

    rosNodePtr m_nh;
    std::string m_frame_id;
    rosPublisher<ros_visualization_msgs::MarkerArray> m_marker_publisher;
    int m_scan_mon_fieldset; // active fieldset assuming m_scan_mon_fieldset >= 1 (default: 1)
    std::vector<sick_scan_xd::SickScanMonField> m_scan_mon_fields;
    std::vector<ros_visualization_msgs::Marker> m_scan_mon_field_marker;
    std::vector<ros_visualization_msgs::Marker> m_scan_mon_field_legend;
    std::vector<ros_visualization_msgs::Marker> m_scan_fieldset_legend;
    std::vector<ros_visualization_msgs::Marker> m_scan_outputstate_legend;
    double m_marker_output_legend_offset_x;
    sick_scan_xd::SickCloudTransform m_add_transform_xyz_rpy; // Apply an additional transform to the cartesian pointcloud, default: "0,0,0,0,0,0" (i.e. no transform)

  }; /* class SickScanMarker */

} /* namespace sick_scan_xd */
#endif /* SICK_SCAN_MARKER_H_ */
