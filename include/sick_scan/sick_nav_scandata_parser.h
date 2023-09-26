#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * sick_nav_scandata_parser parses data telegrams from NAV-350.
 *
 * Copyright (C) 2023, Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2023, SICK AG, Waldkirch
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

#ifndef SICK_NAV_SCANDATA_PARSER_H_
#define SICK_NAV_SCANDATA_PARSER_H_

#include <string>
#include <vector>

#include <sick_scan/sick_ros_wrapper.h>
#include <sick_scan/sick_generic_parser.h>
#include <sick_scan/sick_nav_scandata.h>

namespace sick_scan_xd
{
    /** Parse binary NAV350 position data telegrams, i.e. parse NAV350 "sAN mNPOSGetData version errorCode wait mask poseData [x y phi optPoseData [outputMode timestamp meanDev navMode infoState quantUsedReflectors]]
    **  landmarkData [landmarkFilter reflectors [cart [X Y] polar [D Phi] optLandmarkData [optLandmarkData…]]] scanData [contentType scaleFactor scaleOffset startAngle angleRes data [aData]]"
    */
    bool parseNAV350BinaryPositionData(const uint8_t* receiveBuffer, int receiveBufferLength, NAV350mNPOSData& navdata);

    /** Parse binary NAV350 landmark data */
    bool parseNAV350BinaryLandmarkData(const uint8_t* receiveBuffer, int& receivePos, int receiveBufferLength, NAV350LandmarkData& landmarkData);

    /** Parse binary NAV350 landmark data received by response "sAN mNMAPDoMapping" after request "sMN mNMAPDoMapping" */
    bool parseNAV350BinaryLandmarkDataDoMappingResponse(const uint8_t* receiveBuffer, int receiveBufferLength, NAV350LandmarkDataDoMappingResponse& landmarkData);

    /** Creates and returns the sopas command "sMN mNLAYAddLandmark landmarkData {x y type subtype size layerID {ID}}" */
    std::vector<uint8_t> createNAV350BinaryAddLandmarkRequest(const NAV350LandmarkData& landmarkData, int nav_curr_layer);

    /** Creates and returns the sopas command "sMN mNLAYAddLandmark landmarkData {x y type subtype size layerID {ID}}" */
    std::vector<uint8_t> createNAV350BinaryAddLandmarkRequest(const std::vector<sick_scan_xd::NAV350ImkLandmark> landmarks);

    /** Creates and returns the sopas command "sMN mNPOSSetSpeed X Y Phi timestamp coordBase" */
    std::vector<uint8_t> createNAV350BinarySetSpeedRequest(const sick_scan_msg::NAVOdomVelocity& msg);
    
    /** Parse binary NAV350 position data telegrams, i.e. parse NAV350 "sAN mNPOSGetData ..." and converts the data to a ros_sensor_msgs::LaserScan message */
    bool parseNAV350BinaryPositionData(const uint8_t* receiveBuffer, int receiveBufferLength, short& elevAngleX200, double& elevationAngleInRad, rosTime& recvTimeStamp,
      bool config_sw_pll_only_publish, double config_time_offset, SickGenericParser* parser_, int& numEchos, ros_sensor_msgs::LaserScan & msg,
      sick_scan_msg::NAVPoseData& nav_pose_msg, sick_scan_msg::NAVLandmarkData& nav_landmark_msg, NAV350mNPOSData& navdata);

    /** Convert NAV350PoseData to ros transform */
    ros_geometry_msgs::TransformStamped convertNAVPoseDataToTransform(NAV350PoseData& poseData, rosTime recvTimeStamp, double config_time_offset,
      const std::string& tf_parent_frame_id, const std::string& tf_child_frame_id, SickGenericParser* parser_);

    /** Rotates a cartesian 2D position (x, y) by a given angle offset */
    void rotateXYbyAngleOffset(float& x, float& y, double angle_offset);
    
    /** Converts a cartesian 2D position from NAV to ROS coordinates */
    void convertNAVCartPos2DtoROSPos2D(int32_t nav_posx_mm, int32_t nav_posy_mm, float& ros_posx_m, float& ros_posy_m, double nav_angle_offset);

    /** Converts a cartesian 3D pose from NAV to ROS coordinates */
    void convertNAVCartPos3DtoROSPos3D(int32_t nav_posx_mm, int32_t nav_posy_mm, uint32_t nav_phi_mdeg, float& ros_posx_m, float& ros_posy_m, float& ros_yaw_rad, double nav_angle_offset);

    /** Converts NAV350 landmark data to a ROS VisualMarker message */
    ros_visualization_msgs::MarkerArray convertNAVLandmarkDataToMarker(const std::vector<NAV350ReflectorData>& reflectors, ros_std_msgs::Header& header, SickGenericParser* parser_);

    /** Import a NAV350 landmark layout from imk file. The NAV350 landmark layout can be stored in imk files using SOPAS ET. Each line in an imk file is formatted "globID x_mm y_mm type subtype size_mm layer1 layer2 layer3" */
    std::vector<sick_scan_xd::NAV350ImkLandmark> readNAVIMKfile(const std::string& nav_imk_file);

    /** Unittest for parseNAV350BinaryPositionData(): creates, serializes and deserializes NAV350 position data telegrams and checks the identity of results */
    bool parseNAV350BinaryUnittest();

} /* namespace sick_scan_xd */
#endif /* SICK_NAV_SCANDATA_PARSER_H_ */
