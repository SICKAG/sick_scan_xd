#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * sick_nav_scandata defines the data telegrams from NAV-350.
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

#ifndef SICK_NAV_SCANDATA_H_
#define SICK_NAV_SCANDATA_H_

#include <string>
#include <vector>

#include <sick_scan/sick_ros_wrapper.h>

namespace sick_scan_xd
{
    /** Container for NAV350 optional pose data */
    class NAV350OptPoseData
    {
    public:
      uint8_t outputMode = 0;
      uint32_t timestamp = 0;
      int32_t meanDev = 0;
      uint8_t navMode = 0;
      uint32_t infoState  = 0;
      uint8_t quantUsedReflectors = 0;
    };

    /** Container for NAV350 pose data */
    class NAV350PoseData
    {
    public:
      int32_t x = 0; // x-position in mm
      int32_t y = 0; // y-position in mm
      uint32_t phi = 0; // orientation in 0 ... 360000 mdeg
      uint16_t optPoseDataValid = 0;
      NAV350OptPoseData optPoseData;
    };

    /** Container for NAV350 Cartesian data */
    class NAV350CartesianData
    {
    public:
      int32_t x = 0;
      int32_t y = 0;
    };

    /** Container for NAV350 Polar data */
    class NAV350PolarData
    {
    public:
      uint32_t dist = 0;
      uint32_t phi = 0;
    };

    /** Container for NAV350 optional reflector data */
    class NAV350OptReflectorData
    {
    public:
      uint16_t localID = 0;
      uint16_t globalID = 0;
      uint8_t type = 0;
      uint16_t subType = 0;
      uint16_t quality = 0;
      uint32_t timestamp = 0;
      uint16_t size = 0;
      uint16_t hitCount = 0;
      uint16_t meanEcho = 0;
      uint16_t startIndex = 0;
      uint16_t endIndex = 0;
    };

    /** Container for NAV350 Reflector data */
    class NAV350ReflectorData
    {
    public:
      uint16_t cartesianDataValid = 0;
      NAV350CartesianData cartesianData;
      uint16_t polarDataValid = 0;
      NAV350PolarData polarData;
      uint16_t optReflectorDataValid = 0;
      NAV350OptReflectorData optReflectorData;
      std::string print()
      {
        std::stringstream s;
        s << "cartesianDataValid=" << (int)cartesianDataValid << ", cartesianData.x=" << cartesianData.x << ", cartesianData.y=" << cartesianData.y 
          << ", polarDataValid=" << (int)polarDataValid << ", polarData.dist=" << polarData.dist << ", polarData.phi=" << polarData.phi 
          << ", optReflectorDataValid=" << (int)optReflectorDataValid << ", localID=" << optReflectorData.localID  << ", globalID=" << optReflectorData.globalID  << ", type=" << (int)optReflectorData.type  << ", subType=" << (int)optReflectorData.subType
          << ", quality=" << optReflectorData.quality  << ", timestamp=" << optReflectorData.timestamp  << ", size=" << optReflectorData.size  << ", hitCount=" << optReflectorData.hitCount  << ", meanEcho=" << optReflectorData.meanEcho
          << ", startIndex=" << optReflectorData.startIndex  << ", endIndex=" << optReflectorData.endIndex;
        return s.str();
      }
    };

    /** Container for NAV350 Landmark data */
    class NAV350LandmarkData
    {
    public:
      uint8_t landmarkFilter = 0;
      uint16_t numReflectors = 0;
      std::vector<NAV350ReflectorData> reflectors;
    };

    /** Container for NAV350 landmark data received by response "sAN mNMAPDoMapping" after request "sMN mNMAPDoMapping" */
    class NAV350LandmarkDataDoMappingResponse
    {
    public:
      uint8_t errorCode = 0;
      uint16_t landmarkDataValid = 0;
      NAV350LandmarkData landmarkData;
      void print()
      {
        ROS_INFO_STREAM("NAV350LandmarkDataDoMappingResponse: errorCode=" << (int)errorCode << ", landmarkDataValid=" << (int)landmarkDataValid << ", landmarkFilter=" << (int)landmarkData.landmarkFilter << ", numReflectors=" << (int)landmarkData.numReflectors);
        for(int reflector_cnt = 0; reflector_cnt < landmarkData.reflectors.size(); reflector_cnt++)
          ROS_INFO_STREAM("NAV350LandmarkDataDoMappingResponse: reflector[" << reflector_cnt << "]={" << landmarkData.reflectors[reflector_cnt].print() << "}");
      }
    };

    /** Container for NAV350 scan data */
    class NAV350ScanData
    {
    public:
      std::string contentType = ""; // "DIST1" or "ANGL1"
      float scaleFactor = 0; // multiplier, always 1 for NAV350
      float scaleOffset = 0; // offset, always 0 for NAV350
      int32_t startAngle = 0; // 0 ... +360000 mdeg
      uint16_t angleRes = 0; // angular step, in 1/1000 deg, 250 mdeg (fix)
      uint32_t timestampStart = 0; // timestamp of scan start in ms
      uint16_t numData = 0; // number of scan data
      std::vector<uint32_t> data; // DIST in mm or ANGL in 1/10000 degree
    };

    /** Container for NAV350 remission data */
    class NAV350RemissionData
    {
    public:
      std::string contentType = ""; // "RSSI1"
      float scaleFactor = 0; // multiplier, always 1 for NAV350
      float scaleOffset = 0; // offset, always 0 for NAV350
      int32_t startAngle = 0; // 0 ... +360000 mdeg
      uint16_t angleRes = 0; // angular step, in 1/1000 deg, 250 mdeg (fix)
      uint32_t timestampStart = 0; // timestamp of scan start in ms
      uint16_t numData = 0; // number of scan data
      std::vector<uint16_t> data; // remission data
    };

    /** Container for NAV350 position data telegrams, i.e. content of NAV350 mNPOSGetData telegrams */
    class NAV350mNPOSData
    {
    public:
      uint16_t version = 0;
      uint8_t errorCode = 0;
      uint8_t wait = 0;
      uint8_t mask = 0;
      uint16_t poseDataValid = 0;
      NAV350PoseData poseData;
      uint16_t landmarkDataValid = 0;
      NAV350LandmarkData landmarkData;
      uint16_t scanDataValid = 0;
      std::vector<NAV350ScanData> scanData; // no scan data (scanDataValid = 0) or DIST1 scan data (scanDataValid = 1) or DIST1 + ANGL1 scan data (scanDataValid = 2)
      uint16_t remissionDataValid = 0;
      NAV350RemissionData remissionData;
      float angleOffset = (float)(-M_PI);
    };

    
    /** Container for NAV350 landmark data imported from imk file. Each line in an imk file is formatted "globID x_mm y_mm type subtype size_mm layer1 layer2 layer3" */
    class NAV350ImkLandmark
    {
    public:
      uint16_t globID = 0;
      int32_t x_mm;
      int32_t y_mm;
      uint8_t type = 0;
      uint16_t subType = 0;
      uint16_t size_mm;
      std::vector<uint16_t> layerID;
    };

} /* namespace sick_scan_xd */
#endif /* SICK_NAV_SCANDATA_H_ */
