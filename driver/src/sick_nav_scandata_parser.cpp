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
#include <fstream>
#include <string>
#include <sstream>

#include "softwarePLL.h"
#include <sick_scan/sick_scan_common.h>
#include <sick_scan/sick_lmd_scandata_parser.h>
#include <sick_scan/sick_nav_scandata_parser.h>

namespace sick_scan_xd
{
    template<typename T> static void appendToBuffer(std::vector<uint8_t>& data_buffer, const T& value)
    {
      T dst_value = value;
      swap_endian((unsigned char *) &dst_value, sizeof(dst_value));
      size_t pos = data_buffer.size();
      for (size_t n = 0; n < sizeof(dst_value); n++)
        data_buffer.push_back(0);
      memcpy(&data_buffer[pos], &dst_value, sizeof(dst_value));
    }

    template<typename T> static bool readFromBuffer(const uint8_t* receiveBuffer, int& pos, int receiveBufferLength, T& value, const char* file, int line)
    {
      if(pos + sizeof(value) <= receiveBufferLength)
      {
        memcpy(&value, receiveBuffer + pos, sizeof(value));
        swap_endian((unsigned char *) &value, sizeof(value));
        pos += sizeof(value);
        return true;
      }
      else
      {
        ROS_WARN_STREAM("readFromBuffer(): read pos = " << pos << " + sizeof(value) = " << sizeof(value) << " exceeds receiveBufferLength = " << receiveBufferLength << " (" << file << ":" << line << ")");
        return false;
      }
    }

    /** Serializes NAV350mNPOSData into a binary data buffer */
    static void writeNAV350BinaryPositionData(const NAV350mNPOSData& navdata, std::vector<uint8_t>& data_buffer)
    {
      data_buffer.clear();
      uint32_t stx = 0x02020202;
      uint32_t payload_size = 0;
      appendToBuffer(data_buffer, stx);
      appendToBuffer(data_buffer, payload_size);
      std::string sopas_cmd = "sAN mNPOSGetData ";
      std::copy(sopas_cmd.begin(), sopas_cmd.end(), std::back_inserter(data_buffer));
      appendToBuffer(data_buffer, navdata.version);
      appendToBuffer(data_buffer, navdata.errorCode);
      appendToBuffer(data_buffer, navdata.wait);
      appendToBuffer(data_buffer, navdata.mask);
      appendToBuffer(data_buffer, navdata.poseDataValid);
      if (navdata.poseDataValid > 0)
      {
        appendToBuffer(data_buffer, navdata.poseData.x);
        appendToBuffer(data_buffer, navdata.poseData.y);
        appendToBuffer(data_buffer, navdata.poseData.phi);
        appendToBuffer(data_buffer, navdata.poseData.optPoseDataValid);
        if (navdata.poseData.optPoseDataValid > 0)
        {
          appendToBuffer(data_buffer, navdata.poseData.optPoseData.outputMode);
          appendToBuffer(data_buffer, navdata.poseData.optPoseData.timestamp);
          appendToBuffer(data_buffer, navdata.poseData.optPoseData.meanDev);
          appendToBuffer(data_buffer, navdata.poseData.optPoseData.navMode);
          appendToBuffer(data_buffer, navdata.poseData.optPoseData.infoState);
          appendToBuffer(data_buffer, navdata.poseData.optPoseData.quantUsedReflectors);
        }
      }
      appendToBuffer(data_buffer, navdata.landmarkDataValid);
      if (navdata.landmarkDataValid > 0)
      {
        appendToBuffer(data_buffer, navdata.landmarkData.landmarkFilter);
        appendToBuffer(data_buffer, navdata.landmarkData.numReflectors);
        for(int reflectorCnt = 0; reflectorCnt < navdata.landmarkData.numReflectors; reflectorCnt++)
        {
          appendToBuffer(data_buffer, navdata.landmarkData.reflectors[reflectorCnt].cartesianDataValid);
          if (navdata.landmarkData.reflectors[reflectorCnt].cartesianDataValid > 0)
          {
            appendToBuffer(data_buffer, navdata.landmarkData.reflectors[reflectorCnt].cartesianData.x);
            appendToBuffer(data_buffer, navdata.landmarkData.reflectors[reflectorCnt].cartesianData.y);
          }
          appendToBuffer(data_buffer, navdata.landmarkData.reflectors[reflectorCnt].polarDataValid);
          if (navdata.landmarkData.reflectors[reflectorCnt].polarDataValid > 0)
          {
            appendToBuffer(data_buffer, navdata.landmarkData.reflectors[reflectorCnt].polarData.dist);
            appendToBuffer(data_buffer, navdata.landmarkData.reflectors[reflectorCnt].polarData.phi);
          }
          appendToBuffer(data_buffer, navdata.landmarkData.reflectors[reflectorCnt].optReflectorDataValid);
          if (navdata.landmarkData.reflectors[reflectorCnt].optReflectorDataValid > 0)
          {
            appendToBuffer(data_buffer, navdata.landmarkData.reflectors[reflectorCnt].optReflectorData.localID);
            appendToBuffer(data_buffer, navdata.landmarkData.reflectors[reflectorCnt].optReflectorData.globalID);
            appendToBuffer(data_buffer, navdata.landmarkData.reflectors[reflectorCnt].optReflectorData.type);
            appendToBuffer(data_buffer, navdata.landmarkData.reflectors[reflectorCnt].optReflectorData.subType);
            appendToBuffer(data_buffer, navdata.landmarkData.reflectors[reflectorCnt].optReflectorData.quality);
            appendToBuffer(data_buffer, navdata.landmarkData.reflectors[reflectorCnt].optReflectorData.timestamp);
            appendToBuffer(data_buffer, navdata.landmarkData.reflectors[reflectorCnt].optReflectorData.size);
            appendToBuffer(data_buffer, navdata.landmarkData.reflectors[reflectorCnt].optReflectorData.hitCount);
            appendToBuffer(data_buffer, navdata.landmarkData.reflectors[reflectorCnt].optReflectorData.meanEcho);
            appendToBuffer(data_buffer, navdata.landmarkData.reflectors[reflectorCnt].optReflectorData.startIndex);
            appendToBuffer(data_buffer, navdata.landmarkData.reflectors[reflectorCnt].optReflectorData.endIndex);
          }
        }
      }
      appendToBuffer(data_buffer, navdata.scanDataValid);
      for (int channel = 0; channel < navdata.scanDataValid; channel++)
      {
        std::copy(navdata.scanData[channel].contentType.begin(), navdata.scanData[channel].contentType.end(), std::back_inserter(data_buffer));
        appendToBuffer(data_buffer, navdata.scanData[channel].scaleFactor);
        appendToBuffer(data_buffer, navdata.scanData[channel].scaleOffset);
        appendToBuffer(data_buffer, navdata.scanData[channel].startAngle);
        appendToBuffer(data_buffer, navdata.scanData[channel].angleRes);
        appendToBuffer(data_buffer, navdata.scanData[channel].timestampStart);
        appendToBuffer(data_buffer, navdata.scanData[channel].numData);
        for(int data_cnt = 0; data_cnt < navdata.scanData[channel].numData; data_cnt++)
        {
          appendToBuffer(data_buffer, navdata.scanData[channel].data[data_cnt]);
        }
      }
      appendToBuffer(data_buffer, navdata.remissionDataValid);
      if (navdata.remissionDataValid > 0)
      {
        std::copy(navdata.remissionData.contentType.begin(), navdata.remissionData.contentType.end(), std::back_inserter(data_buffer));
        appendToBuffer(data_buffer, navdata.remissionData.scaleFactor);
        appendToBuffer(data_buffer, navdata.remissionData.scaleOffset);
        appendToBuffer(data_buffer, navdata.remissionData.startAngle);
        appendToBuffer(data_buffer, navdata.remissionData.angleRes);
        appendToBuffer(data_buffer, navdata.remissionData.timestampStart);
        appendToBuffer(data_buffer, navdata.remissionData.numData);
        for(int data_cnt = 0; data_cnt < navdata.remissionData.numData; data_cnt++)
        {
          appendToBuffer(data_buffer, navdata.remissionData.data[data_cnt]);
        }
      }
      // write payload size
      payload_size = data_buffer.size() - 8;
      std::vector<uint8_t> payload_buffer;
      appendToBuffer(payload_buffer, payload_size);
      memcpy(&data_buffer[4], &payload_buffer[0], payload_buffer.size());
      data_buffer.push_back(0); // dummy crc
    }

    /** Unittest for parseNAV350BinaryPositionData(): creates, serializes and deserializes NAV350 position data telegrams and checks the identity of results */
    bool parseNAV350BinaryUnittest()
    {
      NAV350mNPOSData navdata_src, navdata_dst;
      std::vector<uint8_t> data_buffer_src, data_buffer_dst;
      data_buffer_src.reserve(32*1024);
      data_buffer_dst.reserve(32*1024);
      // Create dummy NAV350 position data
      navdata_src.version = 1;
      navdata_src.errorCode = 0;
      navdata_src.wait = 1;
      navdata_src.mask = 2;
      navdata_src.poseDataValid = 1;
      navdata_src.poseData.x = +1234;
      navdata_src.poseData.y = -1234;
      navdata_src.poseData.phi = M_PI / 2;
      navdata_src.poseData.optPoseDataValid = 1;
      navdata_src.poseData.optPoseData. outputMode = 1;
      navdata_src.poseData.optPoseData. timestamp = 123456789;
      navdata_src.poseData.optPoseData. meanDev = 789;
      navdata_src.poseData.optPoseData. navMode = 3;
      navdata_src.poseData.optPoseData. infoState  = 1234;
      navdata_src.poseData.optPoseData. quantUsedReflectors = 5;
      navdata_src.landmarkDataValid = 1;
      navdata_src.landmarkData.landmarkFilter = 1;
      navdata_src.landmarkData.numReflectors = 25;
      navdata_src.landmarkData.reflectors.resize(navdata_src.landmarkData.numReflectors);
      for(int reflector_cnt = 0; reflector_cnt < navdata_src.landmarkData.numReflectors; reflector_cnt++)
      {
        navdata_src.landmarkData.reflectors[reflector_cnt].cartesianDataValid = 1;
        navdata_src.landmarkData.reflectors[reflector_cnt].cartesianData.x = 2 * reflector_cnt + 0;
        navdata_src.landmarkData.reflectors[reflector_cnt].cartesianData.y = 2 * reflector_cnt + 1;
        navdata_src.landmarkData.reflectors[reflector_cnt].polarDataValid = 1;
        navdata_src.landmarkData.reflectors[reflector_cnt].polarData.dist = 1.1415 * 2 * reflector_cnt;
        navdata_src.landmarkData.reflectors[reflector_cnt].polarData.phi = M_PI * reflector_cnt / navdata_src.landmarkData.numReflectors;
        navdata_src.landmarkData.reflectors[reflector_cnt].optReflectorDataValid = 1;
        navdata_src.landmarkData.reflectors[reflector_cnt].optReflectorData.localID = 10 * reflector_cnt + 0;
        navdata_src.landmarkData.reflectors[reflector_cnt].optReflectorData.globalID = 10 * reflector_cnt + 1;
        navdata_src.landmarkData.reflectors[reflector_cnt].optReflectorData.type = 1;
        navdata_src.landmarkData.reflectors[reflector_cnt].optReflectorData.subType = 2;
        navdata_src.landmarkData.reflectors[reflector_cnt].optReflectorData.quality = 10 * reflector_cnt + 2;
        navdata_src.landmarkData.reflectors[reflector_cnt].optReflectorData.timestamp = 10 * reflector_cnt + 3;
        navdata_src.landmarkData.reflectors[reflector_cnt].optReflectorData.size = 10 * reflector_cnt + 4;
        navdata_src.landmarkData.reflectors[reflector_cnt].optReflectorData.hitCount = 10 * reflector_cnt + 5;
        navdata_src.landmarkData.reflectors[reflector_cnt].optReflectorData.meanEcho = 10 * reflector_cnt + 6;
        navdata_src.landmarkData.reflectors[reflector_cnt].optReflectorData.startIndex = 10 * reflector_cnt + 7;
        navdata_src.landmarkData.reflectors[reflector_cnt].optReflectorData.endIndex = 10 * reflector_cnt + 8;
      }
      navdata_src.scanDataValid = 2;
      navdata_src.scanData.resize(navdata_src.scanDataValid);
      navdata_src.scanData[0].contentType = "DIST1";
      navdata_src.scanData[0].scaleFactor = 1;
      navdata_src.scanData[0].scaleOffset = 0;
      navdata_src.scanData[0].startAngle = 0;
      navdata_src.scanData[0].angleRes = 250;
      navdata_src.scanData[0].timestampStart = 123456789;
      navdata_src.scanData[0].numData = 1024;
      navdata_src.scanData[0].data.resize(navdata_src.scanData[0].numData);
      for(int data_cnt = 0; data_cnt < navdata_src.scanData[0].numData; data_cnt++)
        navdata_src.scanData[0].data[data_cnt] = data_cnt;
      navdata_src.scanData[1].contentType = "ANGL1";
      navdata_src.scanData[1].scaleFactor = 1;
      navdata_src.scanData[1].scaleOffset = 0;
      navdata_src.scanData[1].startAngle = 0;
      navdata_src.scanData[1].angleRes = 250;
      navdata_src.scanData[1].timestampStart = 123456789;
      navdata_src.scanData[1].numData = 1024;
      navdata_src.scanData[1].data.resize(navdata_src.scanData[1].numData);
      for(int data_cnt = 0; data_cnt < navdata_src.scanData[1].numData; data_cnt++)
        navdata_src.scanData[1].data[data_cnt] = data_cnt * navdata_src.scanData[1].angleRes;
      navdata_src.remissionDataValid = 1;
      navdata_src.remissionData.contentType = "RSSI1";
      navdata_src.remissionData.scaleFactor = 1;
      navdata_src.remissionData.scaleOffset = 0;
      navdata_src.remissionData.startAngle = 0;
      navdata_src.remissionData.angleRes = 250;
      navdata_src.remissionData.timestampStart = 123456789;
      navdata_src.remissionData.numData = 1024;
      navdata_src.remissionData.data.resize(navdata_src.remissionData.numData);
      for(int data_cnt = 0; data_cnt < navdata_src.remissionData.numData; data_cnt++)
        navdata_src.remissionData.data[data_cnt] = data_cnt;
      // Serialize to NAV350 position data to binary buffer
      writeNAV350BinaryPositionData(navdata_src, data_buffer_src);
       // Parse serialized to NAV350 position data
     if (!parseNAV350BinaryPositionData(data_buffer_src.data(), data_buffer_src.size(), navdata_dst))
      {
        ROS_ERROR_STREAM("## ERROR parseNAV350BinaryUnittest(): parseNAV350BinaryPositionData failed");
        return false;
      }
      // Re-serialize to NAV350 position data to binary buffer
      writeNAV350BinaryPositionData(navdata_dst, data_buffer_dst);
      // Result of serialization and deserialization must be identical
      if (data_buffer_src.size() != data_buffer_dst.size() || memcmp(data_buffer_src.data(), data_buffer_dst.data(), data_buffer_src.size()) != 0)
      {
        ROS_ERROR_STREAM("## ERROR parseNAV350BinaryUnittest(): parseNAV350BinaryPositionData failed");
        return false;
      }
      ROS_DEBUG_STREAM("parseNAV350BinaryUnittest() passed (" << data_buffer_src.size() << " byte datagram)");
      return true;
    }

    /** Parse binary NAV350 landmark data */
    bool parseNAV350BinaryLandmarkData(const uint8_t* receiveBuffer, int& receivePos, int receiveBufferLength, NAV350LandmarkData& landmarkData)
    {
      bool success = true;
      success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.landmarkFilter, __FILE__, __LINE__);
      success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.numReflectors, __FILE__, __LINE__);
      landmarkData.reflectors = std::vector<NAV350ReflectorData>(landmarkData.numReflectors);
      for(int reflectorCnt = 0; reflectorCnt < landmarkData.numReflectors; reflectorCnt++)
      {
        success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.reflectors[reflectorCnt].cartesianDataValid, __FILE__, __LINE__);
        if (landmarkData.reflectors[reflectorCnt].cartesianDataValid > 0)
        {
          success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.reflectors[reflectorCnt].cartesianData.x, __FILE__, __LINE__);
          success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.reflectors[reflectorCnt].cartesianData.y, __FILE__, __LINE__);
        }
        success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.reflectors[reflectorCnt].polarDataValid, __FILE__, __LINE__);
        if (landmarkData.reflectors[reflectorCnt].polarDataValid > 0)
        {
          success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.reflectors[reflectorCnt].polarData.dist, __FILE__, __LINE__);
          success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.reflectors[reflectorCnt].polarData.phi, __FILE__, __LINE__);
        }
        success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.reflectors[reflectorCnt].optReflectorDataValid, __FILE__, __LINE__);
        if (landmarkData.reflectors[reflectorCnt].optReflectorDataValid > 0)
        {
          success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.reflectors[reflectorCnt].optReflectorData.localID, __FILE__, __LINE__);
          success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.reflectors[reflectorCnt].optReflectorData.globalID, __FILE__, __LINE__);
          success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.reflectors[reflectorCnt].optReflectorData.type, __FILE__, __LINE__);
          success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.reflectors[reflectorCnt].optReflectorData.subType, __FILE__, __LINE__);
          success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.reflectors[reflectorCnt].optReflectorData.quality, __FILE__, __LINE__);
          success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.reflectors[reflectorCnt].optReflectorData.timestamp, __FILE__, __LINE__);
          success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.reflectors[reflectorCnt].optReflectorData.size, __FILE__, __LINE__);
          success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.reflectors[reflectorCnt].optReflectorData.hitCount, __FILE__, __LINE__);
          success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.reflectors[reflectorCnt].optReflectorData.meanEcho, __FILE__, __LINE__);
          success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.reflectors[reflectorCnt].optReflectorData.startIndex, __FILE__, __LINE__);
          success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.reflectors[reflectorCnt].optReflectorData.endIndex, __FILE__, __LINE__);
        }
      }
      return success;
    }


    /** Parse binary NAV350 position data telegrams, i.e. parse NAV350 "sAN mNPOSGetData version errorCode wait mask poseData [x y phi optPoseData [outputMode timestamp meanDev navMode infoState quantUsedReflectors]]
    **  landmarkData [landmarkFilter reflectors [cart [X Y] polar [D Phi] optLandmarkData [optLandmarkData…]]] scanData [contentType scaleFactor scaleOffset startAngle angleRes data [aData]]"
    */
    bool parseNAV350BinaryPositionData(const uint8_t* receiveBuffer, int receiveBufferLength, NAV350mNPOSData& navdata)
    {
      // Init return value
      navdata = NAV350mNPOSData();

      // Parse header
      if (receiveBuffer == 0 || receiveBufferLength < 17 + 9 || memcmp(receiveBuffer, "\x02\x02\x02\x02", 4) != 0)
      {
        ROS_ERROR_STREAM("## ERROR parseNAV350BinaryPositionData(): invalid telegram (" << __FILE__ << ":" << __LINE__ << ")");
        return false;
      }
      bool success = true;
      int receivePos = 4;
      uint32_t payload_size = 0;
      success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, payload_size, __FILE__, __LINE__);
      if (!success || (int)(payload_size) + 9 > receiveBufferLength)
      {
        ROS_ERROR_STREAM("## ERROR parseNAV350BinaryPositionData(): invalid telegram size (" << __FILE__ << ":" << __LINE__ << ")");
        return false;
      }

      // Parse "sAN mNPOSGetData "
      if (strncmp((const char*)receiveBuffer + receivePos, "sAN mNPOSGetData ", 17) != 0)
      {
        ROS_ERROR_STREAM("## ERROR parseNAV350BinaryPositionData(): telegram does not start with \"sAN mNPOSGetData \"");
        return false;
      }
      receivePos += 17;

      // Parse version errorCode wait mask poseData
      success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.version, __FILE__, __LINE__);
      success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.errorCode, __FILE__, __LINE__);
      success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.wait, __FILE__, __LINE__);
      success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.mask, __FILE__, __LINE__);
      success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.poseDataValid, __FILE__, __LINE__);
      if (navdata.errorCode)
      {
        std::map<uint8_t, std::string> errorCodeStr = { {0, "no error"}, {1, "wrong operating mode"}, {2, "asynchrony Method terminated"}, {3, "invalid data"}, {4, "no position available"}, {5, "timeout"}, {6, "method already active"}, {7, "general error"} };
        ROS_WARN_STREAM("## WARNING parseNAV350BinaryPositionData(): NAV350 mNPOSGetData errorCode = " << (int)navdata.errorCode << " (\"" << errorCodeStr[navdata.errorCode] << "\")");
      }

      // Parse poseData x y phi optPoseData
      if (navdata.poseDataValid > 0)
      {
        success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.poseData.x, __FILE__, __LINE__);
        success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.poseData.y, __FILE__, __LINE__);
        success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.poseData.phi, __FILE__, __LINE__);
        success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.poseData.optPoseDataValid, __FILE__, __LINE__);
        if (navdata.poseData.optPoseDataValid > 0)
        {
           success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.poseData.optPoseData.outputMode, __FILE__, __LINE__);
           success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.poseData.optPoseData.timestamp, __FILE__, __LINE__);
           success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.poseData.optPoseData.meanDev, __FILE__, __LINE__);
           success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.poseData.optPoseData.navMode, __FILE__, __LINE__);
           success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.poseData.optPoseData.infoState, __FILE__, __LINE__);
           success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.poseData.optPoseData.quantUsedReflectors, __FILE__, __LINE__);
        }
      }

      // Parse landmark data
      success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.landmarkDataValid, __FILE__, __LINE__);
      if (navdata.landmarkDataValid > 0)
      {
        success &= parseNAV350BinaryLandmarkData(receiveBuffer, receivePos, receiveBufferLength, navdata.landmarkData);
      }

      // Parse NAVScan data
      success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.scanDataValid, __FILE__, __LINE__);
      if (navdata.scanDataValid > 2)
          ROS_ERROR_STREAM("## ERROR parseNAV350BinaryPositionData(): \"ScanData\" = " << (int)navdata.scanDataValid << ", max. 2 channel DIST1 and ANGL1 expected");
      navdata.scanData.resize(navdata.scanDataValid);
      for (int channel = 0; channel < navdata.scanDataValid; channel++)
      {
        if (strncmp((const char*)receiveBuffer + receivePos, "DIST1", 5) != 0 && strncmp((const char*)receiveBuffer + receivePos, "ANGL1", 5) != 0)
          ROS_ERROR_STREAM("## ERROR parseNAV350BinaryPositionData(): \"DIST1\" resp. \"ANGL1\" not found");
        navdata.scanData[channel].contentType = std::string((const char*)receiveBuffer + receivePos, 5);
        receivePos += 5;
        success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.scanData[channel].scaleFactor, __FILE__, __LINE__);
        success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.scanData[channel].scaleOffset, __FILE__, __LINE__);
        success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.scanData[channel].startAngle, __FILE__, __LINE__);
        success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.scanData[channel].angleRes, __FILE__, __LINE__);
        if (std::abs(navdata.scanData[channel].scaleFactor - 1.0f) > FLT_EPSILON || std::abs(navdata.scanData[channel].scaleOffset - 0.0f) > FLT_EPSILON || navdata.scanData[channel].startAngle != 0 || navdata.scanData[channel].angleRes != 250)
        {
          ROS_WARN_STREAM("## WARNING parseNAV350BinaryPositionData(): unexpected scan data parameter: scaleFactor=" << navdata.scanData[channel].scaleFactor  << ", scaleOffset=" << navdata.scanData[channel].scaleOffset
            << ", startAngle=" << navdata.scanData[channel].startAngle << ", angleRes=" << navdata.scanData[channel].angleRes);
        }
        success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.scanData[channel].timestampStart, __FILE__, __LINE__);
        success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.scanData[channel].numData, __FILE__, __LINE__);
        navdata.scanData[channel].data = std::vector<uint32_t>(navdata.scanData[channel].numData);
        for(int data_cnt = 0; data_cnt < navdata.scanData[channel].numData; data_cnt++)
        {
          success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.scanData[channel].data[data_cnt], __FILE__, __LINE__);
        }
      }

      // Parse remission data
      success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.remissionDataValid, __FILE__, __LINE__);
      if (navdata.remissionDataValid > 0)
      {
        if (strncmp((const char*)receiveBuffer + receivePos, "RSSI1", 5) != 0)
          ROS_ERROR_STREAM("## ERROR parseNAV350BinaryPositionData(): \"RSSI1\" not found");
        navdata.remissionData.contentType = std::string((const char*)receiveBuffer + receivePos, 5);
        receivePos += 5;
        success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.remissionData.scaleFactor, __FILE__, __LINE__);
        success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.remissionData.scaleOffset, __FILE__, __LINE__);
        success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.remissionData.startAngle, __FILE__, __LINE__);
        success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.remissionData.angleRes, __FILE__, __LINE__);
        success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.remissionData.timestampStart, __FILE__, __LINE__);
        success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.remissionData.numData, __FILE__, __LINE__);
        navdata.remissionData.data = std::vector<uint16_t>(navdata.remissionData.numData);
        for(int data_cnt = 0; data_cnt < navdata.remissionData.numData; data_cnt++)
        {
          success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, navdata.remissionData.data[data_cnt], __FILE__, __LINE__);
        }
      }

      if (!success)
        ROS_ERROR_STREAM("## ERROR parseNAV350BinaryPositionData(): parsing error");
      return success;
    }

    /** Parse binary NAV350 landmark data received by response "sAN mNMAPDoMapping" after request "sMN mNMAPDoMapping" */
    bool parseNAV350BinaryLandmarkDataDoMappingResponse(const uint8_t* receiveBuffer, int receiveBufferLength, NAV350LandmarkDataDoMappingResponse& landmarkData)
    {
      // Init return value
      landmarkData = NAV350LandmarkDataDoMappingResponse();
      // Parse header
      if (receiveBuffer == 0 || receiveBufferLength < 19 + 9 || memcmp(receiveBuffer, "\x02\x02\x02\x02", 4) != 0)
      {
        ROS_ERROR_STREAM("## ERROR parseNAV350LandmarkDataDoMappingResponse(): invalid telegram (" << __FILE__ << ":" << __LINE__ << ")");
        return false;
      }
      bool success = true;
      int receivePos = 4;
      uint32_t payload_size = 0;
      success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, payload_size, __FILE__, __LINE__);
      if (!success || (int)(payload_size) + 9 > receiveBufferLength)
      {
        ROS_ERROR_STREAM("## ERROR parseNAV350LandmarkDataDoMappingResponse(): invalid telegram size (" << __FILE__ << ":" << __LINE__ << ")");
        return false;
      }
      // Parse "sAN mNMAPDoMapping "
      if (strncmp((const char*)receiveBuffer + receivePos, "sAN mNMAPDoMapping ", 19) != 0)
      {
        ROS_ERROR_STREAM("## ERROR parseNAV350LandmarkDataDoMappingResponse(): telegram does not start with \"sAN mNMAPDoMapping \"");
        return false;
      }
      receivePos += 19;
      // Parse errorCode
      success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.errorCode, __FILE__, __LINE__);
      success &= readFromBuffer(receiveBuffer, receivePos, receiveBufferLength, landmarkData.landmarkDataValid, __FILE__, __LINE__);
      if (landmarkData.errorCode)
      {
        std::map<uint8_t, std::string> errorCodeStr = { {0, "no error"}, {1, "wrong operating mode"}, {2, "asynchrony Method terminated"}, {5, "timeout"}, {6, "method already active"}, {7, "general error"} };
        ROS_WARN_STREAM("## WARNING parseNAV350LandmarkDataDoMappingResponse(): NAV350 landmarkData.errorCode = " << (int)landmarkData.errorCode << " (\"" << errorCodeStr[landmarkData.errorCode] << "\")");
      }
      // Parse landmarkData
      if (landmarkData.landmarkDataValid > 0)
      {
        success &= parseNAV350BinaryLandmarkData(receiveBuffer, receivePos, receiveBufferLength, landmarkData.landmarkData);
      }
      return success;
    }

    /** Creates and returns the sopas command "sMN mNLAYAddLandmark landmarkData {x y type subtype size layerID {ID}}" */
    std::vector<uint8_t> createNAV350BinaryAddLandmarkRequest(const NAV350LandmarkData& landmarkData, int nav_curr_layer)
    {
      std::string sopas_start = "sMN mNLAYAddLandmark ";
      std::vector<uint8_t> request(sopas_start.begin(), sopas_start.end());
      appendToBuffer(request, (uint16_t)landmarkData.reflectors.size());
      for(int reflector_cnt = 0; reflector_cnt < landmarkData.reflectors.size(); reflector_cnt++)
      {
        if (landmarkData.reflectors[reflector_cnt].cartesianDataValid == 0)
          ROS_ERROR_STREAM("## ERROR createNAV350BinaryAddLandmarkRequest(): " << (reflector_cnt + 1) << ". reflector has no valid cartesian data");
        if (landmarkData.reflectors[reflector_cnt].optReflectorDataValid == 0)
          ROS_ERROR_STREAM("## ERROR createNAV350BinaryAddLandmarkRequest(): " << (reflector_cnt + 1) << ". reflector has no valid type and subtype");
        appendToBuffer(request, (int32_t)landmarkData.reflectors[reflector_cnt].cartesianData.x);
        appendToBuffer(request, (int32_t)landmarkData.reflectors[reflector_cnt].cartesianData.y);
        request.push_back((uint8_t)landmarkData.reflectors[reflector_cnt].optReflectorData.type);
        request.push_back((uint8_t)landmarkData.reflectors[reflector_cnt].optReflectorData.subType);
        appendToBuffer(request, (uint16_t)landmarkData.reflectors[reflector_cnt].optReflectorData.size);
        appendToBuffer(request, (uint16_t)1);
        appendToBuffer(request, (uint16_t)nav_curr_layer);
      }
      return request;
    }

    /** Creates and returns the sopas command "sMN mNLAYAddLandmark landmarkData {x y type subtype size layerID {ID}}" */
    std::vector<uint8_t> createNAV350BinaryAddLandmarkRequest(const std::vector<sick_scan_xd::NAV350ImkLandmark> landmarks)
    {
      std::string sopas_start = "sMN mNLAYAddLandmark ";
      std::vector<uint8_t> request(sopas_start.begin(), sopas_start.end());
      appendToBuffer(request, (uint16_t)landmarks.size());
      for(int reflector_cnt = 0; reflector_cnt < landmarks.size(); reflector_cnt++)
      {
        appendToBuffer(request, (int32_t)landmarks[reflector_cnt].x_mm);
        appendToBuffer(request, (int32_t)landmarks[reflector_cnt].y_mm);
        request.push_back((uint8_t)landmarks[reflector_cnt].type);
        request.push_back((uint8_t)landmarks[reflector_cnt].subType);
        appendToBuffer(request, (uint16_t)landmarks[reflector_cnt].size_mm);
        appendToBuffer(request, (uint16_t)landmarks[reflector_cnt].layerID.size());
        for(int layer_cnt = 0; layer_cnt < landmarks[reflector_cnt].layerID.size(); layer_cnt++)
          appendToBuffer(request, (uint16_t)landmarks[reflector_cnt].layerID[layer_cnt]);
      }
      return request;
    }

    /** Creates and returns the sopas command "sMN mNPOSSetSpeed X Y Phi timestamp coordBase" */
    std::vector<uint8_t> createNAV350BinarySetSpeedRequest(const sick_scan_msg::NAVOdomVelocity& msg)
    {
      std::string sopas_start = "sMN mNPOSSetSpeed ";
      std::vector<uint8_t> request(sopas_start.begin(), sopas_start.end());
      appendToBuffer(request, (int16_t)(1000.0 * msg.vel_x));
      appendToBuffer(request, (int16_t)(1000.0 * msg.vel_y));
      appendToBuffer(request, (int32_t)(1000.0 * msg.omega * 180.0 / M_PI));
      appendToBuffer(request, (uint32_t)(msg.timestamp));
      appendToBuffer(request, (uint8_t)(msg.coordbase));
      return request;
    }

    /** Parse binary NAV350 position data telegrams, i.e. parse NAV350 "sAN mNPOSGetData ..." and converts the data to a ros_sensor_msgs::LaserScan message */
    bool parseNAV350BinaryPositionData(const uint8_t* receiveBuffer, int receiveBufferLength, short& elevAngleX200, double& elevationAngleInRad, rosTime& recvTimeStamp,
      bool config_sw_pll_only_publish, double config_time_offset, SickGenericParser* parser_, int& numEchos, ros_sensor_msgs::LaserScan & msg, 
      sick_scan_msg::NAVPoseData& nav_pose_msg, sick_scan_msg::NAVLandmarkData& nav_landmark_msg, NAV350mNPOSData& navdata)
    {
      // Init return values
      elevAngleX200 = 0;
      elevationAngleInRad = 0;
      numEchos = 1;
      navdata = NAV350mNPOSData();
      double nav_angle_offset = parser_->getCurrentParamPtr()->getScanAngleShift();

      // NAV-350 operation manual: scan frequency 8 Hz, angular resolution 0.25 deg, scanning 0 to 360 deg, 1440 points per scan
      ROS_HEADER_SEQ(msg.header, elevAngleX200);
      msg.range_min = parser_->get_range_min();
      msg.range_max = parser_->get_range_max();
      msg.angle_min = (float)(0.0 + nav_angle_offset);
      msg.angle_max = (float)(2.0 * M_PI + nav_angle_offset);
      msg.angle_increment = (float)(0.25 * M_PI / 180.0);
      msg.scan_time = 1.0f / 8.0f;
      msg.time_increment = msg.scan_time * msg.angle_increment / (float)(2.0 * M_PI);
      uint32_t nav_timestampStart = 0; // timestamp of scan start in ms

      // Parse "sAN mNPOSGetData "
      ROS_DEBUG_STREAM("NAV350: " << receiveBufferLength << " bytes received: " << DataDumper::binDataToAsciiString(&receiveBuffer[0], std::min<int>(64, receiveBufferLength)));
      if (!parseNAV350BinaryPositionData(receiveBuffer, receiveBufferLength, navdata))
      {
        ROS_ERROR_STREAM("## ERROR parseNAV350BinaryPositionData(): parsing error");
        return false;    
      }
      navdata.angleOffset = nav_angle_offset;

      // Convert NAV350PoseData to sick_scan_msg::NAVPoseData
      nav_pose_msg = sick_scan_msg::NAVPoseData();
      nav_pose_msg.pose_valid = 0;
      if (navdata.poseDataValid > 0)
      {
        nav_pose_msg.x = navdata.poseData.x;
        nav_pose_msg.y = navdata.poseData.y;
        nav_pose_msg.phi = navdata.poseData.phi;
        nav_pose_msg.opt_pose_data_valid = navdata.poseData.optPoseDataValid;
        if (navdata.poseData.optPoseDataValid > 0)
        {
          nav_pose_msg.output_mode = navdata.poseData.optPoseData.outputMode;
          nav_pose_msg.timestamp = navdata.poseData.optPoseData.timestamp;
          nav_pose_msg.mean_dev = navdata.poseData.optPoseData.meanDev;
          nav_pose_msg.nav_mode = navdata.poseData.optPoseData.navMode;
          nav_pose_msg.info_state = navdata.poseData.optPoseData.infoState;
          nav_pose_msg.quant_used_reflectors = navdata.poseData.optPoseData.quantUsedReflectors;
        }
        // Convert pose into ros coordinates
        convertNAVCartPos3DtoROSPos3D(nav_pose_msg.x, nav_pose_msg.y, nav_pose_msg.phi, nav_pose_msg.pose_x, nav_pose_msg.pose_y, nav_pose_msg.pose_yaw, nav_angle_offset); // position in ros coordinates in meter, yaw angle in radians
        nav_pose_msg.pose_valid = 1;
        ROS_DEBUG_STREAM("NAV350 PoseData: x=" << nav_pose_msg.pose_x << ", y=" << nav_pose_msg.pose_y << ", yaw=" << (nav_pose_msg.pose_yaw*180.0/M_PI)
          << " (lidar: x=" << nav_pose_msg.x << ", y=" << nav_pose_msg.y << ", phi=" << nav_pose_msg.phi << ")");
      }
      else
      {
        ROS_DEBUG_STREAM("NAV350: no PoseData");
      }

      // Convert NAV350LandmarkData to sick_scan_msg::NAVLandmarkData
      nav_landmark_msg = sick_scan_msg::NAVLandmarkData();
      if (navdata.landmarkDataValid > 0)
      {
        ROS_DEBUG_STREAM("NAV350 LandmarkData: " << navdata.landmarkData.reflectors.size() << " reflectors");
        nav_landmark_msg.landmark_filter = navdata.landmarkData.landmarkFilter;
        nav_landmark_msg.num_reflectors = navdata.landmarkData.numReflectors;
        nav_landmark_msg.reflectors.resize(navdata.landmarkData.reflectors.size());
        for(int reflector_cnt = 0; reflector_cnt < navdata.landmarkData.reflectors.size(); reflector_cnt++)
        {
          nav_landmark_msg.reflectors[reflector_cnt].cartesian_data_valid = navdata.landmarkData.reflectors[reflector_cnt].cartesianDataValid;
          nav_landmark_msg.reflectors[reflector_cnt].polar_data_valid = navdata.landmarkData.reflectors[reflector_cnt].polarDataValid;
          nav_landmark_msg.reflectors[reflector_cnt].opt_reflector_data_valid = navdata.landmarkData.reflectors[reflector_cnt].optReflectorDataValid;
          nav_landmark_msg.reflectors[reflector_cnt].pos_valid = 0;
          if (navdata.landmarkData.reflectors[reflector_cnt].cartesianDataValid > 0)
          {
            nav_landmark_msg.reflectors[reflector_cnt].x = navdata.landmarkData.reflectors[reflector_cnt].cartesianData.x;
            nav_landmark_msg.reflectors[reflector_cnt].y = navdata.landmarkData.reflectors[reflector_cnt].cartesianData.y;
            // Convert landmark position into ros coordinates in meter
            convertNAVCartPos2DtoROSPos2D(nav_landmark_msg.reflectors[reflector_cnt].x, nav_landmark_msg.reflectors[reflector_cnt].y, 
              nav_landmark_msg.reflectors[reflector_cnt].pos_x, nav_landmark_msg.reflectors[reflector_cnt].pos_y, nav_angle_offset);
            nav_landmark_msg.reflectors[reflector_cnt].pos_valid = 1;
          }
          if (navdata.landmarkData.reflectors[reflector_cnt].polarDataValid > 0)
          {
            nav_landmark_msg.reflectors[reflector_cnt].dist = navdata.landmarkData.reflectors[reflector_cnt].polarData.dist;
            nav_landmark_msg.reflectors[reflector_cnt].phi = navdata.landmarkData.reflectors[reflector_cnt].polarData.phi;
          }
          if (navdata.landmarkData.reflectors[reflector_cnt].optReflectorDataValid > 0)
          {
            nav_landmark_msg.reflectors[reflector_cnt].local_id = navdata.landmarkData.reflectors[reflector_cnt].optReflectorData.localID;
            nav_landmark_msg.reflectors[reflector_cnt].global_id = navdata.landmarkData.reflectors[reflector_cnt].optReflectorData.globalID;
            nav_landmark_msg.reflectors[reflector_cnt].type  = navdata.landmarkData.reflectors[reflector_cnt].optReflectorData.type;
            nav_landmark_msg.reflectors[reflector_cnt].sub_type = navdata.landmarkData.reflectors[reflector_cnt].optReflectorData.subType;
            nav_landmark_msg.reflectors[reflector_cnt].quality = navdata.landmarkData.reflectors[reflector_cnt].optReflectorData.quality;
            nav_landmark_msg.reflectors[reflector_cnt].timestamp = navdata.landmarkData.reflectors[reflector_cnt].optReflectorData.timestamp;
            nav_landmark_msg.reflectors[reflector_cnt].size = navdata.landmarkData.reflectors[reflector_cnt].optReflectorData.size;
            nav_landmark_msg.reflectors[reflector_cnt].hit_count = navdata.landmarkData.reflectors[reflector_cnt].optReflectorData.hitCount;
            nav_landmark_msg.reflectors[reflector_cnt].mean_echo = navdata.landmarkData.reflectors[reflector_cnt].optReflectorData.meanEcho;
            nav_landmark_msg.reflectors[reflector_cnt].start_index = navdata.landmarkData.reflectors[reflector_cnt].optReflectorData.startIndex;
            nav_landmark_msg.reflectors[reflector_cnt].end_index = navdata.landmarkData.reflectors[reflector_cnt].optReflectorData.endIndex;
          }
          ROS_DEBUG_STREAM("NAV350 LandmarkData: reflector[" << reflector_cnt << "]"
            << ": cartesian=" << (int)(nav_landmark_msg.reflectors[reflector_cnt].cartesian_data_valid) << ", x=" << nav_landmark_msg.reflectors[reflector_cnt].x << ", y=" << nav_landmark_msg.reflectors[reflector_cnt].y 
            << ", polar=" << (int)(nav_landmark_msg.reflectors[reflector_cnt].polar_data_valid) << ", dist=" << nav_landmark_msg.reflectors[reflector_cnt].dist << ", phi=" << nav_landmark_msg.reflectors[reflector_cnt].phi
            << ", pos_valid=" << (int)(nav_landmark_msg.reflectors[reflector_cnt].pos_valid) << ", pos_x=" << nav_landmark_msg.reflectors[reflector_cnt].pos_x << ", pos_y=" << nav_landmark_msg.reflectors[reflector_cnt].pos_y
            << ", optValid=" << (int)(navdata.landmarkData.reflectors[reflector_cnt].optReflectorDataValid) << ", local_id=" << nav_landmark_msg.reflectors[reflector_cnt].local_id << ", global_id=" << nav_landmark_msg.reflectors[reflector_cnt].global_id);
        }
      }
      else
      {
        ROS_DEBUG_STREAM("NAV350: no LandmarkData");
      }

      // Convert scan data (NAV350ScanData) to LaserScan message
      if (navdata.scanData.empty())
        ROS_WARN_STREAM("## WARNING NAV350: no scan data channel in mNPOSGetData message");
      for(int channel = 0; channel < navdata.scanData.size(); channel++)
        ROS_DEBUG_STREAM("NAV350 scan data channel " << (channel + 1) << " of " << navdata.scanData.size() << ": " << navdata.scanData[channel].contentType << ", startAngle=" << navdata.scanData[channel].startAngle << ", angleRes=" << navdata.scanData[channel].angleRes << ", numPoints=" << navdata.scanData[channel].numData << ", timestampStart=" << navdata.scanData[channel].timestampStart);
      msg.ranges.clear();
      msg.intensities.clear();
      if (navdata.scanDataValid > 0 && navdata.scanData.size() > 0)
      {
        msg.angle_min = (float)(0.001 * navdata.scanData[0].startAngle * M_PI / 180.0 + nav_angle_offset);
        msg.angle_increment = (float)(0.001 * navdata.scanData[0].angleRes * M_PI / 180.0);
        msg.angle_max = (float)(msg.angle_min + msg.angle_increment * navdata.scanData[0].numData);
        nav_timestampStart = navdata.scanData[0].timestampStart;
        NAV350ScanData* dist_scan_data = 0;
        for(int channel = 0; dist_scan_data == 0 && channel < navdata.scanData.size(); channel++)
          if (navdata.scanData[channel].contentType == "DIST1")
            dist_scan_data = &navdata.scanData[channel];
        if (!dist_scan_data || dist_scan_data->numData <= 0)
        {
          ROS_ERROR_STREAM("## ERROR NAV350: No channel DIST1 found in scan data of mNPOSGetData message");
        }
        else
        {
          float scaleFactor = dist_scan_data->scaleFactor; // multiplier, always 1 for NAV350
          float scaleOffset = dist_scan_data->scaleOffset; // offset, always 0 for NAV350
          msg.ranges.resize(dist_scan_data->numData);
          if (std::fabs(scaleFactor - 1.0f) > FLT_EPSILON || std::fabs(scaleOffset) > FLT_EPSILON)
          {
            ROS_WARN_STREAM("## WARNING NAV350: using DIST1 scaleFactor=" << scaleFactor << " (expected: scaleFactor:=1) and scaleOffset=" << scaleOffset << " (expected: scaleOffset:=0) in mNPOSGetData message");
            for(int point_cnt = 0; point_cnt < dist_scan_data->numData; point_cnt++)
            {
              msg.ranges[point_cnt] = 0.001f * (scaleFactor * (float)dist_scan_data->data[point_cnt] + scaleOffset); // DIST in mm to range in m
            }
          }
          else
          {
            for(int point_cnt = 0; point_cnt < dist_scan_data->numData; point_cnt++)
            {
              msg.ranges[point_cnt] = 0.001f * (float)dist_scan_data->data[point_cnt]; // DIST in mm to range in m
            }
          }
        }
      }

      // Convert scan intensities (NAV350RemissionData) to LaserScan message
      if (navdata.remissionDataValid > 0)
      {
        ROS_DEBUG_STREAM("NAV350 remission data: " << navdata.remissionData.contentType << ", startAngle=" << navdata.remissionData.startAngle << ", angleRes=" << navdata.remissionData.angleRes << ", numPoints=" << navdata.remissionData.numData);
        if (navdata.remissionData.contentType != "RSSI1" || navdata.remissionData.numData != msg.ranges.size())
          ROS_WARN_STREAM("## WARNING NAV350: remissionData.contentType=" << navdata.remissionData.contentType << " (expected: RSSI1), remissionData.numData=" << navdata.remissionData.numData << " (expected: " << msg.ranges.size() << " points) in mNPOSGetData message");
        msg.intensities.resize(navdata.remissionData.numData);
        for(int point_cnt = 0; point_cnt < navdata.remissionData.numData; point_cnt++)
        {
          msg.intensities[point_cnt] = (float)navdata.remissionData.data[point_cnt];
        }
      }

      // Update Software-PLL by sensor timestamp
      if (nav_timestampStart > 0)
      {
        uint32_t recvTimeStampSec = (uint32_t)sec(recvTimeStamp), recvTimeStampNsec = (uint32_t)nsec(recvTimeStamp);
        bool softwarePLLready = SoftwarePLL::instance().updatePLL(recvTimeStampSec, recvTimeStampNsec, nav_timestampStart, nav_timestampStart);
        if (softwarePLLready)
        {
          SoftwarePLL::instance().getCorrectedTimeStamp(recvTimeStampSec, recvTimeStampNsec, nav_timestampStart);
          recvTimeStamp = rosTime(recvTimeStampSec, recvTimeStampNsec);
          msg.header.stamp = recvTimeStamp + rosDurationFromSec(config_time_offset); // recvTimeStamp updated by software-pll
          ROS_DEBUG_STREAM("NAV350: SoftwarePLL ready: NAV-timestamp=" << nav_timestampStart << " ms, ROS-timestamp=" << rosTimeToSeconds(recvTimeStamp) << " sec.");
        }
        else
        {
          ROS_DEBUG_STREAM("NAV350: SoftwarePLL still initializing, NAV-timestamp=" << nav_timestampStart << " ms, ROS-timestamp=" << rosTimeToSeconds(recvTimeStamp) << " sec.");
        }
        if (config_sw_pll_only_publish == true)
        {
          incSoftwarePLLPacketReceived();
        }
      }
      nav_pose_msg.header = msg.header;
      nav_landmark_msg.header = msg.header;

      return true;    
    }

    /** Rotates a cartesian 2D position (x, y) by a given angle offset */
    void rotateXYbyAngleOffset(float& x, float& y, double angle_offset)
    {
      if (std::abs(angle_offset - (+M_PI)) <= FLT_EPSILON || std::abs(angle_offset - (-M_PI)) <= FLT_EPSILON)
      {
        x = (-x);
        y = (-y);
      }
      else if (std::abs(angle_offset) <= FLT_EPSILON)
      {
        x = (+x);
        y = (+y);
      }
      else
      {
        x = (float)(x * cos(angle_offset) - y * sin(angle_offset));
        y = (float)(x * sin(angle_offset) + y * cos(angle_offset));
      }
    }

    /** Converts a cartesian 2D position from NAV to ROS coordinates */
    void convertNAVCartPos2DtoROSPos2D(int32_t nav_posx_mm, int32_t nav_posy_mm, float& ros_posx_m, float& ros_posy_m, double nav_angle_offset)
    {
      ros_posx_m = (float)(1.0e-3 * nav_posx_mm);
      ros_posy_m = (float)(1.0e-3 * nav_posy_mm);
      rotateXYbyAngleOffset(ros_posx_m, ros_posy_m, nav_angle_offset);
    }

    /** Converts a cartesian 3D pose from NAV to ROS coordinates */
    void convertNAVCartPos3DtoROSPos3D(int32_t nav_posx_mm, int32_t nav_posy_mm, uint32_t nav_phi_mdeg, float& ros_posx_m, float& ros_posy_m, float& ros_yaw_rad, double nav_angle_offset)
    {
      convertNAVCartPos2DtoROSPos2D(nav_posx_mm, nav_posy_mm, ros_posx_m, ros_posy_m, nav_angle_offset); // position in ros coordinates in meter
      ros_yaw_rad = (float)(1.0e-3 * nav_phi_mdeg * M_PI / 180.0 + nav_angle_offset); // yaw angle in radians
    }

    /** Convert NAV350PoseData to ros transform */
    ros_geometry_msgs::TransformStamped convertNAVPoseDataToTransform(NAV350PoseData& poseData, rosTime recvTimeStamp, double config_time_offset, 
      const std::string& tf_parent_frame_id, const std::string& tf_child_frame_id, SickGenericParser* parser_)
    {
      double nav_angle_offset = parser_->getCurrentParamPtr()->getScanAngleShift(); // NAV350: -PI
      ros_geometry_msgs::TransformStamped tf;
      // Convert to ROS transform
      float posx = 0, posy = 0, yaw = 0;
      convertNAVCartPos3DtoROSPos3D(poseData.x, poseData.y, poseData.phi, posx, posy, yaw, nav_angle_offset); // position in ros coordinates in meter, yaw angle in radians
      // Convert timestamp from lidar to system time
      if (poseData.optPoseDataValid > 0 && poseData.optPoseData.timestamp > 0 && SoftwarePLL::instance().IsInitialized())
      {
        uint32_t recvTimeStampSec = (uint32_t)sec(recvTimeStamp), recvTimeStampNsec = (uint32_t)nsec(recvTimeStamp);
        SoftwarePLL::instance().getCorrectedTimeStamp(recvTimeStampSec, recvTimeStampNsec, poseData.optPoseData.timestamp);
        tf.header.stamp = rosTime(recvTimeStampSec, recvTimeStampNsec) + rosDurationFromSec(config_time_offset); // recvTimeStamp updated by software-pll
        // Check function convSystemtimeToLidarTimestamp inverse to getCorrectedTimeStamp (debugging only)
        // uint32_t lidar_timestamp = 0;
        // SoftwarePLL::instance().convSystemtimeToLidarTimestamp(recvTimeStampSec, recvTimeStampNsec, lidar_timestamp);
        // if (poseData.optPoseData.timestamp != lidar_timestamp)
        //   ROS_ERROR_STREAM("## ERROR ticks_in=" << poseData.optPoseData.timestamp << ", time=" << rosTimeToSeconds(rosTime(recvTimeStampSec, recvTimeStampNsec)) << ", ticks_out=" << lidar_timestamp);
      }
      else
      {
        tf.header.stamp = recvTimeStamp;
      }
      // Convert to ros transform message
      tf.header.frame_id = tf_parent_frame_id;
      tf.child_frame_id = tf_child_frame_id;
      tf.transform.translation.x = posx;
      tf.transform.translation.y = posy;
      tf.transform.translation.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();
      return tf;
    }

    /** Converts NAV350 landmark data to a ROS VisualMarker message */
    ros_visualization_msgs::MarkerArray convertNAVLandmarkDataToMarker(const std::vector<NAV350ReflectorData>& reflectors, ros_std_msgs::Header& header, SickGenericParser* parser_)
    {
      double nav_angle_offset = parser_->getCurrentParamPtr()->getScanAngleShift(); // NAV350: -PI
      ros_visualization_msgs::MarkerArray marker_array;
      marker_array.markers.reserve(1 + 2 * reflectors.size());
      marker_array.markers.push_back(ros_visualization_msgs::Marker());
      marker_array.markers.back().header = header;
      marker_array.markers.back().action = ros_visualization_msgs::Marker::DELETEALL;    
      for(int reflector_cnt = 0; reflector_cnt < reflectors.size(); reflector_cnt++)
      {
        if (reflectors[reflector_cnt].cartesianDataValid > 0)
        {
          float reflector_posx = 0, reflector_posy = 0;
          convertNAVCartPos2DtoROSPos2D(reflectors[reflector_cnt].cartesianData.x, reflectors[reflector_cnt].cartesianData.y, reflector_posx, reflector_posy, nav_angle_offset);
          float reflector_size = 0.001f * (float)((reflectors[reflector_cnt].optReflectorDataValid > 0) ? (reflectors[reflector_cnt].optReflectorData.size) : 80);
          int32_t reflector_id = ((reflectors[reflector_cnt].optReflectorDataValid > 0) ? (reflectors[reflector_cnt].optReflectorData.localID) : (reflector_cnt + 1));
          ros_visualization_msgs::Marker marker_cylinder;
          marker_cylinder.ns = "sick_scan";
          marker_cylinder.id = reflector_cnt;
          marker_cylinder.type = ros_visualization_msgs::Marker::CYLINDER;
          marker_cylinder.scale.x = reflector_size;
          marker_cylinder.scale.y = reflector_size;
          marker_cylinder.scale.z = reflector_size;
          marker_cylinder.pose.position.x = reflector_posx;
          marker_cylinder.pose.position.y = reflector_posy;
          marker_cylinder.pose.position.z = 0.0;
          marker_cylinder.pose.orientation.x = 0.0;
          marker_cylinder.pose.orientation.y = 0.0;
          marker_cylinder.pose.orientation.z = 0.0;
          marker_cylinder.pose.orientation.w = 1.0;
          marker_cylinder.action = ros_visualization_msgs::Marker::ADD;
          marker_cylinder.color.r = 1.0f;
          marker_cylinder.color.g = 0.0f;
          marker_cylinder.color.b = 0.0f;
          marker_cylinder.color.a = 0.5f;
          marker_cylinder.lifetime = rosDurationFromSec(0); // lifetime 0 indicates forever
          marker_cylinder.header = header;
          marker_array.markers.push_back(marker_cylinder);

          ros_visualization_msgs::Marker marker_text;
          marker_text.ns = "sick_scan";
          marker_text.id = reflector_cnt + reflectors.size();
          marker_text.type = ros_visualization_msgs::Marker::TEXT_VIEW_FACING;
          marker_text.text = std::to_string(reflector_id);
          marker_text.scale.x = reflector_size;
          marker_text.scale.y = reflector_size;
          marker_text.scale.z = 2.0f * reflector_size;
          marker_text.pose.position.x = reflector_posx;
          marker_text.pose.position.y = reflector_posy + 1.5f * reflector_size;
          marker_text.pose.position.z = 0.0;
          marker_text.pose.orientation.x = 0.0;
          marker_text.pose.orientation.y = 0.0;
          marker_text.pose.orientation.z = 0.0;
          marker_text.pose.orientation.w = 1.0;
          marker_text.action = ros_visualization_msgs::Marker::ADD;
          marker_text.color.r = 1.0f;
          marker_text.color.g = 1.0f;
          marker_text.color.b = 1.0f;
          marker_text.color.a = 1.0f;
          marker_text.lifetime = rosDurationFromSec(0); // lifetime 0 indicates forever
          marker_text.header = header;
          marker_array.markers.push_back(marker_text);

        }
      }
      return marker_array;
    }

    /** Import a NAV350 landmark layout from imk file. The NAV350 landmark layout can be stored in imk files using SOPAS ET. Each line in an imk file is formatted "globID x_mm y_mm type subtype size_mm layer1 layer2 layer3" */
    std::vector<sick_scan_xd::NAV350ImkLandmark> readNAVIMKfile(const std::string& nav_imk_file)
    {
      std::vector<sick_scan_xd::NAV350ImkLandmark> navImkLandmarks;
      std::ifstream imk_stream(nav_imk_file);
      if(imk_stream.is_open())
      {
        std::string line;
        while (getline(imk_stream, line))
        {
          // Ignore comments and headers
          for(int n = 0; n < line.size(); n++)
          {
            if (isspace(line[n]))
              line[n] = ' ';
          }
          if (line.empty() || line[0] == '#' || strncmp(line.c_str(), "globID", 6) == 0)
            continue;
          // Split into arguments
          std::stringstream ss(line);
          std::string token;
          std::vector<std::string> args;
          while (std::getline(ss, token, ' '))
          {
            if (!token.empty())
              args.push_back(token);
          }
          // Ignore empty lines
          if (args.empty())
            continue;
          // Parse arguments
          if (args.size() < 7)
          {
            ROS_WARN_STREAM("## ERROR readNAVIMKfile(" << nav_imk_file << "): parse error, line \"" << line << "\" ignored.");
            continue;
          }
          sick_scan_xd::NAV350ImkLandmark landmark;
          landmark.globID = (uint16_t)(atoi(args[0].c_str()) & 0xFFFF);
          landmark.x_mm = (int32_t)(atoi(args[1].c_str()) & 0xFFFFFFFF);
          landmark.y_mm = (int32_t)(atoi(args[2].c_str()) & 0xFFFFFFFF);
          landmark.type = (uint8_t)(atoi(args[3].c_str()) & 0xFF);
          landmark.subType = (uint16_t)(atoi(args[4].c_str()) & 0xFFFF);
          landmark.size_mm = (uint16_t)(atoi(args[5].c_str()) & 0xFFFF);
          for(int n = 6; n < args.size(); n++)
            landmark.layerID.push_back((uint16_t)(atoi(args[n].c_str()) & 0xFFFF));
          navImkLandmarks.push_back(landmark);
        }      
      }
      ROS_INFO_STREAM("sick_scan_xd::readNAVIMKfile(\"" << nav_imk_file << "\"): " << navImkLandmarks.size() << " landmarks:");
      for(int n = 0; n < navImkLandmarks.size(); n++)
      {
        std::stringstream imk_stream;
        imk_stream << (int)navImkLandmarks[n].globID << " " << (int)navImkLandmarks[n].x_mm << " " << (int)navImkLandmarks[n].y_mm << " " << (int)navImkLandmarks[n].type << " " << (int)navImkLandmarks[n].subType << " " << (int)navImkLandmarks[n].size_mm;
        for(int m = 0; m < navImkLandmarks[n].layerID.size(); m++)
          imk_stream << " " << (int)navImkLandmarks[n].layerID[m];
        ROS_INFO_STREAM("landmark " << (n + 1) << ": " << imk_stream.str());
      }
      return navImkLandmarks;
    }

} /* namespace sick_scan_xd */
