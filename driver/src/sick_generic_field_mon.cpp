/**
* \file
* \brief Field Monitoring Handling
* Copyright (C) 2020, 2021, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2020, 2021, SICK AG, Waldkirch
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
*  Last modified: 29th May 2018
*
*      Authors:
*         Michael Lehning <michael.lehning@lehning.de>
*
*/

#include <sick_scan/sick_scan_common.h>
#include <sick_scan/sick_generic_field_mon.h>
#include "sick_scan/sick_scan_messages.h"

namespace sick_scan_xd
{

  /*
  ** @brief Converts a point of a segmented field to carthesian coordinates
  ** @param[in] range range in meter
  ** @param[in] angle_rad angle in radians
  ** @param[out] x x in meter in ros coordinate system
  ** @param[out] y y in meter in ros coordinate system
  */
  void SickScanMonFieldConverter::segmentedFieldPointToCarthesian(float range, float angle_rad, float& x, float& y)
  {
      y = -range * std::cos(angle_rad); // y_ros = -x_sick = -range * std::cos(angle_rad)
      x = range * std::sin(angle_rad);  // x_ros = +y_sick = +range * std::sin(angle_rad)
  }

  /*
  ** @brief Converts a rectangular field to carthesian coordinates, i.e. to 4 corner points carthesian (ros) coordinates
  ** @param[in] distRefPointMeter range of ref point (rect center) in meter
  ** @param[in] angleRefPointRad angle of ref point (rect center) in radians
  ** @param[in] rotAngleRad rotation of rectangle in radians
  ** @param[in] rectWidthMeter width of rectangle in meter
  ** @param[in] rectLengthMeter width of rectangle in meter
  ** @param[out] points_x x of corner points in meter in ros coordinate system
  ** @param[out] points_y y of corner points in meter in ros coordinate system
  */
  void SickScanMonFieldConverter::rectangularFieldToCarthesian(float distRefPointMeter, float angleRefPointRad, float rotAngleRad, float rectWidthMeter, float rectLengthMeter, float points_x[4], float points_y[4])
  {
    // unrotated rectangle at base point, x_ros = +y_sick, y_ros = -x_sick
    // base point is the lower left corner of the rectangle
    points_x[0] = 0;
    points_y[0] = 0;
    points_x[1] = 0;
    points_y[1] = -rectWidthMeter;
    points_x[2] = rectLengthMeter;
    points_y[2] = -rectWidthMeter;
    points_x[3] = rectLengthMeter;
    points_y[3] = 0;
    // rotate by rotAngleRad
    float sin_angle = sinf(rotAngleRad);
    float cos_angle = cosf(rotAngleRad);
    for(int n = 0; n < 4; n++)
    {
      float x = points_x[n], y = points_y[n];
      points_x[n] = x * cos_angle - y * sin_angle;
      points_y[n] = x * sin_angle + y * cos_angle;
    }
    // move to refPoint
    float refPointX = 0, refPointY = 0;
    segmentedFieldPointToCarthesian(distRefPointMeter, angleRefPointRad, refPointX, refPointY);
    for(int n = 0; n < 4; n++)
    {
      points_x[n] += refPointX;
      points_y[n] += refPointY;
    }
    // ROS_DEBUG_STREAM("rectangularFieldToCarthesian: distRefPointMeter=" << distRefPointMeter << ", angleRefPoint=" << (angleRefPointRad * 180.0 / M_PI) << ", rotAngle=" << (rotAngleRad * 180.0 / M_PI) << ", rectWidthMeter=" << rectWidthMeter << ", rectLengthMeter=" << rectLengthMeter);
  }

  /*
  ** @brief Converts a dynamic field to carthesian coordinates. The dynamic field is just converted into 2 rectangular fields,
  ** each rectangular field described by to 4 corner points carthesian (ros) coordinates.
  ** The first rectangular field has size rectWidthMeter x maxLengthMeter, the second rectangular field has size rectWidthMeter x rectLengthMeter.
  ** The rectangular fields are returned by 4 corner points (points_x[n], points_y[n]) with 0 <= n < 4 for the first (big) rectangle and 4 <= n < 8 for the second (smaller) rectangle.
  ** @param[in] distRefPointMeter range of ref point (rect center) in meter
  ** @param[in] angleRefPointRad angle of ref point (rect center) in radians
  ** @param[in] rotAngleRad rotation of rectangle in radians
  ** @param[in] rectWidthMeter width of rectangle in meter
  ** @param[in] rectLengthMeter width of rectangle in meter at v = 0
  ** @param[in] maxSpeedMeterPerSec max speed (unused)
  ** @param[in] maxLengthMeter width of rectangle in meter at v = max. speed
  ** @param[out] points_x x of corner points in meter in ros coordinate system
  ** @param[out] points_y y of corner points in meter in ros coordinate system
  */
  void SickScanMonFieldConverter::dynamicFieldPointToCarthesian(float distRefPointMeter, float angleRefPointRad, float rotAngleRad, float rectWidthMeter, float rectLengthMeter, float maxSpeedMeterPerSec, float maxLengthMeter, float points_x[8], float points_y[8])
  {
    // set 2 rectangular fields: 1. rectangular field with rectWidthMeter x maxLengthMeter, 2.nd rectangular field with rectWidthMeter x rectLengthMeter
   float field1_points_x[4], field1_points_y[4], field2_points_x[4], field2_points_y[4];
   rectangularFieldToCarthesian(distRefPointMeter, angleRefPointRad, rotAngleRad, rectWidthMeter, maxLengthMeter, field1_points_x, field1_points_y);
   rectangularFieldToCarthesian(distRefPointMeter, angleRefPointRad, rotAngleRad, rectWidthMeter, rectLengthMeter, field2_points_x, field2_points_y);
   for(int n = 0; n < 4; n++)
   {
     points_x[n + 0] = field1_points_x[n];
     points_y[n + 0] = field1_points_y[n];
     points_x[n + 4] = field2_points_x[n];
     points_y[n + 4] = field2_points_y[n];
   }
  }

  /* Null, because instance will be initialized on demand. */
  SickScanFieldMonSingleton *SickScanFieldMonSingleton::instance = 0;

  SickScanFieldMonSingleton *SickScanFieldMonSingleton::getInstance()
  {
    if (instance == 0)
    {
      instance = new SickScanFieldMonSingleton();
    }

    return instance;
  }

  SickScanFieldMonSingleton::SickScanFieldMonSingleton()
  {
    this->monFields.resize(48);
  }

   /*!
  \brief Parse binary LIDinputstate message and set active field set
  \param datagramm: Pointer to datagram data
  \param datagram_length: Number of bytes in datagram
  */
  int SickScanFieldMonSingleton::parseAsciiLIDinputstateMsg(unsigned char* datagram, int datagram_length)
  {
    ROS_ERROR("SickScanFieldMonSingleton::parseAsciiLIDinputstateMsg not implemented.");
    int exitCode=ExitSuccess;
    return (exitCode);
  }

  /*!
  \brief Parse binary LIDinputstate message and set active field set
  \param datagramm: Pointer to datagram data
  \param datagram_length: Number of bytes in datagram
  */
  int SickScanFieldMonSingleton::parseBinaryLIDinputstateMsg(unsigned char* datagramm, int datagram_length, sick_scan_msg::LIDinputstateMsg& inputstate_msg)
  {
    int exitCode=ExitSuccess;
    if(datagram_length > 36)
    {
      // Read N output states
      int number_of_input_states = 4; // LMS5xx, TiM7xx and TiM7xxS have 4 digital inputs (M12 connector)
      inputstate_msg.header.stamp = rosTimeNow();
      inputstate_msg.input_state.clear();
      inputstate_msg.input_state.reserve(number_of_input_states);
      int fieldset = 0;
      for(int offset = 35; offset >= 32; offset--) // datagramm[32]=INT1, datagramm[33]=INT2, datagramm[34]=INT3, datagramm[35]=INT4 
      {
        fieldset = (fieldset << 1);
        fieldset |= ((datagramm[offset] == 1) ? 1 : 0);
        inputstate_msg.input_state.push_back(datagramm[offset]);
      }
      fieldset += 1; // FieldSet in range 1 to 16, i.e. 0x00000000 is fieldset 1, 0x00000001 is fieldset 2, ..., 0x01000000 is fieldset 9, ..., 0x01010101 is fieldset 16
      if (getFieldSelectionMethod() == 1) // ActiveFieldset from telegram
      {
        fieldset = getActiveFieldset();
      }
      else // default: ActiveFieldset from LIDinputstate
      {
        setActiveFieldset(fieldset);
      }
      inputstate_msg.active_fieldset = fieldset;
      // Read 2 byte version + 4 byte system counter
      int msg_start_idx = 26; // start after 4 byte STX + payload length + "... LIDinputstate "
      datagramm += msg_start_idx;
      datagram_length -= msg_start_idx;
      if( !readBinaryBuffer(datagramm, datagram_length, inputstate_msg.version_number)
        || !readBinaryBuffer(datagramm, datagram_length, inputstate_msg.system_counter))
      {
          ROS_ERROR_STREAM("## ERROR parseBinaryLIDinputstateMsg(): error parsing version_number and system_counter (" << __FILE__ << ":" << __LINE__ << ")");
          return ExitError;
      }
      // Read timestamp state
      datagramm += 4;
      datagram_length -= 4;
      if( !readBinaryBuffer(datagramm, datagram_length, inputstate_msg.time_state))
      {
          ROS_ERROR_STREAM("## ERROR parseBinaryLIDinputstateMsg(): error parsing time_state (" << __FILE__ << ":" << __LINE__ << ")");
          return ExitError;
      }
      // Read optional timestamp
      if(inputstate_msg.time_state > 0)
      {
        if(!readBinaryBuffer(datagramm, datagram_length, inputstate_msg.year)
        || !readBinaryBuffer(datagramm, datagram_length, inputstate_msg.month)
        || !readBinaryBuffer(datagramm, datagram_length, inputstate_msg.day)
        || !readBinaryBuffer(datagramm, datagram_length, inputstate_msg.hour)
        || !readBinaryBuffer(datagramm, datagram_length, inputstate_msg.minute)
        || !readBinaryBuffer(datagramm, datagram_length, inputstate_msg.second)
        || !readBinaryBuffer(datagramm, datagram_length, inputstate_msg.microsecond))
        {
            ROS_ERROR_STREAM("## ERROR parseBinaryLIDinputstateMsg(): error parsing timestamp (" << __FILE__ << ":" << __LINE__ << ")");
            return ExitError;
        }
      }
    }
    else
    {
      exitCode = ExitError;
    }
    return exitCode;
  }

  /*!
  \brief Parse binary ActiveFieldSet response "sRA ActiveFieldSet"
  \param datagramm: Pointer to datagram data
  \param datagram_length: Number of bytes in datagram
  */
  void SickScanFieldMonSingleton::parseActiveFieldSetResponse(unsigned char* datagram, int datagram_length, uint16_t* active_field_set)
  {
    // Example: \x02\x02\x02\x02\x00\x00\x00\x15sRA ActiveFieldSet \x00\x01
    int arg_start_idx = 27; // start after 4 byte STX + 4 byte payload length + 19 byte "sRA ActiveFieldSet "
    datagram += arg_start_idx;
    datagram_length -= arg_start_idx;
    readBinaryBuffer(datagram, datagram_length, *active_field_set);
  }

  /*!
  \brief Parse binary FieldSetSelectionMethod response "sRA FieldSetSelectionMethod"
  \param datagramm: Pointer to datagram data
  \param datagram_length: Number of bytes in datagram
  */
  void SickScanFieldMonSingleton::parseFieldSetSelectionMethodResponse(unsigned char* datagram, int datagram_length, uint8_t* field_set_selection_method)
  {
    int arg_start_idx = 36; // start after 4 byte STX + 4 byte payload length + 28 byte "sRA FieldSetSelectionMethod "
    datagram += arg_start_idx;
    datagram_length -= arg_start_idx;
    readBinaryBuffer(datagram, datagram_length, *field_set_selection_method);
  }

  /*!
  \brief Converts a LIDinputstateMsg to a readable string
  */
  std::string SickScanFieldMonSingleton::LIDinputstateMsgToString(const sick_scan_msg::LIDinputstateMsg& inputstate_msg)
  {
    std::stringstream state_str;
    state_str << "LIDinputstateMsg = { " << "version_number: " << (uint32_t)inputstate_msg.version_number << ", system_counter: " << (uint32_t)inputstate_msg.system_counter << ", states: (";
    for(int state_cnt = 0; state_cnt < inputstate_msg.input_state.size(); state_cnt++)
        state_str << (state_cnt > 0 ? ", " :"") << (uint32_t)inputstate_msg.input_state[state_cnt];
    state_str << "), active_fieldset: " << inputstate_msg.active_fieldset << ", time state: " << (uint32_t)inputstate_msg.time_state
      << ", date: " << std::setfill('0') << std::setw(4) << (uint32_t)inputstate_msg.year << "-" << std::setfill('0') << std::setw(2) << (uint32_t)inputstate_msg.month << "-" << std::setfill('0') << std::setw(2) << (uint32_t)inputstate_msg.day
      << ", time: " << std::setfill('0') << std::setw(2) << (uint32_t)inputstate_msg.hour << ":" << std::setfill('0') << std::setw(2) << (uint32_t)inputstate_msg.minute << ":" << std::setfill('0') << std::setw(2) << (uint32_t)inputstate_msg.second
      << "." << std::setfill('0') << std::setw(6) << (uint32_t)inputstate_msg.microsecond << " }";
    return state_str.str();
  }

  /*!
  \brief Parsing Ascii datagram
  \param datagram: Pointer to datagram data
  \param datagram_length: Number of bytes in datagram
  */
  int SickScanFieldMonSingleton::parseAsciiDatagram(std::vector<unsigned char> datagramm, float rectFieldAngleRefPointOffsetRad)
  {
    ROS_ERROR("SickScanFieldMonSingleton::parseAsciiDatagram not implemented.");
    int exitCode=ExitSuccess;
    return (exitCode);
  }

  int SickScanFieldMonSingleton::parseBinaryDatagram(std::vector<unsigned char> datagram, float rectFieldAngleRefPointOffsetRad)
  {
    if (datagram.size() < 41) // at least 41 byte required to read the field configuration
    {
      return 0;
    }
    int exitCode = ExitSuccess;
    int fieldNumberFromCMD=0;
    std::string sDatagramm( datagram.begin()+8, datagram.end() );
    sscanf(sDatagramm.c_str(), "sRA field%d", &fieldNumberFromCMD);
    float distScaleFactor;
    float distScaleFactorOffset;
    uint32_t angScaleFactor;
    int32_t angScaleFactorOffset;
    uint8_t fieldType;
    uint8_t fieldNumber;
    uint16_t segmentedFieldConfigured = 0;
    uint16_t rectangularFieldConfigured = 0;
    uint16_t dynamicFieldConfigured = 0;
    uint32_t dataPtrOffset = 0;

    unsigned char *dataPtr= &(datagram[0]);
    memcpy(&distScaleFactor, dataPtr  + 21, 4);
    memcpy(&distScaleFactorOffset, dataPtr  + 25, 4);
    memcpy(&angScaleFactor, dataPtr  + 29, 4);
    memcpy(&angScaleFactorOffset, dataPtr  + 33, 4);
    memcpy(&fieldType, dataPtr  + 37, 1);
    memcpy(&fieldNumber, dataPtr  + 38, 1);
    memcpy(&segmentedFieldConfigured, dataPtr  + 39, 2);
    swap_endian((unsigned char *) &distScaleFactor, 4);
    swap_endian((unsigned char *) &distScaleFactorOffset, 4);
    swap_endian((unsigned char *) &angScaleFactor, 4);
    swap_endian((unsigned char *) &angScaleFactorOffset, 4);
    swap_endian((unsigned char *) &fieldType, 1);
    swap_endian((unsigned char *) &fieldNumber, 1);
    swap_endian((unsigned char *) &segmentedFieldConfigured, 2);
    monFields[fieldNumberFromCMD].fieldType() = (SickScanMonFieldType)(fieldType);
    dataPtrOffset = 41;
    if(segmentedFieldConfigured==1) // configuration for segmented fields
    {
      uint16_t numOfFieldPoints;
      uint16_t angIDX;
      uint16_t startDist,stopDist;
      memcpy(&numOfFieldPoints, dataPtr  + 41, 2);
      swap_endian((unsigned char *) &numOfFieldPoints, 2);
      monFields[fieldNumberFromCMD].pushFieldPointCarthesian(0, 0);
      for(uint16_t point=0;point<numOfFieldPoints;point++)
      {
        memcpy(&angIDX, dataPtr  + 43+point*6, 2);
        memcpy(&startDist, dataPtr  + 45+point*6, 2);
        memcpy(&stopDist, dataPtr  + 47+point*6, 2);
        swap_endian((unsigned char *) &angIDX, 2);
        swap_endian((unsigned char *) &startDist, 2);
        swap_endian((unsigned char *) &stopDist, 2);
        float angRad= (float)((angIDX*angScaleFactor/1e4+angScaleFactorOffset/1e4)*deg2rad);
        float distMeter=(stopDist*distScaleFactor+distScaleFactorOffset)/1000.0f;
        float point_x = 0, point_y = 0;
        SickScanMonFieldConverter::segmentedFieldPointToCarthesian(distMeter, angRad, point_x, point_y);
        monFields[fieldNumberFromCMD].pushFieldPointCarthesian(point_x, point_y);
      }
      dataPtrOffset = 43 + numOfFieldPoints * 6;
    }
    memcpy(&rectangularFieldConfigured, dataPtr + dataPtrOffset, 2);
    swap_endian((unsigned char *) &rectangularFieldConfigured, 2);
    dataPtrOffset += 2;
    if(rectangularFieldConfigured==1) // configuration for rectangular fields
    {
      int32_t angleRefPoint;
      uint16_t distRefPoint;
      int32_t rotAngle;
      uint32_t rectWidth, rectLength;
      memcpy(&angleRefPoint, dataPtr + dataPtrOffset + 0, 4);
      memcpy(&distRefPoint, dataPtr + dataPtrOffset + 4, 2);
      memcpy(&rotAngle, dataPtr + dataPtrOffset + 6, 4);
      memcpy(&rectWidth, dataPtr + dataPtrOffset + 10, 4);
      memcpy(&rectLength, dataPtr + dataPtrOffset + 14, 4);
      dataPtrOffset += 18;
      swap_endian((unsigned char *) &angleRefPoint, 4);
      swap_endian((unsigned char *) &distRefPoint, 2);
      swap_endian((unsigned char *) &rotAngle, 4);
      swap_endian((unsigned char *) &rectWidth, 4);
      swap_endian((unsigned char *) &rectLength, 4);
      float angleRefPointRad= (float)((angleRefPoint/1e4+angScaleFactorOffset/1e4)*deg2rad) - rectFieldAngleRefPointOffsetRad;
      float distRefPointMeter=(distRefPoint*distScaleFactor+distScaleFactorOffset)/1000.0f;
      float rotAngleRad= (float)((rotAngle/1e4)*deg2rad);
      float rectWidthMeter=(rectWidth)/1000.0f;
      float rectLengthMeter=(rectLength)/1000.0f;
      float points_x[4] = {0}, points_y[4] = {0};
      SickScanMonFieldConverter::rectangularFieldToCarthesian(distRefPointMeter, angleRefPointRad, rotAngleRad, rectWidthMeter, rectLengthMeter, points_x, points_y);
      for(int n = 0; n < 4; n++)
        monFields[fieldNumberFromCMD].pushFieldPointCarthesian(points_x[n], points_y[n]);
    }
    dataPtrOffset += 2; // 2 byte radial field configured, always 0, currently not supported (not used in LMS5xx and TiM7xx)
    memcpy(&dynamicFieldConfigured, dataPtr + dataPtrOffset, 2);
    swap_endian((unsigned char *) &dynamicFieldConfigured, 2);
    dataPtrOffset += 2;
    if(dynamicFieldConfigured==1) // configuration for dynamic fields
    {
      int32_t angleRefPoint;
      uint16_t distRefPoint;
      int32_t rotAngle;
      uint32_t rectWidth, rectLength;
      int16_t maxSpeed;
      uint32_t maxLength;
      memcpy(&angleRefPoint, dataPtr + dataPtrOffset + 0, 4);
      memcpy(&distRefPoint, dataPtr + dataPtrOffset + 4, 2);
      memcpy(&rotAngle, dataPtr + dataPtrOffset + 6, 4);
      memcpy(&rectWidth, dataPtr + dataPtrOffset + 10, 4);
      memcpy(&rectLength, dataPtr + dataPtrOffset + 14, 4);
      memcpy(&maxSpeed, dataPtr + dataPtrOffset + 18, 2);
      memcpy(&maxLength, dataPtr + dataPtrOffset + 20, 4);
      dataPtrOffset += 24;
      swap_endian((unsigned char *) &angleRefPoint, 4);
      swap_endian((unsigned char *) &distRefPoint, 2);
      swap_endian((unsigned char *) &rotAngle, 4);
      swap_endian((unsigned char *) &rectWidth, 4);
      swap_endian((unsigned char *) &rectLength, 4);
      swap_endian((unsigned char *) &maxSpeed, 2);
      swap_endian((unsigned char *) &maxLength, 4);
      float angleRefPointRad= (float)((angleRefPoint/1e4+angScaleFactorOffset/1e4)*deg2rad) - rectFieldAngleRefPointOffsetRad;
      float distRefPointMeter=(distRefPoint*distScaleFactor+distScaleFactorOffset)/1000.0f;
      float rotAngleRad= (float)((rotAngle/1e4)*deg2rad);
      float rectWidthMeter=(rectWidth)/1000.0f;
      float rectLengthMeter=(rectLength)/1000.0f;
      float maxSpeedMeterPerSec=(maxSpeed)/1000.0f;
      float maxLengthMeter=(maxLength)/1000.0f;
      float points_x[8] = {0}, points_y[8] = {0};
      SickScanMonFieldConverter::dynamicFieldPointToCarthesian(distRefPointMeter, angleRefPointRad, rotAngleRad, rectWidthMeter, rectLengthMeter, maxSpeedMeterPerSec, maxLengthMeter, points_x, points_y);
      for(int n = 0; n < 8; n++)
        monFields[fieldNumberFromCMD].pushFieldPointCarthesian(points_x[n], points_y[n]);
    }

    return (exitCode);
  }

}
