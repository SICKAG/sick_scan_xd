#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * Copyright (C) 2018, Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2018, SICK AG, Waldkirch
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
 *  Created on: 28th May 2018
 *
 *      Authors:
 *       Michael Lehning <michael.lehning@lehning.de>
 *
 */

#ifndef SICK_GENERIC_FIELD_MON_H_
#define SICK_GENERIC_FIELD_MON_H_

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <vector>

#include <sick_scan/sick_ros_wrapper.h>

namespace sick_scan_xd
{
  enum SickScanMonFieldType
  {
    MON_FIELD_RADIAL = 0, // not supported
    MON_FIELD_RECTANGLE = 1,
    MON_FIELD_SEGMENTED = 2,
    MON_FIELD_DYNAMIC = 3
  };

  class SickScanMonFieldConverter
  { 
  public:
    /*
    ** @brief Converts a point of a segmented field to carthesian coordinates
    ** @param[in] range range in meter
    ** @param[in] angle_rad angle in radians
    ** @param[out] x x in meter in ros coordinate system
    ** @param[out] y y in meter in ros coordinate system
    */
    static void segmentedFieldPointToCarthesian(float range, float angle_rad, float& x, float& y);

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
    static void rectangularFieldToCarthesian(float distRefPointMeter, float angleRefPointRad, float rotAngleRad, float rectWidthMeter, float rectLengthMeter, float points_x[4], float points_y[4]);

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
    static void dynamicFieldPointToCarthesian(float distRefPointMeter, float angleRefPointRad, float rotAngleRad, float rectWidthMeter, float rectLengthMeter, float maxSpeedMeterPerSec, float maxLengthMeter, float points_x[8], float points_y[8]);

  };

  class SickScanMonField
  { 
  public:

    SickScanMonFieldType& fieldType(void) { return m_fieldType; }
    const SickScanMonFieldType& fieldType(void) const { return m_fieldType; }

    void pushFieldPointCarthesian(float x, float y)
    {
      m_fieldPoints_X.push_back(x);
      m_fieldPoints_Y.push_back(y);
    }

    int getPointCount(void) const
    { 
      assert(m_fieldPoints_X.size() == m_fieldPoints_Y.size());
      return (int)m_fieldPoints_X.size(); 
    }

    const std::vector<float>& getFieldPointsX(void) const { return m_fieldPoints_X; }
    const std::vector<float>& getFieldPointsY(void) const { return m_fieldPoints_Y; }
    
  private:
    SickScanMonFieldType m_fieldType = MON_FIELD_RADIAL;
    std::vector<float> m_fieldPoints_X;
    std::vector<float> m_fieldPoints_Y;
  };


  class SickScanFieldMonSingleton
  {
  private:
    /* Here will be the instance stored. */
    static SickScanFieldMonSingleton *instance;

    /* Private constructor to prevent instancing. */
    SickScanFieldMonSingleton();

    std::vector<SickScanMonField>monFields;
    int active_mon_fieldset = 0;
    int mon_field_selection_method = 0; // FieldSetSelectionMethod: 0 = digital inputs (default), 1 = telegram "sWN ActiveFieldSet"

  public:
    /* Static access method. */
    static SickScanFieldMonSingleton *getInstance();

    const std::vector<SickScanMonField>& getMonFields(void) const { return monFields; }

    void setActiveFieldset(int active_fieldset) { active_mon_fieldset = active_fieldset; } // asssumes 1 <= active_fieldset <= max active fieldsets
    int getActiveFieldset(void) { return active_mon_fieldset; }                            // asssumes 1 <= active_fieldset <= max active fieldsets

    void setFieldSelectionMethod(int field_selection_method) { mon_field_selection_method = field_selection_method; } // FieldSetSelectionMethod: 0 = digital inputs (default), 1 = telegram "sWN ActiveFieldSet"
    int getFieldSelectionMethod(void) { return mon_field_selection_method; }

    int parseAsciiLIDinputstateMsg(unsigned char* datagram, int datagram_length);
    int parseBinaryLIDinputstateMsg(unsigned char* datagram, int datagram_length, sick_scan_msg::LIDinputstateMsg& inputstate_msg);

    void parseActiveFieldSetResponse(unsigned char* datagram, int datagram_length, uint16_t* active_field_set);
    void parseFieldSetSelectionMethodResponse(unsigned char* datagram, int datagram_length, uint8_t* field_set_selection_method);

    int parseBinaryDatagram(std::vector<unsigned char> datagramm, float rectFieldAngleRefPointOffsetRad);

    int parseAsciiDatagram(std::vector<unsigned char> datagramm, float rectFieldAngleRefPointOffsetRad);
  
    std::string LIDinputstateMsgToString(const sick_scan_msg::LIDinputstateMsg& inputstate_msg);
  };


#if 0
  class SickScanRadar
  {
  public:
    SickScanRadar(SickScanCommon *commonPtr_)
    {
      commonPtr = commonPtr_;
    }
    void setEmulation(bool _emul);
    bool getEmulation(void);
    int parseDatagram(rosTime timeStamp, unsigned char *receiveBuffer, int actual_length, bool useBinaryProtocol);
    int parseAsciiDatagram(char* datagram, size_t datagram_length, sick_scan_msg::RadarScan *msgPtr, std::vector<SickScanRadarObject> &objectList, std::vector<SickScanRadarRawTarget> &rawTargetList); /* , SickScanConfig &config, */ // sensor_msgs::LaserScan &msg, int &numEchos, int &echoMask);
    void simulateAsciiDatagram(unsigned char * receiveBuffer, int* actual_length);
  private:
//		SickScanCommon *commonPtr;
    void simulateAsciiDatagramFromFile(unsigned char *receiveBuffer, int *actual_length, std::string filePattern);
    bool emul;
  };
#endif

} /* namespace sick_scan_xd */
#endif // SICK_GENERIC_RADAR_H_
