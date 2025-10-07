#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * sick_lmd_scandata_parser parses result telegrams of type LMDscandata.
 *
 * Copyright (C) 2022, Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2022, SICK AG, Waldkirch
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

#ifndef SICK_LMD_SCANDATA_PARSER_H_
#define SICK_LMD_SCANDATA_PARSER_H_

#include <string>
#include <vector>

#include <sick_scan/sick_ros_wrapper.h>
#include <sick_scan/sick_generic_parser.h>


namespace sick_scan_xd
{

  /** Parse common result telegrams, i.e. parse telegrams of type LMDscandata received from the lidar */
  bool parseCommonBinaryResultTelegram(const uint8_t* receiveBuffer, int receiveBufferLength, short& elevAngleX200, double elevAngleTelegramValToDeg, double& elevationAngleInRad, rosTime& recvTimeStamp,
    bool config_sw_pll_only_publish, bool use_generation_timestamp, SickGenericParser* parser_, bool& FireEncoder, sick_scan_msg::Encoder& EncoderMsg, int& numEchos,
    std::vector<float>& vang_vec, std::vector<float>& azimuth_vec, ros_sensor_msgs::LaserScan & msg);

    /** Increments the number of packets received in the SoftwarePLL */
    void incSoftwarePLLPacketReceived();

    void configureAngleParameters(ros_sensor_msgs::LaserScan& msg,
      double startAngle,
      double sizeOfSingleAngularStep,
      int numberOfItems,
      SickGenericParser* parser);
} /* namespace sick_scan_xd */
#endif /* SICK_LMD_SCANDATA_PARSER_H_ */
