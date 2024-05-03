#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
* Copyright (C) 2017, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2017, SICK AG, Waldkirch
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
 *  Created on: 24.05.2012
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 * Utility functions for parsing scanner specific sopas requests and responses
 *
 */

#ifndef SICK_SCAN_PARSE_UTIL_H_
#define SICK_SCAN_PARSE_UTIL_H_

#include <string>
#include <vector>

#include <sick_scan/sick_ros_wrapper.h>

namespace sick_scan_xd
{
  // returns the given angle in rad normalized to angle_min ... angle_max, assuming (angle_max - angle_min) == 2 * PI
  double normalizeAngleRad(double angle_rad, double angle_min, double angle_max);

  // Converts a string to a 6D pose x,y,z,roll,pitch,yaw in [m] resp. [rad]
  std::vector<float> parsePose(const std::string& pose_xyz_rpy_str);

  class SickScanParseUtil
  {
  public:

    /*
    * Convert LMPscancfg from / to sopas requests/responses
    */

    class LMPscancfgSector
    {
    public:
      uint32_t angular_resolution = 0; // angular resolution in 1/10000 deg
      int32_t start_angle = 0; // start angle in 1/10000 deg
      int32_t stop_angle = 0; // stop angle in 1/10000 deg
    };

    class LMPscancfg
    {
    public:
      uint32_t scan_frequency = 0; // scan frequency in 1/100 Hz
      int16_t active_sector_cnt = 0; // number of active sectors
      std::vector<LMPscancfgSector> sector_cfg;
      std::string print() const;
    };

    /** @brief Parse the sopas reply to "sRN LMPscancfg" and convert to LMPscancfg */
    static bool SopasToLMPscancfg(const std::string& sopas_reply, LMPscancfg& scancfg);

    /** @brief Convert LMPscancfg to sopas request "sMN mLMPsetscancfg ..." */
    static bool LMPscancfgToSopas(const LMPscancfg& scancfg, std::string& sopas_cmd);

  }; // class SickScanParseUtil
} // namespace sick_scan_xd

#endif // SICK_SCAN_PARSE_UTIL_H_
