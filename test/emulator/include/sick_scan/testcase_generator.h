/*
 * @brief sim_loc_testcase_generator generates testcases for SIM Localization driver.
 * The generator creates deterministic and random based result port telegrams.
 *
 * Copyright (C) 2019 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2019 SICK AG, Waldkirch
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
 *  Copyright 2019 SICK AG
 *  Copyright 2019 Ing.-Buero Dr. Michael Lehning
 *
 */
#ifndef __SIM_LOC_TESTCASE_GENERATOR_H_INCLUDED
#define __SIM_LOC_TESTCASE_GENERATOR_H_INCLUDED

#include "sick_scan/ros_wrapper.h"
#include "sick_scan/result_port_parser.h"

namespace sick_scan
{
  /*!
   * class TestcaseGenerator generates testcases for SIM Localization driver.
   * It creates deterministic and random based result port telegrams.
   */
  class TestcaseGenerator
  {
  public:
    
    /*!
     * Creates and returns a deterministic default testcase for result port telegrams (binary telegrams and SickLocResultPortTelegramMsg)
     * @return SickLocResultPortTestcaseMsg with the binary telegram and SickLocResultPortTelegramMsg
     */
    static sick_scan::SickLocResultPortTestcaseMsg createDefaultResultPortTestcase(void);
  
    /*!
     * Creates and returns a random testcase for result port telegrams (binary telegrams and SickLocResultPortTelegramMsg)
     * @return SickLocResultPortTestcaseMsg with the binary telegram and SickLocResultPortTelegramMsg
     */
    static sick_scan::SickLocResultPortTestcaseMsg createRandomResultPortTestcase(void);
  
    /*!
     * Creates and returns a result port telegrams (binary telegrams and SickLocResultPortTelegramMsg)
     * simulating sensor moving in circles with a current position given by radius in meter and yaw angle
     * in radians
     * @param[in] circle_radius radius of circle in meter
     * @param[in] circle_yaw current angle in radians
     * @return SickLocResultPortTestcaseMsg with the binary telegram and SickLocResultPortTelegramMsg
     */
    static sick_scan::SickLocResultPortTestcaseMsg createResultPortCircles(double circle_radius, double circle_yaw);
  
    /*!
     * Creates and returns a synthetical cola response to a cola command request.
     * Note: Just a few cola responses are implemented for test purposes, f.e. responses to "LocRequestTimestamp".
     * By default, a response: "sAN <command_name>" without any parameter is returned (sAN: Response to sMN)
     * @param[in] cola_request Cola request from client
     * @return Synthetical cola response from server
     */
    static sick_scan::SickLocColaTelegramMsg createColaResponse(const sick_scan::SickLocColaTelegramMsg & cola_request, const std::string& scanner_type);
  
    /*!
     * Returns the result pose interval, i.e. the interval in number of scans
     * 1 (default): result with each processed scan
     * 2: result with every second processed scan
     * n: result with every n.th processed scan
     * This parameter can be set by cola command "LocSetResultPoseInterval", f.e. "sMN LocSetResultPoseInterval 1"
     * @return result pose interval
     */
    static uint32_t ResultPoseInterval(void){ return s_u32ResultPoseInterval; }

    /*!
    * Returns true, if LMCstartmeas has been requested (i.e. send LMDscandata and LMDscandatamon, LMCstartmeas is enabled), 
    * or false otherwise (i.e. do not send LMDscandata or LMDscandatamon, LMCstartmeas is disabled)
    * @return  LMCstartmeas status (true or false)
    */
    static bool SendScandataEnabled(void);

    /*!
     * Returns true, if localization is active (default), otherwise false (localization deactivated)
     * @return result telegrams are activated (true) or deactivated
     */
    static bool LocalizationEnabled(void);

    /*!
     * Returns true, if result telegrams are activated (i.e. localization on and result telegrams active), otherwise false (result telegrams deactivated)
     * @return result telegrams are activated (true, default) or deactivated
     */
    static bool ResultTelegramsEnabled(void);
    
  protected:
  
    /*!
     * Converts value x to hex string
     * @param x value to be converted
     * @return x as hex string
     */
    template <typename T> static std::string hexstr(const T & x)
    {
      std::stringstream hex_stream;
      hex_stream << std::hex << std::uppercase << x;
      return hex_stream.str();
    }
  
    /*!
     * Converts value x to decimal string
     * @param x value to be converted
     * @return x as hex string
     */
    template <typename T> static std::string decstr(const T & x)
    {
      std::stringstream dec_stream;
      dec_stream << std::dec << std::uppercase << x;
      return dec_stream.str();
    }
  
    /*!
     * Creates and returns a timestamp in milliseconds ticks.
     * To simulate time jitter, network latency and time drift,
     * a random value of +/- 2 milliseconds is added.
     * @return timestamp in milliseconds ticks
     */
    static uint32_t createTimestampTicksMilliSec(void);
  
    static uint32_t s_u32ResultPoseInterval; ///< result pose interval, i.e. the interval in number of scans (default: 1, i.e. result telegram with each processed scan)
    static std::map<std::string, int32_t> s_controller_settings; ///< test server int32 settings, set by sMN or sRN requests
    static std::map<std::string, std::string> s_controller_settings_str; ///< test server string settings, set by sMN or sRN requests
    
  }; // class TestcaseGenerator
  
} // namespace sick_scan
#endif // __SIM_LOC_TESTCASE_GENERATOR_H_INCLUDED
