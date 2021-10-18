/*
 * @brief pcapng_json_parser parses jsonfiles converted from pcapng-files by pcap_json_converter.
 *
 * Copyright (C) 2021 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2021 SICK AG, Waldkirch
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
 *  Copyright 2021 SICK AG
 *  Copyright 2021 Ing.-Buero Dr. Michael Lehning
 *
 */
#ifndef __PCAPNG_JSON_PARSER_H_INCLUDED
#define __PCAPNG_JSON_PARSER_H_INCLUDED

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "sick_scan/ros_wrapper.h"

namespace sick_scan
{
  /*!
   * @brief class JsonScanData: utility container for binary scandata incl. timestamp
   */
  class JsonScanData
  {
  public:

    JsonScanData(double _timestamp = 0, const std::vector<uint8_t>& _data = std::vector<uint8_t>()) : timestamp(_timestamp), data(_data) {}
    double timestamp; // relative timestamp in seconds
    std::vector<uint8_t> data; // binary cola data 0x02020202....
  };

  /*!
   * @brief class PcapngJsonParser parses jsonfiles converted from pcapng-files by pcap_json_converter.
   */
  class PcapngJsonParser
  {
  public:

    /*!
     * @brief Parses a jsonfile and returns a list of binary scandata messages of given type.
     * @param[in] json_filename jsonfile incl. path, f.e. "tim781s_scandata.pcapng.json
     * @param[in] scandatatypes list of scandata message types, f.e. "sSN LMDscandata,sSN LMDscandatamon"
     * @param[in] start_time offset for the relativ time stamps
     * @param[out] scandata list of binary scandata messages incl. timestamp
     * @return true on sucess, false on error
     */
    static bool parseJsonfile(const std::string & json_filename, const std::vector<std::string> & scandatatypes, double start_time, std::vector<sick_scan::JsonScanData> & scandata);
    
    /*!
     * @brief Splits a comma separated string into its parts.
     * @param[in] s comma separated string
     * @return list of substrings
     */
    static std::vector<std::string> split(const std::string & s, char delimiter);
      
  protected:
    
  }; // class PcapngJsonParser
  
} // namespace sick_scan
#endif // __PCAPNG_JSON_PARSER_H_INCLUDED
