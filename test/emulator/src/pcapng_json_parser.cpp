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
#include "sick_scan/ros_wrapper.h"
#include <fstream>
#include <sstream>
#include <string.h>
#include <jsoncpp/json/json.h>

#include "sick_scan/pcapng_json_parser.h"
#include "sick_scan/utils.h"

/*!
 * @brief Splits a comma separated string into its parts.
 * @param[in] s comma separated string
 * @return list of substrings
 */
std::vector<std::string> sick_scan::PcapngJsonParser::split(const std::string & s, char delimiter)
{
  std::vector<std::string> substrings;
  std::string part;
  std::istringstream ss(s);
  while (std::getline(ss, part, delimiter))
     substrings.push_back(part);
  return substrings;
}

/*!
 * @brief Parses a jsonfile and returns a list of binary scandata messages of given type.
 * @param[in] json_filename jsonfile incl. path, f.e. "tim781s_scandata.pcapng.json
 * @param[in] scandatatypes list of scandata message types, f.e. "sSN LMDscandata,sSN LMDscandatamon"
 * @param[in] start_time offset for the relativ time stamps
 * @param[out] scandata list of binary scandata messages
 * @return true on sucess, false on error
 */
bool sick_scan::PcapngJsonParser::parseJsonfile(const std::string & json_filename, const std::vector<std::string> & scandatatypes, double start_time, std::vector<sick_scan::JsonScanData> & scandata)
{
  try
  {
    // convert scandatatypes to hex string
    std::vector<std::string> scandatatypes_hex;
    scandatatypes_hex.reserve(scandatatypes.size());
    for(int type_cnt = 0; type_cnt < scandatatypes.size(); type_cnt++)
    {
      const std::string & scandatatype = scandatatypes[type_cnt];
      std::stringstream hex_stream;
      for(int char_cnt = 0; char_cnt < scandatatype.size(); char_cnt++)
      {
        if(char_cnt > 0)
          hex_stream << ":";
        hex_stream << std::setfill('0') << std::setw(2) << std::hex << (int)(scandatatype[char_cnt] & 0xFF);
      }
      scandatatypes_hex.push_back(hex_stream.str());
    }
    // Read and parse json file
    std::ifstream json_file(json_filename);
    if(!json_file.is_open())
    {
      ROS_WARN_STREAM("## WARNING sick_scan::PcapngJsonParser::parseJsonfile: error reading file \"" << json_filename << "\".");   
      return false;
    }
    ROS_INFO_STREAM("sick_scan::PcapngJsonParser: parsing file \"" << json_filename << "\"...");
    Json::Reader json_reader;
    Json::Value json_root;
    json_reader.parse(json_file, json_root);
    int msg_cnt = 0;
    // Loop over all json objects "tcp"
    for (Json::Value::ArrayIndex json_idx = 0; json_idx != json_root.size(); json_idx++)
    {
      if(json_root[json_idx].isMember("_source") && json_root[json_idx]["_source"].isMember("layers") && json_root[json_idx]["_source"]["layers"].isMember("tcp"))
      {
        Json::Value & json_tcp = json_root[json_idx]["_source"]["layers"]["tcp"];
        std::string tcp_timestamp, tcp_payload;
        if(json_tcp.isMember("Timestamps") && json_tcp["Timestamps"].isMember("tcp.time_relative"))
          tcp_timestamp = json_tcp["Timestamps"]["tcp.time_relative"].asString();
        if(json_tcp.isMember("tcp.payload"))
          tcp_payload = json_tcp["tcp.payload"].asString();
        // Check payload against scandatatypes
        bool type_check_passed = scandatatypes.empty(); // empty scandatatypes means all types
        for(int type_cnt = 0; type_check_passed == false && type_cnt < scandatatypes_hex.size(); type_cnt++)
        {
          if(tcp_payload.find(scandatatypes_hex[type_cnt]) != std::string::npos)
            type_check_passed = true;
        }
        if(!type_check_passed)
          continue; // different scandatatype
        if(tcp_timestamp.size() > 0 && tcp_payload.size() > 4)
        {
          // Parse timestamp and payload
          double msg_timestamp = std::stod(tcp_timestamp) + start_time;
          std::vector<uint8_t> msg_payload;
          msg_payload.reserve(tcp_payload.size()/3 + 1);
          std::string hexval;
          std::istringstream payload_stream(tcp_payload);
          while (std::getline(payload_stream, hexval, ':'))
          {
            msg_payload.push_back(std::stoul(hexval, 0, 16) & 0xFF);
          }
          scandata.push_back(sick_scan::JsonScanData(msg_timestamp, msg_payload));
          // std::cout << "msg_timestamp=" << msg_timestamp << ", msg_payload=" << sick_scan::Utils::toHexString(msg_payload) << std::endl;
          msg_cnt++;
        }
      }
    }
    ROS_INFO_STREAM("sick_scan::PcapngJsonParser: " << msg_cnt << " messages in file \"" << json_filename << "\" successfully parsed.");   
    return true;
  }
  catch(const std::exception& e)
  {
    ROS_WARN_STREAM("## WARNING sick_scan::PcapngJsonParser::parseJsonfile: exception \"" << e.what() << "\" in file \"" << json_filename << "\".");   
    std::cerr << e.what() << '\n';
  }
  return false;
}
