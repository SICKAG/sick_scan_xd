/*
 * @brief sim_loc_utils contains a collection of utility functions for SIM Localization.
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

#include "sick_scan/ros_wrapper.h"
#include <iomanip>
#include <string>
#include <vector>
//#include <boost/algorithm/hex.hpp>
//#include <boost/algorithm/string.hpp>
#include "sick_scan/utils.h"

/*
 * Converts and returns binary data to hex string
 * @param[in] binary_data binary input data
 * @return hex string
 */
std::string sick_scan_xd::Utils::toHexString(const std::vector<uint8_t> & binary_data)
{
  //std::string hex_string;
  //hex_string.reserve(binary_data.size() * 2);
  //boost::algorithm::hex(binary_data.begin(), binary_data.end(), std::back_inserter(hex_string));
  //return hex_string;
  std::stringstream hex_string;
  for(int n = 0; n < binary_data.size(); n++)
    hex_string << std::setfill('0') << std::setw(2) << std::hex << (int)(binary_data[n]);
  return hex_string.str();
}

/*!
 * Converts and returns binary data to ascii string with non-ascii data represented as "\x<hexvalue>"
 * @param[in] binary_data binary input data
 * @return hex string
 */
std::string sick_scan_xd::Utils::toAsciiString(const uint8_t* binary_data, int length)
{
  std::stringstream out;
  for(int n = 0; n < length; n++)
  {
    int val = (int)(binary_data[n] & 0xFF);
    if ((val == 0x20) || (val >= 48 && val <= 57) || (val >= 65 && val <= 90) || (val >= 97 && val <= 122))
    {
      out << std::string(1,(char)(val & 0xFF));
      /*char s[2] = {0};
      sprintf(s, "%c", (char)(val & 0xFF));
      out << s;*/
    }
    else
    {
      out <<  "\\x" << std::setfill('0') << std::setw(2) << std::hex << val;
    }
  }
  return out.str();
}

/*!
 * Replaces all substrings of a string by another string.
 * https://stackoverflow.com/questions/3418231/replace-part-of-a-string-with-another-string/3418285
 */
void sick_scan_xd::Utils::replaceAll(std::string& str, const std::string& from, const std::string& to) 
{
  // boost::replace_all(str, from, to);
  if(!str.empty() && !from.empty())
  {
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) 
    {
      str.replace(start_pos, from.length(), to);
      start_pos += to.length();
    }
  }
}

/*
 * Shortcut to replace linefeeds by colon-separators
 */
void sick_scan_xd::Utils::flattenString(std::string & s)
{
  replaceAll(s, "\n", ", ");
  replaceAll(s, ": , ", ": ");
  while(s.find("  ") != std::string::npos)
    replaceAll(s, "  ", " ");
}

/*!
 * Splits a string into its space separated substrings
 */
std::vector<std::string> sick_scan_xd::Utils::splitSpaces(const std::string & s)
{
  std::vector<std::string> parts;
  parts.push_back("");
  for(int n = 0; n < s.size(); n++)
  {
    if(isspace(s[n]))
    {
      if(!parts.back().empty())
      {
        parts.push_back("");
        parts.back().reserve(s.size() + 1 - n);
      }
    }
    else
    {
      parts.back().push_back(s[n]);
    }
  }
  // std::vector<std::string> boost_parts;
  // boost::split(boost_parts, s, boost::algorithm::is_space());
  // assert(parts.size() == boost_parts.size());
  // for(int n = 0; n < parts.size() && n < boost_parts.size(); n++)
  //   assert(parts[n] == boost_parts[n]);
  return parts;
}
