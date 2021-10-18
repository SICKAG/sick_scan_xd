/*
 * @brief sim_loc_cola_converter converts between Cola-ASCII and Cola-Binary telegrams.
 * See Operation-Instruction-v1.1.0.241R.pdf, chapter 5.8 "About CoLa-A telegrams", page 46-48,
 * Telegram-Listing-v1.1.0.241R.pdf, chapter 2.3.9 "Command: LocRequestTimestamp", page 21, and
 * Technical_information_Telegram_Listing_NAV_LOC_en_IM0076556.PDF for further details about
 * Cola telegrams.
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
#include <sstream>
#include <string.h>

#include "sick_scan/cola_converter.h"
#include "sick_scan/cola_parser.h"

/*!
 * @brief static ascii table to convert binary to ascii, f.e. s_ascii_table[0x02]:="<STX>", s_ascii_table[0x03]:="<ETX>", s_ascii_table[0x41]:="A" and so on
 */
const std::string sick_scan::ColaAsciiBinaryConverter::s_ascii_table[256] =
  {
    "", "<SOH>", "<STX>", "<ETX>", "<EOT>", "<ENQ>", "<ACK>", "<BEL>", "\b", "<HT>", "\n", "<VT>", "<FF>", "\n",
    "<SO>", "<SI>", "<DLE>", "<DC1>", "<DC2>", "<DC3>", "<DC4>", "<NAK>", "<SYN>", "<ETB>", "<CAN>", "<EM>", "<SUB>", "<ESC>", "<FS>", "<GS>", "<RS>", "<US>",
    " ", "!", "\"", "#", "$", "%", "&", "'", "(", ")", "*", "+", ",", "-", ".", "/",
    "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", ":", ";", "<", "=", ">", "?", "@",
    "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "[", "\\", "]", "^", "_", "`",
    "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z", "{", "|", "}", "~", "<DEL>",
    "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
    "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
    "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
    "", "", "", "", "", "", "", ""
  };

/*!
 * @brief static ascii map to convert ascii to binary, f.e. s_ascii_map["<STX>"]:=0x02, s_ascii_map["<ETX>"]:=0x03, s_ascii_map["A"]:=0x41 and so on
 */
const std::map<std::string, uint8_t> sick_scan::ColaAsciiBinaryConverter::s_ascii_map =
  {
    {"<SOH>", 0x01}, {"<STX>", 0x02}, {"<ETX>", 0x03}, {"<EOT>", 0x04}, {"<ENQ>", 0x05}, {"<ACK>", 0x06}, {"<BEL>", 0x07}, {"\b", 0x08}, {"<HT>", 0x09},
    {"\n", 0x0A}, {"<VT>", 0x0B}, {"<FF>", 0x0C}, {"\n", 0x0D}, {"<SO>", 0x0E}, {"<SI>", 0x0F}, {"<DLE>", 0x10}, {"<DC1>", 0x11}, {"<DC2>", 0x12},
    {"<DC3>", 0x13}, {"<DC4>", 0x14}, {"<NAK>", 0x15}, {"<SYN>", 0x16}, {"<ETB>", 0x17}, {"<CAN>", 0x18}, {"<EM>", 0x19}, {"<SUB>", 0x1A}, {"<ESC>", 0x1B},
    {"<FS>", 0x1C}, {"<GS>", 0x1D}, {"<RS>", 0x1E}, {"<US>", 0x1F}, {" ", 0x20}, {"!", 0x21}, {"\"", 0x22}, {"#", 0x23}, {"$", 0x24}, {"%", 0x25}, {"&", 0x26},
    {"'", 0x27}, {"(", 0x28}, {")", 0x29}, {"*", 0x2A}, {"+", 0x2B}, {",", 0x2C}, {"-", 0x2D}, {".", 0x2E}, {"/", 0x2F},
    {"0", 0x30}, {"1", 0x31}, {"2", 0x32}, {"3", 0x33}, {"4", 0x34}, {"5", 0x35}, {"6", 0x36}, {"7", 0x37}, {"8", 0x38}, {"9", 0x39}, {":", 0x3A}, {";", 0x3B}, {"=", 0x3D}, {"?", 0x3F}, {"@", 0x40},
    {"A", 0x41}, {"B", 0x42}, {"C", 0x43}, {"D", 0x44}, {"E", 0x45}, {"F", 0x46}, {"G", 0x47}, {"H", 0x48}, {"I", 0x49}, {"J", 0x4A}, {"K", 0x4B}, {"L", 0x4C}, {"M", 0x4D}, {"N", 0x4E}, {"O", 0x4F},
    {"P", 0x50}, {"Q", 0x51}, {"R", 0x52}, {"S", 0x53}, {"T", 0x54}, {"U", 0x55}, {"V", 0x56}, {"W", 0x57}, {"X", 0x58}, {"Y", 0x59}, {"Z", 0x5A}, {"[", 0x5B}, {"\\", 0x5C}, {"]", 0x5D}, {"^", 0x5E}, {"_", 0x5F}, {"`", 0x60},
    {"a", 0x61}, {"b", 0x62}, {"c", 0x63}, {"d", 0x64}, {"e", 0x65}, {"f", 0x66}, {"g", 0x67}, {"h", 0x68}, {"i", 0x69}, {"j", 0x6A}, {"k", 0x6B}, {"l", 0x6C}, {"m", 0x6D}, {"n", 0x6E}, {"o", 0x6F},
    {"p", 0x70}, {"q", 0x71}, {"r", 0x72}, {"s", 0x73}, {"t", 0x74}, {"u", 0x75}, {"v", 0x76}, {"w", 0x77}, {"x", 0x78}, {"y", 0x79}, {"z", 0x7A}, {"{", 0x7B}, {"|", 0x7C}, {"}", 0x7D}, {"~", 0x7E}, {"<DEL>", 0x7F}
  };

/*!
 * @brief Converts and returns a Cola-ASCII telegram to string.
 * @param[in] cola_telegram Cola-ASCII telegram, starting with 0x02 and ending with 0x03
 * @return Cola-ASCII string, f.e. "<STX>sMN SetAccessMode 3 F4724744<ETX>"
 */
std::string sick_scan::ColaAsciiBinaryConverter::ConvertColaAscii(const std::vector<uint8_t> & cola_telegram)
{
  std::stringstream cola_ascii;
  for(std::vector<uint8_t>::const_iterator iter = cola_telegram.cbegin(); iter != cola_telegram.cend(); iter++)
  {
    // Convert binary to ascii by lookup table
    cola_ascii << s_ascii_table[((*iter) & 0xFF)];
  }
  return cola_ascii.str();
}

/*!
 * @brief Converts and returns a Cola telegram from Cola-ASCII to Cola-Binary.
 * @param[in] cola_telegram Cola-ASCII string, f.e. "<STX>sMN SetAccessMode 3 F4724744<ETX>"
 * @return Cola-ASCII telegram, f.e. { 0x02, 0x73, 0x4D, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63,  0x65,
 * 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x33, 0x20, 0x46, 0x34, 0x37, 0x32, 0x34, 0x37, 0x34, 0x34, 0x03 }
 */
std::vector<uint8_t> sick_scan::ColaAsciiBinaryConverter::ConvertColaAscii(const std::string & cola_telegram)
{
  std::vector<uint8_t> cola_ascii;
  cola_ascii.reserve(cola_telegram.size());
  for(size_t char_cnt = 0; char_cnt < cola_telegram.size(); char_cnt++)
  {
    cola_ascii.push_back(0); // default, overwrite by value converted from ascii
    // Convert ascii string to binary by lookup table
    for(std::map<std::string, uint8_t>::const_iterator map_iter = s_ascii_map.cbegin(); map_iter != s_ascii_map.cend(); map_iter++)
    {
      if(strncmp(map_iter->first.c_str(), cola_telegram.c_str() + char_cnt, map_iter->first.size()) == 0)
      {
        cola_ascii.back() = map_iter->second;
        char_cnt += (map_iter->first.size() - 1);
        break;
      }
    }
  }
  return cola_ascii;
}

static uint8_t CRC8XOR(uint8_t* msgBlock, size_t len)
{
  uint8_t xorVal = 0x00;
  for (size_t i = 0; i < len; i++)
    xorVal ^= (msgBlock[i]);
  return xorVal;
}

/*!
 * @brief Converts and returns a Cola telegram from Cola-ASCII to Cola-Binary.
 * @param[in] cola_telegram Cola-ASCII telegram, f.e. { 0x02, 0x73, 0x4D, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63,  0x65,
 * 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x33, 0x20, 0x46, 0x34, 0x37, 0x32, 0x34, 0x37, 0x34, 0x34, 0x03 }
 * ("<STX>sMN SetAccessMode 3 F4724744<ETX>")
 * @param[in] parameter_is_ascii Command parameter given in ascii (f.e. if 1, a parameter 0x31 == ASCII-"1" will be converted to 0x01 (default), otherwise to 0x01)
 *            parameter_is_ascii = 1: Command parameter given in ascii and will be converted to binary
 *            parameter_is_ascii = 0: Command parameter given binary and will be copied to binary output
 *            parameter_is_ascii = -1: Auto determination (1. assumption is true, 2. assumption is false in case of errors)
 * @return Cola-Binary telegram
 */
std::vector<uint8_t> sick_scan::ColaAsciiBinaryConverter::ColaAsciiToColaBinary(const std::vector<uint8_t> & cola_telegram, int parameter_is_ascii)
{
  // Split telegram in command_type (sRN,sRA,sMN,sMA,sWN), command_name ("SetAccessMode", "LocSetResultPoseEnabled",
  // "LocRequestTimestamp", etc.) and command parameter
  sick_scan::SickLocColaTelegramMsg cola_msg = sick_scan::ColaParser::decodeColaTelegram(cola_telegram);
  if(cola_msg.command_type < 0 || cola_msg.command_type >= sick_scan::ColaParser::MAX_COLA_COMMAND_NUMBER)
  {
    ROS_ERROR_STREAM("## ERROR in ColaAsciiToColaBinary(): invalid SOPAS command type " << cola_msg.command_type);
    return std::vector<uint8_t>();
  }
  return ColaTelegramToColaBinary(cola_msg, parameter_is_ascii);
}

    /*!
     * @brief Converts and returns a Cola telegram to Cola-Binary.
     * @param[in] cola_telegram Cola telegram,
     * @param[in] parameter_is_ascii Command parameter given in ascii (f.e. if 1, a parameter 0x31 == ASCII-"1" will be converted to 0x01 (default), otherwise to 0x01)
     * @param[in] parameter_is_ascii Command parameter given in ascii (f.e. if 1, a parameter 0x31 == ASCII-"1" will be converted to 0x01 (default), otherwise to 0x01)
     *            parameter_is_ascii = 1: Command parameter given in ascii and will be converted to binary
     *            parameter_is_ascii = 0: Command parameter given binary and will be copied to binary output
     *            parameter_is_ascii = -1: Auto determination (1. assumption is true, 2. assumption is false in case of errors)
     * @return Cola-Binary telegram
     */
std::vector<uint8_t> sick_scan::ColaAsciiBinaryConverter::ColaTelegramToColaBinary(const sick_scan::SickLocColaTelegramMsg & cola_msg, int parameter_is_ascii)
{
  std::vector<uint8_t> cola_binary;
  cola_binary.reserve(64*1024);
  const uint8_t binary_stx = 0x02;
  const uint8_t binary_separator = 0x20;
  std::string command_type =  sick_scan::ColaParser::convertSopasCommand((sick_scan::ColaParser::COLA_SOPAS_COMMAND)cola_msg.command_type);
    
  // Encode the payload
  std::vector<uint8_t> binary_payload;
  binary_payload.reserve(64*1024);
  for(size_t n = 0; n < command_type.size(); n++)
    binary_payload.push_back(command_type[n]);
  binary_payload.push_back(binary_separator);
  for(size_t n = 0; n < cola_msg.command_name.size(); n++)
    binary_payload.push_back(cola_msg.command_name[n]);
  if(cola_msg.parameter.size() > 0)
  {
    binary_payload.push_back(binary_separator);
    for(size_t n = 0; n < cola_msg.parameter.size(); n++)
    {
      std::string parameter = cola_msg.parameter[n];
      if(parameter_is_ascii != 0) // parameter given in ascii, default: true
      {
        if ((parameter.size() % 2) != 0)
          parameter = "0" + cola_msg.parameter[n];
        for (size_t m = 1; m < parameter.size(); m += 2)
        {
          try
          {
            uint8_t val = static_cast<uint8_t>(std::stoul(parameter.substr(m - 1, 2), 0, 16) & 0xFF);
            binary_payload.push_back(val);
          }
          catch (const std::exception & exc)
          {
            if(parameter_is_ascii < 0) // auto determination, 1. assumption is true, 2. assumption is false in case of errors
            {
              return ColaTelegramToColaBinary(cola_msg, 0);
            }
            else // parameter_is_ascii is true, but parameter aren't -> error
            {
              ROS_ERROR_STREAM("## ERROR in ColaTelegramToColaBinary(): can't convert parameter value " << parameter.substr(m - 1, 2) << " to hex, exception " << exc.what());
              return cola_binary;
            }
          }
        }
      }
      else // copy parameter (parameter is byte array, default: false)
      {
        if(n > 0)
          binary_payload.push_back(binary_separator); // parameter is byte array -> payload including 0x20
        for (size_t m = 0; m < parameter.size(); m++)
        {
          uint8_t val = static_cast<uint8_t>(parameter[m] & 0xFF);
          binary_payload.push_back(val);
        }
      }
    }
  }
  
  // Concatenate Cola binary telegram: { 4 byte STX } + { 4 byte length } + { payload } + { checksum }
  for(size_t n = 0; n < 4; n++)
    cola_binary.push_back(binary_stx); // 4 byte STX
  uint32_t payload_length = binary_payload.size();
  for(int n = 24; n >= 0; n-=8)
    cola_binary.push_back((payload_length >> n) & 0xFF); // 4 byte payload length, big endian
  for(size_t n = 0; n < binary_payload.size(); n++)
    cola_binary.push_back(binary_payload[n]); // payload
  uint8_t checksum = CRC8XOR(binary_payload.data(), binary_payload.size());
  cola_binary.push_back(checksum); // CRC8XOR checksum
  
  return cola_binary;
}

/*!
 * @brief Converts and returns a Cola telegram from Cola-ASCII to Cola-Binary.
 * @param[in] cola_telegram Cola-Binary telegram
 * @param[in] parameter_to_ascii Conversion of command parameter to ascii (f.e. if true, a parameter 0x01 == ASCII-"1" will be converted to 0x31 (default), otherwise to 0x01)
 * @return Cola-ASCII telegram, f.e. { 0x02, 0x73, 0x4D, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63,  0x65,
 * 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x33, 0x20, 0x46, 0x34, 0x37, 0x32, 0x34, 0x37, 0x34, 0x34, 0x03 }
 * ("<STX>sMN SetAccessMode 3 F4724744<ETX>")
 */
std::vector<uint8_t> sick_scan::ColaAsciiBinaryConverter::ColaBinaryToColaAscii(const std::vector<uint8_t> & cola_telegram, bool parameter_to_ascii)
{
  const uint8_t binary_separator = 0x20;
  std::vector<uint8_t> cola_ascii;
  // Decode header
  if(!IsColaBinary(cola_telegram))
  {
    ROS_ERROR_STREAM("## ERROR in ColaBinaryToColaAscii(): cola telegram is not Cola-Binary");
    return cola_ascii;
  }
  uint32_t telegram_length = ColaBinaryTelegramLength(cola_telegram);
  if(telegram_length != cola_telegram.size())
  {
    ROS_ERROR_STREAM("## ERROR in ColaBinaryToColaAscii(): invalid cola telegram length (" << cola_telegram.size() << " byte, expected " << telegram_length << " byte)");
    return cola_ascii;
  }
  // Extract payload
  std::vector<uint8_t> binary_payload;
  binary_payload.reserve(cola_telegram.size());
  for(size_t n = 8; n < telegram_length - 1; n++)
    binary_payload.push_back(cola_telegram[n]);
  // Check crc checksum
  uint8_t checksum = CRC8XOR(binary_payload.data(), binary_payload.size());
  if(checksum != cola_telegram.back())
  {
    ROS_ERROR_STREAM("## ERROR in ColaBinaryToColaAscii(): invalid checksum (expected: " << checksum << ", received: " << cola_telegram.back() << ")");
    return cola_ascii;
  }
  // Find start of command parameter
  size_t parameter_start_idx = 0;
  while(parameter_start_idx < binary_payload.size() && binary_payload[parameter_start_idx] != binary_separator)
    parameter_start_idx++;
  parameter_start_idx++;
  while(parameter_start_idx < binary_payload.size() && binary_payload[parameter_start_idx] != binary_separator)
    parameter_start_idx++;
  parameter_start_idx++;
  // Concatenate Cola-ASCII telegram
  std::vector<uint8_t> stx = sick_scan::ColaParser::binarySTX();
  std::vector<uint8_t> etx = sick_scan::ColaParser::binaryETX();
  cola_ascii.reserve(2 * cola_telegram.size());
  // Copy STX
  for(size_t n = 0; n < stx.size(); n++)
    cola_ascii.push_back(stx[n]);
  if(parameter_to_ascii) // Convert command parameter to ASCII, default: true
  {
    // Copy command type and name
    for (size_t n = 0; n < binary_payload.size() && n < parameter_start_idx; n++)
      cola_ascii.push_back(binary_payload[n]);
    // Convert command parameter to ASCII
    for (size_t n = parameter_start_idx; n < binary_payload.size(); n++)
    {
      std::stringstream hexstring;
      hexstring << std::hex << (uint32_t) binary_payload[n];
      for (size_t m = 0; m < hexstring.str().size(); m++)
        cola_ascii.push_back(hexstring.str()[m]);
    }
  }
  else
  {
    // Copy command type, name and parameter
    for (size_t n = 0; n < binary_payload.size(); n++)
      cola_ascii.push_back(binary_payload[n]);
  }
  // Copy ETX
  for(size_t n = 0; n < etx.size(); n++)
    cola_ascii.push_back(etx[n]);
  return cola_ascii;
}

/*!
 * @brief Returns true for Cola-Binary, if a given telegram is Cola-Binary encoded and starts with 4 Bytes
 *        { 0x02, 0x02, 0x02, 0x02 }, otherwise false (Cola-ASCII telegram)
 * @param[in] cola_telegram Cola telegram (Cola-ASCII to Cola-Binary)
 * @return true for Cola-Binary, false for Cola-ASCII
 */
bool sick_scan::ColaAsciiBinaryConverter::IsColaBinary(const std::vector<uint8_t> & cola_telegram)
{
  const uint8_t binary_stx = 0x02;
  return cola_telegram.size() >= 4 && cola_telegram[0] == binary_stx && cola_telegram[1] == binary_stx
    && cola_telegram[2] == binary_stx && cola_telegram[3] == binary_stx;
}

/*!
 * @brief Decodes the header and returns the length of a Cola-Binary telegram
 * @param[in] cola_telegram Cola-Binary telegram
 * @return expected telegram length incl. header, payload and crc, or 0 in case of errors.
 */
uint32_t sick_scan::ColaAsciiBinaryConverter::ColaBinaryTelegramLength(const std::vector<uint8_t> & cola_telegram)
{
  if(cola_telegram.size() >= 8 && IsColaBinary(cola_telegram))
  {
    uint32_t payload_length = 0;
    for(size_t n = 4; n < 8; n++)
    {
      payload_length = payload_length << 8;
      payload_length |= ((cola_telegram[n]) & 0xFF);
    }
    return payload_length + 9; // 8 byte header + payload_length + 1 byte checksum
  }
  return 0;
}