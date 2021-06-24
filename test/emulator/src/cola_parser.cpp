/*
 * @brief sim_loc_cola_parser parses and converts binary Cola telegrams to ros messages SickLocColaTelegramMsg
 * and vice versa.
 *
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
#include <cassert>
#include <boost/algorithm/string.hpp>

#include "sick_scan/cola_parser.h"

/*!
 * @brief static table to convert COLA_SOPAS_COMMAND to string, f.e. s_command_type_string[sRN]:="sRN", s_command_type_string[sRA]:="sRA" and so on
 */
const std::string sick_scan::ColaParser::s_command_type_string[MAX_COLA_COMMAND_NUMBER] =
  {
    "sINVALID", "sRN", "sRA", "sMN", "sAN", "sMA", "sWN", "sWA", "sEN", "sEA", "sSN", "sFA"
  };

/*!
 * @brief static map to convert COLA_SOPAS_COMMANDs from string to enum, f.e. s_command_type_map["sRN"]:=sRN, s_command_type_map["sRA"]:=sRA and so on
 */
const std::map<std::string, sick_scan::ColaParser::COLA_SOPAS_COMMAND> sick_scan::ColaParser::s_command_type_map =
  {
    {"", sINVALID}, {"sINVALID", sINVALID}, {"sRN", sRN}, {"sRA", sRA}, {"sMN", sMN}, {"sAN", sAN}, {"sMA", sMA}, {"sWN", sWN}, {"sWA", sWA}, {"sEN", sEN}, {"sEA", sEA}, {"sSN", sSN}, {"sFA", sFA}
  };

/*!
 * @brief All Cola-ACSII telegrams start with s_cola_ascii_start_tag := "<STX>" ("Start of TeXt")
 */
const std::string sick_scan::ColaParser::s_cola_ascii_start_tag = "<STX>";

/*!
 * @brief All Cola-ACSII telegrams start with s_cola_ascii_start_tag := "<ETX>" ("End of TeXt")
 */
const std::string sick_scan::ColaParser::s_cola_ascii_end_tag = "<ETX>";

/*!
 * @brief Create and returns a Cola telegram (type SickLocColaTelegramMsg).
 * @param[in] command_type One of the SOPAS Commands enumerated in COLA_SOPAS_COMMAND (sRN, sRA, sMN, sMA, or sWN)
 * @param[in] command_name Name of command like "SetAccessMode", "LocSetResultPoseEnabled", "LocRequestTimestamp", etc.
 * @param[in] parameter Optional list of command parameter
 * @return Cola telegram of type SickLocColaTelegramMsg
 */
sick_scan::SickLocColaTelegramMsg sick_scan::ColaParser::createColaTelegram(const COLA_SOPAS_COMMAND & command_type,
  const std::string & command_name, const std::vector<std::string> & parameter)
{
  sick_scan::SickLocColaTelegramMsg cola_telegram;
  cola_telegram.header.stamp = ROS::now();
  cola_telegram.command_type = command_type;
  cola_telegram.command_name = command_name;
  cola_telegram.parameter = parameter;
  return cola_telegram;
}

/*!
 * @brief Decodes and returns a Cola message of type SickLocColaTelegramMsg from a Cola-Binary telegram.
 * Note: If decoding fails (invalid binary telegram or parse error), command_type of the returned Cola telegram message
 * will be sINVALID.
 * @param[in] cola_binary Cola-Binary telegram, f.e. { 0x02, 0x73, 0x4D, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63,  0x65,
 * 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x33, 0x20, 0x46, 0x34, 0x37, 0x32, 0x34, 0x37, 0x34, 0x34, 0x03 }
 * @return Cola telegram message (type SickLocColaTelegramMsg)
 */
sick_scan::SickLocColaTelegramMsg sick_scan::ColaParser::decodeColaTelegram(const std::vector<uint8_t> & cola_binary)
{
  std::string cola_ascii = sick_scan::ColaAsciiBinaryConverter::ConvertColaAscii(cola_binary);
  return decodeColaTelegram(cola_ascii);
}

/*!
 * @brief Converts and returns a Cola message of type SickLocColaTelegramMsg from a Cola-ASCII telegram.
 * @param[in] cola_ascii Cola-ASCII telegram, f.e. "<STX>sMN LocRequestTimestamp<ETX>"
 * @return Cola telegram message (type SickLocColaTelegramMsg)
 */
sick_scan::SickLocColaTelegramMsg sick_scan::ColaParser::decodeColaTelegram(const std::string & cola_ascii)
{
  // Check and remove start and end tags ("<STX>" and "<ETX>")
  std::string cola_ascii_cmd;
  if (cola_ascii.size() > s_cola_ascii_start_tag.size() + s_cola_ascii_end_tag.size()
    && cola_ascii.substr(0, s_cola_ascii_start_tag.size()) == s_cola_ascii_start_tag
    && cola_ascii.substr(cola_ascii.size() - s_cola_ascii_end_tag.size()) == s_cola_ascii_end_tag)
  {
    cola_ascii_cmd = cola_ascii.substr(s_cola_ascii_start_tag.size(), cola_ascii.size() - s_cola_ascii_start_tag.size() - s_cola_ascii_end_tag.size());
  }
  else
  {
    cola_ascii_cmd = cola_ascii;
  }
  // Split in command_type, command_name and optional parameter by spaces
  std::vector<std::string> cola_parts;
  boost::split(cola_parts, cola_ascii_cmd, boost::algorithm::is_space());
  if(cola_parts.size() < 2) // at least command_type and command_name required
  {
    ROS_WARN_STREAM("## ERROR Parse error in ColaParser::decodeColaTelegram(\"" << cola_ascii_cmd << "\"): to few arguments, at least command_type and command_name required");
    return createColaTelegram(sINVALID, "");
  }
  // Convert command_type from string to COLA_SOPAS_COMMAND
  sick_scan::ColaParser::COLA_SOPAS_COMMAND command_type = convertSopasCommand(cola_parts[0]);
  if(command_type == sINVALID)
  {
    ROS_WARN_STREAM("## ERROR Parse error in ColaParser::decodeColaTelegram(\"" << cola_ascii_cmd << "\"): invalid command_type \"" << cola_parts[0] << "\"");
    return createColaTelegram(sINVALID, "");
  }
  // Check command_name
  if(cola_parts[1].empty())
  {
    ROS_WARN_STREAM("## ERROR Parse error in ColaParser::decodeColaTelegram(\"" << cola_ascii_cmd << "\"): invalid command_name \"" << cola_parts[1] << "\"");
    return createColaTelegram(sINVALID, "");
  }
  // Append command parameter
  sick_scan::SickLocColaTelegramMsg cola_telegram = createColaTelegram(command_type, cola_parts[1]);
  if(cola_parts.size() > 2)
  {
    cola_telegram.parameter.reserve(cola_parts.size() - 2);
    for(size_t n = 2; n < cola_parts.size(); n++)
      cola_telegram.parameter.push_back(cola_parts[n]);
  }
  return cola_telegram;
}

/*!
 * @brief Encodes and returns a Cola Binary telegram from ros message SickLocColaTelegramMsg.
 * @param[in] cola_telegram Cola telegram, f.e. createColaTelegram(sMN, "SetAccessMode", {"3", "F4724744"})
 * @return Cola-Binary telegram, f.e. { 0x02, 0x73, 0x4D, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63,  0x65,
 * 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x33, 0x20, 0x46, 0x34, 0x37, 0x32, 0x34, 0x37, 0x34, 0x34, 0x03 }
 */
std::vector<uint8_t> sick_scan::ColaParser::encodeColaTelegram(const sick_scan::SickLocColaTelegramMsg & cola_telegram, bool parameter_is_ascii)
{
  assert(cola_telegram.command_type > sINVALID && cola_telegram.command_type < MAX_COLA_COMMAND_NUMBER);
  std::string cola_ascii;
  cola_ascii.reserve(64*1024);
  cola_ascii += s_cola_ascii_start_tag;
  cola_ascii += convertSopasCommand((COLA_SOPAS_COMMAND)cola_telegram.command_type);
  cola_ascii += " ";
  cola_ascii += cola_telegram.command_name;
  for(size_t n = 0; n < cola_telegram.parameter.size(); n++)
  {
    if( n == 0 || parameter_is_ascii)
      cola_ascii += " ";
    cola_ascii += cola_telegram.parameter[n];
  }
  cola_ascii += s_cola_ascii_end_tag;
  return sick_scan::ColaAsciiBinaryConverter::ConvertColaAscii(cola_ascii);
}

/*!
 * @brief Encodes and returns a Cola telegram.
 * @param[in] command_type One of the SOPAS Commands enumerated in COLA_SOPAS_COMMAND (sRN, sRA, sMN, sMA, or sWN)
 * @param[in] command_name Name of command like "SetAccessMode", "LocSetResultPoseEnabled", "LocRequestTimestamp", etc.
 * @param[in] parameter Optional list of command parameter
 * @return Cola-Binary telegram
 */
std::vector<uint8_t> sick_scan::ColaParser::encodeColaTelegram(const COLA_SOPAS_COMMAND & command_type, const std::string & command_name,
  const std::vector<std::string> & parameter, bool parameter_is_ascii)
{
  return encodeColaTelegram(createColaTelegram(command_type, command_name, parameter), parameter_is_ascii);
}

/*!
 * @brief Converts and returns a COLA_SOPAS_COMMAND to string.
 * Example: convertSopasCommand(sMN) returns "sMN".
 * @param[in] command_type One of the SOPAS Commands enumerated in COLA_SOPAS_COMMAND (sRN, sRA, sMN, sMA, or sWN)
 * @return COLA_SOPAS_COMMAND as string.
 */
std::string sick_scan::ColaParser::convertSopasCommand(sick_scan::ColaParser::COLA_SOPAS_COMMAND command_type)
{
  return s_command_type_string[command_type];
}

/*!
 * @brief Converts and returns a COLA_SOPAS_COMMAND from string.
 * Example: convertSopasCommand("sMN") returns sMN.
 * @param[in] sopas_command One of the SOPAS commands ("sRN", "sRA", "sMN", "sMA", or "sWN")
 * @return COLA_SOPAS_COMMAND from string.
 */
sick_scan::ColaParser::COLA_SOPAS_COMMAND sick_scan::ColaParser::convertSopasCommand(const std::string & sopas_command)
{
  std::map<std::string, sick_scan::ColaParser::COLA_SOPAS_COMMAND>::const_iterator iter_command_type = s_command_type_map.find(sopas_command);
  sick_scan::ColaParser::COLA_SOPAS_COMMAND command_type = (iter_command_type != s_command_type_map.cend()) ? (iter_command_type->second) : sINVALID;
  return command_type;
}

/*!
 * Converts and returns the parameter of a cola ascii telegram into a numeric value.
 * @param[in] cola_arg parameter of a cola ascii telegram
 * @param[in] base numeric base (10 for decimal values, 16 for hex strings or -1 for autodetection with base 10 in case of +/-sign, otherwise hex)
 * @param[in] default_value default value returned in case of parse errors
 * @return parameter converted to integer value
 */
int32_t sick_scan::ColaParser::convertColaArg(const std::string & cola_arg, int base, int32_t default_value)
{
  try
  {
    if(base < 0)
      base = ((cola_arg.find_first_of("+-") != std::string::npos) ? 10 : 16); // base 10 if +/-sign in cola_arg, otherwise hex
    return std::stoi(cola_arg, 0, base);
  }
  catch(const std::exception & exc)
  {
    ROS_WARN_STREAM("## ERROR ColaParser::convertColaArg(" << cola_arg << ") failed, exception " << exc.what());
  }
  return default_value;
}

/*!
 * Converts and returns the parameter of a cola ascii telegram into a numeric value.
 * @param[in] cola_arg parameter of a cola ascii telegram
 * @param[in] base numeric base (10 for decimal values, 16 for hex strings or -1 for autodetection with base 10 in case of +/-sign, otherwise hex)
 * @param[in] default_value default value returned in case of parse errors
 * @return parameter converted to integer value
 */
uint32_t sick_scan::ColaParser::convertColaArg(const std::string & cola_arg, int base, uint32_t default_value)
{
  try
  {
    if(base < 0)
      base = ((cola_arg.find_first_of("+-") != std::string::npos) ? 10 : 16); // base 10 if +/-sign in cola_arg, otherwise hex
    return (uint32_t)std::stoul(cola_arg, 0, base);
  }
  catch(const std::exception & exc)
  {
    ROS_WARN_STREAM("## ERROR ColaParser::convertColaArg(" << cola_arg << ") failed, exception " << exc.what());
  }
  return default_value;
}

/*!
 * Converts and returns the parameter of a cola ascii response into a boolean value.
 * @param[in] cola_response_arg parameter of a cola ascii response
 * @param[in] default_value default value returned in case of parse errors
 * @return parameter converted to boolean value
 */
bool sick_scan::ColaParser::convertColaResponseBool(const std::string & cola_response_arg, bool default_value)
{
  return ((sick_scan::ColaParser::convertColaArg(cola_response_arg, 10, (default_value ? 1 : 0)) > 0) ? true : false);
}
