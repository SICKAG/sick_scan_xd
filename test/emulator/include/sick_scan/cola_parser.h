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
#ifndef __SIM_LOC_COLA_PARSER_H_INCLUDED
#define __SIM_LOC_COLA_PARSER_H_INCLUDED

#include "sick_scan/ros_wrapper.h"
#include "sick_scan/cola_converter.h"

namespace sick_scan
{
  /*!
   * @brief class ColaParser parses and converts binary Cola telegrams to ros messages SickLocColaTelegramMsg
   * and vice versa.
   *
   * See Operation-Instruction-v1.1.0.241R.pdf, chapter 5.8 "About CoLa-A telegrams", page 46-48,
   * Telegram-Listing-v1.1.0.241R.pdf, chapter 2.3.9 "Command: LocRequestTimestamp", page 21, and
   * Technical_information_Telegram_Listing_NAV_LOC_en_IM0076556.PDF for further details about
   * Cola telegrams.
   */
  class ColaParser
  {
  public:
  
    /*!
     * @brief Enumeration of SOPAS commands in cola telegrams:
     * The command_type in SickLocColaTelegramMsg is one of the following SOPAS Commands:
     * sINVALID (0, uninitialized, command_type should never have this value)
     * sRN (1, Read by name request), or
     * sRA (2, Read by name response), or
     * sMN (3, Method by name request), or
     * sMA (4, Method by name response), or
     * sWN (5, Write by name request)
     */
    typedef enum COLA_SOPAS_COMMAND_ENUM
    {
       sINVALID = 0, ///< uninitialized, command_type should never have this value)
       sRN = 1,      ///< Read by name (request)
       sRA = 2,      ///< Read by name (response)
       sMN = 3,      ///< Method by name (request)
       sAN = 4,      ///< Response to sMN
       sMA = 5,      ///< Method by name (response)
       sWN = 6,      ///< Write by name (request)
       sWA = 7,      ///< Write answer (response)
       sEN = 8,      ///< Event by name (request)
       sEA = 9,      ///< Event answer (response)
       sSN = 10,     ///< Answer (response)
       sFA = 11,     ///< Error Answer with error code (response)
       MAX_COLA_COMMAND_NUMBER ///< Number of possible COLA_SOPAS_COMMANDs incl. invalid command
    } COLA_SOPAS_COMMAND;
  
    /*!
     * @brief Creates and returns a Cola telegram (type SickLocColaTelegramMsg).
     * @param[in] command_type One of the SOPAS Commands enumerated in COLA_SOPAS_COMMAND (sRN, sRA, sMN, sMA, or sWN)
     * @param[in] command_name Name of command like "SetAccessMode", "LocSetResultPoseEnabled", "LocRequestTimestamp", etc.
     * @param[in] parameter Optional list of command parameter
     * @return Cola telegram of type SickLocColaTelegramMsg
     */
    static sick_scan::SickLocColaTelegramMsg createColaTelegram(const COLA_SOPAS_COMMAND & command_type,
      const std::string & command_name, const std::vector<std::string> & parameter = std::vector<std::string>());
  
    /*!
     * @brief Decodes and returns a Cola message of type SickLocColaTelegramMsg from a Cola-Binary telegram.
     * Note: If decoding fails (invalid binary telegram or parse error), command_type of the returned Cola telegram message
     * will be sINVALID.
     * @param[in] cola_binary Cola-Binary telegram, f.e. { 0x02, 0x73, 0x4D, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63,  0x65,
     * 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x33, 0x20, 0x46, 0x34, 0x37, 0x32, 0x34, 0x37, 0x34, 0x34, 0x03 }
     * @return Cola telegram message (type SickLocColaTelegramMsg)
     */
    static sick_scan::SickLocColaTelegramMsg decodeColaTelegram(const std::vector<uint8_t> & cola_binary);
  
    /*!
     * @brief Converts and returns a Cola message of type SickLocColaTelegramMsg from a Cola-ASCII telegram.
     * @param[in] cola_ascii Cola-ASCII telegram, f.e. "<STX>sMN LocRequestTimestamp<ETX>"
     * @return Cola telegram message (type SickLocColaTelegramMsg)
     */
    static sick_scan::SickLocColaTelegramMsg decodeColaTelegram(const std::string & cola_ascii);
  
    /*!
     * @brief Encodes and returns a Cola Binary telegram from ros message SickLocColaTelegramMsg.
     * @param[in] cola_telegram Cola telegram, f.e. createColaTelegram(sMN, "SetAccessMode", {"3", "F4724744"})
     * @return Cola-Binary telegram, f.e. { 0x02, 0x73, 0x4D, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63,  0x65,
     * 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x33, 0x20, 0x46, 0x34, 0x37, 0x32, 0x34, 0x37, 0x34, 0x34, 0x03 }
     */
    static std::vector<uint8_t> encodeColaTelegram(const sick_scan::SickLocColaTelegramMsg & cola_telegram, bool parameter_is_ascii = true);
  
    /*!
     * @brief Encodes and returns a Cola telegram.
     * @param[in] command_type One of the SOPAS Commands enumerated in COLA_SOPAS_COMMAND (sRN, sRA, sMN, sMA, or sWN)
     * @param[in] command_name Name of command like "SetAccessMode", "LocSetResultPoseEnabled", "LocRequestTimestamp", etc.
     * @param[in] parameter Optional list of command parameter
     * @return Cola-Binary telegram
     */
    static std::vector<uint8_t> encodeColaTelegram(const COLA_SOPAS_COMMAND & command_type, const std::string & command_name,
      const std::vector<std::string> & parameter = std::vector<std::string>(), bool parameter_is_ascii = true);
  
    /*!
     * @brief Returns the "start of text" tag in a Cola ASCII command, i.e. "<STX>".
     * @return "<STX>"
     */
    static const std::string & asciiSTX(void) { return s_cola_ascii_start_tag; }
  
    /*!
     * @brief Returns the "end of text" tag in a Cola ASCII command, i.e. "<ETX>".
     * @return "<ETX>"
     */
    static const std::string & asciiETX(void) { return s_cola_ascii_end_tag; }
  
    /*!
     * @brief Returns the binary "start of text" tag in a Cola-Binary command, i.e. {0x02}.
     * @return {0x02}
     */
    static std::vector<uint8_t> binarySTX(void){ return sick_scan::ColaAsciiBinaryConverter::ConvertColaAscii(asciiSTX()); }
  
    /*!
     * @brief Returns the binary "end of text" tag in a Cola-Binary command, i.e. {0x03}.
     * @return {0x03}
     */
    static std::vector<uint8_t> binaryETX(void){ return sick_scan::ColaAsciiBinaryConverter::ConvertColaAscii(asciiETX()); }
  
    /*!
     * @brief Converts and returns a COLA_SOPAS_COMMAND to string.
     * Example: convertSopasCommand(sMN) returns "sMN".
     * @param[in] command_type One of the SOPAS Commands enumerated in COLA_SOPAS_COMMAND (sRN, sRA, sMN, sMA, or sWN)
     * @return COLA_SOPAS_COMMAND as string.
     */
    static std::string convertSopasCommand(COLA_SOPAS_COMMAND command_type);
  
    /*!
     * @brief Converts and returns a COLA_SOPAS_COMMAND from string.
     * Example: convertSopasCommand("sMN") returns sMN.
     * @param[in] sopas_command One of the SOPAS commands ("sRN", "sRA", "sMN", "sMA", or "sWN")
     * @return COLA_SOPAS_COMMAND from string.
     */
    static COLA_SOPAS_COMMAND convertSopasCommand(const std::string & sopas_command);

    /*!
     * Converts and returns the parameter of a cola ascii telegram into a numeric value.
     * @param[in] cola_arg parameter of a cola ascii telegram
     * @param[in] base numeric base (10 for decimal values, 16 for hex strings or -1 for autodetection with base 10 in case of +/-sign, otherwise hex)
     * @param[in] default_value default value returned in case of parse errors
     * @return parameter converted to integer value
     */
    static int32_t convertColaArg(const std::string & cola_arg, int base = 10, int32_t default_value = 0);
    
    /*!
     * Converts and returns the parameter of a cola ascii telegram into an unsigned numeric value.
     * @param[in] cola_arg parameter of a cola ascii telegram
     * @param[in] base numeric base (10 for decimal values, 16 for hex strings or -1 for autodetection with base 10 in case of +/-sign, otherwise hex)
     * @param[in] default_value default value returned in case of parse errors
     * @return parameter converted to integer value
     */
    static uint32_t convertColaArg(const std::string & cola_arg, int base = 10, uint32_t default_value = 0);
    
    /*!
     * Converts and returns the parameter of a cola ascii response into a boolean value.
     * @param[in] cola_response_arg parameter of a cola ascii response
     * @param[in] default_value default value returned in case of parse errors
     * @return parameter converted to boolean value
     */
    static bool convertColaResponseBool(const std::string & cola_response_arg, bool default_value);

  protected:
  
    static const std::string s_command_type_string[MAX_COLA_COMMAND_NUMBER]; ///< static table to convert COLA_SOPAS_COMMAND to string, f.e. s_command_type_string[sRN]:="sRN", s_command_type_string[sRA]:="sRA" and so on
    static const std::map<std::string, COLA_SOPAS_COMMAND> s_command_type_map; ///< static map to convert COLA_SOPAS_COMMANDs from string to enum, f.e. s_command_type_map["sRN"]:=sRN, s_command_type_map["sRA"]:=sRA and so on
    static const std::string s_cola_ascii_start_tag; ///< All Cola-ACSII telegrams start with s_cola_ascii_start_tag := "<STX>" ("Start of TeXt")
    static const std::string s_cola_ascii_end_tag; ///< All Cola-ACSII telegrams start with s_cola_ascii_start_tag := "<ETX>" ("End of TeXt")
    
  }; // class ColaParser
  
} // namespace sick_scan
#endif // __SIM_LOC_COLA_PARSER_H_INCLUDED
