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
#ifndef __SIM_LOC_COLA_CONVERTER_H_INCLUDED
#define __SIM_LOC_COLA_CONVERTER_H_INCLUDED

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "sick_scan/ros_wrapper.h"

namespace sick_scan
{
  /*!
   * @brief class ColaAsciiBinaryConverter converts between Cola-ASCII and Cola-Binary telegrams.
   * See Operation-Instruction-v1.1.0.241R.pdf, chapter 5.8 "About CoLa-A telegrams", page 46-48,
   * Telegram-Listing-v1.1.0.241R.pdf, chapter 2.3.9 "Command: LocRequestTimestamp", page 21, and
   * Technical_information_Telegram_Listing_NAV_LOC_en_IM0076556.PDF for further details about
   * Cola telegrams.
   */
  class ColaAsciiBinaryConverter
  {
  public:
    
    /*!
     * @brief Converts and returns a Cola-ASCII telegram to string.
     * @param[in] cola_telegram Cola-ASCII telegram, starting with 0x02 and ending with 0x03
     * @return Cola-ASCII string, f.e. "<STX>sMN SetAccessMode 3 F4724744<ETX>"
     */
    static std::string ConvertColaAscii(const std::vector<uint8_t> & cola_telegram);
  
    /*!
     * @brief Converts and returns a Cola telegram from Cola-ASCII to Cola-Binary.
     * @param[in] cola_telegram Cola-ASCII string, f.e. "<STX>sMN SetAccessMode 3 F4724744<ETX>"
     * @return Cola-ASCII telegram, f.e. { 0x02, 0x73, 0x4D, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63,  0x65,
     * 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x33, 0x20, 0x46, 0x34, 0x37, 0x32, 0x34, 0x37, 0x34, 0x34, 0x03 }
     */
    static std::vector<uint8_t> ConvertColaAscii(const std::string & cola_telegram);
  
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
    static std::vector<uint8_t> ColaTelegramToColaBinary(const sick_scan::SickLocColaTelegramMsg & cola_telegram, int parameter_is_ascii = -1);

    /*!
     * @brief Converts and returns a Cola telegram from Cola-ASCII to Cola-Binary.
     * @param[in] cola_telegram Cola-ASCII telegram, f.e. { 0x02, 0x73, 0x4D, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63,  0x65,
     * 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x33, 0x20, 0x46, 0x34, 0x37, 0x32, 0x34, 0x37, 0x34, 0x34, 0x03 }
     * ("<STX>sMN SetAccessMode 3 F4724744<ETX>")
     * @param[in] parameter_is_ascii Command parameter given in ascii (f.e. if 1, a parameter 0x31 == ASCII-"1" will be converted to 0x01 (default), otherwise to 0x01)
     * @param[in] parameter_is_ascii Command parameter given in ascii (f.e. if 1, a parameter 0x31 == ASCII-"1" will be converted to 0x01 (default), otherwise to 0x01)
     *            parameter_is_ascii = 1: Command parameter given in ascii and will be converted to binary
     *            parameter_is_ascii = 0: Command parameter given binary and will be copied to binary output
     *            parameter_is_ascii = -1: Auto determination (1. assumption is true, 2. assumption is false in case of errors)
     * @return Cola-Binary telegram
     */
    static std::vector<uint8_t> ColaAsciiToColaBinary(const std::vector<uint8_t> & cola_telegram, int parameter_is_ascii = -1);

    /*!
     * @brief Converts and returns a Cola telegram from Cola-ASCII to Cola-Binary.
     * @param[in] cola_telegram Cola-Binary telegram
     * @param[in] parameter_to_ascii Conversion of command parameter to ascii (f.e. if true, a parameter 0x01 will be converted to 0x31 == ASCII-"1" (default), otherwise to 0x01)
     * @return Cola-ASCII telegram, f.e. { 0x02, 0x73, 0x4D, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63,  0x65,
     * 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x33, 0x20, 0x46, 0x34, 0x37, 0x32, 0x34, 0x37, 0x34, 0x34, 0x03 }
     * ("<STX>sMN SetAccessMode 3 F4724744<ETX>")
     */
    static std::vector<uint8_t> ColaBinaryToColaAscii(const std::vector<uint8_t> & cola_telegram, bool parameter_to_ascii = true);
  
    /*!
     * @brief Returns true for Cola-Binary, if a given telegram is Cola-Binary encoded and starts with 4 Bytes
     *        { 0x02, 0x02, 0x02, 0x02 }, otherwise false (Cola-ASCII telegram)
     * @param[in] cola_telegram Cola telegram (Cola-ASCII to Cola-Binary)
     * @return true for Cola-Binary, false for Cola-ASCII
     */
    static bool IsColaBinary(const std::vector<uint8_t> & cola_telegram);

    /*!
     * @brief Decodes the header and returns the length of a Cola-Binary telegram
     * @param[in] cola_telegram Cola-Binary telegram
     * @return expected telegram length incl. header, payload and crc, or 0 in case of errors.
     */
    static uint32_t ColaBinaryTelegramLength(const std::vector<uint8_t> & cola_telegram);
    
  protected:
    
    /*
     * member data
     */
    
    static const std::string s_ascii_table[256]; ///< static ascii table to convert binary to ascii, f.e. s_ascii_table[0x02]:="<STX>", s_ascii_table[0x03]:="<ETX>", s_ascii_table[0x41]:="A" and so on
    static const std::map<std::string, uint8_t> s_ascii_map; ///< static ascii map to convert ascii to binary, f.e. s_ascii_map["<STX>"]:=0x02, s_ascii_map["<ETX>"]:=0x03, s_ascii_map["A"]:=0x41 and so on
    
  }; // class ColaAsciiBinaryConverter
  
} // namespace sick_scan
#endif // __SIM_LOC_COLA_CONVERTER_H_INCLUDED
