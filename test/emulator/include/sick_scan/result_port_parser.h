/*
 * @brief sim_loc_result_port_parser implements a parser for
 * result port telegrams for SIM Localization.
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
#ifndef __SIM_LOC_RESULT_PORT_PARSER_H_INCLUDED
#define __SIM_LOC_RESULT_PORT_PARSER_H_INCLUDED

#include "sick_scan/ros_wrapper.h"

namespace sick_scan
{
  /*!
   * class ResultPortParser implements a parser for
   * result port telegrams for SIM Localization.
   */
  class ResultPortParser
  {
  public:
    
    /*!
     * Constructor
     * @param[in] frame_id frame_id of published ros messages (type SickLocResultPortTelegramMsg)
     */
    ResultPortParser(const std::string & frame_id = "");
  
    /*!
     * Decodes a result port telegram from binary data.
     * @param[in] binary_data binary data (106 byte binary result port telegram), f.e. { 0x53, 0x49, 0x43, 0x4B, 0x00, ... }
     * @return true if binary_data successfully decode, false otherwise.
     */
    virtual bool decode(const std::vector<uint8_t> & binary_data);
  
    /*!
     * Encodes the result port telegram and returns its binary data.
     * @return binary data (106 byte binary result port telegram), f.e. { 0x53, 0x49, 0x43, 0x4B, 0x00, ... }
     */
    virtual std::vector<uint8_t> encode(void);
  
    /*!
     * Returns the result port telegram.
     */
    virtual sick_scan::SickLocResultPortTelegramMsg & getTelegramMsg(void) { return m_result_port_telegram; }
  
    /*!
     * Returns the result port telegram.
     */
    virtual const sick_scan::SickLocResultPortTelegramMsg & getTelegramMsg(void) const { return m_result_port_telegram; }
    
  protected:

    /*!
     * Shortcut to print error messages and to throw an std::invalid_argument exception in case of assertion failures
     */
    static void parseAssert(bool assertion, const std::string & assertion_msg, const std::string & info, const std::string & file, int line);

    /*!
     * Copies the next N=sizeof(value) bytes from binary_data to a value (number type), starting at binary_data[start_byte].
     * @param[in] binary_data binary data, at least start_byte+sizeof(value) byte binary data
     * @param[in] start_byte first byte to start copying from binary_data
     * @param[out] value destination
     * @param[in] info descriptional message, used in exception in case of errors
     * @param[in] little_endian true: binary_data encoded in little endian format, false (default): binary_data encoded in big endian format
     * @return number of bytes decoded := sizeof(value)
     * @throws std::invalid_argument in case of parse errors
     */
    template<typename T> size_t copyBytesToValue(const std::vector<uint8_t> & binary_data, size_t start_byte, T & value, const std::string & info = "", bool little_endian = false);
  
    /*!
     * Copies the next N=dst_array.size() bytes from binary_data to dst_array, starting at binary_data[start_byte].
     * @param[in] binary_data binary data, at least start_byte+sizeof(value) byte binary data
     * @param[in] start_byte first byte to start copying from binary_data
     * @param[out] dst_array destination
     * @param[in] info descriptional message, used in exception in case of errors
     * @return number of bytes decoded := dst_array.size()
     * @throws std::invalid_argument in case of parse errors
     */
    template<typename T> size_t copyBytesToArray(const std::vector<uint8_t> & binary_data, size_t start_byte, std::vector<T> & dst_array, const std::string & info = "");

    /*!
     * Computes and returns the checksum of a result port telegram.
     *
     * Checksum := CRC16-CCITT over length of header (52 bytes) and payload (52 bytes) without 2 bytes of this trailer. Size: UInt16 = 2 byte
     * Checksum details (See chapter 5.9 "About result port telegrams" of the operation manual for further details):
     * Width: 16 bits
     * Truncated polynomial: 0x1021 CRC polynomials with orders of x16 + x12 + x5 + 1 (counted without the leading '1' bit)
     * Initial value = 0xFFFF
     *
     * Additional note: According to http://srecord.sourceforge.net/crc16-ccitt.html, CRC16-CCITT is specified by
     * Width = 16 bits, Truncated polynomial = 0x1021, Initial value = 0xFFFF, Input data is NOT reflected, Output CRC is NOT reflected.
     * This is often referred as "CRC-16/CCITT-FALSE" (because of "reflected_in/out=FALSE", width=16 poly=0x1021 init=0xffff refin=false refout=false),
     * in contrast to other flavors like "CRC-16/MCRF4XX" (width=16 poly=0x1021 init=0xffff refin=true refout=true).
     *
     * This CRC checksum uses the implementation by https://github.com/madler/crcany (sources under the zlib license,
     * permitting free commercial use) with algorithm "CRC-16/CCITT-FALSE" (crc16ccitt_false.c and crc16ccitt_false.h).
     * Other crc checksum algorithms may be used if required.
     *
     * @param[in] binary_data binary data of result port telegram
     * @param[in] binary_data_with_trailer true (default): binary_data (input) contains 2 byte trailer
     * @return CRC16 checksum
     */
    virtual uint16_t computeChecksum(const std::vector<uint8_t> & binary_data, bool binary_data_with_trailer = true);

    /*!
     * Returns true, if the PayloadType of a telegram_header indicates a little endian payload, or false otherwise.
     * @param[in] payload_type the PayloadType of a telegram_header
     * @return true for little endian payloads, false otherwise
     */
    virtual bool isLittleEndianPayload(uint16_t payload_type);

    /*!
     * Decodes the header of a result port telegram from binary data.
     * @param[in] binary_data binary data, at least start_byte+52 byte binary result port telegram header
     * @param[in] start_byte first byte to start decoding in binary_data
     * @param[out] telegram_header decoded result port telegram header
     * @return number of bytes decoded
     * @throws std::invalid_argument in case of parse errors
     */
    virtual size_t decodeResultPortHeader(const std::vector<uint8_t> & binary_data, size_t start_byte, sick_scan::SickLocResultPortHeaderMsg & telegram_header);

    /*!
     * Decodes the payload of a result port telegram from binary data.
     * @param[in] binary_data binary data, at least start_byte+52 byte binary result port telegram payload
     * @param[in] start_byte first byte to start decoding in binary_data
     * @param[out] telegram_payload decoded result port telegram payload
     * @return number of bytes decoded
     * @throws std::invalid_argument in case of parse errors
     */
    virtual size_t decodeResultPortPayload(const std::vector<uint8_t> & binary_data, size_t start_byte, sick_scan::SickLocResultPortPayloadMsg & telegram_payload);

    /*!
     * Decodes the trailer of a result port telegram from binary data.
     * @param[in] binary_data binary data, at least start_byte+2 byte binary result port telegram trailer
     * @param[in] start_byte first byte to start decoding in binary_data
     * @param[out] telegram_trailer decoded result port telegram trailer
     * @return number of bytes decoded
     * @throws std::invalid_argument in case of parse errors
     */
    virtual size_t decodeResultPortTrailer(const std::vector<uint8_t> & binary_data, size_t start_byte, sick_scan::SickLocResultPortCrcMsg & telegram_trailer);

    /*!
     * Encodes a value to binary data.
     * @param[in] value source
     * @param[out] binary_data binary data (destination buffer)
     * @param[in] little_endian true: binary_data encoded in little endian format, false (default): binary_data encoded in big endian format
     */
    template<typename T> void encodePushValue(T value, std::vector<uint8_t> & binary_data, bool little_endian = false);
    
    /*!
     * Encodes the header of the result port telegram and append its binary data to binary_data (destination).
     * @param[in] telegram_header header of result port telegram
     * @param[out] binary_data destination buffer
     */
    virtual void encodeResultPortHeader(const sick_scan::SickLocResultPortHeaderMsg & telegram_header, std::vector<uint8_t> & binary_data);

    /*!
     * Encodes the payload of the result port telegram and append its binary data to binary_data (destination).
     * @param[in] telegram_payload payload of result port telegram
     * @param[out] binary_data destination buffer
     */
    virtual void encodeResultPortPayload(const sick_scan::SickLocResultPortPayloadMsg & telegram_payload, std::vector<uint8_t> & binary_data);

    /*!
     * Encodes the checksum (trailer) of the result port telegram and append its binary data to binary_data (destination).
     * @param[in] checksum checksum (trailer) of result port telegram
     * @param[out] binary_data destination buffer
     */
    virtual void encodeResultPortTrailer(uint16_t checksum, std::vector<uint8_t> & binary_data);

    /*
     * member data
     */

    std::string m_publish_frame_id; ///< frame_id of published ros messages (type SickLocResultPortTelegramMsg)
    sick_scan::SickLocResultPortTelegramMsg m_result_port_telegram; ///< the result port telegram decoded from binary data
    bool m_little_endian_payload; ///< true if payload type is 0x06c2 (little endian), default: false (payload encoded in big endian format)
  
  };
  
} // namespace sick_scan
#endif // __SIM_LOC_RESULT_PORT_PARSER_H_INCLUDED
