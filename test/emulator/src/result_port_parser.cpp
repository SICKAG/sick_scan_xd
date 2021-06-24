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
#include <stdexcept>
#include "sick_scan/ros_wrapper.h"

#include "crc16ccitt_false.h"
#include "sick_scan/result_port_parser.h"

/** assert macro to verify result telegrams, throws an std::invalid_argument exception in case of assertion failures */
#define PARSE_ASSERT(assertion,info) sick_scan::ResultPortParser::parseAssert((assertion),(#assertion),info,__FILE__,__LINE__)

/*
 * Constructor of class ResultPortParser, which implements a parser for
 * result port telegrams for SIM Localization.
 * @param[in] frame_id frame_id of published ros messages (type SickLocResultPortTelegramMsg)
 */
sick_scan::ResultPortParser::ResultPortParser(const std::string & frame_id) : m_publish_frame_id(frame_id), m_result_port_telegram(), m_little_endian_payload(false)
{
}

/*
 * Shortcut to print error messages and to throw an std::invalid_argument exception in case of assertion failures
 */
void sick_scan::ResultPortParser::parseAssert(bool assertion, const std::string & assertion_msg, const std::string & info, const std::string & file, int line)
{
  if(!assertion)
  {
    std::stringstream exc_info;
    exc_info << "sick_scan::ResultPortParser: assertion \"" << assertion_msg << "\" failed, " << info << " (" << file << ":" << line;
    throw std::invalid_argument(exc_info.str());
  }
}

/*
 * Copies the next N=sizeof(value) bytes from binary_data to a value (number type), starting at binary_data[start_byte].
 * @param[in] binary_data binary data, at least start_byte+sizeof(value) byte binary data
 * @param[in] start_byte first byte to start copying from binary_data
 * @param[out] value destination
 * @param[in] info descriptional message, used in exception in case of errors
 * @param[in] little_endian true: binary_data encoded in little endian format, false (default): binary_data encoded in big endian format
 * @return number of bytes decoded
 * @throws std::invalid_argument in case of parse errors
 */
template<typename T> size_t sick_scan::ResultPortParser::copyBytesToValue(const std::vector<uint8_t> & binary_data, size_t start_byte, T & value, const std::string & info, bool little_endian)
{
  PARSE_ASSERT(binary_data.size() >= start_byte + sizeof(value), std::string("ResultPortParser::copyBytesToValue(): invalid size (") + info + ")");
  value = 0;
  if(little_endian) // Little endian: LSB first, MSB last
  {
    for (int n = (int)sizeof(value) - 1; n >= 0; n--)
    {
      value = ((value << 8) | (binary_data[start_byte + n]));
    }
  }
  else // Big endian: MSB first, LSB last
  {
    for (size_t n = 0; n < sizeof(value); n++)
    {
      value = ((value << 8) | (binary_data[start_byte + n]));
    }
  }
  return sizeof(value);
}

/*
 * Copies the next N=dst_array.size() bytes from binary_data to dst_array, starting at binary_data[start_byte].
 * @param[in] binary_data binary data, at least start_byte+sizeof(value) byte binary data
 * @param[in] start_byte first byte to start copying from binary_data
 * @param[out] dst_array destination
 * @param[in] info descriptional message, used in exception in case of errors
 * @return number of bytes decoded := dst_array.size()
 * @throws std::invalid_argument in case of parse errors
 */
template<typename T> size_t sick_scan::ResultPortParser::copyBytesToArray(const std::vector<uint8_t> & binary_data, size_t start_byte, std::vector<T> & dst_array, const std::string & info)
{
  PARSE_ASSERT(binary_data.size() >= start_byte + dst_array.size(), std::string("ResultPortParser::copyBytesToArray(): invalid size (") + info + ")");
  for (size_t n = 0; n < dst_array.size(); n++)
  {
    dst_array[n] = binary_data[start_byte + n];
  }
  return dst_array.size();
}

/*
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
uint16_t sick_scan::ResultPortParser::computeChecksum(const std::vector<uint8_t> & binary_data, bool binary_data_with_trailer)
{
  PARSE_ASSERT(binary_data_with_trailer == false || binary_data.size() >= 2, std::string("ResultPortParser::computeChecksum(): invalid input, binary_data.size() = ") + std::to_string(binary_data.size()));
  size_t len = binary_data_with_trailer ? (binary_data.size() - 2) : (binary_data.size());
  unsigned checksum1 = ::crc16ccitt_false_bit(0xFFFF, binary_data.data(), len);
  unsigned checksum2 = ::crc16ccitt_false_byte(0xFFFF, binary_data.data(), len);
  unsigned checksum3 = ::crc16ccitt_false_word(0xFFFF, binary_data.data(), len);
  PARSE_ASSERT(checksum1 == checksum2, std::string("ResultPortParser::computeChecksum(): ambigous checksums ") + std::to_string(checksum1) + "," + std::to_string(checksum2));
  PARSE_ASSERT(checksum1 == checksum3, std::string("ResultPortParser::computeChecksum(): ambigous checksums ") + std::to_string(checksum1) + "," + std::to_string(checksum3));
  return (uint16_t)(checksum1 & 0xFFFF);
}

/*
 * Returns true, if the PayloadType of a telegram_header indicates a little endian payload, or false otherwise.
 * @param[in] payload_type the PayloadType of a telegram_header
 * @return true for little endian payloads, false otherwise
 */
bool sick_scan::ResultPortParser::isLittleEndianPayload(uint16_t payload_type)
{
  return (payload_type == 0x06c2);
}

/*
 * Decodes the header of a result port telegram from binary data.
 * @param[in] binary_data binary data, at least start_byte+52 byte binary result port telegram header
 * @param[in] start_byte first byte to start decoding in binary_data
 * @param[out] telegram_header decoded result port telegram header
 * @return number of bytes decoded
 * @throws std::invalid_argument in case of parse errors
 */
size_t sick_scan::ResultPortParser::decodeResultPortHeader(const std::vector<uint8_t> & binary_data, size_t start_byte, sick_scan::SickLocResultPortHeaderMsg & telegram_header)
{
  size_t bytes_decoded = 0;

  // Decode MagicWord: Magic word SICK (0x53 0x49 0x43 0x4B). Size: 4 × UInt8 = 4 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_header.magicword, "Header.MagicWord");
  PARSE_ASSERT(telegram_header.magicword == 0x5349434B, std::string("ResultPortParser::decodeResultPortHeader(): invalid Header.MagicWord ") + std::to_string(telegram_header.magicword));

  // Decode Length: Length of telegram incl. header, payload, and trailer. Size: UInt32 = 4 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_header.length, "Header.Length");
  PARSE_ASSERT(telegram_header.length == 106, std::string("ResultPortParser::decodeResultPortHeader(): invalid Header.Length ") + std::to_string(telegram_header.length));

  // Decode PayloadType: Payload type 0x06c2 = Little Endian, 0x0642 = Big Endian. Size: UInt16 = 2 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_header.payloadtype, "Header.PayloadType");
  PARSE_ASSERT(telegram_header.payloadtype == 0x06c2 || telegram_header.payloadtype == 0x0642, std::string("ResultPortParser::decodeResultPortHeader(): invalid PayloadType ") + std::to_string(telegram_header.payloadtype));
  m_little_endian_payload = isLittleEndianPayload(telegram_header.payloadtype);
  
  
  // Decode PayloadVersion: Version of PayloadType structure. Size: UInt16 = 2 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_header.payloadversion, "Header.PayloadVersion");
  
  // Decode OrderNumber: Order number of the localization controller. Size: UInt32 = 4 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_header.ordernumber, "Header.OrderNumber");
  
  // Decode SerialNumber: Serial number of the localization controller. Size: UInt32 = 4 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_header.serialnumber, "Header.SerialNumber");
  
  // Decode FW_Version: Software version of the localization controller. Size: 20 × UInt8 = 20 byte
  telegram_header.fw_version = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  bytes_decoded += copyBytesToArray(binary_data, start_byte + bytes_decoded, telegram_header.fw_version, "Header.FW_Version");
  
  // Decode TelegramCounter: Telegram counter since last start-up. Size: UInt32 = 4 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_header.telegramcounter, "Header.TelegramCounter");
  
  // Decode SystemTime: Not used. Size: NTP = 8 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_header.systemtime, "Header.SystemTime");
  
  PARSE_ASSERT(bytes_decoded == 52, std::string("ResultPortParser::decodeResultPortHeader(): ") + std::to_string(bytes_decoded) + " bytes decoded, expected 52 byte");
  return bytes_decoded;
}

/*
 * Decodes the payload of a result port telegram from binary data.
 * @param[in] binary_data binary data, at least start_byte+52 byte binary result port telegram payload
 * @param[in] start_byte first byte to start decoding in binary_data
 * @param[out] telegram_payload decoded result port telegram payload
 * @return number of bytes decoded
 * @throws std::invalid_argument in case of parse errors
 */
size_t sick_scan::ResultPortParser::decodeResultPortPayload(const std::vector<uint8_t> & binary_data, size_t start_byte, sick_scan::SickLocResultPortPayloadMsg & telegram_payload)
{
  size_t bytes_decoded = 0;
  
  // Decode ErrorCode: ErrorCode 0: OK, ErrorCode 1: UNKNOWNERROR. Size: UInt16 = 2 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_payload.errorcode, "Payload.ErrorCode", m_little_endian_payload);
  
  // Decode ScanCounter: Counter of related scan data. Size: UInt32 = 4 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_payload.scancounter, "Payload.ScanCounter", m_little_endian_payload);
  
  // Decode Timestamp: Time stamp of the pose [ms]. The time stamp indicates the time at which the pose is calculated. Size: UInt32 = 4 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_payload.timestamp, "Payload.Timestamp", m_little_endian_payload);
  
  // Decode PoseX: Position X of the vehicle on the map in cartesian global coordinates [mm]. Size: Int32 = 4 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_payload.posex, "Payload.PoseX", m_little_endian_payload);
  
  // Decode PoseY: Position Y of the vehicle on the map in cartesian global coordinates [mm]. Size: Int32 = 4 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_payload.posey, "Payload.PoseY", m_little_endian_payload);
  
  // Decode PoseYaw: Orientation (yaw) of the vehicle on the map [mdeg] Size: Int32 = 4 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_payload.poseyaw, "Payload.PoseYaw", m_little_endian_payload);
  
  // Decode Reserved1: Reserved. Size: UInt32 = 4 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_payload.reserved1, "Payload.Reserved1", m_little_endian_payload);
  
  // Decode Reserved2: Reserved. Size: Int32 = 4 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_payload.reserved2, "Payload.Reserved2", m_little_endian_payload);
  
  // Decode Quality: Quality of pose [0 … 100], 1 = bad pose quality, 100 = good pose quality. Size: UInt8 = 1 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_payload.quality, "Payload.Quality", m_little_endian_payload);
  PARSE_ASSERT(telegram_payload.quality >= 0 && telegram_payload.quality <= 100, std::string("ResultPortParser::decodeResultPortPayload(): invalid Payload.Quality ") + std::to_string(telegram_payload.quality));
  
  // Decode OutliersRatio: Ratio of beams that cannot be assigned to the current reference map [%]. Size: UInt8 = 1 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_payload.outliersratio, "Payload.OutliersRatio", m_little_endian_payload);
  PARSE_ASSERT(telegram_payload.outliersratio >= 0 && telegram_payload.outliersratio <= 100, std::string("ResultPortParser::decodeResultPortPayload(): invalid Payload.OutliersRatio ") + std::to_string(telegram_payload.outliersratio));
  
  // Decode CovarianceX: Covariance c1 of the pose X [mm^2]. Size: Int32 = 4 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_payload.covariancex, "Payload.CovarianceX", m_little_endian_payload);
  
  // Decode CovarianceY: Covariance c5 of the pose Y [mm^2]. Size: Int32 = 4 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_payload.covariancey, "Payload.", m_little_endian_payload);
  
  // Decode CovarianceYaw: Covariance c9 of the pose Yaw [mdeg^2]. Size: Int32 = 4 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_payload.covarianceyaw, "Payload.CovarianceYaw", m_little_endian_payload);
  
  // Decode Reserved3: Reserved. Size: UInt64 = 8 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_payload.reserved3, "Payload.Reserved3", m_little_endian_payload);
  
  PARSE_ASSERT(bytes_decoded == 52, std::string("ResultPortParser::decodeResultPortPayload(): ") + std::to_string(bytes_decoded) + " bytes decoded, expected 52 byte");
  return bytes_decoded;
}

/*
 * Decodes the trailer of a result port telegram from binary data.
 * @param[in] binary_data binary data, at least start_byte+2 byte binary result port telegram trailer
 * @param[in] start_byte first byte to start decoding in binary_data
 * @param[out] telegram_trailer decoded result port telegram trailer
 * @return number of bytes decoded
 * @throws std::invalid_argument in case of parse errors
 */
size_t sick_scan::ResultPortParser::decodeResultPortTrailer(const std::vector<uint8_t> & binary_data, size_t start_byte, sick_scan::SickLocResultPortCrcMsg & telegram_trailer)
{
  size_t bytes_decoded = 0;
  
  // Decode Checksum: CRC16-CCITT over length of header (52 bytes) and payload (52 bytes) without 2 bytes of this trailer. Size: UInt16 = 2 byte
  bytes_decoded += copyBytesToValue(binary_data, start_byte + bytes_decoded, telegram_trailer.checksum, "Payload.Checksum");
  
  PARSE_ASSERT(bytes_decoded == 2, std::string("ResultPortParser::decodeResultPortTrailer(): ") + std::to_string(bytes_decoded) + " bytes decoded, expected 2 byte");
  return bytes_decoded;
}

/*
 * Decodes a result port telegram from binary data.
 * @param[in] binary_data binary data (106 byte binary result port telegram), f.e. { 0x53, 0x49, 0x43, 0x4B, 0x00, ... }
 * @return true if binary_data successfully decode, false otherwise.
 */
bool sick_scan::ResultPortParser::decode(const std::vector<uint8_t> & binary_data)
{
  try
  {
    size_t bytes_decoded = 0;
    PARSE_ASSERT(binary_data.size() >= 106, std::string("ResultPortParser::decode(): ") + std::to_string(binary_data.size()) + " byte binary data, expected 106 byte result port telegram");
    m_result_port_telegram.header.stamp = ROS::now();
    m_result_port_telegram.header.frame_id = m_publish_frame_id;
  
    // Decode result port header
    bytes_decoded += decodeResultPortHeader(binary_data, bytes_decoded, m_result_port_telegram.telegram_header);
  
    // Decode result port payload
    bytes_decoded += decodeResultPortPayload(binary_data, bytes_decoded, m_result_port_telegram.telegram_payload);
  
    // Decode result port crc
    bytes_decoded += decodeResultPortTrailer(binary_data, bytes_decoded, m_result_port_telegram.telegram_trailer);
    PARSE_ASSERT(bytes_decoded == m_result_port_telegram.telegram_header.length, std::string("ResultPortParser::decode(): ") + std::to_string(bytes_decoded) + " bytes decoded, expected " + std::to_string(m_result_port_telegram.telegram_header.length) + " byte (telegram_header.Length))");
  
    // Verify Checksum := CRC16-CCITT over length of header (52 bytes) and payload (52 bytes) without 2 bytes of this trailer. Size: UInt16 = 2 byte
    // Checksum details (See chapter 5.9 "About result port telegrams" of the operation manual for further details):
    // Width: 16 bits, Initial value = 0xFFFF, Truncated polynomial: 0x1021 CRC polynomials with orders of x16 + x12 + x5 + 1 (counted without the leading '1' bit)
    uint16_t checksum = computeChecksum(binary_data);
    PARSE_ASSERT(checksum == m_result_port_telegram.telegram_trailer.checksum, std::string("ResultPortParser::decode(): invalid checksum ") + std::to_string(m_result_port_telegram.telegram_trailer.checksum) + " decoded, expected checksum " + std::to_string(checksum));
  
    return true;
  }
  catch(const std::invalid_argument & exc)
  {
    ROS_ERROR_STREAM("## ERROR in sick_scan::ResultPortParser::decode(): exception " << exc.what());
  }
  return false;
}

/*
 * Encodes a value to binary data.
 * @param[in] value source
 * @param[out] binary_data binary data (destination buffer)
 * @param[in] little_endian true: binary_data encoded in little endian format, false (default): binary_data encoded in big endian format
 */
template<typename T> void sick_scan::ResultPortParser::encodePushValue(T value, std::vector<uint8_t> & binary_data, bool little_endian)
{
  if(little_endian) // Little endian: LSB first, MSB last
  {
    for (size_t n = 0; n < sizeof(value); n++)
    {
      binary_data.push_back(value & 0xFF);
      value = (value >> 8);
    }
  }
  else // Big endian: MSB first, LSB last
  {
    for (int n = sizeof(value) - 1; n >= 0; n--)
    {
      binary_data.push_back((value >> (8*n)) & 0xFF);
    }
  }
}

/*
 * Encodes the header of the result port telegram and append its binary data to binary_data (destination).
 * @param[in] telegram_header header of result port telegram
 * @param[out] binary_data destination buffer
 */
void sick_scan::ResultPortParser::encodeResultPortHeader(const sick_scan::SickLocResultPortHeaderMsg & telegram_header, std::vector<uint8_t> & binary_data)
{
  encodePushValue(telegram_header.magicword, binary_data);
  encodePushValue(telegram_header.length, binary_data);
  encodePushValue(telegram_header.payloadtype, binary_data);
  encodePushValue(telegram_header.payloadversion, binary_data);
  encodePushValue(telegram_header.ordernumber, binary_data);
  encodePushValue(telegram_header.serialnumber, binary_data);
  for (size_t n = 0; n < telegram_header.fw_version.size(); n++)
    binary_data.push_back(telegram_header.fw_version[n]);
  encodePushValue(telegram_header.telegramcounter, binary_data);
  encodePushValue(telegram_header.systemtime, binary_data);
}

/*
 * Encodes the payload of the result port telegram and append its binary data to binary_data (destination).
 * @param[in] telegram_payload payload of result port telegram
 * @param[out] binary_data destination buffer
 */
void sick_scan::ResultPortParser::encodeResultPortPayload(const sick_scan::SickLocResultPortPayloadMsg & telegram_payload, std::vector<uint8_t> & binary_data)
{
  encodePushValue(telegram_payload.errorcode, binary_data, m_little_endian_payload);
  encodePushValue(telegram_payload.scancounter, binary_data, m_little_endian_payload);
  encodePushValue(telegram_payload.timestamp, binary_data, m_little_endian_payload);
  encodePushValue(telegram_payload.posex, binary_data, m_little_endian_payload);
  encodePushValue(telegram_payload.posey, binary_data, m_little_endian_payload);
  encodePushValue(telegram_payload.poseyaw, binary_data, m_little_endian_payload);
  encodePushValue(telegram_payload.reserved1, binary_data, m_little_endian_payload);
  encodePushValue(telegram_payload.reserved2, binary_data, m_little_endian_payload);
  encodePushValue(telegram_payload.quality, binary_data, m_little_endian_payload);
  encodePushValue(telegram_payload.outliersratio, binary_data, m_little_endian_payload);
  encodePushValue(telegram_payload.covariancex, binary_data, m_little_endian_payload);
  encodePushValue(telegram_payload.covariancey, binary_data, m_little_endian_payload);
  encodePushValue(telegram_payload.covarianceyaw, binary_data, m_little_endian_payload);
  encodePushValue(telegram_payload.reserved3, binary_data, m_little_endian_payload);
}

/*
 * Encodes the checksum (trailer) of the result port telegram and append its binary data to binary_data (destination).
 * @param[in] checksum checksum (trailer) of result port telegram
 * @param[out] binary_data destination buffer
 */
void sick_scan::ResultPortParser::encodeResultPortTrailer(uint16_t checksum, std::vector<uint8_t> & binary_data)
{
  encodePushValue(checksum, binary_data);
}

/*
 * Encodes the result port telegram and returns its binary data.
 * @return binary data (106 byte binary result port telegram), f.e. { 0x53, 0x49, 0x43, 0x4B, 0x00, ... }
 */
std::vector<uint8_t> sick_scan::ResultPortParser::encode(void)
{
  std::vector<uint8_t> binary_data;
  binary_data.reserve(106);
  m_little_endian_payload = isLittleEndianPayload(m_result_port_telegram.telegram_header.payloadtype);
  encodeResultPortHeader(m_result_port_telegram.telegram_header, binary_data);
  encodeResultPortPayload(m_result_port_telegram.telegram_payload, binary_data);
  uint16_t checksum = computeChecksum(binary_data, false);
  m_result_port_telegram.telegram_trailer.checksum = checksum;
  encodeResultPortTrailer(checksum, binary_data);
  return binary_data;
}
