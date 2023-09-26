/*
 * @brief sick_scan2 test_server_cola_msg implements the lidar specific messages,
 * i.e. message receiving and message creation to simulate lidar devices with
 * cola communication over tcp.
 *
 * Note: sick_scan2 test_server_cola_msg does not implement the functions of lidar sensors,
 * it just implements a simple message handling to test the sick_scan2 ros drivers.
 *
 * Copyright (C) 2020 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2020 SICK AG, Waldkirch
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
 *  Copyright 2020 SICK AG
 *  Copyright 2020 Ing.-Buero Dr. Michael Lehning
 *
 */
#include "sick_scan/binPrintf.hpp"
#include "sick_scan/tcp/colaa.hpp"
#include "sick_scan/tcp/colab.hpp"
#include "sick_scan/test_server/test_server_cola_msg.h"

static uint8_t calcPayloadChecksum(const std::vector<uint8_t> & payload)
{
  uint8_t checksum = 0;
	for (int i = 0; i < payload.size(); i++)
	{
		checksum = checksum ^ payload[i]; // XOR
	}
  return checksum;
}

static std::vector<uint8_t> encodeColaTelegram(const std::vector<uint8_t> & payload, bool is_binary)
{
  // Encode the telegram (cola-a or cola-b)
  std::vector<uint8_t> telegram;
  telegram.reserve(12 + payload.size());
  if(is_binary)
  {
    uint8_t checksum = calcPayloadChecksum(payload);
    for(int i = 0; i < 4; i++)
      telegram.push_back(0x02);
    uint32_t payload_length = payload.size();
    for(int i = 3; i >= 0; i--)
      telegram.push_back((payload_length >> (8 * i)) & 0xFF); // payload_length in big endian
    telegram.insert(telegram.end(), payload.begin(), payload.end());
    telegram.push_back(checksum);
  }
  else
  {
      telegram.push_back(0x02); // <STX> := 0x02
      telegram.insert(telegram.end(), payload.begin(), payload.end());
      telegram.push_back(0x03); // <ETX> := 0x03
  }
  return telegram;
}

static std::vector<uint8_t> encodeColaTelegram(const std::string & command, const std::vector<std::string> command_parameter, bool is_binary)
{
  std::vector<uint8_t> payload;
  payload.insert(payload.end(), command.begin(), command.end());
  for(int n = 0; n < command_parameter.size(); n++)
  {
    std::string parameter = command_parameter[n];
    if(parameter.size() > 254)
    {
      parameter.resize(254);
      ROS_WARN_STREAM("encodeColaTelegram(): resized " << parameter.size() << " byte parameter " << command_parameter[n] << " to " << parameter.size() << " byte parameter " << parameter);
    }
    if(is_binary)
    {
      if(n == 0)
        payload.push_back(0x20); // space
      payload.push_back(0);
      payload.push_back((uint8_t)(parameter.size() & 0xFF));
      payload.insert(payload.end(), parameter.begin(), parameter.end());
    }
    else
    {
      payload.push_back(0x20); // space
      UINT8 buffer[256];
      UINT16 len = colaa::addUINT8ToBuffer(buffer, (UINT8)(parameter.size() & 0xFF));
      for(int i = 0; i < len; i++)
        payload.push_back(buffer[i]);
      payload.push_back(0x20); // space
      payload.insert(payload.end(), parameter.begin(), parameter.end());
    }
  }
  std::vector<uint8_t> telegram = encodeColaTelegram(payload, true);
  return telegram;
}

static std::vector<uint8_t> encodeColaTelegram(const std::string & command, const std::vector<int64_t> & parameter_value, const std::vector<size_t> & parameter_size, bool is_binary)
{
  // Encode the payload (cola-a or cola-b)
  std::vector<uint8_t> payload;
  assert(parameter_value.size() == parameter_size.size());
  payload.reserve(command.size() + 16 * parameter_value.size() + 16);
  payload.insert(payload.end(), command.begin(), command.end());
  for(int n = 0; n < parameter_value.size(); n++)
  {
    if(is_binary)
    {
      if(n == 0)
        payload.push_back(0x20); // space
      size_t len = parameter_size[n];
      int64_t value = parameter_value[n];
      for(int i = len - 1; i >= 0; i--)
        payload.push_back((value >> (8 * i)) & 0xFF); // Cola-B always in big endian
    }
    else
    {
      payload.push_back(0x20); // space
      UINT8 buffer[256];
      UINT16 len = colaa::addUINT32ToBuffer(buffer, (UINT32)(parameter_value[n] & 0xFFFFFFFF));
      for(int i = 0; i < len; i++)
        payload.push_back(buffer[i]);
    }
  }
  // Encode the telegram (cola-a or cola-b)
  std::vector<uint8_t> telegram = encodeColaTelegram(payload, is_binary);
  return telegram;
}

static bool receive(sick_scan_xd::ServerSocket & tcp_client_socket, size_t nr_bytes, bool little_endian, std::vector<uint8_t> & value, bool read_blocking = true)
{
  value.clear();
  value.resize(nr_bytes);
  if (!tcp_client_socket.read(nr_bytes, value.data(), read_blocking))
    return false; // no data available
  return true;
}

static bool receive(sick_scan_xd::ServerSocket & tcp_client_socket, size_t nr_bytes, bool little_endian, size_t & value, bool read_blocking = true)
{
  value = 0;
  std::vector<uint8_t> buffer;
  if(!receive(tcp_client_socket, nr_bytes, little_endian, buffer, read_blocking) || buffer.size() < nr_bytes)
    return false; // no data available or communication error
  if(little_endian)
  {
    for(int n = nr_bytes - 1; n >= 0; n--)
    {
      value = ((value << 8) | (buffer[n] & 0xFF));
    }
  }
  else
  {
    for(int n = 0; n < nr_bytes; n++)
    {
      value = ((value << 8) | (buffer[n] & 0xFF));
    }
  }
  return true;
}

/*
 * Constructor
 * @param[in] send_scan_data_rate frequency to generate and send scan data (default: 20 Hz)
 * @param[in] scan_data_payload scan data payload (without the message header)
 */
sick_scan_xd::test::TestServerColaMsg::TestServerColaMsg(rosNodePtr nh, double send_scan_data_rate, const std::vector<uint8_t> & scan_data_payload)
: m_nh(nh), m_send_scan_data_rate(send_scan_data_rate), m_send_scan_data(false), m_send_scan_data_cnt(0)
{
  m_scan_data_payload = scan_data_payload;

  // Create dictionaries for cola ascii and binary telegrams.
  // m_colaRequestResponseMap[is_binary?1:0] maps a request (f.e. "sAN SetAccessMode")
  // to the response payload (f.e. "sAN SetAccessMode 1")
  std::map<int, std::vector<uint8_t>> sRN_fieldResponses = {
    {  0, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x4c,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x30,0x30,0x20,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x83,0xff,0xff,0x3c,0xb0,0x02,0x01,0x00,0x01,0x00,0x03,0x01,0x2c,0xff,0xff,0x01,0x62,0x01,0xd2,0xff,0xff,0x01,0xa3,0x01,0xe6,0xff,0xff,0x00,0xce,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x0b,0x73,0x65,0x67,0x6d,0x65,0x6e,0x74,0x65,0x64,0x5f,0x31,0x00,0x00,0x6a} },
    {  1, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x54,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x30,0x31,0x20,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x83,0xff,0xff,0x3c,0xb0,0x01,0x02,0x00,0x00,0x00,0x01,0x00,0x0d,0xbb,0xa0,0x00,0x7d,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc8,0x00,0x00,0x00,0xc8,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x15,0x72,0x65,0x63,0x74,0x61,0x6e,0x67,0x6c,0x65,0x5f,0x66,0x69,0x65,0x6c,0x64,0x5f,0x30,0x5f,0x64,0x65,0x67,0x00,0x00,0x0d} },
    {  2, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x4f,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x30,0x32,0x20,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x83,0xff,0xff,0x3c,0xb0,0x01,0x03,0x00,0x00,0x00,0x01,0x00,0x0f,0x75,0x6b,0x00,0x7f,0x00,0x06,0xdd,0xd0,0x00,0x00,0x00,0xc8,0x00,0x00,0x00,0xc8,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x10,0x72,0x65,0x63,0x74,0x66,0x69,0x65,0x6c,0x64,0x5f,0x34,0x35,0x5f,0x64,0x65,0x67,0x00,0x00,0x0b} },
    {  3, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x52,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x30,0x33,0x20,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x83,0xff,0xff,0x3c,0xb0,0x03,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x1b,0x77,0x40,0x00,0xfa,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xe8,0x00,0x00,0x01,0x2c,0x09,0x60,0x00,0x00,0x05,0xdc,0x00,0x01,0x00,0x0d,0x64,0x79,0x6e,0x61,0x6d,0x69,0x63,0x5f,0x66,0x69,0x65,0x6c,0x64,0x00,0x00,0x9a} },
    {  4, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x2d,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x30,0x34,0x20,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0x88,0xff,0xff,0x3c,0xb0,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x62} },
    {  5, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x2d,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x30,0x35,0x20,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0x88,0xff,0xff,0x3c,0xb0,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x63} },
    {  6, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x2d,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x30,0x36,0x20,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0x88,0xff,0xff,0x3c,0xb0,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x60} },
    {  7, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x2d,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x30,0x37,0x20,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0x88,0xff,0xff,0x3c,0xb0,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x61} },
    {  8, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x2d,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x30,0x38,0x20,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0x88,0xff,0xff,0x3c,0xb0,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x6e} },
    {  9, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x2d,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x30,0x39,0x20,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0x88,0xff,0xff,0x3c,0xb0,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x6f} },
    { 10, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x31,0x30,0x20,0x00,0x33} },
    { 11, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x31,0x31,0x20,0x00,0x32} },
    { 12, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x31,0x32,0x20,0x00,0x31} },
    { 13, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x31,0x33,0x20,0x00,0x30} },
    { 14, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x31,0x34,0x20,0x00,0x37} },
    { 15, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x31,0x35,0x20,0x00,0x36} },
    { 16, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x31,0x36,0x20,0x00,0x35} },
    { 17, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x31,0x37,0x20,0x00,0x34} },
    { 18, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x31,0x38,0x20,0x00,0x3b} },
    { 19, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x31,0x39,0x20,0x00,0x3a} },
    { 20, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x32,0x30,0x20,0x00,0x30} },
    { 21, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x32,0x31,0x20,0x00,0x31} },
    { 22, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x32,0x32,0x20,0x00,0x32} },
    { 23, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x32,0x33,0x20,0x00,0x33} },
    { 24, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x32,0x34,0x20,0x00,0x34} },
    { 25, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x32,0x35,0x20,0x00,0x35} },
    { 26, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x32,0x36,0x20,0x00,0x36} },
    { 27, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x32,0x37,0x20,0x00,0x37} },
    { 28, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x32,0x38,0x20,0x00,0x38} },
    { 29, {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0e,0x73,0x52,0x41,0x20,0x66,0x69,0x65,0x6c,0x64,0x30,0x32,0x39,0x20,0x00,0x39} }
  };
  for(int is_binary_idx = 0; is_binary_idx <= 1; is_binary_idx++)
  {
    m_colaRequestResponseMap[is_binary_idx] = {
      {"sMN SetAccessMode", encodeColaTelegram("sAN SetAccessMode", {1}, {1}, is_binary_idx > 0)},
      {"sWN EIHstCola", encodeColaTelegram("sWA EIHstCola", {}, {}, is_binary_idx > 0)},
      {"sRN FirmwareVersion", encodeColaTelegram("sRA FirmwareVersion", {8, 1, 0, 0, 0}, {2, 1, 1, 1, 1}, is_binary_idx > 0)},
      {"sRN OrdNum", encodeColaTelegram("sRA OrdNum", {7, 1234567}, {1, 4}, is_binary_idx > 0)},
      {"sWN TransmitTargets", encodeColaTelegram("sWA TransmitTargets", {}, {}, is_binary_idx > 0)},
      {"sWN TransmitObjects", encodeColaTelegram("sWA TransmitObjects", {}, {}, is_binary_idx > 0)},
      {"sWN TCTrackingMode", encodeColaTelegram("sWA TCTrackingMode", {}, {}, is_binary_idx > 0)},
      {"sRN SCdevicestate", encodeColaTelegram("sRA SCdevicestate", {1}, {1}, is_binary_idx > 0)},
      {"sRN DItype", encodeColaTelegram("sRA DItype F RMS2731C.636111", {}, {}, is_binary_idx > 0)},
      {"sRN ODoprh", encodeColaTelegram("sRA ODoprh", {451}, {2}, is_binary_idx > 0)},
      {"sMN mSCloadappdef", encodeColaTelegram("sAN mSCloadappdef", {}, {}, is_binary_idx > 0)},
      {"sRN SerialNumber", encodeColaTelegram("sRA SerialNumber",  {"18340008"}, is_binary_idx > 0)},
      {"sRN ODpwrc", encodeColaTelegram("sRA ODpwrc", {20}, {1}, is_binary_idx > 0)},
      {"sRN LocationName", encodeColaTelegram("sRA LocationName B not defined", {}, {}, is_binary_idx > 0)},
      {"sEN LMDradardata", encodeColaTelegram("sEA LMDradardata", {1}, {1}, is_binary_idx > 0)},
      {"sRN LMPoutputRange", encodeColaTelegram("sRA LMPoutputRange", {1, 2500, -450000, 1850000}, {2, 4, 4, 4}, is_binary_idx > 0)},
      {"sWN LMPoutputRange", encodeColaTelegram("sWA LMPoutputRange", {}, {}, is_binary_idx > 0)},
      {"sWN LMDscandatacfg", encodeColaTelegram("sWA LMDscandatacfg", {}, {}, is_binary_idx > 0)},
      {"sRN LMDscandatacfg", encodeColaTelegram("sRA LMDscandatacfg", {1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, is_binary_idx > 0)},
      {"sWN SetActiveApplications", encodeColaTelegram("sWA SetActiveApplications", {}, {}, is_binary_idx > 0)},
      {"sWN FREchoFilter", encodeColaTelegram("sWA FREchoFilter", {}, {}, is_binary_idx > 0)},
      {"sEN InertialMeasurementUnit", encodeColaTelegram("sEA InertialMeasurementUnit", {}, {}, is_binary_idx > 0)},
      {"sRN DeviceIdent", encodeColaTelegram("sRA DeviceIdent", {"MRS1xxxC", "1.4.3.0B"}, is_binary_idx > 0)}, // todo: DeviceIdent from config // DeviceIdent 8 MRS1xxxx 8 1.3.0.0R.
      {"sMN LMCstartmeas", encodeColaTelegram("sAN LMCstartmeas", {}, {}, is_binary_idx > 0)},
      {"sMN LMCstopmeas", encodeColaTelegram("sAN LMCstopmeas", {}, {}, is_binary_idx > 0)},
      {"sMN Run", encodeColaTelegram("sAN Run", {}, {}, is_binary_idx > 0)},
      {"sEN LMDscandata", encodeColaTelegram("sEA LMDscandata", {}, {}, is_binary_idx > 0)},
      {"sRN field000", sRN_fieldResponses[0]},
      {"sRN field001", sRN_fieldResponses[1]},
      {"sRN field002", sRN_fieldResponses[2]},
      {"sRN field003", sRN_fieldResponses[3]},
      {"sRN field004", sRN_fieldResponses[4]},
      {"sRN field005", sRN_fieldResponses[5]},
      {"sRN field006", sRN_fieldResponses[6]},
      {"sRN field007", sRN_fieldResponses[7]},
      {"sRN field008", sRN_fieldResponses[8]},
      {"sRN field009", sRN_fieldResponses[9]},
      {"sRN field010", sRN_fieldResponses[10]},
      {"sRN field011", sRN_fieldResponses[11]},
      {"sRN field012", sRN_fieldResponses[12]},
      {"sRN field013", sRN_fieldResponses[13]},
      {"sRN field014", sRN_fieldResponses[14]},
      {"sRN field015", sRN_fieldResponses[15]},
      {"sRN field016", sRN_fieldResponses[16]},
      {"sRN field017", sRN_fieldResponses[17]},
      {"sRN field018", sRN_fieldResponses[18]},
      {"sRN field019", sRN_fieldResponses[19]},
      {"sRN field020", sRN_fieldResponses[20]},
      {"sRN field021", sRN_fieldResponses[21]},
      {"sRN field022", sRN_fieldResponses[22]},
      {"sRN field023", sRN_fieldResponses[23]},
      {"sRN field024", sRN_fieldResponses[24]},
      {"sRN field025", sRN_fieldResponses[25]},
      {"sRN field026", sRN_fieldResponses[26]},
      {"sRN field027", sRN_fieldResponses[27]},
      {"sRN field028", sRN_fieldResponses[28]},
      {"sRN field029", sRN_fieldResponses[29]}
    };
  }
}

/*
 * @brief Receives a cola telegram if data on a tcp socket are available.
 * Non-blocking function (i.e. it returns immediately) if no data available.
 * If data available, this function returns after a complete message has been received, or an error occured.
 * @param[in] tcp_client_socket socket to read from
 * @param[out] message cola telegram received from client (without the leading <STX> or trailing <ETX> bytes)
 * @param[out] is_binary always true for LDMRS
 * @return true, if a cola telegram has been received, false otherwise
 */
bool sick_scan_xd::test::TestServerColaMsg::receiveMessage(sick_scan_xd::ServerSocket & tcp_client_socket, std::vector<uint8_t> & cola_telegram, bool & is_binary)
{
  is_binary = false;
  cola_telegram.clear();
  // Receive <STX>
  size_t stx_received = 0, payload_length = 0;
  if (!receive(tcp_client_socket, 4, false, stx_received, false))
    return false; // no data available
  if(stx_received == 0x02020202) // i.e. binary Cola-B
  {
    is_binary = true;
    // Read payload length and payload
    // telegram length := 8 byte header + payload_length + 1 byte checksum
    if (!receive(tcp_client_socket, 4, false, payload_length))
      return false; // no data available
    if (!receive(tcp_client_socket, payload_length + 1, false, cola_telegram))
      return false; // communication error
    cola_telegram.pop_back(); // 1 byte checksum currently ignored
    return true;
  }
  else if(((stx_received >> 24) & 0xFF) == 0x02) // i.e. Ascii Cola-A
  {
    cola_telegram.reserve(1024);
    cola_telegram.push_back((stx_received >> 16) & 0xFF);
    cola_telegram.push_back((stx_received >> 8) & 0xFF);
    cola_telegram.push_back((stx_received) & 0xFF);
    // Read until "<ETX>" := 0x03
    uint8_t byte = 0;
    while(tcp_client_socket.read(1, &byte))
    {
      if(byte == 0x03)
        break;
      cola_telegram.push_back(byte);
    }
    if(byte != 0x03)
      return false; // communication error, <ETX> not received
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("sick_scan_xd::test::TestServerColaMsg::receiveMessage(): received 4 byte 0x" << std::hex << stx_received << ", expected <STX>");
    return false;
  }  
}

/*
 * @brief Generate a response to a message received from client.
 * @param[in] message_received message received from client (without the leading <STX> or trailing <ETX> bytes)
 * @param[in] is_binary true for binary messages, false for ascii messages
 * @param[out] response response to the client
 * @return true, if a response has been created, false otherwise (no response required or invalid message received)
 */
bool sick_scan_xd::test::TestServerColaMsg::createResponse(const std::vector<uint8_t> & message_received, bool is_binary, std::vector<uint8_t> & response)
{
  response.clear();
  // ROS_INFO_STREAM("sick_scan_xd::test::TestServerColaMsg::createResponse(): received cola-" << (is_binary?"b":"a") << " telegram \"" << binDumpVecToString(&message_received, true) << "\"");
  // Get response from dictionary
  std::string received_str(message_received.begin(), message_received.end());
  const std::map<std::string, std::vector<uint8_t>> & colaRequestResponseMap = m_colaRequestResponseMap[is_binary?1:0];
  for(std::map<std::string, std::vector<uint8_t>>::const_iterator iter = colaRequestResponseMap.cbegin(); iter != colaRequestResponseMap.cend(); iter++)
  {    
    if(received_str.find(iter->first) != std::string::npos)
    {
      response = iter->second;
      break;
    }
  }
  // Check start / stop scan data
  int send_scan_data = -1;
  if(received_str.find("sEN LMDscandata ") != std::string::npos && message_received.size() > 16) // start / stop scan data command
  {
    if(is_binary && message_received[16] == 0x00)
      send_scan_data = 0;
    else if(!is_binary && message_received[16] == '0')
      send_scan_data = 0;
    else
      send_scan_data = 1;
  }
  else if(received_str.find("sMN LMCstartmeas") != std::string::npos)
    send_scan_data = 1;
  else if(received_str.find("sMN LMCstopmeas") != std::string::npos)
    send_scan_data = 0;
  if(send_scan_data >= 0) // start / stop scan data command received
  {
    m_send_scan_data = ((send_scan_data > 0) ? true : false);
    if(m_send_scan_data)
      m_last_scan_data = std::chrono::system_clock::now(); // start scan data with the next cycle
    ROS_INFO_STREAM("sick_scan_xd::test::TestServerThread::createResponse(): received " << message_received.size() 
          << " byte message " << binDumpVecToString(&message_received, true) << " -> " << (m_send_scan_data ? "start" : "stop") << " sending scan data");
  }
  return response.size() > 0;
}

/*
 * @brief Generate a scan data message.
 * @param[out] scandata scan data message
 * @return true, if a a scan data message has been created, false otherwise (f.e. if a sensor does not generate scan data)
 */
bool sick_scan_xd::test::TestServerColaMsg::createScandata(std::vector<uint8_t> & scandata)
{
  scandata.clear();
  if(!m_send_scan_data || std::chrono::duration<double>(std::chrono::system_clock::now() - m_last_scan_data).count() < 1/m_send_scan_data_rate) // frequency to generate and send scan data (default: 20 Hz)
  {
    return false; // scan data disabled
  }

  // Encode example scan data 
  std::string command = "sRA LMDscandata ";
  std::vector<uint8_t> payload;
  payload.reserve(command.size() + m_scan_data_payload.size() + 16);
  payload.insert(payload.end(), command.begin(), command.end());
  payload.insert(payload.end(), m_scan_data_payload.begin(), m_scan_data_payload.end());
  scandata = encodeColaTelegram(payload, true);
  
  // Increase counter and range for the next scan
  m_last_scan_data = std::chrono::system_clock::now();
  m_send_scan_data_cnt += 1;
  ROS_DEBUG_STREAM("sick_scan_xd::test::TestServerColaMsg::createScandata(" << m_send_scan_data_cnt << "): " << scandata.size() << " byte scan data generated");
  if(m_send_scan_data_cnt <= 1)
    ROS_DEBUG_STREAM("sick_scan_xd::test::TestServerColaMsg::createScandata(): Generating " << scandata.size() << " byte scan data with " << m_send_scan_data_rate << " Hz");

  return true;
}
