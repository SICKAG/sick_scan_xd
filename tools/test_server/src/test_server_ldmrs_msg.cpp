/*
 * @brief sick_scan2 test_server_lidar_msg implements the ldmrs specific messages,
 * i.e. message receiving and message creation to simulate ldmrs devices.
 *
 * Note: sick_scan2 test_server_lidar_msg does not implement the functions of lidar sensors,
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

#include "sick_scan/test_server/test_server_ldmrs_msg.h"

/*
 * Constructor
 * @param[in] send_scan_data_rate frequency to generate and send scan data (default: 20 Hz)
 * @param[in] scan_data_payload scan data payload (without the message header)
 */
sick_scan_xd::test::TestServerLDMRSMsg::TestServerLDMRSMsg(rosNodePtr nh, double send_scan_data_rate, const std::vector<uint8_t> & scan_data_payload)
: m_nh(nh), m_send_scan_data_rate(send_scan_data_rate), m_send_scan_data(false), m_send_scan_data_cnt(0), m_delta_range_cm(0), m_delta_range_step(1)
{
  m_scan_data_payload = scan_data_payload;
}

/*
 * @brief Creates and returns the 24 byte message header containing the magic word 0xAFFEC0C2, the data type and payload length.
 * @param[in] data_type data type identifier, f.e. 0x2010 for a command request or 0x2020 for a command response
 * @param[in] payload_length length of payload in byte
 * @return 24 byte message header
 */
std::vector<uint8_t> sick_scan_xd::test::TestServerLDMRSMsg::createMessageHeader(size_t data_type, size_t payload_length)
{
  std::vector<uint8_t> msg_header(24);
  std::vector<uint8_t> magic_word = { 0xaf, 0xfe, 0xc0, 0xc2 };
  msg_header[0] = magic_word[0];
  msg_header[1] = magic_word[1];
  msg_header[2] = magic_word[2];
  msg_header[3] = magic_word[3];
  msg_header[8] = ((payload_length >> 24) & 0xFF);
  msg_header[9] = ((payload_length >> 16) & 0xFF);
  msg_header[10] = ((payload_length >> 8) & 0xFF);
  msg_header[11] = ((payload_length) & 0xFF);
  msg_header[14] = ((data_type >> 8) & 0xFF);
  msg_header[15] = ((data_type) & 0xFF);
  return msg_header;
}

/*
 * @brief Receives a LDMRS message if data on a tcp socket are available.
 * Non-blocking function (i.e. it returns immediately) if no data available.
 * If data available, this function returns after a complete message has been received, or an error occured.
 * @param[in] tcp_client_socket socket to read from
 * @param[out] message message received from client
 * @param[out] is_binary always true for LDMRS
 * @return true, if a message has been received, false otherwise
 */
bool sick_scan_xd::test::TestServerLDMRSMsg::receiveMessage(sick_scan_xd::ServerSocket & tcp_client_socket, std::vector<uint8_t> & message, bool & is_binary)
{
  is_binary = true;
  message.clear();
  assert(m_nh);

  // Note from telegram listing: "a mixture of little and big endian encoding is used. While the message
  // header, the frame around the data payload, is encoded using big endian format, the
  // payload itself is always encoded in little endian format.
  // Be sure to encode/decode correctly!"

  // Receive 24 byte message header
  uint8_t magic_word[4] = { 0xaf, 0xfe, 0xc0, 0xc2 };
  uint8_t header[24] = {0};
  if (!tcp_client_socket.read(sizeof(header), &header[0], false))
  {
    return false; // no data available or error receiving message header
  }
  if (memcmp(&header[0], &magic_word[0], 4) != 0)
  {
    ROS_ERROR_STREAM("## ERROR sick_scan_xd::test::TestServerLDMRSMsg::receiveMessage(): error receiving magic word 0xAFFEC0C2");
    return false; // error receiving magic word
  }
  
  // Decode 4 byte payload length (big endian)
  size_t payload_length = 0;
  for(int n = 8; n < 12; n++)
    payload_length = ((payload_length << 8) | (header[n] & 0xFF));
  
  // Decode 2 byte data type (big endian)
  size_t data_type = 0;
  for(int n = 14; n < 16; n++)
    data_type = ((data_type << 8) | (header[n] & 0xFF));
  
  // Decode 8 byte ntp time (big endian)
  uint64_t ntp_time = 0;
  for(int n = 16; n < 24; n++)
    ntp_time = ((ntp_time << 8) | (header[n] & 0xFF));
  
  // Read payload
  message.resize(24 + payload_length);
  memcpy(message.data(), &header[0], sizeof(header));
  if (!tcp_client_socket.read(payload_length, message.data() + 24))
  {
    ROS_ERROR_STREAM("## ERROR sick_scan_xd::test::TestServerLDMRSMsg::receiveMessage(): error receiving " << payload_length << " byte tcp payload");
    return false; // error receiving tcp payload
  }

  return true;
}

/*
 * @brief Generate a response to a message received from client.
 * @param[in] message_received message received from client
 * @param[in] is_binary true for binary messages, false for ascii messages
 * @param[out] response response to the client
 * @return true, if a response has been created, false otherwise (no response required or invalid message received)
 */
bool sick_scan_xd::test::TestServerLDMRSMsg::createResponse(const std::vector<uint8_t> & message_received, bool is_binary, std::vector<uint8_t> & response)
{
  std::vector<uint8_t> payload;
  response.clear();
  if(message_received.size() < 24)
  {
    ROS_ERROR_STREAM("## ERROR sick_scan_xd::test::TestServerLDMRSMsg::createResponse(): invalid input message");
    return false;
  }

  // Note from telegram listing: "a mixture of little and big endian encoding is used. While the message
  // header, the frame around the data payload, is encoded using big endian format, the
  // payload itself is always encoded in little endian format.
  // Be sure to encode/decode correctly!"

  // Decode 2 byte data type (big endian)
  size_t data_type = 0;
  for(int n = 14; n < 16; n++)
    data_type = ((data_type << 8) | (message_received[n] & 0xFF));
  if(data_type != 0x2010)
  {
    return false; // not a command => no response
  }
  // Decode 2 byte command id (little endian)
  if(message_received.size() < 26)
  {
    ROS_ERROR_STREAM("## ERROR sick_scan_xd::test::TestServerLDMRSMsg::createResponse(): invalid command message");
    return false;
  }
  size_t command_id = (message_received[24] & 0xFF) | (message_received[25] << 8);
  std::map<int, std::string> command_id_names = {
    {0x0000, "\"Reset\""}, {0x0001, "\"Get Status\""}, {0x0004, "\"Save Config\""}, {0x0010, "\"Set Parameter\""}, {0x0011, "\"Get Parameter\""}, {0x001A, "\"Reset Default Parameters\""}, 
    {0x0020, "\"Start Measure\""}, {0x0021, "\"Stop Measure\""}, {0x0030, "\"Set NTP Timestamp Sec\""}, {0x0031, "\"Set NTP Timestamp Frac Sec\""}
  };
  std::map<int, std::string> parameter_id_names = {
    {0x1000, "\"IP address\""}, {0x1001, "\"TCP Port\""}, {0x1002, "\"Subnet Mask\""}, {0x1003, "\"Standard gateway\""}, {0x1010, "\"CAN Base ID\""}, {0x1011, "\"CAN Baud Rate\""}, 
    {0x1012, "\"Data Output Flag\""}, {0x1013, "\"maxObjectsViaCAN\""}, {0x1014, "\"ContourPointDensity\""}, {0x1016, "\"CAN object data options\""}, {0x1017, "\"Minimum Object Age\""}, 
    {0x1100, "\"Start angle\""}, {0x1101, "\"End angle\""}, {0x1102, "\"Scan frequency\""}, {0x1103, "\"Sync angle offset\""}, {0x1104, "\"angular resolution type\""}, {0x1105, "\"angleTicksPerRotation\""}, 
    {0x1108, "\"RangeReduction\""}, {0x1109, "\"Upside down mode\""}, {0x110A, "\"Ignore near range\""}, {0x110B, "\"Sensitivity control active\""}, {0x1200, "\"SensorMounting_X\""}, {0x1201, "\"SensorMounting_Y\""}, 
    {0x1202, "\"SensorMounting_Z\""}, {0x1203, "\"SensorMounting_Yaw\""}, {0x1204, "\"SensorMounting_Pitch\""}, {0x1205, "\"SensorMounting_Roll\""}, {0x1206, "\"VehicleFrontToFrontAxle\""}, 
    {0x1207, "\"FrontAxleToRearAxle\""}, {0x1208, "\"RearAxleToVehicleRear\""}, {0x120A, "\"SteerRatioType\""}, {0x120C, "\"SteerRatioPoly0\""}, {0x120D, "\"SteerRatioPoly1\""}, {0x120E, "\"SteerRatioPoly2\""}, 
    {0x120F, "\"SteerRatioPoly3\""}, {0x1210, "\"Vehicle Motion Data Flags\""}, {0x2208, "\"EnableSensorInfo\""}, {0x3302, "\"BeamTilt\""}, {0x3500, "\"Timemeter\""}, {0x3600, "\"Enable APD control\""}, 
    {0x4000, "\"NumSectors\""}, {0x4001, "\"StartAngle, Sector 1\""}, {0x4002, "\"StartAngle, Sector 2\""}, {0x4003, "\"StartAngle, Sector 3\""}, {0x4004, "\"StartAngle, Sector 4\""}, {0x4005, "\"StartAngle, Sector 5\""}, 
    {0x4006, "\"StartAngle, Sector 6\""}, {0x4007, "\"StartAngle, Sector 7\""}, {0x4008, "\"StartAngle, Sector 8\""}, {0x4009, "\"Angular resolution, Sector 1\""}, {0x400A, "\"Angular resolution, Sector 2\""}, 
    {0x400B, "\"Angular resolution, Sector 3\""}, {0x400C, "\"Angular resolution, Sector 4\""}, {0x400D, "\"Angular resolution, Sector 5\""}, {0x400E, "\"Angular resolution, Sector 6\""}, {0x400F, "\"Angular resolution, Sector 7\""}, 
    {0x4010, "\"Angular resolution, Sector 8\""}, {0x7000, "\"Detailed error code for FlexRes\""}
  };
  
  // Command "Reset"
  if(command_id == 0x0000)
  {
    return false; // Reply: For this command (Reset), no reply is sent
  }
  // Command "Get status"
  else if(command_id == 0x0001)
  {
    payload.resize(32);  // 32 byte status information
    payload[0] = 0x01;   // id status request
    payload[1] = 0x00;   // id status request
    payload[3] = 0x12;   // firmware version 1.23.4
    payload[2] = 0x34;   // firmware version 1.23.4
    payload[5] = 0x56;   // fpga version 5.67.8
    payload[4] = 0x78;   // fpga version 5.67.8
    payload[13] = 0x01;  // temperature 0x01EC = 23.88 degree celsius
    payload[12] = 0xEC;  // temperature 0x01EC = 23.88 degree celsius
    payload[15] = 0x20;  // serial number 0 (year)
    payload[14] = 0x21;  // serial number 0 (calender week)
    payload[17] = 0x00;  // serial number 1
    payload[16] = 0x01;  // serial number 1
    payload[19] = 0x00;  // serial number 2
    payload[18] = 0x01;  // serial number 2

    payload[21] = 0x20;  // version timestamp YY
    payload[20] = 0x20;  // version timestamp YY
    payload[23] = 0x05;  // version timestamp MM
    payload[22] = 0x06;  // version timestamp DD
    payload[25] = 0x07;  // version timestamp hh
    payload[24] = 0x08;  // version timestamp mm

    payload[27] = 0x20;  // version timestamp YY
    payload[26] = 0x20;  // version timestamp YY
    payload[29] = 0x01;  // version timestamp MM
    payload[28] = 0x02;  // version timestamp DD
    payload[31] = 0x03;  // version timestamp hh
    payload[30] = 0x04;  // version timestamp mm

  }
  // Command "Set Parameter"
  else if(command_id == 0x0010)
  {
    size_t parameter_id = (message_received.size() >= 29) ? ((message_received[28] & 0xFF) | (message_received[29] << 8)) : 0;
    uint32_t parameter_value = (message_received.size() >= 33) ? ((message_received[30] & 0xFF) | (message_received[31] << 8) | (message_received[32] << 16) | (message_received[33] << 24)) : 0;
    payload.resize(2); // 2 byte acknowledge with command id
    payload[0] = 0x10; // id status request
    ROS_INFO_STREAM("sick_scan_xd::test::TestServerLDMRSMsg::createResponse(): command id 0x10 \"Set Parameter\", parameter id 0x" 
     << std::hex << parameter_id << " " << parameter_id_names[parameter_id] << ", parameter value 0x" << std::hex << parameter_value);
  }
  // Command "Get Parameter", Read a single parameter from the LD-MRS
  else if(command_id == 0x0011)
  {
    size_t parameter_id = (message_received.size() >= 29) ? ((message_received[28] & 0xFF) | (message_received[29] << 8)) : 0;
    uint32_t parameter_value = (message_received.size() >= 33) ? ((message_received[30] & 0xFF) | (message_received[31] << 8) | (message_received[32] << 16) | (message_received[33] << 24)) : 0;
    payload.resize(8); // 8 byte parameter information
    payload[0] = 0x11; // id status request
    payload[2] = message_received[28];
    payload[3] = message_received[29];
    ROS_INFO_STREAM("sick_scan_xd::test::TestServerLDMRSMsg::createResponse(): command id 0x11 \"Get Parameter\", parameter id 0x" 
     << std::hex << parameter_id << parameter_id_names[parameter_id] << ", parameter value 0x" << std::hex << parameter_value);
  }
  // Start Measure
  else if(command_id == 0x0020)
  {
    m_send_scan_data = true;
    m_last_scan_data = std::chrono::system_clock::now() + std::chrono::seconds(5); // start scanning in 5 seconds
    payload.resize(2); // 2 byte acknowledge with command id
    payload[0] = 0x20; // id status request
    ROS_INFO_STREAM("sick_scan_xd::test::TestServerLDMRSMsg::createResponse(): command id 0x20 -> start scanning in 5 seconds");
  }
  // Stop Measure
  else if(command_id == 0x0021)
  {
    m_send_scan_data = false;
    payload.resize(2); // 2 byte acknowledge with command id
    payload[0] = 0x21; // id status request
    ROS_INFO_STREAM("sick_scan_xd::test::TestServerLDMRSMsg::createResponse(): command id 0x21 -> stop scanning");
  }
  // Default: a command will be acknowledged by the same command id without any command reply data
  else
  {
    payload.resize(2); // 2 byte acknowledge with command id
    payload[0] = (command_id & 0xFF);        // command id (LSB)
    payload[1] = ((command_id >> 8) & 0xFF); // command id (MSB)
  }

  // response := 24 byte header (big endian) + payload (little endian)
  std::vector<uint8_t> msg_header = createMessageHeader(0x2020, payload.size());
  response.reserve(msg_header.size() + payload.size());
  response.insert(response.end(), msg_header.begin(), msg_header.end());
  if(!payload.empty())
    response.insert(response.end(), payload.begin(), payload.end());

  ROS_INFO_STREAM("sick_scan_xd::test::TestServerLDMRSMsg::createResponse(): command id 0x" << std::hex << command_id << " " << command_id_names[command_id] 
    << ", " << std::dec << response.size() << " byte response");
  return true;
}

/*
 * @brief Generate a scan data message.
 * @param[out] scandata scan data message
 * @return true, if a a scan data message has been created, false otherwise (f.e. if a sensor does not generate scan data)
 */
bool sick_scan_xd::test::TestServerLDMRSMsg::createScandata(std::vector<uint8_t> & scandata)
{
  scandata.clear();
  if(!m_send_scan_data || std::chrono::duration<double>(std::chrono::system_clock::now() - m_last_scan_data).count() < 1/m_send_scan_data_rate) // frequency to generate and send scan data (default: 20 Hz)
  {
    return false; // scan data disabled
  }

  // Simulate the scan of a moving object: Loop over all scan points in the payload 
  // and increase/decrease the range of all scan points
  // scan_points_start := payload + 44 = Start Scan Point List
  // scan_points_start + 2 : 2 byte horizontal angle in ticks
  // scan_points_start + 4 : 2 byte range in cm
  // each scan point encoded with 10 byte
  std::vector<uint8_t> payload = m_scan_data_payload;
  for(size_t range_offset = 48; range_offset + 10 < payload.size(); range_offset += 10)
  {
    size_t range_cm = (payload[range_offset] & 0xFF) | (payload[range_offset + 1] << 8);
    range_cm += m_delta_range_cm;
    payload[range_offset] = (range_cm & 0xFF);
    payload[range_offset + 1] = ((range_cm >> 8) & 0xFF);
  }

  // response := 24 byte header (big endian) + payload (little endian)
  std::vector<uint8_t> msg_header = createMessageHeader(0x2202, payload.size());
  scandata.reserve(msg_header.size() + payload.size());
  scandata.insert(scandata.end(), msg_header.begin(), msg_header.end());
  scandata.insert(scandata.end(), payload.begin(), payload.end());

  // Increase counter and range for the next scan
  m_last_scan_data = std::chrono::system_clock::now();
  m_send_scan_data_cnt += 1;
  m_delta_range_cm += m_delta_range_step;
  if(m_delta_range_cm > 500 && m_delta_range_step > 0)
  {
    m_delta_range_cm = 500;
    m_delta_range_step = -m_delta_range_step;
  }
  if(m_delta_range_cm < 0 && m_delta_range_step < 0)
  {
    m_delta_range_cm = 0;
    m_delta_range_step = -m_delta_range_step;
  }
  ROS_INFO_STREAM("sick_scan_xd::test::TestServerLDMRSMsg::createScandata(" << m_send_scan_data_cnt << "): " << scandata.size() << " byte scan data generated");
  if(m_send_scan_data_cnt <= 1)
    ROS_INFO_STREAM("sick_scan_xd::test::TestServerLDMRSMsg::createScandata(): Generating " << scandata.size() << " byte scan data with " << m_send_scan_data_rate << " Hz");

  return true;
}
