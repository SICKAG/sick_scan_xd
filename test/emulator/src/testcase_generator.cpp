/*
 * @brief sim_loc_testcase_generator generates testcases for SIM Localization driver.
 * The generator creates deterministic and random based result port telegrams.
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

#include "sick_scan/cola_parser.h"
#include "sick_scan/random_generator.h"
#include "sick_scan/testcase_generator.h"
#include "sick_scan/utils.h"


/*!
 * result pose interval, i.e. the interval in number of scans (default: 1, i.e. result telegram with each processed scan)
 */
uint32_t sick_scan::TestcaseGenerator::s_u32ResultPoseInterval = 1;

/*!
 * test server int32 settings, set by sMN or sRN requests
 */
std::map<std::string, int32_t> sick_scan::TestcaseGenerator::s_controller_settings = {
  {"IsSystemReady", 1},        // 0:false, 1:true (default)
  {"LMCstartmeas", 0},         // 0:false (do not send LMDscandata or LMDscandatamon), 1:true (send LMDscandata and LMDscandatamon)
  {"LocState", 2},             // controller state: 0:BOOTING, 1:IDLE, 2:LOCALIZING, 3:DEMO_MAPPING
  {"LocResultPort", 2201},     // tcp port for result telegrams (default: 2201)
  {"LocResultMode", 0},        // 0:stream (default), 1:poll
  {"LocResultState", 1},       // result output: 0: disabled, 1: enabled
  {"LocResultEndianness", 0},  // 0: big endian (default), 1: little endian
  {"LocMapState", 1},          // map state: 0:not active, 1:active
  {"LocRequestResultData", 1}  // in poll mode, trigger sending the localization result of the next processed scan via TCP interface.
};

/*!
 * test server string settings, set by sMN or sRN requests
 */
std::map<std::string, std::string> sick_scan::TestcaseGenerator::s_controller_settings_str;

/*!
 * Returns true, if LMCstartmeas has been requested (i.e. send LMDscandata and LMDscandatamon, LMCstartmeas is enabled), 
 * or false otherwise (i.e. do not send LMDscandata or LMDscandatamon, LMCstartmeas is disabled)
 * @return  LMCstartmeas status (true or false)
 */
bool sick_scan::TestcaseGenerator::SendScandataEnabled(void)
{
  return s_controller_settings["LMCstartmeas"] > 0; // 0:false (do not send LMDscandata or LMDscandatamon), 1:true (send LMDscandata and LMDscandatamon)
}

/*!
 * Returns true, if localization is active (default), otherwise false (localization deactivated)
 * @return result telegrams are activated (true) or deactivated
 */
bool sick_scan::TestcaseGenerator::LocalizationEnabled(void)
{
  return s_controller_settings["LocState"] == 2; // localization on
}

/*!
 * Returns true, if result telegrams are activated (i.e. localization on and result telegrams active), otherwise false (result telegrams deactivated)
 * @return result telegrams are activated (true, default) or deactivated
 */
bool sick_scan::TestcaseGenerator::ResultTelegramsEnabled(void)
{
  return LocalizationEnabled() && s_controller_settings["LocResultState"] > 0; // localization on and result telegrams activated, otherwise result telegrams deactivated
}

/*!
 * Creates and returns a deterministic default testcase for result port telegrams (binary telegrams and SickLocResultPortTelegramMsg)
 * @return SickLocResultPortTestcaseMsg with the binary telegram and SickLocResultPortTelegramMsg
 */
sick_scan::SickLocResultPortTestcaseMsg sick_scan::TestcaseGenerator::createDefaultResultPortTestcase(void)
{
  sick_scan::SickLocResultPortTestcaseMsg testcase;
  
  // ROS Header with sequence id, timestamp and frame id
  testcase.header.stamp = ROS::now();
  testcase.header.frame_id = "sick_localization_testcase";

  // binary encoded result port telegram (default example)
  testcase.binary_data = {
    0x53, 0x49, 0x43, 0x4B, 0x00, 0x00, 0x00, 0x6A, 0x06, 0x42, 0x00, 0x01, 0x00, 0x10, 0xC0, 0x58, 0x01, 0x22, 0xA2, 0x72,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x4C, 0x53, 0x20, 0x56, 0x30, 0x2E, 0x31, 0x2E, 0x39, 0x2E, 0x78, 0x42,
    0x00, 0x00, 0x02, 0x6D, 0x83, 0xAA, 0x8C, 0x0C, 0x8E, 0x14, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x6F, 0x00, 0x34,
    0xEC, 0xF3, 0x00, 0x00, 0x00, 0x5D, 0x00, 0x00, 0x00, 0x21, 0x00, 0x00, 0x45, 0xE7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x37, 0x00, 0x00, 0x00, 0x80, 0x89, 0x00, 0x00, 0x99, 0x93, 0x00, 0x12, 0x78, 0x9F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x62, 0x11 };

  // decoded result port telegram
  sick_scan::ResultPortParser result_port_parser(testcase.header.frame_id);
  std::vector<uint8_t> recoded_telegram;
  if(!result_port_parser.decode(testcase.binary_data) || (recoded_telegram = result_port_parser.encode()) != testcase.binary_data)
  {
    ROS_ERROR_STREAM("## ERROR TestcaseGenerator::createDefaultResultPortTestcase: sick_scan::ResultPortParser::decode() failed. " << testcase.binary_data.size() << " byte input (hex):");
    ROS_ERROR_STREAM(sick_scan::Utils::toHexString(testcase.binary_data));
    ROS_ERROR_STREAM("## output (decoded): " << sick_scan::Utils::flattenToString(result_port_parser.getTelegramMsg()));
    ROS_ERROR_STREAM("## recoded:");
    ROS_ERROR_STREAM(sick_scan::Utils::toHexString(recoded_telegram));
  }
  testcase.telegram_msg = result_port_parser.getTelegramMsg();

  return testcase;
}

/*!
 * Creates and returns a random testcase for result port telegrams (binary telegrams and SickLocResultPortTelegramMsg)
 * @return SickLocResultPortTestcaseMsg with the binary telegram and SickLocResultPortTelegramMsg
 */
sick_scan::SickLocResultPortTestcaseMsg sick_scan::TestcaseGenerator::createRandomResultPortTestcase(void)
{
  // Random number generators
  static sick_scan::UniformRandomInteger random1_generator(0, 1);
  static sick_scan::UniformRandomInteger random8_generator(0, 255);
  static sick_scan::UniformRandomInteger random32_generator(-INT32_MAX, INT32_MAX);
  static sick_scan::UniformRandomInteger random_yaw_generator(-180000, 180000);
  static sick_scan::UniformRandomInteger random_quality_generator(0, 100);
  static sick_scan::UniformRandomInteger random_covariance_generator(0, INT32_MAX);
  
  // Create default SickLocResultPortTelegramMsg
  static ROS::Time start_time = ROS::now();
  static sick_scan::SickLocResultPortTestcaseMsg default_testcase = createDefaultResultPortTestcase();
  sick_scan::SickLocResultPortTestcaseMsg testcase = default_testcase;
  sick_scan::SickLocResultPortTelegramMsg & telegram_msg = testcase.telegram_msg;
  
  // Modify SickLocResultPortTelegramMsg with random values
  telegram_msg.telegram_header.payloadtype = ((random1_generator.generate() > 0) ? 0x06c2 : 0x0642); // Payload type: 0x06c2 = Little Endian, 0x0642 = Big Endian. Size: UInt16 = 2 byte
  telegram_msg.telegram_header.ordernumber = (uint32_t)random32_generator.generate();                // Order number of the localization controller. Size: UInt32 = 4 byte
  telegram_msg.telegram_header.serialnumber = (uint32_t)random32_generator.generate();               // Serial number of the localization controller. Size: UInt32 = 4 byte
  for(size_t n = 0; n < telegram_msg.telegram_header.fw_version.size(); n++)
    telegram_msg.telegram_header.fw_version[n] = (uint8_t)random8_generator.generate();              // Software version of the localization controller. Size: 20 × UInt8 = 20 byte
  telegram_msg.telegram_payload.posex = random32_generator.generate();                               // Position X of the vehicle on the map in cartesian global coordinates [mm]. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.posey = random32_generator.generate();                               // Position Y of the vehicle on the map in cartesian global coordinates [mm]. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.poseyaw = random_yaw_generator.generate();                           // Orientation (yaw) of the vehicle on the map [mdeg], range -180 to +180 deg assumed. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.reserved1 = (uint32_t)random32_generator.generate();                 // Reserved. Size: UInt32 = 4 byte
  telegram_msg.telegram_payload.reserved2 = random32_generator.generate();                           // Reserved. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.quality = (uint8_t)random_quality_generator.generate();              // Quality of pose [0 … 100], 1 = bad pose quality, 100 = good pose quality. Size: UInt8 = 1 byte
  telegram_msg.telegram_payload.outliersratio = (uint8_t)random_quality_generator.generate();        // Ratio of beams that cannot be assigned to the current reference map [%]. Size: UInt8 = 1 byte
  telegram_msg.telegram_payload.covariancex = random_covariance_generator.generate();                // Covariance c1 of the pose X [mm^2]. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.covariancey = random_covariance_generator.generate();                // Covariance c5 of the pose Y [mm^2]. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.covarianceyaw = random_covariance_generator.generate();              // Covariance c9 of the pose Yaw [mdeg^2]. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.reserved3 = (((uint64_t)random32_generator.generate() << 32) | (uint64_t)random32_generator.generate()); // Reserved. Size: UInt64 = 8 byte
  
  // Update telegram timestamps
  double delta_time_seconds = ROS::seconds(ROS::now() - start_time);
  telegram_msg.telegram_payload.timestamp = createTimestampTicksMilliSec();    // Time stamp of the pose [ms]. The time stamp indicates the time at which the pose is calculated. Size: UInt32 = 4 byte
  telegram_msg.telegram_header.systemtime += (uint64_t)(delta_time_seconds);   // SystemTime not used. Size: NTP = 8 byte
  
  // Re-encode the modified result port telegram (SickLocResultPortTelegramMsg)
  sick_scan::ResultPortParser result_port_parser(testcase.header.frame_id);
  result_port_parser.getTelegramMsg() = telegram_msg;
  testcase.binary_data = result_port_parser.encode();
  testcase.telegram_msg = result_port_parser.getTelegramMsg();
  
  // Increment telegram counter for next testcase
  default_testcase.telegram_msg.telegram_header.telegramcounter += 1; // Telegram counter since last start-up. Size: UInt32 = 4 byte
  default_testcase.telegram_msg.telegram_payload.scancounter += 1;    // Counter of related scan data. Size: UInt32 = 4 byte
  
  // Update testcase timestamp
  testcase.header.stamp = ROS::now();
  return testcase;
}

/*!
 * Creates and returns a result port telegrams (binary telegrams and SickLocResultPortTelegramMsg)
 * simulating sensor moving in circles with a current position given by radius in meter and yaw angle
 * in radians
 * @param[in] circle_radius radius of circle in meter
 * @param[in] circle_yaw current angle in radians
 * @return SickLocResultPortTestcaseMsg with the binary telegram and SickLocResultPortTelegramMsg
 */
sick_scan::SickLocResultPortTestcaseMsg sick_scan::TestcaseGenerator::createResultPortCircles(double circle_radius, double circle_yaw)
{
  // Create default SickLocResultPortTelegramMsg
  static ROS::Time start_time = ROS::now();
  static sick_scan::SickLocResultPortTestcaseMsg default_testcase = createDefaultResultPortTestcase();
  sick_scan::SickLocResultPortTestcaseMsg testcase = default_testcase;
  sick_scan::SickLocResultPortTelegramMsg & telegram_msg = testcase.telegram_msg;

  // Set current position and orientation
  telegram_msg.telegram_payload.posex = (int32_t)(1000.0 * circle_radius * std::cos(circle_yaw));  // Position X of the vehicle on the map in cartesian global coordinates [mm]. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.posey = (int32_t)(1000.0 * circle_radius * std::sin(circle_yaw));  // Position Y of the vehicle on the map in cartesian global coordinates [mm]. Size: Int32 = 4 byte
  double orientation = sick_scan::Utils::normalizeAngle(circle_yaw + M_PI_2);        // Orienation := circle_yaw + 90 degree
  telegram_msg.telegram_payload.poseyaw = (int32_t)(1000.0 * orientation * 180.0 / M_PI);          // Orientation (yaw) of the vehicle on the map [mdeg], range -180 to +180 deg assumed. Size: Int32 = 4 byte
  
  // Update telegram timestamps
  double delta_time_seconds = ROS::seconds(ROS::now() - start_time);
  telegram_msg.telegram_payload.timestamp = createTimestampTicksMilliSec();             // Time stamp of the pose [ms]. The time stamp indicates the time at which the pose is calculated. Size: UInt32 = 4 byte
  telegram_msg.telegram_header.systemtime += (uint64_t)(delta_time_seconds);            // SystemTime not used. Size: NTP = 8 byte
  
  // Re-encode the modified result port telegram (SickLocResultPortTelegramMsg)
  sick_scan::ResultPortParser result_port_parser(testcase.header.frame_id);
  result_port_parser.getTelegramMsg() = telegram_msg;
  testcase.binary_data = result_port_parser.encode();
  testcase.telegram_msg = result_port_parser.getTelegramMsg();
  
  // Increment telegram counter for next testcase
  default_testcase.telegram_msg.telegram_header.telegramcounter += 1; // Telegram counter since last start-up. Size: UInt32 = 4 byte
  default_testcase.telegram_msg.telegram_payload.scancounter += 1;    // Counter of related scan data. Size: UInt32 = 4 byte
  
  // Update testcase timestamp
  testcase.header.stamp = ROS::now();
  return testcase;
}

/*!
 * Creates and returns a synthetical cola response to a cola command request.
 * Note: Just a few cola responses are implemented for test purposes, f.e. responses to "LocRequestTimestamp".
 * By default, a response: "sAN <command_name>" without any parameter is returned (sAN: Response to sMN)
 * @param[in] cola_request Cola request from client
 * @return Synthetical cola response from server
 */
sick_scan::SickLocColaTelegramMsg sick_scan::TestcaseGenerator::createColaResponse(const sick_scan::SickLocColaTelegramMsg & cola_request, const std::string& scanner_type)
{
  static std::map<sick_scan::ColaParser::COLA_SOPAS_COMMAND, std::map<std::string, sick_scan::SickLocColaTelegramMsg>> s_mapped_responses; // static responses for requests
  static bool s_mapped_responses_initialized = false;
  if(!s_mapped_responses_initialized)
  {
    s_mapped_responses[sick_scan::ColaParser::sEN] = { // static responses for sEN requests
      {"ECRChangeArr", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sEA"), "ECRChangeArr", {"01"})},
      {"LFErec", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sEA"), "LFErec", {"01"})},
      {"LIDinputstate", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sEA"), "LIDinputstate", {"01"})},
      {"LIDoutputstate", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sEA"), "LIDoutputstate", {"01"})},
      {"LMDscandata", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sEA"), "LMDscandata", {"01"})}
    };
    s_mapped_responses[sick_scan::ColaParser::sMN] = { // static responses for sMN requests
      {"Run", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sAN"), "Run", {"01"})},
      {"SetAccessMode", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sAN"), "SetAccessMode", {"01"})}
    };
    s_mapped_responses[sick_scan::ColaParser::sRN] = { // static responses for sRN requests
      // todo: read from configured json-file! The following "sRA fieldxxx" responses are hardcoded from file 20210113_tim871s_elephant.pcapng
      {"field000", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field000", {"3f8000000000000000000d05fff92230020100010000000000000000000100084669656c643030310000"})},
      {"field001", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field001", {"3f8000000000000000000d05fff9223002020001000200f6ffff00fa0195ffff012c000000000000000100084669656c643030320000"})},
      {"field002", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field002", {"3f8000000000000000000d05fff922300203000100030195ffff012c026bffff009d02a3ffff00a0000000000000000100084669656c643030330000"})},
      {"field003", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field003", {"3f8000000000000000000d05fff92230020400010006010affff04be0115ffff047e0120ffff04410135ffff03e90154ffff03ad0178ffff0377000000000000000100084669656c643030340000"})},
      {"field004", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field004", {"3f8000000000000000000d05fff92230020500010003017cffff037b018effff037901b0ffff0383000000000000000100084669656c643030350000"})},
      {"field005", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field005", {"3f8000000000000000000d05fff9223002060001000401baffff039201c1ffff03a101dfffff03fd0211ffff04fd000000000000000100084669656c643030360000"})},
      {"field006", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field006", {"3f8000000000000000000d05fff92230020700010000000000000000000100084669656c643030370000"})},
      {"field007", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field007", {"3f8000000000000000000d05fff92230020800010000000000000000000100084669656c643030380000"})},
      {"field008", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field008", {"3f8000000000000000000d05fff92230020900010000000000000000000100084669656c643030390000"})},
      {"field009", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field009", {"3f8000000000000000000d05fff92230020a00010000000000000000000100084669656c643031300000"})},
      {"field010", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field010", {"3f8000000000000000000d05fff92230020b00010000000000000000000100084669656c643031310000"})},
      {"field011", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field011", {"3f8000000000000000000d05fff92230020c00010000000000000000000100084669656c643031320000"})},
      {"field012", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field012", {"3f8000000000000000000d05fff92230020d00010000000000000000000100084669656c643031330000"})},
      {"field013", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field013", {"3f8000000000000000000d05fff92230020e00010000000000000000000100084669656c643031340000"})},
      {"field014", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field014", {"3f8000000000000000000d05fff92230020f00010000000000000000000100084669656c643031350000"})},
      {"field015", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field015", {"3f8000000000000000000d05fff92230021000010000000000000000000100084669656c643031360000"})},
      {"field016", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field016", {"3f8000000000000000000d05fff92230021100010000000000000000000100084669656c643031370000"})},
      {"field017", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field017", {"3f8000000000000000000d05fff92230021200010000000000000000000100084669656c643031380000"})},
      {"field018", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field018", {"3f8000000000000000000d05fff92230021300010000000000000000000100084669656c643031390000"})},
      {"field019", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field019", {"3f8000000000000000000d05fff92230021400010000000000000000000100084669656c643032300000"})},
      {"field020", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field020", {"3f8000000000000000000d05fff92230021500010000000000000000000100084669656c643032310000"})},
      {"field021", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field021", {"3f8000000000000000000d05fff92230021600010000000000000000000100084669656c643032320000"})},
      {"field022", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field022", {"3f8000000000000000000d05fff92230021700010000000000000000000100084669656c643032330000"})},
      {"field023", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field023", {"3f8000000000000000000d05fff92230021800010000000000000000000100084669656c643032340000"})},
      {"field024", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field024", {"3f8000000000000000000d05fff92230021900010000000000000000000100084669656c643032350000"})},
      {"field025", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field025", {"3f8000000000000000000d05fff92230021a00010000000000000000000100084669656c643032360000"})},
      {"field026", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field026", {"3f8000000000000000000d05fff92230021b00010000000000000000000100084669656c643032370000"})},
      {"field027", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field027", {"3f8000000000000000000d05fff92230021c00010000000000000000000100084669656c643032380000"})},
      {"field028", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field028", {"3f8000000000000000000d05fff92230021d00010000000000000000000100084669656c643032390000"})},
      {"field029", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field029", {"3f8000000000000000000d05fff92230021e00010000000000000000000100084669656c643033300000"})},
      {"field030", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field030", {"3f8000000000000000000d05fff92230021f00010000000000000000000100084669656c643033310000"})},
      {"field031", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field031", {"3f8000000000000000000d05fff92230022000010000000000000000000100084669656c643033320000"})},
      {"field032", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field032", {"3f8000000000000000000d05fff92230022100010000000000000000000100084669656c643033330000"})},
      {"field033", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field033", {"3f8000000000000000000d05fff92230022200010000000000000000000100084669656c643033340000"})},
      {"field034", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field034", {"3f8000000000000000000d05fff92230022300010000000000000000000100084669656c643033350000"})},
      {"field035", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field035", {"3f8000000000000000000d05fff92230022400010000000000000000000100084669656c643033360000"})},
      {"field036", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field036", {"3f8000000000000000000d05fff92230022500010000000000000000000100084669656c643033370000"})},
      {"field037", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field037", {"3f8000000000000000000d05fff92230022600010000000000000000000100084669656c643033380000"})},
      {"field038", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field038", {"3f8000000000000000000d05fff92230022700010000000000000000000100084669656c643033390000"})},
      {"field039", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field039", {"3f8000000000000000000d05fff92230022800010000000000000000000100084669656c643034300000"})},
      {"field040", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field040", {"3f8000000000000000000d05fff92230022900010000000000000000000100084669656c643034310000"})},
      {"field041", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field041", {"3f8000000000000000000d05fff92230022a00010000000000000000000100084669656c643034320000"})},
      {"field042", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field042", {"3f8000000000000000000d05fff92230022b00010000000000000000000100084669656c643034330000"})},
      {"field043", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field043", {"3f8000000000000000000d05fff92230022c00010000000000000000000100084669656c643034340000"})},
      {"field044", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field044", {"3f8000000000000000000d05fff92230022d00010000000000000000000100084669656c643034350000"})},
      {"field045", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field045", {"3f8000000000000000000d05fff92230022e00010000000000000000000100084669656c643034360000"})},
      {"field046", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field046", {"3f8000000000000000000d05fff92230022f00010000000000000000000100084669656c643034370000"})},
      {"field047", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field047", {"3f8000000000000000000d05fff92230023000010000000000000000000100084669656c643034380000"})},
      {"FirmwareVersion", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "FirmwareVersion", {"000556332e3133"})},
      // todo: fieldset from config ...
      {"LIDinputstate", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "LIDinputstate", {"000000000000000000000000"})}, // activate fieldset 0
      //{"LIDinputstate", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "LIDinputstate", {"000000000000010000000000"})}, // activate fieldset 1
      {"LMPoutputRange", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "LMPoutputRange", {"000100000d05fff9223000225510"})},
      {"LocationName", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "LocationName", {"000b6e6f7420646566696e6564"})},
      {"ODoprh", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "ODoprh", {"00000161"})},
      {"ODpwrc", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "ODpwrc", {"00000017"})},
      {"LMDscandatacfg", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "LMDscandatacfg", {"01000101000000000000010001"})},
      {"SCdevicestate", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "SCdevicestate", {"00"})}
    };
    s_mapped_responses[sick_scan::ColaParser::sWN] = { // static responses for sWN requests
      {"EIHstCola", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sWA"), "EIHstCola", {""})},
      {"LMDscandatacfg", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sWA"), "LMDscandatacfg", {""})},
      {"LMPoutputRange", sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sWA"), "LMPoutputRange", {""})}
    };
    if(scanner_type == "sick_lms_5xx") // overwrite for LMS5xx
    {
      s_mapped_responses[sick_scan::ColaParser::sRN]["field000"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field000", {"400000000000000000000683ffff3cb0020100010003012cffff016201d2ffff01a301e6ffff00ce0000000000000001000b7365676d656e7465645f310000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field001"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field001", {"400000000000000000000683ffff3cb0010200000001000dbba0007d00000000000000c8000000c8000000000001001572656374616e676c655f6669656c645f305f6465670000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field002"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field002", {"400000000000000000000683ffff3cb0010300000001000f756b007f0006ddd0000000c8000000c80000000000010010726563746669656c645f34355f6465670000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field003"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field003", {"400000000000000000000683ffff3cb003040000000000000001001b774000fa00000000000003e80000012c0960000005dc0001000d64796e616d69635f6669656c640000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field004"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field004", {"400000000000000000001388ffff3cb002000000000000000000000100000000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field005"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field005", {"400000000000000000001388ffff3cb002000000000000000000000100000000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field006"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field006", {"400000000000000000001388ffff3cb002000000000000000000000100000000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field007"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field007", {"400000000000000000001388ffff3cb002000000000000000000000100000000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field008"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field008", {"400000000000000000001388ffff3cb002000000000000000000000100000000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field009"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field009", {"400000000000000000001388ffff3cb002000000000000000000000100000000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field010"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field010", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field011"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field011", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field012"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field012", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field013"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field013", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field014"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field014", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field015"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field015", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field016"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field016", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field017"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field017", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field018"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field018", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field019"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field019", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field020"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field020", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field021"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field021", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field022"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field022", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field023"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field023", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field024"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field024", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field025"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field025", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field026"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field026", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field027"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field027", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field028"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field028", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field029"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field029", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field030"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field031"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field032"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field033"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field034"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field035"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field036"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field037"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field038"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field039"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field040"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field041"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field042"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field043"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field044"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field045"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field046"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field047"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["LMPoutputRange"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "LMPoutputRange", {"000100000683ffff3cb0001c3a90"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["LMDscandatacfg"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "LMDscandatacfg", {"01000100000000000000010001"});
    }
    if(scanner_type == "sick_lms_1xx") // overwrite for LMS1xx
    {
      s_mapped_responses[sick_scan::ColaParser::sRN]["field000"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field000", {"3f8000000000000000001388fff92230020100010029019affff0176019bffff0175019cffff0171019dffff015f019effff0154019fffff015101a0ffff015101a1ffff015101a2ffff015101a3ffff015101a4ffff015101a5ffff015101a6ffff015201a7ffff015301a8ffff014601a9ffff014601aaffff014601abffff014901acffff014901adffff014b01aeffff014901afffff014901b0ffff014901b1ffff014a01b2ffff014301b3ffff014301b4ffff014301b5ffff014801b6ffff014801b7ffff014801b8ffff014d01b9ffff014e01baffff014c01bbffff014a01bcffff014701bdffff014701beffff014701bfffff014a01c0ffff014601c1ffff014601c2ffff0146000000000000000100064649454c44310000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field001"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field001", {"3f8000000000000000001388fff922300102000000010019e4b7045efffeee90000001900000006400000000000100064649454c44320000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field002"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field002", {"3f8000000000000000001388fff9223003030000000000000001001b774003e800000000000007d0000003e81388000007d0000100064649454c44330000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field003"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field003", {"3f8000000000000000001388fff9223002000000000000000000000100000000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field004"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field004", {"3f8000000000000000001388fff9223002000000000000000000000100000000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field005"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field005", {"3f8000000000000000001388fff9223002000000000000000000000100000000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field006"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field006", {"3f8000000000000000001388fff9223002000000000000000000000100000000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field007"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field007", {"3f8000000000000000001388fff9223002000000000000000000000100000000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field008"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field008", {"3f8000000000000000001388fff9223002000000000000000000000100000000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field009"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field009", {"3f8000000000000000001388fff9223002000000000000000000000100000000"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field010"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field010", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field011"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field011", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field012"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field012", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field013"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field013", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field014"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field014", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field015"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field015", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field016"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field016", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field017"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field017", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field018"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field018", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field019"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field019", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field020"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field020", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field021"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field021", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field022"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field022", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field023"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field023", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field023"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field023", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field024"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field024", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field025"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field025", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field026"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field026", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field027"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field027", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field028"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field028", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field029"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "field029", {"00"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field030"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field031"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field032"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field033"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field034"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field035"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field036"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field037"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field038"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field039"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field040"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field041"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field042"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field043"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field044"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field045"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field046"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["field047"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sFA"), "", {"03"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["LMPoutputRange"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "LMPoutputRange", {"000100001388fff922330022550d"});
      s_mapped_responses[sick_scan::ColaParser::sRN]["LMDscandatacfg"] = sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sRA"), "LMDscandatacfg", {"01000000000000000000010001"});
    }
    s_mapped_responses_initialized = true;
  }

  // Return response from static table, if request can be found in s_mapped_responses
  for(std::map<sick_scan::ColaParser::COLA_SOPAS_COMMAND, std::map<std::string, sick_scan::SickLocColaTelegramMsg>>::iterator iter_cmd = s_mapped_responses.begin(); iter_cmd != s_mapped_responses.end(); iter_cmd++)
  {
    if(cola_request.command_type == iter_cmd->first)
    {
      std::map<std::string, sick_scan::SickLocColaTelegramMsg>& mapped_response = iter_cmd->second;
      if(mapped_response.find(cola_request.command_name) != mapped_response.end())
      {
        return mapped_response[cola_request.command_name];
      }
    }
  }

  // Generate a synthetical response to LocRequestTimestamp requests: "sAN LocRequestTimestamp <timestamp>" with uint32_t timestamp in hex and ms
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocRequestTimestamp")
  {
    static sick_scan::UniformRandomInteger time_jitter_network_ms(0, 2);
    // Simulate some network latency
    ROS::sleep(0.001 * time_jitter_network_ms.generate());
    // Create current timestamp in ticks
    uint32_t ticks_ms = createTimestampTicksMilliSec();
    // Simulate some network latency
    ROS::sleep(0.001 * time_jitter_network_ms.generate());
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {hexstr(ticks_ms)});
  }
  
  // Set settings from Configuration Telegrams
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LMCstartmeas")
  {
    s_controller_settings["LMCstartmeas"] = 1;
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sAN"), "LMCstartmeas", {"00"});
  }
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LMCstopmeas")
  {
    s_controller_settings["LMCstartmeas"] = 0;
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::convertSopasCommand("sAN"), "LMCstopmeas", {"00"});
  }
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocStartLocalizing")
  {
    s_controller_settings["LocState"] = 2;
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocStop")
  {
    s_controller_settings["LocState"] = 1;
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocStopAndSave")
  {
    s_controller_settings["LocState"] = 1;
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocSetResultPort" && cola_request.parameter.size() == 1)
  {
    s_controller_settings["LocResultPort"] = std::strtol(cola_request.parameter[0].c_str(), 0, 0);
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1), decstr(1)});
  }
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocSetResultMode" && cola_request.parameter.size() == 1)
  {
    s_controller_settings["LocResultMode"] = std::strtol(cola_request.parameter[0].c_str(), 0, 0);
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocSetResultPoseEnabled" && cola_request.parameter.size() == 1)
  {
    s_controller_settings["LocResultState"] = std::strtol(cola_request.parameter[0].c_str(), 0, 0);
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocSetResultEndianness" && cola_request.parameter.size() == 1)
  {
    s_controller_settings["LocResultEndianness"] = std::strtol(cola_request.parameter[0].c_str(), 0, 0);;
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocSetPose" && cola_request.parameter.size() == 4)
  {
    int32_t posex_mm = std::strtol(cola_request.parameter[0].c_str(), 0, 0);
    int32_t posey_mm = std::strtol(cola_request.parameter[1].c_str(), 0, 0);
    int32_t yaw_mdeg = std::strtol(cola_request.parameter[2].c_str(), 0, 0);
    int32_t uncertainty = std::strtol(cola_request.parameter[3].c_str(), 0, 0);
    bool success = (posex_mm >= -300000 && posex_mm <= +300000 && posey_mm >= -300000 && posey_mm <= +300000
      && yaw_mdeg >= -180000 && yaw_mdeg <= +180000 && uncertainty >= 0 && uncertainty < 300000);
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(success?1:0)});
  }

  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocSetResultPoseInterval" && cola_request.parameter.size() == 1)
  {
    s_u32ResultPoseInterval = std::strtoul(cola_request.parameter[0].c_str(), 0, 0);
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }

  /* Start of test server responses for new service requests (release 4 or later) */

  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "DevSetLidarConfig" && cola_request.parameter.size() == 15)
  {
    for(size_t n = 0; n < cola_request.parameter.size(); n++)
      s_controller_settings_str["DevSetLidarConfig_"+std::to_string(n)] = cola_request.parameter[n];
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1), decstr(1)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "DevGetLidarConfig" && cola_request.parameter.size() == 1)
  {
    std::vector<std::string> config_parameter;
    for(size_t n = 1; n < 15; n++)
      // if(n != 9)
        config_parameter.push_back(s_controller_settings_str["DevSetLidarConfig_"+std::to_string(n)]);
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, config_parameter);
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocSetMap" && cola_request.parameter.size() == 2)
  {
    s_controller_settings_str["LocSetMap"] = cola_request.parameter[1];
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1), decstr(1)});
  }
  
  if(cola_request.command_name == "LocMap")//if(cola_request.command_type == sick_scan::ColaParser::sRN && cola_request.command_name == "LocMap" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sRA, cola_request.command_name, {s_controller_settings_str["LocSetMap"]});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sRN && cola_request.command_name == "LocMapState" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocInitializePose" && cola_request.parameter.size() == 4)
  {
    for(size_t n = 0; n < cola_request.parameter.size(); n++)
      s_controller_settings_str["LocInitializePose_"+std::to_string(n)] = cola_request.parameter[n];
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sRN && cola_request.command_name == "LocInitialPose" && cola_request.parameter.size() == 0)
  {
    std::vector<std::string> parameter;
    for(size_t n = 0; n < 4; n++)
      parameter.push_back(s_controller_settings_str["LocInitializePose_"+std::to_string(n)]);
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, parameter);
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocSetPoseQualityCovWeight" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["LocSetPoseQualityCovWeight"] = cola_request.parameter[0];
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sRN && cola_request.command_name == "LocPoseQualityCovWeight" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {s_controller_settings_str["LocSetPoseQualityCovWeight"]});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocSetPoseQualityMeanDistWeight" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["LocSetPoseQualityMeanDistWeight"] = cola_request.parameter[0];
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sRN && cola_request.command_name == "LocPoseQualityMeanDistWeight" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {s_controller_settings_str["LocSetPoseQualityMeanDistWeight"]});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocSetReflectorsForSupportActive" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["LocSetReflectorsForSupportActive"] = cola_request.parameter[0];
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sRN && cola_request.command_name == "LocReflectorsForSupportActive" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {s_controller_settings_str["LocSetReflectorsForSupportActive"]});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocSetOdometryActive" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["LocSetOdometryActive"] = cola_request.parameter[0];
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1), decstr(1)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sRN && cola_request.command_name == "LocOdometryActive" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {s_controller_settings_str["LocSetOdometryActive"]});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocSetOdometryPort" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["LocSetOdometryPort"] = cola_request.parameter[0];
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1), decstr(1)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sRN && cola_request.command_name == "LocOdometryPort" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {s_controller_settings_str["LocSetOdometryPort"]});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocSetOdometryRestrictYMotion" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["LocSetOdometryRestrictYMotion"] = cola_request.parameter[0];
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sRN && cola_request.command_name == "LocOdometryRestrictYMotion" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {s_controller_settings_str["LocSetOdometryRestrictYMotion"]});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocSetAutoStartActive" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["LocSetAutoStartActive"] = cola_request.parameter[0];
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sRN && cola_request.command_name == "LocAutoStartActive" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {s_controller_settings_str["LocSetAutoStartActive"]});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocSetAutoStartSavePoseInterval" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["LocSetAutoStartSavePoseInterval"] = cola_request.parameter[0];
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sRN && cola_request.command_name == "LocAutoStartSavePoseInterval" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {s_controller_settings_str["LocSetAutoStartSavePoseInterval"]});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocSetRingBufferRecordingActive" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["LocSetRingBufferRecordingActive"] = cola_request.parameter[0];
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sRN && cola_request.command_name == "LocRingBufferRecordingActive" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {s_controller_settings_str["LocSetRingBufferRecordingActive"]});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "DevGetLidarIdent" && cola_request.parameter.size() == 1)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {"TestcaseGenerator" + cola_request.parameter[0]});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "DevGetLidarState" && cola_request.parameter.size() == 1)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(2), decstr(2), decstr(2)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "GetSoftwareVersion" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {"1.0"});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocAutoStartSavePose" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocForceUpdate" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocSaveRingBufferRecording" && cola_request.parameter.size() == 2)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(2)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "LocStartDemoMapping" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "ReportUserMessage" && cola_request.parameter.size() == 2)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "SavePermanent" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sRN && cola_request.command_name == "LocResultPort" && cola_request.parameter.size() == 0)
  {
    int32_t port = ((s_controller_settings["LocResultPort"]) > 0 ? s_controller_settings["LocResultPort"] : 2201);
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sRA, cola_request.command_name, {hexstr(port)});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sRN && cola_request.command_name == "LocResultMode" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sRA, cola_request.command_name, {decstr(s_controller_settings["LocResultMode"])});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sRN && cola_request.command_name == "LocResultEndianness" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sRA, cola_request.command_name, {decstr(s_controller_settings["LocResultEndianness"])});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sRN && cola_request.command_name == "LocResultState" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sRA, cola_request.command_name, {decstr(s_controller_settings["LocResultState"])});
  }
  
  if(cola_request.command_type == sick_scan::ColaParser::sRN && cola_request.command_name == "LocResultPoseInterval" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sRA, cola_request.command_name, {decstr(s_u32ResultPoseInterval)});
  }  
  
  if(cola_request.command_type == sick_scan::ColaParser::sMN && cola_request.command_name == "DevSetIMUActive" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["DevSetIMUActive"] = cola_request.parameter[0];
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }  

  if(cola_request.command_type == sick_scan::ColaParser::sRN && cola_request.command_name == "DevIMUActive" && cola_request.parameter.size() == 0)
  {
    return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sRA, cola_request.command_name, {s_controller_settings_str["DevSetIMUActive"]});
  }  

  /* End of test server responses for new service requests (release 4 or later) */

  // Create sAN responses to sMN requests resp. sRA responses to sRN requests
  if(cola_request.command_type == sick_scan::ColaParser::sMN || cola_request.command_type == sick_scan::ColaParser::sRN)
  {
    sick_scan::ColaParser::COLA_SOPAS_COMMAND response_type = sick_scan::ColaParser::sINVALID;
    if(cola_request.command_type == sick_scan::ColaParser::sMN)
      response_type = sick_scan::ColaParser::sAN; // sAN responses to sMN requests
    else if(cola_request.command_type == sick_scan::ColaParser::sRN)
      response_type = sick_scan::ColaParser::sRA; // sRA responses to sRN requests
    for(std::map<std::string, int32_t>::iterator iter_settings = s_controller_settings.begin(); iter_settings != s_controller_settings.end(); iter_settings++)
    {
      if(cola_request.command_name == iter_settings->first)
      {
        return sick_scan::ColaParser::createColaTelegram(response_type, cola_request.command_name, {hexstr(iter_settings->second)});
      }
    }
  }
  
  // Default response: "sAN <command_name>" without parameter (sAN: Response to sMN)
  return sick_scan::ColaParser::createColaTelegram(sick_scan::ColaParser::sAN, cola_request.command_name);
}

/*!
 * Creates and returns a timestamp in milliseconds ticks.
 * To simulate time jitter, network latency and time drift,
 * a random value of +/- 2 milliseconds is added.
 * @return timestamp in milliseconds ticks
 */
uint32_t sick_scan::TestcaseGenerator::createTimestampTicksMilliSec(void)
{
  static ROS::Time start = ROS::now();
  static sick_scan::UniformRandomInteger time_jitter_ticks_ms(-2, +2);
  // Create current timestamp in ticks
  ROS::Duration timestamp = (ROS::now() - start);
  uint32_t seconds = 0, nanoseconds = 0;
  ROS::splitTime(timestamp, seconds, nanoseconds);
  uint32_t ticks_ms = (((uint64_t)seconds * 1000 + (uint64_t)nanoseconds/1000000 + 1000) & 0xFFFFFFFF);
  // Create some jitter, simulation network latency and time drift
  ticks_ms += time_jitter_ticks_ms.generate();
  return ticks_ms;
}