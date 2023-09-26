#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
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
#ifndef __SICK_SCAN_TEST_SERVER_LDMRS_MESSAGE_H_INCLUDED
#define __SICK_SCAN_TEST_SERVER_LDMRS_MESSAGE_H_INCLUDED

#include <sick_scan/sick_ros_wrapper.h>
#include "sick_scan/server_socket.h"
#include "sick_scan/test_server/test_server_lidar_msg.h"

namespace sick_scan_xd
{
  namespace test
  {
    /*!
    * class TestServerLDMRSMsg implements interface TestServerLidarMsg for the LDMRS.
    */
    class TestServerLDMRSMsg : public TestServerLidarMsg
    {
    public:

      /*
       * Constructor
       * @param[in] send_scan_data_rate frequency to generate and send scan data (default: 20 Hz)
       * @param[in] scan_data_payload scan data payload (without the message header)
       */
      TestServerLDMRSMsg(rosNodePtr nh, double send_scan_data_rate = 20.0, const std::vector<uint8_t> & scan_data_payload = std::vector<uint8_t>());

      /*
       * @brief Receives a LDMRS message if data on a tcp socket are available.
       * Non-blocking function (i.e. it returns immediately) if no data available.
       * If data available, this function returns after a complete message has been received, or an error occured.
       * @param[in] tcp_client_socket socket to read from
       * @param[out] message message received from client
       * @param[out] is_binary always true for LDMRS
       * @return true, if a message has been received, false otherwise
       */
      virtual bool receiveMessage(sick_scan_xd::ServerSocket & tcp_client_socket, std::vector<uint8_t> & message, bool & is_binary);

      /*
       * @brief Generate a response to a message received from client.
       * @param[in] message_received message received from client
       * @param[in] is_binary true for binary messages, false for ascii messages
       * @param[out] response response to the client
       * @return true, if a response has been created, false otherwise (no response required or invalid message received)
       */
      virtual bool createResponse(const std::vector<uint8_t> & message_received, bool is_binary, std::vector<uint8_t> & response);

      /*
       * @brief Generate a scan data message.
       * @param[out] scandata scan data message
       * @return true, if a a scan data message has been created, false otherwise (f.e. if a sensor does not generate scan data)
       */
      virtual bool createScandata(std::vector<uint8_t> & scandata);

    protected:

      /*
       * @brief Creates and returns the 24 byte message header containing the magic word 0xAFFEC0C2, the data type and payload length.
       * @param[in] data_type data type identifier, f.e. 0x2010 for a command request or 0x2020 for a command response
       * @param[in] payload_length length of payload in byte
       * @return 24 byte message header
       */
      static std::vector<uint8_t> createMessageHeader(size_t data_type, size_t payload_length);

      rosNodePtr m_nh;
      double m_send_scan_data_rate; // frequency to generate and send scan data (default: 20 Hz)
      bool m_send_scan_data; // true after command "start measure", false after command "stop measure"
      std::vector<uint8_t> m_scan_data_payload; // scan data payload (without the message header)
      size_t m_send_scan_data_cnt; // counts the number of scan data sent to client
      std::chrono::system_clock::time_point m_last_scan_data; // timestamp of last scan data
      int m_delta_range_cm;   // simulate the scan of a moving object by increasing or decreasing the range of scan points by m_delta_range_cm
      int m_delta_range_step; // step in cm to increase or decrease the range of scan points

    }; // class TestServerLDMRSMsg

  } // namespace test
} // namespace sick_scan_xd
#endif // __SICK_SCAN_TEST_SERVER_LDMRS_MESSAGE_H_INCLUDED
