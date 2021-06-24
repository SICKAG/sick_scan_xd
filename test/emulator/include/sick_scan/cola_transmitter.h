/*
 * @brief sim_loc_cola_transmitter connects to the localization controller, sends cola command requests,
 * and waits for the response with timeout.
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
#ifndef __SIM_LOC_COLA_TRANSMITTER_H_INCLUDED
#define __SIM_LOC_COLA_TRANSMITTER_H_INCLUDED

#include "sick_scan/client_socket.h"
#include "sick_scan/fifo_buffer.h"

namespace sick_scan
{
  /*!
   * Class sick_scan::ColaTransmitter connects to the localization controller and
   * sends or receives cola telegrams. Base class for ColaSender and ColaReceiver.
   */
  class ColaTransmitter
  {
  public:
    
    /*!
     * Constructor.
     * @param[in] server_adress ip adress of the localization controller, default: 192.168.0.1
     * @param[in] tcp_port tcp port for command requests, default: 2111 for command requests and 2112 for  command responses
     * @param[in] default_receive_timeout default timeout in seconds for receive functions
     */
    ColaTransmitter(const std::string & server_adress = "192.168.0.1", int tcp_port = 2111, double default_receive_timeout = 1);
    
    /*!
     * Destructor, closes all tcp connections.
     */
    virtual ~ColaTransmitter();
  
    /*!
     * Connects to the localization server.
     * @return true on success, false on failure (localization server unknown or unreachable)
     */
    virtual bool connect(void);
  
    /*!
     * Closes the tcp connection to the localization server.
     * @return always true
     */
    virtual bool close(void);
  
    /*!
     * Send data to the localization server.
     * @param[in] data data to be send
     * @param[out] send_timestamp send timestamp in seconds (ros timestamp immediately before tcp send)
     * @return true on success, false on failure
     */
    virtual bool send(const std::vector<uint8_t> & data, ROS::Time & send_timestamp);
  
    /*!
     * Send data to the localization server.
     * @param[in] socket socket to write to
     * @param[in] data data to be send
     * @param[out] send_timestamp send timestamp in seconds (ros timestamp immediately before tcp send)
     * @return true on success, false on failure
     */
    static bool send(boost::asio::ip::tcp::socket & socket, const std::vector<uint8_t> & data, ROS::Time & send_timestamp);
    
    /*!
     * Receive a cola telegram from the localization server.
     * @param[out] telegram telegram received (Cola-Binary or Cola-Ascii)
     * @param[in] timeout timeout in seconds
     * @param[out] receive_timestamp receive timestamp in seconds (ros timestamp immediately after first response byte received)
     * @return true on success, false on failure (connection error or timeout)
     */
    virtual bool receive(std::vector<uint8_t> & telegram, double timeout, ROS::Time & receive_timestamp);
  
    /*!
     * Receive a cola telegram from a socket.
     * @param[in] socket socket to read from
     * @param[out] telegram telegram received (Cola-Binary or Cola-Ascii)
     * @param[in] timeout timeout in seconds
     * @param[out] receive_timestamp receive timestamp in seconds (ros timestamp immediately after first response byte received)
     * @return true on success, false on failure (connection error or timeout)
     */
    static bool receive(boost::asio::ip::tcp::socket & socket, std::vector<uint8_t> & telegram, double timeout,ROS::Time & receive_timestamp);
  
    /*!
     * Starts a thread to receive response telegrams from the localization server.
     * The receiver thread pushes responses to a fifo buffer, which can be popped by waitPopResponse().
     * @return always true
     */
    virtual bool startReceiverThread(void);
  
    /*!
     * Stops the thread to receive response telegrams from the localization server (if a thread has been started by startReceiverThread=.
     * @return always true
     */
    virtual bool stopReceiverThread(void);
  
    /*!
     * Returns a response telegram from the localization server.
     * This function waits with timout, until the receiver thread received a response telegram from the localization server.
     * Note: The receiver thread must have been started by startReceiverThread(), otherwise waitPopResponse() will fail
     * after timout.
     * @param[out] telegram telegram received (Cola-Binary or Cola-Ascii)
     * @param[in] timeout timeout in seconds
     * @param[out] receive_timestamp receive timestamp in seconds (ros timestamp immediately after first response byte received)
     * @return true on success, false on failure (connection error or timeout)
     */
    virtual bool waitPopResponse(std::vector<uint8_t> & telegram, double timeout, ROS::Time & receive_timestamp);
  
  protected:
  
    /*!
     * class ServerColaRequest: utility container for a response received from localization server
     * (telegram data plus timestamp)
     */
    class ColaResponseContainer
    {
    public:
      ColaResponseContainer(const std::vector<uint8_t> & data = std::vector<uint8_t>(),const ROS::Time & timestamp = ROS::now())
      : telegram_data(data), receive_timestamp(timestamp) {} ///< Constructor
      std::vector<uint8_t> telegram_data; ///< received telegram_data (Cola-Ascii or Cola-Binary)
      ROS::Time receive_timestamp;        ///< receive timestamp in seconds (ros timestamp immediately after first response byte received)
    };
  
    /*!
     * Returns true, if data end with etx, i.e. the trailing (etx.size()) byte in data are identical to etx.
     * @param[in] data data received
     * @param[in] etx ETX tag (binary or ascii)
     * @return true if data end with etx, false otherwise
     */
    static bool dataEndWithETX(const std::vector<uint8_t> & data, const std::vector<uint8_t> & etx);

    /*!
     * Thread callback, receives response telegrams from localization server and pushes them to m_response_fifo.
     */
    void runReceiverThreadCb(void);
    
    /*
     * member data
     */

    std::string m_server_adress;                        ///< ip adress of the localization controller, default: 192.168.0.1
    int m_tcp_port;                                     ///< tcp port of the localization controller, default: 2111 for command requests and 2112 for  command responses
    boost::asio::io_service m_ioservice;                ///< boost io service for tcp connections
    sick_scan::ClientSocket m_tcp_socket; ///< tcp socket connected to the localization controller
    double m_receive_timeout;                           ///< default timeout in seconds for receive functions
    bool m_receiver_thread_running;                     ///< true: m_receiver_thread is running, otherwise false
    boost::thread* m_receiver_thread;                   ///< thread to receive responses from localization server
    sick_scan::FifoBuffer<ColaResponseContainer, boost::mutex> m_response_fifo; ///< fifo buffer for receiver thread for responses from localization server
  
  }; // class ColaTransmitter
  
} // namespace sick_scan
#endif // __SIM_LOC_COLA_TRANSMITTER_H_INCLUDED
