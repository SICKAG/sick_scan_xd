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

#include <thread>
#include <mutex>

#include "sick_scan/ros_wrapper.h"

#include "sick_scan/cola_parser.h"
#include "sick_scan/cola_transmitter.h"
#include "sick_scan/utils.h"

static std::mutex s_ColaTransmitterSendMutex; // mutex to lock ColaTransmitter::send

/*!
 * Constructor.
 * @param[in] server_adress ip adress of the localization controller, default: 192.168.0.1
 * @param[in] tcp_port tcp port for command requests, default: 2111 for command requests and 2112 for  command responses
 * @param[in] default_receive_timeout default timeout in seconds for receive functions
 */
sick_scan::ColaTransmitter::ColaTransmitter(const std::string & server_adress, int tcp_port, double default_receive_timeout)
: m_server_adress(server_adress), m_tcp_port(tcp_port), m_ioservice(), m_tcp_socket(m_ioservice),
  m_receive_timeout(default_receive_timeout), m_receiver_thread_running(false), m_receiver_thread(0)
{

}

/*!
 * Destructor, closes all tcp connections.
 */
sick_scan::ColaTransmitter::~ColaTransmitter()
{
  m_receiver_thread_running = false;
  close();
  stopReceiverThread();
}

/*!
 * Connects to the localization server.
 * @return true on success, false on failure (localization server unknown or unreachable)
 */
bool sick_scan::ColaTransmitter::connect(void)
{
  return m_tcp_socket.connect(m_ioservice, m_server_adress, m_tcp_port);
}

/*!
 * Closes the tcp connection to the localization server.
 * @return always true
 */
bool sick_scan::ColaTransmitter::close(void)
{
  try
  {
    m_tcp_socket.close();
    m_ioservice.stop();
    return true;
  }
  catch(std::exception & exc)
  {
    ROS_WARN_STREAM("ColaTransmitter::closeTcpConnections(): exception " << exc.what() << " on closing connection.");
  }
  return false;
}

/*!
 * Send data to the localization server.
 * @param[in] data data to be send
 * @param[out] send_timestamp send timestamp in seconds (ros timestamp immediately before tcp send)
 * @return true on success, false on failure
 */
bool sick_scan::ColaTransmitter::send(const std::vector<uint8_t> & data, ROS::Time & send_timestamp)
{
  return send(m_tcp_socket.socket(), data, send_timestamp);
}

/*!
 * Send data to the localization server.
 * @param[in] socket socket to write to
 * @param[in] data data to be send
 * @param[out] send_timestamp send timestamp in seconds (ros timestamp immediately before tcp send)
 * @return true on success, false on failure
 */
bool sick_scan::ColaTransmitter::send(boost::asio::ip::tcp::socket & socket, const std::vector<uint8_t> & data, ROS::Time & send_timestamp)
{
  std::lock_guard<std::mutex> send_lock_guard(s_ColaTransmitterSendMutex);
  try
  {
    if (ROS::ok() && socket.is_open())
    {
      boost::system::error_code errorcode;
      send_timestamp = ROS::now();
      size_t bytes_written = boost::asio::write(socket, boost::asio::buffer(data.data(), data.size()), boost::asio::transfer_exactly(data.size()), errorcode);
      if (!errorcode && bytes_written == data.size())
        return true;
      ROS_WARN_STREAM("## ERROR ColaTransmitter::send: tcp socket write error, " << bytes_written << " of " << data.size() << " bytes written, errorcode " << errorcode.value() << " \"" << errorcode.message() << "\"");
    }
  }
  catch(std::exception & exc)
  {
    ROS_WARN_STREAM("## ERROR ColaTransmitter::send(): exception " << exc.what());
  }
  return false;
}

/*!
 * Receive a cola telegram from the localization server.
 * @param[out] telegram telegram received (Cola-Binary or Cola-Ascii)
 * @param[in] timeout timeout in seconds
 * @param[out] receive_timestamp receive timestamp in seconds (ros timestamp immediately after first response byte received)
 * @return true on success, false on failure
 */
bool sick_scan::ColaTransmitter::receive(std::vector<uint8_t> & telegram, double timeout, ROS::Time & receive_timestamp)
{
  return receive(m_tcp_socket.socket(), telegram, timeout, receive_timestamp);
}

/*!
 * Receive a cola telegram from a socket.
 * @param[in] socket socket to read from
 * @param[out] telegram telegram received (Cola-Binary or Cola-Ascii)
 * @param[in] timeout timeout in seconds
 * @param[out] receive_timestamp receive timestamp in seconds (ros timestamp immediately after first response byte received)
 * @return true on success, false on failure
 */
bool sick_scan::ColaTransmitter::receive(boost::asio::ip::tcp::socket & socket, std::vector<uint8_t> & telegram, double timeout, ROS::Time & receive_timestamp)
{
  telegram.clear();
  telegram.reserve(1024);
  try
  {
    std::vector<uint8_t> binETX = sick_scan::ColaParser::binaryETX();
    ROS::Time start_time = ROS::now();
    while (ROS::ok() && socket.is_open())
    {
      // Read 1 byte
      uint8_t byte_received = 0;
      boost::system::error_code errorcode;
      // Possibly better than receiving byte for byte: use boost::asio::read_until to read until <ETX> received
      if (socket.available() > 0 && boost::asio::read(socket, boost::asio::buffer(&byte_received, 1), boost::asio::transfer_exactly(1), errorcode) > 0 && !errorcode)
      {
        if (telegram.empty())
          receive_timestamp = ROS::now(); // timestamp after first byte received
        telegram.push_back(byte_received);
      }
      else
        ROS::sleep(0.0001);
      // Check for "<ETX>" (message completed) and return if received data ends with "<ETX>"
      bool is_binary_cola = sick_scan::ColaAsciiBinaryConverter::IsColaBinary(telegram);
      if(is_binary_cola)
      {
        // Cola-Binary: Check telegram length and return if all bytes received
        uint32_t telegram_length = sick_scan::ColaAsciiBinaryConverter::ColaBinaryTelegramLength(telegram);
        if(telegram_length > 0 && telegram.size() >= telegram_length)
          return true; // all bytes received, telegram completed
      }
      else
      {
        // Cola-ASCII: Check for "<ETX>" (message completed) and return if received data ends with "<ETX>"
        if (dataEndWithETX(telegram, binETX))
          return true; // <ETX> received, telegram completed
      }
      // Check for timeout
      if (ROS::seconds(ROS::now() - start_time) >= timeout)
      {
        // ROS_DEBUG_STREAM("ColaTransmitter::receive(): timeout, " << telegram.size() << " byte received: " << sick_scan::Utils::toHexString(telegram));
        break;
      }
    }
  }
  catch(std::exception & exc)
  {
    ROS_WARN_STREAM("## ERROR ColaTransmitter::receive(): exception " << exc.what());
  }
  return false; // no tcp connection or timeout
}

/*!
 * Returns true, if data end with etx, i.e. the trailing (etx.size()) byte in data are identical to etx.
 * @param[in] data data received
 * @param[in] etx ETX tag (binary or ascii)
 * @return true if data end with etx, false otherwise
 */
bool sick_scan::ColaTransmitter::dataEndWithETX(const std::vector<uint8_t> & data, const std::vector<uint8_t> & etx)
{
  if(data.size() < etx.size())
    return false;
  for(size_t n = data.size() - etx.size(), m = 0; m <  etx.size(); n++, m++)
  {
    if(data[n] != etx[m])
      return false;
  }
  return true;
}

/*!
 * Starts a thread to receive response telegrams from the localization server.
 * The receiver thread pushes responses to a fifo buffer, which can be popped by waitPopResponse().
 * @return always true
 */
bool sick_scan::ColaTransmitter::startReceiverThread(void)
{
  stopReceiverThread();
  m_receiver_thread_running = true;
  m_receiver_thread = new boost::thread(&sick_scan::ColaTransmitter::runReceiverThreadCb, this);
  return true;
}

/*!
 * Stops the thread to receive response telegrams from the localization server (if a thread has been started by startReceiverThread=.
 * @return always true
 */
bool sick_scan::ColaTransmitter::stopReceiverThread(void)
{
  m_receiver_thread_running = false;
  if(m_receiver_thread)
  {
    m_receiver_thread->join();
    delete(m_receiver_thread);
    m_receiver_thread = 0;
  }
  return true;
}

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
bool sick_scan::ColaTransmitter::waitPopResponse(std::vector<uint8_t> & telegram, double timeout, ROS::Time & receive_timestamp)
{
  ROS::Time start_time = ROS::now();
  while(ROS::ok() && m_receiver_thread_running && m_response_fifo.empty())
  {
    ROS::sleep(0.0001);
    m_response_fifo.waitOnceForElement();
    if(ROS::seconds(ROS::now() - start_time) >= timeout)
      break;
  }
  if(!m_response_fifo.empty())
  {
    ColaResponseContainer response = m_response_fifo.pop();
    telegram = response.telegram_data;
    receive_timestamp = response.receive_timestamp;
    return true;
  }
  return false; // timeout
}

/*!
 * Thread callback, receives response telegrams from localization server and pushes them to m_response_fifo.
 */
void sick_scan::ColaTransmitter::runReceiverThreadCb(void)
{
  while(ROS::ok() && m_receiver_thread_running)
  {
    ColaResponseContainer response;
    if(receive(response.telegram_data, m_receive_timeout, response.receive_timestamp))
      m_response_fifo.push(response);
    else
      m_response_fifo.notify();
  }
}
