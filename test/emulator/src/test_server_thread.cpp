/*
 * @brief sim_loc_test_server_thread implements a simple tcp server thread,
 * simulating a localization controller for unittests. It runs a thread to listen
 * and accept tcp connections from clients and generates telegrams to test
 * the ros driver for sim localization.
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
#include <limits.h>
#include "sick_scan/ros_wrapper.h"

#include "sick_scan/cola_parser.h"
#include "sick_scan/cola_transmitter.h"
#include "sick_scan/pcapng_json_parser.h"
#include "sick_scan/random_generator.h"
#include "sick_scan/test_server_thread.h"
#include "sick_scan/testcase_generator.h"
#include "sick_scan/utils.h"

#if defined _MSC_VER && defined min
#undef min
#endif
#if defined _MSC_VER && defined max
#undef max
#endif

/*!
 * Constructor. The server thread does not start automatically, call start() and stop() to start and stop the server.
 * @param[in] nh ros node handle
 * @param[in] ip_port_results ip port for result telegrams, default: 2201
 * @param[in] ip_port_cola ip port for command requests and responses, default: 2111
 * @param[in] start_scandata_immediately default (false): send scandata after switch into measurement mode (true: start send scandata thread immediately)
 */
sick_scan_xd::TestServerThread::TestServerThread(ROS::NodePtr nh, int ip_port_results, int ip_port_cola, const std::string& scanner_type, bool start_scandata_immediately)
: m_ip_port_results(ip_port_results), m_ip_port_cola(ip_port_cola), m_scanner_type(scanner_type), m_start_scandata_immediately(start_scandata_immediately),
  m_tcp_connection_thread_results(0), m_tcp_connection_thread_cola(0), m_tcp_send_scandata_thread(0),
  m_tcp_connection_thread_running(false), m_worker_thread_running(false), m_tcp_send_scandata_thread_running(false),
  m_start_scandata_delay(1), m_result_telegram_rate(10), m_demo_move_in_circles(false), m_error_simulation_enabled(false), m_error_simulation_flag(SIMU_NO_ERROR),
  m_error_simulation_thread(0), m_error_simulation_thread_running(false)
{
  m_scandatafiles = "/tmp/lmd_scandata.pcapng.json"; // default pcapng.json file for testing and debugging
  m_scandatatypes = "sSN LMDscandata ,sSN LIDinputstate ,sSN LIDoutputstate ,sSN LFErec ,sSN InertialMeasurementUnit ,sSN LMDradardata "; // default datatypes (send those datatypes when found in scandata file)
  if(nh)
  {
    ROS::param<std::string>(nh, "/sick_scan_emulator/scandatafiles", m_scandatafiles, m_scandatafiles); // comma separated list of jsonfiles to emulate scandata messages, f.e. "tim781s_scandata.pcapng.json,tim781s_sopas.pcapng.json"
    ROS::param<std::string>(nh, "/sick_scan_emulator/scandatatypes", m_scandatatypes, m_scandatatypes); // comma separated list of scandata message types, f.e. "sSN LMDscandata,sSN LMDscandatamon"
    ROS::param<std::string>(nh, "/sick_scan_emulator/scanner_type", m_scanner_type, m_scanner_type);    // currently supported: "sick_lms_5xx", "sick_tim_7xx", "sick_mrs_6xxx"
    ROS_INFO_STREAM("TestServerThread: scanner_type=\"" << m_scanner_type << "\"");
    ROS::param<bool>(nh, "sick_scan_emulator/start_scandata_immediately", m_start_scandata_immediately, m_start_scandata_immediately);
    ROS_INFO_STREAM("TestServerThread: start_scandata_immediately=\"" << (int)m_start_scandata_immediately << "\"");
    ROS::param<double>(nh, "/sick_scan/test_server/start_scandata_delay", m_start_scandata_delay, m_start_scandata_delay); // delay between scandata activation ("LMCstartmeas" request) and first scandata message, default: 1 second
    std::string result_testcases_topic = "/sick_scan/test_server/result_testcases"; // default topic to publish testcases with result port telegrams (type SickLocResultPortTestcaseMsg)
    ROS::param<double>(nh, "/sick_scan/test_server/result_telegrams_rate", m_result_telegram_rate, m_result_telegram_rate);
    ROS::param<std::string>(nh, "/sick_scan/test_server/result_testcases_topic", result_testcases_topic, result_testcases_topic);
    ROS::param<std::string>(nh, "/sick_scan/test_server/result_testcases_frame_id", m_result_testcases_frame_id, "result_testcases");
    ROS::param<bool>(nh, "/sim_loc_test_server/demo_circles", m_demo_move_in_circles, m_demo_move_in_circles);
    ROS::param<bool>(nh, "/sim_loc_test_server/error_simulation", m_error_simulation_enabled, m_error_simulation_enabled);
    std::string sim_loc_test_server_demo_circles, sim_loc_test_server_error_simulation;
    ROS::param<std::string>(nh, "/system/test_server/demo_circles", sim_loc_test_server_demo_circles, sim_loc_test_server_demo_circles);
    ROS::param<std::string>(nh, "/system/test_server/error_simulation", sim_loc_test_server_error_simulation, sim_loc_test_server_error_simulation);
    if(!sim_loc_test_server_demo_circles.empty())
      m_demo_move_in_circles = std::atoi(sim_loc_test_server_demo_circles.c_str()) > 0;
    if(!sim_loc_test_server_error_simulation.empty())
      m_error_simulation_enabled = std::atoi(sim_loc_test_server_error_simulation.c_str()) > 0;
    // ros publisher for testcases with result port telegrams (type SickLocResultPortTestcaseMsg)
    m_result_testcases_publisher = ROS_CREATE_PUBLISHER(nh, sick_scan_xd::SickLocResultPortTestcaseMsg, result_testcases_topic);
    ROS_INFO_STREAM("TestServerThread: scandatafiles=\"" << m_scandatafiles << "\", scandatatypes=\"" << m_scandatatypes << "\"");
  }
}

/*!
 * Destructor. Stops the server thread and closes all tcp connections.
 */
sick_scan_xd::TestServerThread::~TestServerThread()
{
  stop();
}

/*!
 * Starts the server thread, starts to listen and accept tcp connections from clients.
 * @return true on success, false on failure.
 */
bool sick_scan_xd::TestServerThread::start(void)
{
  if(m_error_simulation_enabled)
  {
    ROS_WARN_STREAM("TestServerThread: running in error simulation mode.");
    ROS_WARN_STREAM("Test server will intentionally react incorrect and will send false data and produce communication errors.");
    ROS_WARN_STREAM("Running test server in error simulation mode is for test purposes only. Do not expect typical results.");
    m_error_simulation_thread_running = true;
    m_error_simulation_thread =  new std::thread(&sick_scan_xd::TestServerThread::runErrorSimulationThreadCb, this);
  }
  else if(m_demo_move_in_circles)
  {
    ROS_INFO_STREAM("TestServerThread: running in demo mode (moving in circles, no error simulation).");
  }
  else
  {
    ROS_INFO_STREAM("TestServerThread: running in normal mode (no error simulation).");
  }
  // Start and run 3 threads to create sockets for new tcp clients on ip ports 2201 (results), 2111 (requests) and 2112 (responses)
  m_tcp_connection_thread_running = true;
  m_tcp_connection_thread_results = new std::thread(&sick_scan_xd::TestServerThread::runConnectionThreadResultCb, this);
  m_tcp_connection_thread_cola = new std::thread(&sick_scan_xd::TestServerThread::runConnectionThreadColaCb, this);
  return true;
}

/*!
 * Stops the server thread and closes all tcp connections.
 * @return true on success, false on failure.
 */
bool sick_scan_xd::TestServerThread::stop(void)
{
  m_tcp_connection_thread_running = false;
  m_error_simulation_thread_running = false;
  m_worker_thread_running = false;
  if(m_error_simulation_thread)
  {
    m_error_simulation_thread->join();
    delete(m_error_simulation_thread);
    m_error_simulation_thread = 0;
  }
  // close 2 threads creating sockets for new tcp clients on ip ports 2201 (results), 2111 (cola command requests)
  std::vector<thread_ptr*> tcp_connection_threads = { &m_tcp_connection_thread_results, &m_tcp_connection_thread_cola };
  for(std::vector<thread_ptr*>::iterator iter_connection_thread = tcp_connection_threads.begin(); iter_connection_thread != tcp_connection_threads.end(); iter_connection_thread++)
  {
    thread_ptr & tcp_connection_thread = **iter_connection_thread;
    if(tcp_connection_thread)
    {
      tcp_connection_thread->join();
      delete(tcp_connection_thread);
      tcp_connection_thread = 0;
    }
  }
  // close send scandata thread
  closeScandataThread();
  // close sockets
  closeTcpConnections();
  // close worker/client threads
  closeWorkerThreads();
  return true;
}

/*!
 * Closes the send scandata thread
 */
void sick_scan_xd::TestServerThread::closeScandataThread(void)
{
  if(m_tcp_send_scandata_thread)
  {
    m_tcp_send_scandata_thread_running = false;
    m_tcp_send_scandata_thread->join();
    delete(m_tcp_send_scandata_thread);
    m_tcp_send_scandata_thread = 0;
  }
}

/*!
 * Closes all tcp connections
 * @param[in] force_shutdown if true, sockets are immediately forced to shutdown
 */
void sick_scan_xd::TestServerThread::closeTcpConnections(bool force_shutdown)
{
  for(std::list<sick_scan_xd::ServerSocket*>::iterator socket_iter = m_tcp_sockets.begin(); socket_iter != m_tcp_sockets.end(); socket_iter++)
  {
    try
    {
      (*socket_iter)->close();
    }
    catch(std::exception & exc)
    {
      ROS_WARN_STREAM("TestServerThread::closeTcpConnections(): exception " << exc.what() << " on closing socket.");
    }
    *socket_iter = 0;
  }
  m_tcp_sockets.clear();
}

/*!
 * Closes a socket.
 * @param[in,out] p_socket socket to be closed
 */
void sick_scan_xd::TestServerThread::closeSocket(socket_ptr & p_socket)
{
  std::lock_guard<std::mutex> worker_thread_lockguard(m_tcp_worker_threads_mutex);
  try
  {
    if(p_socket)
    {
      for(std::list<sick_scan_xd::ServerSocket*>::iterator socket_iter = m_tcp_sockets.begin(); socket_iter != m_tcp_sockets.end(); )
      {
        if(p_socket == (*socket_iter))
        {
          (*socket_iter)->close();
          socket_iter = m_tcp_sockets.erase(socket_iter);
        }
        else
          socket_iter++;
      }
      p_socket = 0;
    }
  }
  catch(std::exception & exc)
  {
    ROS_WARN_STREAM("TestServerThread::closeSocket(): exception " << exc.what() << " on closing socket.");
  }
}

/*!
 * Stops all worker threads
 */
void sick_scan_xd::TestServerThread::closeWorkerThreads(void)
{
  m_worker_thread_running = false;
  std::lock_guard<std::mutex> worker_thread_lockguard(m_tcp_worker_threads_mutex);
  for(std::list<thread_ptr>::iterator thread_iter = m_tcp_worker_threads.begin(); thread_iter != m_tcp_worker_threads.end(); thread_iter++)
  {
    std::thread *p_thread = *thread_iter;
    p_thread->join();
    delete(p_thread);
  }
  m_tcp_worker_threads.clear();
}

/*!
 * Callback for result telegram messages (SickLocResultPortTelegramMsg) from sim_loc_driver.
 * Buffers the last telegram to monitor sim_loc_driver messages in error simulation mode.
 * @param[in] msg result telegram message (SickLocResultPortTelegramMsg)
 */
void sick_scan_xd::TestServerThread::messageCbResultPortTelegrams(const sick_scan_xd::SickLocResultPortTelegramMsg & msg)
{
  m_last_telegram_received.set(msg);
}

/*!
 * Thread callback, listens and accept tcp connections from clients for result telegrams.
 * Starts a new worker thread to generate result port telegrams for each tcp client.
 */
void sick_scan_xd::TestServerThread::runConnectionThreadResultCb(void)
{
  runConnectionThreadGenericCb(m_ip_port_results, &sick_scan_xd::TestServerThread::runWorkerThreadResultCb);
}

/*!
 * Thread callback, listens and accept tcp connections from clients for cola telegrams.
 * Starts a new worker thread to receive command requests for each tcp client.
 */
void sick_scan_xd::TestServerThread::runConnectionThreadColaCb(void)
{
  runConnectionThreadGenericCb(m_ip_port_cola, &sick_scan_xd::TestServerThread::runWorkerThreadColaCb);
}

/*!
 * Thread callback, listens and accept tcp connections from clients.
 * Starts a worker thread for each tcp client.
 */
template<typename Callable>void sick_scan_xd::TestServerThread::runConnectionThreadGenericCb(int ip_port_results, Callable thread_function_cb)
{
  ROS_INFO_STREAM("TestServerThread: connection thread started");
  while(ROS::ok() && m_tcp_connection_thread_running)
  {
    if(m_error_simulation_flag.get() == DONT_LISTEN) // error simulation: testserver does not open listening port
    {
      ROS::sleep(0.1);
      continue;
    }
    
    // normal mode: listen to tcp port, accept and connect to new tcp clients
    sick_scan_xd::ServerSocket* tcp_client_socket = new sick_scan_xd::ServerSocket();
    
    // Accecpt tcp connection
    if(tcp_client_socket->open(ip_port_results))
    {
      ROS_INFO_STREAM("TestServerThread: listening to tcp connections on port " << ip_port_results);
    }
    else
    {
      ROS_ERROR_STREAM("## ERROR TestServerThread: can't open port " << ip_port_results);
      tcp_client_socket->close();
      ROS::sleep(0.1);
      continue; 
    }
    if(m_error_simulation_flag.get() == DONT_ACCECPT) // error simulation: testserver does not does not accecpt tcp clients
    {
      ROS::sleep(0.1);
      continue;
    }

    // Connect to tcp client
    if(tcp_client_socket->connect())
    {
      ROS_INFO_STREAM("TestServerThread: connected to client");
    }
    else
    {
      ROS_ERROR_STREAM("## ERROR TestServerThread: can't connect (port " << ip_port_results << ")");
      tcp_client_socket->close();
      ROS::sleep(0.1);
      continue; 
    }

    // tcp client connected, start worker thread
    ROS_INFO_STREAM("TestServerThread: established new tcp client connection");
    std::lock_guard<std::mutex> worker_thread_lockguard(m_tcp_worker_threads_mutex);
    m_tcp_sockets.push_back(tcp_client_socket);
    m_worker_thread_running = true;
    std::thread* worker_thread = new std::thread(thread_function_cb, this, tcp_client_socket);
    m_tcp_worker_threads.push_back(worker_thread);
    worker_thread->join();
    tcp_client_socket->close();
  }
  closeTcpConnections();
  m_tcp_connection_thread_running = false;
  ROS_INFO_STREAM("TestServerThread: connection thread finished");
}

/*!
 * Worker thread callback, generates and sends result telegrams to a tcp client.
 * There's one result worker thread for each tcp client.
 * @param[in] p_socket socket to send result telegrams to the tcp client
 */
void sick_scan_xd::TestServerThread::runWorkerThreadResultCb(socket_ptr p_socket)
{
  ROS_INFO_STREAM("TestServerThread: worker thread for result telegrams started");
  sick_scan_xd::UniformRandomInteger random_generator(0,255);
  sick_scan_xd::UniformRandomInteger random_length(1, 512);
  sick_scan_xd::UniformRandomInteger random_integer(0, INT_MAX);
  double circle_yaw = 0;
  while(ROS::ok() && m_worker_thread_running && p_socket && p_socket->is_open())
  {
    ROS::sleep((double)sick_scan_xd::TestcaseGenerator::ResultPoseInterval() / m_result_telegram_rate);
    if (m_error_simulation_flag.get() == DONT_SEND) // error simulation: testserver does not send any telegrams
    {
      ROS_DEBUG_STREAM("TestServerThread for cresult telegrams: error simulation, server not sending any telegrams");
      continue;
    }
    if (m_error_simulation_flag.get() == SEND_RANDOM_TCP) // error simulation: testserver sends invalid random tcp packets
    {
      std::vector<uint8_t> random_data = random_generator.generate(random_length.generate()); // binary random data of random size
      if(!p_socket->write(random_data.data(), random_data.size()))
        ROS_ERROR_STREAM("## ERROR TestServerThread: failed to send " << random_data.size() << "byte random data " << sick_scan_xd::Utils::toHexString(random_data));
      else
        ROS_DEBUG_STREAM("TestServerThread for result telegrams: send random data " << sick_scan_xd::Utils::toHexString(random_data));
      continue;
    }
    // create testcase is a result port telegram with random based sythetical data
    sick_scan_xd::SickLocResultPortTestcaseMsg testcase = sick_scan_xd::TestcaseGenerator::createRandomResultPortTestcase();
    if(m_demo_move_in_circles) // simulate a sensor moving in circles
    {
      testcase = sick_scan_xd::TestcaseGenerator::createResultPortCircles(2.0, circle_yaw);
      circle_yaw = sick_scan_xd::Utils::normalizeAngle(circle_yaw + 1.0 * M_PI / 180);
    }
    if (m_error_simulation_flag.get() == SEND_INVALID_TELEGRAMS) // error simulation: testserver sends invalid telegrams (invalid data, false checksums, etc.)
    {
      int number_random_bytes = ((random_integer.generate()) % (testcase.binary_data.size()));
      for(int cnt_random_bytes = 0; cnt_random_bytes < number_random_bytes; cnt_random_bytes++)
      {
        int byte_cnt = ((random_integer.generate()) % (testcase.binary_data.size()));
        testcase.binary_data[byte_cnt] = (uint8_t)(random_generator.generate() & 0xFF);
      }
      ROS_DEBUG_STREAM("TestServerThread for result telegrams: send random binary telegram " << sick_scan_xd::Utils::toHexString(testcase.binary_data));
    }
    // send binary result port telegram to tcp client (if localization is "on")
    if(sick_scan_xd::TestcaseGenerator::LocalizationEnabled() && sick_scan_xd::TestcaseGenerator::ResultTelegramsEnabled())
    {
      if(!p_socket->write(testcase.binary_data.data(), testcase.binary_data.size()))
      {
        std::stringstream error_info;
        error_info << "## ERROR TestServerThread for result telegrams: failed to send " << testcase.binary_data.size() << " bytes binary result port telegram";
        if (m_error_simulation_flag.get() == SIMU_NO_ERROR)
        {
          ROS_WARN_STREAM(error_info.str() << ", close socket and leave worker thread for result telegrams");
          break;
        }
        ROS_DEBUG_STREAM(error_info.str());
      }
      else
      {
        ROS_DEBUG_STREAM("TestServerThread for result telegrams: send binary result port telegram " << sick_scan_xd::Utils::toHexString(testcase.binary_data));
      }
      // publish testcases (SickLocResultPortTestcaseMsg, i.e. binary telegram and SickLocResultPortTelegramMsg messages) for test and verification of sim_loc_driver
      testcase.header.stamp = ROS::now();
      testcase.header.frame_id = m_result_testcases_frame_id;
      ROS_PUBLISH(m_result_testcases_publisher, testcase);
    }
  }
  closeSocket(p_socket);
  ROS_INFO_STREAM("TestServerThread: worker thread for result telegrams finished");
}

/*!
 * Worker thread callback, receives command requests from a tcp client
 * and sends a synthetical command response.
 * There's one request worker thread for each tcp client.
 * @param[in] p_socket socket to receive command requests from the tcp client
 */
void sick_scan_xd::TestServerThread::runWorkerThreadColaCb(socket_ptr p_socket)
{
  ROS_INFO_STREAM("TestServerThread: worker thread for command requests started");
  int iTransmitErrorCnt = 0;
  sick_scan_xd::UniformRandomInteger random_generator(0,255);
  sick_scan_xd::UniformRandomInteger random_length(1, 128);
  sick_scan_xd::UniformRandomAsciiString random_ascii;
  while(ROS::ok() && m_worker_thread_running && p_socket && p_socket->is_open())
  {
    // Read command request from tcp client
    ServerColaRequest request;
    ROS::Time receive_timestamp;
    if(sick_scan_xd::ColaTransmitter::receive(p_socket->connectedSocket(), request.telegram_data, 1, receive_timestamp))
    {
      if (m_error_simulation_flag.get() == DONT_SEND) // error simulation: testserver does not send any telegrams
      {
        ROS_DEBUG_STREAM("TestServerThread for command requests: error simulation, server not sending any telegrams");
        continue;
      }
      if (m_error_simulation_flag.get() == SEND_RANDOM_TCP) // error simulation: testserver sends invalid random tcp packets
      {
        std::vector<uint8_t> random_data = random_generator.generate(random_length.generate()); // binary random data of random size
        if(!p_socket->write(random_data.data(), random_data.size()))
          ROS_ERROR_STREAM("## ERROR TestServerThread for command requests: send random data " << sick_scan_xd::Utils::toHexString(random_data) << " failed.");
        else
          ROS_DEBUG_STREAM("TestServerThread for command requests: send random data " << sick_scan_xd::Utils::toHexString(random_data));
        continue;
      }
      // command requests received, generate and send a synthetical response
      bool cola_binary = sick_scan_xd::ColaAsciiBinaryConverter::IsColaBinary(request.telegram_data);
      if(cola_binary)
        request.telegram_data = sick_scan_xd::ColaAsciiBinaryConverter::ColaBinaryToColaAscii(request.telegram_data);
      std::string ascii_telegram = sick_scan_xd::ColaAsciiBinaryConverter::ConvertColaAscii(request.telegram_data);
      ROS_INFO_STREAM("TestServerThread: received cola request " << ascii_telegram);
      sick_scan_xd::SickLocColaTelegramMsg telegram_msg = sick_scan_xd::ColaParser::decodeColaTelegram(ascii_telegram);
      // Generate a synthetical response depending on the request
      sick_scan_xd::SickLocColaTelegramMsg telegram_answer = sick_scan_xd::TestcaseGenerator::createColaResponse(telegram_msg, m_scanner_type);
      if (m_error_simulation_flag.get() == SEND_INVALID_TELEGRAMS) // error simulation: testserver sends invalid telegrams (invalid data, false checksums, etc.)
      {
        telegram_answer.command_name = random_ascii.generate(random_length.generate()); // random ascii string
        telegram_answer.parameter.clear();
        for(int n = random_length.generate(); n > 0; n--)
          telegram_answer.parameter.push_back(random_ascii.generate(random_length.generate())); // random ascii string
        ROS_DEBUG_STREAM("TestServerThread for result telegrams: send random cola response " << sick_scan_xd::Utils::flattenToString(telegram_answer));
      }
      std::vector<uint8_t> binary_response = sick_scan_xd::ColaParser::encodeColaTelegram(telegram_answer);
      std::string ascii_response = sick_scan_xd::ColaAsciiBinaryConverter::ConvertColaAscii(binary_response);
      if(cola_binary) // binary_response = sick_scan_xd::ColaAsciiBinaryConverter::ColaAsciiToColaBinary(binary_response);
        binary_response = sick_scan_xd::ColaAsciiBinaryConverter::ColaTelegramToColaBinary(telegram_answer);
      ROS_INFO_STREAM("TestServerThread: sending cola response " << ascii_response << " (" << m_scanner_type << ", " << (cola_binary ? "Cola-Binary)" : "Cola-ASCII)"));
      // Send command response to tcp client
      if(cola_binary)
        ROS_DEBUG_STREAM("TestServerThread: sending cola hex response " << sick_scan_xd::Utils::toHexString(binary_response));
      ROS::Time send_timestamp;
      if (!sick_scan_xd::ColaTransmitter::send(p_socket->connectedSocket(), binary_response, send_timestamp))
      {
        ROS_WARN_STREAM("## ERROR TestServerThread: failed to send cola response, ColaTransmitter::send() returned false, data hexdump: " << sick_scan_xd::Utils::toHexString(binary_response));
        iTransmitErrorCnt++;
        if(iTransmitErrorCnt >= 10)
        {
          ROS_WARN_STREAM("## ERROR TestServerThread: " << iTransmitErrorCnt << " transmission errors, giving up.");
          ROS_WARN_STREAM("## ERROR TestServerThread: shutdown after error, aborting");
          ROS::shutdown();
        }
      }
      else
      {
        iTransmitErrorCnt = 0;
      }
      // Start or stop scandata thread
      bool send_scandata_enabled = sick_scan_xd::TestcaseGenerator::SendScandataEnabled() // measurement mode set
        || m_start_scandata_immediately // command line option
        || m_scanner_type == "sick_tim_240"; // TIM240 does not support "LMCstartmeas"
      if(send_scandata_enabled == true && m_tcp_send_scandata_thread == 0)
      {
        // Start scandata thread
        m_tcp_send_scandata_thread_running = true;
        m_tcp_send_scandata_thread = new std::thread(&sick_scan_xd::TestServerThread::runWorkerThreadScandataCb, this, p_socket);
      }
      else if(send_scandata_enabled == false && m_tcp_send_scandata_thread != 0)
      {
        // Stop scandata thread
        closeScandataThread();
      }
    }
    ROS::sleep(0.0001);
  }
  closeScandataThread();
  closeSocket(p_socket);
  ROS_INFO_STREAM("TestServerThread: worker thread for command requests finished");
}

/*!
 * Worker thread callback, sends scandata and scandatamon messages to the tcp client.
 * Reads scandata and scandatamon messages from jsonfile and sends the messages in a loop
 * while m_tcp_send_scandata_thread_running is true.
 * @param[in] p_socket socket to sends scandata and scandatamon messages the tcp client
 */
void sick_scan_xd::TestServerThread::runWorkerThreadScandataCb(socket_ptr p_socket)
{
  std::vector<sick_scan_xd::JsonScanData> binary_messages;
  ROS_INFO_STREAM("TestServerThread: worker thread sending scandata and scandatamon messages started.");
  if(m_scandatafiles.empty())
  {
     ROS_WARN_STREAM("## WARNING TestServerThread::runWorkerThreadScandataCb(): no scandata files configured, aborting worker thread to send scandata.");   
     m_tcp_send_scandata_thread_running = false;
     return;
  }
  if(ROS::ok() && m_tcp_send_scandata_thread_running)
  {
    // Read scandata and scandatamon messages from jsonfile
    std::vector<std::string> scandatafiles = sick_scan_xd::PcapngJsonParser::split(m_scandatafiles, ',');
    std::vector<std::string> scandatatypes = sick_scan_xd::PcapngJsonParser::split(m_scandatatypes, ',');
    double start_timestamp = 0;
    for(int n = 0; n < scandatafiles.size(); n++)
    {
      if(!sick_scan_xd::PcapngJsonParser::parseJsonfile(scandatafiles[n], scandatatypes, start_timestamp, binary_messages) || binary_messages.empty())
      {
        ROS_WARN_STREAM("## WARNING TestServerThread: error reading file \"" << scandatafiles[n] << "\".");   
      }
      else
      {
        ROS_INFO_STREAM("TestServerThread: file \"" << scandatafiles[n] << "\" successfully parsed, " << binary_messages.size() << " messages of types \"" << m_scandatatypes << "\".");
        start_timestamp = binary_messages.back().timestamp + 0.01;
      }
    }
  }
  if(binary_messages.empty())
  {
     ROS_WARN_STREAM("## WARNING TestServerThread::runWorkerThreadScandataCb(): no scandata found, aborting worker thread to send scandata.");   
     m_tcp_send_scandata_thread_running = false;
     return;
  }
  ROS::sleep(m_start_scandata_delay); // delay between scandata activation ("LMCstartmeas" request) and first scandata message, default: 1 second
  double last_msg_timestamp = ((binary_messages.empty()) ? 0 : (binary_messages[0].timestamp));
  int iTransmitErrorCnt = 0;
  for(int msg_cnt = 0, msg_cnt_delta = 1; ROS::ok() && m_tcp_send_scandata_thread_running && p_socket && p_socket->is_open(); msg_cnt+=msg_cnt_delta)
  {
    if(msg_cnt >= (int)binary_messages.size()) // revert messages
    {
      msg_cnt = (int)binary_messages.size() - 2;
      msg_cnt_delta *= -1;
    }
    if(msg_cnt < 0) // revert messages
    {
      msg_cnt = std::min(1, (int)binary_messages.size() - 1);
      msg_cnt_delta *= -1;
    }
    sick_scan_xd::JsonScanData& binary_message = binary_messages[msg_cnt];
    ROS::sleep(std::max(0.001, std::abs(binary_message.timestamp - last_msg_timestamp)));
    // Send messages
    // ROS_DEBUG_STREAM("TestServerThread: sending scan data " << sick_scan_xd::Utils::toHexString(binary_message.data));
    ROS_INFO_STREAM("TestServerThread: sending " << binary_message.data.size() << " byte scan data " 
      << sick_scan_xd::Utils::toAsciiString(&binary_message.data[0], std::min(32, (int)binary_message.data.size())) << " ... ");
    ROS::Time send_timestamp = ROS::now();
    if (!sick_scan_xd::ColaTransmitter::send(p_socket->connectedSocket(), binary_message.data, send_timestamp))
    {
      ROS_WARN_STREAM("## ERROR TestServerThread: failed to send cola response, ColaTransmitter::send() returned false, data hexdump: " << sick_scan_xd::Utils::toHexString(binary_message.data));
      iTransmitErrorCnt++;
      if(iTransmitErrorCnt >= 10)
      {
        ROS_WARN_STREAM("## ERROR TestServerThread: " << iTransmitErrorCnt << " transmission errors, giving up.");
        ROS_WARN_STREAM("## ERROR TestServerThread: shutdown after error, aborting");
        ROS::shutdown();
      }
    }
    else
    {
      iTransmitErrorCnt = 0;
    }
    last_msg_timestamp = binary_message.timestamp;
    ROS::sleep(0.001);
  }
  m_tcp_send_scandata_thread_running = false;
  ROS_INFO_STREAM("TestServerThread: worker thread sending scandata and scandatamon messages finished.");
}

/*!
 * Waits for a given time in seconds, as long as ROS::ok() and m_error_simulation_thread_running == true.
 * @param[in] seconds delay in seconds
 */
void sick_scan_xd::TestServerThread::errorSimulationWait(double seconds)
{
  ROS::Time starttime = ROS::now();
  while(ROS::ok() && m_error_simulation_thread_running && ROS::seconds(ROS::now() - starttime) < seconds)
  {
    ROS::sleep(0.001);
  }
}

/*!
 * Waits for and returns the next telegram message from sick_scan driver.
 * @param[in] timeout_seconds wait timeout in seconds
 * @param[out] telegram_msg last telegram message received
 * @return true, if a new telegram message received, false otherwise (timeout or shutdown)
 */
bool sick_scan_xd::TestServerThread::errorSimulationWaitForTelegramReceived(double timeout_seconds, sick_scan_xd::SickLocResultPortTelegramMsg & telegram_msg)
{
  ROS::Time starttime = ROS::now();
  while(ROS::ok() && m_error_simulation_thread_running)
  {
    telegram_msg = m_last_telegram_received.get();
    if(ROS::timeFromHeader(&telegram_msg.header) > starttime)
      return true; // new telegram received
    if(ROS::seconds(ROS::now() - starttime) >= timeout_seconds)
      return false; // timeout
    ROS::sleep(0.001);
  }
  return false;
}

/*!
 * Thread callback, runs an error simulation and switches m_error_simulation_flag through the error test cases.
 */
void sick_scan_xd::TestServerThread::runErrorSimulationThreadCb(void)
{
  sick_scan_xd::SickLocResultPortTelegramMsg telegram_msg;
  size_t number_testcases = 0, number_testcases_failed = 0;
  ROS_INFO_STREAM("TestServerThread: error simulation thread started");
  
  // Error simulation: start normal execution
  number_testcases++;
  m_error_simulation_flag.set(SIMU_NO_ERROR);
  ROS_INFO_STREAM("TestServerThread: 1. error simulation testcase: normal execution, expecting telegram messages from driver");
  errorSimulationWait(10);
  if(!errorSimulationWaitForTelegramReceived(10, telegram_msg))
  {
    number_testcases_failed++;
    ROS_WARN_STREAM("## ERROR TestServerThread: 1. error simulation testcase: no ros telegram message received, expected SickLocResultPortTelegramMsg from driver");
  }
  else
    ROS_INFO_STREAM("TestServerThread: 1. error simulation testcase: received telegram message \"" << sick_scan_xd::Utils::flattenToString(telegram_msg) << "\", okay");
  
  // Error simulation testcase: testserver does not open listening port for 10 seconds, normal execution afterwards.
  number_testcases++;
  m_error_simulation_flag.set(DONT_LISTEN);
  ROS_INFO_STREAM("TestServerThread: 2. error simulation testcase: server not responding, not listening, no tcp connections accepted.");
  m_worker_thread_running = false;
  ROS::sleep(1.0 / m_result_telegram_rate);
  closeTcpConnections(true);
  errorSimulationWait(10);
  m_error_simulation_flag.set(SIMU_NO_ERROR);
  ROS_INFO_STREAM("TestServerThread: 2. error simulation testcase: switched to normal execution, expecting telegram messages from driver");
  errorSimulationWait(10);
  if(!errorSimulationWaitForTelegramReceived(10, telegram_msg))
  {
    number_testcases_failed++;
    ROS_WARN_STREAM("## ERROR TestServerThread: 2. error simulation testcase: no ros telegram message received, expected SickLocResultPortTelegramMsg from driver");
  }
  else
    ROS_INFO_STREAM("TestServerThread: 2. error simulation testcase: received telegram message \"" << sick_scan_xd::Utils::flattenToString(telegram_msg) << "\", okay");
  
  // Error simulation testcase: testserver does not accecpt tcp clients for 10 seconds, normal execution afterwards.
  number_testcases++;
  m_error_simulation_flag.set(DONT_ACCECPT);
  ROS_INFO_STREAM("TestServerThread: 3. error simulation testcase: server not responding, listening on port " << m_ip_port_results << ", but accepting no tcp clients");
  m_worker_thread_running = false;
  ROS::sleep(1.0 / m_result_telegram_rate);
  closeTcpConnections(true);
  errorSimulationWait(10);
  m_error_simulation_flag.set(SIMU_NO_ERROR);
  errorSimulationWait(10);
  if(!errorSimulationWaitForTelegramReceived(10, telegram_msg))
  {
    number_testcases_failed++;
    ROS_WARN_STREAM("## ERROR TestServerThread: 3. error simulation testcase: no ros telegram message received, expected SickLocResultPortTelegramMsg from driver");
  }
  else
    ROS_INFO_STREAM("TestServerThread: 3. error simulation testcase: received telegram message \"" << sick_scan_xd::Utils::flattenToString(telegram_msg) << "\", okay");
  
  // Error simulation testcase: testserver does not send telegrams for 10 seconds, normal execution afterwards.
  number_testcases++;
  m_error_simulation_flag.set(DONT_SEND);
  ROS_INFO_STREAM("TestServerThread: 4. error simulation testcase: server not sending telegrams");
  errorSimulationWait(10);
  m_error_simulation_flag.set(SIMU_NO_ERROR);
  errorSimulationWait(10);
  if(!errorSimulationWaitForTelegramReceived(10, telegram_msg))
  {
    number_testcases_failed++;
    ROS_WARN_STREAM("## ERROR TestServerThread: 4. error simulation testcase: no ros telegram message received, expected SickLocResultPortTelegramMsg from driver");
  }
  else
    ROS_INFO_STREAM("TestServerThread: 4. error simulation testcase: received telegram message \"" << sick_scan_xd::Utils::flattenToString(telegram_msg) << "\", okay");
  
  // Error simulation testcase: testserver sends random tcp data for 10 seconds, normal execution afterwards.
  number_testcases++;
  m_error_simulation_flag.set(SEND_RANDOM_TCP);
  ROS_INFO_STREAM("TestServerThread: 5. error simulation testcase: server sending random tcp data");
  errorSimulationWait(10);
  m_error_simulation_flag.set(SIMU_NO_ERROR);
  errorSimulationWait(10);
  if(!errorSimulationWaitForTelegramReceived(10, telegram_msg))
  {
    number_testcases_failed++;
    ROS_WARN_STREAM("## ERROR TestServerThread: 5. error simulation testcase: no ros telegram message received, expected SickLocResultPortTelegramMsg from driver");
  }
  else
    ROS_INFO_STREAM("TestServerThread: 5. error simulation testcase: received telegram message \"" << sick_scan_xd::Utils::flattenToString(telegram_msg) << "\", okay");
  
  // Error simulation testcase: testserver sends invalid telegrams (invalid data, false checksums, etc.) for 10 seconds, normal execution afterwards.
  number_testcases++;
  m_error_simulation_flag.set(SEND_INVALID_TELEGRAMS);
  ROS_INFO_STREAM("TestServerThread: 6. error simulation testcase: server sending invalid telegrams");
  errorSimulationWait(10);
  m_error_simulation_flag.set(SIMU_NO_ERROR);
  errorSimulationWait(10);
  if(!errorSimulationWaitForTelegramReceived(10, telegram_msg))
  {
    number_testcases_failed++;
    ROS_WARN_STREAM("## ERROR TestServerThread: 6. error simulation testcase: no ros telegram message received, expected SickLocResultPortTelegramMsg from driver");
  }
  else
    ROS_INFO_STREAM("TestServerThread: 6. error simulation testcase: received telegram message \"" << sick_scan_xd::Utils::flattenToString(telegram_msg) << "\", okay");
  
  // End of error simulation, print summary and exit
  m_error_simulation_thread_running = false;
  ROS_INFO_STREAM("TestServerThread: error simulation thread finished");
  ROS_INFO_STREAM("TestServerThread: error simulation summary: " << (number_testcases - number_testcases_failed) << " of " << number_testcases<< " testcases passed, " << number_testcases_failed << " failures.");
  ROS_INFO_STREAM("TestServerThread: exit with ros::shutdown()");
  ROS::shutdown();
}
