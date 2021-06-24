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
#ifndef __SIM_LOC_TEST_SERVER_THREAD_H_INCLUDED
#define __SIM_LOC_TEST_SERVER_THREAD_H_INCLUDED

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <list>

#include "sick_scan/fifo_buffer.h"
#include "sick_scan/utils.h"

namespace sick_scan
{
  /*!
   * class TestServerThread runs a thread to listen and accept tcp connections from clients
   * and generates telegrams to test the ros driver for sim localization.
   */
  class TestServerThread
  {
  public:
    
    /*!
     * Constructor. The server thread does not start automatically, call start() and stop() to start and stop the server.
     * @param[in] nh ros node handle
     * @param[in] ip_port_results ip port for result telegrams, default: 2201
     * @param[in] ip_port_cola ip port for command requests and responses, default: 2111
     */
    TestServerThread(ROS::NodePtr nh = 0, int ip_port_results = 2201, int ip_port_cola = 2111);
  
    /*!
     * Destructor. Stops the server thread and closes all tcp connections.
     */
    virtual ~TestServerThread();
  
    /*!
     * Starts the server thread, starts to listen and accept tcp connections from clients.
     * @return true on success, false on failure.
     */
    virtual bool start(void);
  
    /*!
     * Stops the server thread and closes all tcp connections.
     * @return true on success, false on failure.
     */
    virtual bool stop(void);

    /*!
     * Callback for result telegram messages (SickLocResultPortTelegramMsg) from sim_loc_driver.
     * Buffers the last telegram to monitor sim_loc_driver messages in error simulation mode.
     * @param[in] msg result telegram message (SickLocResultPortTelegramMsg)
     */
    void messageCbResultPortTelegrams(const sick_scan::SickLocResultPortTelegramMsg & msg);
    /*! ROS2 version of function messageCbResultPortTelegrams */
    virtual void messageCbResultPortTelegramsROS2(const std::shared_ptr<sick_scan::SickLocResultPortTelegramMsg> msg) { messageCbResultPortTelegrams(*msg); }
    
  protected:
  
    /*!
     * class ServerColaRequest: utility container for a received telegram (telegram data plus flag for Cola-Ascii or Cola-Binary)
     */
    class ServerColaRequest
    {
    public:
      ServerColaRequest(const std::vector<uint8_t> & data = std::vector<uint8_t>(), bool ascii = false) : telegram_data(data), telegram_is_ascii(ascii) {} ///< Constructor
      bool telegram_is_ascii;             ///< true: received telegram_data is Cola-Ascii encoded, false: received telegram_data is Cola-Binary encoded
      std::vector<uint8_t> telegram_data; ///< received telegram_data (Cola-Ascii or Cola-Binary)
    };
  
    typedef boost::thread* thread_ptr;                ///< shortcut for pointer to boost::thread
    typedef boost::asio::ip::tcp::socket* socket_ptr; ///< shortcut for pointer to boost::asio::ip::tcp::socket
    
    /*!
    * Closes the send scandata thread
    */
    void closeScandataThread(void);

    /*!
     * Closes all tcp connections
     * @param[in] force_shutdown if true, sockets are immediately forced to shutdown
     */
    virtual void closeTcpConnections(bool force_shutdown = false);

    /*!
     * Closes a socket.
     * @param[in,out] p_socket socket to be closed
     */
    void closeSocket(socket_ptr & p_socket);
    
    /*!
     * Stops all worker threads
     */
    void closeWorkerThreads(void);
    
    /*!
     * Thread callback, listens and accept tcp connections from clients for result telegrams.
     * Starts a new worker thread to generate result port telegrams for each tcp client.
     */
    virtual void runConnectionThreadResultCb(void);
  
    /*!
     * Thread callback, listens and accept tcp connections from clients for cola telegrams.
     * Starts a new worker thread to receive command requests for each tcp client.
     */
    virtual void runConnectionThreadColaCb(void);
  
    /*!
     * Generic thread callback, listens and accept tcp connections from clients.
     * Starts a worker thread running thread_function_cb for each tcp client.
     */
    template<typename Callable> void runConnectionThreadGenericCb(boost::asio::ip::tcp::acceptor & tcp_acceptor_results, int ip_port_results, Callable thread_function_cb);
  
    /*!
     * Worker thread callback, generates and sends result telegrams to a tcp client.
     * There's one result worker thread for each tcp client.
     * @param[in] p_socket socket to send result telegrams to the tcp client
     */
    virtual void runWorkerThreadResultCb(boost::asio::ip::tcp::socket* p_socket);
  
    /*!
     * Worker thread callback, receives command requests from a tcp client
     * and sends a synthetical command response.
     * There's one request worker thread for each tcp client.
     * @param[in] p_socket socket to receive command requests from the tcp client
     */
    virtual void runWorkerThreadColaCb(boost::asio::ip::tcp::socket* p_socket);

    /*!
     * Worker thread callback, sends scandata and scandatamon messages to the tcp client.
     * Reads scandata and scandatamon messages from jsonfile and sends the messages in a loop
     * while m_tcp_send_scandata_thread_running is true.
     * @param[in] p_socket socket to sends scandata and scandatamon messages the tcp client
     */
    virtual void runWorkerThreadScandataCb(boost::asio::ip::tcp::socket* p_socket);
  
    /*!
     * Thread callback, runs an error simulation and switches m_error_simulation_flag through the error test cases.
     */
    virtual void runErrorSimulationThreadCb(void);

    /*!
     * Waits for a given time in seconds, as long as ROS::ok() and m_error_simulation_thread_running == true.
     * @param[in] seconds delay in seconds
     */
    void errorSimulationWait(double seconds);

    /*!
     * Waits for and returns the next telegram message from sick_scan driver.
     * @param[in] timeout_seconds wait timeout in seconds
     * @param[out] telegram_msg last telegram message received
     * @return true, if a new telegram message received, false otherwise (timeout or shutdown)
     */
    bool errorSimulationWaitForTelegramReceived(double timeout_seconds, sick_scan::SickLocResultPortTelegramMsg & telegram_msg);
    
    /*
     * member data
     */

    int m_ip_port_results;                                   ///< ip port for result telegrams, default: The localization controller uses IP port number 2201 to send localization results
    int m_ip_port_cola;                                      ///< ip port for for command requests and responses, default: The localization controller uses IP port number 2111 and 2112 to send telegrams and to request data
    double m_result_telegram_rate;                           ///< frequency to generate and send result port telegrams, default: 10 Hz
    thread_ptr m_tcp_connection_thread_results;              ///< thread to accept tcp clients for result port telegrams
    thread_ptr m_tcp_connection_thread_cola;                 ///< thread to accept tcp clients for command requests and responses
    thread_ptr m_tcp_send_scandata_thread;                   ///< thread to send scandata and scandatamon tcp messages
    bool m_tcp_send_scandata_thread_running;                 ///< true: m_tcp_send_scandata_thread is running, otherwise false
    double m_start_scandata_delay;                           ///< delay between scandata activation ("LMCstartmeas" request) and first scandata message, default: 1 second
    bool m_tcp_connection_thread_running;                    ///< true: m_tcp_connection_thread is running, otherwise false
    boost::asio::io_service m_ioservice;                     ///< boost io service for tcp connections
    boost::asio::ip::tcp::acceptor m_tcp_acceptor_results;   ///< boost acceptor for tcp clients for result telegrams
    boost::asio::ip::tcp::acceptor m_tcp_acceptor_cola;      ///< boost acceptor for tcp clients for command requests and responses
    std::list<boost::asio::ip::tcp::socket*> m_tcp_sockets;  ///< list of tcp sockets (one socket for each tcp client)
    std::list<thread_ptr> m_tcp_worker_threads;              ///< list of tcp worker thread (one thread for each tcp client, generating telegrams)
    boost::mutex m_tcp_worker_threads_mutex;                 ///< mutex to protect m_tcp_worker_threads
    bool m_worker_thread_running;                            ///< true: worker threads started, otherwise false
    sick_scan::SickLocResultPortTestcaseMsgPublisher m_result_testcases_publisher;  ///< ros publisher for testcases with result port telegrams (type SickLocResultPortTestcaseMsg)
    std::string m_result_testcases_frame_id;                 ///< ros frame id of testcase messages (type SickLocResultPortTestcaseMsg), default: "result_testcases"
    bool m_demo_move_in_circles;                             ///< true: simulate a sensor moving in circles, false (default): create random based result port telegrams
    std::string m_scandatafiles;                             ///< comma separated list of jsonfiles to emulate scandata messages, f.e. "tim781s_scandata.pcapng.json,tim781s_sopas.pcapng.json"
    std::string m_scandatatypes;                             ///< comma separated list of scandata message types, f.e. "sSN LMDscandata,sSN LMDscandatamon"
    std::string m_scanner_type;                              ///< currently supported: "sick_lms_5xx", "sick_tim_7xx"

    /*
     * configuration and member data for error simulation
     */

    typedef enum ERROR_SIMULATION_ENUM ///< enumerates testcases for simulation of communication errors
    {
      NO_ERROR = 0,                ///< Default: run test server without simulated errors, send valid telegrams
      DONT_LISTEN,                 ///< Testserver does not open listening port
      DONT_ACCECPT,                ///< Testserver does not accecpt tcp clients
      DONT_SEND,                   ///< Testserver does not send any data
      SEND_RANDOM_TCP,             ///< Testserver sends invalid random tcp packets
      SEND_INVALID_TELEGRAMS       ///< Testserver sends invalid telegrams (invalid data, false checksums, etc.)
    } ERROR_SIMULATION_FLAG;
  
    sick_scan::SetGet<ERROR_SIMULATION_FLAG> m_error_simulation_flag; ///< current error simulation flag, default: NO_ERROR
    bool m_error_simulation_enabled;                         ///< default: false (no error simulation), if true, test server simulates errors and communication failures of type ERROR_SIMULATION_FLAG
    thread_ptr m_error_simulation_thread;                    ///< thread to run error simulation, switches m_error_simulation_flag through the error test cases
    bool m_error_simulation_thread_running;                  ///< true: m_error_simulation_thread is running, otherwise false
    sick_scan::SetGet<sick_scan::SickLocResultPortTelegramMsg> m_last_telegram_received; ///< last telegram message received from sick_scan driver
    
  };
  
} // namespace sick_scan
#endif // __SIM_LOC_TEST_SERVER_THREAD_H_INCLUDED
