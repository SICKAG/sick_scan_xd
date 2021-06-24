/*
 * @brief client_socket encapsulates connecting, closing and setting socket options
 * for tcp client sockets using boost::asio::ip::tcp::socket and boost::asio::io_service.
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

#include "sick_scan/client_socket.h"

/*!
 * Constructor.
 * @param[in] io_service boost io service for tcp connections (several sockets may share one io_service)
 */
sick_scan::ClientSocket::ClientSocket(boost::asio::io_service & io_service) : m_tcp_socket(io_service)
{
}

/*!
 * Destructor, closes all tcp connections.
 */
sick_scan::ClientSocket::~ClientSocket()
{
  close();
}

/*!
 * Connects to a server.
 * @param[in] io_service boost io service for tcp connections (several sockets may share one io_service)
 * @param[in] server_adress ip adress of the localization controller, default: 192.168.0.1
 * @param[in] tcp_port tcp port for command requests, default: 2111 for command requests and 2112 for  command responses
 * @return true on success, false on failure (server unknown or unreachable)
 */
bool sick_scan::ClientSocket::connect(boost::asio::io_service & io_service, const std::string & server_adress, int tcp_port)
{
  try
  {
    // Connect to server
    boost::asio::ip::tcp::resolver tcpresolver(io_service);
    boost::asio::ip::tcp::resolver::query tcpquery(server_adress, std::to_string(tcp_port));
    boost::asio::ip::tcp::resolver::iterator it = tcpresolver.resolve(tcpquery);
    boost::system::error_code errorcode;
    m_tcp_socket.connect(*it, errorcode);
    
    if(!errorcode && m_tcp_socket.is_open())
    {
      // Get and set options for client sockets
      boost::system::error_code socket_option_errorcodes[3];
      boost::asio::ip::tcp::no_delay socket_option_no_delay;
      boost::asio::socket_base::send_buffer_size socket_option_send_buffer_size;
      boost::asio::socket_base::receive_buffer_size socket_option_receive_buffer_size;
      
      m_tcp_socket.get_option(socket_option_no_delay, socket_option_errorcodes[0]);
      m_tcp_socket.get_option(socket_option_send_buffer_size, socket_option_errorcodes[1]);
      m_tcp_socket.get_option(socket_option_receive_buffer_size, socket_option_errorcodes[2]);
      
      if (socket_option_errorcodes[0] || socket_option_no_delay.value() == false)
        m_tcp_socket.set_option(boost::asio::ip::tcp::no_delay(true), socket_option_errorcodes[0]);
      if (socket_option_errorcodes[1] || socket_option_send_buffer_size.value() < 64 * 1024)
        m_tcp_socket.set_option(boost::asio::socket_base::send_buffer_size(64 * 1024), socket_option_errorcodes[1]);
      if (socket_option_errorcodes[2] || socket_option_receive_buffer_size.value() < 64 * 1024)
        m_tcp_socket.set_option(boost::asio::socket_base::receive_buffer_size(64 * 1024), socket_option_errorcodes[2]);
      
      m_tcp_socket.get_option(socket_option_no_delay, socket_option_errorcodes[0]);
      m_tcp_socket.get_option(socket_option_send_buffer_size, socket_option_errorcodes[1]);
      m_tcp_socket.get_option(socket_option_receive_buffer_size, socket_option_errorcodes[2]);
      
      if(socket_option_errorcodes[0] || socket_option_errorcodes[1] || socket_option_errorcodes[2])
      {
        ROS_WARN_STREAM("## ClientSocket::connect(): socket connected to " << server_adress << ":" << tcp_port << ", but socket::get_option() failed, "
          << " socket options error messages: no_delay=" << socket_option_errorcodes[0].message() << ", send_buffer_size=" << socket_option_errorcodes[1].message()
          << ", receive_buffer_size=" << socket_option_errorcodes[2].message());
      }
      ROS_INFO_STREAM("ClientSocket::connect(): socket connected to " << server_adress << ":" << tcp_port << ", socket options values: no_delay=" << socket_option_no_delay.value()
        << ", send_buffer_size=" << socket_option_send_buffer_size.value() << ", receive_buffer_size=" << socket_option_receive_buffer_size.value());
    }
    else
    {
      ROS_WARN_STREAM("ClientSocket::connect(): no connection to localization controller " << server_adress << ":" << tcp_port << ", error " << errorcode.value() << " \"" << errorcode.message() << "\"");
      return false;
    }
    return true;
  }
  catch(std::exception & exc)
  {
    ROS_WARN_STREAM("## ERROR ClientSocket::connect(): connect to " << server_adress << ":" << tcp_port << " failed, exception " << exc.what());
  }
  return false;
}

/*!
 * Closes the tcp connection to the server.
 * @param[in] force_shutdown if true, the socket is shutdown even if it's state is not opened or connected
 * (otherwise the socket is closed, if its state is currently opened)
 * @return true on success (socket closed), false on failure
 */
bool sick_scan::ClientSocket::close(bool force_shutdown)
{
  try
  {
    if (force_shutdown || m_tcp_socket.is_open())
    {
      m_tcp_socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
      m_tcp_socket.close();
    }
    return true;
  }
  catch(std::exception & exc)
  {
    ROS_WARN_STREAM("ColaTransmitter::closeTcpConnections(): exception " << exc.what() << " on closing connection.");
  }
  return false;
}
    
