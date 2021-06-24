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
#ifndef __SIM_LOC_CLIENT_SOCKET_H_INCLUDED
#define __SIM_LOC_CLIENT_SOCKET_H_INCLUDED

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>

namespace sick_scan
{
  /*!
   * Class sick_scan::ClientSocket encapsulates connecting, closing and setting socket options
   * for tcp client sockets implemented by boost::asio::ip::tcp::socket.
   */
  class ClientSocket
  {
  public:
  
    /*!
     * Constructor.
     * @param[in] io_service boost io service for tcp connections (several sockets may share one io_service)
     */
    ClientSocket(boost::asio::io_service & io_service);
  
    /*!
     * Destructor, closes all tcp connections.
     */
    virtual ~ClientSocket();
  
    /*!
     * Connects to a server.
     * @param[in] io_service boost io service for tcp connections (several sockets may share one io_service)
     * @param[in] server_adress ip adress of the localization controller, default: 192.168.0.1
     * @param[in] tcp_port tcp port for command requests, default: 2111 for command requests and 2112 for  command responses
     * @return true on success, false on failure (server unknown or unreachable)
     */
    virtual bool connect(boost::asio::io_service & io_service, const std::string & server_adress, int tcp_port);
  
    /*!
     * Closes the tcp connection to the server.
     * @param[in] force_shutdown if true, the socket is shutdown even if it's state is not opened or connected
     * (otherwise the socket is closed, if its state is currently opened)
     * @return true on success (socket closed), false on failure
     */
    virtual bool close(bool force_shutdown = false);
  
    /*!
     * Returns the tcp client socket implementation
     * @return tcp client socket implementation
     */
    virtual boost::asio::ip::tcp::socket & socket(void) { return m_tcp_socket; }
    
  protected:
  
    boost::asio::ip::tcp::socket m_tcp_socket; ///< tcp client socket implementation
    
  }; // class ClientSocket
  
} // namespace sick_scan
#endif // __SIM_LOC_CLIENT_SOCKET_H_INCLUDED
