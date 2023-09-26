/*
 * @brief server_socket implements a server socket connection.
 *
 * Copyright (C) 2021 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2021 SICK AG, Waldkirch
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
 *  Copyright 2021 SICK AG
 *  Copyright 2021 Ing.-Buero Dr. Michael Lehning
 *
 */
#include <chrono>
#include <thread>

#include "sick_scan/ros_wrapper.h"
#include "sick_scan/server_socket.h"

#ifndef SOCKET_ERROR
#define SOCKET_ERROR (-1)
#endif

#ifndef SD_BOTH
#define SD_BOTH 0x02
#endif

#ifndef _MSC_VER
  typedef struct sockaddr    SOCKADDR;
  typedef struct sockaddr_in SOCKADDR_IN;
  static int closesocket ( int fd )
  {
    return close(fd);
  }
#endif

#define TCP_SELECT_BEFORE_ACCEPT 0
#define TCP_SELECT_BEFORE_READ   0
#define TCP_SELECT_BEFORE_WRITE  0

#if defined _MSC_VER && defined min
#undef min
#endif
#if defined _MSC_VER && defined max
#undef max
#endif

/*!
 * Constructor.
 */
sick_scan_xd::ServerSocket::ServerSocket() :   m_iListenPortNumber(-1), m_tListenSocket(INVALID_SOCKET), m_tConnectedSocket(INVALID_SOCKET)
{
}

/*!
 * Destructor, closes the tcp connections.
 */
sick_scan_xd::ServerSocket::~ServerSocket()
{
}

/*!
 * Opens a listening server socket.
 * @return true on success, false on failure
 */
bool sick_scan_xd::ServerSocket::open(int tcp_port, bool bTcpAnyHost)
{
  SOCKADDR_IN tSockAddr;
  int         reuseaddr = 1;

  // Initialisation

  memset(&tSockAddr,0,sizeof(tSockAddr));
  m_iListenPortNumber = tcp_port;
  m_tListenSocket = INVALID_SOCKET;
  m_tConnectedSocket = INVALID_SOCKET;

  // Create listening socket

  m_tListenSocket = ::socket( PF_INET, SOCK_STREAM, IPPROTO_TCP );
  if( m_tListenSocket == INVALID_SOCKET )
  {
    ROS_ERROR_STREAM("## ERROR ServerSocket::open(" << m_iListenPortNumber << "," << bTcpAnyHost << "): can't open socket");
    return false;
  }

  tSockAddr.sin_family = AF_INET;
  tSockAddr.sin_port = htons((unsigned short)m_iListenPortNumber);
  if( bTcpAnyHost )
  {
    tSockAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  }
  else
  {
    tSockAddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  }

  bool bOkay = true;
  bOkay = bOkay && (setsockopt(m_tListenSocket,SOL_SOCKET,SO_REUSEADDR,(const char*)&reuseaddr,sizeof(reuseaddr)) != SOCKET_ERROR);
  bOkay = bOkay && (bind(m_tListenSocket, (SOCKADDR*)&tSockAddr, sizeof(tSockAddr)) == 0);
  bOkay = bOkay && (listen(m_tListenSocket, 1) == 0);

  if( !bOkay )
  {
    ROS_ERROR_STREAM("## ERROR ServerSocket::open(" << m_iListenPortNumber << "," << bTcpAnyHost << "): bind socket resp. listen failed.");
    closesocket(m_tListenSocket);
    m_tListenSocket = INVALID_SOCKET;
  }

  ROS_INFO_STREAM("ServerSocket: listening to port " << m_iListenPortNumber);
  return bOkay;
}

/*!
 * Waits for a client to connect, creates a socket to read and write.
 * @return true on success, false on failure
 */
bool sick_scan_xd::ServerSocket::connect()
{
  // Socket okay?
  if (m_tListenSocket == INVALID_SOCKET)
  {
    ROS_ERROR_STREAM("## ERROR ServerSocket::connect(port " << m_iListenPortNumber << "): socket not initialized, call open() before connect().");
    return false;
  }

  // Wait for and accept a new connection
# if UTL_TCP_SELECT_BEFORE_ACCEPT
  {
    fd_set readfds;
    FD_ZERO( &readfds );
    FD_SET(m_tListenSocket, &readfds);
    if (select(0, &readfds, NULL, NULL, NULL) <= 0 || FD_ISSET(m_tListenSocket, &readfds) == 0)
    {
      ROS_ERROR_STREAM("## ERROR ServerSocket::connect(port " << m_iListenPortNumber << "): select failed.");
      return false;
    }
  }
# endif // TCP_SELECT_BEFORE_ACCEPT

  m_tConnectedSocket = accept(m_tListenSocket, NULL, NULL);
  if (m_tConnectedSocket == INVALID_SOCKET)
  {
    ROS_ERROR_STREAM("## ERROR ServerSocket::connect(port " << m_iListenPortNumber << "): accept failed.");
    return false;
  }

  ROS_INFO_STREAM("ServerSocket (port " << m_iListenPortNumber << "): connected to client.");
  return true;
}

/*!
 * Reads bytes from the socket.
 * @return number of bytes read, or -1 on error (invalid socket, broken connection)
 */
int sick_scan_xd::ServerSocket::read(int num_bytes, std::vector<uint8_t>& out_buffer, bool read_blocking)
{
  if( m_tListenSocket == INVALID_SOCKET || m_tConnectedSocket == INVALID_SOCKET )
  {
    ROS_ERROR_STREAM("## ERROR ServerSocket::read(port " << m_iListenPortNumber << "): socket not connected");
    return -1;
  }
  if( num_bytes <= 0 )
  {
    return 0;
  }

# if TCP_SELECT_BEFORE_READ
  {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(m_tConnectedSocket, &readfds);
    if (select(0, &readfds, NULL, NULL, NULL) <= 0 || FD_ISSET(m_tConnectedSocket, &readfds ) == 0)
    {
      ROS_ERROR_STREAM("## ERROR ServerSocket::read(port " << m_iListenPortNumber << "): select failed.");
      return -1;
    }
  }
# endif

  // set socket to nonblocking mode
  int recv_flags = 0;
# ifdef _MSC_VER
  u_long recv_mode = (read_blocking ? 0 : 1); // FIONBIO enables or disables the blocking mode for the socket. If iMode = 0, blocking is enabled, if iMode != 0, non-blocking mode is enabled.
  ioctlsocket(m_tConnectedSocket, FIONBIO, &recv_mode);
# else
  if(!read_blocking)
    recv_flags |= MSG_DONTWAIT;
# endif

  std::vector<uint8_t> buffer(num_bytes);
  int nrbytes = 0;
  while (ROS::ok() && nrbytes < num_bytes && m_tListenSocket != INVALID_SOCKET && m_tConnectedSocket != INVALID_SOCKET)
  {
    int n = ::recv(m_tConnectedSocket, (char*)(buffer.data() + nrbytes), num_bytes - nrbytes, recv_flags);
    if(n <= 0)
    {
      if(read_blocking)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      else
      {
        break;
      }
    }
    nrbytes += n;
  }
  out_buffer.insert(out_buffer.end(), buffer.begin(), buffer.end());

  if (read_blocking && nrbytes < num_bytes)
  {
    ROS_ERROR_STREAM("## ERROR ServerSocket::read(port " << m_iListenPortNumber << "): failed, " << nrbytes << " of " << num_bytes << " bytes read from socket.");
  }
  return nrbytes;
}

/*!
 * Read <num_bytes> bytes from the socket.
 * @return true on success, false on failure
 */
bool sick_scan_xd::ServerSocket::read(int num_bytes, uint8_t* out_buffer, bool read_blocking)
{
  std::vector<uint8_t> buffer;
  int nrbytes = read(num_bytes, buffer, read_blocking);
  if(nrbytes > 0)
    memcpy(out_buffer, buffer.data(), std::min(nrbytes, num_bytes));
  return nrbytes == num_bytes;
}

/*!
 * Writes bytes to the socket.
 * @return true on success, false on failure
 */
bool sick_scan_xd::ServerSocket::write(const uint8_t* buffer, int num_bytes, int num_retries_on_error)
{

  if (m_tConnectedSocket == INVALID_SOCKET)
  {
    ROS_ERROR_STREAM("## ERROR ServerSocket::write(port " << m_iListenPortNumber << "): socket not connected");
    return false;
  }
  if (buffer == NULL || num_bytes <= 0)
  {
    return false;
  }

  int nrbytes = 0, nrErrors = 0;
  num_retries_on_error = std::max(num_retries_on_error, 1);
  while (ROS::ok() && num_bytes > 0 && nrErrors < num_retries_on_error && m_tListenSocket != INVALID_SOCKET && m_tConnectedSocket != INVALID_SOCKET)
  {
#   if TCP_SELECT_BEFORE_WRITE
    fd_set writefds;
    FD_ZERO( &writefds );
    FD_SET( m_tConnectedSocket, &writefds );
    if (select( 0, NULL, &writefds, NULL, NULL) <= 0 || FD_ISSET( ptServerSocket->tConnectedSocket, &writefds ) == 0)
    {
      ROS_ERROR_STREAM("## ERROR ServerSocket::write(port " << m_iListenPortNumber << "): select failed.");
      return false;
    }
#   endif
    nrbytes = send(m_tConnectedSocket, (char*)buffer, num_bytes, 0);
    if( nrbytes > 0 )
    {
      num_bytes -= nrbytes;
    }
    else if(nrbytes < 0)
    {
      nrErrors++;
    }
  }
  if( num_bytes > 0 )
  {
    ROS_ERROR_STREAM("## ERROR ServerSocket::write(port " << m_iListenPortNumber << "): failed to send " << num_bytes << " bytes.");
    return false;
  }
  return true;
}

/*!
 * Closes the tcp connections.
 */
void sick_scan_xd::ServerSocket::close(void)
{
  if(m_tConnectedSocket != INVALID_SOCKET)
  {
    shutdown(m_tConnectedSocket,SD_BOTH);
    closesocket(m_tConnectedSocket);
    m_tConnectedSocket = INVALID_SOCKET;
  }
  if(m_tListenSocket != INVALID_SOCKET)
  {
    shutdown(m_tListenSocket,SD_BOTH);
    closesocket(m_tListenSocket);
    m_tListenSocket = INVALID_SOCKET;
  }
}

/*!
 * @return returns true, if the server socket is connected to a client, otherwise false
 */
bool sick_scan_xd::ServerSocket::is_open(void)
{
    return (m_tConnectedSocket != INVALID_SOCKET && m_tListenSocket != INVALID_SOCKET);
}
