#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * @brief udp_sockets implement udp sender and receiver
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
#ifndef __SICK_SCANSEGMENT_XD_UDP_SOCKETS_H
#define __SICK_SCANSEGMENT_XD_UDP_SOCKETS_H

#include <string>
#if defined WIN32 || defined _MSC_VER
#ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif
#include <winsock2.h>
#define UNLINK _unlink
static std::string getErrorMessage(void)
{
    int error_num = WSAGetLastError();
    char error_message[1024] = { 0 };
    FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, error_num, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), error_message, sizeof(error_message), NULL);
    return std::to_string(error_num) + " (" + std::string(error_message) + ")";
}
#else
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <sys/types.h>
#include <sys/socket.h>
typedef int SOCKET;
typedef struct sockaddr SOCKADDR;
#define INVALID_SOCKET (-1)
#define UNLINK unlink
#define closesocket close
static std::string getErrorMessage(void) { return std::to_string(errno) + " (" + std::string(strerror(errno)) + ")"; }
#endif

#include "sick_scan/sick_ros_wrapper.h"
#include "sick_scansegment_xd/common.h"
#include "sick_scan/tcp/wsa_init.hpp"

namespace sick_scansegment_xd
{
    /*
     * Shortcut to convert 4 byte to uint32t assuming little endian.
     */
    static uint32_t Convert4Byte(const uint8_t* p_data)
    {
        return (p_data[0]) | (p_data[1] << 8) | (p_data[2] << 16) | (p_data[3] << 24);
    }

    /*
     * @brief the udp socket to receive udp data
     */
    class UdpReceiverSocketImpl
    {
    public:

        /** Default constructor */
        UdpReceiverSocketImpl() : m_udp_sender(""), m_udp_port(0), m_udp_socket(INVALID_SOCKET), m_running(false), m_recv_blocking(false), m_recv_flags(0)
        {
        }

        /** Destructor, closes the socket */
        ~UdpReceiverSocketImpl()
        {
            try
            {
                if (m_udp_socket != INVALID_SOCKET)
                {
                    // shutdown(m_udp_socket, SHUT_RDWR);
                    closesocket(m_udp_socket);
                    m_udp_socket = INVALID_SOCKET;
                }
            }
            catch (std::exception & e)
            {
                ROS_ERROR_STREAM("## ERROR ~UdpReceiverSocketImpl: can't close socket, " << e.what());
            }
        }

        /*
        ** @brief Opens a udp socket
        ** @param[in] udp_sender ip v4 address of sender (e.g. "192.168.0.100"), or "" for any
        ** @param[in] udp_port port number (e.g. 2115 for multiScan)
        ** @param[in] blocking option to recv blocking (true) or non-blocking (false, default)
        */
        bool Init(const std::string& udp_sender, int udp_port, bool blocking = false)
        {
            m_running = false;
            try
            {
                wsa_init();
                m_udp_sender = udp_sender;
                m_udp_port = udp_port;
                m_udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
                if (m_udp_socket == INVALID_SOCKET)
                {
                    ROS_ERROR_STREAM("## ERROR UdpReceiverSocketImpl::Init(" << m_udp_sender << ":" << m_udp_port << "): can't open socket, error: " << getErrorMessage());
                    return false;
                }
                // #if defined WIN32 || defined _MSC_VER
                // char broadcast_opt = 1, reuse_addr_opt = 1, reuse_port_opt = 1;
                // #else
                // int broadcast_opt = 1, reuse_addr_opt = 1, reuse_port_opt = 1;
                // #endif
                // setsockopt(m_udp_socket, SOL_SOCKET, SO_BROADCAST, &broadcast_opt, sizeof(broadcast_opt));
                // setsockopt(m_udp_socket, SOL_SOCKET, SO_REUSEADDR, &reuse_addr_opt, sizeof(reuse_addr_opt));
                // setsockopt(m_udp_socket, SOL_SOCKET, SO_REUSEPORT, &reuse_port_opt, sizeof(reuse_port_opt));
                struct sockaddr_in sim_servaddr = { 0 };
                if(m_udp_sender.empty())
                    sim_servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
                else
                    sim_servaddr.sin_addr.s_addr = inet_addr(m_udp_sender.c_str()); 
                sim_servaddr.sin_family = AF_INET;
                sim_servaddr.sin_port = htons(m_udp_port);
                ROS_INFO_STREAM("UdpReceiverSocketImpl: udp socket created, binding to port " << ntohs(sim_servaddr.sin_port) << " ... ");
                if (bind(m_udp_socket, (SOCKADDR*)&sim_servaddr, sizeof(sim_servaddr)) < 0)
                {
                    ROS_ERROR_STREAM("## ERROR UdpReceiverSocketImpl::Init(" << m_udp_sender << ":" << m_udp_port << "): can't bind socket, error: " << getErrorMessage());
                    closesocket(m_udp_socket);
                    m_udp_socket = INVALID_SOCKET;
                    return false;
                }
                // Set socket to blocking or non-blocking recv mode
                m_recv_blocking = blocking;
#               ifdef _MSC_VER
                u_long recv_mode = (m_recv_blocking ? 0 : 1); // FIONBIO enables or disables the blocking mode for the socket. If iMode = 0, blocking is enabled, if iMode != 0, non-blocking mode is enabled.
                ioctlsocket(m_udp_socket, FIONBIO, &recv_mode);
#               else
                if (!m_recv_blocking)
                  m_recv_flags |= MSG_DONTWAIT;
#               endif
                m_running = true;
                return true;
            }
            catch (std::exception & e)
            {
                m_udp_socket = INVALID_SOCKET;
                ROS_ERROR_STREAM("## ERROR UdpReceiverSocketImpl::Init(): can't open socket to " << m_udp_sender << ":" << m_udp_port << ", exception: " << e.what());
                return false;
            }
        }

        /** Reads blocking until some data has been received successfully or an error occurs. Returns the number of bytes received. */
        size_t Receive(std::vector<uint8_t>& msg_payload)
        {
            int64_t bytes_received = 0;
            while (m_running && bytes_received <= 0)
            {
              bytes_received = recv(m_udp_socket, (char*)msg_payload.data(), (int)msg_payload.size(), m_recv_flags);
              if (m_recv_blocking && bytes_received < 0)
                return 0; // socket error
            }
            if (!m_running || bytes_received < 0)
                return 0; // socket error
            return (size_t)bytes_received;
        }

        /** Reads blocking until all bytes of a msgpack incl. header and crc have been received or an error occurs. Returns the number of bytes received. */
        size_t Receive(std::vector<uint8_t>& msg_payload, double timeout, const std::vector<uint8_t>& udp_msg_start_seq)
        {
            chrono_system_time start_timestamp = chrono_system_clock::now();
            size_t headerlength = udp_msg_start_seq.size() + sizeof(uint32_t); // 8 byte header: 0x02020202 + Payloadlength
            size_t bytes_received = 0;
            size_t bytes_to_receive = msg_payload.size();
            // Receive \x02\x02\x02\x02 | 4Bytes payloadlength incl. CRC | Payload | CRC32
            while (m_running && bytes_received < bytes_to_receive && (timeout < 0 || sick_scansegment_xd::Seconds(start_timestamp, chrono_system_clock::now()) < timeout))
            {
                int64_t chunk_bytes_received = recv(m_udp_socket, (char*)msg_payload.data() + bytes_received, (int)msg_payload.size() - bytes_received, m_recv_flags);
                if (chunk_bytes_received <= 0)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                // std::cout << "UdpSenderSocketImpl::Receive(): chunk of " << std::dec << chunk_bytes_received << " bytes received: " << std::endl;
                // for(int n = 0; n < chunk_bytes_received; n++)
                //     std::cout << std::setfill('0') << std::setw(2) << std::hex << (int)(msg_payload.data()[bytes_received + n] & 0xFF);
                // std::cout << std::endl;
                if (bytes_received == 0 && chunk_bytes_received > (int64_t)headerlength && std::equal(msg_payload.begin(), msg_payload.begin() + udp_msg_start_seq.size(), udp_msg_start_seq.begin())) // start of new msgpack
                {
                    // Start of new message: restart timeout
                    start_timestamp = chrono_system_clock::now();
                    // Decode 8 byte header: 0x02020202 + Payloadlength
                    size_t Payloadlength= Convert4Byte(msg_payload.data() + udp_msg_start_seq.size());
                    bytes_to_receive = Payloadlength + headerlength + sizeof(uint32_t); // 8 byte header + payload + 4 byte CRC
                    if(bytes_to_receive > msg_payload.size())
                    {
                        ROS_ERROR_STREAM("## ERROR UdpReceiverSocketImpl::Receive(): unexpected payloadlength " << Payloadlength << " byte incl CRC received");
                        break;
                    }
                    bytes_received += chunk_bytes_received;
                }
                else if (bytes_received > 0) // continue receiving a msgpack
                {
                    bytes_received += chunk_bytes_received;
                }
            }
            return (m_running ? bytes_received : 0);
        }

        /** Return the udp port */
        int port(void) const { return m_udp_port; }

        /** Return true, if socket is ready to receive, or false otherwise. Set false to signal stop receiving */
        bool& running(void) { return m_running; }

    protected:

        std::string m_udp_sender; // IP of udp sender
        int m_udp_port;           // udp port
        SOCKET m_udp_socket;      // udp raw socket
        bool m_running;           // true: ready to receive, false: not initialized / stop receiving
        bool m_recv_blocking;     // option to recv blocking (true) or non-blocking (false)
        int m_recv_flags;         // flag for socket recv, 0 (default) or MSG_DONTWAIT for non-blocking operation
    };

    /*!
     * @brief class UdpSenderSocketImpl implements the udp socket for sending udp packages
     */
    class UdpSenderSocketImpl
    {
    public:

        /*!
         * Constructor, opens an udp socket.
         * @param[in] server_address ip address
         * @param[in] udp_port udp port
         */
        UdpSenderSocketImpl(const std::string& server_address = "192.168.0.1", int udp_port = 2115)
            : m_socket_opened(false), m_udp_socket(INVALID_SOCKET) 
        {
            try
            {
                m_server_address = server_address;
                m_udp_port = udp_port;
                if ((m_udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == INVALID_SOCKET)
                {
                    ROS_ERROR_STREAM("## ERROR UdpSenderSocketImpl::init(" << server_address << ":" << udp_port << "): can't create socket, error: " << getErrorMessage());
                }
                else
                {
                    #if defined WIN32 || defined _MSC_VER
                    char broadcast_opt = 1; // reuse_addr_opt = 1
                    #else
                    int broadcast_opt = 1; // reuse_addr_opt = 1
                    #endif
                    if (setsockopt(m_udp_socket, SOL_SOCKET, SO_BROADCAST, &broadcast_opt, sizeof(broadcast_opt)) < 0)
                    {
                        ROS_ERROR_STREAM("## ERROR UdpSenderSocketImpl::init(" << server_address << ":" << udp_port << "): setsockopt(SO_BROADCAST) failed, error: " << getErrorMessage());
                    }
                    // setsockopt(m_udp_socket, SOL_SOCKET, SO_REUSEADDR, &reuse_addr_opt, sizeof(reuse_addr_opt));
                }
            }
            catch (const std::exception& e)
            {
                m_udp_socket = INVALID_SOCKET;
                ROS_ERROR_STREAM("## ERROR UdpSenderSocketImpl(): socket initialization failed, exception: " << e.what());
            }
        }

        /*!
         * Destructor, closes the socket
         */
        ~UdpSenderSocketImpl()
        {
          try
          {
            if (m_udp_socket != INVALID_SOCKET)
            {
                // shutdown(m_udp_socket, SHUT_RDWR);
                closesocket(m_udp_socket);
                m_udp_socket = INVALID_SOCKET;
            }
          }
          catch (const std::exception& e)
          {
              ROS_ERROR_STREAM("## ERROR ~UdpSenderSocketImpl(): socket shutdown and close failed, exception: " << e.what());
          }
        }

        /*!
         * Returns true if the udp socket is opened and ready to send, or false otherwise.
         */
        bool IsOpen(void) const { return (m_udp_socket != INVALID_SOCKET); }

        /*!
         * Sends a binary message.
         */
        bool Send(std::vector<uint8_t>& message)
        {
            size_t bytes_sent = 0;
            if (m_udp_socket != INVALID_SOCKET)
            {
                try
                {
                    struct sockaddr_in sim_servaddr = { 0 };
                    if(m_server_address.empty())
                    {
                        sim_servaddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
                    }
                    else
                    {
                        #if defined WIN32 || defined _MSC_VER
                        sim_servaddr.sin_addr.s_addr = inet_addr(m_server_address.c_str());
                        #else
                        struct in_addr sim_in_addr;
                        if (inet_aton(m_server_address.c_str(), &sim_in_addr) != 0)
                        {
                            sim_servaddr.sin_addr.s_addr = sim_in_addr.s_addr;
                        }
                        else
                        {
                            ROS_ERROR_STREAM("## ERROR UdpSenderSocketImpl()::Send(): inet_aton(" << m_server_address << ") failed (invalid address)");
                            sim_servaddr.sin_addr.s_addr = inet_addr(m_server_address.c_str());
                        }
                        #endif
                    }
                    sim_servaddr.sin_family = AF_INET;
                    sim_servaddr.sin_port = htons(m_udp_port);
                    bytes_sent = sendto(m_udp_socket, (const char*)message.data(), message.size(), 0, (SOCKADDR*)&sim_servaddr, sizeof(sim_servaddr));
                    if (bytes_sent != message.size())
                    {
                        ROS_ERROR_STREAM("## ERROR UdpSenderSocketImpl()::Send() failed, " << bytes_sent << " of " << message.size() << " bytes sent.");
                    }
                }
                catch (const std::exception& e)
                {
                    ROS_ERROR_STREAM("## ERROR UdpSenderSocketImpl()::Send() failed, exception: " << e.what());
                }
            }
            else
            {
                ROS_ERROR_STREAM("## ERROR UdpSenderSocketImpl()::Send(): udp socket not initialized");
            }
            return (bytes_sent == message.size());
        }

    protected:

        bool m_socket_opened;         // true if the udp socket is opened and ready to send, or false otherwise
        std::string m_server_address; // ip address
        int m_udp_port;               // udp port
        SOCKET m_udp_socket;          // udp raw socket
    };
}   // namespace sick_scansegment_xd
#endif // __SICK_SCANSEGMENT_XD_UDP_SOCKETS_H
