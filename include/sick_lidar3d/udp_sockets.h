/*
 * @brief udp_sockets implement udp sender and receiver using boost::asio.
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
#ifndef __SICK_LIDAR3D_UDP_SOCKETS_H
#define __SICK_LIDAR3D_UDP_SOCKETS_H
 
/** Defines required for boost */
#ifdef __MSVC_RUNTIME_CHECKS
#define __RESTORE_MSVC_RUNTIME_CHECKS __MSVC_RUNTIME_CHECKS
#undef __MSVC_RUNTIME_CHECKS // suppress for boost runtime
#endif
#ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
#define __RESTORE_WINSOCK_DEPRECATED_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS // suppress boost/winsock warnings about deprecated API
#endif
#ifndef _WIN32_WINNT
#define _WIN32_WINNT _WIN32_WINNT_WIN10
#endif
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/exception/diagnostic_information.hpp>
#ifdef __RESTORE_MSVC_RUNTIME_CHECKS
#define __MSVC_RUNTIME_CHECKS __RESTORE_MSVC_RUNTIME_CHECKS
#endif
#ifdef __RESTORE_WINSOCK_DEPRECATED_WARNINGS
#undef _WINSOCK_DEPRECATED_NO_WARNINGS
#endif

#include "sick_lidar3d/common.h"

namespace sick_lidar3d
{
    /*
     * Shortcut to convert 4 byte to uint32t assuming little endian.
     */
    static uint32_t Convert4Byte(const uint8_t* p_data)
    {
        return (p_data[0]) | (p_data[1] << 8) | (p_data[2] << 16) | (p_data[3] << 24);
    }

    /*
     * @brief the udp socket to receive udp data, uses boost::asio::ip::udp
     */
    class UdpReceiverSocketImpl
    {
    public:

        /** Default constructor */
        UdpReceiverSocketImpl() : m_udp_sender(""), m_udp_port(0), m_io_service(), m_socket(m_io_service)
        {
        }

        /** Destructor, closes the socket */
        ~UdpReceiverSocketImpl()
        {
            try
            {
              if (m_socket.is_open())
              {
                m_socket.shutdown(m_socket.shutdown_both);
                m_socket.close();
              }
            }
            catch (boost::exception & e)
            {
                LIDAR3D_ERROR_STREAM("## ERROR ~UdpReceiverSocketImpl::Init(): can't close socket, " << boost::diagnostic_information(e));
            }
            catch (std::exception & e)
            {
                LIDAR3D_ERROR_STREAM("## ERROR ~UdpReceiverSocketImpl::Init(): can't open socket, " << e.what());
            }
        }

        /** Opens a udp socket */
        bool Init(const std::string& udp_sender, int udp_port)
        {
            try
            {
                m_udp_sender = udp_sender;
                m_udp_port = udp_port;
                m_socket.open(boost::asio::ip::udp::v4());
                if (!m_socket.is_open())
                {
                    LIDAR3D_ERROR_STREAM("## ERROR UdpReceiverSocketImpl::Init(" << m_udp_sender << ":" << m_udp_port << "): can't open socket.");
                    return false;
                }
                m_socket.set_option(boost::asio::socket_base::reuse_address(true));
                // m_socket.set_option(boost::asio::socket_base::broadcast(true));
                if (m_udp_sender.empty())
                    m_socket.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), m_udp_port)); // receive from any upd sender on port <m_udp_port>
                else
                    m_socket.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(m_udp_sender), m_udp_port)); // receive from <m_udp_sender> on port <m_udp_port>
                return true;
            }
            catch (boost::exception & e)
            {
                LIDAR3D_ERROR_STREAM("## ERROR UdpReceiverSocketImpl::Init(): can't open socket to " << m_udp_sender << ":" << m_udp_port << ", " << boost::diagnostic_information(e));
                return false;
            }
            catch (std::exception & e)
            {
                LIDAR3D_ERROR_STREAM("## ERROR UdpReceiverSocketImpl::Init(): can't open socket to " << m_udp_sender << ":" << m_udp_port << ", " << e.what());
                return false;
            }
        }

        /** Reads blocking until some data has been received successfully or an error occurs. Returns the number of bytes received. */
        size_t Receive(std::vector<uint8_t>& msg_payload)
        {
            size_t bytes_received = m_socket.receive_from(boost::asio::buffer(msg_payload), m_sender);
            return bytes_received;
        }

        /** Reads blocking until all bytes of a msgpack incl. header and crc have been received or an error occurs. Returns the number of bytes received. */
        size_t Receive(std::vector<uint8_t>& msg_payload, double timeout, const std::vector<uint8_t>& udp_msg_start_seq)
        {
            chrono_system_time start_timestamp = chrono_system_clock::now();
            size_t headerlength = udp_msg_start_seq.size() + sizeof(uint32_t); // 8 byte header: 0x02020202 + Payloadlength
            size_t bytes_received = 0;
            size_t bytes_to_receive = msg_payload.size();
            // Receive \x02\x02\x02\x02 | 4Bytes payloadlength incl. CRC | Payload | CRC32
            while (bytes_received < bytes_to_receive && (timeout < 0 || sick_lidar3d::Seconds(start_timestamp, chrono_system_clock::now()) < timeout))
            {
                size_t chunk_bytes_received = m_socket.receive_from(boost::asio::buffer(msg_payload.data() + bytes_received, bytes_to_receive - bytes_received), m_sender);
                if (chunk_bytes_received == 0)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                // std::cout << "UdpSenderSocketImpl::Receive(): chunk of " << std::dec << chunk_bytes_received << " bytes received: " << std::endl;
                // for(int n = 0; n < chunk_bytes_received; n++)
                //     std::cout << std::setfill('0') << std::setw(2) << std::hex << (int)(msg_payload.data()[bytes_received + n] & 0xFF);
                // std::cout << std::endl;
                if (bytes_received == 0 && chunk_bytes_received > headerlength && std::equal(msg_payload.begin(), msg_payload.begin() + udp_msg_start_seq.size(), udp_msg_start_seq.begin())) // start of new msgpack
                {
                    // Decode 8 byte header: 0x02020202 + Payloadlength
                    size_t Payloadlength= Convert4Byte(msg_payload.data() + udp_msg_start_seq.size());
                    bytes_to_receive = Payloadlength + headerlength + sizeof(uint32_t); // 8 byte header + payload + 4 byte CRC
                    if(bytes_to_receive > msg_payload.size())
                    {
                        LIDAR3D_ERROR_STREAM("## ERROR UdpReceiverSocketImpl::Receive(): unexpected payloadlength " << Payloadlength << " byte incl CRC received");
                        break;
                    }
                    bytes_received += chunk_bytes_received;
                }
                else if (bytes_received > 0) // continue receiving a msgpack
                {
                    bytes_received += chunk_bytes_received;
                }
            }
            return bytes_received;
        }

        /** Returns senders ip address */
        std::string SenderIP(void)
        {
            return m_sender.address().to_string();
        }

    protected:

        std::string m_udp_sender;
        int m_udp_port;
        boost::asio::io_service m_io_service;
        boost::asio::ip::udp::socket m_socket;
        boost::asio::ip::udp::endpoint m_sender;
    };

    /*!
     * @brief class UdpSenderSocketImpl implements the udp socket for sending udp packages, uses boost::asio::ip::udp
     */
    class UdpSenderSocketImpl
    {
    public:

        /*!
         * Constructor, opens an udp socket.
         * @param[in] server_address ip address of the localization controller, default: 192.168.0.1
         * @param[in] udp_port udp port, default: 2115
         */
        UdpSenderSocketImpl(const std::string& server_address = "192.168.0.1", int udp_port = 2115)
            : m_io_service(), m_udp_socket(m_io_service), m_socket_opened(false)
        {
            try
            {
                m_udp_socket.open(boost::asio::ip::udp::v4());
                m_socket_opened = m_udp_socket.is_open();
                if (!m_socket_opened)
                {
                    LIDAR3D_ERROR_STREAM("## ERROR UdpSenderSocketImpl(" << server_address << ":" << udp_port << "): can't open udp socket.");
                }
                m_udp_socket.set_option(boost::asio::socket_base::reuse_address(true));
                m_udp_socket.set_option(boost::asio::socket_base::broadcast(true));
                m_udp_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(server_address), udp_port);
            }
            catch (boost::exception & e)
            {
                LIDAR3D_ERROR_STREAM("## ERROR UdpSenderSocketImpl::Init(): can't open socket to " << server_address << ":" << udp_port << ", " << boost::diagnostic_information(e));
            }
            catch (const std::exception& e)
            {
                m_socket_opened = false;
                LIDAR3D_ERROR_STREAM("## ERROR UdpSenderSocketImpl(): socket initialization failed, exception " << e.what());
            }
        }

        /*!
         * Destructor, closes the socket
         */
        ~UdpSenderSocketImpl()
        {
          try
          {
            if (m_socket_opened)
            {
                // m_udp_socket.shutdown(m_udp_socket.shutdown_both);
                m_udp_socket.close();
                m_socket_opened = false;
            }
          }
          catch (boost::exception & e)
          {
              LIDAR3D_ERROR_STREAM("## ERROR UdpSenderSocketImpl::Init(): can't close socket, " << boost::diagnostic_information(e));
          }
          catch (const std::exception& e)
          {
              LIDAR3D_ERROR_STREAM("## ERROR ~UdpSenderSocketImpl(): socket shutdown and close failed, exception " << e.what());
          }
          m_socket_opened = false;
        }

        /*!
         * Returns true if the udp socket is opened and ready to send, or false otherwise.
         */
        bool IsOpen(void) const { return m_socket_opened; }

        /*!
         * Sends a binary message.
         */
        bool Send(std::vector<uint8_t>& message)
        {
            size_t bytes_sent = 0;
            if (m_socket_opened)
            {
                try
                {
                    bytes_sent = m_udp_socket.send_to(boost::asio::buffer(message.data(), message.size()), m_udp_endpoint);
                    if (bytes_sent != message.size())
                    {
                        LIDAR3D_ERROR_STREAM("## ERROR UdpSenderSocketImpl()::Send(): " << bytes_sent << " of " << message.size() << " bytes sent, boost::asio::ip::udp::socket::send_to() failed.");
                    }
                }
                catch (boost::exception & e)
                {
                    LIDAR3D_ERROR_STREAM("## ERROR UdpSenderSocketImpl::Send(): socket error, " << boost::diagnostic_information(e));
                }
                catch (const std::exception& e)
                {
                    LIDAR3D_ERROR_STREAM("## ERROR UdpSenderSocketImpl()::Send(): boost::asio::ip::udp::socket::send_to() failed, exception " << e.what());
                }
            }
            else
            {
                LIDAR3D_ERROR_STREAM("## ERROR UdpSenderSocketImpl()::Send(): udp socket not initialized");
            }
            return (bytes_sent == message.size());
        }

    protected:

        bool m_socket_opened; ///< true if the udp socket is opened and ready to send, or false otherwise
        boost::asio::io_service m_io_service; ///< boost io for udp socket
        boost::asio::ip::udp::socket m_udp_socket; ///< udp socket for binary odom telegrams
        boost::asio::ip::udp::endpoint m_udp_endpoint; ///< udp receiver (i.e. the localizaton server)
    };
}   // namespace sick_lidar3d
#endif // __SICK_LIDAR3D_UDP_SOCKETS_H
