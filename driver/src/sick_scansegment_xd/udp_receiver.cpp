/*
 * @brief udp_receiver receives msgpack raw data by udp.
 * It implements a udp client, connects to multiScan136 (or any other udp-) sender,
 * receives and buffers msgpack raw data.
 *
 * Copyright (C) 2020,2021 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2020,2021 SICK AG, Waldkirch
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

#include "sick_scansegment_xd/fifo.h"
#include "sick_scansegment_xd/udp_receiver.h"
#include "sick_scansegment_xd/udp_sockets.h"

/*
 * Computes the zlib CRC32 checksum, see https://stackoverflow.com/questions/15030011/same-crc32-for-python-and-c
 */
static uint32_t crc32(uint32_t crc, const uint8_t* buf, size_t len)
{
    crc = ~crc;
    while (len--)
    {
        crc ^= *buf++;
        for (int k = 0; k < 8; k++)
            crc = crc & 1 ? (crc >> 1) ^ 0xedb88320 : crc >> 1;
    }
    return ~crc;
}



/*
 * @brief Default constructor.
 */
sick_scansegment_xd::UdpReceiver::UdpReceiver() : m_verbose(false), m_export_udp_msg(false), m_socket_impl(0), m_fifo_impl(0), m_receiver_thread(0), m_run_receiver_thread(false),
    m_udp_recv_buffer_size(0), m_udp_timeout_recv_nonblocking(0), m_udp_sender_timeout(0)
{
}

/*
 * @brief Default destructor.
 */
sick_scansegment_xd::UdpReceiver::~UdpReceiver()
{
    Close();
}

/*
 * @brief Initializes an udp socket to a sender.
 * @param[in] udp_sender ip address of the udp sender, f.e. "127.0.0.1" (localhost, loopback)
 * @param[in] udp_port ip port, f.e. 2115 (default port for multiScan136 emulator)
 * @param[in] udp_input_fifolength max. input fifo length (-1: unlimited, default: 20 for buffering 1 second at 20 Hz), elements will be removed from front if number of elements exceeds the fifo_length
 * @param[in] verbose true: enable debug output, false: quiet mode (default)
 * @param[in] export_udp_msg: true: export binary udp and msgpack data to file (*.udp and *.msg), default: false
 */
bool sick_scansegment_xd::UdpReceiver::Init(const std::string& udp_sender, int udp_port, int udp_input_fifolength, bool verbose, bool export_udp_msg)
{
    if (m_socket_impl || m_fifo_impl || m_receiver_thread)
        Close();

    // todo: move to config // todo: Testen: Synchronisierung auf Anfang der msgpack-Nachrichten

    // Receive \x02\x02\x02\x02 | 4Bytes Laenge des Payloads inkl. CRC | Payload | CRC32
    m_udp_recv_buffer_size = 64 * 1024; // size of buffer to receive udp packages
    m_udp_msg_start_seq = { 0x02, 0x02,  0x02,  0x02 }; // { 0xDC, 0x00, 0x10, 0x82, 0xA5, 'c', 'l', 'a', 's', 's', 0xA4, 'S', 'c', 'a', 'n' }; // any udp message from multiScan136 starts with 15 byte ".....class.Scan"
    m_udp_timeout_recv_nonblocking = 1.0; // in normal mode we receive udp datagrams non-blocking with timeout to enable sync with msgpack start
    m_udp_sender_timeout = 2.0;           // if no udp packages received within 2 second, we switch to blocking udp receive
    m_verbose = verbose;
    m_export_udp_msg = export_udp_msg;    // true : export binary udpand msgpack data to file(*.udp and* .msg), default: false
    m_fifo_impl = new PayloadFifo(udp_input_fifolength);
    m_socket_impl = new UdpReceiverSocketImpl();
    if (!m_socket_impl->Init(udp_sender, udp_port))
    {
        ROS_ERROR_STREAM("## ERROR UdpReceiver::Init(): UdpReceiverSocketImpl::Init(" << udp_sender << "," << udp_port << ") failed.");
        return false;
    }

    return true;
}

/*
 * @brief Starts receiving udp packages in a background thread and pops msgpack data packages to the fifo.
 */
bool sick_scansegment_xd::UdpReceiver::Start(void)
{
    m_run_receiver_thread = true;
    m_receiver_thread = new std::thread(&sick_scansegment_xd::UdpReceiver::Run, this);
    return true;
}

/*
 * @brief Stop to receive data and shutdown the udp socket
 */
void sick_scansegment_xd::UdpReceiver::Close(void)
{
    m_run_receiver_thread = false;
    if (m_fifo_impl)
    {
        m_fifo_impl->Shutdown();
    }
    if (m_receiver_thread)
    {
        m_receiver_thread->join();
        delete m_receiver_thread;
        m_receiver_thread = 0;
    }
    if (m_socket_impl)
    {
        delete m_socket_impl;
        m_socket_impl = 0;
    }
    if (m_fifo_impl)
    {
        delete m_fifo_impl;
        m_fifo_impl = 0;
    }
}

/*
 * @brief Thread callback, runs the receiver for udp packages and pops msgpack data packages to the fifo.
 */
bool sick_scansegment_xd::UdpReceiver::Run(void)
{
    if (!m_socket_impl)
    {
        ROS_ERROR_STREAM("## ERROR UdpReceiver::Run(): UdpReceiver not initialized, call UdpReceiver::Init() first.");
        return false;
    }
    try
    {
        size_t udp_recv_counter = 0;
        std::vector<uint8_t> udp_payload(m_udp_recv_buffer_size, 0);
        double udp_recv_timeout = -1; // initial timeout: block until first datagram received
        chrono_system_time timestamp_last_udp_recv = chrono_system_clock::now();
        while (m_run_receiver_thread)
        {
            size_t bytes_received = m_socket_impl->Receive(udp_payload, udp_recv_timeout, m_udp_msg_start_seq);
            // std::cout << "UdpReceiver::Run(): " << bytes_received << " bytes received" << std::endl;
            if(bytes_received > m_udp_msg_start_seq.size() + 8 && std::equal(udp_payload.begin(), udp_payload.begin() + m_udp_msg_start_seq.size(), m_udp_msg_start_seq.begin()))
            {
                // Received \x02\x02\x02\x02 | 4Bytes payload length | Payload | CRC32
                bool crc_error = false;
                // UDP message := (4 byte \x02\x02\x02\x02) + (4 byte payload length) + (4 byte CRC)
                uint32_t u32PayloadLength = Convert4Byte(udp_payload.data() + m_udp_msg_start_seq.size());
                uint32_t bytes_to_receive = (uint32_t)(u32PayloadLength + m_udp_msg_start_seq.size() + 2 * sizeof(uint32_t));
                if (bytes_received != bytes_to_receive)
                {
                    ROS_ERROR_STREAM("## ERROR UdpReceiver::Run(): " << bytes_received << " bytes received, " << bytes_to_receive << " bytes expected, u32PayloadLength=" << u32PayloadLength);
                }
                // std::cout << "UdpReceiver: u32PayloadLength = " << u32PayloadLength << " byte" << std::endl;
                // CRC check
                uint32_t u32PayloadCRC = Convert4Byte(udp_payload.data() + bytes_received - sizeof(uint32_t)); // last 4 bytes are CRC
                std::vector<uint8_t> msgpack_payload(udp_payload.begin() + m_udp_msg_start_seq.size() + sizeof(uint32_t), udp_payload.begin() + bytes_received - sizeof(uint32_t));
                uint32_t u32MsgPackCRC = crc32(0, msgpack_payload.data(), msgpack_payload.size());
                if (u32PayloadCRC != u32MsgPackCRC)
                {
                    crc_error = true;
                    ROS_ERROR_STREAM("## ERROR UdpReceiver::Run(): CRC 0x" << std::setfill('0') << std::setw(2) << std::hex << u32PayloadCRC
                        << " received from " << std::dec << bytes_received << " udp bytes different to CRC 0x"
                        << std::setfill('0') << std::setw(2) << std::hex << u32MsgPackCRC << " computed from "
                        << std::dec << (msgpack_payload.size()) << " byte payload, message dropped");
                    ROS_ERROR_STREAM("## ERROR UdpReceiver::Run(): decoded payload size: " << u32PayloadLength << " byte, bytes_to_receive (expected udp message length): "
                        << bytes_to_receive << " byte, bytes_received (received udp message length): " << bytes_received << " byte");
                    continue;
                }
                if (u32PayloadLength != msgpack_payload.size())
                {
                    ROS_ERROR_STREAM("## ERROR UdpReceiver::Run(): u32PayloadLength=" << u32PayloadLength << " different to decoded payload size " << msgpack_payload.size());
                }
                // Push msgpack_payload to input fifo
                if (!crc_error)
                {
                    size_t fifo_length = m_fifo_impl->Push(msgpack_payload, fifo_clock::now(), udp_recv_counter);
                    udp_recv_counter++;
                    if (m_verbose)
                    {
                        ROS_INFO_STREAM("UdpReceiver::Run(): " << bytes_received << " bytes received: " << ToPrintableString(udp_payload, bytes_received));
                        ROS_INFO_STREAM("UdpReceiver::Run(): " << fifo_length << " messages currently in paylod buffer, totally received " << udp_recv_counter << " udp packages");
                    }
                }
                if (m_export_udp_msg) // || crc_error
                {
                    std::ofstream udp_ostream(std::string("udp_received_bin_") + sick_scansegment_xd::FormatNumber(udp_recv_counter, 3, true, false, -1) + ".udp", std::ofstream::binary);
                    std::ofstream msg_ostream(std::string("udp_received_msg_") + sick_scansegment_xd::FormatNumber(udp_recv_counter, 3, true, false, -1) + ".msg", std::ofstream::binary);
                    if (udp_ostream.is_open() && msg_ostream.is_open())
                    {
                        udp_ostream.write((const char*)udp_payload.data(), bytes_received);
                        msg_ostream.write((const char*)msgpack_payload.data(), msgpack_payload.size());
                    }
                }
            }
            else if(bytes_received > 0)
            {
                ROS_ERROR_STREAM("## ERROR UdpReceiver::Run(): Received " << bytes_received << " unexpected bytes");
                if(m_verbose)
                    ROS_ERROR_STREAM(ToHexString(udp_payload, bytes_received));
            }
            if(bytes_received > 0)
                timestamp_last_udp_recv = chrono_system_clock::now();
            if (sick_scansegment_xd::Seconds(timestamp_last_udp_recv, chrono_system_clock::now()) > m_udp_sender_timeout) // if no udp packages received within 1-2 seconds, we switch to blocking udp receive
                udp_recv_timeout = -1;                             // udp timeout, last datagram more than 1 second ago, resync and block until next datagram received
            else                                                   // in normal mode we receive udp datagrams non-blocking with timeout to enable sync with msgpack start
                udp_recv_timeout = m_udp_timeout_recv_nonblocking; // receive non-blocking with timeout
        }
        m_run_receiver_thread = false;
        return true;
    }
    catch (std::exception & e)
    {
        ROS_ERROR_STREAM("## ERROR UdpReceiver::Run(): " << e.what());
    }
    m_run_receiver_thread = false;
    return false;
}

/*
 * @brief Converts a payload to a hex string
 * param[in] payload payload buffer
 * param[in] bytes_received number of received bytes
 */
std::string sick_scansegment_xd::UdpReceiver::ToHexString(const std::vector<uint8_t>& payload, size_t bytes_received)
{
    std::stringstream hexstream;
    for (size_t n = 0; n < bytes_received; n++)
    {
        hexstream << (n > 0 ? " " : "") << std::hex << (int)(payload[n] & 0xFF);
    }
    return hexstream.str();
}

/*
 * @brief Converts a payload to a printable string (alnum characters or '.' for non-printable bytes)
 * param[in] payload payload buffer
 * param[in] bytes_received number of received bytes
 */
std::string sick_scansegment_xd::UdpReceiver::ToPrintableString(const std::vector<uint8_t>& payload, size_t bytes_received)
{
    std::vector<uint8_t> payload_printable(bytes_received + 1);
    for (size_t n = 0; n < bytes_received; n++)
    {
        if (::isprint(payload[n]))
            payload_printable[n] = payload[n];
        else
            payload_printable[n] = '.';
    }
    payload_printable[bytes_received] = 0;
    return std::string((const char*)payload_printable.data());
}
