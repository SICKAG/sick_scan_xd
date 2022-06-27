/*
 * @brief udp_poster implements udp posts to start and stop multiScan136.
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
#include "sick_scansegment_xd/udp_poster.h"
#include "sick_scansegment_xd/udp_sockets.h"

/*
 * @brief Default constructor.
 * @param[in] ip ip address of a multiScan136, f.e. "127.0.0.1" (localhost, loopback) for an emulator or "192.168.0.1" for multiScan136
 * @param[in] udp_port ip port, f.e. 2115 (default port for multiScan136 emulator)
 */
sick_scansegment_xd::UdpPoster::UdpPoster(const std::string& ip, int udp_port)
: m_ip(ip), m_port(udp_port), m_sender_impl(0), m_receiver_impl(0)
{
    m_sender_impl = new UdpSenderSocketImpl(m_ip, m_port);
    if (!m_sender_impl->IsOpen())
    {
        ROS_ERROR_STREAM("## ERROR UdpPoster::Init(): can't open socket, UdpSenderSocketImpl(" << m_ip << "," << m_port << ") failed.");
        delete m_sender_impl;
        m_sender_impl = 0;
    }
    m_receiver_impl = new UdpReceiverSocketImpl();
    if (!m_receiver_impl->Init(m_ip, m_port))
    {
        ROS_ERROR_STREAM("## ERROR UdpPoster::Init(): can't open socket, UdpReceiverSocketImpl::Init(" << m_ip << "," << m_port << ") failed.");
        delete m_receiver_impl;
        m_receiver_impl = 0;
    }
}

/*
 * @brief Default destructor.
 */
sick_scansegment_xd::UdpPoster::~UdpPoster()
{
    if (m_sender_impl)
    {
        delete m_sender_impl;
        m_sender_impl = 0;
    }
    if (m_receiver_impl)
    {
        delete m_receiver_impl;
        m_receiver_impl = 0;
    }
}

/*
 * @brief Returns the ip address to send udp messages.
 */
const std::string& sick_scansegment_xd::UdpPoster::IP(void) const
{
	return m_ip;
}

/*
 * @brief Returns the port to send udp messages.
 */
const int& sick_scansegment_xd::UdpPoster::Port(void) const
{
	return m_port;
}

/*
 * @brief Posts a request message.
 * @param[in] request message to send
 * @param[in] response message received
 * @return true on success, otherwise false
 */
bool sick_scansegment_xd::UdpPoster::Post(const std::string& request, std::string& response)
{
    if (!m_sender_impl)
    {
        ROS_ERROR_STREAM("## ERROR UdpPoster::Post(): udp sender socket not initialized");
        return false;
    }
    std::vector<uint8_t> request_data(request.begin(), request.end());
    if (!m_sender_impl->Send(request_data))
    {
        ROS_ERROR_STREAM("## ERROR UdpPoster::Post(): failed to send " << request_data.size() << " byte message \"" << request << "\"");
        return false;
    }
    if (!m_receiver_impl)
    {
        ROS_ERROR_STREAM("## ERROR UdpPoster::Post(): udp receiver socket not initialized");
        return false;
    }
    std::vector<uint8_t> response_data(1024, 0);
    size_t byte_received = m_receiver_impl->Receive(response_data);
    response_data.resize(byte_received);
    response = std::string(response_data.begin(), response_data.end());
    return true; // todo
}
