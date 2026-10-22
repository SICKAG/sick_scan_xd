#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/**
 * @file udp_poster.h
 * @brief udp_poster implements udp posts to start and stop multiScan136.
 *
 * Copyright (C) 2020-2026, SICK AG, Waldkirch
 * Copyright (C) 2020-2026, Ing.-Buero Dr. Michael Lehning, Hildesheim
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @date 2020 (original), September 2026 (updated)
 * @author Michael Lehning <michael.lehning@lehning.de>
 */
#ifndef __SICK_SCANSEGMENT_XD_UDP_POSTER_H
#define __SICK_SCANSEGMENT_XD_UDP_POSTER_H

#include "sick_scan/sick_ros_wrapper.h"
#include "sick_scansegment_xd/common.h"

namespace sick_scansegment_xd
{
    /*
     * @brief forward declaration of an udp sender socket implementation.
     * Used internally in the UdpPoster.
     */
    class UdpSenderSocketImpl;

    /*
     * @brief forward declaration of an udp receiver socket implementation.
     * Used internally in the UdpPoster.
     */
    class UdpReceiverSocketImpl;

    /*
     * @brief class UdpReceiver receives msgpack raw data by udp.
     * It implements a udp client, connects to multiScan136 (or any other udp-) sender,
     * receives and buffers msgpack raw data.
     */
    class UdpPoster
    {
    public:

        /*
         * @brief Default constructor.
         * @param[in] ip ip address of a multiScan136, f.e. "127.0.0.1" (localhost, loopback) for an emulator or "192.168.0.1" for multiScan136
         * @param[in] udp_port ip port, f.e. 2115 (default port for multiScan136 emulator)
         */
        UdpPoster(const std::string& ip = "192.168.0.1", int udp_port = 2115);

        /*
         * @brief Default destructor.
         */
        ~UdpPoster();

        /*
         * @brief Returns the ip address to send udp messages.
         */
        const std::string& IP(void) const;

        /*
         * @brief Returns the port to send udp messages.
         */
        const int& Port(void) const;

        /*
         * @brief Posts a request message.
         * @param[in] request message to send
         * @param[in] response message received
         * @return true on success, otherwise false
         */
        bool Post(const std::string& request, std::string& response);

    private:

        /*
         * Member data to run a udp receiver
         */
        std::string m_ip; // ip address of a multiScan136 to to send upd messages.
        int m_port;       // udp port to send and receive
        UdpSenderSocketImpl* m_sender_impl;     // implementation of the udp sender socket
        UdpReceiverSocketImpl* m_receiver_impl; // implementation of the udp receiver socket

    };  // class UdpPoster

}   // namespace sick_scansegment_xd
#endif // __SICK_SCANSEGMENT_XD_UDP_RECEIVER_H
