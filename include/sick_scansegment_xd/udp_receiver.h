#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * @brief udp_receiver receives msgpack raw data by udp.
 * It implements a udp client, connects to multiScan136 (or any other udp-) sender,
 * receives and buffers msgpack raw data.
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
#ifndef __SICK_SCANSEGMENT_XD_UDP_RECEIVER_H
#define __SICK_SCANSEGMENT_XD_UDP_RECEIVER_H

#include "sick_scan/sick_ros_wrapper.h"
#include "sick_scansegment_xd/common.h"
#include "sick_scansegment_xd/fifo.h"

namespace sick_scansegment_xd
{
    /*
     * @brief forward declaration of an udp receiver socket implementation.
     * Used internally in the UdpReceiver.
     */
    class UdpReceiverSocketImpl;

    /*
     * @brief class UdpReceiver receives msgpack raw data by udp.
     * It implements a udp client, connects to multiScan136 (or any other udp-) sender,
     * receives and buffers msgpack raw data.
     */
    class UdpReceiver
    {
    public:

        /*
         * @brief Default constructor.
         */
        UdpReceiver();

        /*
         * @brief Default destructor.
         */
        ~UdpReceiver();

        /*
         * @brief Initializes an udp socket to a sender.
         * @param[in] udp_sender ip address of the udp sender, f.e. "127.0.0.1" (localhost, loopback)
         * @param[in] udp_port ip port, f.e. 2115 (default port for multiScan136 emulator)
         * @param[in] udp_input_fifolength max. input fifo length (-1: unlimited, default: 20 for buffering 1 second at 20 Hz), elements will be removed from front if number of elements exceeds the fifo_length
         * @param[in] verbose true: enable debug output, false: quiet mode (default)
         * @param[in] export_udp_msg: true: export binary udp and msgpack data to file (*.udp and *.msg), default: false
         * @param[in] scandataformat ScanDataFormat: 1 for msgpack or 2 for compact scandata, default: 1
         * @param[in] PayloadFifo* fifo: Fifo to handle payload data
         */
        bool Init(const std::string& udp_sender, int udp_port, int udp_input_fifolength = 20, bool verbose = false, bool export_udp_msg = false, int scandataformat = 1, PayloadFifo* fifo = 0);

        /*
         * @brief Starts receiving udp packages in a background thread and pops msgpack data packages to the fifo.
         */
        bool Start(void);

        /*
         * @brief Stops the udp receiver thread
         */
        void Stop(bool do_join = true);

        /*
         * @brief Stop to receive data and shutdown the udp socket
         */
        void Close(void);

        /*
         * @brief Returns the Fifo storing the msgpack data received by this udp receiver.
         */
        PayloadFifo* Fifo(void) { return m_fifo_impl; }

        /*
         * @brief Converts a payload to a hex string
         * param[in] payload payload buffer
         * param[in] bytes_received number of received bytes
         */
        static std::string ToHexString(const std::vector<uint8_t>& payload, size_t bytes_received);

        /*
         * @brief Converts a payload to a printable string (alnum characters or '.' for non-printable bytes)
         * param[in] payload payload buffer
         * param[in] bytes_received number of received bytes
         */
        static std::string ToPrintableString(const std::vector<uint8_t>& payload, size_t bytes_received);

    private:

        /*
         * @brief Thread callback, runs the receiver for udp packages and pops msgpack data packages to the fifo.
         */
        bool Run(void);

        /*
         * Configuration and parameter
         */
        bool m_verbose;                           // true: enable debug output, false: quiet mode (default)
        int m_udp_recv_buffer_size;               // size of buffer to receive udp packages
        std::vector<uint8_t> m_udp_msg_start_seq; // any udp message from multiScan136 starts with 15 byte ".....class.Scan"
        double m_udp_timeout_recv_nonblocking;    // in normal mode we receive udp datagrams non-blocking with timeout to enable sync with msgpack start
        double m_udp_sender_timeout;              // if no udp packages received within some seconds, we switch to blocking udp receive
        bool m_export_udp_msg;                    // true : export binary udpand msgpack data to file(*.udpand* .msg), default: false
        int m_scandataformat;                     // ScanDataFormat: 1 for msgpack or 2 for compact scandata, default: 1

        /*
         * Member data to run a udp receiver
         */
        UdpReceiverSocketImpl* m_socket_impl;     // implementation of the udp receiver socket
        PayloadFifo* m_fifo_impl;                 // implementation of a thread safe fifo buffer to share the payload of udp packages
        bool m_fifo_impl_created;                 // m_fifo_impl has been created and needs be deleted at exit
        std::thread* m_receiver_thread;           // background thread to receive udp packages
        bool m_run_receiver_thread;               // flag to start and stop the udp receiver thread


    };  // class UdpReceiver

}   // namespace sick_scansegment_xd
#endif // __SICK_SCANSEGMENT_XD_UDP_RECEIVER_H
