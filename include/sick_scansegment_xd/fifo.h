#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * @brief fifo implements a thread-safe fifo.
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
#ifndef __SICK_SCANSEGMENT_XD_FIFO_H
#define __SICK_SCANSEGMENT_XD_FIFO_H

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

/*
 * Shortcuts to use either std::chrono::high_resolution_clock or std::chrono::system_clock
 */
typedef std::chrono::time_point<std::chrono::system_clock> fifo_timestamp;
typedef std::chrono::system_clock fifo_clock;
// typedef std::chrono::time_point<std::chrono::high_resolution_clock> fifo_timestamp;
// typedef std::chrono::high_resolution_clock fifo_clock;

namespace sick_scansegment_xd
{
    template <typename T> class Fifo
    {
	public:

        /*
         * @brief Fifo default constructor
         * @param[in] fifo_length max. fifo length (-1: unlimited, default: 20 for buffering 1 second at 20 Hz), elements will be removed from front if number of elements exceeds the fifo_length
         */
        Fifo(int fifo_length = 20) : m_fifo_length(fifo_length), m_shutdown(false), m_num_messages_received(0), m_timestamp_last_msg_received() {}

        /*
         * @brief Fifo destructor
         */
        virtual ~Fifo() {}

        /*
         * @brief Pushes an element to the end of the fifo and returns the new number of elements in the fifo.
         */
        virtual size_t Push(const T& element, const fifo_timestamp timestamp = fifo_clock::now(), size_t counter = 0)
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_queue.push(std::make_tuple(element, timestamp, counter));
            m_num_messages_received++;
            m_timestamp_last_msg_received = timestamp;
            while(m_fifo_length > 0 && m_queue.size() > m_fifo_length)
                m_queue.pop();
            m_cond.notify_all();
            return m_queue.size();
        }

        /*
         * @brief Pops an element from the front of the fifo.
         */
        virtual bool Pop(T& element, fifo_timestamp& timestamp, size_t& counter)
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            while (!m_shutdown && m_queue.empty())
            {
                m_cond.wait(lock);
            }
            if (m_shutdown || m_queue.empty())
                return false;
            const fifo_element& queue_front = m_queue.front();
            element = std::get<0>(queue_front);
            timestamp = std::get<1>(queue_front);
            counter = std::get<2>(queue_front);
            m_queue.pop();
            return true;
        }

        /*
         * @brief Returns the number of elements in the fifo.
         */
        virtual size_t Size(void)
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            return m_queue.size();
        }

        /*
         * @brief Sets the fifo in shutdown mode and interrupts a waiting Pop() call.
         * After Shutdown(), any Pop() will return immediately with false.
         */
        virtual void Shutdown(void)
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_shutdown = true;
            m_cond.notify_all();
        }

        /*
         * @brief Returns the total number of messages pushed to fifo since constructed
         */
        virtual size_t TotalMessagesPushed()
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            return m_num_messages_received;
        }

        /*
         * @brief Returns the time in seconds since the last message has been pushed (i.e. since last message received from lidar)
         */
        virtual double SecondsSinceLastPush()
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            return Seconds(m_timestamp_last_msg_received, fifo_clock::now());
        }

        /*
         * @brief Returns the duration in seconds between a push (start) and pop (end) timestamps
         */
        static double Seconds(const fifo_timestamp& timestamp_start, const fifo_timestamp& timestamp_end = fifo_clock::now())
        {
            return (1.0e-9) * (std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp_end - timestamp_start)).count(); // std::chrono::duration::count() in nanoseconds
        }


    protected:

        typedef std::tuple<T, fifo_timestamp, size_t> fifo_element;
        std::queue<fifo_element> m_queue; // queue to buffer the elements in a fifo
        std::mutex m_mutex;               // mutex to protect multithreaded queue access
        std::condition_variable m_cond;   // condition to wait and notify on push and pop
        int m_fifo_length;                // max. fifo length (-1: unlimited, default: 20 for buffering 1 second at 20 Hz), elements will be removed from front if number of elements exceeds the fifo_length
        bool m_shutdown;                  // if true, fifo is in shutdown mode and Pop returns immediately, default: false
        size_t m_num_messages_received;   // total number of messages pushed to fifo
        fifo_timestamp m_timestamp_last_msg_received; // timestamp of last message pushed to fifo
    };

    /*
     * Fifo for any payload, i.e. a chunk of bytes
     */
    class PayloadFifo : public Fifo<std::vector<uint8_t>>  
    { 
    public:
        /*
         * @brief PayloadFifo default constructor
         * @param[in] fifo_length max. fifo length (-1: unlimited, default: 20 for buffering 1 second at 20 Hz), elements will be removed from front if number of elements exceeds the fifo_length
         */
        PayloadFifo(int fifo_length = 20) : Fifo<std::vector<uint8_t>>(fifo_length) {}
    };

} // namespace sick_scansegment_xd
#endif // __SICK_SCANSEGMENT_XD_FIFO_H
