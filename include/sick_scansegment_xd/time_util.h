#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * @brief time_util.h implements utility functions for time measurement and profiling.
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
#ifndef __SICK_SCANSEGMENT_XD_TIME_UTIL_H
#define __SICK_SCANSEGMENT_XD_TIME_UTIL_H

#include "sick_scan/sick_ros_wrapper.h"
#include "sick_scansegment_xd/common.h"

namespace sick_scansegment_xd
{
    /*
     * class TimingStatistics is a utility class to measure basic statistics (mean, max, stddev and histogram) for the duration of a msgpack in milliseconds.
     */
    class TimingStatistics
    {
    public:

        /*
         * Default constructor, initializes all member with 0
         */
        TimingStatistics() : m_cnt(0), m_sum(0), m_sum_sq(0), m_max(0), m_hist(11) {}

        /*
         * Adds a duration in milliseconds to the time statistics
         */
        void AddTimeMilliseconds(double t)
        {
            m_sum += t;
            m_sum_sq += (t * t);
            m_max = std::max(m_max, t);
            int hist_idx = std::min((int)t, (int)m_hist.size() - 1);
            m_hist[hist_idx] += 1;
            m_cnt++;
        }

        /*
         * Returns the mean duration in milliseconds
         */
        double MeanMilliseconds(void) const
        {
            if(m_cnt > 0)
            {
                return m_sum / (double)m_cnt;
            }
            return 0;
        }

        /*
         * Returns the standard deviation of all durations in milliseconds
         */
        double StddevMilliseconds(void) const
        {
            if (m_cnt > 1)
            {
                double var = (m_sum_sq - (m_sum * m_sum) / m_cnt) / (m_cnt - 1);
                return std::sqrt(var);
            }
            return 0;
        }

        /*
         * Returns the max duration in milliseconds
         */
        double MaxMilliseconds(void)  const
        {
            return m_max;
        }

        /*
         * Prints the duration histogram to string, f.e. "10,5,6,...,3" means 10 durations between 0 and 1 ms, 5 durations between 1 and 2 ms, 3 durations greater 10 ms.
         */
        std::string PrintHistMilliseconds(const std::string& separator = ",") const
        {
            std::stringstream s;
            s << m_hist[0];
            for (size_t n = 1; n < m_hist.size(); n++)
                s << separator << m_hist[n];
            return s.str();
        }

    protected:

        size_t m_cnt;
        double m_sum;
        double m_sum_sq;
        double m_max;
        std::vector<int> m_hist;
    };  // class TimingStatistics

} // namespace sick_scansegment_xd
#endif // __SICK_SCANSEGMENT_XD_TIME_UTIL_H
