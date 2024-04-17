#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * @brief scansegement_threads runs all threads to receive, convert and publish scan data for the sick 3D lidar multiScan136.
 *
 * Copyright (C) 2022 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2022 SICK AG, Waldkirch
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
 *  Copyright 2022 SICK AG
 *  Copyright 2022 Ing.-Buero Dr. Michael Lehning
 *
 */
#ifndef __SICK_SCANSEGEMENT_THREADS_H
#define __SICK_SCANSEGEMENT_THREADS_H

namespace sick_scan_xd
{
  class SickScanServices;
}

#include "sick_scansegment_xd/config.h"

namespace sick_scansegment_xd
{
    /*
	  * @brief Initializes and runs all threads to receive, convert and publish scan data for the sick 3D lidar multiScan136.
	  */
    int run(rosNodePtr node, const std::string& scannerName);

    /*
	  * @brief class MsgPackThreads runs all threads to receive, convert and publish scan data for the sick 3D lidar multiScan136.
	  */
    class MsgPackThreads
    {
    public:

        /*
	     * @brief MsgPackThreads constructor
	     */
        MsgPackThreads();

        /*
	     * @brief MsgPackThreads destructor
	     */
        ~MsgPackThreads();

        /*
	     * @brief Initializes msgpack receiver, converter and publisher and starts msgpack handling and publishing in a background thread.
	     */
        bool start(const sick_scansegment_xd::Config& config);

        /*
	     * @brief Stops running threads and closes msgpack receiver, converter and publisher.
	     */
        bool stop(bool do_join);

        /*
	     * @brief Joins running threads and returns after they finished.
	     */
        void join(void);

    protected:

        /*
        * @brief Thread callback, initializes and runs msgpack receiver, converter and publisher.
        */
        bool runThreadCb(void);

       sick_scansegment_xd::Config m_config;                      // sick_scansegment_xd configuration
       std::thread* m_scansegment_thread;                         // background thread to convert msgpack to ScanSegmentParserOutput data
       bool m_run_scansegment_thread;                             // flag to start and stop the udp converter thread
    };

    sick_scan_xd::SickScanServices* sopasService();

}   // namespace sick_scansegment_xd
#endif // __SICK_SCANSEGEMENT_THREADS_H
