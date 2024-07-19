#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * @brief msgpack_exporter runs a background thread to consume and export msgpack data from the sick 3D lidar multiScan136
 * to optionally csv file or plotted diagram
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
#ifndef __SICK_SCANSEGMENT_XD_MSGPACK_EXPORTER_H
#define __SICK_SCANSEGMENT_XD_MSGPACK_EXPORTER_H

#include "sick_scan/sick_ros_wrapper.h"
#include "sick_scansegment_xd/common.h"
#include "sick_scansegment_xd/fifo.h"
#include "sick_scansegment_xd/msgpack_parser.h"

namespace sick_scansegment_xd
{
    /*
     * @brief class MsgPackExportListenerIF is an interface to listen to exported msgpack data.
     * Instances implementing the interface MsgPackExportListenerIF can be added to a MsgPackExporter
     * and will be notified whenever msgpack data have been received, successfully converted and
     * are ready to publish (or any other possible action with msgpack data)
     */
    class MsgPackExportListenerIF
    {
    public:
        /*
         * Callback function of MsgPackExportListenerIF. HandleMsgPackData() will be called in MsgPackExporter
         * for each registered listener after msgpack data have been received and converted.
         */
        virtual void HandleMsgPackData(const sick_scansegment_xd::ScanSegmentParserOutput& msgpack_data) = 0;
    };

	/*
     * @brief class MsgPackExporter runs a background thread to consume and export msgpack data from the sick 3D lidar multiScan136
     * to optionally csv file or plotted diagram
     */
	class MsgPackExporter
	{
	public:

        /*
         * @brief Default constructor.
         */
        MsgPackExporter();

        /*
         * @brief Initializing constructor
         * @param[in] udp_fifo fifo buffering udp packages (for informational messages only)
         * @param[in] msgpack_fifo fifo buffering ScanSegmentParserOutput data from multiScan136 (for csv export and visualization)
         * @param[in] logfolder output folder for optional csv-files
         * @param[in] export_csv true: export ScanSegmentParserOutput data to csv files
         * @param[in] verbose true: enable debug output, false: quiet mode (default)
         * @param[in] measure_timing true: duration and latency of msgpack conversion and export is measured, default: false
         */
         MsgPackExporter(sick_scansegment_xd::PayloadFifo* udp_fifo, sick_scansegment_xd::Fifo<ScanSegmentParserOutput>* msgpack_fifo, const std::string& logfolder, bool export_csv, bool verbose = false, bool measure_timing = false);

        /*
         * @brief Default destructor.
         */
        ~MsgPackExporter();

        /*
         * @brief Starts a background thread to pops msgpack data packages from the input fifo and optionally export them to csv and/or plot the lidar points.
         */
        bool Start(void);

        /*
         * @brief Stops the background thread
         */
        void Stop(void);

         /*
         * @brief Stops, joins and deletes the background thread
         */
        void Close(void);

        /*
         * @brief Registers a listener to msgpack export data. MsgPackExporter will notify registered listeners
         * whenever msgpack data have been received and successfully converted by calling listener->HandleMsgPackData().
         */
        void AddExportListener(sick_scansegment_xd::MsgPackExportListenerIF* listener);

        /*
         * @brief Removes a registered listener.
         */
        void RemoveExportListener(sick_scansegment_xd::MsgPackExportListenerIF* listener);

    protected:

       /*
        * @brief Plots lidar points (x, y, z) and intensity i using matplotlib via Python-API
        */
       static void PlotXYZI(const std::vector<float>& x, const std::vector<float>& y, const std::vector<float>& z, const std::vector<int>& i);

       /*
        * @brief Thread callback, runs the exporter. Pops msgpack data packages from the input fifo and optionally export them to csv and/or plot the lidar points.
        */
       bool RunCb(void);

       /*
        * @brief Returns the list of registered listeners
        */
       std::list<sick_scansegment_xd::MsgPackExportListenerIF*> GetExportListener();

       /*
        * Configuration and parameter
        */
       std::string m_logfolder;                                 // output folder for optional csv-files
       bool m_export_csv;                                       // true: export ScanSegmentParserOutput data to csv files
       bool m_verbose;                                          // true: enable debug output, false: quiet mode (default)
       bool m_measure_timing;                                   // true: duration and latency of msgpack conversion and export is measured, default: false

       /*
        * Member data to run the exporter
        */
       sick_scansegment_xd::PayloadFifo* m_udp_fifo;                         // fifo buffering udp packages (for informational messages only)
       sick_scansegment_xd::Fifo<ScanSegmentParserOutput>* m_msgpack_fifo;       // input fifo buffering ScanSegmentParserOutput data from multiScan136 (for csv export and visualization)
       std::thread* m_exporter_thread;                                // background thread to export ScanSegmentParserOutput data
       bool m_run_exporter_thread;                                    // flag to start and stop the exporter thread
       std::list< sick_scansegment_xd::MsgPackExportListenerIF*> m_listener; // list of export listener, which will be notified calling listener->HandleMsgPackData() after successful conversion of received msgpack data
       std::mutex m_listener_mutex; // mutex to protect m_listener access


	};  // class MsgPackExporter

}   // namespace sick_scansegment_xd
#endif // __SICK_SCANSEGMENT_XD_MSGPACK_EXPORTER_H
