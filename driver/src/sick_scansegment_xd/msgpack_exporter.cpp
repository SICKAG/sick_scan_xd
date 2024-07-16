/*
 * @brief msgpack_exporter runs a background thread to consume and export msgpack/compact data from the sick 3D lidar multiScan136
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
#include "sick_scansegment_xd/msgpack_exporter.h"
#include "sick_scansegment_xd/time_util.h"

/*
 * @brief Default constructor.
 */
sick_scansegment_xd::MsgPackExporter::MsgPackExporter() : m_udp_fifo(0), m_msgpack_fifo(0), m_logfolder(""), m_export_csv(false), m_verbose(false), m_measure_timing(false), m_exporter_thread(0), m_run_exporter_thread(false)
{
}

/*
 * @brief Initializing constructor
 * @param[in] udp_fifo fifo buffering udp packages (for informational messages only)
 * @param[in] msgpack_fifo fifo buffering ScanSegmentParserOutput data from multiScan136 (for csv export and visualization)
 * @param[in] logfolder output folder for optional csv-files
 * @param[in] export_csv true: export ScanSegmentParserOutput data to csv files
 * @param[in] verbose true: enable debug output, false: quiet mode (default)
 * @param[in] measure_timing true: duration and latency of msgpack conversion and export is measured, default: false
 */
sick_scansegment_xd::MsgPackExporter::MsgPackExporter(sick_scansegment_xd::PayloadFifo* udp_fifo, sick_scansegment_xd::Fifo<ScanSegmentParserOutput>* msgpack_fifo, const std::string& logfolder, bool export_csv, bool verbose, bool measure_timing)
: m_udp_fifo(udp_fifo), m_msgpack_fifo(msgpack_fifo), m_logfolder(logfolder), m_export_csv(export_csv), m_verbose(verbose), m_measure_timing(measure_timing), m_exporter_thread(0), m_run_exporter_thread(false)
{
}

/*
 * @brief Default destructor.
 */
sick_scansegment_xd::MsgPackExporter::~MsgPackExporter()
{
    Close();
}

/*
 * @brief Registers a listener to msgpack export data. MsgPackExporter will notify registered listeners
 * whenever msgpack data have been received and successfully converted by calling listener->HandleMsgPackData().
 */
void sick_scansegment_xd::MsgPackExporter::AddExportListener(sick_scansegment_xd::MsgPackExportListenerIF* listener)
{
    std::unique_lock<std::mutex> lock(m_listener_mutex);
    m_listener.push_back(listener);
}

/*
 * @brief Removes a registered listener.
 */
void sick_scansegment_xd::MsgPackExporter::RemoveExportListener(sick_scansegment_xd::MsgPackExportListenerIF* listener)
{
    std::unique_lock<std::mutex> lock(m_listener_mutex);
    for (std::list<sick_scansegment_xd::MsgPackExportListenerIF*>::iterator iter = m_listener.begin(); iter != m_listener.end(); )
    {
        if (*iter == listener)
            iter = m_listener.erase(iter);
        else
            iter++;
    }
}

/*
 * @brief Returns the list of registered listeners
 */
std::list<sick_scansegment_xd::MsgPackExportListenerIF*> sick_scansegment_xd::MsgPackExporter::GetExportListener()
{
    std::unique_lock<std::mutex> lock(m_listener_mutex);
    return m_listener;
}

/*
 * @brief Starts a background thread to pops msgpack data packages from the input fifo and optionally export them to csv and/or plot the lidar points.
 * Note: If visualization is activated, export must run in the main thread, i.e. this function blocks and returns after pressing ESC, 'q' or 'Q'.
 * If visualization is NOT activated, this function starts a background thread to run the data export, i.e. this function does NOT block and returns immediately.
 */
bool sick_scansegment_xd::MsgPackExporter::Start(void)
{
    m_run_exporter_thread = true;
    m_exporter_thread = new std::thread(&sick_scansegment_xd::MsgPackExporter::RunCb, this); // Without visualization, msgpack export runs in background thread
    return true;
}

/*
 * @brief Stops the background thread
 */
void sick_scansegment_xd::MsgPackExporter::Stop(void)
{
  m_run_exporter_thread = false;
}

/*
 * @brief Stops, joins and deletes the background thread
 */
void sick_scansegment_xd::MsgPackExporter::Close(void)
{
    m_run_exporter_thread = false;
    if (m_exporter_thread)
    {
        if (m_exporter_thread->joinable())
            m_exporter_thread->join();
        delete m_exporter_thread;
        m_exporter_thread = 0;
    }
}

/*
 * @brief Thread callback, runs the exporter. Pops msgpack data packages from the input fifo and optionally export them to csv and/or plot the lidar points.
 */
bool sick_scansegment_xd::MsgPackExporter::RunCb(void)
{
    if (!m_udp_fifo || !m_msgpack_fifo)
    {
        ROS_ERROR_STREAM("## ERROR MsgPack/Compact-Exporter::Run(): MsgPack/Compact-Exporter not initialized.");
        return false;
    }
    try
    {
        fifo_timestamp recv_start_timestamp = fifo_clock::now();
        fifo_timestamp last_print_timestamp = fifo_clock::now();
        size_t msg_exported_counter = 0; // number of exported scandata (msgpack or compact)
        size_t msg_first_udp_counter = 0; // number of udp datagrams received
        size_t max_count_udp_messages_in_fifo = 0;
        size_t max_count_output_messages_in_fifo = 0;
        int msg_cnt_delta_max = 0;
        sick_scansegment_xd::TimingStatistics duration_datahandling_milliseconds;
        while (m_run_exporter_thread)
        {
            sick_scansegment_xd::ScanSegmentParserOutput msgpack_output;
            fifo_timestamp msgpack_timestamp;
            size_t msgpack_counter = 0;
            if (m_msgpack_fifo->Pop(msgpack_output, msgpack_timestamp, msgpack_counter) && m_run_exporter_thread)
            {
                // Notify registered listeners about new scandata (msgpack or compact)
                std::list<sick_scansegment_xd::MsgPackExportListenerIF*> listener = GetExportListener();
                for (std::list<sick_scansegment_xd::MsgPackExportListenerIF*>::iterator iter = listener.begin(); iter != listener.end(); iter++)
                {
                    if (*iter)
                    {
                        (*iter)->HandleMsgPackData(msgpack_output);
                    }
                }
                // Optionally export to csv file
                if (m_export_csv && !m_logfolder.empty())
                {
                    std::string csv_file = m_logfolder + "/scansegment_" + sick_scansegment_xd::FormatNumber(msgpack_output.segmentIndex, 6, true, false, -1) + ".csv";
                    if (!sick_scansegment_xd::MsgPackParser::WriteCSV({ msgpack_output }, csv_file, true))
                        ROS_ERROR_STREAM("## ERROR MsgPackParser::WriteCSV() failed.");
                }
                // Profiling and time measurement
                if (msg_exported_counter == 0) // first time receiving scandata (msgpack or compact)
                {
                    msg_first_udp_counter = msgpack_counter;
                    recv_start_timestamp = msgpack_timestamp;
                }
                msg_exported_counter += 1;
                size_t msg_udp_received_counter = (msgpack_counter - msg_first_udp_counter) + 1;
                if (m_measure_timing)
                {
                    int msg_cnt_delta = std::max(0, (int)msg_udp_received_counter - (int)msg_exported_counter);
                    double packages_lost_rate = std::abs((double)msg_cnt_delta) / (double)msg_udp_received_counter;
                    bool do_print_message = m_verbose &&
                        (sick_scansegment_xd::Fifo<ScanSegmentParserOutput>::Seconds(last_print_timestamp, fifo_clock::now()) > 1.0) && // avoid printing with more than 1 Hz
                        ((msg_exported_counter%100) == 0);
                    if (msg_udp_received_counter != msg_exported_counter && msg_cnt_delta > msg_cnt_delta_max && do_print_message) // Test mode only, multiScan emulator must be started after lidar3d_multiscan_recv
                    {
                        ROS_INFO_STREAM("MsgPack/Compact-Exporter::Run(): " << msg_udp_received_counter << " udp messages received, " << msg_exported_counter << " messages exported, " << (100.0 * packages_lost_rate) << "% package lost");
                        msg_cnt_delta_max = msg_cnt_delta;
                        last_print_timestamp = fifo_clock::now();
                    }
                    size_t current_udp_fifo_size = m_udp_fifo->Size();
                    size_t current_output_fifo_size = m_msgpack_fifo->Size();
                    double duration_datahandling_seconds = sick_scansegment_xd::Fifo<ScanSegmentParserOutput>::Seconds(msgpack_timestamp, fifo_clock::now());
                    duration_datahandling_milliseconds.AddTimeMilliseconds(1000.0 * duration_datahandling_seconds);
                    max_count_udp_messages_in_fifo = std::max(max_count_udp_messages_in_fifo, current_udp_fifo_size + 1);
                    max_count_output_messages_in_fifo = std::max(max_count_output_messages_in_fifo, current_output_fifo_size + 1);
                    double msg_exported_rate = (double)msg_exported_counter / sick_scansegment_xd::Fifo<ScanSegmentParserOutput>::Seconds(recv_start_timestamp, fifo_clock::now());
                    if (m_verbose && ((msg_exported_counter%100) == 0 || sick_scansegment_xd::Fifo<ScanSegmentParserOutput>::Seconds(last_print_timestamp, fifo_clock::now()) > 0.1)) // avoid printing with more than 100 Hz
                    {
                        ROS_INFO_STREAM("MsgPack/Compact-Exporter:   " << current_udp_fifo_size << " udp packages still in input fifo, " << current_output_fifo_size << " messages still in output fifo, current segment index: " << msgpack_output.segmentIndex);
                        ROS_INFO_STREAM("MsgPack/Compact-Exporter: " << msg_udp_received_counter << " udp scandata messages received, " << msg_exported_counter << " messages exported (scan+imu), " << (100.0 * packages_lost_rate) << "% package lost.");
                        ROS_INFO_STREAM("MsgPack/Compact-Exporter: max. " << max_count_udp_messages_in_fifo << " udp messages buffered, max " << max_count_output_messages_in_fifo << " export messages buffered.");
                        std::stringstream s;
                        s << "MsgPack/Compact-Exporter: " << msg_exported_counter << " messages exported at " << std::fixed << std::setprecision(3) << msg_exported_rate << " Hz, mean time: " 
                            << std::fixed << std::setprecision(3) << duration_datahandling_milliseconds.MeanMilliseconds() << " milliseconds/messages, " 
                            << "stddev time: " << duration_datahandling_milliseconds.StddevMilliseconds() << ", " << "max time: " << duration_datahandling_milliseconds.MaxMilliseconds() << " milliseconds between udp receive and messages export, "
                            << "histogram=[" << duration_datahandling_milliseconds.PrintHistMilliseconds() << "]";
                        ROS_INFO_STREAM(s.str());
                        last_print_timestamp = fifo_clock::now();
                    }
                }
            }
        }
        if (m_measure_timing && m_verbose)
        {
            size_t current_udp_fifo_size = m_udp_fifo->Size();
            size_t current_output_fifo_size = m_msgpack_fifo->Size();
            double msg_exported_rate = (double)msg_exported_counter / sick_scansegment_xd::Fifo<ScanSegmentParserOutput>::Seconds(recv_start_timestamp, fifo_clock::now());
            std::stringstream info1, info2;
            info1 << "MsgPack/Compact-Exporter: finished, " << current_udp_fifo_size << " udp packages still in input fifo, " << current_output_fifo_size << " messages still in output fifo"
                << ", max. " << max_count_udp_messages_in_fifo << " udp messages buffered, max " << max_count_output_messages_in_fifo << " export messages buffered.";
            info2 << "MsgPack/Compact-Exporter: " << msg_exported_counter << " messages exported at " << msg_exported_rate << " Hz, mean time: " << duration_datahandling_milliseconds.MeanMilliseconds() << " milliseconds/messages, " 
                << "stddev time: " << duration_datahandling_milliseconds.StddevMilliseconds() << ", " << "max time: " << duration_datahandling_milliseconds.MaxMilliseconds() << " milliseconds between udp receive and messages export, "
                << "histogram=[" << duration_datahandling_milliseconds.PrintHistMilliseconds() << "]";
            ROS_INFO_STREAM(info1.str());
            ROS_INFO_STREAM(info2.str());
            std::cout << info1.str() << std::endl;
            std::cout << info2.str() << std::endl;
        }
        m_run_exporter_thread = false;
        return true;
    }
    catch (std::exception & e)
    {
        ROS_ERROR_STREAM("## ERROR MsgPack/Compact-Exporter::Run(): " << e.what());
    }
    m_run_exporter_thread = false;
    return false;
}
