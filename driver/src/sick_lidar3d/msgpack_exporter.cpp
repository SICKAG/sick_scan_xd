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
#include "sick_lidar3d/msgpack_exporter.h"
#include "sick_lidar3d/python_wrapper.h"
#include "sick_lidar3d/time_util.h"

/*
 * @brief Default constructor.
 */
sick_lidar3d::MsgPackExporter::MsgPackExporter() : m_udp_fifo(0), m_msgpack_fifo(0), m_logfolder(""), m_export_csv(false), m_verbose(false), m_measure_timing(false), m_visualize(false), m_exporter_thread(0), m_run_exporter_thread(false)
{
}

/*
 * @brief Initializing constructor
 * @param[in] udp_fifo fifo buffering udp packages (for informational messages only)
 * @param[in] msgpack_fifo fifo buffering MsgPackParserOutput data from multiScan136 (for csv export and visualization)
 * @param[in] logfolder output folder for optional csv-files
 * @param[in] export_csv true: export MsgPackParserOutput data to csv files
 * @param[in] verbose true: enable debug output, false: quiet mode (default)
 * @param[in] measure_timing true: duration and latency of msgpack conversion and export is measured, default: false
 * @param[in] visualize true: plot MsgPackParserOutput data, false: quiet mode (default)
 */
sick_lidar3d::MsgPackExporter::MsgPackExporter(sick_lidar3d::PayloadFifo* udp_fifo, sick_lidar3d::Fifo<MsgPackParserOutput>* msgpack_fifo, const std::string& logfolder, bool export_csv, bool verbose, bool measure_timing, bool visualize)
: m_udp_fifo(udp_fifo), m_msgpack_fifo(msgpack_fifo), m_logfolder(logfolder), m_export_csv(export_csv), m_verbose(verbose), m_measure_timing(measure_timing), m_visualize(visualize), m_exporter_thread(0), m_run_exporter_thread(false)
{
#ifdef USE_PYTHON
    if (m_visualize)
    {
        PyRun_SimpleString("import matplotlib.pyplot as plt");
        PyRun_SimpleString("import numpy as np");
        PyRun_SimpleString("fig = plt.figure(figsize=(1000/100,1000/100), dpi=100)");
        PyRun_SimpleString("ax = fig.add_subplot(111, projection='3d')");
    }
#else
    m_visualize = false; // Current visualization uses matplotlib and python API
#endif // USE_PYTHON
}

/*
 * @brief Default destructor.
 */
sick_lidar3d::MsgPackExporter::~MsgPackExporter()
{
    Close();
}

/*
 * @brief Registers a listener to msgpack export data. MsgPackExporter will notify registered listeners
 * whenever msgpack data have been received and successfully converted by calling listener->HandleMsgPackData().
 */
void sick_lidar3d::MsgPackExporter::AddExportListener(sick_lidar3d::MsgPackExportListenerIF* listener)
{
    m_listener.push_back(listener);
}

/*
 * @brief Removes a registered listener.
 */
void sick_lidar3d::MsgPackExporter::RemoveExportListener(sick_lidar3d::MsgPackExportListenerIF* listener)
{
    for (std::list<sick_lidar3d::MsgPackExportListenerIF*>::iterator iter = m_listener.begin(); iter != m_listener.end(); )
    {
        if (*iter == listener)
            iter = m_listener.erase(iter);
        else
            iter++;
    }
}

/*
 * @brief Starts a background thread to pops msgpack data packages from the input fifo and optionally export them to csv and/or plot the lidar points.
 * Note: If visualization is activated, export must run in the main thread, i.e. this function blocks and returns after pressing ESC, 'q' or 'Q'.
 * If visualization is NOT activated, this function starts a background thread to run the data export, i.e. this function does NOT block and returns immediately.
 */
bool sick_lidar3d::MsgPackExporter::Start(void)
{
    m_run_exporter_thread = true;
    if (m_visualize)
    {
        return Run(); // Visualization must run in main thread
    }
    else
    {
        m_exporter_thread = new std::thread(&sick_lidar3d::MsgPackExporter::Run, this); // Without visualization, msgpack export runs in background thread
    }
    return true;
}

/*
 * @brief Stops the background thread
 */
void sick_lidar3d::MsgPackExporter::Close(void)
{
    m_run_exporter_thread = false;
    if (m_exporter_thread)
    {
        m_exporter_thread->join();
        delete m_exporter_thread;
        m_exporter_thread = 0;
    }
}

/*
 * @brief Plots lidar points (x, y, z) and intensity i using matplotlib via Python-API
 */
void sick_lidar3d::MsgPackExporter::PlotXYZI(const std::vector<float>& x, const std::vector<float>& y, const std::vector<float>& z, const std::vector<int>& i)
{
#ifdef USE_PYTHON
    assert(x.size() == y.size() && x.size() == z.size() && x.size() == i.size());
    std::stringstream xs, ys, zs, is;
    xs << "x = [";
    ys << "y = [";
    zs << "z = [";
    is << "i = [";
    for (size_t n = 0; n < x.size(); n++)
    {
        xs << (n > 0 ? "," : "") << x[n];
        ys << (n > 0 ? "," : "") << y[n];
        zs << (n > 0 ? "," : "") << z[n];
        is << (n > 0 ? "," : "") << i[n];
    }
    xs << " ]";
    ys << " ]";
    zs << " ]";
    is << " ]";
    PyRun_SimpleString(xs.str().c_str());
    PyRun_SimpleString(ys.str().c_str());
    PyRun_SimpleString(zs.str().c_str());
    PyRun_SimpleString(is.str().c_str());
    PyRun_SimpleString("x_np = np.asarray(x, dtype=np.float32)");
    PyRun_SimpleString("y_np = np.asarray(y, dtype=np.float32)");
    PyRun_SimpleString("z_np = np.asarray(z, dtype=np.float32)");
    PyRun_SimpleString("i_np = np.asarray(i, dtype=np.float32)");
    PyRun_SimpleString("ax.clear()");
    PyRun_SimpleString("ax.set_title('MRS100 emulation')");
    PyRun_SimpleString("ax.set_xlim(-1.5, 1.5)");
    PyRun_SimpleString("ax.set_ylim(-1.5, 1.5)");
    PyRun_SimpleString("ax.set_zlim(-1.5, 1.5)");
    PyRun_SimpleString("ax.set_xlabel('X')");
    PyRun_SimpleString("ax.set_ylabel('Y')");
    PyRun_SimpleString("ax.set_zlabel('Z')");
    PyRun_SimpleString("scatter = ax.scatter(x_np, y_np, z_np, c=i_np, cmap='jet', s=1)");
    PyRun_SimpleString("plt.draw()");
    PyRun_SimpleString("plt.show(block = False)");
    PyRun_SimpleString("plt.pause(0.1)");

#endif // USE_PYTHON
}

/*
 * @brief Runs the exporter in the current thread. Pops msgpack data packages from the input fifo and optionally export them to csv and/or plot the lidar points.
 */
bool sick_lidar3d::MsgPackExporter::Run(void)
{
    m_run_exporter_thread = true;
    return RunCb();
}

/*
 * @brief Thread callback, runs the exporter. Pops msgpack data packages from the input fifo and optionally export them to csv and/or plot the lidar points.
 */
bool sick_lidar3d::MsgPackExporter::RunCb(void)
{
    if (!m_udp_fifo || !m_msgpack_fifo)
    {
        LIDAR3D_ERROR_STREAM("## ERROR MsgPackExporter::Run(): MsgPackExporter not initialized.");
        return false;
    }
    try
    {
        fifo_timestamp recv_start_timestamp = fifo_clock::now();
        fifo_timestamp last_print_timestamp = fifo_clock::now();
        size_t msg_exported_counter = 0; // number of exported msgpacks
        size_t msg_first_udp_counter = 0; // number of udp datagrams received
        size_t max_count_udp_messages_in_fifo = 0;
        size_t max_count_output_messages_in_fifo = 0;
        int msg_cnt_delta_max = 0;
        sick_lidar3d::TimingStatistics duration_datahandling_milliseconds;
        while (m_run_exporter_thread)
        {
            sick_lidar3d::MsgPackParserOutput msgpack_output;
            fifo_timestamp msgpack_timestamp;
            size_t msgpack_counter = 0;
            if (m_msgpack_fifo->Pop(msgpack_output, msgpack_timestamp, msgpack_counter))
            {
                // Notify registered listeners about new msgpack data
                for (std::list<sick_lidar3d::MsgPackExportListenerIF*>::iterator iter = m_listener.begin(); iter != m_listener.end(); iter++)
                {
                    if (*iter)
                    {
                        (*iter)->HandleMsgPackData(msgpack_output);
                    }
                }
                // Optionally export to csv file
                if (m_export_csv)
                {
                    std::string csv_file = m_logfolder + "/msgpack_" + sick_lidar3d::FormatNumber(msgpack_output.segmentIndex, 6, true, false, -1) + ".csv";
                    if (!sick_lidar3d::MsgPackParser::WriteCSV({ msgpack_output }, csv_file, true))
                        LIDAR3D_ERROR_STREAM("## ERROR MsgPackParser::WriteCSV() failed.");
                }
                // Optionally visualize
#               ifdef USE_PYTHON
                if (m_visualize && m_msgpack_fifo->Size() <= 1) // Visualization by matplotlib via python API is blocking, so we just plot when the msgpack fifo is "almost idle"
                {
                    std::vector<float> x, y, z, i;
                    std::vector<int> group_idx, echo_idx, msg_idx;
                    sick_lidar3d::MsgPackParser::ExportXYZI({ msgpack_output }, x, y, z, i, group_idx, echo_idx, msg_idx);
                    assert(x.size() == y.size() && x.size() == z.size() && x.size() == i.size() && x.size() == group_idx.size() && echo_idx.size() == msg_idx.size());
                    PlotXYZI(x, y, z, group_idx);
                    int c = 0;
                    if (KBHIT() && ((c = GETCH()) == 27 || c == 'q' || c == 'Q'))
                    {
                        m_run_exporter_thread = false; // stop after pressing ESC, 'q' or 'Q'.
                    }
                }
#               endif // USE_PYTHON
                // Profiling and time measurement
                if (msg_exported_counter == 0) // first time receiving a msgpack
                {
                    msg_first_udp_counter = msgpack_counter;
                    recv_start_timestamp = msgpack_timestamp;
                }
                msg_exported_counter += 1;
                size_t msg_udp_received_counter = (msgpack_counter - msg_first_udp_counter) + 1;
                if (m_measure_timing)
                {
                    int msg_cnt_delta = (int)msg_udp_received_counter - (int)msg_exported_counter;
                    double packages_lost_rate = std::abs((double)msg_cnt_delta) / (double)msg_udp_received_counter;
                    if (m_verbose && msg_udp_received_counter != msg_exported_counter && msg_cnt_delta > msg_cnt_delta_max) // Test mode only, mrs100 emulator must be started after lidar3d_msr100_recv
                    {
                        LIDAR3D_INFO_STREAM("MsgPackExporter::Run(): " << msg_udp_received_counter << " udp messages received, " << msg_exported_counter << " messages exported, " << (100.0 * packages_lost_rate) << "% package lost");
                        msg_cnt_delta_max = msg_cnt_delta;
                    }
                    size_t current_udp_fifo_size = m_udp_fifo->Size();
                    size_t current_output_fifo_size = m_msgpack_fifo->Size();
                    double duration_datahandling_seconds = sick_lidar3d::Fifo<MsgPackParserOutput>::Seconds(msgpack_timestamp, fifo_clock::now());
                    duration_datahandling_milliseconds.AddTimeMilliseconds(1000.0 * duration_datahandling_seconds);
                    max_count_udp_messages_in_fifo = std::max(max_count_udp_messages_in_fifo, current_udp_fifo_size + 1);
                    max_count_output_messages_in_fifo = std::max(max_count_output_messages_in_fifo, current_output_fifo_size + 1);
                    double msg_exported_rate = (double)msg_exported_counter / sick_lidar3d::Fifo<MsgPackParserOutput>::Seconds(recv_start_timestamp, fifo_clock::now());
                    if (m_verbose && ((msg_exported_counter%100) == 0 || sick_lidar3d::Fifo<MsgPackParserOutput>::Seconds(last_print_timestamp, fifo_clock::now()) > 0.1)) // avoid printing with more than 100 Hz
                    {
                        LIDAR3D_INFO_STREAM("MsgPackExporter:   " << current_udp_fifo_size << " udp packages still in input fifo, " << current_output_fifo_size << " messages still in msgpack output fifo, current message count: " << msgpack_output.segmentIndex);
                        LIDAR3D_INFO_STREAM("MsgPackExporter: " << msg_udp_received_counter << " udp messages received, " << msg_exported_counter << " messages exported, " << (100.0 * packages_lost_rate) << "% package lost.");
                        LIDAR3D_INFO_STREAM("MsgPackExporter: max. " << max_count_udp_messages_in_fifo << " udp messages buffered, max " << max_count_output_messages_in_fifo << " export messages buffered.");
                        LIDAR3D_INFO_STREAM("MsgPackExporter: " << msg_exported_counter << " msgpacks exported at " << msg_exported_rate << " Hz, mean time: " << duration_datahandling_milliseconds.MeanMilliseconds() << " milliseconds/msgpack, " 
                            << "stddev time: " << duration_datahandling_milliseconds.StddevMilliseconds() << ", " << "max time: " << duration_datahandling_milliseconds.MaxMilliseconds() << " milliseconds between udp receive and msgpack export, "
                            << "histogram=[" << duration_datahandling_milliseconds.PrintHistMilliseconds() << "]");
                        last_print_timestamp = fifo_clock::now();
                    }
                }
            }
        }
        if (m_measure_timing && m_verbose)
        {
            size_t current_udp_fifo_size = m_udp_fifo->Size();
            size_t current_output_fifo_size = m_msgpack_fifo->Size();
            double msg_exported_rate = (double)msg_exported_counter / sick_lidar3d::Fifo<MsgPackParserOutput>::Seconds(recv_start_timestamp, fifo_clock::now());
            std::stringstream info1, info2;
            info1 << "MsgPackExporter: finished, " << current_udp_fifo_size << " udp packages still in input fifo, " << current_output_fifo_size << " messages still in msgpack output fifo"
                << ", max. " << max_count_udp_messages_in_fifo << " udp messages buffered, max " << max_count_output_messages_in_fifo << " export messages buffered.";
            info2 << "MsgPackExporter: " << msg_exported_counter << " msgpacks exported at " << msg_exported_rate << " Hz, mean time: " << duration_datahandling_milliseconds.MeanMilliseconds() << " milliseconds/msgpack, " 
                << "stddev time: " << duration_datahandling_milliseconds.StddevMilliseconds() << ", " << "max time: " << duration_datahandling_milliseconds.MaxMilliseconds() << " milliseconds between udp receive and msgpack export, "
                << "histogram=[" << duration_datahandling_milliseconds.PrintHistMilliseconds() << "]";
            LIDAR3D_INFO_STREAM(info1.str());
            LIDAR3D_INFO_STREAM(info2.str());
            std::cout << info1.str() << std::endl;
            std::cout << info2.str() << std::endl;
        }
        m_run_exporter_thread = false;
        return true;
    }
    catch (std::exception & e)
    {
        LIDAR3D_ERROR_STREAM("## ERROR MsgPackExporter::Run(): " << e.what());
    }
    m_run_exporter_thread = false;
    return false;
}
