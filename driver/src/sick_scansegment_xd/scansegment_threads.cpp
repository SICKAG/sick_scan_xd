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
#include "sick_scansegment_xd/scansegment_threads.h"
#include "sick_scansegment_xd/udp_sockets.h"
#include "sick_scansegment_xd/compact_parser.h"
#include "sick_scansegment_xd/msgpack_converter.h"
#include "sick_scansegment_xd/msgpack_exporter.h"
#include "sick_scansegment_xd/msgpack_validator.h"
#include "sick_scansegment_xd/ros_msgpack_publisher.h"
#include "sick_scansegment_xd/scansegment_parser_output.h"
#include "sick_scansegment_xd/udp_receiver.h"
#include "sick_scan/sick_scan_services.h"

#define DELETE_PTR(p) do{if(p){delete(p);(p)=0;}}while(false)

sick_scan_xd::SickScanServices* s_sopas_service = 0;
sick_scan_xd::SickScanServices* sick_scansegment_xd::sopasService() { return s_sopas_service; }

/*
 * @brief Initializes and runs all threads to receive, convert and publish scan data for the sick 3D lidar multiScan136.
 */
int sick_scansegment_xd::run(rosNodePtr node, const std::string& scannerName)
{
    // sick_scansegment_xd configuration
    setDiagnosticStatus(SICK_DIAGNOSTIC_STATUS::INIT, "sick_scan_xd initializing " + scannerName);
    sick_scansegment_xd::Config config;
    if (!config.Init(node))
    {
        ROS_ERROR_STREAM("## ERROR sick_scansegment_xd::run(" << config.scanner_type << "): Config::Init() failed, using default values.");
        return sick_scan_xd::ExitError;
    }
    config.PrintConfig();
    if (scannerName == SICK_SCANNER_SCANSEGMENT_XD_NAME)
    {
        std::vector<int> layer_elevation_table_mdeg = { 22710, 17560, 12480, 7510, 2490, 70, -2430, -7290, -12790, -17280, -21940, -26730, -31860, -34420, -37180, -42790 };
        sick_scansegment_xd::CompactDataParser::SetLayerElevationTable(layer_elevation_table_mdeg);
    }
    // Run sick_scansegment_xd (msgpack receive, convert and publish)
    ROS_INFO_STREAM("sick_scansegment_xd (" << config.scanner_type << ") started.");
    sick_scansegment_xd::MsgPackThreads msgpack_threads;
    if(!msgpack_threads.start(config))
    {
        ROS_ERROR_STREAM("## ERROR sick_scansegment_xd::run(" << config.scanner_type << "): sick_scansegment_xd::MsgPackThreads::start() failed");
        return sick_scan_xd::ExitError;
    }
    // std::cout << "sick_scansegment_xd::run(" << __LINE__ << "): sick_scansegment_xd thread started" << std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    msgpack_threads.join();
    // std::cout << "sick_scansegment_xd::run(" << __LINE__ << "): sick_scansegment_xd thread finished" << std::endl;
    // Close sick_scansegment_xd
    setDiagnosticStatus(SICK_DIAGNOSTIC_STATUS::EXIT, "sick_scan_xd exit");
    // std::cout << "sick_scansegment_xd::run() finishing" << std::endl;
    if(!msgpack_threads.stop(false))
    {
        ROS_ERROR_STREAM("## ERROR sick_scansegment_xd::run(" << config.scanner_type << "): sick_scansegment_xd::MsgPackThreads::stop() failed");
    }
    std::cout << "sick_scansegment_xd (" << config.scanner_type << ") finished." << std::endl;
    return sick_scan_xd::ExitSuccess;
}

/*
 * @brief MsgPackThreads constructor
 */
sick_scansegment_xd::MsgPackThreads::MsgPackThreads()
: m_scansegment_thread(0), m_run_scansegment_thread(false)
{
}

/*
 * @brief MsgPackThreads destructor
 */
sick_scansegment_xd::MsgPackThreads::~MsgPackThreads()
{
    stop(true);
}

/*
 * @brief Initializes msgpack receiver, converter and publisher and starts msgpack handling and publishing in a background thread.
 */
bool sick_scansegment_xd::MsgPackThreads::start(const sick_scansegment_xd::Config& config)
{
    m_config = config;
    m_run_scansegment_thread = true;
    m_scansegment_thread = new std::thread(&sick_scansegment_xd::MsgPackThreads::runThreadCb, this);
    return true;
}

/*
 * @brief Stops running threads and closes msgpack receiver, converter and publisher.
 */
bool sick_scansegment_xd::MsgPackThreads::stop(bool do_join)
{
    m_run_scansegment_thread = false;
    if(m_scansegment_thread)
    {
        if (do_join && m_scansegment_thread->joinable()) // std::thread::joinable() is false if std::thread::join finished successfull before
            m_scansegment_thread->join();
        delete m_scansegment_thread;
        m_scansegment_thread = 0;
    }
    return true;
}

/*
 * @brief Joins running threads and returns after they finished.
 */
void sick_scansegment_xd::MsgPackThreads::join(void)
{
    if(m_scansegment_thread && m_scansegment_thread->joinable())
    {
        m_scansegment_thread->join();
        std::cout << "sick_scansegment_xd::join(): sick_scansegment_xd thread finished" << std::endl;
    }
}

/*
 * @brief Thread callback, initializes and runs msgpack receiver, converter and publisher.
 */
bool sick_scansegment_xd::MsgPackThreads::runThreadCb(void)
{
    ROS_INFO_STREAM("sick_scansegment_xd::runThreadCb() start (" << __LINE__ << "," << (int)m_run_scansegment_thread << "," << (int)rosOk() << ")...");
    if(!m_config.logfolder.empty() && m_config.logfolder != ".")
    {
        sick_scansegment_xd::MkDir(m_config.logfolder);
    }

    // (Re-)initialize and run loop
    while(m_run_scansegment_thread && rosOk())
    {
        ROS_INFO_STREAM("sick_scansegment_xd initializing...");

        // Initialize udp receiver for scan data
        sick_scansegment_xd::UdpReceiver* udp_receiver = 0;
        while(m_run_scansegment_thread && rosOk() && udp_receiver == 0)
        {
            udp_receiver = new sick_scansegment_xd::UdpReceiver();
            if(udp_receiver->Init(m_config.udp_sender, m_config.udp_port, m_config.udp_input_fifolength, m_config.verbose_level > 1, m_config.export_udp_msg, m_config.scandataformat, 0))
            {
                ROS_INFO_STREAM("sick_scansegment_xd: udp socket to " << m_config.udp_sender << ":" << m_config.udp_port << " initialized");
            }
            else
            {
                ROS_ERROR_STREAM("## ERROR sick_scansegment_xd: UdpReceiver::Init(" << m_config.udp_sender << "," << m_config.udp_port << ") failed, retrying...");
                delete(udp_receiver);
                udp_receiver = 0;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }

        // Initialize udp receiver for imu data
        sick_scansegment_xd::UdpReceiver* udp_receiver_imu = 0;
        while(m_run_scansegment_thread && rosOk() && m_config.imu_enable && m_config.scandataformat == SCANDATA_COMPACT && udp_receiver_imu == 0)
        {
            udp_receiver_imu = new sick_scansegment_xd::UdpReceiver();
            if(udp_receiver_imu->Init(m_config.udp_sender, m_config.imu_udp_port, m_config.udp_input_fifolength, m_config.verbose_level > 1, m_config.export_udp_msg, m_config.scandataformat, udp_receiver->Fifo())) // udp receiver for scan and imu data share the same fifo
            {
                // m_config.imu_latency_microsec
                ROS_INFO_STREAM("sick_scansegment_xd: udp socket to " << m_config.udp_sender << ":" << m_config.imu_udp_port << " initialized");
            }
            else
            {
                ROS_ERROR_STREAM("## ERROR sick_scansegment_xd: UdpReceiver::Init(" << m_config.udp_sender << "," << m_config.imu_udp_port << ") failed, retrying...");
                delete(udp_receiver_imu);
                udp_receiver_imu = 0;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }

        // Initialize msgpack converter and connect to udp receiver
        ScanSegmentParserConfig scansegment_parser_config;
        scansegment_parser_config.imu_latency_microsec = m_config.imu_latency_microsec;
        sick_scansegment_xd::MsgPackConverter msgpack_converter(scansegment_parser_config, m_config.add_transform_xyz_rpy, udp_receiver->Fifo(), m_config.scandataformat, m_config.msgpack_output_fifolength, m_config.verbose_level > 1);
        assert(udp_receiver->Fifo());
        assert(msgpack_converter.Fifo());

        // Initialize msgpack exporter and publisher
        sick_scansegment_xd::MsgPackExporter msgpack_exporter(udp_receiver->Fifo(), msgpack_converter.Fifo(), m_config.logfolder, m_config.export_csv, m_config.verbose_level > 0, m_config.measure_timing);
        std::shared_ptr<sick_scansegment_xd::RosMsgpackPublisher> ros_msgpack_publisher = std::make_shared<sick_scansegment_xd::RosMsgpackPublisher>("sick_scansegment_xd", m_config);
        msgpack_exporter.AddExportListener(ros_msgpack_publisher->ExportListener());
        sick_scansegment_xd::MsgPackExportListenerIF* listener = ros_msgpack_publisher->ExportListener();

        // Run udp receiver, msgpack converter and msgpack exporter in background tasks
        if (msgpack_converter.Start() && udp_receiver->Start() && msgpack_exporter.Start())
        {
            ROS_INFO_STREAM("MsgPackThreads: Start msgpack converter, udp receiver and msgpack exporter, receiving from " << m_config.udp_sender << ":" << m_config.udp_port);
            if (udp_receiver_imu)
            {
                if (udp_receiver_imu->Start())
                    ROS_INFO_STREAM("MsgPackThreads: udp receiver for imu data started, receiving from " << m_config.udp_sender << ":" << m_config.imu_udp_port);
                else
                    ROS_ERROR_STREAM("## ERROR sick_scansegment_xd: UdpReceiver::Start() failed for imu data, not receiving imu udp packages from " << m_config.udp_sender << ":" << m_config.imu_udp_port);
            }
        }
        else
        {
            ROS_ERROR_STREAM("## ERROR sick_scansegment_xd: MsgPackConverter::Start(), UdpReceiver::Start() or MsgPackExporter::Start() failed, not receiving udp packages from " << m_config.udp_sender << ":" << m_config.udp_port);
        }

        // Start SOPAS services (ROS-1 or ROS-2 only)
        sick_scan_xd::SickScanCommonTcp* sopas_tcp = 0;
        sick_scan_xd::SickScanServices* sopas_service = 0;
        std::string scannerName = SICK_SCANNER_SCANSEGMENT_XD_NAME;
        sick_scan_xd::SickGenericParser parser = sick_scan_xd::SickGenericParser(scannerName);
        sick_scan_xd::ScannerBasicParam basic_param;
        basic_param.setScannerName(scannerName);
        bool multiscan_write_filtersettings = m_config.host_set_FREchoFilter || m_config.host_set_LFPangleRangeFilter || m_config.host_set_LFPlayerFilter;
        if (m_config.start_sopas_service || m_config.send_sopas_start_stop_cmd || m_config.host_read_filtersettings || multiscan_write_filtersettings)
        {
            ROS_INFO_STREAM("MsgPackThreads: initializing sopas tcp (" << m_config.hostname << ":" << m_config.sopas_tcp_port << ", timeout:" << (0.001*m_config.sopas_timeout_ms) << ", binary:" << m_config.sopas_cola_binary << ")");
            sopas_tcp = new sick_scan_xd::SickScanCommonTcp(m_config.hostname, m_config.sopas_tcp_port, m_config.sopas_timeout_ms, m_config.node, &parser, m_config.sopas_cola_binary ? 'B' : 'A');
            ROS_INFO_STREAM("MsgPackThreads: initializing device");
            sopas_tcp->init_device(); // sopas_tcp->init();
            sopas_tcp->setReadTimeOutInMs(m_config.sopas_timeout_ms);
            ROS_INFO_STREAM("MsgPackThreads: initializing services");
            sopas_service = new sick_scan_xd::SickScanServices(m_config.node, sopas_tcp, &basic_param);
            ROS_INFO_STREAM("MsgPackThreads: ros services initialized");
        }
        else
        {
            ROS_INFO_STREAM("MsgPackThreads: ros services not initialized");
        }

        // Send SOPAS commands to read or optionally write filter settings for (FREchoFilter, LFPangleRangeFilter, LFPlayerFilter)
        if ((m_config.host_read_filtersettings || multiscan_write_filtersettings) && sopas_tcp && sopas_service)
        {
            if (sopas_tcp->isConnected())
            {
                // Optionally send SOPAS commands to write filter settings
                if (multiscan_write_filtersettings)
                {
                    sopas_service->sendAuthorization();//(m_config.client_authorization_pw);
                    sopas_service->writeMultiScanFiltersettings((m_config.host_set_FREchoFilter ? m_config.host_FREchoFilter : -1), 
                        (m_config.host_set_LFPangleRangeFilter ? m_config.host_LFPangleRangeFilter : ""), 
                        (m_config.host_set_LFPlayerFilter ? m_config.host_LFPlayerFilter : ""),
                        (m_config.host_set_LFPintervalFilter ? m_config.host_LFPintervalFilter : ""),
                        m_config.scanner_type);
                }
                // Send SOPAS commands to read filter settings
                sopas_service->sendAuthorization();//(m_config.client_authorization_pw);
                if (sopas_service->queryMultiScanFiltersettings(m_config.host_FREchoFilter, m_config.host_LFPangleRangeFilter, m_config.host_LFPlayerFilter, m_config.msgpack_validator_filter_settings, m_config.scanner_type))
                {
                    // Overwrite configured LFPangleRangeFilter and LFPlayerFilter with settings received from lidar
                    bool angle_range_filter_enabled = ros_msgpack_publisher->initLFPangleRangeFilterSettings(m_config.host_LFPangleRangeFilter);
                    bool layer_filter_enabled = ros_msgpack_publisher->initLFPlayerFilterSettings(m_config.host_LFPlayerFilter);
                    float fullframe_azimuth_min_deg = 0, fullframe_azimuth_max_deg = 0, fullframe_elevation_min_deg = 0, fullframe_elevation_max_deg = 0;
                    ros_msgpack_publisher->GetFullframeAngleRanges(fullframe_azimuth_min_deg, fullframe_azimuth_max_deg, fullframe_elevation_min_deg, fullframe_elevation_max_deg);
                    if (angle_range_filter_enabled)
                        ROS_INFO_STREAM("expected azimuth range of fullframe scans: " << std::fixed << std::setprecision(3) << fullframe_azimuth_min_deg << " to " << fullframe_azimuth_max_deg << " deg");
                    if (layer_filter_enabled)
                        ROS_INFO_STREAM("expected elevation range of fullframe scans: " << std::fixed << std::setprecision(3) << fullframe_elevation_min_deg << " to " << fullframe_elevation_max_deg << " deg");
                }
            }
            else
            {
                ROS_WARN_STREAM("## ERROR sick_scansegment_xd: no sopas tcp connection, multiScan136 filter settings not queried or written");
            }
        }

        // Initialize msgpack validation
        // sick_scansegment_xd::MsgPackValidator msgpack_validator; // default validator expecting full range (all echos, -PI <= azimuth <= PI, -PI/2 <= elevation <= PI/2, all segments)
        sick_scansegment_xd::MsgPackValidator msgpack_validator = sick_scansegment_xd::MsgPackValidator(m_config.msgpack_validator_filter_settings.msgpack_validator_required_echos,
            m_config.msgpack_validator_filter_settings.msgpack_validator_azimuth_start, m_config.msgpack_validator_filter_settings.msgpack_validator_azimuth_end,
            m_config.msgpack_validator_filter_settings.msgpack_validator_elevation_start, m_config.msgpack_validator_filter_settings.msgpack_validator_elevation_end,
            m_config.msgpack_validator_valid_segments, m_config.msgpack_validator_filter_settings.msgpack_validator_layer_filter,
            m_config.msgpack_validator_verbose);
        msgpack_converter.SetValidator(msgpack_validator, m_config.msgpack_validator_enabled, m_config.msgpack_validator_discard_msgpacks_out_of_bounds, m_config.msgpack_validator_check_missing_scandata_interval);

        // Send SOPAS start command
        if(sopas_tcp && sopas_service && m_config.send_sopas_start_stop_cmd)
        {
            if (sopas_tcp->isConnected())
            {
                sopas_service->sendAuthorization();//(m_config.client_authorization_pw);
                if (!sopas_service->sendMultiScanStartCmd(m_config.udp_receiver_ip, m_config.udp_port, m_config.scanner_type, m_config.scandataformat, m_config.imu_enable, m_config.imu_udp_port, 
                    m_config.performanceprofilenumber, m_config.check_udp_receiver_ip, m_config.check_udp_receiver_port))
                {
                    ROS_ERROR_STREAM("## ERROR sick_scansegment_xd: failed to send startup sequence, receiving scan data may fail.");
                }
            }
            else
            {
                ROS_ERROR_STREAM("## ERROR sick_scansegment_xd: no sopas tcp connection, startup sequence not sent, receiving scan data may fail.");
            }
        }

        // Activate message parsing and publishing AFTER initialization completed
        msgpack_converter.SetActive(true);
        ros_msgpack_publisher->SetActive(true);

        // Run event loop and monitor tcp-connection and udp messages
        setDiagnosticStatus(SICK_DIAGNOSTIC_STATUS::OK, "");
        s_sopas_service = sopas_service;
        // Wait for first udp message with initial timeout after start in milliseconds, default: 60*1000
        fifo_timestamp fifo_timestamp_start = fifo_clock::now();
        while(m_run_scansegment_thread && rosOk() && sopas_tcp->isConnected() 
            && udp_receiver->Fifo()->TotalMessagesPushed() <= 1 
            && udp_receiver->Fifo()->Seconds(fifo_timestamp_start, fifo_clock::now()) <= 1.0e-3 * m_config.udp_timeout_ms_initial)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }    
        // Monitor udp packets with timeout for udp messages in milliseconds, default: 10*1000
        while(m_run_scansegment_thread && rosOk())
        {
            if (!sopas_tcp->isConnected())
            {
                ROS_ERROR_STREAM("## ERROR sick_scansegment_xd: sopas tcp connection lost, stop and reconnect...");
                break;
            }
            if (udp_receiver->Fifo()->TotalMessagesPushed() <= 1 || udp_receiver->Fifo()->SecondsSinceLastPush() > 1.0e-3 * m_config.udp_timeout_ms)
            {
                ROS_ERROR_STREAM("## ERROR sick_scansegment_xd: " << (udp_receiver->Fifo()->TotalMessagesPushed()) << " udp messages received");
                if (udp_receiver->Fifo()->TotalMessagesPushed() > 0)
                {
                    ROS_ERROR_STREAM("## ERROR sick_scansegment_xd: last message received " << (udp_receiver->Fifo()->SecondsSinceLastPush()) << " seconds ago.");
                }
                ROS_ERROR_STREAM("## ERROR sick_scansegment_xd: udp receive timeout, stop and reconnect...");
                break;
            }
            // Unit test for restart after timeout error - do not activate except for error and timeout simulation
            // if (udp_receiver->Fifo()->TotalMessagesPushed() > 1000)
            // {
            //     ROS_ERROR_STREAM("## ERROR sick_scansegment_xd: " << (udp_receiver->Fifo()->TotalMessagesPushed()) << " udp messages received, simulating timeout error, aborting to restart");
            //     break;
            // }
            setDiagnosticStatus(SICK_DIAGNOSTIC_STATUS::OK, "");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        s_sopas_service = 0;

        // Close msgpack receiver, converter and exporter
        setDiagnosticStatus(SICK_DIAGNOSTIC_STATUS::EXIT, "sick_scan_xd exit");
        ROS_INFO_STREAM("sick_scansegment_xd finishing (" << __LINE__ << ")");
        msgpack_exporter.RemoveExportListener(ros_msgpack_publisher->ExportListener());
        msgpack_exporter.Stop();
        if (udp_receiver_imu)
        {
            udp_receiver_imu->Close();
            DELETE_PTR(udp_receiver_imu);
        }
        udp_receiver->Stop();
        udp_receiver->Fifo()->Shutdown();
        ROS_INFO_STREAM("sick_scansegment_xd finishing (" << __LINE__ << ")");

        // Send stop command (sopas and/or rest-api)
        if(sopas_tcp && sopas_service && m_config.send_sopas_start_stop_cmd && sopas_tcp->isConnected())
        {
            std::cout << "sick_scansegment_xd exit: sending stop commands..." << std::endl;
            sopas_service->sendAuthorization();//(m_config.client_authorization_pw);
            sopas_service->sendMultiScanStopCmd(m_config.imu_enable);
            std::cout << "sick_scansegment_xd exit: stop commands sent." << std::endl;
        }
        // Stop SOPAS services
        std::cout << "sick_scansegment_xd exit: stopping services and communication (" << __LINE__ << ")" << std::endl;
        try
        {
            // Shutdown, cleanup and exit
            DELETE_PTR(sopas_service);
            DELETE_PTR(sopas_tcp);
            msgpack_converter.Fifo()->Shutdown();
            msgpack_exporter.Close();
            msgpack_converter.Close();
            udp_receiver->Close();
            DELETE_PTR(udp_receiver);
            std::cout << "sick_scansegment_xd exit: services and communication stopped (" << __LINE__ << ")" << std::endl;
        }
        catch(const std::exception& e)
        {
            std::cerr << "## ERROR sick_scansegment_xd exit: exception \"" << e.what() << "\"" << std::endl;
        }
    }

    ROS_INFO_STREAM("sick_scansegment_xd::runThreadCb() finished (" << __LINE__ << ")");
    return true;
}
