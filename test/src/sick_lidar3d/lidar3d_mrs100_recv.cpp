/*
 * @brief lidar3d_mrs100_recv implements a ROS node to receive and publish data from the new sick 3D lidar multiScan136.
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
#include "sick_lidar3d/udp_sockets.h"
#include "sick_lidar3d/config.h"
#include "sick_lidar3d/mrs100_curl.h"
#include "sick_lidar3d/msgpack_converter.h"
#include "sick_lidar3d/msgpack_exporter.h"
#include "sick_lidar3d/msgpack_parser.h"
#include "sick_lidar3d/msgpack_validator.h"
#include "sick_lidar3d/python_wrapper.h"
#include "sick_lidar3d/ros_msgpack_publisher.h"
#include "sick_lidar3d/udp_receiver.h"
#include "sick_scan/sick_scan_services.h"

#define DELETE_PTR(p) do{if(p){delete(p);(p)=0;}}while(false)

/*
 * @brief RunOffline runs lidar3d_mrs100_recv offline in filemode:
 * - Read binary msgpack data from file,
 * - unpack and convert data,
 * - optionally export to csv-file and
 * - measure time
 * @param[in] inputfolder_msgfiles input folder with msgpack files *.msg in offline mode (read from file, decode and optionally export to csv)
 * @param[in] input_msgfiles input msgpack files *.msg to read in offline mode
 * @param[in] verbose_level  verbose_level <= 0: quiet mode, verbose_level == 1: print statistics, verbose_level == 2: print details incl. msgpack data, default: 1
 * @param[in] measure_timing true: duration and latency of msgpack conversion and export is measured, default: true
 * @param[in] export_csv true: export msgpack data to csv file, default: false
 * @param[in] export_folder output folder for csv file, default: "."
 */
static void RunOffline(const std::string& inputfolder_msgfiles, const std::vector<std::string>& input_msgfiles, int verbose_level = 1, bool measure_timing = true, bool export_csv = false, const std::string& export_folder = ".")
{
    // Filemode: Read binary msgpack data from file
    std::vector<std::vector<uint8_t>> input_msgpacks;
    for (int msg_idx = 0; msg_idx < input_msgfiles.size(); msg_idx++)
    {
        // Read binary msgpack data from file
        std::vector<uint8_t> msgpack_data = sick_lidar3d::MsgPackParser::ReadFile(inputfolder_msgfiles + "/" + input_msgfiles[msg_idx]);
        if (!msgpack_data.empty())
            input_msgpacks.push_back(msgpack_data);
    }
    // Filemode: Unpack and convert data, export to csv-file and measure time
    if (!input_msgpacks.empty())
    {
        chrono_system_time timestamp_start = chrono_system_clock::now();
        double duration_seconds = 0;
        size_t execution_count = 0;
        do
        {
            for (int msg_idx = 0; msg_idx < input_msgpacks.size(); msg_idx++, execution_count += 1)
            {
                // Decode and parse the msgpack data
                sick_lidar3d::MsgPackParserOutput msgpack_output;
                sick_lidar3d::MsgPackValidatorData msgpack_validator_data_collector;
                if (!sick_lidar3d::MsgPackParser::Parse(input_msgpacks[msg_idx], fifo_clock::now(), msgpack_output, msgpack_validator_data_collector, sick_lidar3d::MsgPackValidator(), false, false, verbose_level > 1))
                    LIDAR3D_ERROR_STREAM("## ERROR MsgPackParser::Parse() failed on msg file \"" << input_msgfiles[msg_idx] << "\".");
                // Export to csv
                if (export_csv && !sick_lidar3d::MsgPackParser::WriteCSV({ msgpack_output }, export_folder + "/" + sick_lidar3d::FilenameNoPathNoExtension(input_msgfiles[msg_idx]) + ".csv", true))
                    LIDAR3D_ERROR_STREAM("## ERROR MsgPackParser::WriteCSV() failed.");
            }
            duration_seconds = sick_lidar3d::Seconds(timestamp_start, chrono_system_clock::now());
        } while (measure_timing && duration_seconds < 1);
        if (measure_timing && verbose_level > 0)
        {
            LIDAR3D_INFO_STREAM("lidar3d_mrs100_recv: " << execution_count << " msgpacks decoded, parsed and converted in " << duration_seconds << " seconds, " << (1000.0 * duration_seconds / execution_count) << " milliseconds per call.");
        }
    }
}

#include <msgpack11.hpp>
static bool RunOfflineMsgpackHex(const std::string& msgpack_hexfile)
{
    std::ifstream msgpack_fs(msgpack_hexfile);
    if (msgpack_fs.is_open())
    {
        try
        {
            // Read and convert hex string to byte array
            std::string msgpack_hex_str((std::istreambuf_iterator<char>(msgpack_fs)), std::istreambuf_iterator<char>());
            std::vector<uint8_t> msgpack_bin_data(msgpack_hex_str.size() / 2);
            for (int n = 0; n < msgpack_bin_data.size(); n++)
                msgpack_bin_data[n] = (uint8_t)(std::stoul(msgpack_hex_str.substr(2 * n, 2), nullptr, 16) & 0xFF);
            // Unpack the binary msgpack data
            std::string msg_parse_error;
            std::string msgpack_bin_str((char*)msgpack_bin_data.data(), msgpack_bin_data.size());
            // Run offline parser msgpack11::MsgPack
            std::istringstream msgpack_istream1(msgpack_bin_str);
            msgpack11::MsgPack msgpack_unpacked = msgpack11::MsgPack::parse(msgpack_istream1, msg_parse_error);
            LIDAR3D_INFO_STREAM ("RunOfflineMsgpackHex(" << msgpack_hexfile << "): msgpack11::MsgPack::parse() passed.");
            // Run offline parser sick_lidar3d::MsgPackParser
            std::istringstream msgpack_istream2(msgpack_bin_str);
            sick_lidar3d::MsgPackParserOutput msgpack_parser_result;
            sick_lidar3d::MsgPackValidatorData msgpack_validator_data_collector;
            fifo_timestamp timestamp;
            bool success = sick_lidar3d::MsgPackParser::Parse(msgpack_istream2, timestamp, msgpack_parser_result, msgpack_validator_data_collector, sick_lidar3d::MsgPackValidator(), false, false, false);
            LIDAR3D_INFO_STREAM ("RunOfflineMsgpackHex(" << msgpack_hexfile << "): sick_lidar3d::MsgPackParser::Parse() passed, success=" << success);
            return true;
        }
        catch (const std::exception& exc)
        {
            LIDAR3D_ERROR_STREAM("## ERROR msgpack11::MsgPack::parse(): exception " << exc.what());
        }
        catch (...)
        {
            LIDAR3D_ERROR_STREAM("## ERROR msgpack11::MsgPack::parse(): unknown exception ");
        }
    }
    return false;
}

// Send "start" trigger via UDP
static void sendStartTrigger(sick_lidar3d::Config& config)
{
  if (config.mrs100_send_udp_start)
  {
    sick_lidar3d::UdpSenderSocketImpl udp_sender(config.mrs100_ip, config.mrs100_port);
    std::vector<uint8_t> mrs100_send_udp_start_bytes(config.mrs100_send_udp_start_string.begin(), config.mrs100_send_udp_start_string.end());
    if (!udp_sender.IsOpen())
      LIDAR3D_ERROR_STREAM ("## ERROR lidar3d_mrs100_recv: could not open udp socket " << config.mrs100_ip << ":" << config.mrs100_port);
    else if (!udp_sender.Send(mrs100_send_udp_start_bytes))
      LIDAR3D_ERROR_STREAM ("## ERROR lidar3d_mrs100_recv: could not send start string \"" << config.mrs100_send_udp_start_string << "\" using udp socket " << config.mrs100_ip << ":" << config.mrs100_port);
    else
      LIDAR3D_INFO_STREAM ("lidar3d_mrs100_recv: start string sent on udp socket " << config.mrs100_ip << ":" << config.mrs100_port);
  }
}

/*
 * main runs lidar3d_mrs100_recv:
 * - Initialize udp receiver, msgpack converter and ros publisher,
 * - Run threads to receive, convert, export and publish msgpack data,
 * - Optionally save to csv-file and visualize multiScan136 data,
 * - Optionally read and convert msgpack files,
 * - Report cpu times and possible data lost.
 */
int main(int argc, char** argv)
{
    // Configuration
    sick_lidar3d::Config config;
#if defined __ROS_VERSION && __ROS_VERSION > 0
    config.visualize = false; // Visualization using python matplotlib must run in main thread and is disabled due to conflicts with running rclcpp::spin as the main thread. On ROS2 we have rviz for visualization!
#endif
    if (!config.Init(argc, argv))
        LIDAR3D_ERROR_STREAM("## ERROR lidar3d_mrs100_recv: Config::Init() failed, using default values.");
    LIDAR3D_INFO_STREAM("lidar3d_mrs100_recv started.");
    sick_lidar3d::MkDir(config.logfolder);
#ifdef USE_PYTHON
    if (config.visualize)
        sick_lidar3d::PythonWrapper::Init(argv[0]);
#endif

    // Offline filemode: Read binary msgpack data from file, unpack and convert data, optionally export to csv-file and measure time
    // RunOfflineMsgpackHex("msgpack-nominal-udp000001.msgpack.hex");
    RunOffline(config.inputfolder_msgfiles, config.input_msgfiles, config.verbose_level, config.measure_timing, config.export_csv, config.logfolder);

    // Initialize udp receiver
    sendStartTrigger(config);
    sick_lidar3d::UdpReceiver* udp_receiver = 0;
    while(udp_receiver == 0)
    {
      udp_receiver = new sick_lidar3d::UdpReceiver();
      if(udp_receiver->Init(config.udp_sender, config.udp_port, config.udp_input_fifolength, config.verbose_level > 1, config.export_udp_msg))
      {
        LIDAR3D_INFO_STREAM("lidar3d_mrs100_recv: udp socket to " << config.udp_sender << ":" << config.udp_port << " initialized");
      }
      else
      {
        LIDAR3D_ERROR_STREAM("## ERROR lidar3d_mrs100_recv: UdpReceiver::Init(" << config.udp_sender << "," << config.udp_port << ") failed, retrying...");
        delete(udp_receiver);
        udp_receiver = 0;
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }

    // Initialize msgpack converter and connect to udp receiver
    sick_lidar3d::MsgPackConverter msgpack_converter(udp_receiver->Fifo(), config.msgpack_output_fifolength, config.verbose_level > 1);
    assert(udp_receiver->Fifo());
    assert(msgpack_converter.Fifo());

    // Initialize msgpack exporter and publisher
    sick_lidar3d::MsgPackExporter msgpack_exporter(udp_receiver->Fifo(), msgpack_converter.Fifo(), config.logfolder, config.export_csv, config.verbose_level > 0, config.measure_timing, config.visualize);
#if defined __ROS_VERSION && __ROS_VERSION > 0
    std::shared_ptr<sick_lidar3d::RosMsgpackPublisher> ros_msgpack_publisher = std::make_shared<sick_lidar3d::RosMsgpackPublisher>("lidar3d_mrs100_recv", config, 1);
    msgpack_exporter.AddExportListener(ros_msgpack_publisher->ExportListener());
    sick_lidar3d::MsgPackExportListenerIF* listener = ros_msgpack_publisher->ExportListener();
#endif

    // Run udp receiver, msgpack converter and msgpack exporter in background tasks
    if (msgpack_converter.Start() && udp_receiver->Start() && msgpack_exporter.Start())
        LIDAR3D_INFO_STREAM("lidar3d_mrs100_recv: Start msgpack converter, udp receiver and msgpack exporter, receiving from " << config.udp_sender << ":" << config.udp_port);
    else
        LIDAR3D_ERROR_STREAM("## ERROR lidar3d_mrs100_recv: MsgPackConverter::Start(), UdpReceiver::Start() or MsgPackExporter::Start() failed, not receiving udp packages from " << config.udp_sender << ":" << config.udp_port);

    // Send "start" via REST-API
    if (config.mrs100_post_start_stop)
    {
        if(sick_lidar3d::MRS100CURL::postStart(config.mrs100_ip, config.mrs100_dst_ip, config.mrs100_port))
            LIDAR3D_INFO_STREAM("lidar3d_mrs100_recv: Start command posted");
        else
            LIDAR3D_ERROR_STREAM("## ERROR lidar3d_mrs100_recv: Failed to post start command");
    }

    // Send "start" via UDP
    sendStartTrigger(config);

    // Start SOPAS services (ROS-1 or ROS-2 only)
    sick_scan::SickScanCommonTcp* sopas_tcp = 0;
    sick_scan::SickScanServices* sopas_service = 0;
    std::string scannerName = SICK_SCANNER_LIDAR3D_MRS100_NAME;
    sick_scan::SickGenericParser parser = sick_scan::SickGenericParser(scannerName);
    sick_scan::ScannerBasicParam basic_param;
    basic_param.setScannerName(scannerName);
    bool mrs100_write_filtersettings = config.mrs100_set_FREchoFilter || config.mrs100_set_LFPangleRangeFilter || config.mrs100_set_LFPlayerFilter;
    if (config.start_sopas_service || config.send_sopas_mrs100_start_stop_cmd || config.mrs100_read_filtersettings || mrs100_write_filtersettings)
    {
        sopas_tcp = new sick_scan::SickScanCommonTcp(config.mrs100_ip, config.sopas_tcp_port, config.sopas_timeout_ms, config.node, &parser, config.sopas_cola_binary ? 'B' : 'A');
        sopas_tcp->init_device(); // sopas_tcp->init();
        sopas_tcp->setReadTimeOutInMs(config.sopas_timeout_ms);
        sopas_service = new sick_scan::SickScanServices(config.node, sopas_tcp, &basic_param);
        LIDAR3D_INFO_STREAM("SickScanServices: ros services initialized");
    }

    // Send SOPAS commands to read or optionally write filter settings for (FREchoFilter, LFPangleRangeFilter, LFPlayerFilter)
    if ((config.mrs100_read_filtersettings || mrs100_write_filtersettings) && sopas_tcp && sopas_service)
    {
        if (sopas_tcp->isConnected())
        {
            // Optionally send SOPAS commands to write filter settings
            if (mrs100_write_filtersettings)
            {
                sopas_service->sendAuthorization();//(config.client_authorization_pw);
                sopas_service->writeMRS100Filtersettings((config.mrs100_set_FREchoFilter ? config.mrs100_FREchoFilter : -1), (config.mrs100_set_LFPangleRangeFilter ? config.mrs100_LFPangleRangeFilter : ""), (config.mrs100_set_LFPlayerFilter ? config.mrs100_LFPlayerFilter : ""));
            }
            // Send SOPAS commands to read filter settings
            sopas_service->sendAuthorization();//(config.client_authorization_pw);
            sopas_service->queryMRS100Filtersettings(config.mrs100_FREchoFilter, config.mrs100_LFPangleRangeFilter, config.mrs100_LFPlayerFilter, config.msgpack_validator_filter_settings);
        }
        else
        {
            LIDAR3D_WARN_STREAM("## ERROR lidar3d_mrs100_recv: no sopas tcp connection, multiScan136 filter settings not queried or written");
        }
    }

    // Initialize msgpack validation
    // sick_lidar3d::MsgPackValidator msgpack_validator; // default validator expecting full range (all echos, -PI <= azimuth <= PI, -PI/2 <= elevation <= PI/2, all segments)
    sick_lidar3d::MsgPackValidator msgpack_validator = sick_lidar3d::MsgPackValidator(config.msgpack_validator_filter_settings.msgpack_validator_required_echos,
        config.msgpack_validator_filter_settings.msgpack_validator_azimuth_start, config.msgpack_validator_filter_settings.msgpack_validator_azimuth_end,
        config.msgpack_validator_filter_settings.msgpack_validator_elevation_start, config.msgpack_validator_filter_settings.msgpack_validator_elevation_end,
        config.msgpack_validator_valid_segments, config.msgpack_validator_filter_settings.msgpack_validator_layer_filter,
        config.msgpack_validator_verbose);
    msgpack_converter.SetValidator(msgpack_validator, config.msgpack_validator_enabled, config.msgpack_validator_discard_msgpacks_out_of_bounds, config.msgpack_validator_check_missing_scandata_interval);

#if defined __ROS_VERSION && __ROS_VERSION > 0
    ros_msgpack_publisher->SetFullScanAzimuthRange(config.msgpack_validator_filter_settings.msgpack_validator_azimuth_start, config.msgpack_validator_filter_settings.msgpack_validator_azimuth_end);
    ros_msgpack_publisher->SetActive(true);
#endif

    // Send SOPAS start command
    if(sopas_tcp && sopas_service && config.send_sopas_mrs100_start_stop_cmd)
    {
        if (sopas_tcp->isConnected())
        {
            sopas_service->sendAuthorization();//(config.client_authorization_pw);
            sopas_service->sendMRS100StartCmd(config.mrs100_dst_ip, config.mrs100_port);
        }
        else
        {
            LIDAR3D_WARN_STREAM("## ERROR lidar3d_mrs100_recv: no sopas tcp connection, multiScan136 start commands not sent.");
        }
    }

    // Run ros main loop with lidar msgpack publisher, see https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces/
#if defined __ROS_VERSION && __ROS_VERSION > 1
      rclcpp::spin(config.node);
      LIDAR3D_INFO_STREAM("lidar3d_mrs100_recv finishing, ros shutdown.");
    msgpack_exporter.RemoveExportListener(ros_msgpack_publisher->ExportListener());
#elif defined __ROS_VERSION && __ROS_VERSION > 0
    ros::spin();
      LIDAR3D_INFO_STREAM("lidar3d_mrs100_recv finishing, ros shutdown.");
    msgpack_exporter.RemoveExportListener(ros_msgpack_publisher->ExportListener());
#else // Run background task until ENTER key pressed
    if (config.exit_on_keys_esc_q)
        LIDAR3D_INFO_STREAM("lidar3d_mrs100_recv: running receive and export threads, press ESC, q or Q to quit...");
    while(true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        int c;
        if (config.exit_on_keys_esc_q && KBHIT() && ((c = GETCH()) == 27 || c == 'q' || c == 'Q'))
        {
            LIDAR3D_INFO_STREAM("lidar3d_mrs100_recv: key " << c << " pressed, aborting...");
            break;
        }
    }
#endif

    LIDAR3D_INFO_STREAM("lidar3d_mrs100_recv finishing.");

    // Send stop command (sopas and/or rest-api)
    if(sopas_tcp && sopas_service && config.send_sopas_mrs100_start_stop_cmd && sopas_tcp->isConnected())
    {
        sopas_service->sendAuthorization();//(config.client_authorization_pw);
        sopas_service->sendMRS100StopCmd();
    }
    if (config.mrs100_post_start_stop)
    {
        if (sick_lidar3d::MRS100CURL::postStop(config.mrs100_ip, config.mrs100_port))
            LIDAR3D_INFO_STREAM("lidar3d_mrs100_recv: Stop command posted");
        else
            LIDAR3D_ERROR_STREAM("## ERROR lidar3d_mrs100_recv: Failed to post start command");
    }
    // Stop SOPAS services
    DELETE_PTR(sopas_service);
    DELETE_PTR(sopas_tcp);

    // Shutdown, cleanup and exit
    rosShutdown();
    msgpack_converter.Fifo()->Shutdown();
    udp_receiver->Fifo()->Shutdown();
    msgpack_exporter.Close();
    msgpack_converter.Close();
    udp_receiver->Close();
    delete(udp_receiver);
    LIDAR3D_INFO_STREAM("lidar3d_mrs100_recv finished.");
    return 0;
}
