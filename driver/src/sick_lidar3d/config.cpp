/*
 * @brief config.cpp implements the configuration (yaml, commandline and default parameters) for project sick_lidar3d.
 *
 * Copyright (C) 2020,2021 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2020,2021 SICK AG, Waldkirch
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
 *
 */
#include <cstring>
#include "sick_lidar3d/config.h"

/** Lookup a table of optional key value pairs and overwrite argument if key found in table */
static bool setOptionalArgument(const std::map<std::string, std::string>& key_value_pairs, const std::string& key, std::string& argument)
{
    std::map<std::string, std::string>::const_iterator key_value_pair_iter = key_value_pairs.find(key);
    if (key_value_pair_iter != key_value_pairs.cend())
    {
        argument = key_value_pair_iter->second;
        LIDAR3D_INFO_STREAM(key << "=\"" << argument << "\" set by commandline");
        return true;
    }
    return false;
}

/** Lookup a table of optional key value pairs and overwrite argument if key found in table */
static bool setOptionalArgument(const std::map<std::string, std::string>& key_value_pairs, const std::string& key, bool& argument)
{
    std::string str_argument;
    if (setOptionalArgument(key_value_pairs, key, str_argument) && !str_argument.empty())
    {
        argument = (str_argument[0] == 'T' || str_argument[0] == 't' || std::stoi(str_argument) > 0);
        LIDAR3D_INFO_STREAM(key << "=" << (argument?"true":"false") << " set by commandline");
        return true;
    }
    return false;
}

/** Lookup a table of optional key value pairs and overwrite argument if key found in table */
static bool setOptionalArgument(const std::map<std::string, std::string>& key_value_pairs, const std::string& key, int& argument)
{
    std::string str_argument;
    if (setOptionalArgument(key_value_pairs, key, str_argument) && !str_argument.empty())
    {
        argument = std::stoi(str_argument);
        LIDAR3D_INFO_STREAM(key << "=" << argument << " set by commandline");
        return true;
    }
    return false;
}

/** Lookup a table of optional key value pairs and overwrite argument if key found in table */
static bool setOptionalArgument(const std::map<std::string, std::string>& key_value_pairs, const std::string& key, float& argument)
{
    std::string str_argument;
    if (setOptionalArgument(key_value_pairs, key, str_argument) && !str_argument.empty())
    {
        argument = std::stof(str_argument);
        LIDAR3D_INFO_STREAM(key << "=" << argument << " set by commandline");
        return true;
    }
    return false;
}

/*
 * @brief Default constructor, initializes the configuration with default values
 */
sick_lidar3d::Config::Config()
{
    node = 0;                          // Created by Config::Init()
    udp_sender = "";                   // Use "" (default) to receive msgpacks from any udp sender, use "127.0.0.1" to restrict to localhost (loopback device), or use the ip-address of a multiScan136 lidar or multiScan136 emulator
    udp_port = 2115;                   // default udp port for multiScan136 resp. multiScan136 emulator is 2115
    publish_topic = "/cloud";          // ros topic to publish received msgpack data converted top PointCloud2 messages, default: "/cloud"
    publish_topic_all_segments = "/cloud_360"; // ros topic to publish PointCloud2 messages of all segments (360 deg), default: "/cloud_360"
    segment_count = 12;                // number of expected segments in 360 degree, multiScan136: 12 segments, 30 degree per segment
    publish_frame_id = "world";        // frame id of ros PointCloud2 messages, default: "world"
    udp_input_fifolength = 20;         // max. udp input fifo length (-1: unlimited, default: 20 for buffering 1 second at 20 Hz), elements will be removed from front if number of elements exceeds the fifo_length
    msgpack_output_fifolength = 20;    // max. msgpack output fifo length (-1: unlimited, default: 20 for buffering 1 second at 20 Hz), elements will be removed from front if number of elements exceeds the fifo_length
    verbose_level = 1;                 // verbose_level <= 0: quiet mode, verbose_level == 1: print statistics, verbose_level == 2: print details incl. msgpack data, default: 1
    measure_timing = true;             // measure_timing == true: duration and latency of msgpack conversion and export is measured, default: true
    visualize = false; // true;        // visualize == true: plot lidarpoints via python wrapper, visualize == false: no diagrams, default: false
    export_csv = false; // true;       // export msgpack data to csv file, default: false
    export_udp_msg = false;            // true : export binary udp and msgpack data to file(*.udp and* .msg), default: false
    exit_on_keys_esc_q = true;         // exit lidar3d_mrs100_recv after pressing key ESC, 'q' or 'Q'
    logfolder = "./logfiles";          // output folder for logfiles, default: "."
    // msgpack-files 20201023 with old format valid until august 2021:
    // inputfolder_msgfiles = "../../sick_lidar3d_priv/python/polarscan_reader_test/MRS100_Emulator_20201023"; // Input folder with msgpack files *.msg in offline mode (read from file, decode and optionally export to csv)
    // input_msgfiles.clear();
    // input_msgfiles_templ = "udp_received_msg_%03d.msg";  // Input msgpack files * .msg to read in offline / test mode
    // inputfolder_msgfiles = "../../sick_lidar3d_priv/python/polarscan_reader_test/mrs100dataparser-1.1.1R-msgpacks"; // Input folder with msgpack files *.msg in offline mode (read from file, decode and optionally export to csv)
    // input_msgfiles = { "pcap_sensor_nonav_multiecho.msgpack", "pcap_sensor_nonav_singlecho.msgpack" };
    // msgpack-files 2021 with tokenized msgpack keys:
    inputfolder_msgfiles = "../../sick_lidar3d_priv/python/polarscan_reader_test/mrs100-tokenized-msgpacks"; // Input folder with msgpack files *.msg in offline mode (read from file, decode and optionally export to csv)
    input_msgfiles = { "20210929_mrs100_token.msgpack" };
    input_msgfiles_templ = "";
    mrs100_post_start_stop = false;      // Post start and stop commands to start/stop MRS using Rest-API
    mrs100_ip = "192.168.0.1";          // IP address of multiScan136 to post start and stop commands using Rest-API
    mrs100_dst_ip = "";                 // UDP destination IP address (host ip used in setIpAddress posted using Rest-API)
    mrs100_port = 2115;                 // UDP port of multiScan136 to post start and stop commands using Rest-API
    mrs100_send_udp_start = false;      // Send udp start string to multiScan136, default: True
    mrs100_send_udp_start_string = "magicalActivate"; // udp string to start multiScan136, default: "magicalActivate"

    // SOPAS default settings
    sopas_tcp_port = "2111";                 // TCP port for SOPAS commands, default port: 2111
    start_sopas_service = true;              // True: sopas services for CoLa-commands are started (ROS only), default: true
    send_sopas_mrs100_start_stop_cmd = true; // True: multiScan136 start and stop command sequence ("sWN ScanDataEnable 0/1" etc.) are sent after driver start and stop, default: true
    sopas_cola_binary = false;               // False: SOPAS uses CoLa-A (ascii, default, recommended), CoLa-B (true, binary) currently experimental
    sopas_timeout_ms = 5000;                 // Timeout for SOPAS response in milliseconds, default: 5000
    client_authorization_pw = "F4724744";    // Default password for client authorization

    // MSR100 default filter settings
    mrs100_read_filtersettings = true;                            // Read multiScan136 settings for FREchoFilter, LFPangleRangeFilter and LFPlayerFilter at startup, default: true
    mrs100_FREchoFilter = 1;                                      // Optionally set FREchoFilter with 0 for FIRST_ECHO (EchoCount=1), 1 for ALL_ECHOS (EchoCount=3), or 2 for LAST_ECHO (EchoCount=1)
    mrs100_set_FREchoFilter = false;                              // If true, FREchoFilter is set at startup (default: false)
    mrs100_LFPangleRangeFilter = "0 -180.0 +180.0 -90.0 +90.0 1"; // Optionally set LFPangleRangeFilter to "<enabled> <azimuth_start> <azimuth_stop> <elevation_start> <elevation_stop> <beam_increment>" with azimuth and elevation given in degree
    mrs100_set_LFPangleRangeFilter = false;                       // If true, LFPangleRangeFilter is set at startup (default: false)
    mrs100_LFPlayerFilter = "0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1";  // Optionally set LFPlayerFilter to "<enabled> <layer0-enabled> <layer1-enabled> <layer2-enabled> ... <layer15-enabled>" with 1 for enabled and 0 for disabled
    mrs100_set_LFPlayerFilter = false;                            // If true, LFPlayerFilter is set at startup (default: false)

    // msgpack validation default settings
    msgpack_validator_enabled = true; // true: check msgpack data for out of bounds and missing scan data, false: no msgpack validation
    msgpack_validator_verbose = 0;  // 0: print error messages, 1: print error and informational messages, 2: print error and all messages
    msgpack_validator_discard_msgpacks_out_of_bounds = true; // true: msgpacks are discarded if scan data out of bounds detected, false: error message if a msgpack is not validated
    msgpack_validator_check_missing_scandata_interval = 12; // 
    msgpack_validator_filter_settings.msgpack_validator_required_echos = { 0 };               // default: 1 echo, for all echos use { 0, 1, 2 }
    msgpack_validator_filter_settings.msgpack_validator_azimuth_start = (float)(-M_PI);       // default for full scan: -M_PI;
    msgpack_validator_filter_settings.msgpack_validator_azimuth_end = (float)(M_PI);          // default for full scan: +M_PI;
    msgpack_validator_filter_settings.msgpack_validator_elevation_start = (float)(-M_PI/2.0); // default for full scan: -M_PI/2.0;
    msgpack_validator_filter_settings.msgpack_validator_elevation_end = (float)(M_PI/2.0);    // default for full scan: +M_PI/2.0;
    msgpack_validator_filter_settings.msgpack_validator_layer_filter = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }; // default for full scan: 16 layer active, i.e. { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }
    msgpack_validator_valid_segments = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }; // default for full scan: 12 segments, i.e. { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }

}

/*
 * @brief Destructor
 */
sick_lidar3d::Config::~Config()
{
    #if defined __ROS_VERSION && __ROS_VERSION == 1
    if(node)
      delete(node);
    node = 0;
    #endif
}

/*
 * @brief Prints the commandline arguments.
 */
void sick_lidar3d::Config::PrintHelp(void)
{
    LIDAR3D_INFO_STREAM("lidar3d_mrs100_recv receives udp packets from multiScan136 or multiScan136 emulator, unpacks, converts and exports the lidar data.");
    LIDAR3D_INFO_STREAM("Commandline options are:");
    LIDAR3D_INFO_STREAM("-udp_sender=<ip> : ip address of udp sender, Use \"\" (default) to receive msgpacks from any udp sender, use \"127.0.0.1\" to restrict to localhost (loopback device), or use the ip-address of a multiScan136 lidar or multiScan136 emulator");
    LIDAR3D_INFO_STREAM("-udp_port=<port> : udp port for multiScan136 resp. multiScan136 emulator, default: " << udp_port);
    LIDAR3D_INFO_STREAM("-publish_topic=<topic> : ros topic to publish received msgpack data converted top PointCloud2 messages, default: /cloud");
    LIDAR3D_INFO_STREAM("-publish_frame_id=<id> : frame id of ros PointCloud2 messages, default: world");
    LIDAR3D_INFO_STREAM("-udp_input_fifolength=<size> : max. udp input fifo length (-1: unlimited, default: 20 for buffering 1 second at 20 Hz), elements will be removed from front if number of elements exceeds the fifo_length");
    LIDAR3D_INFO_STREAM("-msgpack_output_fifolength=<size> : max. msgpack output fifo length(-1: unlimited, default: 20 for buffering 1 second at 20 Hz), elements will be removed from front if number of elements exceeds the fifo_length");
    LIDAR3D_INFO_STREAM("-verbose_level=[0-2] : verbose_level <= 0: quiet mode, verbose_level == 1: print statistics, verbose_level == 2: print details incl. msgpack data, default: " << verbose_level);
    LIDAR3D_INFO_STREAM("-measure_timing=0|1 : measure_timing == true: duration and latency of msgpack conversion and export is measured, default: " << measure_timing);
    LIDAR3D_INFO_STREAM("-export_csv=0|1 : export msgpack data to csv file, default: false");
    LIDAR3D_INFO_STREAM("-export_udp_msg=0|1 : export binary udp and msgpack data to file(*.udp and* .msg), default: false");
    LIDAR3D_INFO_STREAM("-logfolder=<directory> : output folder for logfiles" );
    LIDAR3D_INFO_STREAM("-inputfolder_msgfiles=<directory> :  input folder with msgpack files *.msg in offline mode (read from file, decode and optionally export to csv), default: " << inputfolder_msgfiles);
    LIDAR3D_INFO_STREAM("-input_msgfiles=<filetemplate> : input msgpack files * .msg to read in offline / test mode, default:" << input_msgfiles_templ);
    LIDAR3D_INFO_STREAM("-mrs100_post_start_stop=0|1 : post start and stop commands to start/stop multiScan136 using Rest-API, default:" << mrs100_post_start_stop);
    LIDAR3D_INFO_STREAM("-mrs100_ip=<ip-address> : ip address of multiScan136 to post start and stop commands using Rest-API, default:" << mrs100_ip);
    LIDAR3D_INFO_STREAM("-mrs100_dst_ip=<ip-address> : UDP destination IP address (ip address of udp receiver), default:\"" << mrs100_dst_ip << "\"");
    LIDAR3D_INFO_STREAM("-mrs100_port=<port> :  udp port of multiScan136 to post start and stop commands using Rest-API, default:" << mrs100_port);
    LIDAR3D_INFO_STREAM("-mrs100_send_udp_start=0|1 : send udp start string to multiScan136, default:" << mrs100_send_udp_start);
    LIDAR3D_INFO_STREAM("-mrs100_send_udp_start_string=<string> : udp string to start multiScan136, default: \"magicalActivate\"" << mrs100_send_udp_start_string);
}

/*
 * @brief Initializes sick_lidar3d configuration by commandline arguments and yaml-file.
 */
bool sick_lidar3d::Config::Init(int argc, char** argv)
{
    // Read yaml configuration
    #if defined __ROS_VERSION && __ROS_VERSION > 1

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node = rclcpp::Node::make_shared("sick_lidar3d", "", node_options);
    rclcpp::Logger node_logger = node->get_logger();

    node->declare_parameter<std::string>("udp_sender", udp_sender);
    node->get_parameter("udp_sender", udp_sender);

    node->declare_parameter<int>("udp_port", udp_port);
    node->get_parameter("udp_port", udp_port);

    node->declare_parameter<std::string>("publish_topic", publish_topic);
    node->get_parameter("publish_topic", publish_topic);

    node->declare_parameter<std::string>("publish_topic_all_segments", publish_topic_all_segments);
    node->get_parameter("publish_topic_all_segments", publish_topic_all_segments);

    node->declare_parameter<int>("segment_count", segment_count);
    node->get_parameter("segment_count", segment_count);

    node->declare_parameter<std::string>("publish_frame_id", publish_frame_id);
    node->get_parameter("publish_frame_id", publish_frame_id);

    node->declare_parameter<int>("udp_input_fifolength", udp_input_fifolength);
    node->get_parameter("udp_input_fifolength", udp_input_fifolength);

    node->declare_parameter<int>("msgpack_output_fifolength", msgpack_output_fifolength);
    node->get_parameter("msgpack_output_fifolength", msgpack_output_fifolength);

    node->declare_parameter<int>("verbose_level", verbose_level);
    node->get_parameter("verbose_level", verbose_level);

    node->declare_parameter<bool>("measure_timing", measure_timing);
    node->get_parameter("measure_timing", measure_timing);

    node->declare_parameter<bool>("visualize", visualize);
    node->get_parameter("visualize", visualize);

    node->declare_parameter<bool>("export_csv", export_csv);
    node->get_parameter("export_csv", export_csv);

    node->declare_parameter<bool>("export_udp_msg", export_udp_msg);
    node->get_parameter("export_udp_msg", export_udp_msg);

    node->declare_parameter<bool>("exit_on_keys_esc_q", exit_on_keys_esc_q);
    node->get_parameter("exit_on_keys_esc_q", exit_on_keys_esc_q);

    node->declare_parameter<std::string>("logfolder", logfolder);
    node->get_parameter("logfolder", logfolder);

    node->declare_parameter<std::string>("inputfolder_msgfiles", inputfolder_msgfiles);
    node->get_parameter("inputfolder_msgfiles", inputfolder_msgfiles);

    node->declare_parameter<std::string>("input_msgfiles", input_msgfiles_templ);
    node->get_parameter("input_msgfiles", input_msgfiles_templ);

    node->declare_parameter<bool>("mrs100_post_start_stop", mrs100_post_start_stop);
    node->get_parameter("mrs100_post_start_stop", mrs100_post_start_stop);

    node->declare_parameter<std::string>("mrs100_ip", mrs100_ip);
    node->get_parameter("mrs100_ip", mrs100_ip);

    node->declare_parameter<std::string>("mrs100_dst_ip", mrs100_dst_ip);
    node->get_parameter("mrs100_dst_ip", mrs100_dst_ip);

    node->declare_parameter<int>("mrs100_port", mrs100_port);
    node->get_parameter("mrs100_port", mrs100_port);

    node->declare_parameter<bool>("mrs100_send_udp_start", mrs100_send_udp_start);
    node->get_parameter("mrs100_send_udp_start", mrs100_send_udp_start);

    node->declare_parameter<std::string>("mrs100_send_udp_start_string", mrs100_send_udp_start_string);
    node->get_parameter("mrs100_send_udp_start_string", mrs100_send_udp_start_string);

    node->declare_parameter<std::string>("sopas_tcp_port", sopas_tcp_port);
    node->get_parameter("sopas_tcp_port", sopas_tcp_port);

    node->declare_parameter<bool>("start_sopas_service", start_sopas_service);
    node->get_parameter("start_sopas_service", start_sopas_service);

    node->declare_parameter<bool>("send_sopas_mrs100_start_stop_cmd", send_sopas_mrs100_start_stop_cmd);
    node->get_parameter("send_sopas_mrs100_start_stop_cmd", send_sopas_mrs100_start_stop_cmd);

    node->declare_parameter<bool>("sopas_cola_binary", sopas_cola_binary);
    node->get_parameter("sopas_cola_binary", sopas_cola_binary);

    node->declare_parameter<int>("sopas_timeout_ms", sopas_timeout_ms);
    node->get_parameter("sopas_timeout_ms", sopas_timeout_ms);

    node->declare_parameter<std::string>("client_authorization_pw", client_authorization_pw);
    node->get_parameter("client_authorization_pw", client_authorization_pw);

    node->declare_parameter<bool>("mrs100_read_filtersettings", mrs100_read_filtersettings);
    node->get_parameter("mrs100_read_filtersettings", mrs100_read_filtersettings);

    node->declare_parameter<int>("mrs100_FREchoFilter", mrs100_FREchoFilter);
    node->get_parameter("mrs100_FREchoFilter", mrs100_FREchoFilter);

    node->declare_parameter<bool>("mrs100_set_FREchoFilter", mrs100_set_FREchoFilter);
    node->get_parameter("mrs100_set_FREchoFilter", mrs100_set_FREchoFilter);

    node->declare_parameter<std::string>("mrs100_LFPangleRangeFilter", mrs100_LFPangleRangeFilter);
    node->get_parameter("mrs100_LFPangleRangeFilter", mrs100_LFPangleRangeFilter);

    node->declare_parameter<bool>("mrs100_set_LFPangleRangeFilter", mrs100_set_LFPangleRangeFilter);
    node->get_parameter("mrs100_set_LFPangleRangeFilter", mrs100_set_LFPangleRangeFilter);

    node->declare_parameter<std::string>("mrs100_LFPlayerFilter", mrs100_LFPlayerFilter);
    node->get_parameter("mrs100_LFPlayerFilter", mrs100_LFPlayerFilter);

    node->declare_parameter<bool>("mrs100_set_LFPlayerFilter", mrs100_set_LFPlayerFilter);
    node->get_parameter("mrs100_set_LFPlayerFilter", mrs100_set_LFPlayerFilter);

    // msgpack validation settings
    std::string str_msgpack_validator_required_echos = "0";
    std::string str_msgpack_validator_valid_segments = "0 1 2 3 4 5 6 7 8 9 10 11";
    std::string str_msgpack_validator_layer_filter = "1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1";
    float msgpack_validator_azimuth_start_deg = msgpack_validator_filter_settings.msgpack_validator_azimuth_start * (float)(180.0 / M_PI);
    float msgpack_validator_azimuth_end_deg = msgpack_validator_filter_settings.msgpack_validator_azimuth_end * (float)(180.0 / M_PI);
    float msgpack_validator_elevation_start_deg = msgpack_validator_filter_settings.msgpack_validator_elevation_start * (float)(180.0 / M_PI);
    float msgpack_validator_elevation_end_deg = msgpack_validator_filter_settings.msgpack_validator_elevation_end * (float)(180.0 / M_PI);
    node->declare_parameter<bool>("msgpack_validator_enabled", msgpack_validator_enabled);
    node->get_parameter("msgpack_validator_enabled", msgpack_validator_enabled);
    node->declare_parameter<int>("msgpack_validator_verbose", msgpack_validator_verbose);
    node->get_parameter("msgpack_validator_verbose", msgpack_validator_verbose);
    node->declare_parameter<bool>("msgpack_validator_discard_msgpacks_out_of_bounds", msgpack_validator_discard_msgpacks_out_of_bounds);
    node->get_parameter("msgpack_validator_discard_msgpacks_out_of_bounds", msgpack_validator_discard_msgpacks_out_of_bounds);
    node->declare_parameter<int>("msgpack_validator_check_missing_scandata_interval", msgpack_validator_check_missing_scandata_interval);
    node->get_parameter("msgpack_validator_check_missing_scandata_interval", msgpack_validator_check_missing_scandata_interval);
    node->declare_parameter<std::string>("msgpack_validator_required_echos", str_msgpack_validator_required_echos);
    node->get_parameter("msgpack_validator_required_echos", str_msgpack_validator_required_echos);
    sick_lidar3d::util::parseVector(str_msgpack_validator_required_echos, msgpack_validator_filter_settings.msgpack_validator_required_echos);
    node->declare_parameter<float>("msgpack_validator_azimuth_start", msgpack_validator_azimuth_start_deg);
    node->get_parameter("msgpack_validator_azimuth_start", msgpack_validator_azimuth_start_deg);
    node->declare_parameter<float>("msgpack_validator_azimuth_end", msgpack_validator_azimuth_end_deg);
    node->get_parameter("msgpack_validator_azimuth_end", msgpack_validator_azimuth_end_deg);
    node->declare_parameter<float>("msgpack_validator_elevation_start", msgpack_validator_elevation_start_deg);
    node->get_parameter("msgpack_validator_elevation_start", msgpack_validator_elevation_start_deg);
    node->declare_parameter<float>("msgpack_validator_elevation_end", msgpack_validator_elevation_end_deg);
    node->get_parameter("msgpack_validator_elevation_end", msgpack_validator_elevation_end_deg);
    msgpack_validator_filter_settings.msgpack_validator_azimuth_start = msgpack_validator_azimuth_start_deg * (float)(M_PI / 180);
    msgpack_validator_filter_settings.msgpack_validator_azimuth_end = msgpack_validator_azimuth_end_deg * (float)(M_PI / 180);
    msgpack_validator_filter_settings.msgpack_validator_elevation_start = msgpack_validator_elevation_start_deg * (float)(M_PI / 180);
    msgpack_validator_filter_settings.msgpack_validator_elevation_end = msgpack_validator_elevation_end_deg * (float)(M_PI / 180);
    node->declare_parameter<std::string>("msgpack_validator_valid_segments", str_msgpack_validator_valid_segments);
    node->get_parameter("msgpack_validator_valid_segments", str_msgpack_validator_valid_segments);
    node->declare_parameter<std::string>("msgpack_validator_layer_filter", str_msgpack_validator_layer_filter);
    node->get_parameter("msgpack_validator_layer_filter", str_msgpack_validator_layer_filter);
    sick_lidar3d::util::parseVector(str_msgpack_validator_valid_segments, msgpack_validator_valid_segments);
    sick_lidar3d::util::parseVector(str_msgpack_validator_layer_filter, msgpack_validator_filter_settings.msgpack_validator_layer_filter);

    #elif defined __ROS_VERSION && __ROS_VERSION > 0

    ros::init(argc, argv, "sick_lidar3d");
    node = new ros::NodeHandle();
    ros::param::param<std::string>("/sick_lidar3d/ros__parameters/udp_sender", udp_sender, udp_sender);
    ros::param::param<int>("/sick_lidar3d/ros__parameters/udp_port", udp_port, udp_port);
    ros::param::param<std::string>("/sick_lidar3d/ros__parameters/publish_topic", publish_topic, publish_topic);
    ros::param::param<std::string>("/sick_lidar3d/ros__parameters/publish_topic_all_segments", publish_topic_all_segments, publish_topic_all_segments);
    ros::param::param<int>("/sick_lidar3d/ros__parameters/segment_count", segment_count, segment_count);
    ros::param::param<std::string>("/sick_lidar3d/ros__parameters/publish_frame_id", publish_frame_id, publish_frame_id);
    ros::param::param<int>("/sick_lidar3d/ros__parameters/udp_input_fifolength", udp_input_fifolength, udp_input_fifolength);
    ros::param::param<int>("/sick_lidar3d/ros__parameters/msgpack_output_fifolength", msgpack_output_fifolength, msgpack_output_fifolength);
    ros::param::param<int>("/sick_lidar3d/ros__parameters/verbose_level", verbose_level, verbose_level);
    ros::param::param<bool>("/sick_lidar3d/ros__parameters/measure_timing", measure_timing, measure_timing);
    ros::param::param<bool>("/sick_lidar3d/ros__parameters/visualize", visualize, visualize);
    ros::param::param<bool>("/sick_lidar3d/ros__parameters/export_csv", export_csv, export_csv);
    ros::param::param<bool>("/sick_lidar3d/ros__parameters/export_udp_msg", export_udp_msg, export_udp_msg);
    ros::param::param<bool>("/sick_lidar3d/ros__parameters/exit_on_keys_esc_q", exit_on_keys_esc_q, exit_on_keys_esc_q);
    ros::param::param<std::string>("/sick_lidar3d/ros__parameters/logfolder", logfolder, logfolder);
    ros::param::param<std::string>("/sick_lidar3d/ros__parameters/inputfolder_msgfiles", inputfolder_msgfiles, inputfolder_msgfiles);
    ros::param::param<std::string>("/sick_lidar3d/ros__parameters/input_msgfiles", input_msgfiles_templ, input_msgfiles_templ);
    ros::param::param<bool>("/sick_lidar3d/ros__parameters/mrs100_post_start_stop", mrs100_post_start_stop, mrs100_post_start_stop);
    ros::param::param<std::string>("/sick_lidar3d/ros__parameters/mrs100_ip", mrs100_ip, mrs100_ip);
    ros::param::param<std::string>("/sick_lidar3d/ros__parameters/mrs100_dst_ip", mrs100_dst_ip, mrs100_dst_ip);
    ros::param::param<int>("/sick_lidar3d/ros__parameters/mrs100_port", mrs100_port, mrs100_port);
    ros::param::param<bool>("/sick_lidar3d/ros__parameters/mrs100_send_udp_start", mrs100_send_udp_start, mrs100_send_udp_start);
    ros::param::param<std::string>("/sick_lidar3d/ros__parameters/mrs100_send_udp_start_string", mrs100_send_udp_start_string, mrs100_send_udp_start_string);
    ros::param::param<std::string>("/sick_lidar3d/ros__parameters/sopas_tcp_port", sopas_tcp_port, sopas_tcp_port);
    ros::param::param<bool>("/sick_lidar3d/ros__parameters/start_sopas_service", start_sopas_service, start_sopas_service);
    ros::param::param<bool>("/sick_lidar3d/ros__parameters/send_sopas_mrs100_start_stop_cmd", send_sopas_mrs100_start_stop_cmd, send_sopas_mrs100_start_stop_cmd);
    ros::param::param<bool>("/sick_lidar3d/ros__parameters/sopas_cola_binary", sopas_cola_binary, sopas_cola_binary);
    ros::param::param<int>("/sick_lidar3d/ros__parameters/sopas_timeout_ms", sopas_timeout_ms, sopas_timeout_ms);
    ros::param::param<std::string>("/sick_lidar3d/ros__parameters/client_authorization_pw", client_authorization_pw, client_authorization_pw);
    // MSR100 filter settings
    ros::param::param<bool>("/sick_lidar3d/ros__parameters/mrs100_read_filtersettings", mrs100_read_filtersettings, mrs100_read_filtersettings);
    ros::param::param<int>("/sick_lidar3d/ros__parameters/mrs100_FREchoFilter", mrs100_FREchoFilter, mrs100_FREchoFilter);
    ros::param::param<bool>("/sick_lidar3d/ros__parameters/mrs100_set_FREchoFilter", mrs100_set_FREchoFilter, mrs100_set_FREchoFilter);
    ros::param::param<std::string>("/sick_lidar3d/ros__parameters/mrs100_LFPangleRangeFilter", mrs100_LFPangleRangeFilter, mrs100_LFPangleRangeFilter);
    ros::param::param<bool>("/sick_lidar3d/ros__parameters/mrs100_set_LFPangleRangeFilter", mrs100_set_LFPangleRangeFilter, mrs100_set_LFPangleRangeFilter);
    ros::param::param<std::string>("/sick_lidar3d/ros__parameters/mrs100_LFPlayerFilter", mrs100_LFPlayerFilter, mrs100_LFPlayerFilter);
    ros::param::param<bool>("/sick_lidar3d/ros__parameters/mrs100_set_LFPlayerFilter", mrs100_set_LFPlayerFilter, mrs100_set_LFPlayerFilter);
    // msgpack validation settings
    std::string str_msgpack_validator_required_echos = "0";
    std::string str_msgpack_validator_valid_segments = "0 1 2 3 4 5 6 7 8 9 10 11";
    std::string str_msgpack_validator_layer_filter = "1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1";
    float msgpack_validator_azimuth_start_deg = msgpack_validator_filter_settings.msgpack_validator_azimuth_start * 180.0 / M_PI;
    float msgpack_validator_azimuth_end_deg = msgpack_validator_filter_settings.msgpack_validator_azimuth_end * 180.0 / M_PI;
    float msgpack_validator_elevation_start_deg = msgpack_validator_filter_settings.msgpack_validator_elevation_start * 180.0 / M_PI;
    float msgpack_validator_elevation_end_deg = msgpack_validator_filter_settings.msgpack_validator_elevation_end * 180.0 / M_PI;
    ros::param::param<bool>("/sick_lidar3d/ros__parameters/msgpack_validator_enabled", msgpack_validator_enabled, msgpack_validator_enabled);
    ros::param::param<int>("/sick_lidar3d/ros__parameters/msgpack_validator_verbose", msgpack_validator_verbose, msgpack_validator_verbose);
    ros::param::param<bool>("/sick_lidar3d/ros__parameters/msgpack_validator_discard_msgpacks_out_of_bounds", msgpack_validator_discard_msgpacks_out_of_bounds, msgpack_validator_discard_msgpacks_out_of_bounds);
    ros::param::param<int>("/sick_lidar3d/ros__parameters/msgpack_validator_check_missing_scandata_interval", msgpack_validator_check_missing_scandata_interval, msgpack_validator_check_missing_scandata_interval);
    ros::param::param<std::string>("/sick_lidar3d/ros__parameters/msgpack_validator_required_echos", str_msgpack_validator_required_echos, str_msgpack_validator_required_echos);
    sick_lidar3d::util::parseVector(str_msgpack_validator_required_echos, msgpack_validator_filter_settings.msgpack_validator_required_echos);
    ros::param::param<float>("/sick_lidar3d/ros__parameters/msgpack_validator_azimuth_start", msgpack_validator_azimuth_start_deg, msgpack_validator_azimuth_start_deg);
    ros::param::param<float>("/sick_lidar3d/ros__parameters/msgpack_validator_azimuth_end", msgpack_validator_azimuth_end_deg, msgpack_validator_azimuth_end_deg);
    ros::param::param<float>("/sick_lidar3d/ros__parameters/msgpack_validator_elevation_start", msgpack_validator_elevation_start_deg, msgpack_validator_elevation_start_deg);
    ros::param::param<float>("/sick_lidar3d/ros__parameters/msgpack_validator_elevation_end", msgpack_validator_elevation_end_deg, msgpack_validator_elevation_end_deg);
    msgpack_validator_filter_settings.msgpack_validator_azimuth_start = msgpack_validator_azimuth_start_deg * M_PI / 180;
    msgpack_validator_filter_settings.msgpack_validator_azimuth_end = msgpack_validator_azimuth_end_deg * M_PI / 180;
    msgpack_validator_filter_settings.msgpack_validator_elevation_start = msgpack_validator_elevation_start_deg * M_PI / 180;
    msgpack_validator_filter_settings.msgpack_validator_elevation_end = msgpack_validator_elevation_end_deg * M_PI / 180;
    ros::param::param<std::string>("/sick_lidar3d/ros__parameters/msgpack_validator_valid_segments", str_msgpack_validator_valid_segments, str_msgpack_validator_valid_segments);
    ros::param::param<std::string>("/sick_lidar3d/ros__parameters/msgpack_validator_layer_filter", str_msgpack_validator_layer_filter, str_msgpack_validator_layer_filter);
    sick_lidar3d::util::parseVector(str_msgpack_validator_valid_segments, msgpack_validator_valid_segments);
    sick_lidar3d::util::parseVector(str_msgpack_validator_layer_filter, msgpack_validator_filter_settings.msgpack_validator_layer_filter);

    #endif

    // Parse commandline arguments
    std::map<std::string, std::string> cli_parameter_map;
    for (int n = 1; n < argc; n++)
    {
        std::stringstream cli_argument(argv[n]);
        std::string cli_token;
        std::vector<std::string> cli_tokens;
        // split argument "-key=value" into 2 tokens
        while (getline(cli_argument, cli_token, '='))
        {
            cli_tokens.push_back(cli_token);
        }
        if (cli_tokens.size() == 2 && !cli_tokens[0].empty() && !cli_tokens[1].empty())
        {
            // remove leading '-'  for all "-key=value" arguments
            if (cli_tokens[0][0] == '-') 
                cli_tokens[0] = cli_tokens[0].substr(1);
            // remove trailing ':' for all "key:=value" arguments
            if (cli_tokens[0][(cli_tokens[0].length() - 1)] == ':') 
                cli_tokens[0] = cli_tokens[0].substr(0, cli_tokens[0].length() - 1);
            // append key-value-pair to map
            cli_parameter_map[cli_tokens[0]] = cli_tokens[1];
        }
    }
    for(std::map<std::string, std::string>::const_iterator iter = cli_parameter_map.cbegin(); iter != cli_parameter_map.cend(); iter++)
        LIDAR3D_INFO_STREAM("commandline argument found: \"" << iter->first << "\"=\"" << iter->second << "\"");

    // Overwrite with commandline arguments
    setOptionalArgument(cli_parameter_map, "udp_sender", udp_sender);
    setOptionalArgument(cli_parameter_map, "udp_port", udp_port);
    setOptionalArgument(cli_parameter_map, "publish_topic", publish_topic);
    setOptionalArgument(cli_parameter_map, "publish_topic_all_segments", publish_topic_all_segments);
    setOptionalArgument(cli_parameter_map, "segment_count", segment_count);
    setOptionalArgument(cli_parameter_map, "publish_frame_id", publish_frame_id);
    setOptionalArgument(cli_parameter_map, "udp_input_fifolength", udp_input_fifolength);
    setOptionalArgument(cli_parameter_map, "msgpack_output_fifolength", msgpack_output_fifolength);
    setOptionalArgument(cli_parameter_map, "verbose_level", verbose_level);
    setOptionalArgument(cli_parameter_map, "measure_timing", measure_timing);
    setOptionalArgument(cli_parameter_map, "visualize", visualize);
    setOptionalArgument(cli_parameter_map, "export_csv", export_csv);
    setOptionalArgument(cli_parameter_map, "export_udp_msg", export_udp_msg);
    setOptionalArgument(cli_parameter_map, "exit_on_keys_esc_q", exit_on_keys_esc_q);
    setOptionalArgument(cli_parameter_map, "logfolder", logfolder);
    setOptionalArgument(cli_parameter_map, "inputfolder_msgfiles", inputfolder_msgfiles);
    setOptionalArgument(cli_parameter_map, "input_msgfiles", input_msgfiles_templ);
    setOptionalArgument(cli_parameter_map, "mrs100_post_start_stop", mrs100_post_start_stop);
    setOptionalArgument(cli_parameter_map, "mrs100_ip", mrs100_ip);
    setOptionalArgument(cli_parameter_map, "mrs100_dst_ip", mrs100_dst_ip);
    setOptionalArgument(cli_parameter_map, "mrs100_port", mrs100_port);
    setOptionalArgument(cli_parameter_map, "mrs100_send_udp_start", mrs100_send_udp_start);;
    setOptionalArgument(cli_parameter_map, "mrs100_send_udp_start_string", mrs100_send_udp_start_string);
    setOptionalArgument(cli_parameter_map, "sopas_tcp_port", sopas_tcp_port);
    setOptionalArgument(cli_parameter_map, "start_sopas_service", start_sopas_service);
    setOptionalArgument(cli_parameter_map, "send_sopas_mrs100_start_stop_cmd", send_sopas_mrs100_start_stop_cmd);
    setOptionalArgument(cli_parameter_map, "sopas_cola_binary", sopas_cola_binary);
    setOptionalArgument(cli_parameter_map, "sopas_timeout_ms", sopas_timeout_ms);
    setOptionalArgument(cli_parameter_map, "client_authorization_pw", client_authorization_pw);
    setOptionalArgument(cli_parameter_map, "mrs100_read_filtersettings", mrs100_read_filtersettings);
    setOptionalArgument(cli_parameter_map, "mrs100_FREchoFilter", mrs100_FREchoFilter);
    setOptionalArgument(cli_parameter_map, "mrs100_set_FREchoFilter", mrs100_set_FREchoFilter);
    setOptionalArgument(cli_parameter_map, "mrs100_LFPangleRangeFilter", mrs100_LFPangleRangeFilter);
    setOptionalArgument(cli_parameter_map, "mrs100_set_LFPangleRangeFilter", mrs100_set_LFPangleRangeFilter);
    setOptionalArgument(cli_parameter_map, "mrs100_LFPlayerFilter", mrs100_LFPlayerFilter);
    setOptionalArgument(cli_parameter_map, "mrs100_set_LFPlayerFilter", mrs100_set_LFPlayerFilter);
    setOptionalArgument(cli_parameter_map, "msgpack_validator_enabled", msgpack_validator_enabled);
    setOptionalArgument(cli_parameter_map, "msgpack_validator_verbose", msgpack_validator_verbose);
    setOptionalArgument(cli_parameter_map, "msgpack_validator_discard_msgpacks_out_of_bounds", msgpack_validator_discard_msgpacks_out_of_bounds);
    setOptionalArgument(cli_parameter_map, "msgpack_validator_check_missing_scandata_interval", msgpack_validator_check_missing_scandata_interval);
    std::string cli_msgpack_validator_required_echos, cli_msgpack_validator_valid_segments, cli_msgpack_validator_layer_filter;
    float cli_msgpack_validator_azimuth_start_deg, cli_msgpack_validator_azimuth_end_deg, cli_msgpack_validator_elevation_start_deg, cli_msgpack_validator_elevation_end_deg;
    if (setOptionalArgument(cli_parameter_map, "msgpack_validator_required_echos", cli_msgpack_validator_required_echos))
        sick_lidar3d::util::parseVector(cli_msgpack_validator_required_echos, msgpack_validator_filter_settings.msgpack_validator_required_echos);
    if (setOptionalArgument(cli_parameter_map, "msgpack_validator_azimuth_start", cli_msgpack_validator_azimuth_start_deg))
        msgpack_validator_filter_settings.msgpack_validator_azimuth_start = cli_msgpack_validator_azimuth_start_deg * (float)M_PI / 180.0f;
    if (setOptionalArgument(cli_parameter_map, "msgpack_validator_azimuth_end", cli_msgpack_validator_azimuth_end_deg))
        msgpack_validator_filter_settings.msgpack_validator_azimuth_end = cli_msgpack_validator_azimuth_end_deg * (float)M_PI / 180.0f;
    if (setOptionalArgument(cli_parameter_map, "msgpack_validator_elevation_start", cli_msgpack_validator_elevation_start_deg))
        msgpack_validator_filter_settings.msgpack_validator_elevation_start = cli_msgpack_validator_elevation_start_deg * (float)M_PI / 180.0f;
    if (setOptionalArgument(cli_parameter_map, "msgpack_validator_elevation_end", cli_msgpack_validator_elevation_end_deg))
        msgpack_validator_filter_settings.msgpack_validator_elevation_end = cli_msgpack_validator_elevation_end_deg * (float)M_PI / 180.0f;
    if (setOptionalArgument(cli_parameter_map, "msgpack_validator_valid_segments", cli_msgpack_validator_valid_segments))
        sick_lidar3d::util::parseVector(cli_msgpack_validator_valid_segments, msgpack_validator_valid_segments);
    if (setOptionalArgument(cli_parameter_map, "msgpack_validator_layer_filter", cli_msgpack_validator_layer_filter))
        sick_lidar3d::util::parseVector(cli_msgpack_validator_layer_filter, msgpack_validator_filter_settings.msgpack_validator_layer_filter);

    /* Overwrite with commandline arguments (obsolete)
    for (int n = 1; n < argc; n++)
    {
        if(strncmp(argv[n],"-udp_sender=", 12) == 0 || strncmp(argv[n],"udp_sender:=", 12) == 0)
            udp_sender = std::string(argv[n] + 12);
        if(strncmp(argv[n],"-udp_port=", 10) == 0 || strncmp(argv[n],"udp_port:=", 10) == 0)
            udp_port = std::atoi(argv[n] + 10);
        if(strncmp(argv[n],"-publish_topic=", 15) == 0 || strncmp(argv[n],"publish_topic:=", 15) == 0)
            publish_topic = std::string(argv[n] + 15);
        if(strncmp(argv[n],"-publish_frame_id=", 18) == 0 || strncmp(argv[n],"publish_frame_id:=", 18) == 0)
            publish_frame_id = std::string(argv[n] + 18);
        if(strncmp(argv[n],"-udp_input_fifolength=", 22) == 0 || strncmp(argv[n],"udp_input_fifolength:=", 22) == 0)
            udp_input_fifolength = std::atoi(argv[n] + 22);
        if(strncmp(argv[n],"-msgpack_output_fifolength=", 27) == 0 || strncmp(argv[n],"msgpack_output_fifolength:=", 27) == 0)
            msgpack_output_fifolength = std::atoi(argv[n] + 27);
        if(strncmp(argv[n],"-verbose_level=", 15) == 0 || strncmp(argv[n],"verbose_level:=", 15) == 0)
            verbose_level = std::atoi(argv[n] + 15);
        if(strncmp(argv[n],"-measure_timing=", 16) == 0 || strncmp(argv[n],"measure_timing:=", 16) == 0)
            measure_timing = (std::atoi(argv[n] + 16) > 0);
        if(strncmp(argv[n],"-visualize=", 11) == 0 || strncmp(argv[n],"visualize:=", 11) == 0)
            visualize = (std::atoi(argv[n] + 11) > 0);
        if(strncmp(argv[n],"-export_csv=", 12) == 0 || strncmp(argv[n],"export_csv:=", 12) == 0)
            export_csv = (std::atoi(argv[n] + 12) > 0);
        if (strncmp(argv[n], "-export_udp_msg=", 16) == 0 || strncmp(argv[n], "export_udp_msg:=", 16) == 0)
            export_udp_msg = (std::atoi(argv[n] + 16) > 0);
        if(strncmp(argv[n],"-exit_on_keys_esc_q=", 20) == 0 || strncmp(argv[n],"exit_on_keys_esc_q:=", 20))
            exit_on_keys_esc_q = (std::atoi(argv[n] + 20) > 0);
        if(strncmp(argv[n],"-logfolder=", 11) == 0 || strncmp(argv[n],"logfolder:=", 11) == 0)
            logfolder = std::string(argv[n] + 11);
        if(strncmp(argv[n],"-inputfolder_msgfiles=", 22) == 0 || strncmp(argv[n],"inputfolder_msgfiles:=", 22) == 0)
            inputfolder_msgfiles = std::string(argv[n] + 22);
        if(strncmp(argv[n],"-input_msgfiles=", 16) == 0 || strncmp(argv[n],"input_msgfiles:=", 16) == 0)
            input_msgfiles_templ = std::string(argv[n] + 16);
        if (strncmp(argv[n], "-mrs100_post_start_stop=", 24) == 0 || strncmp(argv[n], "mrs100_post_start_stop:=", 24) == 0)
            mrs100_post_start_stop = (std::atoi(argv[n] + 24) > 0);
        if (strncmp(argv[n], "-mrs100_ip=", 11) == 0 || strncmp(argv[n], "mrs100_ip:=", 11) == 0)
            mrs100_ip = std::string(argv[n] + 11);
        if (strncmp(argv[n], "-mrs100_dst_ip=", 15) == 0 || strncmp(argv[n], "mrs100_dst_ip:=", 15) == 0)
            mrs100_dst_ip = std::string(argv[n] + 15);
        if (strncmp(argv[n], "-mrs100_port=", 13) == 0 || strncmp(argv[n], "mrs100_port:=", 13) == 0)
            mrs100_port = std::atoi(argv[n] + 13);
        if (strncmp(argv[n], "-mrs100_send_udp_start=", 23) == 0 || strncmp(argv[n], "mrs100_send_udp_start:=", 23) == 0)
            mrs100_send_udp_start = (std::atoi(argv[n] + 23) > 0);
        if (strncmp(argv[n], "-mrs100_send_udp_start_string=", 30) == 0 || strncmp(argv[n], "mrs100_send_udp_start_string:=", 30) == 0)
            mrs100_send_udp_start_string = std::string(argv[n] + 30);
        if (strncmp(argv[n], "-msgpack_validator_required_echos=", 34) == 0 || strncmp(argv[n], "msgpack_validator_required_echos:=", 34) == 0)
            sick_lidar3d::util::parseVector(std::string(argv[n] + 34), msgpack_validator_filter_settings.msgpack_validator_required_echos);
        if (strstr(argv[n], "help") != 0 || strstr(argv[n], "?") != 0)
            PrintHelp();
        // LIDAR3D_INFO_STREAM(argv[n]);
    } */

    // Resolve input_msgfiles (offline modus, test only)
    if(!input_msgfiles_templ.empty())
    {
        input_msgfiles.clear();
        for(int cnt = 0; cnt < 0xFFFF; cnt++)
        {
            char temp_file[64*1024];
            SPRINTF(temp_file, input_msgfiles_templ.c_str(), cnt);
            if(sick_lidar3d::FileReadable(inputfolder_msgfiles + "/" + temp_file))
                input_msgfiles.push_back(temp_file);
            else if(cnt > 1)
                break;
        }
    }

    LIDAR3D_INFO_STREAM("udp_sender:                       " << udp_sender);
    LIDAR3D_INFO_STREAM("udp_port:                         " << udp_port);
    LIDAR3D_INFO_STREAM("publish_topic:                    " << publish_topic);
    LIDAR3D_INFO_STREAM("publish_topic_all_segments:       " << publish_topic_all_segments);
    LIDAR3D_INFO_STREAM("segment_count:                    " << segment_count);
    LIDAR3D_INFO_STREAM("publish_frame_id:                 " << publish_frame_id);
    LIDAR3D_INFO_STREAM("udp_input_fifolength:             " << udp_input_fifolength);
    LIDAR3D_INFO_STREAM("msgpack_output_fifolength:        " << msgpack_output_fifolength);
    LIDAR3D_INFO_STREAM("verbose_level:                    " << verbose_level);
    LIDAR3D_INFO_STREAM("measure_timing:                   " << measure_timing);
    LIDAR3D_INFO_STREAM("visualize:                        " << visualize);
    LIDAR3D_INFO_STREAM("export_csv:                       " << export_csv);
    LIDAR3D_INFO_STREAM("export_udp_msg:                   " << export_udp_msg);
    LIDAR3D_INFO_STREAM("exit_on_keys_esc_q:               " << exit_on_keys_esc_q);
    LIDAR3D_INFO_STREAM("logfolder:                        " << logfolder);
    LIDAR3D_INFO_STREAM("inputfolder_msgfiles:             " << inputfolder_msgfiles);
    LIDAR3D_INFO_STREAM("input_msgfiles:                   " << input_msgfiles_templ);
    LIDAR3D_INFO_STREAM("mrs100_post_start_stop:           " << mrs100_post_start_stop);
    LIDAR3D_INFO_STREAM("mrs100_ip:                        " << mrs100_ip);
    LIDAR3D_INFO_STREAM("mrs100_dst_ip:                    " << mrs100_dst_ip);
    LIDAR3D_INFO_STREAM("mrs100_port:                      " << mrs100_port);
    LIDAR3D_INFO_STREAM("mrs100_send_udp_start:            " << mrs100_send_udp_start);
    LIDAR3D_INFO_STREAM("mrs100_send_udp_start_string:     " << mrs100_send_udp_start_string);
    LIDAR3D_INFO_STREAM("sopas_tcp_port:                   " << sopas_tcp_port);
    LIDAR3D_INFO_STREAM("start_sopas_service:              " << start_sopas_service);
    LIDAR3D_INFO_STREAM("send_sopas_mrs100_start_stop_cmd: " << send_sopas_mrs100_start_stop_cmd);
    LIDAR3D_INFO_STREAM("sopas_cola_binary:                " << sopas_cola_binary);
    LIDAR3D_INFO_STREAM("sopas_timeout_ms:                 " << sopas_timeout_ms);
    LIDAR3D_INFO_STREAM("mrs100_read_filtersettings:       " << mrs100_read_filtersettings);
    LIDAR3D_INFO_STREAM("mrs100_FREchoFilter:              " << mrs100_FREchoFilter);
    LIDAR3D_INFO_STREAM("mrs100_set_FREchoFilter:          " << mrs100_set_FREchoFilter);
    LIDAR3D_INFO_STREAM("mrs100_LFPangleRangeFilter:       " << mrs100_LFPangleRangeFilter);
    LIDAR3D_INFO_STREAM("mrs100_set_LFPangleRangeFilter:   " << mrs100_set_LFPangleRangeFilter);
    LIDAR3D_INFO_STREAM("mrs100_LFPlayerFilter:            " << mrs100_LFPlayerFilter);
    LIDAR3D_INFO_STREAM("mrs100_set_LFPlayerFilter:        " << mrs100_set_LFPlayerFilter);
    LIDAR3D_INFO_STREAM("msgpack_validator_enabled:                         " << msgpack_validator_enabled);
    LIDAR3D_INFO_STREAM("msgpack_validator_verbose:                         " << msgpack_validator_verbose);
    LIDAR3D_INFO_STREAM("msgpack_validator_discard_msgpacks_out_of_bounds:  " << msgpack_validator_discard_msgpacks_out_of_bounds);
    LIDAR3D_INFO_STREAM("msgpack_validator_check_missing_scandata_interval: " << msgpack_validator_check_missing_scandata_interval);
    LIDAR3D_INFO_STREAM("msgpack_validator_required_echos:                  " << sick_lidar3d::util::printVector(msgpack_validator_filter_settings.msgpack_validator_required_echos));
    LIDAR3D_INFO_STREAM("msgpack_validator_azimuth_start:                   " << msgpack_validator_filter_settings.msgpack_validator_azimuth_start << " [rad]");
    LIDAR3D_INFO_STREAM("msgpack_validator_azimuth_end:                     " << msgpack_validator_filter_settings.msgpack_validator_azimuth_end << " [rad]");
    LIDAR3D_INFO_STREAM("msgpack_validator_elevation_start:                 " << msgpack_validator_filter_settings.msgpack_validator_elevation_start << " [rad]");
    LIDAR3D_INFO_STREAM("msgpack_validator_elevation_end:                   " << msgpack_validator_filter_settings.msgpack_validator_elevation_end << " [rad]");
    LIDAR3D_INFO_STREAM("msgpack_validator_valid_segments:                  " << sick_lidar3d::util::printVector(msgpack_validator_valid_segments));
    LIDAR3D_INFO_STREAM("msgpack_validator_layer_filter:                    " << sick_lidar3d::util::printVector(msgpack_validator_filter_settings.msgpack_validator_layer_filter));

    return true;
}
