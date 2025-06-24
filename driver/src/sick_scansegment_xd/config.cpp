/*
 * @brief config.cpp implements the configuration (yaml, commandline and default parameters) for project sick_scansegment_xd.
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
#include <sick_scan/sick_ros_wrapper.h>
#include "sick_scansegment_xd/config.h"
#include <sick_scan/sick_scan_common_tcp.h>
#include <sick_scan/sick_generic_parser.h>

#define ROS_DECL_GET_PARAMETER(node,name,value) do{rosDeclareParam((node),(name),(value));rosGetParam((node),(name),(value));}while(false)

/** Lookup a table of optional key value pairs and overwrite argument if key found in table */
static bool setOptionalArgument(const std::map<std::string, std::string>& key_value_pairs, const std::string& key, std::string& argument)
{
    std::map<std::string, std::string>::const_iterator key_value_pair_iter = key_value_pairs.find(key);
    if (key_value_pair_iter != key_value_pairs.cend())
    {
        argument = key_value_pair_iter->second;
        ROS_INFO_STREAM(key << "=\"" << argument << "\" set by commandline");
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
        ROS_INFO_STREAM(key << "=" << (argument?"true":"false") << " set by commandline");
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
        ROS_INFO_STREAM(key << "=" << argument << " set by commandline");
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
        ROS_INFO_STREAM(key << "=" << argument << " set by commandline");
        return true;
    }
    return false;
}

/** Lookup a table of optional key value pairs and overwrite argument if key found in table */
static bool setOptionalArgument(const std::map<std::string, std::string>& key_value_pairs, const std::string& key, double& argument)
{
    std::string str_argument;
    if (setOptionalArgument(key_value_pairs, key, str_argument) && !str_argument.empty())
    {
        argument = std::stod(str_argument);
        ROS_INFO_STREAM(key << "=" << argument << " set by commandline");
        return true;
    }
    return false;
}

/** Returns true, if endianess of the current system (destination target) is big endian, otherwise false. */
bool sick_scansegment_xd::Config::SystemIsBigEndian(void)
{
	// Get endianess of the system (destination target) by comparing MSB and LSB of a int32 number
	uint32_t u32_one = 1;
	uint8_t* p_one = (uint8_t*)&u32_one;
	uint8_t lsb = p_one[0];
	uint8_t msb = p_one[3];
	bool dstTargetIsBigEndian = (lsb == 0 && msb != 0);
	bool dstTargetIsLittleEndian = (lsb != 0 && msb == 0);
	assert(dstTargetIsBigEndian || dstTargetIsLittleEndian);
	return dstTargetIsBigEndian;
}

/*
 * @brief Default constructor, initializes the configuration with default values
 */
sick_scansegment_xd::Config::Config()
{
    node = 0;                           // Created by Config::Init()
    udp_sender = "";                    // Use "" (default) to receive msgpacks from any udp sender, use "127.0.0.1" to restrict to localhost (loopback device), or use the ip-address of a multiScan136 lidar or multiScan136 emulator
    udp_port = 2115;                    // default udp port for multiScan136 resp. multiScan136 emulator is 2115
    publish_frame_id = "world";            // frame id of ros Laserscan messages, default: "world_<layer-id>"
    publish_imu_frame_id = "sick_imu";     // frame id of ros IMU messages, default: "sick_imu"
    publish_laserscan_segment_topic = "scan_segment";     // topic of ros Laserscan segment messages
    publish_laserscan_fullframe_topic = "scan_fullframe"; //topic of ros Laserscan fullframe messages
    udp_input_fifolength = 20;             // max. udp input fifo length (-1: unlimited, default: 20 for buffering 1 second at 20 Hz), elements will be removed from front if number of elements exceeds the fifo_length
    msgpack_output_fifolength = 20;        // max. msgpack output fifo length (-1: unlimited, default: 20 for buffering 1 second at 20 Hz), elements will be removed from front if number of elements exceeds the fifo_length
    verbose_level = 1;                     // verbose_level <= 0: quiet mode, verbose_level == 1: print statistics, verbose_level == 2: print details incl. msgpack data, default: 1
    measure_timing = true;                 // measure_timing == true: duration and latency of msgpack conversion and export is measured, default: true
    export_csv = false;                    // export msgpack data to csv file, default: false
    export_udp_msg = false;                // true : export binary udp and msgpack data to file(*.udp and* .msg), default: false
    logfolder = "";                        // output folder for logfiles, default: "" (no logging)
    hostname = "192.168.0.1";              // IP address of multiScan136 to post start and stop commands
    udp_receiver_ip = "";                  // UDP destination IP address (host ip used in setIpAddress posted)
    // port = 2115;                           // UDP port of multiScan136 to post start and stop commands
    // send_udp_start = false;                // Send udp start string to multiScan136, default: True
    // send_udp_start_string = "magicalActivate"; // udp string to start multiScan136, default: "magicalActivate"
    udp_timeout_ms = 10000;                  // Timeout for udp messages in milliseconds, default: 10*1000
    udp_timeout_ms_initial = 60000;          // Initial timeout for udp messages after start in milliseconds, default: 60*1000
    scandataformat = 2;                      // ScanDataFormat: 1 for msgpack or 2 for compact scandata, default: 2
    performanceprofilenumber = -1;           // Set performance profile by sending "sWN PerformanceProfileNumber" if performanceprofilenumber >= 0 (picoScan), default: -1
    imu_enable = false;                       // IMU enabled or disabled
    imu_topic = "imu";                       // ROS topic for IMU messages
    imu_udp_port = 7503;                     // default udp port for multiScan imu data is 7503
    imu_latency_microsec = 0;                // imu latency in microseconds

    // SOPAS default settings
    sopas_tcp_port = "2111";                 // TCP port for SOPAS commands, default port: 2111
    start_sopas_service = true;              // True: sopas services for CoLa-commands are started (ROS only), default: true
    send_sopas_start_stop_cmd = true;        // True: multiScan136 start and stop command sequence ("sWN ScanDataEnable 0/1" etc.) are sent after driver start and stop, default: true
    sopas_cola_binary = false;               // False: SOPAS uses CoLa-A (ascii, default, recommended), CoLa-B (true, binary) currently experimental
    sopas_timeout_ms = 5000;                 // Timeout for SOPAS response in milliseconds, default: 5000
    user_level = 3;                          // Default user level for client authorization (3 -> "authorized client", 4 -> "service")
    user_level_password = "F4724744";        // Default password for client authorization 
    

    // MSR100 default filter settings
    host_read_filtersettings = true;                            // Read multiScan136 settings for FREchoFilter, LFPangleRangeFilter and LFPlayerFilter at startup, default: true
    host_FREchoFilter = 1;                                      // Optionally set FREchoFilter with 0 for FIRST_ECHO (EchoCount=1), 1 for ALL_ECHOS (EchoCount=3), or 2 for LAST_ECHO (EchoCount=1)
    host_set_FREchoFilter = false;                              // If true, FREchoFilter is set at startup (default: false)
    host_LFPangleRangeFilter = "0 -180.0 +180.0 -90.0 +90.0 1"; // Optionally set LFPangleRangeFilter to "<enabled> <azimuth_start> <azimuth_stop> <elevation_start> <elevation_stop> <beam_increment>" with azimuth and elevation given in degree
    host_set_LFPangleRangeFilter = false;                       // If true, LFPangleRangeFilter is set at startup (default: false)
    host_LFPlayerFilter = "0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1";  // (Multiscan136 only, not for picoscan) Optionally set LFPlayerFilter to "<enabled> <layer0-enabled> <layer1-enabled> <layer2-enabled> ... <layer15-enabled>" with 1 for enabled and 0 for disabled
    host_set_LFPlayerFilter = false;                            // If true (Multiscan136 only, always false for picoscan), LFPlayerFilter is set at startup (default: false)
    host_LFPintervalFilter = "0 1";                             // Optionally set LFPintervalFilter to "<enabled> <N>" with 1 for enabled and 0 for disabled and N to reduce output to every N-th scan
    host_set_LFPintervalFilter = false;                         // If true, LFPintervalFilter is set at startup (default: false)

    // msgpack validation default settings
    msgpack_validator_enabled = false; // true: check msgpack data for out of bounds and missing scan data, false (default): no msgpack validation
    msgpack_validator_verbose = 0;    // 0: print error messages, 1: print error and informational messages, 2: print error and all messages
    msgpack_validator_discard_msgpacks_out_of_bounds = true; // true: msgpacks are discarded if scan data out of bounds detected, false: error message if a msgpack is not validated
    msgpack_validator_check_missing_scandata_interval = 12; //
    msgpack_validator_filter_settings.msgpack_validator_required_echos = { 0 };               // default: 1 echo, for all echos use { 0, 1, 2 }
    msgpack_validator_filter_settings.msgpack_validator_azimuth_start = (float)(-M_PI);       // default for full scan: -M_PI;
    msgpack_validator_filter_settings.msgpack_validator_azimuth_end = (float)(M_PI);          // default for full scan: +M_PI;
    msgpack_validator_filter_settings.msgpack_validator_elevation_start = (float)(-M_PI/2.0); // default for full scan: -M_PI/2.0;
    msgpack_validator_filter_settings.msgpack_validator_elevation_end = (float)(M_PI/2.0);    // default for full scan: +M_PI/2.0;
    msgpack_validator_filter_settings.msgpack_validator_layer_filter = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }; // default for full scan: 16 layer active, i.e. { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }
    msgpack_validator_valid_segments = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }; // default for full scan: 12 segments, i.e. { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }

    // Configuration of laserscan messages (ROS only):
    // Parameter "laserscan_layer_filter" sets a mask to create laserscan messages for configured layer (0: no laserscan message, 1: create laserscan messages for this layer)
    // Use "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0" to activate resp. "1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1" to activate laserscan messages for all 16 layers of the Multiscan136
    // Default is "0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0", i.e. laserscan messages for layer 5, (elevation -0.07 degree, max number of scan points)
    laserscan_layer_filter = { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

}

/*
 * @brief Destructor
 */
sick_scansegment_xd::Config::~Config()
{
}

/*
 * @brief Prints the commandline arguments.
 */
void sick_scansegment_xd::Config::PrintHelp(void)
{
    ROS_INFO_STREAM("sick_scansegment_xd receives udp packets from multiScan136 or multiScan136 emulator, unpacks, converts and exports the lidar data.");
    ROS_INFO_STREAM("Commandline options are:");
    ROS_INFO_STREAM("-udp_sender=<ip> : ip address of udp sender, Use \"\" (default) to receive msgpacks from any udp sender, use \"127.0.0.1\" to restrict to localhost (loopback device), or use the ip-address of a multiScan136 lidar or multiScan136 emulator");
    ROS_INFO_STREAM("-udp_port=<port> : udp port for multiScan136 resp. multiScan136 emulator, default: " << udp_port);
    ROS_INFO_STREAM("-udp_input_fifolength=<size> : max. udp input fifo length (-1: unlimited, default: 20 for buffering 1 second at 20 Hz), elements will be removed from front if number of elements exceeds the fifo_length");
    ROS_INFO_STREAM("-msgpack_output_fifolength=<size> : max. msgpack output fifo length(-1: unlimited, default: 20 for buffering 1 second at 20 Hz), elements will be removed from front if number of elements exceeds the fifo_length");
    ROS_INFO_STREAM("-verbose_level=[0-2] : verbose_level <= 0: quiet mode, verbose_level == 1: print statistics, verbose_level == 2: print details incl. msgpack data, default: " << verbose_level);
    ROS_INFO_STREAM("-measure_timing=0|1 : measure_timing == true: duration and latency of msgpack conversion and export is measured, default: " << measure_timing);
    ROS_INFO_STREAM("-export_csv=0|1 : export msgpack data to csv file, default: false");
    ROS_INFO_STREAM("-export_udp_msg=0|1 : export binary udp and msgpack data to file(*.udp and* .msg), default: false");
    ROS_INFO_STREAM("-logfolder=<directory> : output folder for logfiles" );
    ROS_INFO_STREAM("-hostname=<ip-address> : ip address of multiScan136 to post start and stop commands default:" << hostname);
    ROS_INFO_STREAM("-udp_receiver_ip=<ip-address> : UDP destination IP address (ip address of udp receiver), default:\"" << udp_receiver_ip << "\"");
    // ROS_INFO_STREAM("-port=<port> :  udp port of multiScan136 to post start and stop commands default:" << port);
    // ROS_INFO_STREAM("-send_udp_start=0|1 : send udp start string to multiScan136, default:" << send_udp_start);
    // ROS_INFO_STREAM("-send_udp_start_string=<string> : udp string to start multiScan136, default: \"magicalActivate\"" << send_udp_start_string);
    ROS_INFO_STREAM("-scandataformat=1|2 : set ScanDataFormat, 1 for msgpack or 2 for compact scandata, default: " << scandataformat);
    ROS_INFO_STREAM("-performanceprofilenumber=[1-9] : set PerformanceProfileNumber or -1 to disable, default: " << performanceprofilenumber);
    ROS_INFO_STREAM("-imu_enable=0|1 : enable or disable IMU data, default: " << imu_enable);
    ROS_INFO_STREAM("-imu_topic=<name> : ROS topic of IMU messages, default: " << imu_topic);
    ROS_INFO_STREAM("-imu_udp_port=<port>: udp port for multiScan imu data, default: " << imu_udp_port);
    ROS_INFO_STREAM("-imu_latency_microsec=<micro_sec>: imu latency in microseconds, default: " << imu_latency_microsec);
}

/*
 * @brief Initializes sick_scansegment_xd configuration
 * @param[in] node ROS node handle (always 0 on non-ros-targets)
 */
bool sick_scansegment_xd::Config::Init(rosNodePtr _node)
{
    node = _node;

    ROS_DECL_GET_PARAMETER(node, "scanner_type", scanner_type);
    ROS_DECL_GET_PARAMETER(node, "hostname", hostname);
    ROS_DECL_GET_PARAMETER(node, "udp_sender", udp_sender);
    ROS_DECL_GET_PARAMETER(node, "udp_port", udp_port);
    ROS_DECL_GET_PARAMETER(node, "check_udp_receiver_ip", check_udp_receiver_ip);
    ROS_DECL_GET_PARAMETER(node, "check_udp_receiver_port", check_udp_receiver_port);
    ROS_DECL_GET_PARAMETER(node, "all_segments_min_deg", all_segments_min_deg);
    ROS_DECL_GET_PARAMETER(node, "all_segments_max_deg", all_segments_max_deg);
    ROS_DECL_GET_PARAMETER(node, "publish_frame_id", publish_frame_id);
    ROS_DECL_GET_PARAMETER(node, "publish_imu_frame_id", publish_imu_frame_id);
    ROS_DECL_GET_PARAMETER(node, "publish_laserscan_segment_topic", publish_laserscan_segment_topic);
    ROS_DECL_GET_PARAMETER(node, "publish_laserscan_fullframe_topic", publish_laserscan_fullframe_topic);
    ROS_DECL_GET_PARAMETER(node, "udp_input_fifolength", udp_input_fifolength);
    ROS_DECL_GET_PARAMETER(node, "msgpack_output_fifolength", msgpack_output_fifolength);
    ROS_DECL_GET_PARAMETER(node, "verbose_level", verbose_level);
    ROS_DECL_GET_PARAMETER(node, "measure_timing", measure_timing);
    ROS_DECL_GET_PARAMETER(node, "export_csv", export_csv);
    ROS_DECL_GET_PARAMETER(node, "export_udp_msg", export_udp_msg);
    ROS_DECL_GET_PARAMETER(node, "logfolder", logfolder);
    ROS_DECL_GET_PARAMETER(node, "udp_receiver_ip", udp_receiver_ip);
    // ROS_DECL_GET_PARAMETER(node, "send_udp_start_port", port);
    // ROS_DECL_GET_PARAMETER(node, "send_udp_start", send_udp_start);
    // ROS_DECL_GET_PARAMETER(node, "send_udp_start_string", send_udp_start_string);
    ROS_DECL_GET_PARAMETER(node, "udp_timeout_ms", udp_timeout_ms);
    ROS_DECL_GET_PARAMETER(node, "udp_timeout_ms_initial", udp_timeout_ms_initial);
    ROS_DECL_GET_PARAMETER(node, "scandataformat", scandataformat);
    ROS_DECL_GET_PARAMETER(node, "performanceprofilenumber", performanceprofilenumber);    
    ROS_DECL_GET_PARAMETER(node, "imu_enable", imu_enable);
    ROS_DECL_GET_PARAMETER(node, "imu_topic", imu_topic);
    ROS_DECL_GET_PARAMETER(node, "imu_udp_port", imu_udp_port);
    ROS_DECL_GET_PARAMETER(node, "imu_latency_microsec", imu_latency_microsec);
    ROS_DECL_GET_PARAMETER(node, "sopas_tcp_port", sopas_tcp_port);
    ROS_DECL_GET_PARAMETER(node, "start_sopas_service", start_sopas_service);
    ROS_DECL_GET_PARAMETER(node, "send_sopas_start_stop_cmd", send_sopas_start_stop_cmd);
    ROS_DECL_GET_PARAMETER(node, "sopas_cola_binary", sopas_cola_binary);
    ROS_DECL_GET_PARAMETER(node, "sopas_timeout_ms", sopas_timeout_ms);
    ROS_DECL_GET_PARAMETER(node, "user_level", user_level);
    ROS_DECL_GET_PARAMETER(node, "user_level_password", user_level_password);
    // MSR100 filter settings
    ROS_DECL_GET_PARAMETER(node, "host_read_filtersettings", host_read_filtersettings);
    ROS_DECL_GET_PARAMETER(node, "host_FREchoFilter", host_FREchoFilter);
    ROS_DECL_GET_PARAMETER(node, "host_set_FREchoFilter", host_set_FREchoFilter);
    ROS_DECL_GET_PARAMETER(node, "host_LFPangleRangeFilter", host_LFPangleRangeFilter);
    ROS_DECL_GET_PARAMETER(node, "host_set_LFPangleRangeFilter", host_set_LFPangleRangeFilter);
    ROS_DECL_GET_PARAMETER(node, "host_LFPintervalFilter", host_LFPintervalFilter);
    ROS_DECL_GET_PARAMETER(node, "host_set_LFPintervalFilter", host_set_LFPintervalFilter);
    if (scanner_type != SICK_SCANNER_PICOSCAN_NAME)
    {
        ROS_DECL_GET_PARAMETER(node, "host_LFPlayerFilter", host_LFPlayerFilter);
        ROS_DECL_GET_PARAMETER(node, "host_set_LFPlayerFilter", host_set_LFPlayerFilter);
    }
    else
    {
        host_LFPlayerFilter = "";
        host_set_LFPlayerFilter = false;
    }
    // msgpack validation settings
    std::string str_msgpack_validator_required_echos = "0";
    std::string str_msgpack_validator_valid_segments = "0 1 2 3 4 5 6 7 8 9 10 11";
    std::string str_msgpack_validator_layer_filter = "1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1";
    float msgpack_validator_azimuth_start_deg = msgpack_validator_filter_settings.msgpack_validator_azimuth_start * 180.0 / M_PI;
    float msgpack_validator_azimuth_end_deg = msgpack_validator_filter_settings.msgpack_validator_azimuth_end * 180.0 / M_PI;
    float msgpack_validator_elevation_start_deg = msgpack_validator_filter_settings.msgpack_validator_elevation_start * 180.0 / M_PI;
    float msgpack_validator_elevation_end_deg = msgpack_validator_filter_settings.msgpack_validator_elevation_end * 180.0 / M_PI;
    ROS_DECL_GET_PARAMETER(node, "msgpack_validator_enabled", msgpack_validator_enabled);
    ROS_DECL_GET_PARAMETER(node, "msgpack_validator_verbose", msgpack_validator_verbose);
    ROS_DECL_GET_PARAMETER(node, "msgpack_validator_discard_msgpacks_out_of_bounds", msgpack_validator_discard_msgpacks_out_of_bounds);
    ROS_DECL_GET_PARAMETER(node, "msgpack_validator_check_missing_scandata_interval", msgpack_validator_check_missing_scandata_interval);
    ROS_DECL_GET_PARAMETER(node, "msgpack_validator_required_echos", str_msgpack_validator_required_echos);
    sick_scansegment_xd::util::parseVector(str_msgpack_validator_required_echos, msgpack_validator_filter_settings.msgpack_validator_required_echos);
    ROS_DECL_GET_PARAMETER(node, "msgpack_validator_azimuth_start", msgpack_validator_azimuth_start_deg);
    ROS_DECL_GET_PARAMETER(node, "msgpack_validator_azimuth_end", msgpack_validator_azimuth_end_deg);
    ROS_DECL_GET_PARAMETER(node, "msgpack_validator_elevation_start", msgpack_validator_elevation_start_deg);
    ROS_DECL_GET_PARAMETER(node, "msgpack_validator_elevation_end", msgpack_validator_elevation_end_deg);
    msgpack_validator_filter_settings.msgpack_validator_azimuth_start = msgpack_validator_azimuth_start_deg * M_PI / 180;
    msgpack_validator_filter_settings.msgpack_validator_azimuth_end = msgpack_validator_azimuth_end_deg * M_PI / 180;
    msgpack_validator_filter_settings.msgpack_validator_elevation_start = msgpack_validator_elevation_start_deg * M_PI / 180;
    msgpack_validator_filter_settings.msgpack_validator_elevation_end = msgpack_validator_elevation_end_deg * M_PI / 180;
    ROS_DECL_GET_PARAMETER(node, "msgpack_validator_valid_segments", str_msgpack_validator_valid_segments);
    ROS_DECL_GET_PARAMETER(node, "msgpack_validator_layer_filter", str_msgpack_validator_layer_filter);
    sick_scansegment_xd::util::parseVector(str_msgpack_validator_valid_segments, msgpack_validator_valid_segments);
    sick_scansegment_xd::util::parseVector(str_msgpack_validator_layer_filter, msgpack_validator_filter_settings.msgpack_validator_layer_filter);
    // Apply an additional transform to the cartesian pointcloud, default: "0,0,0,0,0,0" (i.e. no transform)
    std::string str_add_transform_xyz_rpy = "0,0,0,0,0,0";
    ROS_DECL_GET_PARAMETER(node, "add_transform_xyz_rpy", str_add_transform_xyz_rpy);
    bool add_transform_check_dynamic_updates = false;
    ROS_DECL_GET_PARAMETER(node, "add_transform_check_dynamic_updates", add_transform_check_dynamic_updates);
    add_transform_xyz_rpy = sick_scan_xd::SickCloudTransform(node, str_add_transform_xyz_rpy, true, add_transform_check_dynamic_updates);

    // Configuration of laserscan messages (ROS only), activate/deactivate laserscan messages for each layer
    std::string str_laserscan_layer_filter = "0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0";
    ROS_DECL_GET_PARAMETER(node, "laserscan_layer_filter", str_laserscan_layer_filter);
    sick_scansegment_xd::util::parseVector(str_laserscan_layer_filter, laserscan_layer_filter);

    if (imu_enable && scandataformat != 2)
    {
      ROS_ERROR_STREAM("## ERROR sick_scansegment_xd::Config::Init(): IMU requieres scandataformat 2, IMU deactivated.");
      imu_enable = false;
    }

    return true;
}

/*
 * @brief Initializes sick_scansegment_xd configuration by commandline arguments and yaml-file.
 */
bool sick_scansegment_xd::Config::Init(int argc, char** argv)
{
    // Read yaml configuration
    #if defined __ROS_VERSION && __ROS_VERSION > 1
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node = rclcpp::Node::make_shared("sick_scansegment_xd", "", node_options);
    Init(node);
    #elif defined __ROS_VERSION && __ROS_VERSION > 0
    ros::init(argc, argv, "sick_scansegment_xd");
    node = new ros::NodeHandle();
    Init(node);
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
        ROS_INFO_STREAM("commandline argument found: \"" << iter->first << "\"=\"" << iter->second << "\"");

    // Overwrite with commandline arguments
    setOptionalArgument(cli_parameter_map, "udp_sender", udp_sender);
    setOptionalArgument(cli_parameter_map, "udp_port", udp_port);
    setOptionalArgument(cli_parameter_map, "check_udp_receiver_ip", check_udp_receiver_ip);
    setOptionalArgument(cli_parameter_map, "check_udp_receiver_port", check_udp_receiver_port);
    setOptionalArgument(cli_parameter_map, "all_segments_min_deg", all_segments_min_deg);
    setOptionalArgument(cli_parameter_map, "all_segments_max_deg", all_segments_max_deg);
    setOptionalArgument(cli_parameter_map, "publish_frame_id", publish_frame_id);
    setOptionalArgument(cli_parameter_map, "publish_imu_frame_id", publish_imu_frame_id);
    setOptionalArgument(cli_parameter_map, "publish_laserscan_segment_topic", publish_laserscan_segment_topic);
    setOptionalArgument(cli_parameter_map, "publish_laserscan_fullframe_topic", publish_laserscan_fullframe_topic);
    setOptionalArgument(cli_parameter_map, "udp_input_fifolength", udp_input_fifolength);
    setOptionalArgument(cli_parameter_map, "msgpack_output_fifolength", msgpack_output_fifolength);
    setOptionalArgument(cli_parameter_map, "verbose_level", verbose_level);
    setOptionalArgument(cli_parameter_map, "measure_timing", measure_timing);
    setOptionalArgument(cli_parameter_map, "export_csv", export_csv);
    setOptionalArgument(cli_parameter_map, "export_udp_msg", export_udp_msg);
    setOptionalArgument(cli_parameter_map, "logfolder", logfolder);
    setOptionalArgument(cli_parameter_map, "hostname", hostname);
    setOptionalArgument(cli_parameter_map, "udp_receiver_ip", udp_receiver_ip);
    // setOptionalArgument(cli_parameter_map, "send_udp_start", send_udp_start);;
    // setOptionalArgument(cli_parameter_map, "send_udp_start_string", send_udp_start_string);
    setOptionalArgument(cli_parameter_map, "udp_timeout_ms", udp_timeout_ms);
    setOptionalArgument(cli_parameter_map, "udp_timeout_ms_initial", udp_timeout_ms_initial);
    setOptionalArgument(cli_parameter_map, "scandataformat", scandataformat);
    setOptionalArgument(cli_parameter_map, "performanceprofilenumber", performanceprofilenumber);
    setOptionalArgument(cli_parameter_map, "imu_enable", imu_enable);
    setOptionalArgument(cli_parameter_map, "imu_topic", imu_topic);
    setOptionalArgument(cli_parameter_map, "imu_udp_port", imu_udp_port);
    setOptionalArgument(cli_parameter_map, "imu_latency_microsec", imu_latency_microsec);
    setOptionalArgument(cli_parameter_map, "sopas_tcp_port", sopas_tcp_port);
    setOptionalArgument(cli_parameter_map, "start_sopas_service", start_sopas_service);
    setOptionalArgument(cli_parameter_map, "send_sopas_start_stop_cmd", send_sopas_start_stop_cmd);
    setOptionalArgument(cli_parameter_map, "sopas_cola_binary", sopas_cola_binary);
    setOptionalArgument(cli_parameter_map, "sopas_timeout_ms", sopas_timeout_ms);
    setOptionalArgument(cli_parameter_map, "user_level", user_level);
    setOptionalArgument(cli_parameter_map, "user_level_password", user_level_password);
    setOptionalArgument(cli_parameter_map, "host_read_filtersettings", host_read_filtersettings);
    setOptionalArgument(cli_parameter_map, "host_FREchoFilter", host_FREchoFilter);
    setOptionalArgument(cli_parameter_map, "host_set_FREchoFilter", host_set_FREchoFilter);
    setOptionalArgument(cli_parameter_map, "host_LFPangleRangeFilter", host_LFPangleRangeFilter);
    setOptionalArgument(cli_parameter_map, "host_set_LFPangleRangeFilter", host_set_LFPangleRangeFilter);
    setOptionalArgument(cli_parameter_map, "host_LFPlayerFilter", host_LFPlayerFilter);
    setOptionalArgument(cli_parameter_map, "host_set_LFPlayerFilter", host_set_LFPlayerFilter);
    setOptionalArgument(cli_parameter_map, "host_LFPintervalFilter", host_LFPintervalFilter);
    setOptionalArgument(cli_parameter_map, "host_set_LFPintervalFilter", host_set_LFPintervalFilter);
    setOptionalArgument(cli_parameter_map, "msgpack_validator_enabled", msgpack_validator_enabled);
    setOptionalArgument(cli_parameter_map, "msgpack_validator_verbose", msgpack_validator_verbose);
    setOptionalArgument(cli_parameter_map, "msgpack_validator_discard_msgpacks_out_of_bounds", msgpack_validator_discard_msgpacks_out_of_bounds);
    setOptionalArgument(cli_parameter_map, "msgpack_validator_check_missing_scandata_interval", msgpack_validator_check_missing_scandata_interval);
    std::string cli_msgpack_validator_required_echos, cli_msgpack_validator_valid_segments, cli_msgpack_validator_layer_filter, cli_laserscan_layer_filter;
    float cli_msgpack_validator_azimuth_start_deg, cli_msgpack_validator_azimuth_end_deg, cli_msgpack_validator_elevation_start_deg, cli_msgpack_validator_elevation_end_deg;
    if (setOptionalArgument(cli_parameter_map, "msgpack_validator_required_echos", cli_msgpack_validator_required_echos))
        sick_scansegment_xd::util::parseVector(cli_msgpack_validator_required_echos, msgpack_validator_filter_settings.msgpack_validator_required_echos);
    if (setOptionalArgument(cli_parameter_map, "msgpack_validator_azimuth_start", cli_msgpack_validator_azimuth_start_deg))
        msgpack_validator_filter_settings.msgpack_validator_azimuth_start = cli_msgpack_validator_azimuth_start_deg * (float)M_PI / 180.0f;
    if (setOptionalArgument(cli_parameter_map, "msgpack_validator_azimuth_end", cli_msgpack_validator_azimuth_end_deg))
        msgpack_validator_filter_settings.msgpack_validator_azimuth_end = cli_msgpack_validator_azimuth_end_deg * (float)M_PI / 180.0f;
    if (setOptionalArgument(cli_parameter_map, "msgpack_validator_elevation_start", cli_msgpack_validator_elevation_start_deg))
        msgpack_validator_filter_settings.msgpack_validator_elevation_start = cli_msgpack_validator_elevation_start_deg * (float)M_PI / 180.0f;
    if (setOptionalArgument(cli_parameter_map, "msgpack_validator_elevation_end", cli_msgpack_validator_elevation_end_deg))
        msgpack_validator_filter_settings.msgpack_validator_elevation_end = cli_msgpack_validator_elevation_end_deg * (float)M_PI / 180.0f;
    if (setOptionalArgument(cli_parameter_map, "msgpack_validator_valid_segments", cli_msgpack_validator_valid_segments))
        sick_scansegment_xd::util::parseVector(cli_msgpack_validator_valid_segments, msgpack_validator_valid_segments);
    if (setOptionalArgument(cli_parameter_map, "msgpack_validator_layer_filter", cli_msgpack_validator_layer_filter))
        sick_scansegment_xd::util::parseVector(cli_msgpack_validator_layer_filter, msgpack_validator_filter_settings.msgpack_validator_layer_filter);
    if (setOptionalArgument(cli_parameter_map, "laserscan_layer_filter", cli_laserscan_layer_filter))
        sick_scansegment_xd::util::parseVector(cli_laserscan_layer_filter, laserscan_layer_filter);

    PrintConfig();

    if (imu_enable && scandataformat != 2)
    {
      ROS_ERROR_STREAM("## ERROR sick_scansegment_xd::Config::Init(): IMU requieres scandataformat 2, IMU deactivated.");
      imu_enable = false;
    }

    return true;
}

/*
 * @brief Prints the current settings.
 */
void sick_scansegment_xd::Config::PrintConfig(void)
{
    ROS_INFO_STREAM("sick_scansegment_xd configuration:");
    ROS_INFO_STREAM("scanner_type:                     " << scanner_type);
    ROS_INFO_STREAM("udp_sender:                       " << udp_sender);
    ROS_INFO_STREAM("udp_port:                         " << udp_port);
    ROS_INFO_STREAM("check_udp_receiver_ip:            " << check_udp_receiver_ip);
    ROS_INFO_STREAM("check_udp_receiver_port:          " << check_udp_receiver_port);
    ROS_INFO_STREAM("all_segments_min_deg:             " << all_segments_min_deg);
    ROS_INFO_STREAM("all_segments_max_deg:             " << all_segments_max_deg);
    ROS_INFO_STREAM("publish_frame_id:                 " << publish_frame_id);
    ROS_INFO_STREAM("publish_imu_frame_id:             " << publish_imu_frame_id);
    ROS_INFO_STREAM("publish_laserscan_segment_topic:  " << publish_laserscan_segment_topic);
    ROS_INFO_STREAM("publish_laserscan_fullframe_topic:" << publish_laserscan_fullframe_topic);
    ROS_INFO_STREAM("udp_input_fifolength:             " << udp_input_fifolength);
    ROS_INFO_STREAM("msgpack_output_fifolength:        " << msgpack_output_fifolength);
    ROS_INFO_STREAM("verbose_level:                    " << verbose_level);
    ROS_INFO_STREAM("measure_timing:                   " << measure_timing);
    ROS_INFO_STREAM("export_csv:                       " << export_csv);
    ROS_INFO_STREAM("export_udp_msg:                   " << export_udp_msg);
    ROS_INFO_STREAM("logfolder:                        " << logfolder);
    ROS_INFO_STREAM("hostname:                         " << hostname);
    ROS_INFO_STREAM("udp_receiver_ip:                  " << udp_receiver_ip);
    //ROS_INFO_STREAM("send_udp_start_port:              " << port);
    //ROS_INFO_STREAM("send_udp_start:                   " << send_udp_start);
    //ROS_INFO_STREAM("send_udp_start_string:            " << send_udp_start_string);
    ROS_INFO_STREAM("udp_timeout_ms:                   " << udp_timeout_ms);
    ROS_INFO_STREAM("udp_timeout_ms_initial:           " << udp_timeout_ms_initial);
    ROS_INFO_STREAM("scandataformat:                   " << scandataformat);
    ROS_INFO_STREAM("performanceprofilenumber:         " << performanceprofilenumber);
    ROS_INFO_STREAM("imu_enable:                       " << imu_enable);
    ROS_INFO_STREAM("imu_topic:                        " << imu_topic);
    ROS_INFO_STREAM("imu_udp_port:                     " << imu_udp_port);
    ROS_INFO_STREAM("imu_latency_microsec:             " << imu_latency_microsec);
    ROS_INFO_STREAM("sopas_tcp_port:                   " << sopas_tcp_port);
    ROS_INFO_STREAM("start_sopas_service:              " << start_sopas_service);
    ROS_INFO_STREAM("send_sopas_start_stop_cmd:        " << send_sopas_start_stop_cmd);
    ROS_INFO_STREAM("sopas_cola_binary:                " << sopas_cola_binary);
    ROS_INFO_STREAM("sopas_timeout_ms:                 " << sopas_timeout_ms);
    ROS_INFO_STREAM("host_read_filtersettings:         " << host_read_filtersettings);
    ROS_INFO_STREAM("host_FREchoFilter:                " << host_FREchoFilter);
    ROS_INFO_STREAM("host_set_FREchoFilter:            " << host_set_FREchoFilter);
    ROS_INFO_STREAM("host_LFPangleRangeFilter:         " << host_LFPangleRangeFilter);
    ROS_INFO_STREAM("host_set_LFPangleRangeFilter:     " << host_set_LFPangleRangeFilter);
    ROS_INFO_STREAM("host_LFPlayerFilter:              " << host_LFPlayerFilter);
    ROS_INFO_STREAM("host_set_LFPlayerFilter:          " << host_set_LFPlayerFilter);
    ROS_INFO_STREAM("host_LFPintervalFilter:           " << host_LFPintervalFilter);
    ROS_INFO_STREAM("host_set_LFPintervalFilter:       " << host_set_LFPintervalFilter);
    ROS_INFO_STREAM("laserscan_layer_filter:           " << sick_scansegment_xd::util::printVector(laserscan_layer_filter));
    ROS_INFO_STREAM("msgpack_validator_enabled:                         " << msgpack_validator_enabled);
    ROS_INFO_STREAM("msgpack_validator_verbose:                         " << msgpack_validator_verbose);
    ROS_INFO_STREAM("msgpack_validator_discard_msgpacks_out_of_bounds:  " << msgpack_validator_discard_msgpacks_out_of_bounds);
    ROS_INFO_STREAM("msgpack_validator_check_missing_scandata_interval: " << msgpack_validator_check_missing_scandata_interval);
    ROS_INFO_STREAM("msgpack_validator_required_echos:                  " << sick_scansegment_xd::util::printVector(msgpack_validator_filter_settings.msgpack_validator_required_echos));
    ROS_INFO_STREAM("msgpack_validator_azimuth_start:                   " << msgpack_validator_filter_settings.msgpack_validator_azimuth_start << " [rad]");
    ROS_INFO_STREAM("msgpack_validator_azimuth_end:                     " << msgpack_validator_filter_settings.msgpack_validator_azimuth_end << " [rad]");
    ROS_INFO_STREAM("msgpack_validator_elevation_start:                 " << msgpack_validator_filter_settings.msgpack_validator_elevation_start << " [rad]");
    ROS_INFO_STREAM("msgpack_validator_elevation_end:                   " << msgpack_validator_filter_settings.msgpack_validator_elevation_end << " [rad]");
    ROS_INFO_STREAM("msgpack_validator_valid_segments:                  " << sick_scansegment_xd::util::printVector(msgpack_validator_valid_segments));
    ROS_INFO_STREAM("msgpack_validator_layer_filter:                    " << sick_scansegment_xd::util::printVector(msgpack_validator_filter_settings.msgpack_validator_layer_filter));

}
