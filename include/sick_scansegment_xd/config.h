#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * @brief config.h implements the configuration (yaml, commandline and default parameters) for project sick_scansegment_xd.
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
#ifndef __SICK_SCANSEGMENT_XD_CONFIG_H
#define __SICK_SCANSEGMENT_XD_CONFIG_H

#include "sick_scan/sick_ros_wrapper.h"
#include <sick_scan/sick_cloud_transform.h>
#include "sick_scan/sick_range_filter.h"
#include "sick_scansegment_xd/common.h"

namespace sick_scansegment_xd
{
    /*
     * @brief Container for filter settings for msgpack validator, returned from  by queryMultiScanFiltersettings()
     */
    class MsgpackValidatorFilterConfig
    {
    public:
        std::vector<int> msgpack_validator_required_echos; // { 0, 1, 2 }
        float msgpack_validator_azimuth_start;             // default for full scan: -M_PI;
        float msgpack_validator_azimuth_end;               // default for full scan: +M_PI;
        float msgpack_validator_elevation_start;           // default for full scan: -M_PI/2.0;
        float msgpack_validator_elevation_end;             // default for full scan: +M_PI/2.0;
        std::vector<int> msgpack_validator_layer_filter;   // default for full scan: 16 layer active, i.e. { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }
    };

    /*
     * @brief class sick_scansegment_xd::Config implements the configuration
     * (yaml, commandline and default parameters) for project sick_scansegment_xd.
     */
    class Config
    {
    public:

        /** Returns true, if endianess of the current system (destination target) is big endian, otherwise false. */
        static bool SystemIsBigEndian(void);

        /*
         * @brief Default constructor, initializes the configuration with default values
         */
        Config();

        /*
         * @brief Destructor
         */
        ~Config();

        /*
         * @brief Initializes sick_scansegment_xd configuration by commandline arguments and yaml-file.
         */
        bool Init(int argc, char** argv);

        /*
         * @brief Initializes sick_scansegment_xd configuration
         * @param[in] node ROS node handle (always 0 on non-ros-targets)
         */
        bool Init(rosNodePtr node);

        /*
         * @brief Prints the commandline arguments.
         */
        void PrintHelp(void);

        /*
         * @brief Prints the current settings.
         */
        void PrintConfig(void);

        /*
         * sick_scansegment_xd configuration
         */

        std::string scanner_type;                    // currently supported: "sick_multiscan" and "sick_picoscan"

        std::string udp_sender;                     // = ""; // Use "" (default) to receive msgpacks from any udp sender, use "127.0.0.1" to restrict to localhost (loopback device), or use the ip-address of a Multiscan136 lidar or Multiscan136 emulator
        int udp_port;                               // = 2115; // default udp port for multiScan136 resp. multiScan136 emulator is 2115

        bool check_udp_receiver_ip = false;         // check udp_receiver_ip by sending and receiving a udp test message
        int check_udp_receiver_port = 2116;         // udp port to check udp_receiver_ip

        double all_segments_min_deg = -180;         // -180 // angle range covering all segments: all segments pointcloud on topic publish_topic_all_segments is published, 
        double all_segments_max_deg = +180;         // +180 // if received segments cover angle range from all_segments_min_deg to all_segments_max_deg. -180...+180 for MultiScan136 (360 deg fullscan)

        std::string publish_frame_id;               // = "world"; // frame id of ros Laserscan messages, default: "world"
        std::string publish_imu_frame_id;           // = "sick_imu"; // frame id of ros IMU messages, default: "sick_imu"
        std::string publish_laserscan_segment_topic;   // topic of ros Laserscan segment messages
        std::string publish_laserscan_fullframe_topic; //topic of ros Laserscan fullframe messages
        int udp_input_fifolength;                   // = 20; // max. udp input fifo length(-1: unlimited, default: 20 for buffering 1 second at 20 Hz), elements will be removed from front if number of elements exceeds the fifo_length
        int msgpack_output_fifolength;              // = 20; // max. msgpack output fifo length(-1: unlimited, default: 20 for buffering 1 second at 20 Hz), elements will be removed from front if number of elements exceeds the fifo_length
        int verbose_level;                          // = 1; // verbose_level <= 0: quiet mode, verbose_level == 1: print statistics, verbose_level == 2: print details incl. msgpack data, default: 1
        bool measure_timing;                        // = true; // measure_timing == true: duration and latency of msgpack conversion and export is measured, default: true
        bool export_csv;                            // = false; // export msgpack data to csv file, default: false
        bool export_udp_msg;                        // = false; // true : export binary udpand msgpack data to file(*.udp and* .msg), default: false
        std::string logfolder;                      // = "./logfiles"; // output folder for logfiles, default: "."
        std::string hostname;                       // IP address of multiScan136 to post start and stop commands
        std::string udp_receiver_ip;                // UDP destination IP address (ip address of udp receiver)
        // int port;                                   // IP port of multiScan136 to post start and stop commands
        // bool send_udp_start;                        // Send udp start string to multiScan136, default: False (obsolete)
        // std::string send_udp_start_string;          // udp string to start multiScan136, default: "magicalActivate"
        int udp_timeout_ms;                         // Timeout for udp messages in milliseconds, default: 10*1000
        int udp_timeout_ms_initial;                 // Initial timeout for udp messages after start in milliseconds, default: 60*1000
        int scandataformat;                         // ScanDataFormat: 1 for msgpack or 2 for compact scandata, default: 1
        int performanceprofilenumber;               // Set performance profile by sending "sWN PerformanceProfileNumber" if performanceprofilenumber >= 0 (picoScan), default: -1
        bool imu_enable;                            // IMU enabled or disabled
        std::string imu_topic;                      // ROS topic for IMU messages
        int imu_udp_port;                           // default udp port for multiScan imu data is 7503
        int imu_latency_microsec;                   // imu latency in microseconds

        // SOPAS settings
        std::string sopas_tcp_port;                 // TCP port for SOPAS commands, default port: 2111
        bool start_sopas_service;                   // True: sopas services for CoLa-commands are started (ROS only), default: true
        bool send_sopas_start_stop_cmd;             // True: multiScan136 start and stop command sequece ("sWN ScanDataEnable 0/1" etc.) are sent after driver start and stop, default: true
        bool sopas_cola_binary;                     // False: SOPAS uses CoLa-A (ascii, default, recommended), CoLa-B (true, binary) currently experimental
        int sopas_timeout_ms;                       // Timeout for SOPAS response in milliseconds, default: 5000
        int user_level = 3;                         // Default User Level (for picoScan, multiScan: 4)
        std::string user_level_password = "F4724744";  // Default password for client authorization

        // MSR100 filter settings
        bool host_read_filtersettings;             // True  // Read multiScan136 settings for FREchoFilter, LFPangleRangeFilter and LFPlayerFilter at startup, default: true
        int host_FREchoFilter;                     // 1     // Optionally set FREchoFilter with 0 for FIRST_ECHO (EchoCount=1), 1 for ALL_ECHOS (EchoCount=3), or 2 for LAST_ECHO (EchoCount=1)
        bool host_set_FREchoFilter;                // False // If true, FREchoFilter is set at startup (default: false)
        std::string host_LFPangleRangeFilter;      // "0 -180.0 +180.0 -90.0 +90.0 1" // Optionally set LFPangleRangeFilter to "<enabled> <azimuth_start> <azimuth_stop> <elevation_start> <elevation_stop> <beam_increment>" with azimuth and elevation given in degree
        bool host_set_LFPangleRangeFilter;         // False // If true, LFPangleRangeFilter is set at startup (default: false)
        std::string host_LFPlayerFilter;           // "0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1" (Multiscan136 only, not for picoscan) // Optionally set LFPlayerFilter to "<enabled> <layer0-enabled> <layer1-enabled> <layer2-enabled> ... <layer15-enabled>" with 1 for enabled and 0 for disabled
        bool host_set_LFPlayerFilter;              // False // If true (Multiscan136 only, always false for picoscan), LFPlayerFilter is set at startup (default: false)
        std::string host_LFPintervalFilter;           // "0 0" (Multiscan136 only, not for picoscan) // OOptionally set LFPintervalFilter to "<enabled> <N>" with 1 for enabled and 0 for disabled and N to reduce output to every N-th scan
        bool host_set_LFPintervalFilter;              // False // If true (Multiscan136 only, always false for picoscan), LFPintervalFilter is set at startup (default: false)
        // msgpack validation
        bool msgpack_validator_enabled; // true: check msgpack data for out of bounds and missing scan data, false: no msgpack validation
        int msgpack_validator_verbose;  // 0: print error messages, 1: print error and informational messages, 2: print error and all messages
        bool msgpack_validator_discard_msgpacks_out_of_bounds; // true: msgpacks are discarded if scan data out of bounds detected, false: error message if a msgpack is not validated
        int msgpack_validator_check_missing_scandata_interval; // check msgpack for missing scandata after collecting N msgpacks, default: N = 12 segments. Increase this value to tolerate udp packet drops. Use 12 to check each full scan.
        MsgpackValidatorFilterConfig msgpack_validator_filter_settings; // required_echos, azimuth_start, azimuth_end. elevation_start, elevation_end, layer_filter
        std::vector<int> msgpack_validator_valid_segments; // indices of valid segmentes, default for full scan: 12 segments, i.e. { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }

        // Apply an additional transform to the cartesian pointcloud, default: "0,0,0,0,0,0" (i.e. no transform)
        sick_scan_xd::SickCloudTransform add_transform_xyz_rpy;

        // Configuration of laserscan messages (ROS only), activate/deactivate laserscan messages for each layer
        std::vector<int> laserscan_layer_filter; // Default: { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, i.e. laserscan messages for layer 5 activated (elevation -0.07 degree, max number of scan points)

        rosNodePtr node; // NodePtr node; // ROS node handle (always 0 on non-ros-targets)

    }; // class Config

    /*
    * @brief Config utility functions
    */
    namespace util
    {

        /** Splits a string into substrings by a given delimiter */
        inline void parseVector(const std::string str, std::vector<std::string>& vec, char delim = ' ')
        {
            vec.clear();
            std::istringstream in_stream(str);
            std::string token;
            while (std::getline(in_stream, token, delim))
            {
                vec.push_back(token);
            }
        }

        /** Splits a string into a list of float values */
        inline void parseVector(const std::string str, std::vector<float>& vec, char delim = ' ')
        {
            vec.clear();
            std::vector<std::string> token;
            sick_scansegment_xd::util::parseVector(str, token, delim);
            for(int n = 0; n < token.size(); n++)
                vec.push_back(std::stof(token[n]));
        }

        /** Splits a string into a list of int values */
        inline void parseVector(const std::string str, std::vector<int>& vec, char delim = ' ')
        {
            vec.clear();
            std::vector<std::string> token;
            sick_scansegment_xd::util::parseVector(str, token, delim);
            for(int n = 0; n < token.size(); n++)
                vec.push_back(std::stoi(token[n]));
        }

        /** Prints a list of values to string */
        template <typename T> inline std::string printVector(const std::vector<T>& vec, const std::string& delim = " ")
        {
            std::stringstream s;
            for(int n = 0; n < vec.size(); n++)
                s << (n > 0 ? delim : "") << vec[n];
            return s.str();
        }


    } // namespace util

} // namespace sick_scansegment_xd
#endif // __SICK_SCANSEGMENT_XD_COMMON_H
