#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * @brief class ScanSegmentParserOutput is the output container for unpacked and converted msgpack and compact data for multiScan136 and picoScan.
 * In case of multiScan136, ScanSegmentParserOutput has 16 groups (layers), each group has 3 echos, each echo has a list of LidarPoint data in catesian coordinates
 * (x, y, z in meter and intensity). In case of picoScan, ScanSegmentParserOutput has 1 layer.
 *
 * Usage example for msgpack data:
 *
 * std::ifstream msgpack_istream("polarscan_testdata_000.msg", std::ios::binary);
 * sick_scansegment_xd::ScanSegmentParserOutput scansegment_output;
 * sick_scansegment_xd::MsgPackParser::Parse(msgpack_istream, scansegment_output);
 *
 * sick_scansegment_xd::MsgPackParser::WriteCSV({ scansegment_output }, "polarscan_testdata_000.csv")
 *
 * for (int groupIdx = 0; groupIdx < scansegment_output.scandata.size(); groupIdx++)
 * {
 * 	 for (int echoIdx = 0; echoIdx < scansegment_output.scandata[groupIdx].size(); echoIdx++)
 * 	 {
 * 	   std::vector<sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint>& scanline = scansegment_output.scandata[groupIdx][echoIdx];
 * 	   std::cout << (groupIdx + 1) << ". group, " << (echoIdx + 1) << ". echo: ";
 * 	   for (int pointIdx = 0; pointIdx < scanline.size(); pointIdx++)
 * 	   {
 * 		  sick_scansegment_xd::ScanSegmentParserOutput::PointXYZI& point = scanline[pointIdx];
 * 		  std::cout << (pointIdx > 0 ? "," : "") << "(" << point.x << "," << point.y << "," << point.z << "," << point.i << ")";
 * 	   }
 * 	   std::cout << std::endl;
 * 	 }
 * }
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
#ifndef __SICK_SCANSEGMENT_XD_PARSER_OUTPUT_H
#define __SICK_SCANSEGMENT_XD_PARSER_OUTPUT_H

#include "sick_scan/sick_ros_wrapper.h"

namespace sick_scansegment_xd
{
    /*
    * @brief class ScanSegmentParserConfig is a container for configuration and settings for multiScan and picoScan parser
    */
    class ScanSegmentParserConfig
    {
    public:
        int imu_latency_microsec = 0; // imu latency in microseconds
    };


    /*
    * @brief class CompactImuData is a container for imu data in compact format
    */
    class CompactImuData
    {
    public:
        bool valid = false;
        float acceleration_x = 0; // 4 bytes float in m/s^2, acceleration along the x-axis including gravity
        float acceleration_y = 0; // 4 bytes float in m/s^2, acceleration along the y-axis including gravity
        float acceleration_z = 0; // 4 bytes float in m/s^2, acceleration along the z-axis including gravity
        float angular_velocity_x = 0; // 4 bytes float in rad/s
        float angular_velocity_y = 0; // 4 bytes float in rad/s
        float angular_velocity_z = 0; // 4 bytes float in rad/s
        float orientation_w = 0; // 4 bytes float, orientation quaternion w
        float orientation_x = 0; // 4 bytes float, orientation quaternion x
        float orientation_y = 0; // 4 bytes float, orientation quaternion y
        float orientation_z = 0; // 4 bytes float, orientation quaternion z
        std::string to_string() const; // returns a human readable description of the imu data
    };

    class ScanSegmentParserOutput
    {
    public:

        ScanSegmentParserOutput();

        /*
         * @brief class LidarPoint is a data point in cartesian coordinates with x, y, z in meter and an intensity value.
         * Additionally, polar coordinates with azimuth and elevation in radians and distance in meter are given plus the
         * group index (0 up to 15 for multiScan136) and the echo index (0 up to 2).
         */
        class LidarPoint
        {
        public:
            LidarPoint() : x(0), y(0), z(0), i(0), range(0), azimuth(0), elevation(0), groupIdx(0), echoIdx(0), pointIdx(0), lidar_timestamp_microsec(0), reflectorbit(0) {}
            LidarPoint(float _x, float _y, float _z, float _i, float _range, float _azimuth, float _elevation, int _groupIdx, int _echoIdx, int _pointIdx, uint64_t _lidar_timestamp_microsec, uint8_t _reflector_bit)
                : x(_x), y(_y), z(_z), i(_i), range(_range), azimuth(_azimuth), elevation(_elevation), groupIdx(_groupIdx), echoIdx(_echoIdx), pointIdx(_pointIdx), lidar_timestamp_microsec(_lidar_timestamp_microsec), reflectorbit(_reflector_bit) {}
            float x; // cartesian x coordinate in meter
            float y; // cartesian y coordinate in meter
            float z; // cartesian z coordinate in meter
            float i; // intensity
            float range;     // polar coordinate range in meter
            float azimuth;   // polar coordinate azimuth in radians
            float elevation; // polar coordinate elevation in radians
            int groupIdx;    // group index (layer), 0 <= groupIdx < 16 for multiScan136
            int echoIdx;     // echo index, 0 <= echoIdx < 3 for multiScan136
            int pointIdx;    // point index, 0 <= pointIdx < 30 resp. 0 <= pointIdx < 240 for multiScan136
            uint64_t lidar_timestamp_microsec; // lidar timestamp in microseconds
            uint8_t reflectorbit; // optional reflector bit, 0 or 1, default: 0
        };

        /*
         * @brief type Scanline is a vector of LidarPoint data. multiScan136 and picoScan transmit up to 3 echos, each echo is a Scanline.
         */
        class Scanline
        {
        public:
            std::vector<LidarPoint> points; // list of all scan points
        };

        /*
         * @brief type Scangroup is a vector of Scanlines. multiScan136 transmits 16 groups (layers), each group has max. 3 echos (3 scanlines).
         */
        class Scangroup
        {
        public:
            Scangroup() : timestampStart_sec(0), timestampStart_nsec(0), timestampStop_sec(0), timestampStop_nsec(0), scanlines() {}
            uint32_t timestampStart_sec;
            uint32_t timestampStart_nsec;
            uint32_t timestampStop_sec;
            uint32_t timestampStop_nsec;
            std::vector<Scanline> scanlines;
        };

        /*
         * @brief scandata contains all data of a msgpack or compact scan.
         */
        std::vector<Scangroup> scandata;

        /*
         * @brief optional imu data
         */
        CompactImuData imudata;

        /*
         * @brief Timestamp of scandata (message received time or measurement time)
         */
        std::string timestamp;   // timestamp in string format "<seconds>.<mikroseconds>"
        uint32_t timestamp_sec;  // seconds part of timestamp
        uint32_t timestamp_nsec; // nanoseconds part of timestamp

        /*
         * @brief Counter for each message (each scandata decoded from msgpack or compact data)
         */
        int segmentIndex;
        int telegramCnt;
    };

    /*
    * @brief return a formatted timestamp "<sec>.<millisec>".
    * @param[in] sec second part of timestamp
    * @param[in] nsec nanosecond part of timestamp
    * @return "<sec>.<millisec>"
    */
    std::string Timestamp(uint32_t sec, uint32_t nsec);

    /*
    * @brief return a timestamp of the current time (i.e. std::chrono::system_clock::now() formatted by "YYYY-MM-DD hh-mm-ss.msec").
    */
    std::string Timestamp(const std::chrono::system_clock::time_point& now);
}
#endif // __SICK_SCANSEGMENT_XD_PARSER_OUTPUT_H
