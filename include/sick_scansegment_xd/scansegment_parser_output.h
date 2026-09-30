/**
 * @file sick_scansegment_xd_parser_output.h
 * @brief Data structures and utility functions for parsed scan segment output
 *        of multiScan and picoScan devices.
 *
 * Copyright (C) 2026, SICK AG, Waldkirch
 * Copyright (C) 2026, Ing.-Buero Dr. Michael Lehning, Hildesheim
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @date September 2026
 * @author Michael Lehning <michael.lehning@lehning.de>
 */

#ifndef SICK_SCANSEGMENT_XD_PARSER_OUTPUT_H
#define SICK_SCANSEGMENT_XD_PARSER_OUTPUT_H

#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
#include "sick_scan/sick_ros_wrapper.h"

namespace sick_scansegment_xd
{
    /**
     * @brief Container for configuration and settings for multiScan and picoScan parsers.
     */
    class ScanSegmentParserConfig
    {
    public:
        int imu_latency_microsec = 0; ///< IMU latency in microseconds.
    };

    /**
     * @brief Container for IMU data in Compact format.
     */
    class CompactImuData
    {
    public:
        bool valid = false; ///< True if the IMU data is valid.

        float acceleration_x = 0; ///< Acceleration along the x-axis including gravity in m/s^2.
        float acceleration_y = 0; ///< Acceleration along the y-axis including gravity in m/s^2.
        float acceleration_z = 0; ///< Acceleration along the z-axis including gravity in m/s^2.

        float angular_velocity_x = 0; ///< Angular velocity around the x-axis in rad/s.
        float angular_velocity_y = 0; ///< Angular velocity around the y-axis in rad/s.
        float angular_velocity_z = 0; ///< Angular velocity around the z-axis in rad/s.

        float orientation_w = 0; ///< Orientation quaternion component w.
        float orientation_x = 0; ///< Orientation quaternion component x.
        float orientation_y = 0; ///< Orientation quaternion component y.
        float orientation_z = 0; ///< Orientation quaternion component z.

        /**
         * @brief Returns a human-readable description of the IMU data.
         * @return Human-readable description of the IMU data.
         */
        std::string to_string() const;
    };

    /**
     * @brief Output container for unpacked and converted MsgPack and Compact
     *        scan data from multiScan136 and picoScan.
     *
     * For multiScan136, ScanSegmentParserOutput contains 16 groups (layers).
     * Each group contains up to 3 echoes, and each echo contains a list of
     * LidarPoint data in Cartesian coordinates (x, y, z in meters) with
     * intensity information.
     *
     * For picoScan, ScanSegmentParserOutput contains one layer.
     *
     * @par Usage example for MsgPack data
     * @code
     * std::ifstream msgpack_istream("polarscan_testdata_000.msg", std::ios::binary);
     * sick_scansegment_xd::ScanSegmentParserOutput scansegment_output;
     * sick_scansegment_xd::MsgPackParser::Parse(msgpack_istream, scansegment_output);
     *
     * sick_scansegment_xd::MsgPackParser::WriteCSV({ scansegment_output }, "polarscan_testdata_000.csv");
     *
     * for (int groupIdx = 0; groupIdx < scansegment_output.scandata.size(); groupIdx++)
     * {
     *     for (int echoIdx = 0; echoIdx < scansegment_output.scandata[groupIdx].scanlines.size(); echoIdx++)
     *     {
     *         std::vector<sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint>& scanline =
     *             scansegment_output.scandata[groupIdx].scanlines[echoIdx].points;
     *         std::cout << (groupIdx + 1) << ". group, " << (echoIdx + 1) << ". echo: ";
     *         for (int pointIdx = 0; pointIdx < scanline.size(); pointIdx++)
     *         {
     *             sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint& point = scanline[pointIdx];
     *             std::cout << (pointIdx > 0 ? "," : "") << "(" << point.x << "," << point.y << "," << point.z << "," << point.i << ")";
     *         }
     *         std::cout << std::endl;
     *     }
     * }
     * @endcode
     */
    class ScanSegmentParserOutput
    {
    public:
        ScanSegmentParserOutput();

        /**
         * @brief LiDAR point in Cartesian and polar coordinates.
         *
         * A LidarPoint contains Cartesian coordinates x, y and z in meters
         * and an intensity value. Additionally, polar coordinates are given
         * by range in meters and azimuth and elevation in radians.
         *
         * The point also contains the group index (0 to 15 for multiScan136),
         * echo index (0 to 2), point index, LiDAR timestamp and an optional
         * reflector bit.
         */
        class LidarPoint
        {
        public:
            LidarPoint() : x(0), y(0), z(0), i(0), range(0), azimuth(0), elevation(0), groupIdx(0), echoIdx(0), pointIdx(0), lidar_timestamp_microsec(0), reflectorbit(0) {}
            LidarPoint(float _x, float _y, float _z, float _i, float _range, float _azimuth, float _elevation, int _groupIdx, int _echoIdx, int _pointIdx, uint64_t _lidar_timestamp_microsec, uint8_t _reflector_bit)
                : x(_x), y(_y), z(_z), i(_i), range(_range), azimuth(_azimuth), elevation(_elevation), groupIdx(_groupIdx), echoIdx(_echoIdx), pointIdx(_pointIdx), lidar_timestamp_microsec(_lidar_timestamp_microsec), reflectorbit(_reflector_bit) {}

            float x;         ///< Cartesian x coordinate in meters.
            float y;         ///< Cartesian y coordinate in meters.
            float z;         ///< Cartesian z coordinate in meters.
            float i;         ///< Intensity.
            float range;     ///< Polar coordinate range in meters.
            float azimuth;   ///< Polar coordinate azimuth in radians.
            float elevation; ///< Polar coordinate elevation in radians.

            int groupIdx; ///< Group (layer) index, 0 <= groupIdx < 16 for multiScan136.
            int echoIdx;  ///< Echo index, 0 <= echoIdx < 3 for multiScan136.
            int pointIdx; ///< Point index, 0 <= pointIdx < 30 or 0 <= pointIdx < 240 for multiScan136.

            uint64_t lidar_timestamp_microsec; ///< LiDAR timestamp in microseconds.
            uint8_t reflectorbit;              ///< Optional reflector bit, 0 or 1, default: 0.
        };

        /**
         * @brief Container for one scanline (echo).
         *
         * multiScan136 and picoScan transmit up to 3 echoes. Each echo is
         * represented by one Scanline.
         */
        class Scanline
        {
        public:
            std::vector<LidarPoint> points; ///< List of all LiDAR points in this scanline.
        };

        /**
         * @brief Container for a group (layer) of scanlines.
         *
         * multiScan136 transmits 16 groups (layers). Each group contains
         * up to 3 echoes represented by Scanline objects.
         */
        class Scangroup
        {
        public:
            Scangroup() : timestampStart_sec(0), timestampStart_nsec(0), timestampStop_sec(0), timestampStop_nsec(0), scanlines() {}

            uint32_t timestampStart_sec;  ///< Seconds part of the group start timestamp.
            uint32_t timestampStart_nsec; ///< Nanoseconds part of the group start timestamp.
            uint32_t timestampStop_sec;   ///< Seconds part of the group stop timestamp.
            uint32_t timestampStop_nsec;  ///< Nanoseconds part of the group stop timestamp.

            std::vector<Scanline> scanlines; ///< Scanlines (echoes) of this group.
        };

        /**
         * @brief All scan data decoded from one MsgPack or Compact scan.
         */
        std::vector<Scangroup> scandata;

        /**
         * @brief Optional IMU data.
         */
        CompactImuData imudata;

        /**
         * @brief Timestamp of the scan data (message receive time or measurement time).
         */
        std::string timestamp;   ///< Timestamp in string format "<seconds>.<microseconds>".
        uint32_t timestamp_sec;  ///< Seconds part of the timestamp.
        uint32_t timestamp_nsec; ///< Nanoseconds part of the timestamp.

        /**
         * @brief Counters associated with the decoded scan segment.
         */
        int segmentIndex = 0;     ///< Counter for decoded scan segments.
        uint64_t telegramCnt = 0; ///< Telegram counter.
    };

    /**
     * @brief Returns a formatted timestamp "<sec>.<millisec>".
     *
     * @param[in] sec Seconds part of the timestamp.
     * @param[in] nsec Nanoseconds part of the timestamp.
     * @return Timestamp formatted as "<sec>.<millisec>".
     */
    std::string Timestamp(uint32_t sec, uint32_t nsec);

    /**
     * @brief Returns a formatted timestamp for the given system clock time.
     *
     * The timestamp is formatted as "YYYY-MM-DD hh-mm-ss.msec".
     *
     * @param[in] now System clock time point.
     * @return Formatted timestamp.
     */
    std::string Timestamp(const std::chrono::system_clock::time_point& now);

} // namespace sick_scansegment_xd

#endif // SICK_SCANSEGMENT_XD_PARSER_OUTPUT_H