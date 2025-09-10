#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * @brief RosMsgpackPublisher publishes PointCloud2 messages with msgpack data
 * received from multiScan136.
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
#ifndef __SICK_SCANSEGMENT_XD_ROS_MSGPACK_PUBLISHER_H
#define __SICK_SCANSEGMENT_XD_ROS_MSGPACK_PUBLISHER_H

#include "sick_scan/sick_ros_wrapper.h"
#include "sick_scansegment_xd/config.h"
#include "sick_scansegment_xd/msgpack_exporter.h"
#include <unordered_set>
namespace sick_scansegment_xd
{
    /*
     * class PointXYZRAEI32f is just a container for a single multiScan136 point in cartesian coordinates (x, y, z), polar coordinates (range, azimuth, elevation), and intensity i
     */
    class PointXYZRAEI32f
    {
    public:
        PointXYZRAEI32f() : x(0), y(0), z(0), range(0), azimuth(0), elevation(0), i(0), layer(0), echo(0), lidar_timestamp_microsec(0), reflectorbit(0), infringed(0) {}
        PointXYZRAEI32f(float _x, float _y, float _z, float _range, float _azimuth, float _elevation, float _i, int _layer, int _echo, uint64_t _lidar_timestamp_microsec, uint8_t _reflector_bit) 
            : x(_x), y(_y), z(_z), range(_range), azimuth(_azimuth), elevation(_elevation), i(_i), layer(_layer), echo(_echo), lidar_timestamp_microsec(_lidar_timestamp_microsec), reflectorbit(_reflector_bit), infringed(0) {}
        float x;         // cartesian x coordinate in meter
        float y;         // cartesian y coordinate in meter
        float z;         // cartesian z coordinate in meter
        float range;     // polar coordinate range in meter
        float azimuth;   // polar coordinate azimuth in radians
        float elevation; // polar coordinate elevation in radians
        float i;         // intensity
        int layer;       // group index (layer), 0 <= layer < 16 for multiScan136
        int echo;        // echo index, 0 <= echo < 3 for multiScan136
        uint64_t lidar_timestamp_microsec; // lidar timestamp in microseconds
        uint8_t reflectorbit; // optional reflector bit, 0 or 1, default: 0
        uint8_t infringed;    // optional infringed bit, 0 or 1, default: 0
    };
  
    /** @brief Container for the field properties of a PointCloud2Msg with all fields (where each field contains x, y, z, i, range, azimuth, elevation, layer, echo, reflector) */
    class PointCloudFieldProperty
    {
    public:
        PointCloudFieldProperty(const std::string& _name, uint8_t _datatype, size_t _datasize, size_t _fieldoffset) : name(_name), datatype(_datatype), datasize(_datasize), fieldoffset(_fieldoffset) {}
        std::string name = "";   // id like "x", "y", "z", "i", "range", "azimuth", "elevation", "layer", "echo", "reflector"
        uint8_t datatype = 0;    // datatype like PointField::FLOAT32 or PointField::INT8
        size_t datasize = 0;     // number of bytes, e.g. sizeof(float) or sizeof(int8_t)
        size_t fieldoffset = 0;  // offset in bytes in structure PointXYZRAEI32f
    };

    /** @brief Set of field names for Cartesian pointclouds, i.e. pointclouds with fields x, y, z */
    const std::unordered_set<std::string> MinimalCartesianPointCloudFields = {
        "x", "y", "z"
    };

    /** @brief Set of field names for Polar pointclouds, i.e. pointclouds with fields azimuth, elevation, range */
    const std::unordered_set<std::string> MinimalPolarPointCloudFields = {
        "azimuth", "elevation", "range"
    };

    /** @brief Configuration of customized pointclouds */
    class CustomPointCloudConfiguration
    {
    public:
        CustomPointCloudConfiguration() {}
        CustomPointCloudConfiguration(const std::string& cfg_name, const std::string& cfg_str);
        const std::string& cfgName(void) const { return m_cfg_name; }                        // name of configuration, e.g. custom_pointcloud_cartesian_segmented
        bool publish(void) const { return m_publish; }                                       // if true, pointcloud will be published (otherwise not)
        const std::string& topic(void) const { return m_topic; }                             // ros topic to publish the pointcloud
        const std::string& frameid(void) const { return m_frameid ; }                        // ros frame_id of the pointcloud
        bool fullframe(void) const { return m_update_method == 0; }                          // returns true for fullframe pointcloud, or false for segmented pointcloud
        int coordinateNotation(void) const { return  m_coordinate_notation; }                // 0 = cartesian, 1 = polar, 2 = both cartesian and polar, 3 = customized fields
        PointCloud2MsgPublisher& publisher(void) { return m_publisher; }                     // ros publisher of customized pointcloud
        inline bool fieldEnabled(const std::string& fieldname)                               // returns true, if a field given its name (like "x", "y", "z", "i", etc.) is enabled (i.e. activated in the launchfile), otherwise false
        { 
            return m_field_enabled[fieldname]; 
        }
        inline bool pointEnabled(sick_scansegment_xd::PointXYZRAEI32f& lidar_point) // returns true, if a point is enabled (i.e. properties echo, layer, reflectorbit etc. are activated in the launchfile), otherwise false
        {
            bool range_modified = false;
	        bool point_enabled = m_echo_enabled[lidar_point.echo] 
                && m_layer_enabled[lidar_point.layer] 
                && m_reflector_enabled[lidar_point.reflectorbit] 
                && m_infringed_enabled[lidar_point.infringed]
                && m_range_filter.apply(lidar_point.range, range_modified); // note: range can be set depending on filter settings
            if (range_modified)
            {
				m_range_filter.applyXYZ(lidar_point.x, lidar_point.y, lidar_point.z, lidar_point.azimuth, lidar_point.elevation);
            }
            return point_enabled;
        }
        void print(void) const;
    protected:
        static std::string printValuesEnabled(const std::map<std::string,bool>& mapped_values, const std::string& delim = ",");
        static std::string printValuesEnabled(const std::map<int8_t,bool>& mapped_values, const std::string& delim = ",");
        std::string m_cfg_name = "";   // name of configuration, e.g. custom_pointcloud_cartesian_segmented
        bool m_publish = false;        // if true, pointcloud will be published (otherwise not)
        std::string m_topic = "";      // ros topic to publish the pointcloud
        std::string m_frameid = "";    // ros frame_id of the pointcloud
        int m_coordinate_notation = 0; // 0 = cartesian, 1 = polar, 2 = both cartesian and polar, 3 = customized fields
        int m_update_method = 0;       // 0 = fullframe pointcloud, 1 = segmented pointcloud
        sick_scan_xd::SickRangeFilter m_range_filter; // Optional range filter
        std::map<std::string, bool> m_field_enabled; // names of enabled field names (i.e. field enabled if m_field_enabled[field_name]==true), where field_name is "x", "y", "z", "i", "range", "azimuth", "elevation", "layer", "echo" or "reflector"
        std::map<int8_t, bool> m_echo_enabled; // enabled echos (i.e. point inserted in pointcloud, if m_echo_enabled[echo_idx]==true)
        std::map<int8_t, bool> m_layer_enabled; // enabled layers (i.e. point inserted in pointcloud, if m_layer_enabled[layer_idx]==true)
        std::map<int8_t, bool> m_reflector_enabled; // enabled reflectors (i.e. point inserted in pointcloud, if m_reflector_enabled[reflector_bit]==true)
        std::map<int8_t, bool> m_infringed_enabled; // enabled infringments (i.e. point inserted in pointcloud, if m_infringed_enabled[infringed_bit]==true)
        PointCloud2MsgPublisher m_publisher; // ros publisher of customized pointcloud
    };

    /*
     * @brief class RosMsgpackPublisher implements interface MsgPackExportListenerIF
     * and publishes PointCloud2 messages with msgpack data from multiScan136.
     */
    #if defined __ROS_VERSION && __ROS_VERSION > 1
    class RosMsgpackPublisher : public rclcpp::Node, public sick_scansegment_xd::MsgPackExportListenerIF
    #else
    class RosMsgpackPublisher : public sick_scansegment_xd::MsgPackExportListenerIF
    #endif
    {
    public:

        /*
         * @brief RosMsgpackPublisher constructor
         * @param[in] node_name name of the ros node
         * @param[in] config sick_scansegment_xd configuration, RosMsgpackPublisher uses
         *            config.publish_frame_id: frame id of ros Laserscan messages, default: "world"
         * @param[in] qos quality of service profile for the ros publisher, default: 1
         */
        RosMsgpackPublisher(const std::string& node_name = "sick_scansegment_xd", const sick_scansegment_xd::Config& config = sick_scansegment_xd::Config());

        /*
         * @brief RosMsgpackPublisher destructor
         */
        virtual ~RosMsgpackPublisher();

        /* @brief Determine and initialize all_segments_min/max_deg by LFPangleRangeFilter
        ** host_set_LFPangleRangeFilter = "<enabled> <azimuth_start> <azimuth_stop> <elevation_start> <elevation_stop> <beam_increment>" with azimuth and elevation given in degree
        ** Returns true, if angleRangeFilterSettings are enabled, otherwise false.
        */
        virtual bool initLFPangleRangeFilterSettings(const std::string& host_LFPangleRangeFilter);

        /* @brief Determine and initialize all_segments_elevation_min/max_deg by host_LFPlayerFilter"
        ** host_LFPlayerFilter = "<enabled> <layer0enabled>  <layer1enabled> ...  <layer15enabled>", default: "0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"
        ** Returns true, if layerFilterSettings are enabled, otherwise false.
        */
        virtual bool initLFPlayerFilterSettings(const std::string& host_LFPlayerFilter);

        /*
         * Callback function of MsgPackExportListenerIF. HandleMsgPackData() will be called in MsgPackExporter
         * for each registered listener after msgpack data have been received and converted.
         * This function converts and publishes msgpack data to PointCloud2 messages.
         */
        virtual void HandleMsgPackData(const sick_scansegment_xd::ScanSegmentParserOutput& msgpack_data);

        /*
         * Returns this instance explicitely as an implementation of interface MsgPackExportListenerIF.
         */
        virtual sick_scansegment_xd::MsgPackExportListenerIF* ExportListener(void);

        /*
        * Activates resp. deactivates publishing
        */
        virtual void SetActive(bool active)
        {
            m_active = active;
        }

        /*
        * Returns expected min and max azimuth and elevation angle of a fullframe scan
        */
        virtual void GetFullframeAngleRanges(float& fullframe_azimuth_min_deg, float& fullframe_azimuth_max_deg, float& fullframe_elevation_min_deg, float& fullframe_elevation_max_deg) const
        {
            fullframe_azimuth_min_deg = m_all_segments_azimuth_min_deg;
            fullframe_azimuth_max_deg = m_all_segments_azimuth_max_deg;
            fullframe_elevation_min_deg = m_all_segments_elevation_min_deg;
            fullframe_elevation_max_deg = m_all_segments_elevation_max_deg;
        }

    protected:

        typedef std::map<int,std::map<int,ros_sensor_msgs::LaserScan>> LaserScanMsgMap; // LaserScanMsgMap[echo][layer] := LaserScan message given echo (Multiscan136: max 3 echos) and layer index (Multiscan136: 16 layer)
      
        /*
        * Container to collect all points of 12 segments (12 segments * 30 deg = 360 deg)
        */
        class SegmentPointsCollector
        {
        public:
            SegmentPointsCollector(int telegram_idx = 0) : timestamp_sec(0), timestamp_nsec(0), telegram_cnt(telegram_idx), min_azimuth(0), max_azimuth(0), total_point_count(0), lidar_timestamp_start_microsec(0), lidar_timestamp_stop_microsec(0), lidar_points()
            {
                segment_list.reserve(12);
                telegram_list.reserve(12);
                segment_coverage.clear();
            }
            void appendLidarPoints(const std::vector<std::vector<sick_scansegment_xd::PointXYZRAEI32f>>& points, int32_t segment_idx, int32_t telegram_cnt)
            {
                for (int echoIdx = 0; echoIdx < points.size() && echoIdx < lidar_points.size(); echoIdx++)
                {
                    size_t num_points = points[echoIdx].size();
                    if (num_points > 0)
                    {
                        lidar_points[echoIdx].insert(lidar_points[echoIdx].end(), points[echoIdx].begin(), points[echoIdx].end());
                        assert(lidar_points.size() > echoIdx && lidar_points[echoIdx].size() > 0);
                        if (lidar_timestamp_start_microsec == 0) // first time initialization of lidar start timestamp
                            lidar_timestamp_start_microsec = lidar_points[0].front().lidar_timestamp_microsec;
                        if (lidar_timestamp_stop_microsec == 0) // first time initialization of lidar stop timestamp
                            lidar_timestamp_stop_microsec = lidar_points[0].front().lidar_timestamp_microsec;
                        lidar_timestamp_start_microsec = std::min<uint64_t>(lidar_points[echoIdx].front().lidar_timestamp_microsec, lidar_timestamp_start_microsec); // update lidar start timestamp
                        lidar_timestamp_stop_microsec = std::max<uint64_t>(lidar_points[echoIdx].back().lidar_timestamp_microsec, lidar_timestamp_stop_microsec); // update lidar stop timestamp
                        for (int n = 0; n < num_points; n++)
                        {
                            const sick_scansegment_xd::PointXYZRAEI32f& point = points[echoIdx][n];
                            float elevation_deg = point.elevation * 180.0f / (float)M_PI;
                            float azimuth_fdeg = point.azimuth * 180.0f / (float)M_PI;
                            // Note: The azimuth values of a segment crossing +/-180 degrees are still monotonous, i.e. azimuth values received from the lidar are within -PI to +3*PI.
                            // See compact format specification: "the maximum allowed value range of [-pi, 3*pi] is fully utilized."
                            // To check that the scan points of a fullframe pointclouds covers the complete azimuth range from -180 to +180 degree, we collect the azimuth histogram with 180 deg wrap around.
                            if (azimuth_fdeg > 180.0f)
                                azimuth_fdeg -= 360.0f; // i.e. -180 <= azimuth_deg <= +180
                            int elevation_mdeg = (int)(1000.0f * elevation_deg);
                            int azimuth_ideg = (int)(azimuth_fdeg);
                            segment_coverage[elevation_mdeg][azimuth_ideg] += 1;
                            if (azimuth_fdeg - azimuth_ideg > 0.5f)
                                segment_coverage[elevation_mdeg][azimuth_ideg + 1] += 1;
                            if (azimuth_fdeg - azimuth_ideg < -0.5f)
                                segment_coverage[elevation_mdeg][azimuth_ideg - 1] += 1;
                        }
                    }
                }
                segment_list.push_back(segment_idx);
                telegram_list.push_back(telegram_cnt);
                // for (std::map<int, std::map<int, int>>::iterator segment_coverage_elevation_iter = segment_coverage.begin(); segment_coverage_elevation_iter != segment_coverage.end(); segment_coverage_elevation_iter++)
                // {
                //     const int& elevation_deg = segment_coverage_elevation_iter->first;
                //     std::map<int, int>& azimuth_histogram = segment_coverage_elevation_iter->second;
                //     for (std::map<int, int>::iterator segment_coverage_azimuth_iter = azimuth_histogram.begin(); segment_coverage_azimuth_iter != azimuth_histogram.end(); segment_coverage_azimuth_iter++)
                //     {
                //         const int& azimuth_deg = segment_coverage_azimuth_iter->first;
                //         int cnt = segment_coverage_azimuth_iter->second;
                //         std::cout << "seg[" << elevation_deg << "][" << azimuth_deg << "]=" << cnt << " ";
                //     }
                //     std::cout << std::endl;
                // }
                }
            
            // Returns the last segment index appended by appendLidarPoints
            int32_t lastSegmentIdx()
            {
                return segment_list.empty() ? -1 : segment_list.back();
             }
             // Returns true, if all scans in all elevation angles cover azimuth from all_segments_azimuth_min_deg to all_segments_azimuth_max_deg
             // and all elevation angles cover all_segments_elevation_min_deg to all_segments_elevation_max_deg.
             // Otherwise allSegmentsCovered returns false.
             bool allSegmentsCovered(float all_segments_azimuth_min_deg, float all_segments_azimuth_max_deg, float all_segments_elevation_min_deg, float all_segments_elevation_max_deg)
             {
                float elevation_deg_min = 999, elevation_deg_max = -999;
                for (std::map<int, std::map<int, int>>::iterator segment_coverage_elevation_iter = segment_coverage.begin(); segment_coverage_elevation_iter != segment_coverage.end(); segment_coverage_elevation_iter++)
                {
                    int azimuth_deg_first = 999, azimuth_deg_last = -999;
                    float elevation_deg = 0.001f * (segment_coverage_elevation_iter->first);
                    elevation_deg_min = std::min<float>(elevation_deg, elevation_deg_min);
                    elevation_deg_max = std::max<float>(elevation_deg, elevation_deg_max);
                    std::map<int, int>& azimuth_histogram = segment_coverage_elevation_iter->second;
                    for (std::map<int, int>::iterator segment_coverage_azimuth_iter = azimuth_histogram.begin(); segment_coverage_azimuth_iter != azimuth_histogram.end(); segment_coverage_azimuth_iter++)
                    {
                        const int& azimuth_deg = segment_coverage_azimuth_iter->first;
                        const int& azimuth_cnt = segment_coverage_azimuth_iter->second;
                        if (azimuth_cnt > 0 && azimuth_deg >= (int)all_segments_azimuth_min_deg && azimuth_deg <= (int)all_segments_azimuth_max_deg)
                        {
                            azimuth_deg_first = std::min<int>(azimuth_deg_first, azimuth_deg);
                            azimuth_deg_last = std::max<int>(azimuth_deg_last, azimuth_deg);
                        }
                    }
                    bool azimuth_success = (azimuth_deg_last - azimuth_deg_first + 1 >= all_segments_azimuth_max_deg - all_segments_azimuth_min_deg);
                    // Check azimuth_histogram[azimuth_deg] > 0 for all azimuth_deg in range (azimuth_deg_first, azimuth_deg_last)
                    for(int azimuth_deg = azimuth_deg_first; azimuth_success && azimuth_deg <= azimuth_deg_last; azimuth_deg++)
                    {
                        if (azimuth_histogram[azimuth_deg] <= 0)
                            azimuth_success = false;
                    }
                    // ROS_INFO_STREAM("    SegmentPointsCollector::allSegmentsCovered(): lastSegmentIdx=" << lastSegmentIdx() << ", total_point_count=" << total_point_count 
                    //      << ", cur_elevation=" << elevation_deg << ", azimuth=(" << azimuth_deg_first << "," << azimuth_deg_last << ")"
                    //      << ", azimuth_range=(" << all_segments_azimuth_min_deg << "," << all_segments_azimuth_max_deg << "), azimuth_success=" << azimuth_success);
                    if (!azimuth_success)
                        return false;
                }
                bool elevation_success = (elevation_deg_max - elevation_deg_min + 1 >= all_segments_elevation_max_deg - all_segments_elevation_min_deg);
                // ROS_INFO_STREAM("    SegmentPointsCollector::allSegmentsCovered(): lastSegmentIdx=" << lastSegmentIdx() << ", total_point_count=" << total_point_count 
                //     << ", elevation_deg_min=" << elevation_deg_min << ", elevation_deg_max=" << elevation_deg_max
                //     << ", all_segments_elevation_min_deg=" << all_segments_elevation_min_deg << ", all_segments_elevation_max_deg=" << all_segments_elevation_max_deg
                //     << ", elevation_success=" << elevation_success);
                if (!elevation_success)
                    return false;
                return true; // all scans in all elevation angles cover azimuth from all_segments_azimuth_min_deg to all_segments_azimuth_max_deg
             }

            // Returns the number of echos
            int numEchos(void) const { return (int)lidar_points.size(); }

            uint32_t timestamp_sec;   // seconds part of timestamp of the first segment (system time)
            uint32_t timestamp_nsec;  // nanoseconds part of timestamp of the first segment (system time)
            // int32_t segment_count; // number of segments collected
            int32_t telegram_cnt;     // telegram counter (must be continuously incremented) 
            float min_azimuth;        // min azimuth of all points in radians
            float max_azimuth;        // max azimuth of all points in radians
            size_t total_point_count; // total number of points in all segments
            uint64_t lidar_timestamp_start_microsec; // lidar start timestamp in microseconds
            uint64_t lidar_timestamp_stop_microsec;  // lidar stop timestamp in microseconds
            std::vector<std::vector<sick_scansegment_xd::PointXYZRAEI32f>> lidar_points; // list of PointXYZRAEI32f: lidar_points[echoIdx] are the points of all segments of an echo (idx echoIdx)
            std::vector<int32_t> segment_list; // list of all collected segment indices
            std::vector<int32_t> telegram_list; // list of all collected telegram counters
            std::map<int, std::map<int, int>> segment_coverage; // (elevation,azimuth) histogram: segment_coverage[elevation][azimuth] > 0: elevation in mdeg and azimuth in deg covered (otherwise no hits)
        };

        /*
        * Converts the lidarpoints to a customized PointCloud2Msg containing configured fields (e.g. x, y, z, i, range, azimuth, elevation, layer, echo, reflector).
        * @param[in] timestamp_sec seconds part of timestamp (system time)
        * @param[in] timestamp_nsec nanoseconds part of timestamp (system time)
        * @param[in] lidar_timestamp_start_microsec lidar start timestamp in microseconds (lidar ticks)
        * @param[in] lidar_points list of PointXYZRAEI32f: lidar_points[echoIdx] are the points of one echo
        * @param[in] pointcloud_cfg configuration of customized pointcloud
        * @param[out] pointcloud_msg customized pointcloud message
        */
        void convertPointsToCustomizedFieldsCloud(uint32_t timestamp_sec, uint32_t timestamp_nsec, uint64_t lidar_timestamp_start_microsec,
            const std::vector<std::vector<sick_scansegment_xd::PointXYZRAEI32f>>& lidar_points, CustomPointCloudConfiguration& pointcloud_cfg, PointCloud2Msg& pointcloud_msg);

        void convertPointsToLaserscanMsg(uint32_t timestamp_sec, uint32_t timestamp_nsec, const std::vector<std::vector<sick_scansegment_xd::PointXYZRAEI32f>>& lidar_points, size_t total_point_count, LaserScanMsgMap& laser_scan_msg_map, const std::string& frame_id, bool is_fullframe);

        /** Shortcut to publish a PointCloud2Msg */
        void publishPointCloud2Msg(rosNodePtr node, PointCloud2MsgPublisher& publisher, PointCloud2Msg& pointcloud_msg, int32_t num_echos, int32_t segment_idx, int coordinate_notation, const std::string& topic);

        /** Shortcut to publish Laserscan messages */
        void publishLaserScanMsg(rosNodePtr node, LaserscanMsgPublisher& laserscan_publisher, LaserScanMsgMap& laser_scan_msg_map, int32_t num_echos, int32_t segment_idx);

        /** Shortcut to publish a PointCloud2Msg */
        // void publish(rosNodePtr node, PointCloud2MsgPublisher& publisher, PointCloud2Msg& pointcloud_msg, PointCloud2Msg& pointcloud_msg_polar, 
        //     LaserscanMsgPublisher& laserscan_publisher, LaserScanMsgMap& laser_scan_msg_map, int32_t num_echos, int32_t segment_idx);

        /** Prints (elevation,azimuth) values of all lidar points */
        std::string printElevationAzimuthTable(const std::vector<std::vector<sick_scansegment_xd::PointXYZRAEI32f>>& lidar_points);

        /** Prints (elevation,azimuth) values of the coverage table of collected lidar points */
        std::string printCoverageTable(const std::map<int, std::map<int, int>>& elevation_azimuth_histograms);

        /** Checks if a PointCloud2Msg has all required fields */
        bool hasPointcloudRequiredFields(const PointCloud2Msg& pointcloud_msg, const std::unordered_set<std::string>& required_fields) const;

        bool m_active; // activate publishing
        rosNodePtr m_node; // ros node handle
        std::string m_frame_id;       // frame id of ros Laserscan messages, default: "world"
        std::string m_imu_frame_id;   // frame_if of ros IMU messages, default: "sick_imu"
        float m_all_segments_azimuth_min_deg = -180;  // angle range covering all segments: all segments pointcloud on topic publish_topic_all_segments is published, 
        float m_all_segments_azimuth_max_deg = +180;  // if received segments cover azimuth angle range from m_all_segments_azimuth_min_deg to m_all_segments_azimuth_max_deg. -180...+180 for multiScan136 (360 deg fullscan)
        float m_all_segments_elevation_min_deg = 0;   // angle range covering all segments: all segments pointcloud on topic publish_topic_all_segments is published, 
        float m_all_segments_elevation_max_deg = 0;   // if received segments cover elevation angle range from m_all_segments_elevation_min_deg to m_all_segments_elevation_max_deg.
        int m_host_FREchoFilter;                      // configuration from launchfile: Optionally set FREchoFilter with 0 for FIRST_ECHO (EchoCount=1), 1 for ALL_ECHOS (EchoCount=3), or 2 for LAST_ECHO (EchoCount=1)
        bool m_host_set_FREchoFilter;                 // configuration from launchfile: If true, FREchoFilter is set at startup (default: false)
        SegmentPointsCollector m_points_collector;    // collects all points of 12 segments (12 segments * 30 deg = 360 deg)
        LaserscanMsgPublisher m_publisher_laserscan_360;     // ros publisher to publish LaserScan messages of all segments (360 degree)
        LaserscanMsgPublisher m_publisher_laserscan_segment; // ros publisher to publish LaserScan messages of the current segment
        ImuMsgPublisher m_publisher_imu;                     // ros publisher to publish Imu messages
        bool m_publisher_imu_initialized = false;            // imu messages enabled, ros publisher for Imu messages initialized
        double m_scan_time = 0;                              // scan_time = 1 / scan_frequency = time for a full 360-degree rotation of the sensor
        std::vector<int> m_laserscan_layer_filter;           // Configuration of laserscan messages (ROS only), activate/deactivate laserscan messages for each layer
	    std::vector<CustomPointCloudConfiguration> m_custom_pointclouds_cfg; // Configuration of customized pointclouds

    };  // class RosMsgpackPublisher

}   // namespace sick_scansegment_xd
#endif // __SICK_SCANSEGMENT_XD_ROS_MSGPACK_PUBLISHER_H
