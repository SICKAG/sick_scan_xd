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

namespace sick_scansegment_xd
{
    /*
     * class PointXYZRAEI32f is just a container for a single multiScan136 point in cartesian coordinates (x, y, z), polar coordinates (range, azimuth, elevation), and intensity i
     */
    class PointXYZRAEI32f
    {
    public:
        PointXYZRAEI32f() : x(0), y(0), z(0), range(0), azimuth(0), elevation(0), i(0), layer(0), echo(0) {}
        PointXYZRAEI32f(float _x, float _y, float _z, float _range, float _azimuth, float _elevation, float _i, int _layer, int _echo) 
            : x(_x), y(_y), z(_z), range(_range), azimuth(_azimuth), elevation(_elevation), i(_i), layer(_layer), echo(_echo) {}
        float x;         // cartesian x coordinate in meter
        float y;         // cartesian y coordinate in meter
        float z;         // cartesian z coordinate in meter
        float range;     // polar coordinate range in meter
        float azimuth;   // polar coordinate azimuth in radians
        float elevation; // polar coordinate elevation in radians
        float i;         // intensity
        int layer;       // group index (layer), 0 <= layer < 16 for multiScan136
        int echo;        // echo index, 0 <= echo < 3 for multiScan136

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
         *            config.publish_topic: ros topic to publish received msgpack data converted to PointCloud2 messages, default: "/cloud"
         *            config.publish_topic_all_segments: ros topic to publish PointCloud2 messages of all segments (360 deg), default: "/cloud_fullframe"
         *            config.all_segments_min_deg, config.all_segments_min_deg: angle range covering all segments: all segments pointcloud on topic publish_topic_all_segments is published, 
         *            if received segments cover angle range from all_segments_min_deg to all_segments_max_deg. -180...+180 for MultiScan136 (360 deg fullscan)
         *            config.publish_frame_id: frame id of ros PointCloud2 messages, default: "world"
         * @param[in] qos quality of service profile for the ros publisher, default: 1
         */
        RosMsgpackPublisher(const std::string& node_name = "sick_scansegment_xd", const sick_scansegment_xd::Config& config = sick_scansegment_xd::Config());

        /*
         * @brief RosMsgpackPublisher destructor
         */
        virtual ~RosMsgpackPublisher();

        /*
         * Callback function of MsgPackExportListenerIF. HandleMsgPackData() will be called in MsgPackExporter
         * for each registered listener after msgpack data have been received and converted.
         * This function converts and publishes msgpack data to PointCloud2 messages.
         */
        virtual void HandleMsgPackData(const sick_scansegment_xd::MsgPackParserOutput& msgpack_data);

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

    protected:

        typedef std::map<int,std::map<int,ros_sensor_msgs::LaserScan>> LaserScanMsgMap; // LaserScanMsgMap[echo][layer] := LaserScan message given echo (Multiscan136: max 3 echos) and layer index (Multiscan136: 16 layer)
      
         /*
          * Container to collect all points of 12 segments (12 segments * 30 deg = 360 deg)
          */
         class SegmentPointsCollector
         {
         public:
             SegmentPointsCollector(int telegram_idx = 0) : timestamp_sec(0), timestamp_nsec(0), telegram_cnt(telegram_idx), min_azimuth(0), max_azimuth(0), total_point_count(0), lidar_points()
             {
                 segment_list.reserve(12);
                 telegram_list.reserve(12);
                segment_coverage.clear();
             }
             void appendLidarPoints(const std::vector<std::vector<sick_scansegment_xd::PointXYZRAEI32f>>& points, int32_t segment_idx, int32_t telegram_cnt)
             {
                 for (int echoIdx = 0; echoIdx < points.size() && echoIdx < lidar_points.size(); echoIdx++)
                {
                     lidar_points[echoIdx].insert(lidar_points[echoIdx].end(), points[echoIdx].begin(), points[echoIdx].end());
                    for (int n = 0; n < points[echoIdx].size(); n++)
                    {
                        const sick_scansegment_xd::PointXYZRAEI32f& point = points[echoIdx][n];
                        float elevation_deg = point.elevation * 180.0f / (float)M_PI;
                        float azimuth_fdeg = point.azimuth * 180.0f / (float)M_PI;
                        int elevation_mdeg = (int)(1000.0f * elevation_deg);
                        int azimuth_ideg = (int)(azimuth_fdeg);
                        segment_coverage[elevation_mdeg][azimuth_ideg] += 1;
                        if (azimuth_fdeg - azimuth_ideg > 0.5f)
                            segment_coverage[elevation_mdeg][azimuth_ideg + 1] += 1;
                        if (azimuth_fdeg - azimuth_ideg < -0.5f)
                            segment_coverage[elevation_mdeg][azimuth_ideg - 1] += 1;
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
             // Returns true, if all scans in all elevation angles cover azimuth from all_segments_min_deg to all_segments_max_deg (otherwise false)
             bool allSegmentsCovered(float all_segments_min_deg, float all_segments_max_deg)
             {
                int azimuth_min = (int)all_segments_min_deg;
                int azimuth_max = (int)all_segments_max_deg;
                for (std::map<int, std::map<int, int>>::iterator segment_coverage_elevation_iter = segment_coverage.begin(); segment_coverage_elevation_iter != segment_coverage.end(); segment_coverage_elevation_iter++)
                {
                    int azimuth_deg_first = 999, azimuth_deg_last = -999;
                    const int& elevation_deg = segment_coverage_elevation_iter->first;
                    std::map<int, int>& azimuth_histogram = segment_coverage_elevation_iter->second;
                    for (std::map<int, int>::iterator segment_coverage_azimuth_iter = azimuth_histogram.begin(); segment_coverage_azimuth_iter != azimuth_histogram.end(); segment_coverage_azimuth_iter++)
                    {
                        const int& azimuth_deg = segment_coverage_azimuth_iter->first;
                        int azimuth_cnt = segment_coverage_azimuth_iter->second;
                        if (azimuth_cnt > 0)
                            azimuth_deg_first = MIN(azimuth_deg_first, azimuth_deg);
                    }
                    for(azimuth_deg_last = azimuth_deg_first; azimuth_deg_last <= azimuth_deg_first + 360; azimuth_deg_last++)
                    {
                        if (azimuth_histogram[azimuth_deg_last] <= 0)
                            break;
                    }
                    bool success = (azimuth_deg_last - azimuth_deg_first >= all_segments_max_deg - all_segments_min_deg);
                    // ROS_INFO_STREAM("SegmentPointsCollector::allSegmentsCovered(): lastSegmentIdx=" << lastSegmentIdx() << ", total_point_count=" << total_point_count 
                    //     << ", cur_elevation=" << elevation_deg << ", azimuth=(" << azimuth_deg_first << "," << azimuth_deg_last << "), " << "ret=" << success);
                    if (!success)
                        return false;
                }
                return true; // all scans in all elevation angles cover azimuth from all_segments_min_deg to all_segments_max_deg
             }

             uint32_t timestamp_sec;   // seconds part of timestamp of the first segment
             uint32_t timestamp_nsec;  // nanoseconds part of timestamp of the first segment
             // int32_t segment_count; // number of segments collected
             int32_t telegram_cnt;     // telegram counter (must be continuously incremented) 
             float min_azimuth;        // min azimuth of all points in radians
             float max_azimuth;        // max azimuth of all points in radians
             size_t total_point_count; // total number of points in all segments
             std::vector<std::vector<sick_scansegment_xd::PointXYZRAEI32f>> lidar_points; // list of PointXYZRAEI32f: lidar_points[echoIdx] are the points of all segments of an echo (idx echoIdx)
             std::vector<int32_t> segment_list; // list of all collected segment indices
             std::vector<int32_t> telegram_list; // list of all collected telegram counters
             std::map<int, std::map<int, int>> segment_coverage; // segment histogram: segment_coverage[elevation][azimuth] > 0: elevation in mdeg and azimuth in deg covered (otherwise no hits)
         };
  
         /*
          * Converts the lidarpoints from a msgpack to a PointCloud2Msg and to LaserScan messages for each layer.
          * Note: For performance reasons, LaserScan messages are not created for the collected 360-degree scans (i.e. is_cloud_360 is true).
          * @param[in] timestamp_sec seconds part of timestamp
          * @param[in] timestamp_nsec  nanoseconds part of timestamp
          * @param[in] last_timestamp_sec seconds part of last timestamp
          * @param[in] last_timestamp_nsec  nanoseconds part of last timestamp
          * @param[in] lidar_points list of PointXYZRAEI32f: lidar_points[echoIdx] are the points of one echo
          * @param[in] total_point_count total number of points in all echos
          * @param[in] echo_count number of echos
          * @param[out] pointcloud_msg cartesian pointcloud message
          * @param[out] pointcloud_msg_polar polar pointcloud message
          * @param[out] laser_scan_msg_map laserscan message: ros_sensor_msgs::LaserScan for each echo and layer is laser_scan_msg_map[echo][layer]
          */
         void convertPointsToCloud(uint32_t timestamp_sec, uint32_t timestamp_nsec, const std::vector<std::vector<sick_scansegment_xd::PointXYZRAEI32f>>& lidar_points, size_t total_point_count, 
            PointCloud2Msg& pointcloud_msg, PointCloud2Msg& pointcloud_msg_polar, LaserScanMsgMap& laser_scan_msg_map, bool is_cloud_360);
      
        /*
         * Shortcut to publish a PointCloud2Msg
         */
        void publish(rosNodePtr node, PointCloud2MsgPublisher& publisher, PointCloud2Msg& pointcloud_msg, PointCloud2Msg& pointcloud_msg_polar, 
            LaserscanMsgPublisher& laserscan_publisher, LaserScanMsgMap& laser_scan_msg_map, int32_t num_echos, int32_t segment_idx);

        bool m_active; // activate publishing
        rosNodePtr m_node; // ros node handle
        std::string m_frame_id;    // frame id of ros PointCloud2 messages, default: "world"
        // int m_segment_count = 12;  // number of expected segments in 360 degree, multiScan136: 12 segments, 30 deg per segment
        float m_all_segments_min_deg = -180; // angle range covering all segments: all segments pointcloud on topic publish_topic_all_segments is published, 
        float m_all_segments_max_deg = +180; // if received segments cover angle range from all_segments_min_deg to all_segments_max_deg. -180...+180 for multiScan136 (360 deg fullscan)
        SegmentPointsCollector m_points_collector; // collects all points of 12 segments (12 segments * 30 deg = 360 deg)
        std::string m_publish_topic;                         // ros topic to publish received msgpack data converted to PointCloud2 messages, default: "/cloud"
        std::string m_publish_topic_all_segments;            // ros topic to publish PointCloud2 messages of all segments (360 deg), default: "/cloud_fullframe"
        PointCloud2MsgPublisher m_publisher_cur_segment;     // ros publisher to publish PointCloud2 messages of the current segment
        PointCloud2MsgPublisher m_publisher_all_segments;    // ros publisher to publish PointCloud2 messages of all segments (360 degree)
        LaserscanMsgPublisher m_publisher_laserscan_segment; // ros publisher to publish LaserScan messages of the current segment
        LaserscanMsgPublisher m_publisher_laserscan_360;     // ros publisher to publish LaserScan messages of all segments (360 degree)
        double m_scan_time = 0;                              // scan_time = 1 / scan_frequency = time for a full 360-degree rotation of the sensor
        std::vector<int> m_laserscan_layer_filter;           // Configuration of laserscan messages (ROS only), activate/deactivate laserscan messages for each layer


    };  // class RosMsgpackPublisher

}   // namespace sick_scansegment_xd
#endif // __SICK_SCANSEGMENT_XD_ROS_MSGPACK_PUBLISHER_H
