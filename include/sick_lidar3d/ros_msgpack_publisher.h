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
#ifndef __SICK_LIDAR3D_ROS_MSGPACK_PUBLISHER_H
#define __SICK_LIDAR3D_ROS_MSGPACK_PUBLISHER_H
#if defined __ROS_VERSION && __ROS_VERSION > 0

#include "sick_lidar3d/config.h"
#include "sick_lidar3d/msgpack_exporter.h"

namespace sick_lidar3d
{
    /*
     * class PointXYZI32f is just a container for a single multiScan136 point in cartesian coordinates x, y, z and intensity i
     */
    class PointXYZI32f
    {
    public:
        PointXYZI32f() : x(0), y(0), z(0), i(0) {}
        PointXYZI32f(float _x, float _y, float _z, float _i) : x(_x), y(_y), z(_z), i(_i) {}
        float x;
        float y;
        float z;
        float i;
    };
  
    /*
     * @brief class RosMsgpackPublisher implements interface MsgPackExportListenerIF
     * and publishes PointCloud2 messages with msgpack data from multiScan136.
     */
    #if defined __ROS_VERSION && __ROS_VERSION > 1
    class RosMsgpackPublisher : public rclcpp::Node, public sick_lidar3d::MsgPackExportListenerIF
    #else
    class RosMsgpackPublisher : public sick_lidar3d::MsgPackExportListenerIF
    #endif
    {
    public:

        /*
         * @brief RosMsgpackPublisher constructor
         * @param[in] node_name name of the ros node
         * @param[in] config sick_lidar3d configuration, RosMsgpackPublisher uses
         *            config.publish_topic: ros topic to publish received msgpack data converted to PointCloud2 messages, default: "/cloud"
         *            config.publish_topic_all_segments: ros topic to publish PointCloud2 messages of all segments (360 deg), default: "/cloud_360"
         *            config.segment_count: number of expected segments in 360 degree, multiScan136: 12 segments, 30 deg per segment
         *            config.publish_frame_id: frame id of ros PointCloud2 messages, default: "world"
         *            config.exit_on_keys_esc_q: true: shutdown and exit lidar3d_mrs100_recv after pressing key ESC, 'q' or 'Q'
         * @param[in] qos quality of service profile for the ros publisher, default: 1
         */
        RosMsgpackPublisher(const std::string& node_name = "lidar3d_mrs100_recv", const sick_lidar3d::Config& config = sick_lidar3d::Config(), const rosQoS& qos = 1);

        /*
         * @brief RosMsgpackPublisher destructor
         * @param[in] node_name name of the ros node
         * @param[in] topic ros topic to publish received msgpack data converted top PointCloud2 messages, default: "/cloud"
         * @param[in] qos quality of service profile for the ros publisher, default: 1
         * @param[in] exit_on_keys_esc_q true: shutdown and exit lidar3d_mrs100_recv after pressing key ESC, 'q' or 'Q'
         */
        virtual ~RosMsgpackPublisher();

        /*
         * Callback function of MsgPackExportListenerIF. HandleMsgPackData() will be called in MsgPackExporter
         * for each registered listener after msgpack data have been received and converted.
         * This function converts and publishes msgpack data to PointCloud2 messages.
         */
        virtual void HandleMsgPackData(const sick_lidar3d::MsgPackParserOutput& msgpack_data);

        /*
         * Returns this instance explicitely as an implementation of interface MsgPackExportListenerIF.
         */
        virtual sick_lidar3d::MsgPackExportListenerIF* ExportListener(void);

        /*
         * Sets min and max azimuth of a full scan in radians, default: -M_PI, +M_PI
         */
        virtual void SetFullScanAzimuthRange(float min_azimuth = -M_PI, float max_azimuth = +M_PI)
        {
            m_min_azimuth = min_azimuth;
            m_max_azimuth = max_azimuth;
        }

        /*
        * Activates resp. deactivates publishing
        */
        virtual void SetActive(bool active)
        {
            m_active = active;
        }

    protected:
      
         /*
          * Container to collect all points of 12 segments (12 segments * 30 deg = 360 deg)
          */
         class SegmentPointsCollector
         {
         public:
             SegmentPointsCollector(int segment_idx = 0, int telegram_idx = 0) : timestamp_sec(0), timestamp_nsec(0), segment_count(segment_idx), telegram_cnt(telegram_idx), min_azimuth(0), max_azimuth(0), total_point_count(0), lidar_points()
             {
                 segment_list.reserve(12);
                 telegram_list.reserve(12);
             }
             void appendLidarPoints(const std::vector<std::vector<sick_lidar3d::PointXYZI32f>>& points, int32_t segment_idx, int32_t telegram_cnt)
             {
                 for (int echoIdx = 0; echoIdx < points.size() && echoIdx < lidar_points.size(); echoIdx++)
                     lidar_points[echoIdx].insert(lidar_points[echoIdx].end(), points[echoIdx].begin(), points[echoIdx].end());
                 segment_list.push_back(segment_idx);
                 telegram_list.push_back(telegram_cnt);
             }

             uint32_t timestamp_sec;   // seconds part of timestamp of the first segment
             uint32_t timestamp_nsec;  // nanoseconds part of timestamp of the first segment
             int32_t segment_count;    // number of segments collected
             int32_t telegram_cnt;     // telegram counter (must be continuously incremented) 
             float min_azimuth;        // min azimuth of all points in radians
             float max_azimuth;        // max azimuth of all points in radians
             size_t total_point_count; // total number of points in all segments
             std::vector<std::vector<sick_lidar3d::PointXYZI32f>> lidar_points; // list of PointXYZI32f: lidar_points[echoIdx] are the points of all segments of an echo (idx echoIdx)
             std::vector<int32_t> segment_list; // list of all collected segment indices
             std::vector<int32_t> telegram_list; // list of all collected telegram counters
         };
  
         /*
          * Converts the lidarpoints from a msgpack to a PointCloud2Msg.
          * @param[in] timestamp_sec seconds part of timestamp
          * @param[in] timestamp_nsec  nanoseconds part of timestamp
          * @param[in] lidar_points list of PointXYZI32f: lidar_points[echoIdx] are the points of one echo
          * @param[in] total_point_count total number of points in all echos
          * @param[in] echo_count number of echos
          * @param[out] pointcloud_msg PointCloud2Msg result
          */
         void convertPointsToCloud(uint32_t timestamp_sec, uint32_t timestamp_nsec, const std::vector<std::vector<sick_lidar3d::PointXYZI32f>>& lidar_points,
            size_t total_point_count, PointCloud2Msg& pointcloud_msg);
      
        /*
         * Shortcut to publish a PointCloud2Msg
         */
        void publish(PointCloud2MsgPublisher& publisher, PointCloud2Msg& pointcloud_msg);

        bool m_active; // activate publishing
        bool m_exit_on_keys_esc_q; // true: shutdown and exit lidar3d_mrs100_recv after pressing key ESC, 'q' or 'Q'
        std::string m_frame_id;    // frame id of ros PointCloud2 messages, default: "world"
        rosClock m_ros_clock;      // clock for message timestamps
        int m_segment_count = 12;  // number of expected segments in 360 degree, multiScan136: 12 segments, 30 deg per segment
        float m_min_azimuth; // min azimuth of a full scan in radians, default: -M_PI
        float m_max_azimuth; // max azimuth of a full scan in radians, default: +M_PI
        SegmentPointsCollector m_points_collector; // collects all points of 12 segments (12 segments * 30 deg = 360 deg)
        std::string m_publish_topic; // ros topic to publish received msgpack data converted to PointCloud2 messages, default: "/cloud"
        std::string m_publish_topic_all_segments; // ros topic to publish PointCloud2 messages of all segments (360 deg), default: "/cloud_360"
        PointCloud2MsgPublisher m_publisher_cur_segment; // ros publisher to publish PointCloud2 message of the current segment
        PointCloud2MsgPublisher m_publisher_all_segments; // ros publisher to publish PointCloud2 message of all segments (360 degree)

    };  // class RosMsgpackPublisher
}   // namespace sick_lidar3d
#endif // __ROS_VERSION && __ROS_VERSION > 0
#endif // __SICK_LIDAR3D_ROS_MSGPACK_PUBLISHER_H
