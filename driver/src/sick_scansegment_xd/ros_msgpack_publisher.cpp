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
 *
 */

#include <sick_scan/sick_generic_callback.h>
#include "sick_scansegment_xd/ros_msgpack_publisher.h"
#if defined ROSSIMU
#include "sick_scan/pointcloud_utils.h"
#endif

/*
 * @brief RosMsgpackPublisher constructor
 * @param[in] node_name name of the ros node
 * @param[in] config sick_scansegment_xd configuration, RosMsgpackPublisher uses
 *            config.publish_topic: ros topic to publish received msgpack data converted to PointCloud2 messages, default: "/cloud"
 *            config.publish_topic_all_segments: ros topic to publish PointCloud2 messages of all segments (360 deg), default: "/cloud_fullframe"
 *            config.all_segments_min_deg, config.all_segments_min_deg: angle range covering all segments: all segments pointcloud on topic publish_topic_all_segments is published, 
 *            if received segments cover angle range from all_segments_min_deg to all_segments_max_deg. -180...+180 for multiScan136 (360 deg fullscan)
 *            config.publish_frame_id: frame id of ros PointCloud2 messages, default: "world"
 * @param[in] qos quality of service profile for the ros publisher, default: 1
 */
sick_scansegment_xd::RosMsgpackPublisher::RosMsgpackPublisher(const std::string& node_name, const sick_scansegment_xd::Config& config)
#if defined __ROS_VERSION && __ROS_VERSION > 1
	: Node(node_name)
#endif
{
	m_active = false;
    m_frame_id = config.publish_frame_id;
	m_publish_topic = config.publish_topic;
	m_publish_topic_all_segments = config.publish_topic_all_segments;
	// m_segment_count = config.segment_count;
	m_all_segments_min_deg = (float)config.all_segments_min_deg;
    m_all_segments_max_deg = (float)config.all_segments_max_deg;
	m_node = config.node;
	m_laserscan_layer_filter = config.laserscan_layer_filter;

#if defined __ROS_VERSION && __ROS_VERSION > 1 // ROS-2 publisher
    rosQoS qos = rclcpp::SystemDefaultsQoS();
    QoSConverter qos_converter;
    int qos_val = -1;
    rosDeclareParam(m_node, "ros_qos", qos_val);
    rosGetParam(m_node, "ros_qos", qos_val);
    if (qos_val >= 0)
        qos = qos_converter.convert(qos_val);
	if(m_publish_topic != "")
	{
	    m_publisher_cur_segment = create_publisher<PointCloud2Msg>(m_publish_topic, qos);
		ROS_INFO_STREAM("RosMsgpackPublisher: publishing PointCloud2 messages on topic \"" << m_publisher_cur_segment->get_topic_name() << "\"");
	    m_publisher_laserscan_segment = create_publisher<ros_sensor_msgs::LaserScan>("~/scan_segment", qos);
		ROS_INFO_STREAM("RosMsgpackPublisher: publishing LaserScan segment messages on topic \"" << m_publisher_laserscan_segment->get_topic_name() << "\"");
	}
	if(m_publish_topic_all_segments != "")
	{
	    m_publisher_all_segments = create_publisher<PointCloud2Msg>(m_publish_topic_all_segments, qos);
	    // m_publisher_laserscan_360 = create_publisher<ros_sensor_msgs::LaserScan>("scan_360", qos);
	}
#elif defined __ROS_VERSION && __ROS_VERSION > 0 // ROS-1 publisher
    int qos = 16 * 12 * 3; // 16 layers, 12 segments, 3 echos
	int qos_val = -1;
    rosDeclareParam(m_node, "ros_qos", qos_val);
    rosGetParam(m_node, "ros_qos", qos_val);
    if (qos_val >= 0)
        qos = qos_val;
	if(m_publish_topic != "")
	{
		m_publisher_cur_segment = m_node->advertise<PointCloud2Msg>(m_publish_topic, qos);
	    m_publisher_laserscan_segment = m_node->advertise<ros_sensor_msgs::LaserScan>("scan_segment", qos);
	}
	if(m_publish_topic_all_segments != "")
	{
		m_publisher_all_segments = m_node->advertise<PointCloud2Msg>(m_publish_topic_all_segments, qos);
	    // m_publisher_laserscan_360 = m_node->advertise<ros_sensor_msgs::LaserScan>("scan_360", qos);
	}
#endif
}

/*
 * @brief RosMsgpackPublisher destructor
 */
sick_scansegment_xd::RosMsgpackPublisher::~RosMsgpackPublisher()
{
}

/*
 * Shortcut to publish a PointCloud2Msg
 */
void sick_scansegment_xd::RosMsgpackPublisher::publish(rosNodePtr node, PointCloud2MsgPublisher& publisher, PointCloud2Msg& pointcloud_msg, PointCloud2Msg& pointcloud_msg_polar, 
    LaserscanMsgPublisher& laserscan_publisher, LaserScanMsgMap& laser_scan_msg_map, int32_t num_echos, int32_t segment_idx)
{
    sick_scan::PointCloud2withEcho cloud_msg_with_echo(&pointcloud_msg, num_echos, segment_idx);
    sick_scan::PointCloud2withEcho cloud_msg_polar_with_echo(&pointcloud_msg_polar, num_echos, segment_idx);
    notifyPolarPointcloudListener(node, &cloud_msg_polar_with_echo);
    notifyCartesianPointcloudListener(node, &cloud_msg_with_echo);
#if defined __ROS_VERSION && __ROS_VERSION > 1
	publisher->publish(pointcloud_msg);
#elif defined __ROS_VERSION && __ROS_VERSION > 0
	publisher.publish(pointcloud_msg);
#elif defined ROSSIMU
    // plotPointCloud(pointcloud_msg);
#endif
	for(LaserScanMsgMap::iterator laser_scan_echo_iter = laser_scan_msg_map.begin(); laser_scan_echo_iter != laser_scan_msg_map.end(); laser_scan_echo_iter++)
	{
		int echo_idx = laser_scan_echo_iter->first;
		std::map<int,ros_sensor_msgs::LaserScan>& laser_scan_layer_map = laser_scan_echo_iter->second;
		for(std::map<int,ros_sensor_msgs::LaserScan>::iterator laser_scan_msg_iter = laser_scan_layer_map.begin(); laser_scan_msg_iter != laser_scan_layer_map.end(); laser_scan_msg_iter++)
		{
			int layer_idx = laser_scan_msg_iter->first;
			ros_sensor_msgs::LaserScan& laser_scan_msg = laser_scan_msg_iter->second;
			if (laser_scan_msg.ranges.size() > 0)
			{
#if defined __ROS_VERSION && __ROS_VERSION > 1
				laserscan_publisher->publish(laser_scan_msg);
#elif defined __ROS_VERSION && __ROS_VERSION > 0
				laserscan_publisher.publish(laser_scan_msg);
#endif
				// ROS_INFO_STREAM("publish LaserScan: segment_idx:" << segment_idx << ", frame_id:" << laser_scan_msg.header.frame_id << ", num_points:" << laser_scan_msg.ranges.size() << ", angle_min:" << (laser_scan_msg.angle_min * 180.0 / M_PI) << ", angle_max:" << (laser_scan_msg.angle_max * 180.0 / M_PI));
				// usleep(1000);
			}
		}
	}
}

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
void sick_scansegment_xd::RosMsgpackPublisher::convertPointsToCloud(uint32_t timestamp_sec, uint32_t timestamp_nsec, const std::vector<std::vector<sick_scansegment_xd::PointXYZRAEI32f>>& lidar_points, size_t total_point_count, 
  PointCloud2Msg& pointcloud_msg, PointCloud2Msg& pointcloud_msg_polar, LaserScanMsgMap& laser_scan_msg_map, bool is_cloud_360)
{
  // set pointcloud header
  // pointcloud_msg.header.stamp = rosTimeNow();
  pointcloud_msg.header.stamp.sec = timestamp_sec;
#if defined __ROS_VERSION && __ROS_VERSION > 1
  pointcloud_msg.header.stamp.nanosec = timestamp_nsec;
#elif defined __ROS_VERSION && __ROS_VERSION > 0
  pointcloud_msg.header.stamp.nsec = timestamp_nsec;
#endif
  pointcloud_msg.header.frame_id = m_frame_id;
  
  // set pointcloud field properties
  int numChannels = 4; // "x", "y", "z", "i"
  std::string channelId[] = { "x", "y", "z", "i" };
  pointcloud_msg.height = 1;
  pointcloud_msg.width = (uint32_t)total_point_count;
  pointcloud_msg.is_bigendian = false;
  pointcloud_msg.is_dense = true;
  pointcloud_msg.point_step = numChannels * sizeof(float);
  pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width;
  pointcloud_msg.fields.resize(numChannels);
  for (int i = 0; i < numChannels; i++)
  {
    pointcloud_msg.fields[i].name = channelId[i];
    pointcloud_msg.fields[i].offset = i * sizeof(float);
    pointcloud_msg.fields[i].count = 1;
    pointcloud_msg.fields[i].datatype = PointField::FLOAT32;
  }

  pointcloud_msg_polar = pointcloud_msg;
  pointcloud_msg_polar.fields[0].name = "range";
  pointcloud_msg_polar.fields[1].name = "azimuth";
  pointcloud_msg_polar.fields[2].name = "elevation";
  
  // set pointcloud data values
  pointcloud_msg.data.clear();
  pointcloud_msg.data.resize(pointcloud_msg.row_step * pointcloud_msg.height, 0);
  float* pfdata = reinterpret_cast<float*>(&pointcloud_msg.data[0]);
  pointcloud_msg_polar.data.clear();
  pointcloud_msg_polar.data.resize(pointcloud_msg_polar.row_step * pointcloud_msg_polar.height, 0);
  float* pfdata_polar = reinterpret_cast<float*>(&pointcloud_msg_polar.data[0]);
  size_t data_cnt = 0;
  int echoIdx, pointIdx;
  for (echoIdx = 0; echoIdx < lidar_points.size(); echoIdx++)
  {
    for (pointIdx = 0; data_cnt < numChannels * pointcloud_msg.width && pointIdx < lidar_points[echoIdx].size(); pointIdx++, data_cnt+=4)
    {
		pfdata[data_cnt + 0] = lidar_points[echoIdx][pointIdx].x;
		pfdata[data_cnt + 1] = lidar_points[echoIdx][pointIdx].y;
		pfdata[data_cnt + 2] = lidar_points[echoIdx][pointIdx].z;
		pfdata[data_cnt + 3] = lidar_points[echoIdx][pointIdx].i;
		pfdata_polar[data_cnt + 0] = lidar_points[echoIdx][pointIdx].range;
		pfdata_polar[data_cnt + 1] = lidar_points[echoIdx][pointIdx].azimuth;
		pfdata_polar[data_cnt + 2] = lidar_points[echoIdx][pointIdx].elevation;
		pfdata_polar[data_cnt + 3] = lidar_points[echoIdx][pointIdx].i;
		int echo = lidar_points[echoIdx][pointIdx].echo;
		int layer = lidar_points[echoIdx][pointIdx].layer;
		bool layer_enabled = (m_laserscan_layer_filter.empty() ? 1 : (m_laserscan_layer_filter[layer]));
		if (!is_cloud_360 && layer_enabled)
		{
			// laser_scan_msg = laser_scan_msg_map[layer]
			ros_sensor_msgs::LaserScan& laser_scan_msg = laser_scan_msg_map[echo][layer];
			if (laser_scan_msg.ranges.size() == 0) // Initialize new LaserScan message
			{
				laser_scan_msg.ranges.clear();
				laser_scan_msg.intensities.clear();
				laser_scan_msg.ranges.reserve(total_point_count);
				laser_scan_msg.intensities.reserve(total_point_count);
				laser_scan_msg.angle_min = lidar_points[echoIdx][pointIdx].azimuth;
				laser_scan_msg.angle_max = lidar_points[echoIdx][pointIdx].azimuth;
				laser_scan_msg.range_min = lidar_points[echoIdx][pointIdx].range;
				laser_scan_msg.range_max = lidar_points[echoIdx][pointIdx].range;
			}
			else
			{
				laser_scan_msg.range_min = std::min(lidar_points[echoIdx][pointIdx].range, laser_scan_msg.range_min);
				laser_scan_msg.range_max = std::max(lidar_points[echoIdx][pointIdx].range, laser_scan_msg.range_max);
				laser_scan_msg.angle_min = std::min(lidar_points[echoIdx][pointIdx].azimuth, laser_scan_msg.angle_min);
				laser_scan_msg.angle_max = std::max(lidar_points[echoIdx][pointIdx].azimuth, laser_scan_msg.angle_max);
			}
			// Append point to LaserScan message
			laser_scan_msg.ranges.push_back(lidar_points[echoIdx][pointIdx].range);
			laser_scan_msg.intensities.push_back(lidar_points[echoIdx][pointIdx].i);
		}
    }
  }
  if (!is_cloud_360)
  {
	for(LaserScanMsgMap::iterator laser_scan_echo_iter = laser_scan_msg_map.begin(); laser_scan_echo_iter != laser_scan_msg_map.end(); laser_scan_echo_iter++)
	{
		int echo_idx = laser_scan_echo_iter->first;
		std::map<int,ros_sensor_msgs::LaserScan>& laser_scan_layer_map = laser_scan_echo_iter->second;
		for(std::map<int,ros_sensor_msgs::LaserScan>::iterator laser_scan_msg_iter = laser_scan_layer_map.begin(); laser_scan_msg_iter != laser_scan_layer_map.end(); laser_scan_msg_iter++)
		{
			int layer_idx = laser_scan_msg_iter->first;
			ros_sensor_msgs::LaserScan& laser_scan_msg = laser_scan_msg_iter->second;
			if (laser_scan_msg.ranges.size() > 1 && laser_scan_msg.angle_max > laser_scan_msg.angle_min)
			{
				float angle_diff = laser_scan_msg.angle_max - laser_scan_msg.angle_min;
				while (angle_diff > (float)(2.0 * M_PI))
					angle_diff -= (float)(2.0 * M_PI);
				while (angle_diff < 0)
					angle_diff += (float)(2.0 * M_PI);
				laser_scan_msg.angle_increment = angle_diff / (float)(laser_scan_msg.ranges.size() - 1);
				laser_scan_msg.range_min -= 1.0e-03f;
				laser_scan_msg.range_max += 1.0e-03f;
				laser_scan_msg.header = pointcloud_msg.header;
				laser_scan_msg.header.frame_id = m_frame_id + "_" + std::to_string(layer_idx);
				// scan_time = 1 / scan_frequency = time for a full 360-degree rotation of the sensor
				laser_scan_msg.scan_time = m_scan_time;
				// time_increment = 1 / measurement_frequency = scan_time / (number of scan points in a full 360-degree rotation of the sensor)
				laser_scan_msg.time_increment = laser_scan_msg.scan_time / (float)(laser_scan_msg.ranges.size() * 2.0 * M_PI / angle_diff);
				// ROS_INFO_STREAM("convert to LaserScan: frame_id=" << laser_scan_msg.header.frame_id << ", num_points=" << laser_scan_msg.ranges.size() << ", angle_min=" << (laser_scan_msg.angle_min * 180.0 / M_PI) << ", angle_max=" << (laser_scan_msg.angle_max * 180.0 / M_PI));
			}
			else
			{
				laser_scan_msg.ranges.clear();
				laser_scan_msg.intensities.clear();
			}
		}
	}
  }
}

/*
 * Callback function of MsgPackExportListenerIF. HandleMsgPackData() will be called in MsgPackExporter
 * for each registered listener after msgpack data have been received and converted.
 * This function converts and publishes msgpack data to PointCloud2 messages.
 */
void sick_scansegment_xd::RosMsgpackPublisher::HandleMsgPackData(const sick_scansegment_xd::MsgPackParserOutput& msgpack_data)
{
	if (!m_active)
		return; // publishing deactivated
	// Reorder points in consecutive lidarpoints for echo 0, echo 1 and echo 2 as described in https://github.com/michael1309/sick_lidar3d_pretest/issues/5
	size_t echo_count = 0;           // number of echos (multiScan136: 1 or 3 echos)
	size_t point_count_per_echo = 0; // number of points per echo
	size_t total_point_count = 0;    // total number of points in all echos
	int32_t segment_idx = msgpack_data.segmentIndex;
	int32_t telegram_cnt = msgpack_data.telegramCnt;
	for (int groupIdx = 0; groupIdx < msgpack_data.scandata.size(); groupIdx++)
	{
		echo_count = std::max(msgpack_data.scandata[groupIdx].scanlines.size(), echo_count);
		for (int echoIdx = 0; echoIdx < msgpack_data.scandata[groupIdx].scanlines.size(); echoIdx++)
		{
			total_point_count += msgpack_data.scandata[groupIdx].scanlines[echoIdx].points.size();
			point_count_per_echo = std::max(msgpack_data.scandata[groupIdx].scanlines[echoIdx].points.size(), point_count_per_echo);
		}
	}
	float lidar_points_min_azimuth = +2.0f * (float)M_PI, lidar_points_max_azimuth = -2.0f * (float)M_PI;
	std::vector<std::vector<sick_scansegment_xd::PointXYZRAEI32f>> lidar_points(echo_count);
	for (int echoIdx = 0; echoIdx < echo_count; echoIdx++)
	{
		lidar_points[echoIdx].reserve(point_count_per_echo);
	}
	for (int groupIdx = 0; groupIdx < msgpack_data.scandata.size(); groupIdx++)
	{
		for (int echoIdx = 0; echoIdx < msgpack_data.scandata[groupIdx].scanlines.size(); echoIdx++)
		{
			const std::vector<sick_scansegment_xd::MsgPackParserOutput::LidarPoint>& scanline = msgpack_data.scandata[groupIdx].scanlines[echoIdx].points;
			for (int pointIdx = 0; pointIdx < scanline.size(); pointIdx++)
			{
				const sick_scansegment_xd::MsgPackParserOutput::LidarPoint& point = scanline[pointIdx];
				lidar_points[echoIdx].push_back(sick_scansegment_xd::PointXYZRAEI32f(point.x, point.y, point.z, point.range, point.azimuth, point.elevation, point.i, point.groupIdx, point.echoIdx));
				lidar_points_min_azimuth = std::min(lidar_points_min_azimuth, point.azimuth);
				lidar_points_max_azimuth = std::max(lidar_points_max_azimuth, point.azimuth);
			}
		}
	}
	
    // Versendung von Vollumläufen als ROS-Nachricht:
	// a. Prozess läuft an
	// b. Segmente werden verworfen, bis ein Segment mit Startwinkel 0° eintrifft.
	// c. Es werden dann 12 Segmente aufgesammelt, bis 360° erreicht sind.
	// d. Die 12 Segmente werden nur ausgegeben, wenn keines von den Segmenten korrupt ist.
	// e. Die 12 Segmente werden als eine Pointcloud aufgesammelt und dann als eine Pointcloud2-Nachricht versendet.
	// f. Es werden intern zwei Topics verwendet:
	// 	  i.   Topic für 30° (Segmente)
	// 	  ii.  Topic für 360° (Vollumlauf)
	// 	  iii. Ist ein Topic leer, dann wird auf diesem Kanal nichts publiziert.
	// 	  iv.  Konfiguration erfolgt über YAML-Datei.
	if(m_publish_topic_all_segments != "")
	{
		float precheck_min_azimuth_deg = m_points_collector.min_azimuth * 180.0f / (float)M_PI;
		float precheck_max_azimuth_deg = m_points_collector.max_azimuth * 180.0f / (float)M_PI;
		bool publish_cloud_360 = (precheck_max_azimuth_deg - precheck_min_azimuth_deg + 1 >= m_all_segments_max_deg - m_all_segments_min_deg - 1) // fast pre-check
		    && m_points_collector.allSegmentsCovered(m_all_segments_min_deg, m_all_segments_max_deg); // all segments collected in m_points_collector
		// ROS_INFO_STREAM("RosMsgpackPublisher::HandleMsgPackData(): segment_idx=" << segment_idx << ", m_points_collector.lastSegmentIdx=" << m_points_collector.lastSegmentIdx() 
		//     << ", m_points_collector.total_point_count=" << m_points_collector.total_point_count << ", m_points_collector.allSegmentsCovered=" << publish_cloud_360);
		if (m_points_collector.total_point_count <= 0 || m_points_collector.telegram_cnt <= 0 || publish_cloud_360 || m_points_collector.lastSegmentIdx() > segment_idx) 
		{
			// 1. publish 360 degree point cloud if all segments collected
			// 2. start a new collection of all points (first time call, all segments covered, or segment index wrap around)
			if (m_points_collector.total_point_count > 0 && m_points_collector.telegram_cnt > 0 && publish_cloud_360)
			{
				// publish 360 degree point cloud
					// scan_time = 1 / scan_frequency = time for a full 360-degree rotation of the sensor
                    m_scan_time = (msgpack_data.timestamp_sec + 1.0e-9 * msgpack_data.timestamp_nsec) - (m_points_collector.timestamp_sec + 1.0e-9 * m_points_collector.timestamp_nsec);
                    LaserScanMsgMap laser_scan_msg_map; // laser_scan_msg_map[echo][layer] := LaserScan message given echo (Multiscan136: max 3 echos) and layer index (Multiscan136: 16 layer)
					PointCloud2Msg pointcloud_msg, pointcloud_msg_polar;
					convertPointsToCloud(m_points_collector.timestamp_sec, m_points_collector.timestamp_nsec, m_points_collector.lidar_points, m_points_collector.total_point_count, 
					    pointcloud_msg, pointcloud_msg_polar, laser_scan_msg_map, true);
					publish(m_node, m_publisher_all_segments, pointcloud_msg, pointcloud_msg_polar, m_publisher_laserscan_360, laser_scan_msg_map, 
					    std::max(1, (int)echo_count), -1); // pointcloud, number of echos, segment index (or -1 if pointcloud contains data from multiple segments)
					// ROS_INFO_STREAM("RosMsgpackPublisher::HandleMsgPackData(): cloud_fullframe published, " << m_points_collector.total_point_count << " points, " << pointcloud_msg.data.size() << " byte, "
					//     << m_points_collector.segment_list.size() << " segments (" << sick_scansegment_xd::util::printVector(m_points_collector.segment_list, ",") << "), "
					//     << m_points_collector.telegram_list.size() << " telegrams (" << sick_scansegment_xd::util::printVector(m_points_collector.telegram_list, ",") << "), "
					//     << "min_azimuth=" << (m_points_collector.min_azimuth * 180.0 / M_PI) << ", max_azimuth=" << (m_points_collector.max_azimuth * 180.0 / M_PI) << " [deg]");
			}
			// Start a new 360 degree collection
			m_points_collector = SegmentPointsCollector(telegram_cnt);
			m_points_collector.timestamp_sec = msgpack_data.timestamp_sec;
			m_points_collector.timestamp_nsec = msgpack_data.timestamp_nsec;
			m_points_collector.total_point_count = total_point_count;
			m_points_collector.lidar_points = std::vector<std::vector<sick_scansegment_xd::PointXYZRAEI32f>>(lidar_points.size());
			for (int echoIdx = 0; echoIdx < lidar_points.size(); echoIdx++)
				m_points_collector.lidar_points[echoIdx].reserve(12 * lidar_points[echoIdx].size());
			m_points_collector.appendLidarPoints(lidar_points, segment_idx, telegram_cnt);
			m_points_collector.min_azimuth = lidar_points_min_azimuth;
			m_points_collector.max_azimuth = lidar_points_max_azimuth;
		}
		else if (telegram_cnt == m_points_collector.telegram_cnt + 1) // append lidar points to m_points_collector
		{
			if (m_points_collector.lidar_points.size() < lidar_points.size())
				m_points_collector.lidar_points.resize(lidar_points.size());
			// m_points_collector.segment_count = segment_idx;
			m_points_collector.telegram_cnt = telegram_cnt;
			m_points_collector.total_point_count += total_point_count;
			m_points_collector.appendLidarPoints(lidar_points, segment_idx, telegram_cnt);
			m_points_collector.min_azimuth = std::min(m_points_collector.min_azimuth, lidar_points_min_azimuth);
			m_points_collector.max_azimuth = std::max(m_points_collector.max_azimuth, lidar_points_max_azimuth);
		}
		else
		{
			ROS_WARN_STREAM("## WARNING RosMsgpackPublisher::HandleMsgPackData(): current segment: " << segment_idx << ", last segment in collector: " << m_points_collector.lastSegmentIdx() 
				<< ", current telegram: " << telegram_cnt << ", last telegram in collector: " << m_points_collector.telegram_cnt
				<< ", datagram(s) missing, 360-degree-pointcloud not published");
			m_points_collector = SegmentPointsCollector(telegram_cnt); // reset pointcloud collector
		}
		// ROS_INFO_STREAM("RosMsgpackPublisher::HandleMsgPackData(): segment_idx " << segment_idx << " of " << m_segment_count << ", " << m_points_collector.total_point_count << " points in collector");
	}
  
    // Publish PointCloud2 message for the current segment
    if(m_publish_topic != "")
	{
        LaserScanMsgMap laser_scan_msg_map; // laser_scan_msg_map[echo][layer] := LaserScan message given echo (Multiscan136: max 3 echos) and layer index (Multiscan136: 16 layer)
		PointCloud2Msg pointcloud_msg_segment, pointcloud_msg_segment_polar;
		convertPointsToCloud(msgpack_data.timestamp_sec, msgpack_data.timestamp_nsec, lidar_points, total_point_count, pointcloud_msg_segment, pointcloud_msg_segment_polar, laser_scan_msg_map, false);
		publish(m_node, m_publisher_cur_segment, pointcloud_msg_segment, pointcloud_msg_segment_polar, m_publisher_laserscan_segment, laser_scan_msg_map, 
		    std::max(1, (int)echo_count), segment_idx); // pointcloud, number of echos, segment index (or -1 if pointcloud contains data from multiple segments)
	}
	
}

/*
 * Returns this instance explicitely as an implementation of interface MsgPackExportListenerIF.
 */
sick_scansegment_xd::MsgPackExportListenerIF* sick_scansegment_xd::RosMsgpackPublisher::ExportListener(void) { return this; }
