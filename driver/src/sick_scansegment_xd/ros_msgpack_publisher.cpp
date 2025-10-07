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
#include <climits>

#include <sick_scan/sick_generic_callback.h>
#include "sick_scansegment_xd/compact_parser.h"
#include "sick_scansegment_xd/ros_msgpack_publisher.h"
#if defined ROSSIMU
#include "sick_scan/pointcloud_utils.h"
#endif

/** Configuration of customized pointclouds */

sick_scansegment_xd::CustomPointCloudConfiguration::CustomPointCloudConfiguration(const std::string& cfg_name, const std::string& cfg_str)
{
		// Split into list of parameters, each parameter is a key-value-pair
		std::vector<std::string> parameters;
		sick_scansegment_xd::util::parseVector(cfg_str, parameters, ' ');
		std::map<std::string, std::string> key_value_pairs;
		for(int n = 0; n < parameters.size(); n++)
		{
				// Split into key-value-pairs
				std::vector<std::string> key_value_pair;
				sick_scansegment_xd::util::parseVector(parameters[n], key_value_pair, '=');
				if (key_value_pair.size() != 2)
				{
						ROS_ERROR_STREAM("## ERROR CustomPointCloudConfiguration(name=" << cfg_name << ", value=" << cfg_str << "): can't parse " << parameters[n] << ", expected key=value format, check configuration");
						continue;
				}
				key_value_pairs[key_value_pair[0]] = key_value_pair[1];
				ROS_DEBUG_STREAM("CustomPointCloudConfiguration(" << cfg_name << "): " << key_value_pair[0] << " = " << key_value_pair[1]);
		}
		m_cfg_name = cfg_name;
		m_publish = (key_value_pairs["publish"].empty() ? false : std::stoi(key_value_pairs["publish"]) > 0);
		m_topic = key_value_pairs["topic"];
		m_frameid = key_value_pairs["frameid"];
		m_coordinate_notation = (key_value_pairs["coordinateNotation"].empty() ? 0 : std::stoi(key_value_pairs["coordinateNotation"]));
		m_update_method = (key_value_pairs["updateMethod"].empty() ? 0 : std::stoi(key_value_pairs["updateMethod"]));
		if (m_coordinate_notation == 0) // coordinateNotation=0: cartesian (default, pointcloud has fields x,y,z,i)
				key_value_pairs["fields"] = "x,y,z,i";
		else if (m_coordinate_notation == 1) // coordinateNotation=1: polar (pointcloud has fields azimuth,elevation,r,i)
				key_value_pairs["fields"] = "azimuth,elevation,range,i";
		else if (m_coordinate_notation == 2) // both cartesian and polar (pointcloud has fields x,y,z,azimuth,elevation,r,i)
				key_value_pairs["fields"] = "x,y,z,azimuth,elevation,range,i";
		else if (m_coordinate_notation != 3) // coordinateNotation=3: customized pointcloud fields from configuration
				ROS_ERROR_STREAM("## ERROR CustomPointCloudConfiguration(name=" << cfg_name << ", value=" << cfg_str << "): coordinateNotation has invalid value " << m_coordinate_notation << ", check configuration");
		if (!key_value_pairs["rangeFilter"].empty()) // Configuration of optional range filter
		{
			const std::string& range_filter_str = key_value_pairs["rangeFilter"];
			std::vector<std::string> range_filter_args;
			sick_scansegment_xd::util::parseVector(range_filter_str, range_filter_args, ',');
			if(range_filter_args.size() == 3)
			{
				float range_min = std::stof(range_filter_args[0]);
				float range_max = std::stof(range_filter_args[1]);
				int range_filter_handling = std::stoi(range_filter_args[2]);
				m_range_filter = sick_scan_xd::SickRangeFilter(range_min, range_max, (sick_scan_xd::RangeFilterResultHandling)range_filter_handling);
			}
			else if(!range_filter_args.empty())
			{
				ROS_ERROR_STREAM("## ERROR CustomPointCloudConfiguration(name=" << cfg_name << ", value=" << cfg_str << "): rangeFilter has invalid value " << range_filter_str << ", check configuration");
			}
		}
		std::vector<std::string> fields;
		std::vector<int> echos, layers, reflectors, infringed;
		sick_scansegment_xd::util::parseVector(key_value_pairs["fields"], fields, ',');
		sick_scansegment_xd::util::parseVector(key_value_pairs["echos"], echos, ',');
		sick_scansegment_xd::util::parseVector(key_value_pairs["layers"], layers, ',');
		sick_scansegment_xd::util::parseVector(key_value_pairs["reflectors"], reflectors, ',');
		sick_scansegment_xd::util::parseVector(key_value_pairs["infringed"], infringed, ',');
		for(int n = 0; n < layers.size(); n++)
		   layers[n] -= 1; // layer_ids in the launchfile enumerate from 1 up to 16, layer_idx starts from 0, i.e. layer_idx = layer_id - 1
		// default if not configured in launchfile: all fields, echos, layers, reflectors and infringements enabled
		if (fields.empty())
		    fields = { "x", "y", "z", "i", "range", "azimuth", "elevation", "layer", "echo", "reflector" };
		if (echos.empty())
		    echos = { 0, 1, 2 };
		if (layers.empty())
		    layers = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
		if (reflectors.empty())
		    reflectors = { 0, 1 };
		if (infringed.empty())
		    infringed = { 0, 1 };
    // convert properties to enabled-map for faster access
		for(int n = 0; n < fields.size(); n++)
				m_field_enabled[fields[n]] = true;
		for(int n = 0; n < echos.size(); n++)
				m_echo_enabled[echos[n]] = true;
		for(int n = 0; n < layers.size(); n++)
				m_layer_enabled[layers[n]] = true;
		for(int n = 0; n < reflectors.size(); n++)
				m_reflector_enabled[reflectors[n]] = true;
		for(int n = 0; n < infringed.size(); n++)
				m_infringed_enabled[infringed[n]] = true;
		// Topic and frame id must always be set, topics must be different for each pointcloud
		if (m_topic.empty() || m_frameid.empty())
		{
				ROS_ERROR_STREAM("## ERROR CustomPointCloudConfiguration(name=" << cfg_name << ", value=" << cfg_str << "): topic and frameid required, pointcloud will not be published, check configuration");
				m_publish = false;
		}
}

void sick_scansegment_xd::CustomPointCloudConfiguration::print(void) const
{
		ROS_INFO_STREAM("CustomPointCloudConfiguration(" << m_cfg_name << "): publish = " << m_publish);
		ROS_INFO_STREAM("CustomPointCloudConfiguration(" << m_cfg_name << "): topic = " << m_topic);
		ROS_INFO_STREAM("CustomPointCloudConfiguration(" << m_cfg_name << "): frameid = " << m_frameid);
		ROS_INFO_STREAM("CustomPointCloudConfiguration(" << m_cfg_name << "): coordinate_notation = " << m_coordinate_notation);
		ROS_INFO_STREAM("CustomPointCloudConfiguration(" << m_cfg_name << "): update_method = " << m_update_method);
		ROS_INFO_STREAM("CustomPointCloudConfiguration(" << m_cfg_name << "): fields_enabled = " << printValuesEnabled(m_field_enabled));
		ROS_INFO_STREAM("CustomPointCloudConfiguration(" << m_cfg_name << "): echos_enabled = " << printValuesEnabled(m_echo_enabled));
		ROS_INFO_STREAM("CustomPointCloudConfiguration(" << m_cfg_name << "): layers_enabled = " << printValuesEnabled(m_layer_enabled));
		ROS_INFO_STREAM("CustomPointCloudConfiguration(" << m_cfg_name << "): reflector_enabled = " << printValuesEnabled(m_reflector_enabled));
		ROS_INFO_STREAM("CustomPointCloudConfiguration(" << m_cfg_name << "): infringed_enabled = " << printValuesEnabled(m_infringed_enabled));
		ROS_INFO_STREAM("CustomPointCloudConfiguration(" << m_cfg_name << "): range_filter = " << m_range_filter.print());
}

std::string sick_scansegment_xd::CustomPointCloudConfiguration::printValuesEnabled(const std::map<std::string,bool>& mapped_values, const std::string& delim)
{
		std::stringstream s;
		for(auto iter = mapped_values.cbegin(); iter != mapped_values.cend(); iter++)
		{
				if(iter->second)
						s << (s.str().empty() ? "" : delim) << iter->first;
		}
		return s.str();
}

std::string sick_scansegment_xd::CustomPointCloudConfiguration::printValuesEnabled(const std::map<int8_t,bool>& mapped_values, const std::string& delim)
{
		std::stringstream s;
		for(auto iter = mapped_values.cbegin(); iter != mapped_values.cend(); iter++)
		{
				if(iter->second)
						s << (s.str().empty() ? "" : delim) << (int)(iter->first);
		}
		return s.str();
}

/* @brief Determine and initialize all_segments_min/max_deg by LFPangleRangeFilter
** host_set_LFPangleRangeFilter = "<enabled> <azimuth_start> <azimuth_stop> <elevation_start> <elevation_stop> <beam_increment>" with azimuth and elevation given in degree
** Returns true, if angleRangeFilterSettings are enabled, otherwise false.
*/
bool sick_scansegment_xd::RosMsgpackPublisher::initLFPangleRangeFilterSettings(const std::string& host_LFPangleRangeFilter)
{
	std::vector<std::string> parameter_token;
	sick_scansegment_xd::util::parseVector(host_LFPangleRangeFilter, parameter_token, ' ');
	try
	{
		if(parameter_token.size() >= 3 && std::stoi(parameter_token[0]) > 0) // LFPangleRangeFilter enabled, i.e. all_segments_min/max_deg given by LFPangleRangeFilter settings
		{
			float all_segments_azimuth_min_deg = std::stof(parameter_token[1]);
			float all_segments_azimuth_max_deg = std::stof(parameter_token[2]);
			m_all_segments_azimuth_min_deg = std::max<float>(m_all_segments_azimuth_min_deg, all_segments_azimuth_min_deg);
			m_all_segments_azimuth_max_deg = std::min<float>(m_all_segments_azimuth_max_deg, all_segments_azimuth_max_deg);
			return true;
		}
	}
	catch(const std::exception& e)
	{
		ROS_ERROR_STREAM("## ERROR in RosMsgpackPublisher: can't parse LFPangleRangeFilter settings, exception " << e.what() << ", check LFPangleRangeFilter configuration in the launchfile");
		throw e; // fatal error: rethrow exception, which will be caught in sick_generic_caller and handled by exiting with error
	}
	return false;
}

/* @brief Determine and initialize all_segments_elevation_min/max_deg by host_LFPlayerFilter"
** host_LFPlayerFilter = "<enabled> <layer0enabled>  <layer1enabled> ...  <layer15enabled>", default: "0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"
** Returns true, if layerFilterSettings are enabled, otherwise false.
*/
bool sick_scansegment_xd::RosMsgpackPublisher::initLFPlayerFilterSettings(const std::string& host_LFPlayerFilter)
{
	std::vector<std::string> parameter_token;
	sick_scansegment_xd::util::parseVector(host_LFPlayerFilter, parameter_token, ' ');
	try
	{
		if(parameter_token.size() >= 17 && std::stoi(parameter_token[0]) > 0) // LFPlayerFilter enabled
		{
			float all_segments_elevation_min_deg = 999;
			float all_segments_elevation_max_deg = -999;
			for(int n = 1; n < parameter_token.size(); n++)
			{
				if(std::stoi(parameter_token[n]) > 0)
				{
					float layer_elevation_deg = sick_scansegment_xd::CompactDataParser::GetElevationDegFromLayerIdx(n - 1);
					all_segments_elevation_min_deg = std::min<float>(all_segments_elevation_min_deg, layer_elevation_deg);
					all_segments_elevation_max_deg = std::max<float>(all_segments_elevation_max_deg, layer_elevation_deg);
				}
			}
			if (all_segments_elevation_max_deg > all_segments_elevation_min_deg)
			{
				m_all_segments_elevation_min_deg = all_segments_elevation_min_deg;
				m_all_segments_elevation_max_deg = all_segments_elevation_max_deg;
				return true;
			}
		}
	}
	catch(const std::exception& e)
	{
		ROS_ERROR_STREAM("## ERROR in RosMsgpackPublisher: can't parse initLFPlayerFilterSettings settings, exception " << e.what() << ", check initLFPlayerFilterSettings configuration in the launchfile");
		throw e; // fatal error: rethrow exception, which will be caught in sick_generic_caller and handled by exiting with error
	}
	return false;
}

/*
 * @brief RosMsgpackPublisher constructor
 * @param[in] node_name name of the ros node
 * @param[in] config sick_scansegment_xd configuration, RosMsgpackPublisher uses
 *            config.publish_frame_id: frame id of ros Laserscan messages, default: "world"
 * @param[in] qos quality of service profile for the ros publisher, default: 1
 */
sick_scansegment_xd::RosMsgpackPublisher::RosMsgpackPublisher(const std::string& node_name, const sick_scansegment_xd::Config& config)
#if defined __ROS_VERSION && __ROS_VERSION > 1
	: Node(node_name)
#endif
{
	m_active = false;
  m_frame_id = config.publish_frame_id; 
	m_imu_frame_id = config.publish_imu_frame_id;
	m_node = config.node;
	m_laserscan_layer_filter = config.laserscan_layer_filter;
	// m_segment_count = config.segment_count;
	m_all_segments_azimuth_min_deg = (float)config.all_segments_min_deg;
  m_all_segments_azimuth_max_deg = (float)config.all_segments_max_deg;
  m_host_FREchoFilter = config.host_FREchoFilter;
	m_host_set_FREchoFilter = config.host_set_FREchoFilter;
	if (config.host_set_LFPangleRangeFilter)
	{
		initLFPangleRangeFilterSettings(config.host_LFPangleRangeFilter);
	}
	if (config.host_set_LFPlayerFilter)
	{
		initLFPlayerFilterSettings(config.host_LFPlayerFilter);
	}
	std::string imu_topic = config.imu_topic;
#if defined __ROS_VERSION && __ROS_VERSION > 1 // ROS-2 publisher
	rosQoS qos = rclcpp::SystemDefaultsQoS();
	QoSConverter qos_converter;
	int qos_val = -1;
	rosDeclareParam(m_node, "ros_qos", qos_val);
	rosGetParam(m_node, "ros_qos", qos_val);
	if (qos_val >= 0)
			qos = qos_converter.convert(qos_val);
	m_publisher_laserscan_segment = create_publisher<ros_sensor_msgs::LaserScan>(config.publish_laserscan_segment_topic, qos);
	ROS_INFO_STREAM("RosMsgpackPublisher: publishing LaserScan segment messages on topic \"" << m_publisher_laserscan_segment->get_topic_name() << "\"");
	m_publisher_laserscan_360 = create_publisher<ros_sensor_msgs::LaserScan>(config.publish_laserscan_fullframe_topic, qos);
	ROS_INFO_STREAM("RosMsgpackPublisher: publishing LaserScan fullframe messages on topic \"" << m_publisher_laserscan_360->get_topic_name() << "\"");
	if (config.imu_enable)
	{
		if (imu_topic[0] != '/')
			imu_topic = "~/" + imu_topic;
		m_publisher_imu = create_publisher<ros_sensor_msgs::Imu>(imu_topic, qos);
		m_publisher_imu_initialized = true;
		ROS_INFO_STREAM("RosMsgpackPublisher: publishing Imu messages on topic \"" << m_publisher_imu->get_topic_name() << "\"");
	}
#elif defined __ROS_VERSION && __ROS_VERSION > 0 // ROS-1 publisher
	int qos = 16 * 12 * 3; // 16 layers, 12 segments, 3 echos
	int qos_val = -1;
	rosDeclareParam(m_node, "ros_qos", qos_val);
	rosGetParam(m_node, "ros_qos", qos_val);
	if (qos_val >= 0)
			qos = qos_val;
	m_publisher_laserscan_segment = m_node->advertise<ros_sensor_msgs::LaserScan>(config.publish_laserscan_segment_topic, qos);
	ROS_INFO_STREAM("RosMsgpackPublisher: publishing LaserScan segment messages on topic \"" << config.publish_laserscan_segment_topic << "\"");
	m_publisher_laserscan_360 = m_node->advertise<ros_sensor_msgs::LaserScan>(config.publish_laserscan_fullframe_topic, qos);
	ROS_INFO_STREAM("RosMsgpackPublisher: publishing LaserScan fullframe messages on topic \"" << config.publish_laserscan_fullframe_topic << "\"");

	if (config.imu_enable)
	{
		m_publisher_imu = m_node->advertise<ros_sensor_msgs::Imu>(imu_topic, qos);
		m_publisher_imu_initialized = true;
		ROS_INFO_STREAM("RosMsgpackPublisher: publishing Imu messages on topic \"" << config.imu_topic << "\"");
	}
#endif
  sick_scan_xd::setImuTopic(imu_topic);

  /* Configuration of customized pointclouds:

	Parameter "custom_pointclouds" lists all customized pointclouds to be published. Each pointcloud is given by its name and configured by the following parameters:
	"<name_of_custom_pointcloud>" type="string" value="list of key-value-pairs"

	The list of key-value-pairs defines the pointcloud properties. List of supported key-value-pairs for customized pointclouds:

	Parameter "coordinateNotation" is an enum to configure pointcloud coordinates:
			coordinateNotation=0: cartesian (default, pointcloud has fields x,y,z,i), identical to customized with fields=x,y,z,i
			coordinateNotation=1: polar (pointcloud has fields azimuth,elevation,r,i), identical to customized with fields=azimuth,elevation,range,i
			coordinateNotation=2: both cartesian and polar (pointcloud has fields x,y,z,azimuth,elevation,r,i), identical to customized with fields=x,y,z,azimuth,elevation,range,i
			coordinateNotation=3: customized pointcloud fields, i.e. the pointcloud has fields configured by parameter "fields"

	Parameter "updateMethod" is an enum to configure fullframe pointclouds versus segmented pointcloud:
			updateMethod=0: fullframe pointcloud (default)
			updateMethod=1: segmented pointcloud

	Parameter "fields" defines the fields of the pointcloud for coordinateNotation == 3 (customized pointcloud fields), e.g.
			fields=x,y,z,i: cartesian pointcloud
			fields=range,azimuth,elevation: polar pointcloud
			or any other combination of x,y,z,i,range,azimuth,elevation,layer,echo,reflector

	Parameter "echos" defines which echos are included in the pointcloud, e.g.
			echos=0,1,2: all echos
			echos=2: last echo
			or any other combination of 0,1,2

	Parameter "layers" defines which echos are included in the pointcloud, e.g
			layers=0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15 for all layers
			layers=0 for the 0 degree layer
			or any other combination of 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15

	Parameter "reflectors" filters the points by the reflector bit, i.e.
			reflectors=0,1 for points with reflector bit set or not set
			reflectors=0 for points with reflector bit not set
			reflectors=1 for points with reflector bit set

	Parameter "infringed" defines filters the points by infringement, i.e.
			infringed=0,1 for points with infringement bit set or not set
			infringed=0 for points with infringement bit not set
			infringed=1 for points with infringement bit set
	Parameter "infringed" is currently not supported (reserved for future use)

	Parameter "topic" defines the ros topic, e.g. topic=/cloud_fullframe for cartesian fullframe pointclouds

	Parameter "frameid" defines the ros frame of the pointcloud, e.g. frameid=world, frameid=map or frameid=base_link

	Parameter "publish" activates or deactivates the pointcloud, e.g. publish=1 to generate and publish, or publish=0 to deactivate that pointcloud

	Examples:
        <!-- List of customized pointclouds -->
        <param name="custom_pointclouds" type="string" value="custom_pointcloud_cartesian_segmented custom_pointcloud_polar_segmented custom_pointcloud_cartesian_fullframe custom_pointcloud_all_fields_fullframe"/>

        <!-- custom_pointcloud_cartesian_segmented: cartesian coordinates, segmented, all echos, all layer, topic "cloud_unstructured_segments" -->
        <param name="custom_pointcloud_cartesian_segmented" type="string" value="coordinateNotation=0 updateMethod=1 echos=0,1,2 layers=0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15 reflectors=0,1 infringed=0,1 topic=/cloud_unstructured_segments frameid=world publish=1"/>

        <!-- custom_pointcloud_polar_segmented: polar coordinates, segmented, all echos, all layer, topic "cloud_polar_unstructured_segments" -->
        <param name="custom_pointcloud_polar_segmented" type="string" value="coordinateNotation=1 updateMethod=1 echos=0,1,2 layers=0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15 reflectors=0,1 infringed=0,1 topic=/cloud_polar_unstructured_segments frameid=world publish=1"/>

        <!-- custom_pointcloud_cartesian_fullframe: cartesian coordinates, fullframe, all echos, all layer, topic "cloud_unstructured_fullframe" -->
        <param name="custom_pointcloud_cartesian_fullframe" type="string" value="coordinateNotation=0 updateMethod=0 echos=0,1,2 layers=0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15 reflectors=0,1 infringed=0,1 topic=/cloud_unstructured_fullframe frameid=world publish=1"/>

        <!-- cloud_all_fields_fullframe: all fields (x,y,z,i,range,azimuth,elevation,layer,echo,reflector), fullframe, all echos, all layer, topic "cloud_unstructured_fullframe" -->
        <param name="custom_pointcloud_all_fields_fullframe" type="string" value="coordinateNotation=3 updateMethod=0 fields=x,y,z,i,range,azimuth,elevation,layer,echo,reflector echos=0,1,2 layers=0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15 reflectors=0,1 infringed=0,1 topic=/cloud_all_fields_fullframe frameid=world publish=1"/>

	*/
	std::string custom_pointclouds = "";
	rosDeclareParam(m_node, "custom_pointclouds", custom_pointclouds);
	rosGetParam(m_node, "custom_pointclouds", custom_pointclouds);
	std::vector<std::string> custom_pointclouds_tokens;
  sick_scansegment_xd::util::parseVector(custom_pointclouds, custom_pointclouds_tokens, ' ');
	for(int n = 0; n < custom_pointclouds_tokens.size(); n++)
	{
		std::string custom_pointcloud_cfg_str = "";
		rosDeclareParam(m_node, custom_pointclouds_tokens[n], custom_pointcloud_cfg_str);
		rosGetParam(m_node, custom_pointclouds_tokens[n], custom_pointcloud_cfg_str);
		CustomPointCloudConfiguration custom_pointcloud_cfg(custom_pointclouds_tokens[n], custom_pointcloud_cfg_str);
		if (custom_pointcloud_cfg.publish())
		{
		  custom_pointcloud_cfg.print();
#if defined __ROS_VERSION && __ROS_VERSION > 1 // ROS-2 publisher
			custom_pointcloud_cfg.publisher() = create_publisher<PointCloud2Msg>(custom_pointcloud_cfg.topic(), qos);
#elif defined __ROS_VERSION && __ROS_VERSION > 0 // ROS-1 publisher
			custom_pointcloud_cfg.publisher() = m_node->advertise<PointCloud2Msg>(custom_pointcloud_cfg.topic(), qos);
#endif
			m_custom_pointclouds_cfg.push_back(custom_pointcloud_cfg);
		}
	}
}

/*
 * @brief RosMsgpackPublisher destructor
 */
sick_scansegment_xd::RosMsgpackPublisher::~RosMsgpackPublisher()
{
}

/** Prints (elevation,azimuth) values of all lidar points */
std::string sick_scansegment_xd::RosMsgpackPublisher::printElevationAzimuthTable(const std::vector<std::vector<sick_scansegment_xd::PointXYZRAEI32f>>& lidar_points)
{
	std::stringstream s;
	for (int echoIdx = 0; echoIdx < lidar_points.size(); echoIdx++)
	{
		s << (echoIdx > 0 ? ", " : "") << "echo" << echoIdx << ":[";
		int last_elevation_mdeg = -99999, last_azimuth_ideg = -99999;
		for (int n = 0; n < lidar_points[echoIdx].size(); n++)
		{
			const sick_scansegment_xd::PointXYZRAEI32f& point = lidar_points[echoIdx][n];
			float elevation_deg = point.elevation * 180.0f / (float)M_PI;
			float azimuth_fdeg = point.azimuth * 180.0f / (float)M_PI;
			int elevation_mdeg = (int)(1000.0f * elevation_deg);
			int azimuth_ideg = (int)(azimuth_fdeg);
			if (elevation_mdeg != last_elevation_mdeg || azimuth_ideg != last_azimuth_ideg)
				s << (n > 0 ? "," : "") << " (" << (elevation_mdeg/1000) << "," << azimuth_ideg << ")";
			last_elevation_mdeg = elevation_mdeg;
			last_azimuth_ideg = azimuth_ideg;
		}
		s << " ]";
	}

	return s.str();
}


/** Prints (elevation,azimuth) values of the coverage table of collected lidar points */
std::string sick_scansegment_xd::RosMsgpackPublisher::printCoverageTable(const std::map<int, std::map<int, int>>& elevation_azimuth_histograms)
{
	std::stringstream s;
	s << "[";
	for (std::map<int, std::map<int, int>>::const_iterator coverage_elevation_iter = elevation_azimuth_histograms.cbegin(); coverage_elevation_iter != elevation_azimuth_histograms.cend(); coverage_elevation_iter++)
	{
		int elevation_mdeg = coverage_elevation_iter->first;
		const std::map<int, int>& azimuth_histogram = coverage_elevation_iter->second;
		for (std::map<int, int>::const_iterator coverage_azimuth_iter = azimuth_histogram.cbegin(); coverage_azimuth_iter != azimuth_histogram.cend(); coverage_azimuth_iter++)
		{
			const int& azimuth_ideg = coverage_azimuth_iter->first;
			const int& azimuth_cnt = coverage_azimuth_iter->second;
			if (azimuth_cnt > 0)
				s << " (" << (elevation_mdeg/1000) << "," << azimuth_ideg << "),";
		}
	}
	s << " ]";
  return s.str();
}

bool sick_scansegment_xd::RosMsgpackPublisher::hasPointcloudRequiredFields(const PointCloud2Msg& pointcloud_msg, const std::unordered_set<std::string>& required_fields) const {
	const size_t numRequiredFields = required_fields.size();
	int found = 0;
	for (const auto& field : pointcloud_msg.fields) {
		if (required_fields.find(field.name) != required_fields.end()) {
			found++;
		}
	}
	return (found == numRequiredFields);
}

/** Shortcut to publish a PointCloud2Msg */
void sick_scansegment_xd::RosMsgpackPublisher::publishPointCloud2Msg(rosNodePtr node, PointCloud2MsgPublisher& publisher, PointCloud2Msg& pointcloud_msg, int32_t num_echos, int32_t segment_idx, int coordinate_notation, const std::string& topic)
{

  bool isCartesian = (coordinate_notation == 0);
  if (!isCartesian) {
  	 isCartesian = hasPointcloudRequiredFields(pointcloud_msg, MinimalCartesianPointCloudFields);
  }
  if (isCartesian)
	{
		sick_scan_xd::PointCloud2withEcho cloud_msg_with_echo(&pointcloud_msg, num_echos, segment_idx, topic);
		notifyCartesianPointcloudListener(node, &cloud_msg_with_echo);
	}
#if defined RASPBERRY && RASPBERRY > 0 // polar pointcloud deactivated on Raspberry for performance reasons
#else
  bool isPolar = (coordinate_notation == 1);
  if (!isPolar) {
	 isPolar = hasPointcloudRequiredFields(pointcloud_msg, MinimalPolarPointCloudFields);
  }

  if (isPolar) // coordinateNotation=1: polar (pointcloud has fields azimuth,elevation,r,i) => notify polar pointcloud listener
	{
		sick_scan_xd::PointCloud2withEcho cloud_msg_with_echo(&pointcloud_msg, num_echos, segment_idx, topic);
		notifyPolarPointcloudListener(node, &cloud_msg_with_echo);
	}
#endif
#if defined __ROS_VERSION && __ROS_VERSION > 1
	publisher->publish(pointcloud_msg);
#elif defined __ROS_VERSION && __ROS_VERSION > 0
	publisher.publish(pointcloud_msg);
#elif defined ROSSIMU
  // plotPointCloud(pointcloud_msg);
#endif
}

/** Shortcut to publish Laserscan messages */
void sick_scansegment_xd::RosMsgpackPublisher::publishLaserScanMsg(rosNodePtr node, LaserscanMsgPublisher& laserscan_publisher, LaserScanMsgMap& laser_scan_msg_map, int32_t num_echos, int32_t segment_idx)
{
#if defined RASPBERRY && RASPBERRY > 0 // laserscan messages deactivated on Raspberry for performance reasons
#else
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
#endif
}

class CustomizedPointXYZRAEI32f : public sick_scansegment_xd::PointXYZRAEI32f
{
public:
  CustomizedPointXYZRAEI32f() : sick_scansegment_xd::PointXYZRAEI32f() {}
  CustomizedPointXYZRAEI32f(uint32_t timestamp_sec, uint32_t timestamp_nsec, uint64_t lidar_timestamp_start_microsec, const sick_scansegment_xd::PointXYZRAEI32f& point) : sick_scansegment_xd::PointXYZRAEI32f(point) 
	{
		if (point.lidar_timestamp_microsec > lidar_timestamp_start_microsec) 
		{
		  time_offset_nanosec = (uint32_t)(1000 * (point.lidar_timestamp_microsec - lidar_timestamp_start_microsec));
			time_offset_sec = (float)(1.0e-6 * (point.lidar_timestamp_microsec - lidar_timestamp_start_microsec));
			lidar_sec = (uint32_t)(point.lidar_timestamp_microsec / 1000000);
			lidar_nsec = (uint32_t)(1000 * (point.lidar_timestamp_microsec % 1000000));
			// assert((uint64_t)lidar_sec * 1000000 + (uint64_t)lidar_nsec / 1000 == point.lidar_timestamp_microsec);
		}
		ring = point.layer;
	}
	// Additional fields for customized point clouds:
  uint32_t time_offset_nanosec = 0; // field "t": 4 byte time offset of a scan point in nano seconds relative to the timestamp in the point cloud header, used by rtabmap for deskewing
	float time_offset_sec = 0; // field "ts": time offset in seconds (otherwise identical to field "t")
  uint32_t lidar_sec = 0;    // field "lidar_sec": 4 byte seconds part of the lidar timestamp in microseconds (lidar time), lidar_sec = (uint32_t)(lidar_timestamp_microsec / 1000000)
  uint32_t lidar_nsec = 0;   // field "lidar_nsec": 4 byte nano seconds part of the lidar timestamp in microseconds (lidar time), lidar_nsec = (uint32_t)(1000 * (lidar_timestamp_microsec % 1000000))
	int8_t ring = 0;           // field "ring": 1 byte layer id, identical to field "layer"
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
void sick_scansegment_xd::RosMsgpackPublisher::convertPointsToCustomizedFieldsCloud(uint32_t timestamp_sec, uint32_t timestamp_nsec, uint64_t lidar_timestamp_start_microsec,
  const std::vector<std::vector<sick_scansegment_xd::PointXYZRAEI32f>>& lidar_points, CustomPointCloudConfiguration& pointcloud_cfg, PointCloud2Msg& pointcloud_msg)
{
  // set pointcloud header
  pointcloud_msg.header.stamp.sec = timestamp_sec;
#if defined __ROS_VERSION && __ROS_VERSION > 1
  pointcloud_msg.header.stamp.nanosec = timestamp_nsec;
#else
  pointcloud_msg.header.stamp.nsec = timestamp_nsec;
#endif
  pointcloud_msg.header.frame_id = pointcloud_cfg.frameid();
  // set pointcloud field properties
	CustomizedPointXYZRAEI32f dummy_lidar_point;
  std::vector<PointCloudFieldProperty> field_properties;
	field_properties.reserve(12);
	if (pointcloud_cfg.fieldEnabled("x"))
	  field_properties.push_back(PointCloudFieldProperty("x", PointField::FLOAT32, sizeof(float), (uint8_t*)&dummy_lidar_point.x - (uint8_t*)&dummy_lidar_point));
	if (pointcloud_cfg.fieldEnabled("y"))
	  field_properties.push_back(PointCloudFieldProperty("y", PointField::FLOAT32, sizeof(float), (uint8_t*)&dummy_lidar_point.y - (uint8_t*)&dummy_lidar_point));
	if (pointcloud_cfg.fieldEnabled("z"))
	  field_properties.push_back(PointCloudFieldProperty("z", PointField::FLOAT32, sizeof(float), (uint8_t*)&dummy_lidar_point.z - (uint8_t*)&dummy_lidar_point));
	if (pointcloud_cfg.fieldEnabled("i"))
	  field_properties.push_back(PointCloudFieldProperty("i", PointField::FLOAT32, sizeof(float), (uint8_t*)&dummy_lidar_point.i - (uint8_t*)&dummy_lidar_point));
	if (pointcloud_cfg.fieldEnabled("range"))
	  field_properties.push_back(PointCloudFieldProperty("range", PointField::FLOAT32, sizeof(float), (uint8_t*)&dummy_lidar_point.range - (uint8_t*)&dummy_lidar_point));
	if (pointcloud_cfg.fieldEnabled("azimuth"))
	  field_properties.push_back(PointCloudFieldProperty("azimuth", PointField::FLOAT32, sizeof(float), (uint8_t*)&dummy_lidar_point.azimuth - (uint8_t*)&dummy_lidar_point));
	if (pointcloud_cfg.fieldEnabled("elevation"))
	  field_properties.push_back(PointCloudFieldProperty("elevation", PointField::FLOAT32, sizeof(float), (uint8_t*)&dummy_lidar_point.elevation - (uint8_t*)&dummy_lidar_point));
	if (pointcloud_cfg.fieldEnabled("t"))
	  field_properties.push_back(PointCloudFieldProperty("t", PointField::UINT32, sizeof(uint32_t), (uint8_t*)&dummy_lidar_point.time_offset_nanosec - (uint8_t*)&dummy_lidar_point));
	if (pointcloud_cfg.fieldEnabled("ts"))
	  field_properties.push_back(PointCloudFieldProperty("ts", PointField::FLOAT32, sizeof(float), (uint8_t*)&dummy_lidar_point.time_offset_sec - (uint8_t*)&dummy_lidar_point));
	if (pointcloud_cfg.fieldEnabled("lidar_sec"))
	  field_properties.push_back(PointCloudFieldProperty("lidar_sec", PointField::UINT32, sizeof(uint32_t), (uint8_t*)&dummy_lidar_point.lidar_sec - (uint8_t*)&dummy_lidar_point));
	if (pointcloud_cfg.fieldEnabled("lidar_nsec"))
	  field_properties.push_back(PointCloudFieldProperty("lidar_nsec", PointField::UINT32, sizeof(uint32_t), (uint8_t*)&dummy_lidar_point.lidar_nsec - (uint8_t*)&dummy_lidar_point));
	if (pointcloud_cfg.fieldEnabled("ring"))
	  field_properties.push_back(PointCloudFieldProperty("ring", PointField::INT8, sizeof(int8_t), (uint8_t*)&dummy_lidar_point.ring - (uint8_t*)&dummy_lidar_point));
	if (pointcloud_cfg.fieldEnabled("layer"))
	  field_properties.push_back(PointCloudFieldProperty("layer", PointField::INT8, sizeof(int8_t), (uint8_t*)&dummy_lidar_point.layer - (uint8_t*)&dummy_lidar_point));
	if (pointcloud_cfg.fieldEnabled("echo"))
	  field_properties.push_back(PointCloudFieldProperty("echo", PointField::INT8, sizeof(int8_t), (uint8_t*)&dummy_lidar_point.echo - (uint8_t*)&dummy_lidar_point));
	if (pointcloud_cfg.fieldEnabled("reflector"))
	  field_properties.push_back(PointCloudFieldProperty("reflector", PointField::INT8, sizeof(int8_t), (uint8_t*)&dummy_lidar_point.reflectorbit - (uint8_t*)&dummy_lidar_point));
	if (pointcloud_cfg.fieldEnabled("infringed"))
	  field_properties.push_back(PointCloudFieldProperty("infringed", PointField::INT8, sizeof(int8_t), (uint8_t*)&dummy_lidar_point.infringed - (uint8_t*)&dummy_lidar_point));
  int num_fields = field_properties.size();
  size_t max_number_of_points = 0;
  for (int echo_idx = 0; echo_idx < lidar_points.size(); echo_idx++)
  {
		max_number_of_points += lidar_points[echo_idx].size();
	}
  pointcloud_msg.height = 1;
  pointcloud_msg.width = max_number_of_points;
  pointcloud_msg.is_bigendian = false;
  pointcloud_msg.is_dense = true;
  pointcloud_msg.point_step = 0;
  pointcloud_msg.fields.resize(num_fields);
  for (int i = 0; i < num_fields; i++)
  {
    pointcloud_msg.fields[i].count = 1;
    pointcloud_msg.fields[i].name = field_properties[i].name;
    pointcloud_msg.fields[i].datatype = field_properties[i].datatype;
    pointcloud_msg.fields[i].offset = pointcloud_msg.point_step;
		pointcloud_msg.point_step += field_properties[i].datasize;
  }
  pointcloud_msg.row_step = pointcloud_msg.point_step * max_number_of_points;
  pointcloud_msg.data.clear();
  pointcloud_msg.data.resize(pointcloud_msg.row_step * pointcloud_msg.height, 0);
  // fill pointcloud data
	int point_cnt = 0;
	for (int echo_idx = 0; echo_idx < lidar_points.size(); echo_idx++)
	{
		for (int point_idx = 0; point_cnt < max_number_of_points && point_idx < lidar_points[echo_idx].size(); point_idx++)
		{
			CustomizedPointXYZRAEI32f cur_lidar_point(timestamp_sec, timestamp_nsec, lidar_timestamp_start_microsec, lidar_points[echo_idx][point_idx]);
			if (pointcloud_cfg.pointEnabled(cur_lidar_point))
			{
				size_t pointcloud_offset = point_cnt * pointcloud_msg.point_step; // offset in bytes in pointcloud_msg.data (destination)
				const uint8_t* src_lidar_point = (const uint8_t*)(&cur_lidar_point); // pointer to source lidar point (type CustomizedPointXYZRAEI32f)
				for (int field_idx = 0; field_idx < field_properties.size(); field_idx++)
				{
					size_t field_offset = field_properties[field_idx].fieldoffset;
					for (int n = 0; n < field_properties[field_idx].datasize; n++, pointcloud_offset++, field_offset++)
					{
						pointcloud_msg.data[pointcloud_offset] = src_lidar_point[field_offset];
					}
				}
				point_cnt++;
			}
		}
	}
	// resize pointcloud to actual number of points
  pointcloud_msg.width = point_cnt;
  pointcloud_msg.row_step = pointcloud_msg.point_step * point_cnt;
  pointcloud_msg.data.resize(pointcloud_msg.row_step * pointcloud_msg.height);
	ROS_DEBUG_STREAM("CustomPointCloudConfiguration " << pointcloud_cfg.cfgName() << ": " << point_cnt << " points per cloud, " << num_fields << " fields per point");
}

/*
 * Converts the lidarpoints from a msgpack to a LaserScan messages for each layer.
 * @param[in] timestamp_sec seconds part of timestamp
 * @param[in] timestamp_nsec  nanoseconds part of timestamp
 * @param[in] lidar_points list of PointXYZRAEI32f: lidar_points[echoIdx] are the points of one echo
 * @param[in] total_point_count total number of points in all echos
 * @param[out] laser_scan_msg_map laserscan message: ros_sensor_msgs::LaserScan for each echo and layer is laser_scan_msg_map[echo][layer]
 * @param[in] frame_id frame id of laserscan message, will be expanded to "<frame_id>_<layer_idx>"
 */
void sick_scansegment_xd::RosMsgpackPublisher::convertPointsToLaserscanMsg(uint32_t timestamp_sec, uint32_t timestamp_nsec, const std::vector<std::vector<sick_scansegment_xd::PointXYZRAEI32f>>& lidar_points, size_t total_point_count,
  LaserScanMsgMap& laser_scan_msg_map, const std::string& frame_id, bool is_fullframe)
{
#if defined RASPBERRY && RASPBERRY > 0 // laserscan messages deactivated on Raspberry for performance reasons
#else
  // Split lidar points into echos, layers and segments
	struct LaserScanMsgPoint
	{
	  LaserScanMsgPoint(float _range = 0, float _azimuth = 0, float _i = 0) : range(_range), azimuth(_azimuth), i(_i) {}
	  LaserScanMsgPoint(const sick_scansegment_xd::PointXYZRAEI32f& lidar_point) : range(lidar_point.range), azimuth(lidar_point.azimuth), i(lidar_point.i) {}
		float range;     // polar coordinate range in meter
		float azimuth;   // polar coordinate azimuth in radians
		float i;         // intensity
	};
	typedef std::vector<LaserScanMsgPoint> LaserScanMsgPoints; // list of laserscan points in one segment
	typedef std::vector<LaserScanMsgPoints> LaserScanMsgSegments; // list of laserscan segments
	typedef std::map<int,std::map<int,LaserScanMsgSegments>> LaserScanMsgEchoLayerSegments; // LaserScanMsgMap[echo][layer][segment] := laserscan points in one segment
	typedef std::map<int,std::map<int,LaserScanMsgPoints>> LaserScanMsgEchoLayerSortedPoints; // LaserScanMsgEchoLayerSortedPoints[echo][layer] := sorted list of concatenated laserscan points
	struct SortSegmentsAscendingAzimuth
	{
			bool operator()(const LaserScanMsgPoints& segment1, const LaserScanMsgPoints& segment2) { return !segment1.empty() && !segment2.empty() && segment1[0].azimuth < segment2[0].azimuth; }
	};
	struct SortSegmentsDescendingAzimuth
	{
			bool operator()(const LaserScanMsgPoints& segment1, const LaserScanMsgPoints& segment2) { return !segment1.empty() && !segment2.empty() && segment1[0].azimuth > segment2[0].azimuth; }
	};
	struct ScanPointPrinter
	{
		static std::string printAzimuthTable(const std::vector<sick_scansegment_xd::PointXYZRAEI32f>& points)
		{
			std::stringstream s;
			for (int pointIdx = 0; pointIdx < points.size(); pointIdx++)
				s << (pointIdx>0?",":"") << std::fixed << std::setprecision(1) << points[pointIdx].azimuth * 180.0 / M_PI;
			return s.str();
		}
		static std::string printAzimuthTable(const LaserScanMsgPoints& points)
		{
			std::stringstream s;
			for (int pointIdx = 0; pointIdx < points.size(); pointIdx++)
				s << (pointIdx>0?",":"") << std::fixed << std::setprecision(1) << points[pointIdx].azimuth * 180.0 / M_PI;
			return s.str();
		}
	};

	// Determine the maximum number of points among all echoes (multi-return scans)
	size_t maxNumberOfPoints = 0;
	for (int echoIdx = 0; echoIdx < lidar_points.size(); echoIdx++)
	{
		// Update maxNumberOfPoints if the current echo contains more points
		maxNumberOfPoints = lidar_points[echoIdx].size() > maxNumberOfPoints
			? lidar_points[echoIdx].size()
			: maxNumberOfPoints;
	}

	LaserScanMsgEchoLayerSegments points_echo_layer_segment_map; 
	for (int echoIdx = 0; echoIdx < lidar_points.size(); echoIdx++)
	{
		int last_layer = INT_MAX;
		float last_azimuth = FLT_MAX;
		for (int pointIdx = 0; pointIdx < lidar_points[echoIdx].size(); pointIdx++)
		{
			sick_scansegment_xd::PointXYZRAEI32f lidar_point = lidar_points[echoIdx][pointIdx];
			// truncate to 65533.0 for non reflectors
			lidar_point.i = (lidar_point.i > 65533.0) ? 65533.0 : lidar_point.i;
			if (lidar_point.reflectorbit > 0) {
				lidar_point.i = 65534.0;
			}

			bool layer_enabled = (m_laserscan_layer_filter.empty() ? 1 : (m_laserscan_layer_filter[lidar_point.layer]));
			if (layer_enabled)
			{
				LaserScanMsgSegments& point_segment = points_echo_layer_segment_map[lidar_point.echo][lidar_point.layer];
				if (point_segment.empty() || last_layer != lidar_point.layer || std::abs(last_azimuth - lidar_point.azimuth) > (2.0*M_PI/180.0)) // start of a new segment
					point_segment.push_back(LaserScanMsgPoints());
				point_segment.back().push_back(LaserScanMsgPoint(lidar_point)); // append lidar point to last segment
				last_layer = lidar_point.layer;
				last_azimuth = lidar_point.azimuth;
			}
		}
	}

	// Sort segments and concatenate points sorted by azimuth
	LaserScanMsgEchoLayerSortedPoints sorted_points_echo_layer_map;
	for(std::map<int,std::map<int,LaserScanMsgSegments>>::iterator iter_echos = points_echo_layer_segment_map.begin(); iter_echos != points_echo_layer_segment_map.end(); iter_echos++)
	{
		const int& echo = iter_echos->first;
		for(std::map<int,LaserScanMsgSegments>::iterator iter_layer = iter_echos->second.begin(); iter_layer != iter_echos->second.end(); iter_layer++)
		{
			const int& layer = iter_layer->first;
			LaserScanMsgSegments& segments = iter_layer->second;
			// All points within a segment should be ordered by ascending azimuth values. Now we have to sort the segments by ascending start azimuth
			int segment_cnt_azimuth_ascending = 0, segment_cnt_azimuth_descending = 0;
			for(int segment_cnt = 0; segment_cnt < segments.size(); segment_cnt++)
			{
				const LaserScanMsgPoints& segment_points = segments[segment_cnt];
				segment_cnt_azimuth_ascending += ((segment_points.size() > 1 && segment_points[0].azimuth < segment_points[1].azimuth) ? 1 : 0);
				segment_cnt_azimuth_descending += ((segment_points.size() > 1 && segment_points[0].azimuth > segment_points[1].azimuth) ? 1 : 0);
		    // if (is_fullframe)
    		//   ROS_INFO_STREAM("convertPointsToLaserscanMsg(" << (is_fullframe ? "fullframe": "segment") << "): echo" << echo << ", layer" << layer << ", segment" << segment_cnt << ", unsorted azimuth=[" << ScanPointPrinter::printAzimuthTable(segment_points) << "]");
			}
			if(segments.size() > 1)
			{
				if(segment_cnt_azimuth_ascending > 0 && segment_cnt_azimuth_descending > 0)
				{
					ROS_ERROR_STREAM("## ERROR convertPointsToLaserscanMsg(): " << segment_cnt_azimuth_ascending << " segments ordered by ascending azimuth, " << segment_cnt_azimuth_descending << " segments ordered by descending azimuth");
					return; // scan points within a segment must be ordered by either ascending or descending azimuth values
				}
				if (segment_cnt_azimuth_ascending > 0)
			    std::sort(segments.begin(), segments.end(), SortSegmentsAscendingAzimuth());
				else if(segment_cnt_azimuth_descending > 0)
					std::sort(segments.begin(), segments.end(), SortSegmentsDescendingAzimuth());
				else
				{
					ROS_ERROR_STREAM("## ERROR convertPointsToLaserscanMsg(): " << segment_cnt_azimuth_ascending << " segments ordered by ascending azimuth, " << segment_cnt_azimuth_descending << " segments ordered by descending azimuth");
					return; // scan points within a segment must be ordered by either ascending or descending azimuth values
				}
			}
			for(int segment_cnt = 0; segment_cnt < segments.size(); segment_cnt++)
			{
				const LaserScanMsgPoints& segment_points = segments[segment_cnt];
			  sorted_points_echo_layer_map[echo][layer].insert(sorted_points_echo_layer_map[echo][layer].end(), segment_points.begin(), segment_points.end());
		    // if (is_fullframe)
    		//   ROS_INFO_STREAM("convertPointsToLaserscanMsg(" << (is_fullframe ? "fullframe": "segment") << "): echo" << echo << ", layer" << layer << ", segment" << segment_cnt << ", sorted azimuth=[" << ScanPointPrinter::printAzimuthTable(segment_points) << "]");
			}
			// Verify all points within the current segment have ascending or descending order (debug and verification only)
			// for(int segment_cnt = 0; segment_cnt < iter_layer->second.size(); segment_cnt++)
			// {
			//   const LaserScanMsgPoints& segment_points = segments[segment_cnt];
			//   bool azimuth_ascending = (segment_points.size() > 1 && segment_points[0].azimuth < segment_points[1].azimuth);
			//   ROS_INFO_STREAM("convertPointsToLaserscanMsg(" << (is_fullframe ? "fullframe": "segment") << "): echo" << echo << ", layer" << layer << ", segment" << segment_cnt << ": " << segment_points.size() << " points, azimuth in " << (azimuth_ascending ? "ascending":"descending") << " order");
			//   for(int point_cnt = 1; point_cnt < segment_points.size(); point_cnt++)
			//   {
			//   	bool ascending = segment_points[point_cnt - 1].azimuth < segment_points[point_cnt].azimuth;
			// 	  if (azimuth_ascending != ascending)
			//      ROS_ERROR_STREAM("## ERROR convertPointsToLaserscanMsg(): echo" << echo << ", layer" << layer << ", segment" << segment_cnt << ": points in segment not ordered by azimuth");
			//   }
			// }
			// Convert to laserscan message, laser_scan_msg = laser_scan_msg_map[layer]
			LaserScanMsgPoints sorted_points = sorted_points_echo_layer_map[echo][layer];
			if (!sorted_points.empty())
			{
				ros_sensor_msgs::LaserScan& laser_scan_msg = laser_scan_msg_map[echo][layer];
				laser_scan_msg.ranges.clear();
				laser_scan_msg.intensities.clear();
				laser_scan_msg.ranges.reserve(sorted_points.size());
				laser_scan_msg.intensities.reserve(sorted_points.size());
				laser_scan_msg.angle_min = sorted_points.front().azimuth;
				laser_scan_msg.angle_max = sorted_points.back().azimuth;
				laser_scan_msg.range_min = sorted_points.front().range;
				laser_scan_msg.range_max = sorted_points.front().range;
				float delta_azimuth_expected = (laser_scan_msg.angle_max - laser_scan_msg.angle_min) / std::max(1.0f, (float)sorted_points.size() - 1.0f);
				for(int point_cnt = 0; point_cnt < sorted_points.size(); point_cnt++)
				{
					laser_scan_msg.ranges.push_back(sorted_points[point_cnt].range);
					laser_scan_msg.intensities.push_back(sorted_points[point_cnt].i);
					laser_scan_msg.range_min = std::min(sorted_points[point_cnt].range, laser_scan_msg.range_min);
					laser_scan_msg.range_max = std::max(sorted_points[point_cnt].range, laser_scan_msg.range_max);
					// Verify all azimuth values monotonously ordered (debug and verification only)
					// if (point_cnt > 0)
					// {
					// 	float delta_azimuth = sorted_points[point_cnt].azimuth - sorted_points[point_cnt - 1].azimuth;
					// 	if (std::abs(delta_azimuth - delta_azimuth_expected) > 0.1 * M_PI / 180.0)
					// 	  ROS_ERROR_STREAM("## ERROR convertPointsToLaserscanMsg(): delta_azimuth: " << (delta_azimuth * 180.0 / M_PI) << ", expected: " << (delta_azimuth_expected * 180.0 / M_PI) << ", " << point_cnt << ". point of " << sorted_points.size() << " points");
					// }
				}
			}
		}
	}

  // Create laserscan messages for all echos and layers
	int num_echos = (int)lidar_points.size();
	int num_echos_publish = num_echos;
	for(LaserScanMsgMap::iterator laser_scan_echo_iter = laser_scan_msg_map.begin(); laser_scan_echo_iter != laser_scan_msg_map.end(); laser_scan_echo_iter++)
	{
		int echo_idx = laser_scan_echo_iter->first;
		bool echo_enabled = true;
		// If only one echo is activated by FREchoFilter, but 3 echos are received, we apply the FREchoFilter for laserscan messages:
    if (m_host_set_FREchoFilter && num_echos > 1 && (m_host_FREchoFilter == 0 || m_host_FREchoFilter == 2)) // m_host_FREchoFilter == 0: FIRST_ECHO only (EchoCount=1), m_host_FREchoFilter == 1: ALL_ECHOS, m_host_FREchoFilter == 2: LAST_ECHO (EchoCount=1)
		{
			num_echos_publish = 1;
			if (m_host_FREchoFilter == 0 && echo_idx > 0)
			  echo_enabled = false; // m_host_FREchoFilter == 0: FIRST_ECHO only (EchoCount=1)
			else if (m_host_FREchoFilter == 2 && echo_idx < num_echos - 1)
			  echo_enabled = false; // m_host_FREchoFilter == 2: LAST_ECHO only (EchoCount=1)
		}
		if (!echo_enabled)
			continue; 
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
				laser_scan_msg.range_min = std::max(0.05f, laser_scan_msg.range_min); // min range of multiScan and picoScan: 0.05 [m]
				

				laser_scan_msg.header.stamp.sec = timestamp_sec;
#if defined __ROS_VERSION && __ROS_VERSION > 1
				laser_scan_msg.header.stamp.nanosec = timestamp_nsec;
#elif defined __ROS_VERSION && __ROS_VERSION > 0
				laser_scan_msg.header.stamp.nsec = timestamp_nsec;
#endif
				laser_scan_msg.header.frame_id = frame_id + "_" + std::to_string(layer_idx + 1);
				if (num_echos_publish > 1)
				  laser_scan_msg.header.frame_id = laser_scan_msg.header.frame_id + "_" + std::to_string(echo_idx);
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
	// Remove filtered echos
	for(LaserScanMsgMap::iterator laser_scan_echo_iter = laser_scan_msg_map.begin(); laser_scan_echo_iter != laser_scan_msg_map.end(); )
	{
		int echo_idx = laser_scan_echo_iter->first;
		bool echo_found = false;
		std::map<int,ros_sensor_msgs::LaserScan>& laser_scan_layer_map = laser_scan_echo_iter->second;
		for(std::map<int,ros_sensor_msgs::LaserScan>::iterator laser_scan_msg_iter = laser_scan_layer_map.begin(); !echo_found && laser_scan_msg_iter != laser_scan_layer_map.end(); laser_scan_msg_iter++)
		{
			int layer_idx = laser_scan_msg_iter->first;
			ros_sensor_msgs::LaserScan& laser_scan_msg = laser_scan_msg_iter->second;
			if (!laser_scan_msg.header.frame_id.empty() && !laser_scan_msg.ranges.empty())
				echo_found = true;
		}
		if(!echo_found)
			laser_scan_echo_iter = laser_scan_msg_map.erase(laser_scan_echo_iter);
		else
			laser_scan_echo_iter++;
	}
#endif // !RASPBERRY
}

/*
 * Callback function of MsgPackExportListenerIF. HandleMsgPackData() will be called in MsgPackExporter
 * for each registered listener after msgpack data have been received and converted.
 * This function converts and publishes msgpack data to PointCloud2 messages.
 */
void sick_scansegment_xd::RosMsgpackPublisher::HandleMsgPackData(const sick_scansegment_xd::ScanSegmentParserOutput& msgpack_data)
{
	if (!m_active)
		return; // publishing deactivated
	// Publish optional IMU data
	if (msgpack_data.scandata.empty() && msgpack_data.imudata.valid)
	{
		ROS_DEBUG_STREAM("Publishing IMU data: { " << msgpack_data.imudata.to_string() << " }");
		// Convert to ros_sensor_msgs::Imu
		ros_sensor_msgs::Imu imu_msg;
		imu_msg.header.stamp.sec = msgpack_data.timestamp_sec;
#if defined __ROS_VERSION && __ROS_VERSION > 1
		imu_msg.header.stamp.nanosec = msgpack_data.timestamp_nsec;
#else
		imu_msg.header.stamp.nsec = msgpack_data.timestamp_nsec;
#endif
		imu_msg.header.frame_id = m_imu_frame_id;
		imu_msg.orientation.w = msgpack_data.imudata.orientation_w;
		imu_msg.orientation.x = msgpack_data.imudata.orientation_x;
		imu_msg.orientation.y = msgpack_data.imudata.orientation_y;
		imu_msg.orientation.z = msgpack_data.imudata.orientation_z;
		imu_msg.angular_velocity.x = msgpack_data.imudata.angular_velocity_x;
		imu_msg.angular_velocity.y = msgpack_data.imudata.angular_velocity_y;
		imu_msg.angular_velocity.z = msgpack_data.imudata.angular_velocity_z;
		imu_msg.linear_acceleration.x = msgpack_data.imudata.acceleration_x;
		imu_msg.linear_acceleration.y = msgpack_data.imudata.acceleration_y;
		imu_msg.linear_acceleration.z = msgpack_data.imudata.acceleration_z;
		// ros imu message definition: A covariance matrix of all zeros will be interpreted as "covariance unknown"
		for(int n = 0; n < 9; n++)
		{
			imu_msg.orientation_covariance[n] = 0;
			imu_msg.angular_velocity_covariance[n] = 0;
			imu_msg.linear_acceleration_covariance[n] = 0;
		}
		// Publish imu message
		sick_scan_xd::notifyImuListener(m_node, &imu_msg);
		if (m_publisher_imu_initialized)
		{
#if defined __ROS_VERSION && __ROS_VERSION > 1
	   m_publisher_imu->publish(imu_msg);
#else
	   m_publisher_imu.publish(imu_msg);
#endif
		}
		return;
	}
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
	uint64_t lidar_timestamp_start_microsec = std::numeric_limits<uint64_t>::max();
	for (int groupIdx = 0; groupIdx < msgpack_data.scandata.size(); groupIdx++)
	{
		for (int echoIdx = 0; echoIdx < msgpack_data.scandata[groupIdx].scanlines.size(); echoIdx++)
		{
			const std::vector<sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint>& scanline = msgpack_data.scandata[groupIdx].scanlines[echoIdx].points;
			for (int pointIdx = 0; pointIdx < scanline.size(); pointIdx++)
			{
				const sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint& point = scanline[pointIdx];
				lidar_points[echoIdx].push_back(sick_scansegment_xd::PointXYZRAEI32f(point.x, point.y, point.z, point.range,
				   point.azimuth, point.elevation, point.i, point.groupIdx, point.echoIdx, point.lidar_timestamp_microsec, point.reflectorbit));
				lidar_points_min_azimuth = std::min(lidar_points_min_azimuth, point.azimuth);
				lidar_points_max_azimuth = std::max(lidar_points_max_azimuth, point.azimuth);
    		lidar_timestamp_start_microsec = std::min(lidar_timestamp_start_microsec, point.lidar_timestamp_microsec);
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
	// if(m_publish_topic_all_segments != "")
	{
		// ROS_INFO_STREAM("RosMsgpackPublisher::HandleMsgPackData(): check allSegmentsCovered, segment_idx=" << segment_idx);
  	// ROS_DEBUG_STREAM("    RosMsgpackPublisher::HandleMsgPackData(): collected azimuth table = " << printElevationAzimuthTable(m_points_collector.lidar_points));
		float precheck_min_azimuth_deg = m_points_collector.min_azimuth * 180.0f / (float)M_PI;
		float precheck_max_azimuth_deg = m_points_collector.max_azimuth * 180.0f / (float)M_PI;
		bool publish_cloud_360 = (precheck_max_azimuth_deg - precheck_min_azimuth_deg + 1 >= m_all_segments_azimuth_max_deg - m_all_segments_azimuth_min_deg - 1) // fast pre-check
		    && m_points_collector.allSegmentsCovered(m_all_segments_azimuth_min_deg, m_all_segments_azimuth_max_deg, m_all_segments_elevation_min_deg, m_all_segments_elevation_max_deg); // all segments collected in m_points_collector
		// ROS_INFO_STREAM("    RosMsgpackPublisher::HandleMsgPackData(): segment_idx=" << segment_idx << ", m_points_collector.lastSegmentIdx=" << m_points_collector.lastSegmentIdx()
		//     << ", m_points_collector.total_point_count=" << m_points_collector.total_point_count
		//     << ", azimuth_range_collected=(" << precheck_min_azimuth_deg << "," << precheck_max_azimuth_deg << ")=" << (precheck_max_azimuth_deg - precheck_min_azimuth_deg)
		//     << ", azimuth_range_configured=(" << m_all_segments_azimuth_min_deg << "," << m_all_segments_azimuth_max_deg << ")=" << (m_all_segments_azimuth_max_deg - m_all_segments_azimuth_min_deg)
		//     << ", m_points_collector.allSegmentsCovered=" << publish_cloud_360);
		if (m_points_collector.total_point_count <= 0 || m_points_collector.telegram_cnt <= 0 || publish_cloud_360 || m_points_collector.lastSegmentIdx() > segment_idx)
		{
			// 1. publish 360 degree point cloud if all segments collected
			// 2. start a new collection of all points (first time call, all segments covered, or segment index wrap around)
			if (m_points_collector.total_point_count > 0 && m_points_collector.telegram_cnt > 0 && publish_cloud_360)
			{
				// publish 360 degree point cloud
				// scan_time = 1 / scan_frequency = time for a full 360-degree rotation of the sensor
				m_scan_time = (msgpack_data.timestamp_sec + 1.0e-9 * msgpack_data.timestamp_nsec) - (m_points_collector.timestamp_sec + 1.0e-9 * m_points_collector.timestamp_nsec);
				for (int cloud_cnt = 0; cloud_cnt < m_custom_pointclouds_cfg.size(); cloud_cnt++)
				{
					CustomPointCloudConfiguration& custom_pointcloud_cfg = m_custom_pointclouds_cfg[cloud_cnt];
					if (custom_pointcloud_cfg.publish() && custom_pointcloud_cfg.fullframe())
					{
						PointCloud2Msg pointcloud_msg_custom_fields;
						convertPointsToCustomizedFieldsCloud(m_points_collector.timestamp_sec, m_points_collector.timestamp_nsec,  m_points_collector.lidar_timestamp_start_microsec,
						  m_points_collector.lidar_points, custom_pointcloud_cfg, pointcloud_msg_custom_fields);
						publishPointCloud2Msg(m_node, custom_pointcloud_cfg.publisher(), pointcloud_msg_custom_fields, std::max(1, (int)echo_count), -1, custom_pointcloud_cfg.coordinateNotation(), custom_pointcloud_cfg.topic());
						// ROS_INFO_STREAM("RosMsgpackPublisher::HandleMsgPackData(): published " << pointcloud_msg_custom_fields.width << "x" << pointcloud_msg_custom_fields.height << " pointcloud, " << pointcloud_msg_custom_fields.fields.size() << " fields/point, " << pointcloud_msg_custom_fields.data.size() << " bytes");
					}
				}
				// publish 360 degree Laserscan message
				LaserScanMsgMap laser_scan_msg_map; // laser_scan_msg_map[echo][layer] := LaserScan message given echo (Multiscan136: max 3 echos) and layer index (Multiscan136: 16 layer)
				convertPointsToLaserscanMsg(m_points_collector.timestamp_sec, m_points_collector.timestamp_nsec, m_points_collector.lidar_points, m_points_collector.total_point_count, laser_scan_msg_map, m_frame_id, true);
				publishLaserScanMsg(m_node, m_publisher_laserscan_360, laser_scan_msg_map, std::max(1, (int)echo_count), -1);
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
		  // ROS_INFO_STREAM("RosMsgpackPublisher::HandleMsgPackData(): started new point collection with segment_idx=" << segment_idx);
		  // ROS_INFO_STREAM("    RosMsgpackPublisher::HandleMsgPackData(): appendLidarPoints, lidar_points_min_azimuth=" << (lidar_points_min_azimuth * 180.0f / M_PI) << ", lidar_points_max_azimuth=" << (lidar_points_max_azimuth* 180.0f / M_PI));
  		// ROS_DEBUG_STREAM("    RosMsgpackPublisher::HandleMsgPackData(): appendLidarPoints, azimuth table = " << printElevationAzimuthTable(lidar_points));
    	// ROS_DEBUG_STREAM("    RosMsgpackPublisher::HandleMsgPackData(): appendLidarPoints, collected azimuth table = " << printElevationAzimuthTable(m_points_collector.lidar_points));
  		// ROS_DEBUG_STREAM("    RosMsgpackPublisher::HandleMsgPackData(): appendLidarPoints, collected coverage table = " << printCoverageTable(m_points_collector.segment_coverage));
		}
		else if (telegram_cnt > m_points_collector.telegram_cnt) // append lidar points to m_points_collector
		{
			if (m_points_collector.lidar_points.size() < lidar_points.size())
				m_points_collector.lidar_points.resize(lidar_points.size());
			// m_points_collector.segment_count = segment_idx;
			m_points_collector.telegram_cnt = telegram_cnt;
			m_points_collector.total_point_count += total_point_count;
			m_points_collector.appendLidarPoints(lidar_points, segment_idx, telegram_cnt);
			m_points_collector.min_azimuth = std::min(m_points_collector.min_azimuth, lidar_points_min_azimuth);
			m_points_collector.max_azimuth = std::max(m_points_collector.max_azimuth, lidar_points_max_azimuth);
		  // ROS_INFO_STREAM("    RosMsgpackPublisher::HandleMsgPackData(): appendLidarPoints, lidar_points_min_azimuth=" << (lidar_points_min_azimuth * 180.0f / M_PI) << ", lidar_points_max_azimuth=" << (lidar_points_max_azimuth* 180.0f / M_PI));
  		// ROS_DEBUG_STREAM("    RosMsgpackPublisher::HandleMsgPackData(): appendLidarPoints, azimuth table = " << printElevationAzimuthTable(lidar_points));
    	// ROS_DEBUG_STREAM("    RosMsgpackPublisher::HandleMsgPackData(): appendLidarPoints, collected azimuth table = " << printElevationAzimuthTable(m_points_collector.lidar_points));
  		// ROS_DEBUG_STREAM("    RosMsgpackPublisher::HandleMsgPackData(): appendLidarPoints, collected coverage table = " << printCoverageTable(m_points_collector.segment_coverage));
		}
		else
		{
			static fifo_timestamp last_print_timestamp = fifo_clock::now();
			if (sick_scansegment_xd::Fifo<ScanSegmentParserOutput>::Seconds(last_print_timestamp, fifo_clock::now()) > 1.0) // avoid printing with more than 1 Hz
			{
					if (m_points_collector.telegram_cnt > telegram_cnt) // probably test enviroment with recorded and repeated telegrams from pcapng- or upd-player
					{
						ROS_INFO_STREAM("RosMsgpackPublisher::HandleMsgPackData(): current telegram cnt: " << telegram_cnt << ", last telegram cnt in collector: " << m_points_collector.telegram_cnt
							<< ", 360-degree-pointcloud not published (ok if sick_scan_xd is running in a test enviroment with recorded and repeated telegrams from pcapng- or upd-player, otherwise not ok)");
					}
					else
					{
						ROS_WARN_STREAM("## WARNING RosMsgpackPublisher::HandleMsgPackData(): current segment: " << segment_idx << ", last segment in collector: " << m_points_collector.lastSegmentIdx()
							<< ", current telegram: " << telegram_cnt << ", last telegram in collector: " << m_points_collector.telegram_cnt
							<< ", datagram(s) missing, 360-degree-pointcloud not published");
						if (m_points_collector.numEchos() > 1)
						{
							ROS_WARN_STREAM("## WARNING RosMsgpackPublisher::HandleMsgPackData(): " << m_points_collector.numEchos() << " echos received. Activate the echo filter in the launchfile to reduce system load (e.g. last echo only)");
						}
					}
					last_print_timestamp = fifo_clock::now();
			}
			m_points_collector = SegmentPointsCollector(telegram_cnt); // reset pointcloud collector
		}
		// ROS_INFO_STREAM("RosMsgpackPublisher::HandleMsgPackData(): segment_idx " << segment_idx << " of " << m_segment_count << ", " << m_points_collector.total_point_count << " points in collector");
	}

	// Publish PointCloud2 message for the current segment
	for (int cloud_cnt = 0; cloud_cnt < m_custom_pointclouds_cfg.size(); cloud_cnt++)
	{
		CustomPointCloudConfiguration& custom_pointcloud_cfg = m_custom_pointclouds_cfg[cloud_cnt];
		if (custom_pointcloud_cfg.publish() && !custom_pointcloud_cfg.fullframe())
		{
			PointCloud2Msg pointcloud_msg_custom_fields;
			convertPointsToCustomizedFieldsCloud(msgpack_data.timestamp_sec, msgpack_data.timestamp_nsec, lidar_timestamp_start_microsec, lidar_points, custom_pointcloud_cfg, pointcloud_msg_custom_fields);
			publishPointCloud2Msg(m_node, custom_pointcloud_cfg.publisher(), pointcloud_msg_custom_fields, std::max(1, (int)echo_count), segment_idx, custom_pointcloud_cfg.coordinateNotation(), custom_pointcloud_cfg.topic());
			ROS_DEBUG_STREAM("publishPointCloud2Msg: " << pointcloud_msg_custom_fields.width << "x" << pointcloud_msg_custom_fields.height << " pointcloud, " << pointcloud_msg_custom_fields.fields.size() << " fields/point, " << pointcloud_msg_custom_fields.data.size() << " bytes");
		}
	}
#if defined RASPBERRY && RASPBERRY > 0 // laserscan messages deactivated on Raspberry for performance reasons
#else
	LaserScanMsgMap laser_scan_msg_map; // laser_scan_msg_map[echo][layer] := LaserScan message given echo (Multiscan136: max 3 echos) and layer index (Multiscan136: 16 layer)
	convertPointsToLaserscanMsg(msgpack_data.timestamp_sec, msgpack_data.timestamp_nsec, lidar_points, total_point_count, laser_scan_msg_map, m_frame_id, false);
	publishLaserScanMsg(m_node, m_publisher_laserscan_segment, laser_scan_msg_map, std::max(1, (int)echo_count), segment_idx);
#endif
}

/*
 * Returns this instance explicitely as an implementation of interface MsgPackExportListenerIF.
 */
sick_scansegment_xd::MsgPackExportListenerIF* sick_scansegment_xd::RosMsgpackPublisher::ExportListener(void) { return this; }
