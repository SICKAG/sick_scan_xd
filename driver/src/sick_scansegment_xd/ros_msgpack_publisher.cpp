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
	// segment and fullframe pointclouds replaced by customized pointcloud configuration
	// m_publish_topic = config.publish_topic;
	// m_publish_topic_all_segments = config.publish_topic_all_segments;
	m_node = config.node;
	m_laserscan_layer_filter = config.laserscan_layer_filter;
	// m_segment_count = config.segment_count;
	m_all_segments_azimuth_min_deg = (float)config.all_segments_min_deg;
  m_all_segments_azimuth_max_deg = (float)config.all_segments_max_deg;
	if (config.host_set_LFPangleRangeFilter) 
	{
		// Determine all_segments_min/max_deg by LFPangleRangeFilter
		// host_set_LFPangleRangeFilter = "<enabled> <azimuth_start> <azimuth_stop> <elevation_start> <elevation_stop> <beam_increment>" with azimuth and elevation given in degree
    std::vector<std::string> parameter_token;
    sick_scansegment_xd::util::parseVector(config.host_LFPangleRangeFilter, parameter_token, ' ');
		try
		{
			if(parameter_token.size() >= 3 && std::stoi(parameter_token[0]) > 0) // LFPangleRangeFilter enabled, i.e. all_segments_min/max_deg given by LFPangleRangeFilter settings
			{
				float all_segments_azimuth_min_deg = std::stof(parameter_token[1]);
				float all_segments_azimuth_max_deg = std::stof(parameter_token[2]);
				m_all_segments_azimuth_min_deg = std::max<float>(m_all_segments_azimuth_min_deg, all_segments_azimuth_min_deg);
				m_all_segments_azimuth_max_deg = std::min<float>(m_all_segments_azimuth_max_deg, all_segments_azimuth_max_deg);
			}
		}
		catch(const std::exception& e)
		{
			ROS_ERROR_STREAM("## ERROR in RosMsgpackPublisher: can't parse LFPangleRangeFilter settings, exception " << e.what() << ", check LFPangleRangeFilter configuration in the launchfile");
			throw e; // fatal error: rethrow exception, which will be caught in sick_generic_caller and handled by exiting with error
		}
	}

#if defined __ROS_VERSION && __ROS_VERSION > 1 // ROS-2 publisher
    rosQoS qos = rclcpp::SystemDefaultsQoS();
    QoSConverter qos_converter;
    int qos_val = -1;
    rosDeclareParam(m_node, "ros_qos", qos_val);
    rosGetParam(m_node, "ros_qos", qos_val);
    if (qos_val >= 0)
        qos = qos_converter.convert(qos_val);
	  m_publisher_laserscan_segment = create_publisher<ros_sensor_msgs::LaserScan>("~/scan_segment", qos);
		ROS_INFO_STREAM("RosMsgpackPublisher: publishing LaserScan segment messages on topic \"" << m_publisher_laserscan_segment->get_topic_name() << "\"");
	  // m_publisher_laserscan_360 = create_publisher<ros_sensor_msgs::LaserScan>("scan_360", qos);
		// ROS_INFO_STREAM("RosMsgpackPublisher: publishing LaserScan fullframe messages on topic \"" << m_publisher_laserscan_360->get_topic_name() << "\"");
	/* segment and fullframe pointclouds replaced by customized pointcloud configuration
	if(m_publish_topic != "")
	{
	    m_publisher_cur_segment = create_publisher<PointCloud2Msg>(m_publish_topic, qos);
		ROS_INFO_STREAM("RosMsgpackPublisher: publishing PointCloud2 messages on topic \"" << m_publisher_cur_segment->get_topic_name() << "\"");
	}
	if(m_publish_topic_all_segments != "")
	{
	    m_publisher_all_segments = create_publisher<PointCloud2Msg>(m_publish_topic_all_segments, qos);
	} 
	*/
#elif defined __ROS_VERSION && __ROS_VERSION > 0 // ROS-1 publisher
    int qos = 16 * 12 * 3; // 16 layers, 12 segments, 3 echos
		int qos_val = -1;
    rosDeclareParam(m_node, "ros_qos", qos_val);
    rosGetParam(m_node, "ros_qos", qos_val);
    if (qos_val >= 0)
        qos = qos_val;
    m_publisher_laserscan_segment = m_node->advertise<ros_sensor_msgs::LaserScan>("scan_segment", qos);
		// m_publisher_laserscan_360 = m_node->advertise<ros_sensor_msgs::LaserScan>("scan_360", qos);
		/* segment and fullframe pointclouds replaced by customized pointcloud configuration
		if(m_publish_topic != "")
		{
			m_publisher_cur_segment = m_node->advertise<PointCloud2Msg>(m_publish_topic, qos);
		}
		if(m_publish_topic_all_segments != "")
		{
			m_publisher_all_segments = m_node->advertise<PointCloud2Msg>(m_publish_topic_all_segments, qos);
		} 
		*/
#endif

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

        /** Shortcut to publish a PointCloud2Msg */
void sick_scansegment_xd::RosMsgpackPublisher::publishPointCloud2Msg(rosNodePtr node, PointCloud2MsgPublisher& publisher, PointCloud2Msg& pointcloud_msg, int32_t num_echos, int32_t segment_idx, int coordinate_notation)
{
  if (coordinate_notation == 0) // coordinateNotation=0: cartesian (default, pointcloud has fields x,y,z,i) => notify cartesian pointcloud listener
	{
		sick_scan_xd::PointCloud2withEcho cloud_msg_with_echo(&pointcloud_msg, num_echos, segment_idx);
		notifyCartesianPointcloudListener(node, &cloud_msg_with_echo);
	}
#if defined RASPBERRY && RASPBERRY > 0 // polar pointcloud deactivated on Raspberry for performance reasons
#else
  if (coordinate_notation == 1) // coordinateNotation=1: polar (pointcloud has fields azimuth,elevation,r,i) => notify polar pointcloud listener
	{
		sick_scan_xd::PointCloud2withEcho cloud_msg_with_echo(&pointcloud_msg, num_echos, segment_idx);
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

/** Shortcut to publish a PointCloud2Msg
void sick_scansegment_xd::RosMsgpackPublisher::publish(rosNodePtr node, PointCloud2MsgPublisher& publisher, PointCloud2Msg& pointcloud_msg, PointCloud2Msg& pointcloud_msg_polar, 
    LaserscanMsgPublisher& laserscan_publisher, LaserScanMsgMap& laser_scan_msg_map, int32_t num_echos, int32_t segment_idx)
{
	publish(node, publisher, pointcloud_msg, num_echos, segment_idx);
#if defined RASPBERRY && RASPBERRY > 0 // polar pointcloud deactivated on Raspberry for performance reasons
#else
 	publish(node, publisher, pointcloud_msg_polar, num_echos, segment_idx);
#endif
  publish(node, laserscan_publisher, laser_scan_msg_map, num_echos, segment_idx);
}
*/

/*
* Converts the lidarpoints to a customized PointCloud2Msg containing configured fields (e.g. x, y, z, i, range, azimuth, elevation, layer, echo, reflector).
* @param[in] timestamp_sec seconds part of timestamp
* @param[in] timestamp_nsec  nanoseconds part of timestamp
* @param[in] last_timestamp_sec seconds part of last timestamp
* @param[in] last_timestamp_nsec  nanoseconds part of last timestamp
* @param[in] lidar_points list of PointXYZRAEI32f: lidar_points[echoIdx] are the points of one echo
* @param[in] pointcloud_cfg configuration of customized pointcloud
* @param[out] pointcloud_msg customized pointcloud message
*/
void sick_scansegment_xd::RosMsgpackPublisher::convertPointsToCustomizedFieldsCloud(uint32_t timestamp_sec, uint32_t timestamp_nsec, const std::vector<std::vector<sick_scansegment_xd::PointXYZRAEI32f>>& lidar_points, 
  CustomPointCloudConfiguration& pointcloud_cfg, PointCloud2Msg& pointcloud_msg)
{
  // set pointcloud header
  pointcloud_msg.header.stamp.sec = timestamp_sec;
#if defined __ROS_VERSION && __ROS_VERSION > 1
  pointcloud_msg.header.stamp.nanosec = timestamp_nsec;
#elif defined __ROS_VERSION && __ROS_VERSION > 0
  pointcloud_msg.header.stamp.nsec = timestamp_nsec;
#endif
  pointcloud_msg.header.frame_id = pointcloud_cfg.frameid();
  // set pointcloud field properties
	sick_scansegment_xd::PointXYZRAEI32f dummy_lidar_point;
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
			sick_scansegment_xd::PointXYZRAEI32f cur_lidar_point = lidar_points[echo_idx][point_idx];
			if (pointcloud_cfg.pointEnabled(cur_lidar_point))
			{
				size_t pointcloud_offset = point_cnt * pointcloud_msg.point_step; // offset in bytes in pointcloud_msg.data (destination)
				const uint8_t* src_lidar_point = (const uint8_t*)(&cur_lidar_point); // pointer to source lidar point (type sick_scansegment_xd::PointXYZRAEI32f)
				for(int field_idx = 0; field_idx < field_properties.size(); field_idx++)
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
 * Converts the lidarpoints from a msgpack to a PointCloud2Msg and to LaserScan messages for each layer.
 * Note: For performance reasons, LaserScan messages are not created for the collected 360-degree scans (i.e. is_cloud_360 is true).
 * @param[in] timestamp_sec seconds part of timestamp
 * @param[in] timestamp_nsec  nanoseconds part of timestamp
 * @param[in] last_timestamp_sec seconds part of last timestamp
 * @param[in] last_timestamp_nsec  nanoseconds part of last timestamp
 * @param[in] lidar_points list of PointXYZRAEI32f: lidar_points[echoIdx] are the points of one echo
 * @param[in] total_point_count total number of points in all echos
 * @param[out] pointcloud_msg cartesian pointcloud message
 * @param[out] pointcloud_msg_polar polar pointcloud message
 * @param[out] laser_scan_msg_map laserscan message: ros_sensor_msgs::LaserScan for each echo and layer is laser_scan_msg_map[echo][layer]
 */
void sick_scansegment_xd::RosMsgpackPublisher::convertPointsToCloud(uint32_t timestamp_sec, uint32_t timestamp_nsec, const std::vector<std::vector<sick_scansegment_xd::PointXYZRAEI32f>>& lidar_points, size_t total_point_count, 
  PointCloud2Msg& pointcloud_msg, PointCloud2Msg& pointcloud_msg_polar, LaserScanMsgMap& laser_scan_msg_map, bool is_cloud_360, const std::string& frame_id)
{
  // set pointcloud header
  // pointcloud_msg.header.stamp = rosTimeNow();
  pointcloud_msg.header.stamp.sec = timestamp_sec;
#if defined __ROS_VERSION && __ROS_VERSION > 1
  pointcloud_msg.header.stamp.nanosec = timestamp_nsec;
#elif defined __ROS_VERSION && __ROS_VERSION > 0
  pointcloud_msg.header.stamp.nsec = timestamp_nsec;
#endif
  pointcloud_msg.header.frame_id = frame_id;
  
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
  pointcloud_msg.data.clear();
  pointcloud_msg.data.resize(pointcloud_msg.row_step * pointcloud_msg.height, 0);
  float* pfdata = reinterpret_cast<float*>(&pointcloud_msg.data[0]);

#if defined RASPBERRY && RASPBERRY > 0 // polar pointcloud deactivated on Raspberry for performance reasons
#else
  pointcloud_msg_polar = pointcloud_msg;
  pointcloud_msg_polar.fields[0].name = "range";
  pointcloud_msg_polar.fields[1].name = "azimuth";
  pointcloud_msg_polar.fields[2].name = "elevation";
  pointcloud_msg_polar.data.clear();
  pointcloud_msg_polar.data.resize(pointcloud_msg_polar.row_step * pointcloud_msg_polar.height, 0);
  float* pfdata_polar = reinterpret_cast<float*>(&pointcloud_msg_polar.data[0]);
#endif	
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
#if defined RASPBERRY && RASPBERRY > 0 // laserscan messages deactivated on Raspberry for performance reasons
#else
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
#endif			
    }
  }
  if (!is_cloud_360)
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
					laser_scan_msg.header.frame_id = frame_id + "_" + std::to_string(layer_idx);
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
#endif		
  }
}

/*
 * Converts the lidarpoints from a msgpack to a LaserScan messages for each layer.
 * Note: For performance reasons, LaserScan messages are not created for the collected 360-degree scans (i.e. is_cloud_360 is true).
 * @param[in] timestamp_sec seconds part of timestamp
 * @param[in] timestamp_nsec  nanoseconds part of timestamp
 * @param[in] last_timestamp_sec seconds part of last timestamp
 * @param[in] last_timestamp_nsec  nanoseconds part of last timestamp
 * @param[in] lidar_points list of PointXYZRAEI32f: lidar_points[echoIdx] are the points of one echo
 * @param[in] total_point_count total number of points in all echos
 * @param[out] laser_scan_msg_map laserscan message: ros_sensor_msgs::LaserScan for each echo and layer is laser_scan_msg_map[echo][layer]
 */
void sick_scansegment_xd::RosMsgpackPublisher::convertPointsToLaserscanMsg(uint32_t timestamp_sec, uint32_t timestamp_nsec, const std::vector<std::vector<sick_scansegment_xd::PointXYZRAEI32f>>& lidar_points, size_t total_point_count, 
  LaserScanMsgMap& laser_scan_msg_map, const std::string& frame_id)
{
  int echoIdx, pointIdx;
  for (echoIdx = 0; echoIdx < lidar_points.size(); echoIdx++)
  {
    for (pointIdx = 0; pointIdx < lidar_points[echoIdx].size(); pointIdx++)
    {
#if defined RASPBERRY && RASPBERRY > 0 // laserscan messages deactivated on Raspberry for performance reasons
#else
			int echo = lidar_points[echoIdx][pointIdx].echo;
			int layer = lidar_points[echoIdx][pointIdx].layer;
			bool layer_enabled = (m_laserscan_layer_filter.empty() ? 1 : (m_laserscan_layer_filter[layer]));
			if ( layer_enabled)
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
#endif			
    }
  }
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
  				laser_scan_msg.header.stamp.sec = timestamp_sec;
#if defined __ROS_VERSION && __ROS_VERSION > 1
  				laser_scan_msg.header.stamp.nanosec = timestamp_nsec;
#elif defined __ROS_VERSION && __ROS_VERSION > 0
  				laser_scan_msg.header.stamp.nsec = timestamp_nsec;
#endif
					laser_scan_msg.header.frame_id = frame_id + "_" + std::to_string(layer_idx);
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
#endif		
  }
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
			const std::vector<sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint>& scanline = msgpack_data.scandata[groupIdx].scanlines[echoIdx].points;
			for (int pointIdx = 0; pointIdx < scanline.size(); pointIdx++)
			{
				const sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint& point = scanline[pointIdx];
				lidar_points[echoIdx].push_back(sick_scansegment_xd::PointXYZRAEI32f(point.x, point.y, point.z, point.range, 
				   point.azimuth, point.elevation, point.i, point.groupIdx, point.echoIdx, point.reflectorbit));
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
	// if(m_publish_topic_all_segments != "")
	{
		float precheck_min_azimuth_deg = m_points_collector.min_azimuth * 180.0f / (float)M_PI;
		float precheck_max_azimuth_deg = m_points_collector.max_azimuth * 180.0f / (float)M_PI;
		bool publish_cloud_360 = (precheck_max_azimuth_deg - precheck_min_azimuth_deg + 1 >= m_all_segments_azimuth_max_deg - m_all_segments_azimuth_min_deg - 1) // fast pre-check
		    && m_points_collector.allSegmentsCovered(m_all_segments_azimuth_min_deg, m_all_segments_azimuth_max_deg, m_all_segments_elevation_min_deg, m_all_segments_elevation_max_deg); // all segments collected in m_points_collector
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
          for (int cloud_cnt = 0; cloud_cnt < m_custom_pointclouds_cfg.size(); cloud_cnt++)
					{
		        CustomPointCloudConfiguration& custom_pointcloud_cfg = m_custom_pointclouds_cfg[cloud_cnt];
						if (custom_pointcloud_cfg.publish() && custom_pointcloud_cfg.fullframe())
						{
							PointCloud2Msg pointcloud_msg_custom_fields;
              convertPointsToCustomizedFieldsCloud(m_points_collector.timestamp_sec, m_points_collector.timestamp_nsec, m_points_collector.lidar_points, custom_pointcloud_cfg, pointcloud_msg_custom_fields);
              publishPointCloud2Msg(m_node, custom_pointcloud_cfg.publisher(), pointcloud_msg_custom_fields, std::max(1, (int)echo_count), -1, custom_pointcloud_cfg.coordinateNotation());
							ROS_DEBUG_STREAM("publishPointCloud2Msg: " << pointcloud_msg_custom_fields.width << "x" << pointcloud_msg_custom_fields.height << " pointcloud, " << pointcloud_msg_custom_fields.fields.size() << " fields/point, " << pointcloud_msg_custom_fields.data.size() << " bytes");
						}
					}
					// PointCloud2Msg pointcloud_msg, pointcloud_msg_polar;
          // LaserScanMsgMap laser_scan_msg_map; // laser_scan_msg_map[echo][layer] := LaserScan message given echo (Multiscan136: max 3 echos) and layer index (Multiscan136: 16 layer)
					// convertPointsToLaserscanMsg(m_points_collector.timestamp_sec, m_points_collector.timestamp_nsec, m_points_collector.lidar_points, m_points_collector.total_point_count, laser_scan_msg_map, m_frame_id);
					// publishLaserscanMsg(m_node, m_publisher_laserscan_360, laser_scan_msg_map, std::max(1, (int)echo_count), -1);
					// segment and fullframe pointclouds replaced by customized pointcloud configuration
					// convertPointsToCloud(m_points_collector.timestamp_sec, m_points_collector.timestamp_nsec, m_points_collector.lidar_points, m_points_collector.total_point_count, 
					//     pointcloud_msg, pointcloud_msg_polar, laser_scan_msg_map, true, m_frame_id);
					// publish(m_node, m_publisher_all_segments, pointcloud_msg, pointcloud_msg_polar, m_publisher_laserscan_360, laser_scan_msg_map, 
					//     std::max(1, (int)echo_count), -1); // pointcloud, number of echos, segment index (or -1 if pointcloud contains data from multiple segments)
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
			convertPointsToCustomizedFieldsCloud(msgpack_data.timestamp_sec, msgpack_data.timestamp_nsec, lidar_points, custom_pointcloud_cfg, pointcloud_msg_custom_fields);
			publishPointCloud2Msg(m_node, custom_pointcloud_cfg.publisher(), pointcloud_msg_custom_fields, std::max(1, (int)echo_count), segment_idx, custom_pointcloud_cfg.coordinateNotation());
			ROS_DEBUG_STREAM("publishPointCloud2Msg: " << pointcloud_msg_custom_fields.width << "x" << pointcloud_msg_custom_fields.height << " pointcloud, " << pointcloud_msg_custom_fields.fields.size() << " fields/point, " << pointcloud_msg_custom_fields.data.size() << " bytes");
		}
	}
#if defined RASPBERRY && RASPBERRY > 0 // laserscan messages deactivated on Raspberry for performance reasons
#else
	LaserScanMsgMap laser_scan_msg_map; // laser_scan_msg_map[echo][layer] := LaserScan message given echo (Multiscan136: max 3 echos) and layer index (Multiscan136: 16 layer)
	PointCloud2Msg pointcloud_msg_all_fields, pointcloud_msg_segment, pointcloud_msg_segment_polar;
	convertPointsToLaserscanMsg(msgpack_data.timestamp_sec, msgpack_data.timestamp_nsec, lidar_points, total_point_count, laser_scan_msg_map, m_frame_id);
	publishLaserScanMsg(m_node, m_publisher_laserscan_segment, laser_scan_msg_map, std::max(1, (int)echo_count), segment_idx);
	// segment and fullframe pointclouds replaced by customized pointcloud configuration
	// convertPointsToCloud(msgpack_data.timestamp_sec, msgpack_data.timestamp_nsec, lidar_points, total_point_count, pointcloud_msg_segment, pointcloud_msg_segment_polar, laser_scan_msg_map, false, m_frame_id);
	// publish(m_node, m_publisher_cur_segment, pointcloud_msg_segment, pointcloud_msg_segment_polar, m_publisher_laserscan_segment, laser_scan_msg_map, 
	//     std::max(1, (int)echo_count), segment_idx); // pointcloud, number of echos, segment index (or -1 if pointcloud contains data from multiple segments)
#endif
}

/*
 * Returns this instance explicitely as an implementation of interface MsgPackExportListenerIF.
 */
sick_scansegment_xd::MsgPackExportListenerIF* sick_scansegment_xd::RosMsgpackPublisher::ExportListener(void) { return this; }
