#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * Copyright (C) 2015, DFKI GmbH
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
 *     * Neither the name of DFKI GmbH nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 *  Created on: 23.11.2015
 *
 *      Authors:
 *         Martin Günther <martin.guenther@dfki.de>
 *         Jochen Sprickerhof <jochen@sprickerhof.de>
 *
 *  Modified and ported to ROS2: 02.10.2020 by Ing.-Buero Dr. Michael Lehning, Hildesheim
 */

#ifndef SICK_LDMRS800001S01_H_
#define SICK_LDMRS800001S01_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include <cfloat>
#include <chrono>
#include <future>
#include <memory>

// #include <dynamic_reconfigure/server.h>
// #include <sick_ldmrs_driver/SickLDMRSDriverConfig.h>
#include "sick_scan/ldmrs/sick_ldmrs_config.hpp"
#include <sick_scan/sick_cloud_transform.h>
#include "sick_scan/sick_range_filter.h"

#include <sick_ldmrs/manager.hpp>
#include <sick_ldmrs/application/BasicApplication.hpp>
#include <sick_ldmrs/datatypes/Object.hpp>

#if __ROS_VERSION == 2 // ROS-2 (Linux or Windows)
#include <sick_scan_xd/msg/sick_ldmrs_object_array.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp> // part of diagnostic_msgs of ROS2, not available on ROS2-Windows until foxy patch 4
#include <diagnostic_updater/publisher.hpp>
#else
#include <sick_scan_xd/SickLdmrsObjectArray.h>
#endif

namespace sick_ldmrs_driver
{

// Adapter to combine publisher for sensor_msgs::msg::PointCloud2 and diagnostic_msgs::msg::DiagnosticArray.
// see https://github.com/ros/diagnostics/blob/eloquent/diagnostic_updater/include/diagnostic_updater/publisher.hpp
// and https://github.com/ros/diagnostics/issues/164
template <class DiagnosedPublisherT> class DiagnosedPublishAdapter : public diagnostic_updater::TopicDiagnostic
{
public:
  template <typename PublisherType> DiagnosedPublishAdapter(PublisherType publisher, diagnostic_updater::Updater & diag,
    const diagnostic_updater::FrequencyStatusParam & freq, const diagnostic_updater::TimeStampStatusParam & stamp)
  : diagnostic_updater::TopicDiagnostic(rosTopicName(publisher), diag, freq, stamp), publisher_(publisher)
  {
  }
  virtual ~DiagnosedPublishAdapter()
  {
  }
  template <typename MessageType> void publish(const std::shared_ptr<MessageType> & message)
  {
    if(message)
      publish(*message);
  }
  template <typename MessageType> void publish(const MessageType & message)
  {
    tick(message.header.stamp);
    rosPublish(publisher_, message); // publisher_->publish(message);
  }
protected:
  DiagnosedPublisherT publisher_;
};

class SickLDMRS : public application::BasicApplication
{
public:
  SickLDMRS(rosNodePtr nh, Manager* manager, std::shared_ptr<diagnostic_updater::Updater> diagnostics);
  virtual ~SickLDMRS();
  void init();
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void validate_config(SickLDMRSDriverConfig &conf);
  void update_config(SickLDMRSDriverConfig &new_config, uint32_t level = 0);
#if defined USE_DYNAMIC_RECONFIGURE && __ROS_VERSION == 1
  void update_config_cb(sick_scan_xd::SickLDMRSDriverConfig &new_config, uint32_t level = 0);
#endif
#if defined USE_DYNAMIC_RECONFIGURE && __ROS_VERSION == 2
  rcl_interfaces::msg::SetParametersResult update_config_cb(const std::vector<rclcpp::Parameter> &parameters);
#endif
  void pubObjects(datatypes::ObjectList &objects);

protected:
  std::shared_ptr<diagnostic_updater::Updater> diagnostics_ = 0;
  void setData(BasicData& data);  // Callback for new data from the manager (scans etc.)
  void validate_flexres_resolution(int &res);
  void validate_flexres_start_angle(double &angle1, double &angle2);
  bool isUpsideDown();
  void printFlexResError();
  std::string flexres_err_to_string(const UINT32 code) const;

private:

  // ROS
  rosNodePtr nh_;
  std::string cloud_topic_val = "cloud";
  rosPublisher<ros_sensor_msgs::PointCloud2> pub_;
  rosPublisher<sick_scan_msg::SickLdmrsObjectArray> object_pub_;
  // Diagnostics
  DiagnosedPublishAdapter<rosPublisher<ros_sensor_msgs::PointCloud2>>* diagnosticPub_ = 0;
  // Dynamic Reconfigure
  SickLDMRSDriverConfig config_;
#if defined USE_DYNAMIC_RECONFIGURE && __ROS_VERSION == 1
  dynamic_reconfigure::Server<sick_scan_xd::SickLDMRSDriverConfig> dynamic_reconfigure_server_;
#endif

  // sick_ldmrs library objects
  Manager* manager_;

  // Expected scan frequency. Must be a member variable for access by diagnostics.
  double expected_frequency_;

  bool initialized_;
  sick_scan_xd::SickCloudTransform m_add_transform_xyz_rpy; // Apply an additional transform to the cartesian pointcloud, default: "0,0,0,0,0,0" (i.e. no transform)
  sick_scan_xd::SickRangeFilter m_range_filter;
};

} /* namespace sick_ldmrs_driver */

#endif /* SICK_LDMRS800001S01_H_ */
