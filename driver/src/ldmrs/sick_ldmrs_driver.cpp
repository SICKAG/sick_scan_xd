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
 *  Created on: 20.11.2015
 *
 *      Authors:
 *         Martin Günther <martin.guenther@dfki.de>
 *         Jochen Sprickerhof <jochen@sprickerhof.de>
 *
 *  Modified and ported to ROS2: 02.10.2020 by Ing.-Buero Dr. Michael Lehning, Hildesheim
 */

#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>

#include "sick_scan/ldmrs/sick_ldmrs_driver.hpp"

#include <sick_ldmrs/datatypes/EvalCaseResults.hpp>
#include <sick_ldmrs/datatypes/EvalCases.hpp>
#include <sick_ldmrs/datatypes/Fields.hpp>
#include <sick_ldmrs/datatypes/Measurement.hpp>
#include <sick_ldmrs/datatypes/Msg.hpp>
#include <sick_ldmrs/datatypes/Scan.hpp>
#include <sick_ldmrs/devices/LD_MRS.hpp>
#include <sick_ldmrs/tools/errorhandler.hpp>
#include <sick_ldmrs/tools/toolbox.hpp>

#define ROS_DEBUG_STREAM(msgstream) RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sick_ldmrs_driver"),msgstream)
#define ROS_INFO_STREAM(msgstream)  RCLCPP_INFO_STREAM(rclcpp::get_logger("sick_ldmrs_driver"),msgstream)
#define ROS_WARN_STREAM(msgstream)  RCLCPP_WARN_STREAM(rclcpp::get_logger("sick_ldmrs_driver"),msgstream)
#define ROS_ERROR_STREAM(msgstream) RCLCPP_ERROR_STREAM(rclcpp::get_logger("sick_ldmrs_driver"),msgstream)

static rclcpp::Clock s_rclcpp_clock; // replacement for ros::Time::now()

namespace sick_ldmrs_driver
{

SickLDMRS::SickLDMRS(rclcpp::Node::SharedPtr nh, Manager *manager, boost::shared_ptr<diagnostic_updater::Updater> diagnostics)
  : application::BasicApplication()
  , diagnostics_(diagnostics)
  , manager_(manager)
  , expected_frequency_(12.5)
  , initialized_(false)
  , nh_(nh)
  , config_(nh)
  , pub_(0)
  , object_pub_(0)
  , diagnosticPub_(0)
{
  // point cloud publisher
  pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::SystemDefaultsQoS()); // pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 100);
  // object_pub_ = nh_->create_publisher<sick_ldmrs_msgs::msg::ObjectArray>("objects", rclcpp::SystemDefaultsQoS()); // object_pub_ = nh_.advertise<sick_ldmrs_msgs::ObjectArray>("objects", 1);
  object_pub_ = nh_->create_publisher<sick_scan2::msg::SickLdmrsObjectArray>("objects", rclcpp::SystemDefaultsQoS()); // object_pub_ = nh_.advertise<sick_ldmrs_msgs::ObjectArray>("objects", 1);

  diagnostics_->setHardwareID("none");   // set from device after connection
  diagnostics_->add("device info", this, &SickLDMRS::produce_diagnostics);
  diagnosticPub_ = new DiagnosedPublishAdapter<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>(pub_, *diagnostics_, // diagnosticPub_ = new diagnostic_updater::DiagnosedPublisher<sensor_msgs::PointCloud2>(...)
      diagnostic_updater::FrequencyStatusParam(&expected_frequency_, &expected_frequency_, 0.1, 10), // frequency should be target +- 10%
      diagnostic_updater::TimeStampStatusParam(-1, 1.3 * 1.0 / 12.5)); // timestamp delta can be from -1 seconds to 1.3x what it ideally is at the lowest frequency
}

SickLDMRS::~SickLDMRS()
{
  delete diagnosticPub_;
}

void SickLDMRS::init()
{
  if (isUpsideDown())
  {
    ROS_ERROR_STREAM("Error: upside down mode is active, please disable!");
  }
  initialized_ = true;
  update_config(config_);

  // dynamic_reconfigure::Server<SickLDMRSDriverConfig>::CallbackType f;
  // f = boost::bind(&SickLDMRS::update_config, this, _1, _2);
  // dynamic_reconfigure_server_.setCallback(f);
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_cb_handle =
    nh_->add_on_set_parameters_callback(std::bind(&SickLDMRS::update_config_cb, this, std::placeholders::_1));
  // nh_->set_on_parameters_set_callback(std::bind(&SickLDMRS::update_config_cb, this, std::placeholders::_1));
}

void SickLDMRS::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  devices::LDMRS* ldmrs;
  ldmrs = dynamic_cast<devices::LDMRS*>(manager_->getFirstDeviceByType(Sourcetype_LDMRS));
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Device information.");

  // REP-138 values (http://www.ros.org/reps/rep-0138.html#diagnostic-keys)
  stat.add("IP Address", ldmrs->getIpAddress());
  stat.add("IP Port", 12002);   // LUX port number
  stat.add("Vendor Name", "SICK");
  stat.add("Product Name", "LD-MRS");
  stat.add("Firmware Version", ldmrs->getFirmwareVersion());  // includes date, e.g. "3.03.5 2015-01-14 13:32"
  stat.add("FPGA Version", ldmrs->getFPGAVersion());          // FPGA version including date
  std::string temperature = std::to_string(ldmrs->getTemperature());
  stat.add("Temperature", temperature);
  stat.add("Device ID", ldmrs->getSerialNumber());
}

void SickLDMRS::setData(BasicData &data)
{
  std::string datatypeStr;
  std::string sourceIdStr;

  switch (data.getDatatype())
  {
  case Datatype_Scan:
    datatypeStr = "Scan (" + ::toString(((Scan&)data).getNumPoints()) + " points)";
    {
      Scan* scan = dynamic_cast<Scan*>(&data);
      std::vector<ScannerInfo> scannerInfos = scan->getScannerInfos();
      if (scannerInfos.size() != 1)
      {
        ROS_ERROR_STREAM("Expected exactly 1 scannerInfo, got " << scannerInfos.size());
        return;
      }

      const Time& time = scannerInfos[0].getStartTimestamp();
      ROS_DEBUG_STREAM("setData(): Scan start time: " <<
                time.toString() << " (" <<
                time.toLongString() << ")");

      PointCloud::Ptr cloud = boost::make_shared<PointCloud>();
      cloud->header.frame_id = config_.frame_id;
      // not using time stamp from scanner here, because it is delayed by up to 1.5 seconds
      cloud->header.stamp = s_rclcpp_clock.now().nanoseconds(); // (ros::Time::now().toSec() - 1 / expected_frequency_) * 1e6;

      cloud->height = 1;
      cloud->width = scan->size();
      for (size_t i = 0; i < scan->size(); ++i)
      {
        const ScanPoint& p = (*scan)[i];
        sick_ldmrs_msgs::SICK_LDMRS_Point np;
        np.x = p.getX();
        np.y = p.getY();
        np.z = p.getZ();
        np.echowidth = p.getEchoWidth();
        np.layer = p.getLayer() + (scannerInfos[0].isRearMirrorSide() ? 4 : 0);
        np.echo = p.getEchoNum();
        np.flags = p.getFlags();
        cloud->points.push_back(np);
      }

      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(*cloud, msg);
      if(diagnosticPub_)
        diagnosticPub_->publish(msg);
    }
    break;
  case Datatype_Objects:
    datatypeStr = "Objects (" + ::toString(((ObjectList&)data).size()) + " objects)";
    pubObjects((ObjectList&)data);
    break;
  case Datatype_Fields:
    datatypeStr = "Fields (" + ::toString(((Fields&)data).getFields().size()) + " fields, " +
                  ::toString(((Fields&)data).getNumberOfValidFields()) + " of which are valid)";
    break;
  case Datatype_EvalCases:
    datatypeStr = "EvalCases (" + ::toString(((EvalCases&)data).getEvalCases().size()) + " cases)";
    break;
  case Datatype_EvalCaseResults:
    datatypeStr = "EvalCaseResults (" + ::toString(((EvalCaseResults&)data).size()) + " case results)";
    break;
  case Datatype_Msg:
    datatypeStr = "Msg (" + ((Msg&)data).toString() + ")";
    diagnostics_->broadcast(diagnostic_msgs::msg::DiagnosticStatus::WARN, ((Msg&)data).toString());
    diagnostics_->force_update();
    break;
  case Datatype_MeasurementList:
    datatypeStr = "MeasurementList (" + ::toString(((MeasurementList&)data).m_list.size()) + " entries)";
    break;
  default:
    datatypeStr = "(unknown)";
  }

  sourceIdStr = ::toString(data.getSourceId());

  ROS_DEBUG_STREAM("setData(): Called with data of type " << datatypeStr << " from ID " << sourceIdStr);
}

void SickLDMRS::validate_config(SickLDMRSDriverConfig &conf)
{
  if (conf.start_angle <= conf.end_angle)
  {
    ROS_WARN_STREAM("Start angle must be greater than end angle. Adjusting start_angle.");
    conf.start_angle = conf.end_angle;  // TODO: - 2 * ticks2rad
  }

  if (conf.angular_resolution_type != sick_ldmrs_driver::SickLDMRSDriverConfig::angular_res_enum::ConstantRes // SickLDMRSDriver_ConstantRes
      && conf.scan_frequency != sick_ldmrs_driver::SickLDMRSDriverConfig::scan_freq_enum::ScanFreq1250) // SickLDMRSDriver_ScanFreq1250)
  {
    ROS_WARN_STREAM("Focused/flexible resolution only available at 12.5 Hz scan frequency. Adjusting scan frequency.");
    conf.scan_frequency = sick_ldmrs_driver::SickLDMRSDriverConfig::scan_freq_enum::ScanFreq1250; // SickLDMRSDriver_ScanFreq1250;
  }

  if (conf.ignore_near_range && conf.layer_range_reduction != sick_ldmrs_driver::SickLDMRSDriverConfig::range_reduction_enum::RangeLowerReduced) // SickLDMRSDriver_RangeLowerReduced)
  {
    ROS_WARN_STREAM("If ignore_near_range is set, layer_range_reduction must be set to 'Lower 4 layers reduced range'. Adjusting layer_range_reduction.");
    conf.layer_range_reduction = sick_ldmrs_driver::SickLDMRSDriverConfig::range_reduction_enum::RangeLowerReduced; // SickLDMRSDriver_RangeLowerReduced;
  }

  validate_flexres_resolution(conf.flexres_resolution1);
  validate_flexres_resolution(conf.flexres_resolution2);
  validate_flexres_resolution(conf.flexres_resolution3);
  validate_flexres_resolution(conf.flexres_resolution4);
  validate_flexres_resolution(conf.flexres_resolution5);
  validate_flexres_resolution(conf.flexres_resolution6);
  validate_flexres_resolution(conf.flexres_resolution7);
  validate_flexres_resolution(conf.flexres_resolution8);
  validate_flexres_start_angle(conf.flexres_start_angle1, conf.flexres_start_angle2);
  validate_flexres_start_angle(conf.flexres_start_angle2, conf.flexres_start_angle3);
  validate_flexres_start_angle(conf.flexres_start_angle3, conf.flexres_start_angle4);
  validate_flexres_start_angle(conf.flexres_start_angle4, conf.flexres_start_angle5);
  validate_flexres_start_angle(conf.flexres_start_angle5, conf.flexres_start_angle6);
  validate_flexres_start_angle(conf.flexres_start_angle6, conf.flexres_start_angle7);
  validate_flexres_start_angle(conf.flexres_start_angle7, conf.flexres_start_angle8);
}

void SickLDMRS::validate_flexres_resolution(int &res)
{
  // Check that res is one of 4/8/16/32. This has to be checked manually here, since
  // the dynamic parameter is an int with min 4 and max 32, so dynamic reconfigure
  // doesn't prevent the user from setting an invalid value inside that range.
  // (Values outside that range will still be clamped automatically.)

  switch (res)
  {
  case sick_ldmrs_driver::SickLDMRSDriverConfig::resolution_enum::Res0125: // SickLDMRSDriver_Res0125:
  case sick_ldmrs_driver::SickLDMRSDriverConfig::resolution_enum::Res0250: // SickLDMRSDriver_Res0250:
  case sick_ldmrs_driver::SickLDMRSDriverConfig::resolution_enum::Res0500: // SickLDMRSDriver_Res0500:
  case sick_ldmrs_driver::SickLDMRSDriverConfig::resolution_enum::Res1000: // SickLDMRSDriver_Res1000:
    break;
  default:
    ROS_WARN_STREAM("Invalid flexres resolution " << res << "! Setting to 32 (= 1 degree).");
    res = sick_ldmrs_driver::SickLDMRSDriverConfig::resolution_enum::Res1000; // SickLDMRSDriver_Res1000;
    break;
  }
}

void SickLDMRS::validate_flexres_start_angle(double &angle1, double &angle2)
{
  // make sure the angles are monotonically decreasing
  if (angle2 > angle1)
  {
    angle2 = angle1;
  }
}

void SickLDMRS::pubObjects(datatypes::ObjectList &objects)
{
  // sick_ldmrs_msgs::msg::ObjectArray oa;
  sick_scan2::msg::SickLdmrsObjectArray oa;
  oa.header.frame_id = config_.frame_id;
  // not using time stamp from scanner here, because it is delayed by up to 1.5 seconds
  oa.header.stamp = s_rclcpp_clock.now(); // ros::Time::now();
  oa.objects.resize(objects.size());

  for (int i = 0; i < objects.size(); i++)
  {
    oa.objects[i].id = objects[i].getObjectId();
    oa.objects[i].tracking_time = s_rclcpp_clock.now() - rclcpp::Duration(objects[i].getObjectAge() / expected_frequency_); // ros::Time::now() - ros::Duration(objects[i].getObjectAge() / expected_frequency_);
    oa.objects[i].last_seen = s_rclcpp_clock.now() - rclcpp::Duration(objects[i].getHiddenStatusAge() / expected_frequency_); // ros::Time::now() - ros::Duration(objects[i].getHiddenStatusAge() / expected_frequency_);
    oa.objects[i].velocity.twist.linear.x = objects[i].getAbsoluteVelocity().getX();
    oa.objects[i].velocity.twist.linear.y = objects[i].getAbsoluteVelocity().getY();
    oa.objects[i].velocity.twist.linear.x = objects[i].getAbsoluteVelocity().getX();
    oa.objects[i].velocity.twist.linear.y = objects[i].getAbsoluteVelocity().getY();
    oa.objects[i].velocity.covariance[0] = objects[i].getAbsoluteVelocitySigma().getX();
    oa.objects[i].velocity.covariance[7] = objects[i].getAbsoluteVelocitySigma().getX();

    oa.objects[i].bounding_box_center.position.x = objects[i].getBoundingBoxCenter().getX();
    oa.objects[i].bounding_box_center.position.y = objects[i].getBoundingBoxCenter().getY();
    oa.objects[i].bounding_box_size.x = objects[i].getBoundingBox().getX();
    oa.objects[i].bounding_box_size.y = objects[i].getBoundingBox().getY();

    oa.objects[i].object_box_center.pose.position.x = objects[i].getCenterPoint().getX();
    oa.objects[i].object_box_center.pose.position.y = objects[i].getCenterPoint().getY();
    // oa.objects[i].object_box_center.pose.orientation = tf::createQuaternionMsgFromYaw(objects[i].getCourseAngle());
    tf2::Quaternion q;
	  q.setRPY(0, 0, objects[i].getCourseAngle());
	  oa.objects[i].object_box_center.pose.orientation.x = q.x();
	  oa.objects[i].object_box_center.pose.orientation.y = q.y();
	  oa.objects[i].object_box_center.pose.orientation.z = q.z();
    oa.objects[i].object_box_center.pose.orientation.w = q.w();
    oa.objects[i].object_box_center.covariance[0] = objects[i].getCenterPointSigma().getX();
    oa.objects[i].object_box_center.covariance[7] = objects[i].getCenterPointSigma().getY();
    oa.objects[i].object_box_center.covariance[35] = objects[i].getCourseAngleSigma();
    oa.objects[i].object_box_size.x = objects[i].getObjectBox().getX();
    oa.objects[i].object_box_size.y = objects[i].getObjectBox().getY();

    datatypes::Polygon2D contour = objects[i].getContourPoints();
    oa.objects[i].contour_points.resize(contour.size());
    for (int j = 0; j < contour.size(); j++)
    {
      oa.objects[i].contour_points[j].x = contour[j].getX();
      oa.objects[i].contour_points[j].y = contour[j].getY();
    }

    //std::cout << objects[i].toString() << std::endl;
  }

  object_pub_->publish(oa); // object_pub_.publish(oa);
}

bool SickLDMRS::isUpsideDown()
{
  devices::LDMRS* ldmrs;
  ldmrs = dynamic_cast<devices::LDMRS*>(manager_->getFirstDeviceByType(Sourcetype_LDMRS));
  if (ldmrs == NULL)
  {
    ROS_WARN_STREAM("isUpsideDown: no connection to LDMRS!");
    return true;
  }

  UINT32 code;
  ldmrs->getParameter(devices::ParaUpsideDownMode, &code);
  return (code != 0);
}

void SickLDMRS::printFlexResError()
{
  devices::LDMRS* ldmrs;
  ldmrs = dynamic_cast<devices::LDMRS*>(manager_->getFirstDeviceByType(Sourcetype_LDMRS));
  if (ldmrs == NULL)
  {
    ROS_WARN_STREAM("printFlexResError: no connection to LDMRS!");
    return;
  }

  UINT32 code;
  ldmrs->getParameter(devices::ParaDetailedError, &code);
  std::string msg = flexres_err_to_string(code);
  ROS_ERROR_STREAM("FlexRes detailed error: " << msg);
}

rcl_interfaces::msg::SetParametersResult SickLDMRS::update_config_cb(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  if(!parameters.empty())
  {
    SickLDMRSDriverConfig new_config = config_;
    for (const auto &parameter : parameters)
    {
      ROS_INFO_STREAM("SickLDMRS::update_config_cb(): parameter " << parameter.get_name() << " (type " << rclcpp::to_string(parameter.get_type()) << ") changed to " << parameter.as_string());
      bool ok = false;
      if(parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
        ok = new_config.set_parameter(parameter.get_name(), parameter.as_bool());
      if(parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        ok = new_config.set_parameter(parameter.get_name(), parameter.as_int());
      if(parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        ok = new_config.set_parameter(parameter.get_name(), parameter.as_double());
      if(parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
        ok = new_config.set_parameter(parameter.get_name(), parameter.as_string());
      if(!ok)
      {
        result.successful = false;
        result.reason = "parameter not supported";
      }
    }
    update_config(new_config);
  }
  return result;
}

void SickLDMRS::update_config(SickLDMRSDriverConfig &new_config, uint32_t level)
{
  validate_config(new_config);
  config_ = new_config;

  if (!initialized_)
    return;

  ROS_INFO_STREAM("Updating config...");

  devices::LDMRS* ldmrs;
  ldmrs = dynamic_cast<devices::LDMRS*>(manager_->getFirstDeviceByType(Sourcetype_LDMRS));
  if (ldmrs == NULL)
  {
    ROS_WARN_STREAM("update_config: no connection to LDMRS!");
    return;
  }

  // TODO: if (new_config.start_angle < config_.end_angle): first update end angle,
  // then start angle to ensure that always start_angle > end_angle; see comments
  // in LuxBase::cmd_setScanAngles().
  if (!ldmrs->setScanAngles(new_config.start_angle, new_config.end_angle))
    ROS_WARN_STREAM("Sending param not successful: ");

  switch (config_.scan_frequency)
  {
  case sick_ldmrs_driver::SickLDMRSDriverConfig::scan_freq_enum::ScanFreq1250: // SickLDMRSDriver_ScanFreq1250:
    expected_frequency_ = 12.5;
    break;
  case sick_ldmrs_driver::SickLDMRSDriverConfig::scan_freq_enum::ScanFreq2500: // SickLDMRSDriver_ScanFreq2500:
    expected_frequency_ = 25.0;
    break;
  case sick_ldmrs_driver::SickLDMRSDriverConfig::scan_freq_enum::ScanFreq5000: // SickLDMRSDriver_ScanFreq5000:
    expected_frequency_ = 50.0;
    break;
  default:
    ROS_ERROR_STREAM("Unknown scan frequency: " << config_.scan_frequency);
    break;
  }

  if (!ldmrs->setScanFrequency(expected_frequency_))
    ROS_WARN_STREAM("Sending param not successful: ScanFrequency");
  if (!ldmrs->setSyncAngleOffset(config_.sync_angle_offset))
    ROS_WARN_STREAM("Sending param not successful: SyncAngleOffset");
  if (!ldmrs->setParameter(devices::ParaContourPointDensity, config_.contour_point_density))
    ROS_WARN_STREAM("Sending param not successful: ContourPointDensity");
  if (!ldmrs->setParameter(devices::ParaMinimumObjectAge, config_.min_object_age))
    ROS_WARN_STREAM("Sending param not successful: MinimumObjectAge");
  if (!ldmrs->setParameter(devices::ParaMaximumPredictionAge, config_.max_prediction_age))
    ROS_WARN_STREAM("Sending param not successful: MaximumPredictionAge");
  if (!ldmrs->setParameter(devices::ParaRangeReduction, config_.layer_range_reduction))
    ROS_WARN_STREAM("Sending param not successful: RangeReduction");
  if (!ldmrs->setParameter(devices::ParaIgnoreNearRange, config_.ignore_near_range ? 1 : 0))
    ROS_WARN_STREAM("Sending param not successful: IgnoreNearRange");
  if (!ldmrs->setParameter(devices::ParaSensitivityControl, config_.sensitivity_control ? 1 : 0))
    ROS_WARN_STREAM("Sending param not successful: SensitivityControl");

  if (config_.angular_resolution_type == sick_ldmrs_driver::SickLDMRSDriverConfig::angular_res_enum::FlexRes) // SickLDMRSDriver_FlexRes)
  {
    std::vector<std::pair<int, int> > res_map, res_map_filtered;
    res_map.push_back(std::pair<int, int>(round(config_.flexres_start_angle1 * rad2deg * 32.0), config_.flexres_resolution1));
    res_map.push_back(std::pair<int, int>(round(config_.flexres_start_angle2 * rad2deg * 32.0), config_.flexres_resolution2));
    res_map.push_back(std::pair<int, int>(round(config_.flexres_start_angle3 * rad2deg * 32.0), config_.flexres_resolution3));
    res_map.push_back(std::pair<int, int>(round(config_.flexres_start_angle4 * rad2deg * 32.0), config_.flexres_resolution4));
    res_map.push_back(std::pair<int, int>(round(config_.flexres_start_angle5 * rad2deg * 32.0), config_.flexres_resolution5));
    res_map.push_back(std::pair<int, int>(round(config_.flexres_start_angle6 * rad2deg * 32.0), config_.flexres_resolution6));
    res_map.push_back(std::pair<int, int>(round(config_.flexres_start_angle7 * rad2deg * 32.0), config_.flexres_resolution7));
    res_map.push_back(std::pair<int, int>(round(config_.flexres_start_angle8 * rad2deg * 32.0), config_.flexres_resolution8));

    // --- skip zero-length sectors
    for (int i = 0; i < res_map.size() - 1; ++i)
    {
      if (res_map[i].first > res_map[i + 1].first)
      {
        res_map_filtered.push_back(res_map[i]);
      }
    }
    if (res_map[7].first > (-1918))   // -1918 = minimum start angle
    {
      res_map_filtered.push_back(res_map[7]);
    }

    // --- ensure constraints are met
    int shots_per_scan = 0;
    double sum_res0125 = 0;
    for (int i = 0; i < res_map_filtered.size() - 1; ++i)
    {
      // sector angle in degrees
      double sector_angle = (res_map_filtered[i].first - res_map_filtered[i + 1].first) / 32.0;

      shots_per_scan += sector_angle * 32.0 / res_map_filtered[i].second;
      if (res_map_filtered[i].second == sick_ldmrs_driver::SickLDMRSDriverConfig::resolution_enum::Res0125) // SickLDMRSDriver_Res0125)
      {
        sum_res0125 += sector_angle;
      }
    }

    if (shots_per_scan > 440)
    {
      ROS_WARN_STREAM("FlexRes: The number of shots per scan must be at most 440. Not updating FlexRes config!");
      return;
    }
    if (sum_res0125 > 20.0)
    {
      ROS_WARN_STREAM("FlexRes: The sectors with a resolution of 0.125 deg must not sum up to more than 20 deg. Not updating FlexRes config!");
      return;
    }

    // --- switch to constant resolution
    // when applying FlexRes params, the angular resolution type has to be set
    // to something other than FlexRes
    if (!ldmrs->setParameter(devices::ParaAngularResolutionType, sick_ldmrs_driver::SickLDMRSDriverConfig::angular_res_enum::ConstantRes)) // SickLDMRSDriver_ConstantRes))
      ROS_WARN_STREAM("Sending param not successful: AngularResolutionType");

    // sleep 10 seconds so that new config is applied by the scanner
    usleep(10e6);

    // --- send FlexRes params to scanner
    if (!ldmrs->setParameter(devices::ParaNumSectors, res_map_filtered.size()))
      printFlexResError();

    for (int i = 0; i < res_map_filtered.size(); ++i)
    {
      // set sector start angle
      if (!ldmrs->setParameter((devices::MrsParameterId)(0x4001 + i), res_map_filtered[i].first))
        printFlexResError();

      // set sector resolution
      if (!ldmrs->setParameter((devices::MrsParameterId)(0x4009 + i), res_map_filtered[i].second))
        printFlexResError();
    }
  }

  // set angular resolution type *after* FlexRes config!
  if (!ldmrs->setParameter(devices::ParaAngularResolutionType, config_.angular_resolution_type))
    ROS_WARN_STREAM("Sending param not successful: AngularResolutionType");

  ROS_INFO_STREAM("... done updating config.");
}

std::string SickLDMRS::flexres_err_to_string(const UINT32 code) const
{
  switch (code)
  {
  case devices::ErrFlexResNumShotsInvalid:
    return "The number of shots per scan is higher than 440.";
  case devices::ErrFlexResSizeOneEighthSectorInvalid:
    return "The sectors with a resolution of 0.125 deg sum up to more than 20 deg.";
  case devices::ErrFlexResFreqInvalid:
    return "The scan frequency is not 12.5Hz.";
  case devices::ErrFlexResSectorsOverlapping:
    return "The start angles of the sectors decrease not strictly monotone.";
  case devices::ErrFlexResScannerNotIdle:
    return "Could not set FlexRes parameter because the sensor is in flex res mode and not idle.";
  case devices::ErrFlexResResolutionInvalid:
    return "The resolution of one sector is not 4, 8, 16 or 32 (0.125 deg, 0.25 deg, 0.5 deg, 1 deg).";
  case devices::ErrFlexResNumSectorsInvalid:
    return "The number of sectors is larger than 8.";
  default:
    std::ostringstream ss;
    ss << "UNKNOWN ERROR CODE (" << code << ")";
    return ss.str();
  }
}

} // namespace sick_ldmrs_driver
