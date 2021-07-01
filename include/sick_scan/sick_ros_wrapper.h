/*
 * @brief Wrapper for systemdependent API to Windows/Linux native, ROS-1 and ROS-2
 *
 * Use
 *   #include <sick_scan/sick_ros_wrapper.h>
 * for
 *   #include <ros/ros.h>
 *
 * Copyright (C) 2021, Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2021, SICK AG, Waldkirch
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
 *     * Neither the name of Osnabr�ck University nor the names of its
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
 *  Created on: 12.01.2021
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 */

#ifndef __SICK_ROS_WRAPPER_H_INCLUDED
#define __SICK_ROS_WRAPPER_H_INCLUDED

#define _USE_MATH_DEFINES
#include <math.h>

#if !defined __ROS_VERSION
#define __ROS_VERSION 0 // default: native Linux or Windows
#endif

#if defined _MSC_VER && defined min
#undef min
#endif
#if defined _MSC_VER && defined max
#undef max
#endif
#define MAX std::max
#define MIN std::min

#if __ROS_VERSION <= 1 // ROS-SIMU (native Linux or Windows) or ROS-1 (Linux only)

#if __ROS_VERSION == 0  // native Linux or Windows uses ros simu
#include <sick_scan/rosconsole_simu.hpp>
#define diagnostic_msgs_DiagnosticStatus_ERROR 2
#endif

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

typedef ros::NodeHandle* rosNodePtr;

#define ros_sensor_msgs sensor_msgs
#define ros_std_msgs std_msgs
#define ros_geometry_msgs geometry_msgs
#define ros_visualization_msgs visualization_msgs

#ifndef diagnostic_msgs_DiagnosticStatus_ERROR
#define diagnostic_msgs_DiagnosticStatus_ERROR diagnostic_msgs::DiagnosticStatus::ERROR
#endif
#define ROS_HEADER_SEQ(msgheader,value) msgheader.seq=value

template <typename T> void rosDeclareParam(rosNodePtr nh, const std::string& param_name, const T& param_value) { }
template <typename T> bool rosGetParam(rosNodePtr nh, const std::string& param_name, T& param_value) { return nh->getParam(param_name, param_value); }
template <typename T> void rosSetParam(rosNodePtr nh, const std::string& param_name, const T& param_value) { nh->setParam(param_name, param_value); }

typedef ros::Duration rosDuration;
typedef ros::Time rosTime;
inline rosTime rosTimeNow(void) { return ros::Time::now(); }
inline uint32_t sec(const rosTime& time) { return time.sec; }
inline uint32_t nsec(const rosTime& time) { return time.nsec; }

template <class T> class rosPublisher : public ros::Publisher
{
public:
    rosPublisher() : ros::Publisher() {}
    rosPublisher(ros::Publisher& _publisher) : ros::Publisher(_publisher) {}
};
template <typename T> rosPublisher<T> rosAdvertise(rosNodePtr nh, const std::string& topic, uint32_t queue_size = 10, int qos = 10)
{
    std::string topic2;
    if(topic.empty() || topic[0] != '/')
      topic2 = std::string("/") + topic;
    else
      topic2 = topic;
    ROS_INFO_STREAM("Publishing on topic \"" << topic2 << "\"");
    ros::Publisher publisher = nh->advertise<T>(topic2, queue_size);
    return rosPublisher<T>(publisher);
}
template <typename T> void rosPublish(rosPublisher<T>& publisher, const T& msg) { publisher.publish(msg); }

inline bool rosOk(void) { return ros::ok(); }
inline void rosSpinOnce(rosNodePtr nh) { ros::spinOnce(); }
inline void rosShutdown(void) { ros::shutdown(); }
inline void rosSleep(double seconds) { ros::Duration(seconds).sleep(); }
inline rosDuration rosDurationFromSec(double seconds) { return rosDuration(seconds); }

#include <sick_scan/RadarScan.h>         // generated by msg-generator
#include <sick_scan/Encoder.h>           // generated by msg-generator
#include <sick_scan/LIDoutputstateMsg.h> // generated by msg-generator
#include <sick_scan/LFErecMsg.h>         // generated by msg-generator
#define sick_scan_msg sick_scan

#elif __ROS_VERSION == 2 // ROS-2 (Linux or Windows)

#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

typedef rclcpp::Node::SharedPtr rosNodePtr;

#define ros_sensor_msgs sensor_msgs::msg
#define ros_std_msgs std_msgs::msg
#define ros_geometry_msgs geometry_msgs::msg
#define ros_visualization_msgs visualization_msgs::msg

#ifndef diagnostic_msgs_DiagnosticStatus_ERROR
#define diagnostic_msgs_DiagnosticStatus_ERROR 2 // diagnostic_msgs::msg::DiagnosticStatus::ERROR
#endif
#define ROS_HEADER_SEQ(msgheader,seq)

#define RCLCPP_LOGGER         rclcpp::get_logger("sick_scan")
#define ROS_FATAL(msg)        RCLCPP_FATAL(RCLCPP_LOGGER,msg)
#define ROS_ERROR(msg)        RCLCPP_ERROR(RCLCPP_LOGGER,msg)
#define ROS_WARN(msg)         RCLCPP_WARN(RCLCPP_LOGGER,msg)
#define ROS_INFO(msg)         RCLCPP_INFO(RCLCPP_LOGGER,msg)
#define ROS_DEBUG(msg)        RCLCPP_DEBUG(RCLCPP_LOGGER,msg)
#define ROS_FATAL_STREAM(msg) RCLCPP_FATAL_STREAM(RCLCPP_LOGGER,msg)
#define ROS_ERROR_STREAM(msg) RCLCPP_ERROR_STREAM(RCLCPP_LOGGER,msg)
#define ROS_WARN_STREAM(msg)  RCLCPP_WARN_STREAM(RCLCPP_LOGGER,msg)
#define ROS_INFO_STREAM(msg)  RCLCPP_INFO_STREAM(RCLCPP_LOGGER,msg)
#define ROS_DEBUG_STREAM(msg) RCLCPP_DEBUG_STREAM(RCLCPP_LOGGER,msg)

template <typename T> void rosDeclareParam(rosNodePtr nh, const std::string& param_name, const T& param_value) { if(!nh->has_parameter(param_name)) nh->declare_parameter<T>(param_name, param_value); }
template <typename T> bool rosGetParam(rosNodePtr nh, const std::string& param_name, T& param_value) 
{ 
    bool bRet = nh->get_parameter(param_name, param_value); 
    ROS_DEBUG_STREAM("rosGetParam(" << param_name << "): " << param_value << ", " << typeid(param_value).name()); 
    return bRet; 
}
template <typename T> void rosSetParam(rosNodePtr nh, const std::string& param_name, const T& param_value) 
{ 
    ROS_DEBUG_STREAM("rosSetParam(" << param_name << "," << param_value << ", " << typeid(param_value).name() << ")"); 
    nh->set_parameter(rclcpp::Parameter(param_name, param_value)); 
}

typedef rclcpp::Duration rosDuration;
typedef rclcpp::Time rosTime; // typedef builtin_interfaces::msg::Time rosTime;
inline rosTime rosTimeNow(void) { return rclcpp::Clock().now(); }
inline double sec(const rosTime& time) { return time.seconds(); }
inline double nsec(const rosTime& time) { return (double)time.nanoseconds(); }

template <class T> class rosPublisher : public rclcpp::Publisher<T>::SharedPtr
{
public:
    rosPublisher() : rclcpp::Publisher<T>::SharedPtr(0) {}
    template <class U> rosPublisher(U& _publisher) : rclcpp::Publisher<T>::SharedPtr(_publisher) {}
};
template <class T> rosPublisher<T> rosAdvertise(rosNodePtr nh, const std::string& topic, uint32_t queue_size = 10, rclcpp::QoS qos = rclcpp::SystemDefaultsQoS())
{
    ROS_INFO_STREAM("Publishing on topic \"" << topic << "\"");
    auto publisher = nh->create_publisher<T>(topic, qos);
    return rosPublisher<T>(publisher);
}
template <typename T> void rosPublish(rosPublisher<T>& publisher, const T& msg) { publisher->publish(msg); }

inline bool rosOk(void) { return rclcpp::ok(); }
inline void rosSpinOnce(rosNodePtr nh) { rclcpp::spin_some(nh); }
inline void rosShutdown(void) { rclcpp::shutdown(); }
inline void rosSleep(double seconds) { rclcpp::sleep_for(std::chrono::nanoseconds((int64_t)(seconds * 1.0e9))); }
inline rosDuration rosDurationFromSec(double seconds) { return rosDuration(std::chrono::nanoseconds((int64_t)(seconds * 1.0e9))); }

#include <sick_scan/msg/radar_scan.hpp>          // generated by msg-generator
#include <sick_scan/msg/encoder.hpp>             // generated by msg-generator
#include <sick_scan/msg/li_doutputstate_msg.hpp> // generated by msg-generator
#include <sick_scan/msg/lf_erec_msg.hpp>         // generated by msg-generator
#define sick_scan_msg sick_scan::msg

#else

#error __ROS_VERSION undefined or unsupported, build with __ROS_VERSION 0, 1 or 2

#endif //__ROS_VERSION

/*
** dynamic reconfiguration and diagnostic_updater currently supported on ROS-1 only, todo...
*/
#if __ROS_VERSION == 2
namespace sick_scan
{
    class SickScanConfig { // sick_scan2/include/sick_scan/SickScanConfig.h
    public:
        std::string frame_id = "cloud";
        std::string imu_frame_id = "imu_link";
        bool intensity = false;
        bool auto_reboot = false;
        double min_ang = -M_PI / 2;
        double max_ang = +M_PI / 2;
        double ang_res = 0;
        int skip = 0;
        bool sw_pll_only_publish = false;
        double time_offset = 0;
        int cloud_output_mode = 0;
    };
}
#else // ROS 1
#include <dynamic_reconfigure/server.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sick_scan/SickScanConfig.h>
#define USE_DYNAMIC_RECONFIGURE
#define USE_DIAGNOSTIC_UPDATER
#endif

/*
** ROS-2 requires lowercase field names in all idl generated message structs
*/
#if __ROS_VERSION > 0
#define radarPreHeader radarpreheader
#define uiVersionNo uiversionno
#define radarPreHeaderDeviceBlock radarpreheaderdeviceblock
#define radarPreHeaderStatusBlock radarpreheaderstatusblock
#define radarPreHeaderMeasurementParam1Block radarpreheadermeasurementparam1block
#define radarPreHeaderArrayEncoderBlock radarpreheaderarrayencoderblock
#define uiIdent uiident
#define udiSerialNo udiserialno
#define bDeviceError bdeviceerror
#define bContaminationWarning bcontaminationwarning
#define bContaminationError bcontaminationerror
#define udiEncoderPos udiencoderpos
#define iEncoderSpeed iencoderspeed
#define uiCycleDuration uicycleduration
#define uiNoiseLevel uinoiselevel
#define uiTelegramCount uitelegramcount
#define uiCycleCount uicyclecount
#define udiSystemCountScan udisystemcountscan
#define udiSystemCountTransmit udisystemcounttransmit
#define uiInputs uiinputs
#define uiOutputs uioutputs
#endif

#endif // __SICK_ROS_WRAPPER_H_INCLUDED
