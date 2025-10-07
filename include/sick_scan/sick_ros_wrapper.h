#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
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
*     * Neither the name of Osnabrueck University nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
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
#include <memory>
#include <mutex>
#include <string>
#include <sstream>
#include <thread>
#include <vector>
#include <chrono>
#include <cstdarg>
#include <cstdint>

#if !defined __ROS_VERSION
#define __ROS_VERSION 0 // default: native Linux or Windows
#endif

// Default target is little endian. Overwrite SICK_TARGET_ENDIANESS in CMakeLists.txt to build for big endian.
#define SICK_LITTLE_ENDIAN 1 // LITTLE_ENDIAN and BIG_ENDIAN might already be defined differently on a system or in a 3rd party headerfile,
#define SICK_BIG_ENDIAN    2 // SICK_TARGET_ENDIANESS, SICK_LITTLE_ENDIAN and SICK_BIG_ENDIAN are used to avoid conflicts
#ifndef SICK_TARGET_ENDIANESS
#define SICK_TARGET_ENDIANESS SICK_LITTLE_ENDIAN
#endif
#if SICK_TARGET_ENDIANESS==SICK_LITTLE_ENDIAN
#define TARGET_IS_LITTLE_ENDIAN 1
#else
#define TARGET_IS_LITTLE_ENDIAN 0
#endif

#if defined _MSC_VER && defined min
#undef min
#endif
#if defined _MSC_VER && defined max
#undef max
#endif

template <typename T> std::string paramToString(const std::vector<T>& param_value)
{
    std::stringstream s;
    s << param_value.size();
    return s.str();
}

template <typename T> std::string paramToString(const T& param_value)
{
    std::stringstream s;
    s << param_value;
    return s.str();
}

bool shutdownSignalReceived();

#if __ROS_VERSION <= 1 // ROS-SIMU (native Linux or Windows) or ROS-1 (Linux only)

#if __ROS_VERSION == 0  // native Linux or Windows uses ros simu
#include <sick_scan/rosconsole_simu.hpp>
#define diagnostic_msgs_DiagnosticStatus_OK    (SICK_DIAGNOSTIC_STATUS::OK)
#define diagnostic_msgs_DiagnosticStatus_WARN  (SICK_DIAGNOSTIC_STATUS::WARN)
#define diagnostic_msgs_DiagnosticStatus_ERROR (SICK_DIAGNOSTIC_STATUS::SICK_DIAG_ERROR)
#endif

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

typedef ros::NodeHandle* rosNodePtr;

#define ros_nav_msgs nav_msgs
#define ros_sensor_msgs sensor_msgs
#define ros_std_msgs std_msgs
#define ros_geometry_msgs geometry_msgs
#define ros_visualization_msgs visualization_msgs

#ifndef diagnostic_msgs_DiagnosticStatus_OK
#define diagnostic_msgs_DiagnosticStatus_OK diagnostic_msgs::DiagnosticStatus::OK
#endif
#ifndef diagnostic_msgs_DiagnosticStatus_WARN
#define diagnostic_msgs_DiagnosticStatus_WARN diagnostic_msgs::DiagnosticStatus::WARN
#endif
#ifndef diagnostic_msgs_DiagnosticStatus_ERROR
#define diagnostic_msgs_DiagnosticStatus_ERROR diagnostic_msgs::DiagnosticStatus::ERROR
#endif
#define ROS_HEADER_SEQ(msgheader,value) msgheader.seq=value

#include "sick_scan/sick_scan_logging.h"

template <typename T> void rosDeclareParam(rosNodePtr nh, const std::string& param_name, const T& param_value) { }
template <typename T> bool rosGetParam(rosNodePtr nh, const std::string& param_name, T& param_value) { return nh->getParam(param_name, param_value); }
template <typename T> void rosSetParam(rosNodePtr nh, const std::string& param_name, const T& param_value) { nh->setParam(param_name, param_value); }

typedef int rosQoS;
typedef ros::Duration rosDuration;
typedef ros::Time rosTime;
inline rosTime rosTimeNow(void) { return ros::Time::now(); }
inline uint32_t sec(const rosTime& time) { return time.sec; }   // return seconds part of ros::Time
inline uint32_t nsec(const rosTime& time) { return time.nsec; } // return nanoseconds part of ros::Time
inline uint32_t sec(const rosDuration& time) { return time.sec; }   // return seconds part of ros::Duration
inline uint32_t nsec(const rosDuration& time) { return time.nsec; } // return nanoseconds part of ros::Duration
inline uint64_t rosNanosecTimestampNow(void) { rosTime now = rosTimeNow(); return (((uint64_t)sec(now)) * (uint64_t)1000000000) + std::min((uint64_t)nsec(now), (uint64_t)1000000000); }
inline double rosTimeToSeconds(const rosTime& time) { return (double)sec(time) + 1.0e-9 * (double)nsec(time); }

template <class T> class rosPublisher : public ros::Publisher
{
public:
    rosPublisher() : ros::Publisher() {}
    rosPublisher(ros::Publisher& _publisher) : ros::Publisher(_publisher) {}
};
template <typename T> rosPublisher<T> rosAdvertise(rosNodePtr nh, const std::string& topic, uint32_t queue_size = 10, rosQoS qos = 10, bool latch = false)
{
    int qos_val = -1;
    rosDeclareParam(nh, "ros_qos", qos_val);
    rosGetParam(nh, "ros_qos", qos_val);
    if (qos_val >= 0)
        qos = qos_val;
    std::string topic2;
    if(topic.empty() || topic[0] != '/')
      topic2 = std::string("/") + topic;
    else
      topic2 = topic;
    ROS_INFO_STREAM("Publishing on topic \"" << topic2 << "\", qos=" << qos);
    ros::Publisher publisher = nh->advertise<T>(topic2, queue_size, latch);
    return rosPublisher<T>(publisher);
}
template <typename T> void rosPublish(rosPublisher<T>& publisher, const T& msg) { publisher.publish(msg); }
template <typename T> std::string rosTopicName(rosPublisher<T>& publisher) { return publisher.getTopic(); }

inline bool rosOk(void) { return !ros::isShuttingDown() && ros::ok() && !shutdownSignalReceived(); }
inline void rosSpin(rosNodePtr nh) { ros::spin(); }
inline void rosSpinOnce(rosNodePtr nh) { ros::spinOnce(); }
inline void rosShutdown(void) { ros::shutdown(); }
inline void rosSleep(double seconds) { ros::Duration(seconds).sleep(); }
inline rosDuration rosDurationFromSec(double seconds) { return rosDuration(seconds); }

#include <sick_scan_xd/RadarScan.h>         // generated by msg-generator
#include <sick_scan_xd/Encoder.h>           // generated by msg-generator
#include <sick_scan_xd/LIDinputstateMsg.h>  // generated by msg-generator
#include <sick_scan_xd/LIDoutputstateMsg.h> // generated by msg-generator
#include <sick_scan_xd/LFErecMsg.h>         // generated by msg-generator
#include <sick_scan_xd/NAVPoseData.h>       // generated by msg-generator
#include <sick_scan_xd/NAVLandmarkData.h>   // generated by msg-generator
#include <sick_scan_xd/NAVOdomVelocity.h>   // generated by msg-generator
#define sick_scan_msg sick_scan_xd

#include "sick_scan_xd/ColaMsgSrv.h"        // generated by srv-generator
#include "sick_scan_xd/ECRChangeArrSrv.h"   // generated by srv-generator
#include "sick_scan_xd/LIDoutputstateSrv.h" // generated by srv-generator
#include "sick_scan_xd/SCdevicestateSrv.h"  // generated by srv-generator
#include "sick_scan_xd/SCrebootSrv.h"       // generated by srv-generator
#include "sick_scan_xd/SCsoftresetSrv.h"    // generated by srv-generator
#include "sick_scan_xd/SickScanExitSrv.h"   // generated by srv-generator
#include "sick_scan_xd/GetContaminationDataSrv.h" // generated by srv-generator
#include "sick_scan_xd/GetContaminationResultSrv.h" // generated by srv-generator
#define sick_scan_srv sick_scan_xd
#if __ROS_VERSION == 1
#include "sick_scan_xd/FieldSetReadSrv.h"  // generated by srv-generator
#include "sick_scan_xd/FieldSetWriteSrv.h" // generated by srv-generator
#endif

template <class T> class rosServiceClient : public ros::ServiceClient
{
public:
    rosServiceClient() : ros::ServiceClient() {}
    template <class U> rosServiceClient(U& _client) : ros::ServiceClient(_client) {}
};
template <class T> class rosServiceServer : public ros::ServiceServer
{
public:
    rosServiceServer() : ros::ServiceServer() {}
    template <class U> rosServiceServer(U& _server) : ros::ServiceServer(_server) {}
};
#define ROS_CREATE_SRV_CLIENT(nh,srv,name) nh->serviceClient<srv>(name)
#define ROS_CREATE_SRV_SERVER(nh,srv,name,cbfunction,cbobject) nh->advertiseService(name,cbfunction,cbobject)

#elif __ROS_VERSION == 2 // ROS-2 (Linux or Windows)

#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#if __has_include(<tf2/LinearMath/Quaternion.hpp>)
#include <tf2/LinearMath/Quaternion.hpp>
#else
#include <tf2/LinearMath/Quaternion.h>
#endif
#include <tf2_ros/transform_broadcaster.h>

typedef rclcpp::Node::SharedPtr rosNodePtr;

#define ros_nav_msgs nav_msgs::msg
#define ros_sensor_msgs sensor_msgs::msg
#define ros_std_msgs std_msgs::msg
#define ros_geometry_msgs geometry_msgs::msg
#define ros_visualization_msgs visualization_msgs::msg

#ifndef diagnostic_msgs_DiagnosticStatus_OK
#define diagnostic_msgs_DiagnosticStatus_OK    0 //diagnostic_msgs::msg::DiagnosticStatus::OK
#endif
#ifndef diagnostic_msgs_DiagnosticStatus_WARN
#define diagnostic_msgs_DiagnosticStatus_WARN  1 // diagnostic_msgs::msg::DiagnosticStatus::WARN
#endif
#ifndef diagnostic_msgs_DiagnosticStatus_ERROR
#define diagnostic_msgs_DiagnosticStatus_ERROR 2 // diagnostic_msgs::msg::DiagnosticStatus::ERROR
#endif
#define ROS_HEADER_SEQ(msgheader,seq)

#include "sick_scan/sick_scan_logging.h"

inline void rosConvParam(const std::string& str_value, std::string& val){ val = str_value; }
inline void rosConvParam(const std::string& str_value, bool& val){ val = std::stoi(str_value) > 0; }
inline void rosConvParam(const std::string& str_value, int& val){ val = std::stoi(str_value); }
inline void rosConvParam(const std::string& str_value, float& val){ val = std::stof(str_value); }
inline void rosConvParam(const std::string& str_value, double& val){ val = std::stod(str_value); }

template <typename T> void rosDeclareParam(rosNodePtr nh, const std::string& param_name, const T& param_value)
{
    try
    {
        if(!nh->has_parameter(param_name)) 
        {
            nh->declare_parameter<T>(param_name, param_value); 
        }
    }
    catch(const std::exception& exc)
    {
        ROS_WARN_STREAM("## WARNING rosDeclareParam(" << param_name << ", " << paramToString(param_value) << ") failed, exception " << exc.what());
    }
}

template <typename T> bool rosGetParam(rosNodePtr nh, const std::string& param_name, T& param_value)
{
    try
    {
        bool bRet = nh->get_parameter(param_name, param_value);
        ROS_DEBUG_STREAM("rosGetParam(" << param_name << "): " << paramToString(param_value) << ", " << typeid(param_value).name());
        return bRet;
    }
    catch(const std::exception& exc)
    {
        ROS_WARN_STREAM("## WARNING rosGetParam(" << param_name << ", " << paramToString(param_value) << ", " << typeid(param_value).name() << ") failed, " << typeid(exc).name() << ", exception " << exc.what());
    }
    try
    {
        std::string str_value;
        bool bRet = nh->get_parameter(param_name, str_value);
        if (std::is_arithmetic<T>::value)
        {
            rosConvParam(str_value, param_value);
            ROS_INFO_STREAM("rosGetParam(" << param_name << "): converted to " << param_value);
            return bRet;
        }
        else
        {
            ROS_WARN_STREAM("## WARNING rosGetParam(" << param_name << ", " << paramToString(param_value) << ") failed.");
        }
    }
    catch(const std::exception& exc)
    {
        ROS_WARN_STREAM("## WARNING rosGetParam(" << param_name << ", " << paramToString(param_value) << ", " << typeid(param_value).name() << ") failed, " << typeid(exc).name() << ", exception " << exc.what());
    }
    return false;
}
template <typename T> void rosSetParam(rosNodePtr nh, const std::string& param_name, const T& param_value)
{
    try
    {
        ROS_DEBUG_STREAM("rosSetParam(" << param_name << "," << paramToString(param_value) << ", " << typeid(param_value).name() << ")");
        nh->set_parameter(rclcpp::Parameter(param_name, param_value));
    }
    catch(const std::exception& exc)
    {
        ROS_WARN_STREAM("## WARNING rosSetParam(" << param_name << ", " << paramToString(param_value) << ", " << typeid(param_value).name() << ") failed, exception " << exc.what());
    }
}

typedef rclcpp::QoS rosQoS;
typedef rclcpp::Duration rosDuration;
typedef rclcpp::Time rosTime; // typedef builtin_interfaces::msg::Time rosTime;
inline rosTime rosTimeNow(void) { return rclcpp::Clock().now(); }
inline uint32_t sec(const rosTime& time) { return (uint32_t)(time.nanoseconds() / 1000000000); }              // return seconds part of rclcpp::Time
inline uint32_t nsec(const rosTime& time) { return (uint32_t)(time.nanoseconds() - 1000000000 * sec(time)); } // return nanoseconds part of rclcpp::Time
inline uint32_t sec(const rosDuration& time) { return (uint32_t)(time.nanoseconds() / 1000000000); }              // return seconds part of rclcpp::Duration
inline uint32_t nsec(const rosDuration& time) { return (uint32_t)(time.nanoseconds() - 1000000000 * sec(time)); } // return nanoseconds part of rclcpp::Duration
inline uint64_t rosNanosecTimestampNow(void) { rosTime now = rosTimeNow(); return (((uint64_t)sec(now)) * (uint64_t)1000000000) + std::min((uint64_t)nsec(now), (uint64_t)1000000000); }
inline double rosTimeToSeconds(const rosTime& time) { return (double)sec(time) + 1.0e-9 * (double)nsec(time); }

class QoSConverter
{
public:
    int convert(const rosQoS& qos) const
    {
        for (std::map<int,rosQoS>::const_iterator qos_iter = m_qos_map.cbegin(); qos_iter != m_qos_map.cend(); qos_iter++)
            if (qos_iter->second == qos)
                return qos_iter->first;
        return 0;
    }
    rosQoS convert(const int& qos) const
    {
        for (std::map<int,rosQoS>::const_iterator qos_iter = m_qos_map.cbegin(); qos_iter != m_qos_map.cend(); qos_iter++)
            if (qos_iter->first == qos)
                return qos_iter->second;
        return rclcpp::SystemDefaultsQoS();
    }
protected:
    std::map<int,rosQoS> m_qos_map = { {0, rclcpp::SystemDefaultsQoS()}, {1, rclcpp::ParameterEventsQoS()}, {2, rclcpp::ServicesQoS()}, {3, rclcpp::ParametersQoS()}, {4, rclcpp::SensorDataQoS()} };
};

template <class T> class rosPublisher : public rclcpp::Publisher<T>::SharedPtr
{
public:
    rosPublisher() : rclcpp::Publisher<T>::SharedPtr(0) {}
    template <class U> rosPublisher(U& _publisher) : rclcpp::Publisher<T>::SharedPtr(_publisher) {}
};
inline void overwriteByOptionalQOSconfig(rosNodePtr nh, rosQoS& qos)
{
    QoSConverter qos_converter;
    int qos_val = -1;
    rosDeclareParam(nh, "ros_qos", qos_val);
    rosGetParam(nh, "ros_qos", qos_val);
    if (qos_val >= 0)
        qos = qos_converter.convert(qos_val);
}
template <class T> rosPublisher<T> rosAdvertise(rosNodePtr nh, const std::string& topic, uint32_t queue_size = 10, rosQoS qos = rclcpp::SystemDefaultsQoS(), bool latch = false)
{
    if (latch)
        qos = qos.transient_local();
    overwriteByOptionalQOSconfig(nh, qos);
    QoSConverter qos_converter;
    ROS_INFO_STREAM("Publishing on topic \"" << topic << "\", qos=" << qos_converter.convert(qos));
    auto publisher = nh->create_publisher<T>(topic, qos);
    return rosPublisher<T>(publisher);
}
template <typename T> void rosPublish(rosPublisher<T>& publisher, const T& msg) { publisher->publish(msg); }
template <typename T> std::string rosTopicName(rosPublisher<T>& publisher) { return publisher->get_topic_name(); }

inline bool rosOk(void) { return !shutdownSignalReceived() && rclcpp::ok(); }
inline void rosSpin(rosNodePtr nh) { rclcpp::spin(nh); }
inline void rosSpinOnce(rosNodePtr nh) { rclcpp::spin_some(nh); }
inline void rosShutdown(void) { rclcpp::shutdown(); }
inline void rosSleep(double seconds) { rclcpp::sleep_for(std::chrono::nanoseconds((int64_t)(seconds * 1.0e9))); }
inline rosDuration rosDurationFromSec(double seconds) { return rosDuration(std::chrono::nanoseconds((int64_t)(seconds * 1.0e9))); }

#include <sick_scan_xd/msg/radar_scan.hpp>          // generated by msg-generator
#include <sick_scan_xd/msg/encoder.hpp>             // generated by msg-generator
#include <sick_scan_xd/msg/li_dinputstate_msg.hpp>   // generated by msg-generator
#include <sick_scan_xd/msg/li_doutputstate_msg.hpp> // generated by msg-generator
#include <sick_scan_xd/msg/lf_erec_msg.hpp>         // generated by msg-generator
#include <sick_scan_xd/msg/nav_pose_data.hpp>       // generated by msg-generator
#include <sick_scan_xd/msg/nav_landmark_data.hpp>   // generated by msg-generator
#include <sick_scan_xd/msg/nav_odom_velocity.hpp>   // generated by msg-generator
#define sick_scan_msg sick_scan_xd::msg

#include "sick_scan_xd/srv/cola_msg_srv.hpp"        // generated by rosidl-generator
#include "sick_scan_xd/srv/ecr_change_arr_srv.hpp"  // generated by rosidl-generator
#include "sick_scan_xd/srv/li_doutputstate_srv.hpp" // generated by rosidl-generator
#include "sick_scan_xd/srv/s_cdevicestate_srv.hpp"  // generated by rosidl-generator
#include "sick_scan_xd/srv/s_creboot_srv.hpp"       // generated by rosidl-generator
#include "sick_scan_xd/srv/s_csoftreset_srv.hpp"    // generated by rosidl-generator
#include "sick_scan_xd/srv/sick_scan_exit_srv.hpp"  // generated by rosidl-generator
#include "sick_scan_xd/srv/get_contamination_data_srv.hpp" // generated by rosidl-generator
#include "sick_scan_xd/srv/get_contamination_result_srv.hpp" // generated by rosidl-generator
#define sick_scan_srv sick_scan_xd::srv
#include "sick_scan_xd/srv/field_set_read_srv.hpp"   // generated by srv-generator
#include "sick_scan_xd/srv/field_set_write_srv.hpp"  // generated by srv-generator

template <class T> class rosServiceClient : public rclcpp::Client<T>::SharedPtr
{
public:
    rosServiceClient() : rclcpp::Client<T>::SharedPtr(0) {}
    template <class U> rosServiceClient(U& _client) : rclcpp::Client<T>::SharedPtr(_client) {}
};
template <class T> class rosServiceServer : public rclcpp::Service<T>::SharedPtr
{
public:
    rosServiceServer() : rclcpp::Service<T>::SharedPtr(0) {}
    template <class U> rosServiceServer(U& _server) : rclcpp::Service<T>::SharedPtr(_server) {}
};
#define ROS_CREATE_SRV_CLIENT(nh,srv,name) nh->create_client<srv>(name)
#define ROS_CREATE_SRV_SERVER(nh,srv,name,cbfunction,cbobject) nh->create_service<srv>(name,std::bind(cbfunction,cbobject,std::placeholders::_1,std::placeholders::_2))

#else

#error __ROS_VERSION undefined or unsupported, build with __ROS_VERSION 0, 1 or 2

#endif //__ROS_VERSION

/*
** dynamic reconfiguration and diagnostic_updater currently supported on ROS-Linux only, todo...
*/
#if __ROS_VERSION == 2 // ROS 2
#ifdef ROS_DIAGNOSTICS_UPDATER_AVAILABLE
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp> // part of diagnostic_msgs of ROS2, not available on ROS2-Windows until foxy patch 4
#include <diagnostic_updater/publisher.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#define USE_DYNAMIC_RECONFIGURE
#define USE_DIAGNOSTIC_UPDATER
namespace sick_scan_xd
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
}
#endif // ROS_DIAGNOSTICS_UPDATER_AVAILABLE
#elif __ROS_VERSION == 1 // ROS 1
#ifndef WIN32
#include <dynamic_reconfigure/server.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
//#include <sick_scan/SickScanConfig.h>
#include <sick_scan_xd/SickLDMRSDriverConfig.h>
#define USE_DYNAMIC_RECONFIGURE
#define USE_DIAGNOSTIC_UPDATER
#endif // WIN32
#include <sick_scan_xd/SickScanConfig.h>
#else
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#endif
#if __ROS_VERSION != 1
namespace sick_scan_xd
{
    class SickScanConfig { // sick_scan2/include/sick_scan/SickScanConfig.h
    public:
        std::string frame_id = "cloud";
        std::string imu_frame_id = "imu_link";
        bool intensity = true; // false;
        bool auto_reboot = false;
        double min_ang = -M_PI / 2;
        double max_ang = +M_PI / 2;
        double ang_res = 0;
        int skip = 0;
        bool sw_pll_only_publish = false;
        bool use_generation_timestamp = true;
        double time_offset = 0;
        int cloud_output_mode = 0;
    };
}
#endif

#endif // __SICK_ROS_WRAPPER_H_INCLUDED
