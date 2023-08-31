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

#if !defined __ROS_VERSION
#define __ROS_VERSION 0 // default: native Linux or Windows
#endif

#if defined _MSC_VER && defined min
#undef min
#endif
#if defined _MSC_VER && defined max
#undef max
#endif

#if defined _MSC_VER
#define MAX_AB(a,b) (((a) > (b)) ? (a) : (b))
#define MIN_AB(a,b) (((a) < (b)) ? (a) : (b))
#else
#define MAX_AB std::max
#define MIN_AB std::min
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


#if __ROS_VERSION <= 1 // ROS-SIMU (native Linux or Windows) or ROS-1 (Linux only)

#if __ROS_VERSION == 0  // native Linux or Windows uses ros simu
#include <sick_scan/rosconsole_simu.hpp>
#define diagnostic_msgs_DiagnosticStatus_OK    0
#define diagnostic_msgs_DiagnosticStatus_WARN  1
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

template <class T> class rosPublisher : public ros::Publisher
{
public:
    rosPublisher() : ros::Publisher() {}
    rosPublisher(ros::Publisher& _publisher) : ros::Publisher(_publisher) {}
};
template <typename T> rosPublisher<T> rosAdvertise(rosNodePtr nh, const std::string& topic, uint32_t queue_size = 10, rosQoS qos = 10)
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
    ros::Publisher publisher = nh->advertise<T>(topic2, queue_size);
    return rosPublisher<T>(publisher);
}
template <typename T> void rosPublish(rosPublisher<T>& publisher, const T& msg) { publisher.publish(msg); }
template <typename T> std::string rosTopicName(rosPublisher<T>& publisher) { return publisher.getTopic(); }

inline bool rosOk(void) { return ros::ok(); }
inline void rosSpin(rosNodePtr nh) { ros::spin(); }
inline void rosSpinOnce(rosNodePtr nh) { ros::spinOnce(); }
inline void rosShutdown(void) { ros::shutdown(); }
inline void rosSleep(double seconds) { ros::Duration(seconds).sleep(); }
inline rosDuration rosDurationFromSec(double seconds) { return rosDuration(seconds); }

#include <sick_scan/RadarScan.h>         // generated by msg-generator
#include <sick_scan/Encoder.h>           // generated by msg-generator
#include <sick_scan/LIDoutputstateMsg.h> // generated by msg-generator
#include <sick_scan/LFErecMsg.h>         // generated by msg-generator
#define sick_scan_msg sick_scan

#include "sick_scan/ColaMsgSrv.h"        // generated by srv-generator
#include "sick_scan/ECRChangeArrSrv.h"   // generated by srv-generator
#include "sick_scan/LIDoutputstateSrv.h" // generated by srv-generator
#include "sick_scan/SCdevicestateSrv.h"  // generated by srv-generator
#include "sick_scan/SCrebootSrv.h"       // generated by srv-generator
#include "sick_scan/SCsoftresetSrv.h"    // generated by srv-generator
#include "sick_scan/SickScanExitSrv.h"   // generated by srv-generator
#define sick_scan_srv sick_scan

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

#define RCLCPP_LOGGER         rclcpp::get_logger("sick_scan")
#define ROS_FATAL(...)        RCLCPP_FATAL(RCLCPP_LOGGER,__VA_ARGS__)
#define ROS_ERROR(...)        RCLCPP_ERROR(RCLCPP_LOGGER,__VA_ARGS__)
#define ROS_WARN(...)         RCLCPP_WARN(RCLCPP_LOGGER,__VA_ARGS__)
#define ROS_INFO(...)         RCLCPP_INFO(RCLCPP_LOGGER,__VA_ARGS__)
#define ROS_DEBUG(...)        RCLCPP_DEBUG(RCLCPP_LOGGER,__VA_ARGS__)
#define ROS_FATAL_STREAM(...) RCLCPP_FATAL_STREAM(RCLCPP_LOGGER,__VA_ARGS__)
#define ROS_ERROR_STREAM(...) RCLCPP_ERROR_STREAM(RCLCPP_LOGGER,__VA_ARGS__)
#define ROS_WARN_STREAM(...)  RCLCPP_WARN_STREAM(RCLCPP_LOGGER,__VA_ARGS__)
#define ROS_INFO_STREAM(...)  RCLCPP_INFO_STREAM(RCLCPP_LOGGER,__VA_ARGS__)
#define ROS_DEBUG_STREAM(...) RCLCPP_DEBUG_STREAM(RCLCPP_LOGGER,__VA_ARGS__)

template <typename T> void rosDeclareParam(rosNodePtr nh, const std::string& param_name, const T& param_value) { if(!nh->has_parameter(param_name)) nh->declare_parameter<T>(param_name, param_value); }
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
        ROS_WARN_STREAM("## ERROR rosGetParam(" << param_name << ", " << paramToString(param_value) << ", " << typeid(param_value).name() << ") failed, exception " << exc.what());
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
        ROS_WARN_STREAM("## ERROR rosSetParam(" << param_name << ", " << paramToString(param_value) << ", " << typeid(param_value).name() << ") failed, exception " << exc.what());
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
template <class T> rosPublisher<T> rosAdvertise(rosNodePtr nh, const std::string& topic, uint32_t queue_size = 10, rosQoS qos = rclcpp::SystemDefaultsQoS())
{
    QoSConverter qos_converter;
    int qos_val = -1;
    rosDeclareParam(nh, "ros_qos", qos_val);
    rosGetParam(nh, "ros_qos", qos_val);
    if (qos_val >= 0)
        qos = qos_converter.convert(qos_val);
    ROS_INFO_STREAM("Publishing on topic \"" << topic << "\", qos=" << qos_converter.convert(qos));
    auto publisher = nh->create_publisher<T>(topic, qos);
    return rosPublisher<T>(publisher);
}
template <typename T> void rosPublish(rosPublisher<T>& publisher, const T& msg) { publisher->publish(msg); }
template <typename T> std::string rosTopicName(rosPublisher<T>& publisher) { return publisher->get_topic_name(); }

inline bool rosOk(void) { return rclcpp::ok(); }
inline void rosSpin(rosNodePtr nh) { rclcpp::spin(nh); }
inline void rosSpinOnce(rosNodePtr nh) { rclcpp::spin_some(nh); }
inline void rosShutdown(void) { rclcpp::shutdown(); }
inline void rosSleep(double seconds) { rclcpp::sleep_for(std::chrono::nanoseconds((int64_t)(seconds * 1.0e9))); }
inline rosDuration rosDurationFromSec(double seconds) { return rosDuration(std::chrono::nanoseconds((int64_t)(seconds * 1.0e9))); }

#include <sick_scan/msg/radar_scan.hpp>          // generated by msg-generator
#include <sick_scan/msg/encoder.hpp>             // generated by msg-generator
#include <sick_scan/msg/li_doutputstate_msg.hpp> // generated by msg-generator
#include <sick_scan/msg/lf_erec_msg.hpp>         // generated by msg-generator
#define sick_scan_msg sick_scan::msg

#include "sick_scan/srv/cola_msg_srv.hpp"        // generated by rosidl-generator
#include "sick_scan/srv/ecr_change_arr_srv.hpp"  // generated by rosidl-generator
#include "sick_scan/srv/li_doutputstate_srv.hpp" // generated by rosidl-generator
#include "sick_scan/srv/s_cdevicestate_srv.hpp"  // generated by rosidl-generator
#include "sick_scan/srv/s_creboot_srv.hpp"       // generated by rosidl-generator
#include "sick_scan/srv/s_csoftreset_srv.hpp"    // generated by rosidl-generator
#include "sick_scan/srv/sick_scan_exit_srv.hpp"  // generated by rosidl-generator

#define sick_scan_srv sick_scan::srv

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
#include <diagnostic_updater/diagnostic_updater.hpp> // part of diagnostic_msgs of ROS2, not available on ROS2-Windows until foxy patch 4
#include <diagnostic_updater/publisher.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#define USE_DYNAMIC_RECONFIGURE
#define USE_DIAGNOSTIC_UPDATER
namespace sick_scan
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
#include <sick_scan/SickLDMRSDriverConfig.h>
#define USE_DYNAMIC_RECONFIGURE
#define USE_DIAGNOSTIC_UPDATER
#endif // WIN32
#include <sick_scan/SickScanConfig.h>
#else
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#endif
#if __ROS_VERSION != 1
namespace sick_scan
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
