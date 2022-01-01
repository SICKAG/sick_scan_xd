/*
 * @brief ros_wrapper encapsulates the ROS API and switches ROS specific implementation
 * between ROS 1 and ROS 2.
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
#include <thread>

#include "sick_scan/ros_wrapper.h"

#if defined __ROS_VERSION && __ROS_VERSION == 2
static rclcpp::Clock s_wrapper_clock;
#endif

/** Creates a new ros node, shortcut for new ros::NodeHandle() on ROS1 resp. rclcpp::Node::make_shared(node_name) on ROS2 */
ROS::NodePtr ROS::createNode(const std::string& node_name)
{
  ROS::NodePtr nh = 0;
#if defined __ROS_VERSION && __ROS_VERSION == 1
  nh = new ros::NodeHandle();
#elif defined __ROS_VERSION && __ROS_VERSION == 2
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  nh = rclcpp::Node::make_shared(node_name, "", node_options);
#endif
  double timestamp_sec = ROS::secondsSinceStart(ROS::now());
  if ((timestamp_sec = ROS::secondsSinceStart(ROS::now())) < 0) // insure start timestamp after initialization of the first node
    ROS_WARN_STREAM("## ERROR ROS::createNode(): time confusion, ROS::secondsSinceStart(ROS::now()) returned negative seconds " << timestamp_sec);
  return nh;
}

#if defined __ROS_VERSION && __ROS_VERSION == 2
/** ROS1-/ROS2-compatible shortcut for rclcpp::init(argc, argv); */
void ROS::init(int argc, char** argv, const std::string nodename)
{
  rclcpp::init(argc, argv);
}
#endif

/** ROS1-/ROS2-compatible shortcut for ros::spin(); */
#if defined __ROS_VERSION && __ROS_VERSION <= 1
void ROS::spin(ROS::NodePtr nh)
{
  ros::spin();
}
#endif

/** Deletes a ros node */
void ROS::deleteNode(ROS::NodePtr & node)
{
#if defined __ROS_VERSION && __ROS_VERSION == 1
  if(node)
  {
    delete(node);
    node = 0;
  }
#endif
}

/** Shortcut to replace ros::Duration(seconds).sleep() by std::thread */
void ROS::sleep(double seconds)
{
  std::this_thread::sleep_for(std::chrono::microseconds((int64_t)(1.0e6*seconds)));
}

/** Shortcut to ros::Time::now() on ROS1 resp. rclcpp::Clock::now() on ROS2 */
ROS::Time ROS::now(void)
{
#if defined __ROS_VERSION && __ROS_VERSION <= 1
  return ros::Time::now();
#elif defined __ROS_VERSION && __ROS_VERSION == 2
  return s_wrapper_clock.now();
#else
  return 0;
#endif
}

/** Splits a ROS::Duration into seconds and nanoseconds part */
void ROS::splitTime(ROS::Duration time, uint32_t& seconds, uint32_t& nanoseconds)
{
#if defined __ROS_VERSION && __ROS_VERSION == 1
  seconds = time.sec;
  nanoseconds = time.nsec;
#elif defined __ROS_VERSION && __ROS_VERSION == 2
  seconds = (uint32_t)(time.nanoseconds() / 1000000000);
  nanoseconds = (uint32_t)(time.nanoseconds() - 1000000000 * seconds);
#endif
}

/** Splits a ROS::Time into seconds and nanoseconds part */
void ROS::splitTime(ROS::Time time, uint32_t& seconds, uint32_t& nanoseconds)
{
#if defined __ROS_VERSION && __ROS_VERSION == 1
  seconds = time.sec;
  nanoseconds = time.nsec;
#elif defined __ROS_VERSION && __ROS_VERSION == 2
  seconds = (uint32_t)(time.nanoseconds() / 1000000000);
  nanoseconds = (uint32_t)(time.nanoseconds() - 1000000000 * seconds);
#endif
}

/** Shortcut to return ros::Time(msg_header->stamp.sec,msg_header->stamp.nsec) on ROS1 resp. rclcpp::Time(msg_header->stamp.sec,msg_header->stamp.nanosec) on ROS2 */
ROS::Time ROS::timeFromHeader(const std_msgs::Header* msg_header)
{
  return ROS::Time(std::max(0,(int32_t)msg_header->stamp.sec), msg_header->stamp.NSEC);
}

/** Returns a time (type ros::Time on ROS1 resp. rclcpp::Time on ROS2) from a given amount of seconds.
 ** Note: ros::Time(1) initializes 1 second, while rclcpp::Time(1) initializes 1 nanosecond.
 ** Do not use the Time constructor with one parameter! Always use ROS::timeFromSec(seconds)
 ** or ROS::Time(int32_t seconds, uint32_t nanoseconds). */
ROS::Time ROS::timeFromSec(double seconds)
{
  int32_t i32seconds = (int32_t)(seconds);
  int32_t i32nanoseconds = (int32_t)(1000000000 * (seconds - i32seconds));
  return ROS::Time(std::max(0,i32seconds), std::max(0,i32nanoseconds));
}

/** Returns a duration (type ros::Duration on ROS1 resp. rclcpp::Duration on ROS2) from a given amount of seconds.
 ** Note: ros::Duration(1) initializes 1 second, while rclcpp::Duration(1) initializes 1 nanosecond.
 ** Do not use the Duration constructor with one parameter! Always use ROS::durationFromSec(seconds)
 ** or ROS::Duration(int32_t seconds, uint32_t nanoseconds). */
ROS::Duration ROS::durationFromSec(double seconds)
{
  int32_t i32seconds = (int32_t)(seconds);
  int32_t i32nanoseconds = (int32_t)(1000000000 * (seconds - i32seconds));
  return ROS::Duration(i32seconds, i32nanoseconds);
}

/** Shortcut to return ros::Duration::toSec() on ROS1 resp. rclcpp::Duration::seconds() on ROS2 */
double ROS::seconds(ROS::Duration duration)
{
#if defined __ROS_VERSION && __ROS_VERSION == 1
  return duration.toSec();
#elif defined __ROS_VERSION && __ROS_VERSION == 2
  return duration.seconds();
#else
  return 0;
#endif
}

/** Shortcut to return the time in seconds since program start (first node started) */
double ROS::secondsSinceStart(const ROS::Time& time)
{
  static ROS::Time s_wrapper_start_time = ROS::now();
  return ROS::seconds(time - s_wrapper_start_time);
}

/** Shortcut to return the timestamp in milliseconds from ROS1 resp. ROS2 time */
uint64_t ROS::timestampMilliseconds(const ROS::Time& time)
{
#if defined __ROS_VERSION && __ROS_VERSION == 1
  return (uint64_t)(time.sec * 1000) + (uint64_t)(time.nsec / 1000000);
#elif defined __ROS_VERSION && __ROS_VERSION == 2
  return (uint64_t)(time.nanoseconds() / 1000000);
#else
  return 0;
#endif
}
