/*
 * macro definitions for logging to support both ros and non-ros.
 *
 * Copyright (C) 2022, Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2022, SICK AG, Waldkirch
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
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 */

#ifndef SICK_LOGGING_H_INCLUDED
#define SICK_LOGGING_H_INCLUDED
#include <string>
#include <sstream>

// fprintf-like conversion of va_args to string, thanks to https://codereview.stackexchange.com/questions/115760/use-va-list-to-format-a-string
#ifdef WIN32
std::string vargs_to_string(const char* const format, ...);
#else
std::string vargs_to_string(const char *const format, ...) __attribute__ ((format (printf, 1, 2)));
#endif

// Common diagnostic status codes for API and ROS
enum SICK_DIAGNOSTIC_STATUS
{
  OK = 0,    // Normal operation mode: lidar is initialized, sick_scan_xd receives scan data, poinclouds are published (equivalent to ros: diagnostic_msgs::DiagnosticStatus::OK)
  WARN = 1,  // non-fatal warning, operation continues  (equivalent to ros: diagnostic_msgs::DiagnosticStatus::WARN)
  SICK_DIAG_ERROR = 2, // general error, should not occure (equivalent to ros: diagnostic_msgs::DiagnosticStatus::ERROR)
  INIT = 3,  // Initialization (startup) phase: establishing communication or initializing lidar after start or reconnect
  EXIT = 4   // sick_scan_xd exits
};

#define SICK_DIAGNOSTIC_STATUS_WARN (SICK_DIAGNOSTIC_STATUS::WARN)
#define SICK_DIAGNOSTIC_STATUS_ERROR (SICK_DIAGNOSTIC_STATUS::SICK_DIAG_ERROR)

// Set the global diagnostic status and message (OK, WARN, ERROR, INIT or EXIT)
void setDiagnosticStatus(SICK_DIAGNOSTIC_STATUS status_code, const std::string& status_message);

// Returns the global diagnostic status and message (OK, WARN, ERROR, INIT or EXIT)
void getDiagnosticStatus(SICK_DIAGNOSTIC_STATUS& status_code, std::string& status_message);

// Notifies all registered log message listener, i.e. all registered listener callbacks are called for all messages of type INFO, WARN, ERROR or FATAL 
void notifyLogMessageListener(int msg_level, const std::string& message);

// Notifies all registered listener about a new diagnostic status
void notifyDiagnosticListener(SICK_DIAGNOSTIC_STATUS status_code, const std::string& status_message);

// Set verbose level 0=DEBUG, 1=INFO, 2=WARN, 3=ERROR, 4=FATAL or 5=QUIET (equivalent to ros::console::levels),
// i.e. print messages on console above the given verbose level.
// Default verbose level is 1 (INFO), i.e. print informational, warnings and error messages.
void setVerboseLevel(int32_t verbose_level);

// Returns the current verbose level 0=DEBUG, 1=INFO, 2=WARN, 3=ERROR, 4=FATAL or 5=QUIET. Default verbose level is 1 (INFO)
int32_t getVerboseLevel();

#if __ROS_VERSION <= 1 // i.e. native Linux or Windows or ROS-1

#define SICK_INFO_LOG(ros_level,...) do{ std::string _msg=vargs_to_string(__VA_ARGS__); if(ros_level>=getVerboseLevel()){ROS_LOG(ros_level,ROSCONSOLE_DEFAULT_NAME,__VA_ARGS__);} notifyLogMessageListener(ros_level,_msg); }while(0)
#define SICK_INFO_LOG_STREAM(ros_level,args) do{ std::stringstream _msg; _msg<<args; if(ros_level>=getVerboseLevel()){ROS_LOG_STREAM(ros_level,ROSCONSOLE_DEFAULT_NAME,args);} notifyLogMessageListener(ros_level,_msg.str()); }while(0)
#define SICK_ERROR_LOG(ros_level,diag_status,...) do{ std::string _msg=vargs_to_string(__VA_ARGS__); setDiagnosticStatus(diag_status,_msg); if(ros_level>=getVerboseLevel()){ROS_LOG(ros_level,ROSCONSOLE_DEFAULT_NAME,__VA_ARGS__);} notifyLogMessageListener(ros_level,_msg); }while(0)
#define SICK_ERROR_LOG_STREAM(ros_level,diag_status,args) do{ std::stringstream _msg; _msg<<args; setDiagnosticStatus(diag_status,_msg.str()); if(ros_level>=getVerboseLevel()){ROS_LOG_STREAM(ros_level,ROSCONSOLE_DEFAULT_NAME,args);} notifyLogMessageListener(ros_level,_msg.str()); }while(0)

#undef ROS_DEBUG
#undef ROS_DEBUG_STREAM
#define ROS_DEBUG(...) SICK_INFO_LOG(::ros::console::levels::Debug,__VA_ARGS__)
#define ROS_DEBUG_STREAM(args) SICK_INFO_LOG_STREAM(::ros::console::levels::Debug,args)

#undef ROS_INFO
#undef ROS_INFO_STREAM
#define ROS_INFO(...) SICK_INFO_LOG(::ros::console::levels::Info,__VA_ARGS__)
#define ROS_INFO_STREAM(args) SICK_INFO_LOG_STREAM(::ros::console::levels::Info,args)

#undef ROS_WARN
#undef ROS_WARN_STREAM
#define ROS_WARN(...) SICK_ERROR_LOG(::ros::console::levels::Warn,SICK_DIAGNOSTIC_STATUS_WARN,__VA_ARGS__)
#define ROS_WARN_STREAM(args) SICK_ERROR_LOG_STREAM(::ros::console::levels::Warn,SICK_DIAGNOSTIC_STATUS_WARN,args)

#undef ROS_ERROR
#undef ROS_ERROR_STREAM
#define ROS_ERROR(...) SICK_ERROR_LOG(::ros::console::levels::Error,SICK_DIAGNOSTIC_STATUS_ERROR,__VA_ARGS__)
#define ROS_ERROR_STREAM(args) SICK_ERROR_LOG_STREAM(::ros::console::levels::Error,SICK_DIAGNOSTIC_STATUS_ERROR,args)

#undef ROS_FATAL
#undef ROS_FATAL_STREAM
#define ROS_FATAL(...) SICK_ERROR_LOG(::ros::console::levels::Fatal,SICK_DIAGNOSTIC_STATUS_ERROR,__VA_ARGS__)
#define ROS_FATAL_STREAM(args) SICK_ERROR_LOG_STREAM(::ros::console::levels::Fatal,SICK_DIAGNOSTIC_STATUS_ERROR,args)

#elif __ROS_VERSION == 2 // i.e. ROS-2

#define SICK_INFO_LOG(ros_level,...) do{ std::string _msg=vargs_to_string(__VA_ARGS__); notifyLogMessageListener(ros_level,_msg); }while(0)
#define SICK_INFO_LOG_STREAM(ros_level,args) do{ std::stringstream _msg; _msg<<args; notifyLogMessageListener(ros_level,_msg.str()); }while(0)
#define SICK_ERROR_LOG(ros_level,diag_status,...) do{ std::string _msg=vargs_to_string(__VA_ARGS__); setDiagnosticStatus(diag_status,_msg); notifyLogMessageListener(ros_level,_msg); }while(0)
#define SICK_ERROR_LOG_STREAM(ros_level,diag_status,args) do{ std::stringstream _msg; _msg<<args; setDiagnosticStatus(diag_status,_msg.str()); notifyLogMessageListener(ros_level,_msg.str()); }while(0)

#define RCLCPP_LOGGER          rclcpp::get_logger("sick_scan_xd")
#define ROS_FATAL(...)         do{ SICK_ERROR_LOG(4,SICK_DIAGNOSTIC_STATUS_ERROR,__VA_ARGS__); RCLCPP_FATAL(RCLCPP_LOGGER,__VA_ARGS__); }while(0)
#define ROS_ERROR(...)         do{ SICK_ERROR_LOG(3,SICK_DIAGNOSTIC_STATUS_ERROR,__VA_ARGS__); RCLCPP_ERROR(RCLCPP_LOGGER,__VA_ARGS__); }while(0)
#define ROS_WARN(...)          do{ SICK_ERROR_LOG(2,SICK_DIAGNOSTIC_STATUS_WARN,__VA_ARGS__);  RCLCPP_WARN(RCLCPP_LOGGER,__VA_ARGS__);  }while(0)
#define ROS_INFO(...)          do{ SICK_INFO_LOG(1,__VA_ARGS__); RCLCPP_INFO(RCLCPP_LOGGER,__VA_ARGS__); }while(0)
#define ROS_DEBUG(...)         do{ RCLCPP_DEBUG(RCLCPP_LOGGER,__VA_ARGS__); }while(0)
#define ROS_FATAL_STREAM(args) do{ SICK_ERROR_LOG_STREAM(4,SICK_DIAGNOSTIC_STATUS_ERROR,args); RCLCPP_FATAL_STREAM(RCLCPP_LOGGER,args); }while(0)
#define ROS_ERROR_STREAM(args) do{ SICK_ERROR_LOG_STREAM(3,SICK_DIAGNOSTIC_STATUS_ERROR,args); RCLCPP_ERROR_STREAM(RCLCPP_LOGGER,args); }while(0)
#define ROS_WARN_STREAM(args)  do{ SICK_ERROR_LOG_STREAM(2,SICK_DIAGNOSTIC_STATUS_WARN,args);  RCLCPP_WARN_STREAM(RCLCPP_LOGGER,args);  }while(0)
#define ROS_INFO_STREAM(args)  do{ SICK_INFO_LOG_STREAM(1,args); RCLCPP_INFO_STREAM(RCLCPP_LOGGER,args); }while(0)
#define ROS_DEBUG_STREAM(args) do{ RCLCPP_DEBUG_STREAM(RCLCPP_LOGGER,args); }while(0)

#endif // __ROS_VERSION
#endif // SICK_LOGGING_H_INCLUDED
