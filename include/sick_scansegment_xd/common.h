#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * @brief common.h contains basic and common definition for project sick_scansegment_xd
 * to support the sick 3D lidar multiScan136.
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
#ifndef __SICK_SCANSEGMENT_XD_COMMON_H
#define __SICK_SCANSEGMENT_XD_COMMON_H

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <string>
#include <sstream>
#include <thread>
#include <vector>

#ifdef _MSC_VER
#  include <conio.h>
#  include <direct.h>
#  define KBHIT() ::_kbhit()
#  define GETCH() ::_getch()
#  define SPRINTF sprintf_s
#else
// #include <curses.h>
#  include <sys/stat.h>
#  include <sys/types.h>
#  define localtime_s(a,b) localtime_r(b,a)
#  define KBHIT() false
#  define GETCH() 0
#  define SPRINTF sprintf
#endif


#define SCANDATA_MSGPACK 1
#define SCANDATA_COMPACT 2

#if defined __ROS_VERSION && __ROS_VERSION > 1

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
typedef sensor_msgs::msg::PointCloud2 PointCloud2Msg;
typedef rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr PointCloud2MsgPublisher;
typedef sensor_msgs::msg::LaserScan LaserScanMsg;
typedef rclcpp::Publisher<LaserScanMsg>::SharedPtr LaserscanMsgPublisher;
typedef sensor_msgs::msg::Imu ImuMsg;
typedef rclcpp::Publisher<ImuMsg>::SharedPtr ImuMsgPublisher;
typedef sensor_msgs::msg::PointField PointField;
typedef rclcpp::Clock rosClock;

#elif defined __ROS_VERSION && __ROS_VERSION > 0

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
typedef sensor_msgs::PointCloud2 PointCloud2Msg;
typedef ros::Publisher PointCloud2MsgPublisher;
typedef ros::Publisher LaserscanMsgPublisher;
typedef ros::Publisher ImuMsgPublisher;
typedef sensor_msgs::PointField PointField;
typedef ros::Time rosClock;

#else

typedef ros_sensor_msgs::PointCloud2 PointCloud2Msg;
typedef rosPublisher<PointCloud2Msg> PointCloud2MsgPublisher;
typedef ros_sensor_msgs::LaserScan LaserScanMsg;
typedef rosPublisher<LaserScanMsg> LaserscanMsgPublisher;
typedef ros_sensor_msgs::Imu ImuMsg;
typedef rosPublisher<ImuMsg> ImuMsgPublisher;
typedef ros_sensor_msgs::PointField PointField;
typedef rosTime rosClock;
typedef int rosQoS;

#endif

typedef std::chrono::system_clock chrono_system_clock;
typedef std::chrono::time_point<std::chrono::system_clock> chrono_system_time;
// typedef std::chrono::high_resolution_clock chrono_highres_clock;
// typedef std::chrono::time_point<std::chrono::high_resolution_clock> chrono_highres_time;

namespace sick_scansegment_xd
{
    /*
     * @brief Returns the duration in seconds
     */
    static double Seconds(const chrono_system_time& timestamp_start, const chrono_system_time& timestamp_end = chrono_system_clock::now())
    {
        return (1.0e-9) * (std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp_end - timestamp_start)).count(); // std::chrono::duration::count() in nanoseconds
    }

    /*
     * @brief Formats a number according to formatting options
     */
    template<typename T> static std::string FormatNumber(const T& number, int width = -1, bool setfill = false, bool setfixed = false, int precision = -1)
    {
        std::stringstream stream;
        if(width >= 0)
            stream << std::setw(width);
        if(setfill)
            stream << std::setfill('0');
        if(setfixed)
            stream << std::fixed;
        if(precision >= 0)
            stream << std::setprecision(3);
        stream << number;
        return stream.str();
    }

    /*
     * @brief Returns true, if a file can be opened for reading, otherwise false.
     * @param[in] filename filename incl. path
     */
    static bool FileReadable(const std::string& filename)
    {
        std::ifstream filestream(filename);
        bool fileexists = filestream.is_open();
        filestream.close();
        return fileexists;
    }

    /*
     * @brief Creates a folder
     * @param[in] folder directory name incl. path
     */
    static bool MkDir(const std::string& folder)
    {
        if (!folder.empty() && folder != ".")
        {
            std::string path = folder;
            #ifdef _MSC_VER
            std::replace(path.begin(), path.end(), '/', '\\');
            return (::_mkdir(path.c_str()) == 0);
            #else
            std::replace(path.begin(), path.end(), '\\', '/');
            return (mkdir(path.c_str(), 0777) == 0);
            #endif
        }
        return false;
    }

    /*
     * @brief Extracts and returns the name of a file without optional path and extension.
     * Example: FilenameNoPathNoExtension("../input/example.msg") returns "example"
     */
    static std::string FilenameNoPathNoExtension(const std::string& filepath)
    {
        size_t sep_pos = filepath.find_last_of("/\\");
        std::string name = ((sep_pos != std::string::npos) ? (filepath.substr(sep_pos + 1)) : filepath);
        size_t ext_pos = name.rfind('.');
        if (ext_pos != std::string::npos)
            name = name.substr(0, ext_pos);
        return name;
    }

} // namespace sick_scansegment_xd
#endif // __SICK_SCANSEGMENT_XD_COMMON_H
