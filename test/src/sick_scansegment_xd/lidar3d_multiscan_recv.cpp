/*
 * @brief lidar3d_multiscan_recv implements a ROS node to receive and publish data from the new sick 3D lidar multiScan136.
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
#include "sick_scan/sick_ros_wrapper.h"
#include "sick_scansegment_xd/scansegement_threads.h"

/*
 * main runs lidar3d_multiscan_recv:
 * - Initialize udp receiver, msgpack converter and ros publisher,
 * - Run threads to receive, convert, export and publish msgpack data,
 * - Optionally save to csv-file,
 * - Optionally read and convert msgpack files,
 * - Report cpu times and possible data lost.
 */
int main(int argc, char** argv)
{
    // Configuration
    sick_scansegment_xd::Config config;
    if (!config.Init(argc, argv))
        ROS_ERROR_STREAM("## ERROR lidar3d_multiscan_recv: Config::Init() failed, using default values.");
    ROS_INFO_STREAM("lidar3d_multiscan_recv started.");

    sick_scansegment_xd::MsgPackThreads msgpack_threads;
    if(!msgpack_threads.start(config))
    {
        ROS_ERROR_STREAM("## ERROR lidar3d_multiscan_recv: sick_scansegment_xd::MsgPackThreads::start() failed");
    }

    // Run event loop
#if defined __ROS_VERSION && __ROS_VERSION > 1
    rclcpp::spin(config.node);
    ROS_INFO_STREAM("lidar3d_multiscan_recv finishing, ros shutdown.");
#elif defined __ROS_VERSION && __ROS_VERSION > 0
    ros::spin();
    ROS_INFO_STREAM("lidar3d_multiscan_recv finishing, ros shutdown.");
#else // Run background task until ENTER key pressed
    while(true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        int c;
        if (KBHIT() && ((c = GETCH()) == 27 || c == 'q' || c == 'Q'))
        {
            ROS_INFO_STREAM("lidar3d_multiscan_recv: key " << c << " pressed, aborting...");
            break;
        }
    }
#endif
    rosShutdown();

    if(!msgpack_threads.stop())
    {
        ROS_ERROR_STREAM("## ERROR lidar3d_multiscan_recv: sick_scansegment_xd::MsgPackThreads::stop() failed");
    }
    ROS_INFO_STREAM("lidar3d_multiscan_recv finished.");
    return 0;
}
