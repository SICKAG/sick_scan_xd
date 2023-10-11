#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
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
 *  Created on: 22nd november 2021
 *
 *      Authors:
 *       Michael Lehning <michael.lehning@lehning.de>
 *
 */

#ifndef SICK_GENERIC_MONITORING_H_
#define SICK_GENERIC_MONITORING_H_

#include <sick_scan/sick_ros_wrapper.h>
#include <sick_scan/sick_generic_laser.h>
#include <sick_scan/sick_generic_parser.h>
#include <sick_scan/sick_scan_common.h>
#include <sick_scan/sick_scan_services.h>

namespace sick_scan_xd
{

  /*
  * @brief class SickScanMonitor monitors incoming scanner messages.
  *        After a read timeout, the tcp connection is closed
  *        and the scanner is re-initialized.
  */
  class SickScanMonitor
  {
  public:

    /** Constructor */
    SickScanMonitor(int read_timeout_millisec = READ_TIMEOUT_MILLISEC_DEFAULT);

    /** Destructor */
    ~SickScanMonitor();

    /*
    * @brief Monitors incoming scanner messages.
    *        In case of read timeouts, checkState returns ExitError, otherwise ExitSuccess.
    */
    sick_scan_xd::ExitCode checkState(NodeRunState runState, SickScanCommonTcp* scanner, sick_scan_xd::SickGenericParser *parser, sick_scan_xd::SickScanServices* services);

    /*
    * @brief Monitors incoming scanner messages.
    *        In case of read timeouts, checkStateReinitOnError closes the tcp connection, re-initializes the scanner.
    *        Returns ExitSuccess (no timeout or successful re-init), or ExitError otherwise.
    */
    sick_scan_xd::ExitCode checkStateReinitOnError(rosNodePtr nh, NodeRunState runState, SickScanCommonTcp* scanner, sick_scan_xd::SickGenericParser *parser, sick_scan_xd::SickScanServices* services);

  protected:

    int m_read_timeout_millisec;   // read timeout in milliseconds, messages are expected after max <read_timeout_millisec> delay (otherwise timeout and scanner re-init)
    NodeRunState m_lastRunState; // runState of last check: scanner_init, scanner_run or scanner_finalize

  };

  /*
  * @brief class PointCloudMonitor monitors point cloud messages.
  *        The ros node will be killed, if no point cloud is published within a given amount of time.
  */
  class PointCloudMonitor
  {
  public:

    /** Constructor */
    PointCloudMonitor();

    /** Destructor */
    ~PointCloudMonitor();

    /*
    * @brief Starts a thread to monitor point cloud messages.
    *        The ros node will be killed, if no point cloud is published within a given amount of time.
    */
    bool startPointCloudMonitoring(rosNodePtr nh, int timeout_millisec = READ_TIMEOUT_MILLISEC_KILL_NODE, const std::string& ros_cloud_topic = "cloud");

    /*
    * @brief Stops the thread to monitor point cloud messages.
    */
    void stopPointCloudMonitoring(void);

  protected:

    /** Callback for point cloud messages */
    void messageCbPointCloud(const ros_sensor_msgs::PointCloud2 & msg);

    /** ROS2-callback for point cloud messages  */
    void messageCbPointCloudROS2(const std::shared_ptr<ros_sensor_msgs::PointCloud2> msg);

    /*
    * @brief Thread callback, runs the point cloud monitoring.
    *        If no point cloud is published within the timeout (150 sec. by default),
    *        the process is killed (and the node is restarted by ros)
    */
    void runMonitoringThreadCb(void);


    rosNodePtr m_nh;
    int m_timeout_millisec;
    std::string m_ros_cloud_topic;
    uint64_t m_last_msg_timestamp_nanosec; // timestamp of last received point cloud message in nanoseconds
    bool m_monitoring_thread_running;
    std::thread* m_monitoring_thread;

  };

} /* namespace sick_scan_xd */
#endif // SICK_GENERIC_MONITORING_H_
