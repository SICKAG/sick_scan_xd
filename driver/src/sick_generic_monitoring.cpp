/*
 * Copyright (C) 2018, Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2018, SICK AG, Waldkirch
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
 *  Created on: 28th May 2018
 *
 *      Authors:
 *       Michael Lehning <michael.lehning@lehning.de>
 *
 */

#include <sick_scan/sick_generic_monitoring.h>

/** Constructor */
sick_scan_xd::SickScanMonitor::SickScanMonitor(int read_timeout_millisec)
{
  m_read_timeout_millisec = read_timeout_millisec;   // read timeout in milliseconds, messages are expected after max <read_timeout_millisec> delay (otherwise timeout and scanner re-init)
  m_lastRunState = scanner_init; // runState of last check: scanner_init, scanner_run or scanner_finalize
}

/** Destructor */
sick_scan_xd::SickScanMonitor::~SickScanMonitor()
{
}

/*
* @brief Monitors incoming scanner messages.
*        In case of read timeouts, checkState returns ExitError, otherwise ExitSuccess.
*/
sick_scan_xd::ExitCode sick_scan_xd::SickScanMonitor::checkState(NodeRunState runState, SickScanCommonTcp* scanner, sick_scan_xd::SickGenericParser *parser, sick_scan_xd::SickScanServices* services)
{
  if(m_lastRunState != runState) // wait for new messages after state change
  {
    m_lastRunState = runState;
    return sick_scan_xd::ExitSuccess; // OK
  }
  // if(scanner->numberOfDatagramInInputFifo() > 0) // still pending messages in receive queue
  // {
  //   return sick_scan_xd::ExitSuccess; // OK
  // }

  if(runState == scanner_run)
  {
    double read_timeout = 0.001 * scanner->getReadTimeOutInMs();                          // current read timeout in seconds
    uint64_t nanosec_last_tcp_msg = scanner->getNanosecTimestampLastTcpMessageReceived(); // timestamp in nanoseconds of the last received tcp message (or 0 if no message received)
    uint64_t nanosec_now = std::max<uint64_t>(nanosec_last_tcp_msg, rosNanosecTimestampNow());      // timestamp in nanoseconds now
    // ROS_INFO_STREAM("SickScanMonitor::checkState(): runState=scanner_run, last_tcp_msg=" << (1.0e-9 * nanosec_last_tcp_msg) << " sec, scanner->numberOfDatagramInInputFifo()=" << scanner->numberOfDatagramInInputFifo());

    if(nanosec_last_tcp_msg == 0)
    {
      return sick_scan_xd::ExitSuccess; // OK, wait for first message
    }

    // Set read timeout to configured value (if we're in state scanner_run)
    scanner->setReadTimeOutInMs(m_read_timeout_millisec);

    // Check timeout
    double dt = 1.0e-9 * (nanosec_now - nanosec_last_tcp_msg);
    if (dt > read_timeout)
    {
      // read timeout
      ROS_ERROR_STREAM("## ERROR SickScanMonitor::checkState(): read timeout after " << dt << " sec, timeout (" << read_timeout << " sec) exceeded." );
      return sick_scan_xd::ExitError; // timeout error
    }

  }
  return sick_scan_xd::ExitSuccess; // OK
}

/*
* @brief Monitors incoming scanner messages.
*        In case of read timeouts, checkStateReinitOnError closes the tcp connection, re-initializes the scanner.
*        Returns ExitSuccess (no timeout or successful re-init), or ExitError otherwise.
*/
sick_scan_xd::ExitCode sick_scan_xd::SickScanMonitor::checkStateReinitOnError(rosNodePtr nh, NodeRunState runState, SickScanCommonTcp* scanner, sick_scan_xd::SickGenericParser *parser, sick_scan_xd::SickScanServices* services)
{
  if (checkState(runState, scanner, parser, services) == sick_scan_xd::ExitSuccess)
  {
    return sick_scan_xd::ExitSuccess; // connection ok, messages have been received within configured timeout.
  }
  else // read timeout or tcp error
  {
    // TCP-reconnect and scanner-restart after timeout
    ROS_ERROR("## ERROR in sick_scan_xd: restarting scanner after read timeout");
    try
    {
      while (rosOk() && scanner->reinit(nh, m_read_timeout_millisec) != sick_scan_xd::ExitSuccess)
      {
        ROS_ERROR("## ERROR in sick_scan_xd: reinit scanner failed, retrying ..");
      }
      ROS_INFO("sick_scan_xd: scanner successfully reinitialized after timeout");
      return sick_scan_xd::ExitSuccess;
    }
    catch(const std::exception& e)
    {
      ROS_ERROR_STREAM("## ERROR in SickScanMonitor::checkStateReinitOnError: exception " << e.what());
    }
    catch(...)
    {
      ROS_ERROR_STREAM("## ERROR in SickScanMonitor::checkStateReinitOnError: unknown exception ");
    }
  }
  return sick_scan_xd::ExitError;
}

/** Constructor */
sick_scan_xd::PointCloudMonitor::PointCloudMonitor()
: m_monitoring_thread(0), m_monitoring_thread_running(false)
{
}

/** Destructor */
sick_scan_xd::PointCloudMonitor::~PointCloudMonitor()
{
  stopPointCloudMonitoring();
}

/*
* @brief Starts a thread to monitor point cloud messages.
*        The ros node will be killed, if no point cloud is published within a given amount of time.
*/
bool sick_scan_xd::PointCloudMonitor::startPointCloudMonitoring(rosNodePtr nh, int timeout_millisec, const std::string& ros_cloud_topic)
{
  m_nh = nh;
  m_timeout_millisec = timeout_millisec;
  m_ros_cloud_topic = ros_cloud_topic;
#if defined __ROS_VERSION && __ROS_VERSION > 0
  m_monitoring_thread_running = true;
  m_monitoring_thread = new std::thread(&sick_scan_xd::PointCloudMonitor::runMonitoringThreadCb, this);
  return true;
#else
  ROS_ERROR("## ERROR PointCloudMonitor supported on ROS only");
  return false;
#endif
}

/*
 * @brief Stops the thread to monitor point cloud messages.
 */
void sick_scan_xd::PointCloudMonitor::stopPointCloudMonitoring(void)
{
  m_monitoring_thread_running = false;
  if(m_monitoring_thread)
  {
    if (m_monitoring_thread->joinable())
       m_monitoring_thread->join();
    delete(m_monitoring_thread);
    m_monitoring_thread = 0;
  }
}

/** Callback for point cloud messages */
void sick_scan_xd::PointCloudMonitor::messageCbPointCloud(const ros_sensor_msgs::PointCloud2 & msg)
{ 
  // ROS_INFO_STREAM("PointCloudMonitor::messageCbPointCloud: new message after " << (1.0e-9 * rosNanosecTimestampNow() - 1.0e-9 * m_last_msg_timestamp_nanosec) << " seconds");
  m_last_msg_timestamp_nanosec = rosNanosecTimestampNow();
}

/** ROS2-callback for point cloud messages  */
void sick_scan_xd::PointCloudMonitor::messageCbPointCloudROS2(const std::shared_ptr<ros_sensor_msgs::PointCloud2> msg) 
{ 
  messageCbPointCloud(*msg); 
}

/*
 * @brief Thread callback, runs the point cloud monitoring.
 *        If no point cloud is published within the timeout (150 sec. by default),
 *        the process is killed (and the node is restarted by ros)
 */
void sick_scan_xd::PointCloudMonitor::runMonitoringThreadCb(void)
{
  // Get process id
#ifdef _MSC_VER  
  DWORD pid = GetCurrentProcessId();
#else  
  pid_t pid = getpid();
#endif  

  // Subscribe to point cloud topic
  m_last_msg_timestamp_nanosec = rosNanosecTimestampNow();
#if defined __ROS_VERSION && __ROS_VERSION == 1
  ros::Subscriber pointcloud_subscriber1, pointcloud_subscriber2;
  pointcloud_subscriber1 = m_nh->subscribe(m_ros_cloud_topic, 1, &sick_scan_xd::PointCloudMonitor::messageCbPointCloud, this);
  if(m_ros_cloud_topic[0] != '/')
    pointcloud_subscriber2 = m_nh->subscribe(std::string("/") + m_ros_cloud_topic, 1, &sick_scan_xd::PointCloudMonitor::messageCbPointCloud, this);
#elif defined __ROS_VERSION && __ROS_VERSION == 2
  rclcpp::Subscription<ros_sensor_msgs::PointCloud2>::SharedPtr pointcloud_subscriber1, pointcloud_subscriber2;
  rosQoS qos = rclcpp::SystemDefaultsQoS();
  overwriteByOptionalQOSconfig(m_nh, qos);
  QoSConverter qos_converter;
  ROS_INFO_STREAM("PointCloudMonitor: subscribing to topic " << m_ros_cloud_topic << ", qos=" << qos_converter.convert(qos));
  pointcloud_subscriber1 = m_nh->create_subscription<ros_sensor_msgs::PointCloud2>(m_ros_cloud_topic, qos, std::bind(&sick_scan_xd::PointCloudMonitor::messageCbPointCloudROS2, this, std::placeholders::_1));
  if(m_ros_cloud_topic[0] != '/')
    pointcloud_subscriber2 = m_nh->create_subscription<ros_sensor_msgs::PointCloud2>(std::string("/") + m_ros_cloud_topic,10,std::bind(&sick_scan_xd::PointCloudMonitor::messageCbPointCloudROS2, this, std::placeholders::_1));
#else
  ROS_ERROR("## ERROR PointCloudMonitor supported on ROS only");
  return;
#endif

  // Run monitoring
  while(rosOk() && m_monitoring_thread_running)
  {
    uint64_t timestamp_now_nanosec = rosNanosecTimestampNow();
    if(timestamp_now_nanosec/1000000 > m_last_msg_timestamp_nanosec/1000000 + m_timeout_millisec)
    {
      // Kill pid due to timeout
      ROS_ERROR_STREAM("## ERROR PointCloudMonitor: last point cloud message on topic \"" << m_ros_cloud_topic << "\" received " << (1.0e-9 * timestamp_now_nanosec - 1.0e-9 * m_last_msg_timestamp_nanosec) << " seconds ago, " << (1.0e-3 * m_timeout_millisec) << " seconds timeout exceeded.");
      std::stringstream kill_cmd;
#ifdef _MSC_VER  
      kill_cmd << "taskkill /F /PID " << pid;
#else  
      kill_cmd << "nohup sleep 1 ; kill -9 " << pid;
#endif  
      ROS_ERROR_STREAM("## ERROR PointCloudMonitor: killing process by command \"" << kill_cmd.str() << "\" for restart");
      int ret = system(kill_cmd.str().c_str());
    }
    else
    {
      // ROS_INFO_STREAM("PointCloudMonitor: last point cloud message on topic \"" << m_ros_cloud_topic << "\" received " << (1.0e-9 * timestamp_now_nanosec - 1.0e-9 * m_last_msg_timestamp_nanosec) << " seconds ago");
    }

    usleep(100000);
  }
  m_monitoring_thread_running = false;
}
