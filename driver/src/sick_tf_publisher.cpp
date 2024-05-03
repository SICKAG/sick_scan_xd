/*
 * Copyright (C) 2024, Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2024, SICK AG, Waldkirch
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
 *  Created on: 15th March 2024
 *
 *      Authors:
 *       Michael Lehning <michael.lehning@lehning.de>
 *
 */

#include <sick_scan/sick_scan_parse_util.h>
#include <sick_scan/sick_tf_publisher.h>
#include <sick_scan/sick_scan_common.h>
#include <sick_scan/sick_generic_parser.h>

namespace sick_scan_xd
{
  /*
  ** @brief On ROS-1 and ROS-2, SickTransformPublisher publishes TF messsages to map a given base frame
  **        (i.e. base coordinates system) to the lidar frame (i.e. lidar coordinates system) and vice versa.
  **
  **        The default base frame id is "map" (which is the default frame in rviz).
  **        The default 6D pose is (x,y,z,roll,pitch,yaw) = (0,0,0,0,0,0) defined by
  **        position (x,y,z) in meter and (roll,pitch,yaw) in radians.
  **        This 6D pose (x,y,z,roll,pitch,yaw) is the transform T[base,lidar] with
  **        parent "base" and child "lidar".
  **
  **        For lidars mounted on a carrier, the lidar pose T[base,lidar] can be configured in the launchfile:
  **        <param name="tf_base_frame_id" type="string" value="map" />              <!-- Frame id of base coordinates system, e.g. "map" (default frame in rviz) -->
  **        <param name="tf_base_lidar_xyz_rpy" type="string" value="0,0,0,0,0,0" /> <!-- T[base,lidar], 6D pose (x,y,z,roll,pitch,yaw) in meter resp. radians with parent "map" and child "cloud" -->
  **        <param name="tf_publish_rate" type="double" value="10" />                <!-- Rate to publish TF messages in hz, use 0 to deactivate TF messages -->
  **
  **        The lidar frame id given by parameter "frame_id" resp. "publish_frame_id".
  **
  **        Note that SickTransformPublisher configures the transform using (x,y,z,roll,pitch,yaw).
  **        In contrast, the ROS static_transform_publisher uses commandline arguments in order (x,y,z,yaw,pitch,roll).
  **
  */

  SickTransformPublisher::SickTransformPublisher(rosNodePtr _nh)
  {
#   if __ROS_VERSION > 0
    if (_nh)
    {
      // SickTransformPublisher configuration
      nh = _nh;
      std::string scanner_type;
      rosDeclareParam(nh, "scanner_type", scanner_type);
      rosGetParam(nh, "scanner_type", scanner_type);
      if (scanner_type == SICK_SCANNER_SCANSEGMENT_XD_NAME || scanner_type == SICK_SCANNER_PICOSCAN_NAME)
      {
        rosDeclareParam(nh, "publish_frame_id", tf_lidar_frame_id);
        rosGetParam(nh, "publish_frame_id", tf_lidar_frame_id);
      }
      else
      {
        rosDeclareParam(nh, "frame_id", tf_lidar_frame_id);
        rosGetParam(nh, "frame_id", tf_lidar_frame_id);
      }
      rosDeclareParam(nh, "tf_base_frame_id", tf_base_frame_id);
      rosGetParam(nh, "tf_base_frame_id", tf_base_frame_id);
      rosDeclareParam(nh, "tf_base_lidar_xyz_rpy", tf_base_lidar_xyz_rpy);
      rosGetParam(nh, "tf_base_lidar_xyz_rpy", tf_base_lidar_xyz_rpy);
      rosDeclareParam(nh, "tf_publish_rate", tf_publish_rate);
      rosGetParam(nh, "tf_publish_rate", tf_publish_rate);
      // Split string tf_base_lidar_xyz_rpy to 6D pose x,y,z,roll,pitch,yaw in [m] resp. [rad]
      tf_base_lidar_pose_vec = sick_scan_xd::parsePose(tf_base_lidar_xyz_rpy);
      if(tf_base_lidar_pose_vec.size() != 6)
      {
        ROS_ERROR_STREAM("## ERROR SickTransformPublisher(): Can't parse config string \"" << tf_base_lidar_xyz_rpy << "\", use 6D pose \"x,y,z,roll,pitch,yaw\" in [m] resp. [rad], using TF messages with default pose (0,0,0,0,0,0)");
        tf_base_lidar_pose_vec = std::vector<float>(6, 0);
      }
      ROS_INFO_STREAM("SickTransformPublisher: broadcasting TF messages parent_frame_id=\"" << tf_base_frame_id << "\", child_frame_id=\"" << tf_lidar_frame_id 
        << "\", (x,y,z,roll,pitch,yaw)=(" << std::fixed << std::setprecision(1) << tf_base_lidar_pose_vec[0] << "," << tf_base_lidar_pose_vec[1] << "," << tf_base_lidar_pose_vec[2] 
        << "," << (tf_base_lidar_pose_vec[3]*180.0/M_PI) << "," << (tf_base_lidar_pose_vec[4]*180.0/M_PI) << "," << (tf_base_lidar_pose_vec[5]*180.0/M_PI)
        << "), rate=" << tf_publish_rate);
    }
#   endif   
  }

  void SickTransformPublisher::run()
  {
#   if __ROS_VERSION > 0
    if(tf_publish_rate > 1.0e-6)
    {
      tf_publish_thread_running = true;
      tf_publish_thread = new std::thread(&sick_scan_xd::SickTransformPublisher::runTFpublishThreadCb, this);
    }
#   endif
  }

  void SickTransformPublisher::stop()
  {
#   if __ROS_VERSION > 0
    tf_publish_thread_running = false;
    if (tf_publish_thread && tf_publish_thread->joinable())
      tf_publish_thread->join();
    if (tf_publish_thread)
      delete tf_publish_thread;
    tf_publish_thread = 0;
#   endif
  }

# if __ROS_VERSION > 0
  void SickTransformPublisher::runTFpublishThreadCb()
  {
    assert(tf_publish_rate > 0);
    assert(tf_base_lidar_pose_vec.size() == 6); // 6D pose x,y,z,roll,pitch,yaw in [m] resp. [rad]
    int publish_sleep_millisec = int(1000.0 / tf_publish_rate);
    ros_geometry_msgs::TransformStamped tf_message;
    tf_message.header.frame_id = tf_base_frame_id;
    tf_message.child_frame_id = tf_lidar_frame_id;
    tf_message.transform.translation.x = tf_base_lidar_pose_vec[0];
    tf_message.transform.translation.y = tf_base_lidar_pose_vec[1];
    tf_message.transform.translation.z = tf_base_lidar_pose_vec[2];
    tf2::Quaternion q;
    q.setRPY(tf_base_lidar_pose_vec[3], tf_base_lidar_pose_vec[4], tf_base_lidar_pose_vec[5]);
    tf_message.transform.rotation.x = q.x();
    tf_message.transform.rotation.y = q.y();
    tf_message.transform.rotation.z = q.z();
    tf_message.transform.rotation.w = q.w();
#   if __ROS_VERSION == 1
    tf2_ros::TransformBroadcaster tf_broadcaster;
#   elif __ROS_VERSION == 2
    tf2_ros::TransformBroadcaster tf_broadcaster(nh);
#   endif
    while (rosOk() && tf_publish_thread_running)
    {
      tf_message.header.stamp = rosTimeNow();
      tf_broadcaster.sendTransform(tf_message);
      std::this_thread::sleep_for(std::chrono::milliseconds(publish_sleep_millisec));
    }
    tf_publish_thread_running = false;
  }
# endif


} // namespace sick_scan_xd
