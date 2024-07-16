/**
*
* \brief pose2d_to_odom_converter: utility to convert pose2D to odometry messages,
*        used for testing rtabmap with multiscan
*
* Copyright (C) 2024 SICK AG, Waldkirch
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
*  Last modified: 10th june 2024
*
*      Authors:
*         Michael Lehning <michael.lehning@lehning.de>
*
*/
#include <sick_scan/sick_ros_wrapper.h>

#if defined __ROS_VERSION && __ROS_VERSION == 1
ros::Publisher nav_odom_publisher;
void process_pose_msg(const ros_geometry_msgs::Pose2D::ConstPtr& pose2d_msg)
{
  ROS_INFO_STREAM("pose2d_to_odom_converter: Pose2D message received, x = " << pose2d_msg->x << "m, y = " << pose2d_msg->y << " m, theta=" << (180.0 * pose2d_msg->theta /M_PI) << " deg");
  ros_nav_msgs::Odometry odom_msg;
  // The pose in odom_msg should be specified in the coordinate frame given by header.frame_id.
  // The twist in odom_msgshould be specified in the coordinate frame given by the child_frame_id.
  tf2::Quaternion theta_quaternion;
  theta_quaternion.setRPY(0, 0, pose2d_msg->theta);
  odom_msg.header.stamp = rosTimeNow();
  odom_msg.header.frame_id = "map";
  odom_msg.child_frame_id = "laser";
  odom_msg.pose.pose.position.x = pose2d_msg->x;
  odom_msg.pose.pose.position.y = pose2d_msg->y;
  odom_msg.pose.pose.position.z = 0;
  odom_msg.pose.pose.orientation.x = theta_quaternion.getX();
  odom_msg.pose.pose.orientation.y = theta_quaternion.getY();
  odom_msg.pose.pose.orientation.z = theta_quaternion.getZ();
  odom_msg.pose.pose.orientation.w = theta_quaternion.getW();
  odom_msg.pose.covariance = {0};
  odom_msg.twist.twist.linear.x = 0;
  odom_msg.twist.twist.linear.y = 0;
  odom_msg.twist.twist.linear.z = 0;
  odom_msg.twist.twist.angular.x = 0;
  odom_msg.twist.twist.angular.y = 0;
  odom_msg.twist.twist.angular.z = 0;
  odom_msg.twist.covariance = {0};
  nav_odom_publisher.publish(odom_msg);
  ROS_INFO_STREAM("pose2d_to_odom_converter: Odometry message published, x = " << odom_msg.pose.pose.position.x << "m, y = " << odom_msg.pose.pose.position.y << " m, yaw = " << (180.0 * pose2d_msg->theta /M_PI) << " deg, frame_id = " << odom_msg.header.frame_id << ", child_frame_id = " << odom_msg.child_frame_id);
}
#endif // __ROS_VERSION == 1


int main(int argc, char** argv)
{
#if defined __ROS_VERSION && __ROS_VERSION == 1
  ros::init(argc, argv, "pose2d_to_odom_converter");
  ros::NodeHandle nh("~");
  rosNodePtr node = &nh;

  ROS_INFO_STREAM("pose2d_to_odom_converter started");
  nav_odom_publisher = nh.advertise<ros_nav_msgs::Odometry>("/rtabmap/odom", 10);
  ros::Subscriber pose2d_subscriber = nh.subscribe<ros_geometry_msgs::Pose2D>("/pose2D", 10, process_pose_msg);

  rosSpin(node);

  ROS_INFO_STREAM("pose2d_to_odom_converter finished");
#endif // __ROS_VERSION == 1
  return 0;
}
