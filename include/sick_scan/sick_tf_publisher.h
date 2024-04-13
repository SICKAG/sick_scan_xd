#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
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

#ifndef SICK_TF_PUBLISHER_H_
#define SICK_TF_PUBLISHER_H_

#include <sick_scan/sick_ros_wrapper.h>

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
  class SickTransformPublisher
  {
  public:

    SickTransformPublisher(rosNodePtr _nh = 0);

    void run();

    void stop();

  protected:
#if __ROS_VERSION > 0
    void runTFpublishThreadCb();
    rosNodePtr nh = 0;
    double tf_publish_rate = 10.0;
    std::string tf_lidar_frame_id = "cloud";
    std::string tf_base_frame_id = "map";
    std::string tf_base_lidar_xyz_rpy = "0,0,0,0,0,0";
    std::vector<float> tf_base_lidar_pose_vec;
    bool tf_publish_thread_running = false;
    std::thread* tf_publish_thread = 0;
#endif // __ROS_VERSION > 0
  }; // class SickTransformPublisher

} // namespace sick_scan_xd
#endif // SICK_TF_PUBLISHER_H_
