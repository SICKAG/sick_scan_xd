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
 *  Created on: February 15, 2026
 *
 *      Authors:
 *       Michael Lehning <michael.lehning@lehning.de>
 *       Boopesh Eswaran <boopesh.mc@gmail.com>
 *
 */

#ifndef SICK_SCAN_LIFECYCLE_NODE_H
#define SICK_SCAN_LIFECYCLE_NODE_H

#if defined __ROS_VERSION && __ROS_VERSION == 2

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "sick_scan/sick_generic_laser.h"
#include "sick_scan/sick_ros_wrapper.h"

namespace sick_scan_xd {

/**
 * @brief SickLifecycleNode manages the SICK LiDAR driver using the ROS 2 Lifecycle state machine.
 * This allows users to control the scanner's state (Configure, Activate, Deactivate, Cleanup)
 * via standard ROS 2 lifecycle services.
 */
  class SickLifecycleNode: public rclcpp_lifecycle::LifecycleNode {
public:
    /**
     * @brief Constructor for the SickLifecycleNode.
     */
    SickLifecycleNode(
      const std::string & node_name, int argc, char ** argv,
      const std::string & scannerName, const rclcpp::NodeOptions & options);

    /**
     * @brief Transition from Unconfigured to Inactive.
     * Synchronizes parameters and prepares the internal node for hardware communication.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & previous_state) override;

    /**
     * @brief Transition from Inactive to Active.
     * Launches the laser driver threads and begins data acquisition and publishing.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & previous_state) override;

    /**
     * @brief Transition from Active to Inactive.
     * Stops the measurement flow and halts publishing while keeping the connection alive.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    /**
     * @brief Transition from Inactive to Unconfigured.
     * Completely stops the scanner threads, closes connections, and resets the internal state.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    /**
     * @brief Transition to Finalized state.
     * Ensures an emergency shutdown and resource release from any state.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

    /**
     * @brief Error Handling Transition.
     * Called when a transition fails or an unhandled error occurs.
     * Moves the state to Unconfigured to allow for a recovery attempts.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_error(const rclcpp_lifecycle::State & previous_state) override;

private:
    /**
     * @brief Internal helper to stop scanner threads.
     */
    void stopScanner();

    int m_argc;
    char ** m_argv;
    std::string m_scannerName;
    bool m_active;
    rosNodePtr m_internal_node;
    rclcpp::NodeOptions m_options;
  };

} // namespace sick_scan_xd

#endif // __ROS_VERSION == 2
#endif // SICK_SCAN_LIFECYCLE_NODE_H
