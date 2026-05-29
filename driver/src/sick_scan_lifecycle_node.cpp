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

#include "sick_scan/sick_scan_lifecycle_node.h"

#if defined __ROS_VERSION && __ROS_VERSION == 2

#if defined SCANSEGMENT_XD_SUPPORT && SCANSEGMENT_XD_SUPPORT > 0
#include "sick_scansegment_xd/scansegment_threads.h"
#endif

namespace sick_scan_xd
{

SickLifecycleNode::SickLifecycleNode(
  const std::string & node_name, int argc, char ** argv,
  const std::string & scannerName, const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, options), m_argc(argc), m_argv(argv),
  m_scannerName(scannerName), m_active(false), m_options(options)
{
  // Create internal node with rosout disabled to avoid publisher conflicts
  rclcpp::NodeOptions internal_options = m_options;
  internal_options.enable_rosout(false);
  m_internal_node = rclcpp::Node::make_shared(
    std::string(
      node_name) + "_internal", "", internal_options);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SickLifecycleNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Transitioning to [Inactive] state: Syncing Parameters.");

  // Sync all parameters from this Lifecycle node to the internal driver node
  auto all_params = this->get_parameters(this->list_parameters({}, 10).names);
  for (const auto & p : all_params) {
    if (!m_internal_node->has_parameter(p.get_name())) {
      m_internal_node->declare_parameter(p.get_name(), p.get_parameter_value());
    }
    m_internal_node->set_parameter(p);
  }
  RCLCPP_INFO(this->get_logger(), "Lifecycle: Configuration successful. Node is [INACTIVE].");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SickLifecycleNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Transitioning to [Active] state: Starting Hardware Threads.");
  int result = 0;
  if (startGenericLaser(m_argc, m_argv, (char *)m_scannerName.c_str(), m_internal_node, &result)) {
    m_active = true;
    RCLCPP_INFO(
      this->get_logger(), "Lifecycle: Activation successful. Node is [ACTIVE] and publishing.");
    return CallbackReturn::SUCCESS;
  }
  return CallbackReturn::FAILURE;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SickLifecycleNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Transitioning to [Inactive] state: Muting Data Stream.");
  m_active = false;
  stopScanner();
  RCLCPP_INFO(this->get_logger(), "Lifecycle: Deactivation complete. Node is [INACTIVE].");
  return CallbackReturn::SUCCESS;
}

// Cleanup: Return to Unconfigured state
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SickLifecycleNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up: Stopping hardware and clearing parameters.");

  this->stopScanner();         // Signal stop
  joinGenericLaser();          // Join threads (Blocking Pop fix)

  // RECONFIGURE LOGIC: Wipe the internal node to allow fresh parameter loading
  m_internal_node.reset();
  rclcpp::NodeOptions internal_options = m_options;
  internal_options.enable_rosout(false);
  m_internal_node = rclcpp::Node::make_shared(
    std::string(
      this->get_name()) + "_internal", "", internal_options);

  m_active = false;
  RCLCPP_INFO(this->get_logger(), "Lifecycle: Cleanup complete. Node is [UNCONFIGURED].");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SickLifecycleNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down: Emergency hardware stop.");
  stopScanner();
  joinGenericLaser();
  return CallbackReturn::SUCCESS;
}

// Error Handling: Move to Unconfigured for recovery
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SickLifecycleNode::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(this->get_logger(), "Hardware Error! Reverting to [Unconfigured] state.");

  this->stopScanner();
  joinGenericLaser();

  // Reset state so user can try 'configure' again immediately
  m_active = false;
  return CallbackReturn::SUCCESS;
}

void SickLifecycleNode::stopScanner()
{
  RCLCPP_INFO(this->get_logger(), "Stopping scanner and closing connections...");

  // Stop the msgpack threads first (for picoScan/multiScan scanners)
#if defined SCANSEGMENT_XD_SUPPORT && SCANSEGMENT_XD_SUPPORT > 0
  sick_scansegment_xd::stopMsgPackThreads();
#endif

  // Stop the scanner
  stopScannerAndExit();
  joinGenericLaser();

  RCLCPP_INFO(this->get_logger(), "Scanner stopped");
}

} // namespace sick_scan_xd

#endif // __ROS_VERSION == 2
