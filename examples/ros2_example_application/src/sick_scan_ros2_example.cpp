/**
* \file
* \brief This project contains a tiny ROS-2 example. It shows how to use sick_scan_xd messages
* and services in a ROS-2 application.
*
* Copyright (C) 2022 Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2022, SICK AG, Waldkirch
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
*  Copyright 2022 SICK AG
*  Copyright 2022 Ing.-Buero Dr. Michael Lehning
*
*/
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <sick_scan_xd/msg/radar_scan.hpp>           // generated in sick_scan_xd by msg-generator
#include <sick_scan_xd/msg/encoder.hpp>              // generated in sick_scan_xd by msg-generator
#include <sick_scan_xd/msg/li_doutputstate_msg.hpp>  // generated in sick_scan_xd by msg-generator
#include <sick_scan_xd/msg/lf_erec_msg.hpp>          // generated in sick_scan_xd by msg-generator
#include <sick_scan_xd/srv/cola_msg_srv.hpp>         // generated in sick_scan_xd by rosidl-generator

#define RCLCPP_LOGGER         rclcpp::get_logger("sick_scan_ros2_example")
#define ROS_INFO_STREAM(...)  RCLCPP_INFO_STREAM(RCLCPP_LOGGER,__VA_ARGS__)
#define ROS_ERROR_STREAM(...) RCLCPP_ERROR_STREAM(RCLCPP_LOGGER,__VA_ARGS__)

/*
* @brief class SickScanMessageReceiver subscribes to pointcloud, lidoutputstate and lferec messages. Tiny example how to subscribe to sick_scan messages.
*/
class SickScanMessageReceiver
{
public:
    /** Constructor */
    SickScanMessageReceiver(rclcpp::Node::SharedPtr node = 0, const std::string& cloud_topic = "cloud", const std::string& lferec_topic = "lferec", const std::string& lidoutputstate_topic = "lidoutputstate")
    {
        if (node)
        {
            m_pointcloud_subscriber = node->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic, 10, std::bind(&SickScanMessageReceiver::messageCbPointCloudROS2, this, std::placeholders::_1));
            m_lferec_subscriber = node->create_subscription<sick_scan_xd::msg::LFErecMsg>(lferec_topic, 10, std::bind(&SickScanMessageReceiver::messageCbLFErecROS2, this, std::placeholders::_1));
            m_lidoutputstate_subscriber = node->create_subscription<sick_scan_xd::msg::LIDoutputstateMsg>(lidoutputstate_topic, 10, std::bind(&SickScanMessageReceiver::messageCbLIDoutputstateROS2, this, std::placeholders::_1));
        }
    }
protected:

    /** ROS2-callback for point cloud messages  */
    void messageCbPointCloudROS2(const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg)
    {
        ROS_INFO_STREAM("sick_scan_ros2_example: pointcloud message received, size " << msg-> width << " x " << msg->height);
    }

    /** ROS2-callback for sick_scan lferec messages  */
    void messageCbLFErecROS2(const std::shared_ptr<sick_scan_xd::msg::LFErecMsg> msg)
    {
        ROS_INFO_STREAM("sick_scan_ros2_example: lferec message received, " << msg->fields_number << " fields");
    }

    /** ROS2-callback for sick_scan lidoutputstate messages  */
    void messageCbLIDoutputstateROS2(const std::shared_ptr<sick_scan_xd::msg::LIDoutputstateMsg> msg)
    {
        ROS_INFO_STREAM("sick_scan_ros2_example: lidoutputstate message received, " << msg->output_state.size() << " output states, " << msg->output_count.size() << " output counter");
    }

    /** Subscriber of pointcloud and sick_scan messages */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_subscriber;
    rclcpp::Subscription<sick_scan_xd::msg::LFErecMsg>::SharedPtr m_lferec_subscriber;
    rclcpp::Subscription<sick_scan_xd::msg::LIDoutputstateMsg>::SharedPtr m_lidoutputstate_subscriber;
};

/** Sends a sick_scan ColaMsg service request */
bool sendSickScanServiceRequest(rclcpp::Node::SharedPtr node, rclcpp::Client<sick_scan_xd::srv::ColaMsgSrv>::SharedPtr sick_scan_srv_colamsg_client, const std::string& cola_msg)
{
    std::shared_ptr<sick_scan_xd::srv::ColaMsgSrv::Request> sick_scan_srv_request = std::make_shared<sick_scan_xd::srv::ColaMsgSrv::Request>();
    sick_scan_srv_request->request = cola_msg;
    ROS_INFO_STREAM("sick_scan_ros2_example: sick_scan service request: \"" << sick_scan_srv_request->request << "\"");
    std::shared_future<std::shared_ptr<sick_scan_xd::srv::ColaMsgSrv::Response>> sick_scan_srv_result = sick_scan_srv_colamsg_client->async_send_request(sick_scan_srv_request);
    if (rclcpp::spin_until_future_complete(node, sick_scan_srv_result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        ROS_INFO_STREAM("sick_scan_ros2_example: sick_scan service response: \"" << sick_scan_srv_result.get()->response << "\"");
        return true;
    }
    else
    {
        ROS_ERROR_STREAM("## ERROR sick_scan_ros2_example: sick_scan service request failed");
        return false;
    }
}

/*
* @brief Tiny ROS-2 example to show how to use sick_scan_xd messages and services in a ROS-2 application. Example for TiM7xx topics.
*/
int main(int argc, char** argv)
{
	// ROS node initialization
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("sick_scan_ros2_example", "", node_options);
    std::string sick_scan_msg_topic = "/sick_tim_7xx"; // Example for tim7xx: messages publiched on topic "/cloud", "/sick_tim_7xx/lferec" and "/sick_tim_7xx/lidoutputstate"
    for (int i = 0; i < argc; i++)
    {
        if (strncmp(argv[i], "topics:=", 8) == 0)
        {
            sick_scan_msg_topic = std::string("/") + std::string(argv[i] + 8);
        }
    }
    ROS_INFO_STREAM("sick_scan_ros2_example initialized");

    // Receive pointcloud, lidoutputstate and lferec messages
    SickScanMessageReceiver sickscan_message_receiver(node, "/cloud", sick_scan_msg_topic + "/lferec", sick_scan_msg_topic + "/lidoutputstate");

    // Create a sick_scan service client
    rclcpp::Client<sick_scan_xd::srv::ColaMsgSrv>::SharedPtr sick_scan_srv_colamsg_client = node->create_client<sick_scan_xd::srv::ColaMsgSrv>("ColaMsg");
    while (rclcpp::ok() && !sick_scan_srv_colamsg_client->wait_for_service(std::chrono::seconds(1)))
    {
       ROS_INFO_STREAM("sick_scan_ros2_example: Waiting for sick_scan service...");
    }
    if (rclcpp::ok())
    {
        if (sick_scan_srv_colamsg_client->wait_for_service(std::chrono::milliseconds(1)))
            ROS_INFO_STREAM("sick_scan_ros2_example: sick_scan service available");
        else
            ROS_ERROR_STREAM("## ERROR sick_scan_ros2_example: sick_scan service not available");
    }

    // Run event loop
    // rclcpp::spin(node);
    rclcpp::Time sick_scan_srv_request_timestamp = rclcpp::Clock().now();
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(1));
        if (rclcpp::Clock().now() > sick_scan_srv_request_timestamp + std::chrono::seconds(1))
        {
            // Send a sick_scan service request example each second
            sendSickScanServiceRequest(node, sick_scan_srv_colamsg_client, "sRN SCdevicestate");
            sick_scan_srv_request_timestamp = rclcpp::Clock().now();
        }
    }

    // Cleanup and exit
    ROS_INFO_STREAM("sick_scan_ros2_example finished");
    return 0;
}
