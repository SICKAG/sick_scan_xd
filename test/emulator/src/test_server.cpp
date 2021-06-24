/*
 * @brief test_server.cpp implements a simple tcp server,
 * simulating a localization controller for unittests.
 *
 * Note: test_server.cpp does not implement the functions of localization controller,
 * it just implements a simple tcp server, accepting tcp connections from clients
 * and generating telegrams to test the ros driver for sim localization.
 *
 * Copyright (C) 2019 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2019 SICK AG, Waldkirch
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
 *  Copyright 2019 SICK AG
 *  Copyright 2019 Ing.-Buero Dr. Michael Lehning
 *
 */
#include "sick_scan/ros_wrapper.h"
#include <string>
#include <vector>

#include "sick_scan/test_server_thread.h"
#include "sick_scan/utils.h"

int main(int argc, char** argv)
{
  // Ros configuration and initialization
  ROS::init(argc, argv, "sick_scan_emulator");
  ROS::NodePtr nh = ROS::createNode("sick_scan_emulator");
  ROS_INFO_STREAM("sick_scan_emulator started.");
  
  // Create a server to simulate a localization controller, incl. a listener thread to accept tcp connections
  int tcp_port_results = 2201; // Default: The localization controller uses IP port number 2201 to send localization results
  int tcp_port_cola = 2111;    // For requests and to transmit settings to the localization controller: IP port number 2111 and 2112 to send telegrams and to request data, SOPAS CoLa-A or CoLa-B protocols
  ROS::param<int>(nh, "/sick_scan/test_server/result_telegrams_tcp_port", tcp_port_results, tcp_port_results);
  ROS::param<int>(nh, "/sick_scan/test_server/cola_telegrams_tcp_port", tcp_port_cola, tcp_port_cola);
  sick_scan::TestServerThread test_server_thread(nh, tcp_port_results, tcp_port_cola);
  
  // Subscribe to sim_loc_driver messages to monitor sim_loc_driver in error simulation mode
  std::string result_telegrams_topic = "/sick_scan/driver/result_telegrams";      // default topic to publish result port telegram messages (type SickLocResultPortTelegramMsg)
  ROS::param<std::string>(nh, "/sick_scan/driver/result_telegrams_topic", result_telegrams_topic, result_telegrams_topic);
#if defined __ROS_VERSION && __ROS_VERSION == 1
  sick_scan::SickLocResultPortTelegramMsgSubscriber result_telegram_subscriber 
    = ROS_CREATE_SUBSCRIBER(nh, sick_scan::SickLocResultPortTelegramMsg, result_telegrams_topic, &sick_scan::TestServerThread::messageCbResultPortTelegrams, &test_server_thread);
#elif defined __ROS_VERSION && __ROS_VERSION == 2
  sick_scan::SickLocResultPortTelegramMsgSubscriber result_telegram_subscriber
    = ROS_CREATE_SUBSCRIBER(nh, sick_scan::SickLocResultPortTelegramMsg, result_telegrams_topic, &sick_scan::TestServerThread::messageCbResultPortTelegramsROS2, &test_server_thread);
#endif
  
  // Start simulation of a localization controller
  test_server_thread.start();

  // Run ros event loop
  ROS::spin(nh);
  
  // Cleanup and exit
  std::cout << "sick_scan_emulator finished." << std::endl;
  ROS_INFO_STREAM("sick_scan_emulator finished.");
  test_server_thread.stop();
  std::cout << "sick_scan_emulator exits." << std::endl;
  ROS_INFO_STREAM("sick_scan_emulator exits.");
  ROS::deleteNode(nh);
  return 0;
}
