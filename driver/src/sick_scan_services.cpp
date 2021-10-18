/*
 * @brief Implementation of ROS services for sick_scan
 *
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
 *  Created on: 12.01.2021
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 * Based on the TiM communication example by SICK AG.
 *
 */

#include "sick_scan/sick_scan_services.h"

sick_scan::SickScanServices::SickScanServices(rosNodePtr nh, sick_scan::SickScanCommonTcp* common_tcp, bool cola_binary)
: m_common_tcp(common_tcp), m_cola_binary(cola_binary)
{
    if(nh)
    {
#if __ROS_VERSION == 2
#define serviceCbColaMsgROS sick_scan::SickScanServices::serviceCbColaMsgROS2
#define serviceCbECRChangeArrROS sick_scan::SickScanServices::serviceCbECRChangeArrROS2
#define serviceCbLIDoutputstateROS sick_scan::SickScanServices::serviceCbLIDoutputstateROS2
#else
#define serviceCbColaMsgROS sick_scan::SickScanServices::serviceCbColaMsg
#define serviceCbECRChangeArrROS sick_scan::SickScanServices::serviceCbECRChangeArr
#define serviceCbLIDoutputstateROS sick_scan::SickScanServices::serviceCbLIDoutputstate
#endif
        auto srv_server_ColaMsg = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::ColaMsgSrv, "ColaMsg", &serviceCbColaMsgROS, this);
        m_srv_server_ColaMsg = rosServiceServer<sick_scan_srv::ColaMsgSrv>(srv_server_ColaMsg);

        auto srv_server_ECRChangeArr = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::ECRChangeArrSrv, "ECRChangeArr", &serviceCbECRChangeArrROS, this);
        m_srv_server_ECRChangeArr = rosServiceServer<sick_scan_srv::ECRChangeArrSrv>(srv_server_ECRChangeArr);
        
        auto srv_server_LIDoutputstate = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::LIDoutputstateSrv, "LIDoutputstate", &serviceCbLIDoutputstateROS, this);
        m_srv_server_LIDoutputstate = rosServiceServer<sick_scan_srv::LIDoutputstateSrv>(srv_server_LIDoutputstate);

        // m_srv_server_ColaMsg = nh->advertiseService("ColaMsg",&sick_scan::SickScanServices::serviceCbColaMsg, this);
        // m_srv_server_ECRChangeArr = nh->advertiseService("ECRChangeArr",&sick_scan::SickScanServices::serviceCbECRChangeArr, this);
        // m_srv_server_LIDoutputstate = nh->advertiseService("LIDoutputstate",&sick_scan::SickScanServices::serviceCbLIDoutputstate, this);

#if __ROS_VERSION == 1
        ROS_INFO_STREAM("SickScanServices: service \"" << srv_server_ColaMsg.getService() << "\" created (\"" << m_srv_server_ColaMsg.getService() << "\")");
        ROS_INFO_STREAM("SickScanServices: service \"" << srv_server_ECRChangeArr.getService() << "\" created (\"" << m_srv_server_ColaMsg.getService() << "\")");
        ROS_INFO_STREAM("SickScanServices: service \"" << srv_server_LIDoutputstate.getService() << "\" created (\"" << m_srv_server_ColaMsg.getService() << "\")");
#elif __ROS_VERSION == 2
        ROS_INFO_STREAM("SickScanServices: service \"" << std::string(srv_server_ColaMsg->get_service_name()) << "\" created (\"" << std::string(m_srv_server_ColaMsg->get_service_name()) << "\")");
        ROS_INFO_STREAM("SickScanServices: service \"" << std::string(srv_server_ECRChangeArr->get_service_name()) << "\" created (\"" << std::string(m_srv_server_ECRChangeArr->get_service_name()) << "\")");
        ROS_INFO_STREAM("SickScanServices: service \"" << std::string(srv_server_LIDoutputstate->get_service_name()) << "\" created (\"" << std::string(m_srv_server_LIDoutputstate->get_service_name()) << "\")");
#endif
    }
}

sick_scan::SickScanServices::~SickScanServices()
{
}

/*!
 * Sends a sopas command and returns the lidar reply.
 * @param[in] sopasCmd sopas command to send, f.e. "sEN ECRChangeArr 1"
 * @param[out] sopasReplyBin response from lidar incl. start/stop byte
 * @param[out] sopasReplyString sopasReplyBin converted to string
 * @return true on success, false in case of errors.
 */
bool sick_scan::SickScanServices::sendSopasAndCheckAnswer(const std::string& sopasCmd, std::vector<unsigned char>& sopasReplyBin, std::string& sopasReplyString)
{
  if(m_common_tcp)
  {
    std::string sopasRequest = std::string("\x02") + sopasCmd + "\x03";
    int result = -1;
    if (m_cola_binary)
    {
      std::vector<unsigned char> reqBinary;
      m_common_tcp->convertAscii2BinaryCmd(sopasRequest.c_str(), &reqBinary);
      result = m_common_tcp->sendSopasAndCheckAnswer(reqBinary, &sopasReplyBin, -1);
    }
    else
    {
      result = m_common_tcp->sendSopasAndCheckAnswer(sopasRequest.c_str(), &sopasReplyBin, -1);
    }
    if (result != 0)
    {
      ROS_ERROR_STREAM("## ERROR SickScanServices::sendSopasAndCheckAnswer: error sending sopas command \"" << sopasCmd << "\"");
    }
    else
    {
      sopasReplyString = m_common_tcp->sopasReplyToString(sopasReplyBin);
      ROS_INFO_STREAM("SickScanServices: Request \"" << sopasCmd << "\" successfully sent, received reply \"" << sopasReplyString << "\"");
      return true;
    }
  }
  else
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendSopasAndCheckAnswer: m_common_tcp not initialized");
  }
  return false;
}

/*!
 * Callback for service ColaMsg (ColaMsg, send a cola message to lidar).
 * @param[in] service_request ros service request to lidar
 * @param[out] service_response service response from lidar
 * @return true on success, false in case of errors.
 */
bool sick_scan::SickScanServices::serviceCbColaMsg(sick_scan_srv::ColaMsgSrv::Request &service_request, sick_scan_srv::ColaMsgSrv::Response &service_response)
{
  std::string sopasCmd = service_request.request;
  std::vector<unsigned char> sopasReplyBin;
  std::string sopasReplyString;

  if(!sendSopasAndCheckAnswer(sopasCmd, sopasReplyBin, sopasReplyString))
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendSopasAndCheckAnswer failed on sending command\"" << sopasCmd << "\"");
    return false;
  }

  ROS_INFO_STREAM("SickScanServices: request: \"" << sopasCmd << "\"");
  ROS_INFO_STREAM("SickScanServices: response: \"" << sopasReplyString << "\"");

  service_response.response = sopasReplyString;
  return true;
}

/*!
 * Callback for service messages (ECRChangeArr, Request status change of monitoring fields on event).
 * Sends a cola telegram "sEN ECRChangeArr {0|1}" and receives the response from the lidar device.
 * @param[in] service_request ros service request to lidar
 * @param[out] service_response service response from lidar
 * @return true on success, false in case of errors.
 */
bool sick_scan::SickScanServices::serviceCbECRChangeArr(sick_scan_srv::ECRChangeArrSrv::Request &service_request, sick_scan_srv::ECRChangeArrSrv::Response &service_response)
{
  std::string sopasCmd = std::string("sEN ECRChangeArr ") + (service_request.active ? "1" : "0");
  std::vector<unsigned char> sopasReplyBin;
  std::string sopasReplyString;

  if(!sendSopasAndCheckAnswer(sopasCmd, sopasReplyBin, sopasReplyString))
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendSopasAndCheckAnswer failed on sending command\"" << sopasCmd << "\"");
    return false;
  }

  ROS_INFO_STREAM("SickScanServices: request: \"" << sopasCmd << "\"");
  ROS_INFO_STREAM("SickScanServices: response: \"" << sopasReplyString << "\"");
  
  return true;
}

/*!
 * Callback for service messages (LIDoutputstate, Request status change of monitoring fields on event).
 * Sends a cola telegram "sEN LIDoutputstate {0|1}" and receives the response from the lidar device.
 * @param[in] service_request ros service request to lidar
 * @param[out] service_response service response from lidar
 * @return true on success, false in case of errors.
 */
bool sick_scan::SickScanServices::serviceCbLIDoutputstate(sick_scan_srv::LIDoutputstateSrv::Request &service_request, sick_scan_srv::LIDoutputstateSrv::Response &service_response)
{
  std::string sopasCmd = std::string("sEN LIDoutputstate ") + (service_request.active ? "1" : "0");
  std::vector<unsigned char> sopasReplyBin;
  std::string sopasReplyString;

  if(!sendSopasAndCheckAnswer(sopasCmd, sopasReplyBin, sopasReplyString))
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendSopasAndCheckAnswer failed on sending command\"" << sopasCmd << "\"");
    return false;
  }

  ROS_INFO_STREAM("SickScanServices: request: \"" << sopasCmd << "\"");
  ROS_INFO_STREAM("SickScanServices: response: \"" << sopasReplyString << "\"");
  
  return true;
}
