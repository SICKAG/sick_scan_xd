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
#include "sick_scan/sick_generic_laser.h"

sick_scan::SickScanServices::SickScanServices(rosNodePtr nh, sick_scan::SickScanCommonTcp* common_tcp, bool cola_binary)
: m_common_tcp(common_tcp), m_cola_binary(cola_binary)
{
    if(nh)
    {
      m_client_authorization_pw = "F4724744";
      rosDeclareParam(nh, "client_authorization_pw", m_client_authorization_pw);
      rosGetParam(nh, "client_authorization_pw", m_client_authorization_pw);

#if __ROS_VERSION == 2
#define serviceCbColaMsgROS sick_scan::SickScanServices::serviceCbColaMsgROS2
#define serviceCbECRChangeArrROS sick_scan::SickScanServices::serviceCbECRChangeArrROS2
#define serviceCbLIDoutputstateROS sick_scan::SickScanServices::serviceCbLIDoutputstateROS2
#define serviceCbSCdevicestateROS sick_scan::SickScanServices::serviceCbSCdevicestateROS2
#define serviceCbSCrebootROS sick_scan::SickScanServices::serviceCbSCrebootROS2
#define serviceCbSCsoftresetROS sick_scan::SickScanServices::serviceCbSCsoftresetROS2
#define serviceCbSickScanExitROS sick_scan::SickScanServices::serviceCbSickScanExitROS2
#else
#define serviceCbColaMsgROS sick_scan::SickScanServices::serviceCbColaMsg
#define serviceCbECRChangeArrROS sick_scan::SickScanServices::serviceCbECRChangeArr
#define serviceCbLIDoutputstateROS sick_scan::SickScanServices::serviceCbLIDoutputstate
#define serviceCbSCdevicestateROS sick_scan::SickScanServices::serviceCbSCdevicestate
#define serviceCbSCrebootROS sick_scan::SickScanServices::serviceCbSCreboot
#define serviceCbSCsoftresetROS sick_scan::SickScanServices::serviceCbSCsoftreset
#define serviceCbSickScanExitROS sick_scan::SickScanServices::serviceCbSickScanExit
#endif
        auto srv_server_ColaMsg = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::ColaMsgSrv, "ColaMsg", &serviceCbColaMsgROS, this);
        m_srv_server_ColaMsg = rosServiceServer<sick_scan_srv::ColaMsgSrv>(srv_server_ColaMsg);

        auto srv_server_ECRChangeArr = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::ECRChangeArrSrv, "ECRChangeArr", &serviceCbECRChangeArrROS, this);
        m_srv_server_ECRChangeArr = rosServiceServer<sick_scan_srv::ECRChangeArrSrv>(srv_server_ECRChangeArr);
        
        auto srv_server_LIDoutputstate = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::LIDoutputstateSrv, "LIDoutputstate", &serviceCbLIDoutputstateROS, this);
        m_srv_server_LIDoutputstate = rosServiceServer<sick_scan_srv::LIDoutputstateSrv>(srv_server_LIDoutputstate);

        auto srv_server_SCdevicestate = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::SCdevicestateSrv, "SCdevicestate", &serviceCbSCdevicestateROS, this);
        m_srv_server_SCdevicestate = rosServiceServer<sick_scan_srv::SCdevicestateSrv>(srv_server_SCdevicestate);

        auto srv_server_SCreboot = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::SCrebootSrv, "SCreboot", &serviceCbSCrebootROS, this);
        m_srv_server_SCreboot = rosServiceServer<sick_scan_srv::SCrebootSrv>(srv_server_SCreboot);

        auto srv_server_SCsoftreset = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::SCsoftresetSrv, "SCsoftreset", &serviceCbSCsoftresetROS, this);
        m_srv_server_SCsoftreset = rosServiceServer<sick_scan_srv::SCsoftresetSrv>(srv_server_SCsoftreset);

        auto srv_server_SickScanExit = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::SickScanExitSrv, "SickScanExit", &serviceCbSickScanExitROS, this);
        m_srv_server_SickScanExit = rosServiceServer<sick_scan_srv::SickScanExitSrv>(srv_server_SickScanExit);

#if __ROS_VERSION == 1
        ROS_INFO_STREAM("SickScanServices: service \"" << srv_server_ColaMsg.getService() << "\" created (\"" << m_srv_server_ColaMsg.getService() << "\")");
        ROS_INFO_STREAM("SickScanServices: service \"" << srv_server_ECRChangeArr.getService() << "\" created (\"" << m_srv_server_ColaMsg.getService() << "\")");
        ROS_INFO_STREAM("SickScanServices: service \"" << srv_server_LIDoutputstate.getService() << "\" created (\"" << m_srv_server_ColaMsg.getService() << "\")");
        ROS_INFO_STREAM("SickScanServices: service \"" << srv_server_SCdevicestate.getService() << "\" created (\"" << m_srv_server_SCdevicestate.getService() << "\")");
        ROS_INFO_STREAM("SickScanServices: service \"" << srv_server_SCreboot.getService() << "\" created (\"" << m_srv_server_SCreboot.getService() << "\")");
        ROS_INFO_STREAM("SickScanServices: service \"" << srv_server_SCsoftreset.getService() << "\" created (\"" << m_srv_server_SCsoftreset.getService() << "\")");
        ROS_INFO_STREAM("SickScanServices: service \"" << srv_server_SickScanExit.getService() << "\" created (\"" << m_srv_server_SickScanExit.getService() << "\")");
#elif __ROS_VERSION == 2
        ROS_INFO_STREAM("SickScanServices: service \"" << std::string(srv_server_ColaMsg->get_service_name()) << "\" created (\"" << std::string(m_srv_server_ColaMsg->get_service_name()) << "\")");
        ROS_INFO_STREAM("SickScanServices: service \"" << std::string(srv_server_ECRChangeArr->get_service_name()) << "\" created (\"" << std::string(m_srv_server_ECRChangeArr->get_service_name()) << "\")");
        ROS_INFO_STREAM("SickScanServices: service \"" << std::string(srv_server_LIDoutputstate->get_service_name()) << "\" created (\"" << std::string(m_srv_server_LIDoutputstate->get_service_name()) << "\")");
        ROS_INFO_STREAM("SickScanServices: service \"" << std::string(srv_server_SCdevicestate->get_service_name()) << "\" created (\"" << std::string(m_srv_server_SCdevicestate->get_service_name()) << "\")");
        ROS_INFO_STREAM("SickScanServices: service \"" << std::string(srv_server_SCreboot->get_service_name()) << "\" created (\"" << std::string(m_srv_server_SCreboot->get_service_name()) << "\")");
        ROS_INFO_STREAM("SickScanServices: service \"" << std::string(srv_server_SCsoftreset->get_service_name()) << "\" created (\"" << std::string(m_srv_server_SCsoftreset->get_service_name()) << "\")");
        ROS_INFO_STREAM("SickScanServices: service \"" << std::string(srv_server_SickScanExit->get_service_name()) << "\" created (\"" << std::string(m_srv_server_SickScanExit->get_service_name()) << "\")");
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

  service_response.success = false;
  if(!sendSopasAndCheckAnswer(sopasCmd, sopasReplyBin, sopasReplyString))
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendSopasAndCheckAnswer failed on sending command\"" << sopasCmd << "\"");
    return false;
  }
  service_response.success = true;

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

  service_response.success = false;
  if(!sendSopasAndCheckAnswer(sopasCmd, sopasReplyBin, sopasReplyString))
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendSopasAndCheckAnswer failed on sending command\"" << sopasCmd << "\"");
    return false;
  }
  service_response.success = true;

  ROS_INFO_STREAM("SickScanServices: request: \"" << sopasCmd << "\"");
  ROS_INFO_STREAM("SickScanServices: response: \"" << sopasReplyString << "\"");
  
  return true;
}

/*!
 * Sends the SOPAS authorization command "sMN SetAccessMode 3 F4724744".
 */
bool sick_scan::SickScanServices::sendAuthorization()
{
  std::string sopasCmd = std::string("sMN SetAccessMode 3 ") + m_client_authorization_pw;
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
 * Sends the SOPAS command "sMN Run", which applies previous send settings
 */
bool sick_scan::SickScanServices::sendRun()
{
  std::string sopasCmd = std::string("sMN Run");
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
* Callbacks for service messages.
* @param[in] service_request ros service request to lidar
* @param[out] service_response service response from lidar
* @return true on success, false in case of errors.
*/
bool sick_scan::SickScanServices::serviceCbSCdevicestate(sick_scan_srv::SCdevicestateSrv::Request &service_request, sick_scan_srv::SCdevicestateSrv::Response &service_response)
{
  std::string sopasCmd = std::string("sRN SCdevicestate");
  std::vector<unsigned char> sopasReplyBin;
  std::string sopasReplyString;

  service_response.success = false;
  service_response.state = 2; // ERROR
  if(!sendSopasAndCheckAnswer(sopasCmd, sopasReplyBin, sopasReplyString))
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendSopasAndCheckAnswer failed on sending command\"" << sopasCmd << "\"");
    return false;
  }
  service_response.success = true;

  std::string response_str((char*)sopasReplyBin.data(), sopasReplyBin.size());
  std::size_t state_pos = response_str.find("SCdevicestate");
  if (state_pos != std::string::npos && state_pos + 14 < sopasReplyBin.size())
  {
    uint8_t state_byte = sopasReplyBin[state_pos + 14];
    if(state_byte >= '0')
      state_byte -= '0';
    service_response.state = state_byte;
  }
  ROS_INFO_STREAM("SickScanServices: request: \"" << sopasCmd << "\"");
  ROS_INFO_STREAM("SickScanServices: response: \"" << sopasReplyString << "\" = \"" << DataDumper::binDataToAsciiString(sopasReplyBin.data(), sopasReplyBin.size()) << "\"");
  
  return true;
}

bool sick_scan::SickScanServices::serviceCbSCreboot(sick_scan_srv::SCrebootSrv::Request &service_request, sick_scan_srv::SCrebootSrv::Response &service_response)
{
  std::string sopasCmd = std::string("sMN mSCreboot");
  std::vector<unsigned char> sopasReplyBin;
  std::string sopasReplyString;

  service_response.success = false;
  if(!sendAuthorization())
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices: sendAuthorization failed for command\"" << sopasCmd << "\"");
    return false;
  }

  if(!sendSopasAndCheckAnswer(sopasCmd, sopasReplyBin, sopasReplyString))
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendSopasAndCheckAnswer failed on sending command\"" << sopasCmd << "\"");
    return false;
  }

  ROS_INFO_STREAM("SickScanServices: request: \"" << sopasCmd << "\"");
  ROS_INFO_STREAM("SickScanServices: response: \"" << sopasReplyString << "\"");
  
  if(!sendRun())
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices: sendRun failed for command\"" << sopasCmd << "\"");
    return false;
  }
  service_response.success = true;

  return true;
}

bool sick_scan::SickScanServices::serviceCbSCsoftreset(sick_scan_srv::SCsoftresetSrv::Request &service_request, sick_scan_srv::SCsoftresetSrv::Response &service_response)
{
  std::string sopasCmd = std::string("sMN mSCsoftreset");
  std::vector<unsigned char> sopasReplyBin;
  std::string sopasReplyString;

  service_response.success = false;
  if(!sendAuthorization())
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices: sendAuthorization failed for command\"" << sopasCmd << "\"");
    return false;
  }

  if(!sendSopasAndCheckAnswer(sopasCmd, sopasReplyBin, sopasReplyString))
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendSopasAndCheckAnswer failed on sending command\"" << sopasCmd << "\"");
    return false;
  }

  ROS_INFO_STREAM("SickScanServices: request: \"" << sopasCmd << "\"");
  ROS_INFO_STREAM("SickScanServices: response: \"" << sopasReplyString << "\"");

  if(!sendRun())
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices: sendRun failed for command\"" << sopasCmd << "\"");
    return false;
  }
  service_response.success = true;

  return true;
}

bool sick_scan::SickScanServices::serviceCbSickScanExit(sick_scan_srv::SickScanExitSrv::Request &service_request, sick_scan_srv::SickScanExitSrv::Response &service_response)
{
  /*
  std::string sopasCmd = std::string("sMN mSCsoftreset");
  std::vector<unsigned char> sopasReplyBin;
  std::string sopasReplyString;

  service_response.success = false;
  if(!sendAuthorization())
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices: sendAuthorization failed for command\"" << sopasCmd << "\"");
    return false;
  }

  if(!sendSopasAndCheckAnswer(sopasCmd, sopasReplyBin, sopasReplyString))
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendSopasAndCheckAnswer failed on sending command\"" << sopasCmd << "\"");
    return false;
  }

  ROS_INFO_STREAM("SickScanServices: request: \"" << sopasCmd << "\"");
  ROS_INFO_STREAM("SickScanServices: response: \"" << sopasReplyString << "\"");

  if(!sendRun())
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices: sendRun failed for command\"" << sopasCmd << "\"");
    return false;
  }
  */
  service_response.success = stopScannerAndExit(false);
  return true;
}
