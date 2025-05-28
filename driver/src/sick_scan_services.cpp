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
#include <iostream>
#include <iomanip>

#include "sick_scan/sick_scan_services.h"
#include "sick_scan/sick_generic_laser.h"
#include "sick_scansegment_xd/udp_poster.h"
#include "sick_scan_common.h"

#define SCANSEGMENT_XD_SOPAS_ARGS_BIG_ENDIAN (true) // Arguments of SOPAS commands are big endian encoded

sick_scan_xd::SickScanServices::SickScanServices(rosNodePtr nh, sick_scan_xd::SickScanCommonTcp* common_tcp, ScannerBasicParam * lidar_param)
: m_common_tcp(common_tcp), m_cola_binary(true)
{
    bool srvSupportColaMsg = true, srvSupportECRChangeArr = true, srvSupportLIDoutputstate = true, srvSupportSCdevicestate = true;
    bool srvSupportSCreboot = true, srvSupportSCsoftreset = true, srvSupportSickScanExit = true;
    bool srvSupportGetContaminationData = false, srvSupportGetContaminationResult = false;
    if(lidar_param)
    {
      m_cola_binary = lidar_param->getUseBinaryProtocol();
      if(lidar_param->getScannerName().compare(SICK_SCANNER_NAV_350_NAME) == 0)
      {
        srvSupportECRChangeArr = false;
        srvSupportLIDoutputstate = false;
        srvSupportSCreboot = false;
        srvSupportSCsoftreset = false;
      }
      if(lidar_param->getScannerName().compare(SICK_SCANNER_MRS_1XXX_NAME) == 0
      || lidar_param->getScannerName().compare(SICK_SCANNER_LMS_1XXX_NAME) == 0
      || lidar_param->getScannerName().compare(SICK_SCANNER_LRS_4XXX_NAME) == 0
      || lidar_param->getScannerName().compare(SICK_SCANNER_SCANSEGMENT_XD_NAME) == 0)
      {
        srvSupportGetContaminationResult = true; // "sRN ContaminationResult" supported by MRS-1000, LMS-1000, LRS-4000, multiScan
      }
      if(lidar_param->getScannerName().compare(SICK_SCANNER_LRS_4XXX_NAME) == 0)
      {
        srvSupportGetContaminationData = true; // "sRN ContaminationData" supported by LRS-4000
      }
    }
    if(nh)
    {
        m_user_level = SickScanCommon::CLIENT_USER_LEVEL;
        m_user_level_password = SickScanCommon::CLIENT_USER_PASSWORD_HASH;

        if (lidar_param->getUseSafetyPasWD()) // TIM_7xxS - 1 layer Safety Lidars
            m_user_level_password = SickScanCommon::CLIENT_USER_PASSWORD_SAFETY_HASH; // use a specific password for safety lidars



        // For segment based lidars and LRS_4xxx use user_level 4 (service) and the default
        // service password hash

        if (SickScanCommon::useUserLevelService(lidar_param->getScannerName()))
        {
          m_user_level = SickScanCommon::SERVICE_USER_LEVEL; // user level "service"
          m_user_level_password = SickScanCommon::SERVICE_USER_PASSWORD_HASH; // Default password hash for service
        }


        rosDeclareParam(nh, "user_level", m_user_level);
        rosGetParam(nh, "user_level", m_user_level);
        rosDeclareParam(nh, "user_level_password", m_user_level_password);
        rosGetParam(nh, "user_level_password", m_user_level_password);

#if __ROS_VERSION == 2
#define serviceCbColaMsgROS sick_scan_xd::SickScanServices::serviceCbColaMsgROS2
#define serviceCbECRChangeArrROS sick_scan_xd::SickScanServices::serviceCbECRChangeArrROS2
#define serviceCbLIDoutputstateROS sick_scan_xd::SickScanServices::serviceCbLIDoutputstateROS2
#define serviceCbSCdevicestateROS sick_scan_xd::SickScanServices::serviceCbSCdevicestateROS2
#define serviceCbSCrebootROS sick_scan_xd::SickScanServices::serviceCbSCrebootROS2
#define serviceCbSCsoftresetROS sick_scan_xd::SickScanServices::serviceCbSCsoftresetROS2
#define serviceCbSickScanExitROS sick_scan_xd::SickScanServices::serviceCbSickScanExitROS2
#define serviceCbGetContaminationDataROS sick_scan_xd::SickScanServices::serviceCbGetContaminationDataROS2
#define serviceCbGetContaminationResultROS sick_scan_xd::SickScanServices::serviceCbGetContaminationResultROS2
#define serviceCbFieldSetReadROS sick_scan_xd::SickScanServices::serviceCbFieldSetReadROS2
#define serviceCbFieldSetWriteROS sick_scan_xd::SickScanServices::serviceCbFieldSetWriteROS2
#else
#define serviceCbColaMsgROS sick_scan_xd::SickScanServices::serviceCbColaMsg
#define serviceCbECRChangeArrROS sick_scan_xd::SickScanServices::serviceCbECRChangeArr
#define serviceCbLIDoutputstateROS sick_scan_xd::SickScanServices::serviceCbLIDoutputstate
#define serviceCbSCdevicestateROS sick_scan_xd::SickScanServices::serviceCbSCdevicestate
#define serviceCbSCrebootROS sick_scan_xd::SickScanServices::serviceCbSCreboot
#define serviceCbSCsoftresetROS sick_scan_xd::SickScanServices::serviceCbSCsoftreset
#define serviceCbSickScanExitROS sick_scan_xd::SickScanServices::serviceCbSickScanExit
#define serviceCbGetContaminationDataROS sick_scan_xd::SickScanServices::serviceCbGetContaminationData
#define serviceCbGetContaminationResultROS sick_scan_xd::SickScanServices::serviceCbGetContaminationResult
#define serviceCbFieldSetReadROS sick_scan_xd::SickScanServices::serviceCbFieldSetRead
#define serviceCbFieldSetWriteROS sick_scan_xd::SickScanServices::serviceCbFieldSetWrite
#endif
#if __ROS_VERSION == 1
#define printServiceCreated(a,b) ROS_INFO_STREAM("SickScanServices: service \"" << a.getService() << "\" created (\"" << b.getService() << "\")");
#elif __ROS_VERSION == 2
#define printServiceCreated(a,b) ROS_INFO_STREAM("SickScanServices: service \"" << a->get_service_name() << "\" created (\"" << b->get_service_name() << "\")");
#else
#define printServiceCreated(a,b)
#endif
        if(srvSupportColaMsg)
        {
          auto srv_server_ColaMsg = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::ColaMsgSrv, "ColaMsg", &serviceCbColaMsgROS, this);
          m_srv_server_ColaMsg = rosServiceServer<sick_scan_srv::ColaMsgSrv>(srv_server_ColaMsg);
          printServiceCreated(srv_server_ColaMsg, m_srv_server_ColaMsg);
        }
        if(srvSupportECRChangeArr)
        {
          auto srv_server_ECRChangeArr = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::ECRChangeArrSrv, "ECRChangeArr", &serviceCbECRChangeArrROS, this);
          m_srv_server_ECRChangeArr = rosServiceServer<sick_scan_srv::ECRChangeArrSrv>(srv_server_ECRChangeArr);
          printServiceCreated(srv_server_ECRChangeArr, m_srv_server_ECRChangeArr);
        }
        if(srvSupportGetContaminationResult)
        {
          auto srv_server_GetContaminationResult = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::GetContaminationResultSrv, "GetContaminationResult", &serviceCbGetContaminationResultROS, this);
          m_srv_server_GetContaminationResult = rosServiceServer<sick_scan_srv::GetContaminationResultSrv>(srv_server_GetContaminationResult);
          printServiceCreated(srv_server_GetContaminationResult, m_srv_server_GetContaminationResult);
        }
        if(srvSupportGetContaminationData)
        {
          auto srv_server_GetContaminationData = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::GetContaminationDataSrv, "GetContaminationData", &serviceCbGetContaminationDataROS, this);
          m_srv_server_GetContaminationData = rosServiceServer<sick_scan_srv::GetContaminationDataSrv>(srv_server_GetContaminationData);
          printServiceCreated(srv_server_GetContaminationData, m_srv_server_GetContaminationData);
        }
        if(srvSupportLIDoutputstate)
        {
          auto srv_server_LIDoutputstate = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::LIDoutputstateSrv, "LIDoutputstate", &serviceCbLIDoutputstateROS, this);
          m_srv_server_LIDoutputstate = rosServiceServer<sick_scan_srv::LIDoutputstateSrv>(srv_server_LIDoutputstate);
          printServiceCreated(srv_server_LIDoutputstate, m_srv_server_LIDoutputstate);
        }
        if(srvSupportSCdevicestate)
        {
          auto srv_server_SCdevicestate = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::SCdevicestateSrv, "SCdevicestate", &serviceCbSCdevicestateROS, this);
          m_srv_server_SCdevicestate = rosServiceServer<sick_scan_srv::SCdevicestateSrv>(srv_server_SCdevicestate);
          printServiceCreated(srv_server_SCdevicestate, m_srv_server_SCdevicestate);
        }
        if(srvSupportSCreboot)
        {
          auto srv_server_SCreboot = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::SCrebootSrv, "SCreboot", &serviceCbSCrebootROS, this);
          m_srv_server_SCreboot = rosServiceServer<sick_scan_srv::SCrebootSrv>(srv_server_SCreboot);
          printServiceCreated(srv_server_SCreboot, m_srv_server_SCreboot);
        }
        if(srvSupportSCsoftreset)
        {
          auto srv_server_SCsoftreset = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::SCsoftresetSrv, "SCsoftreset", &serviceCbSCsoftresetROS, this);
          m_srv_server_SCsoftreset = rosServiceServer<sick_scan_srv::SCsoftresetSrv>(srv_server_SCsoftreset);
          printServiceCreated(srv_server_SCsoftreset, m_srv_server_SCsoftreset);
        }
        if(srvSupportSickScanExit)
        {
          auto srv_server_SickScanExit = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::SickScanExitSrv, "SickScanExit", &serviceCbSickScanExitROS, this);
          m_srv_server_SickScanExit = rosServiceServer<sick_scan_srv::SickScanExitSrv>(srv_server_SickScanExit);
          printServiceCreated(srv_server_SickScanExit, m_srv_server_SickScanExit);
        }
#if __ROS_VERSION > 0
        if(lidar_param->getUseEvalFields() == USE_EVAL_FIELD_TIM7XX_LOGIC)
        {
          auto srv_server_FieldSetRead = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::FieldSetReadSrv, "FieldSetRead", &serviceCbFieldSetReadROS, this);
          m_srv_server_FieldSetRead = rosServiceServer<sick_scan_srv::FieldSetReadSrv>(srv_server_FieldSetRead);
          printServiceCreated(srv_server_FieldSetRead, m_srv_server_FieldSetRead);
        }
        if(lidar_param->getUseEvalFields() == USE_EVAL_FIELD_TIM7XX_LOGIC)
        {
          auto srv_server_FieldSetWrite = ROS_CREATE_SRV_SERVER(nh, sick_scan_srv::FieldSetWriteSrv, "FieldSetWrite", &serviceCbFieldSetWriteROS, this);
          m_srv_server_FieldSetWrite = rosServiceServer<sick_scan_srv::FieldSetWriteSrv>(srv_server_FieldSetWrite);
          printServiceCreated(srv_server_FieldSetWrite, m_srv_server_FieldSetWrite);
        }
#endif
    }
}

sick_scan_xd::SickScanServices::~SickScanServices()
{
}

/*!
 * Sends a sopas command and returns the lidar reply.
 * @param[in] sopasCmd sopas command to send, f.e. "sEN ECRChangeArr 1"
 * @param[out] sopasReplyBin response from lidar incl. start/stop byte
 * @param[out] sopasReplyString sopasReplyBin converted to string
 * @return true on success, false in case of errors.
 */
bool sick_scan_xd::SickScanServices::sendSopasAndCheckAnswer(const std::string& sopasCmd, std::vector<unsigned char>& sopasReplyBin, std::string& sopasReplyString)
{
  if(m_common_tcp)
  {
    ROS_INFO_STREAM("SickScanServices: Sending request \"" << sopasCmd << "\"");
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
bool sick_scan_xd::SickScanServices::serviceCbColaMsg(sick_scan_srv::ColaMsgSrv::Request &service_request, sick_scan_srv::ColaMsgSrv::Response &service_response)
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
bool sick_scan_xd::SickScanServices::serviceCbECRChangeArr(sick_scan_srv::ECRChangeArrSrv::Request &service_request, sick_scan_srv::ECRChangeArrSrv::Response &service_response)
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
  * Callback for service messages (GetContaminationData, Read contamination indication data, LRS-4xxx only).
  * Sends a cola telegram "sRN ContaminationData" and receives the response from the lidar device.
  * @param[in] service_request ros service request to lidar
  * @param[out] service_response service response from lidar
  * @return true on success, false in case of errors.
  */
bool sick_scan_xd::SickScanServices::serviceCbGetContaminationData(sick_scan_srv::GetContaminationDataSrv::Request &service_request, sick_scan_srv::GetContaminationDataSrv::Response &service_response)
{
  std::string sopasCmd = std::string("sRN ContaminationData");
  std::vector<unsigned char> sopasReplyBin;
  std::string sopasReplyString;

  service_response.success = false;
  service_response.data.clear();
  if(!sendSopasAndCheckAnswer(sopasCmd, sopasReplyBin, sopasReplyString))
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendSopasAndCheckAnswer failed on sending command\"" << sopasCmd << "\"");
    return false;
  }
  service_response.success = true;

  std::string response_str((char*)sopasReplyBin.data(), sopasReplyBin.size());
  std::size_t state_pos = response_str.find("ContaminationData");
  int result_idx = 18;
  int num_channels = 12; // One entry (byte) for each of 12 channels. Order of 12 channels: (1/2/3/4/5/6/7/8/9/10/11/12). Status 0 = CM NONE, 1 = CM WARN, 2 = CM ERROR.
  if (state_pos != std::string::npos && state_pos + result_idx < sopasReplyBin.size())
  {
    for(int channel_cnt = 0; channel_cnt < num_channels && state_pos + result_idx < sopasReplyBin.size(); channel_cnt++)
    {
      uint8_t result_byte = sopasReplyBin[state_pos + result_idx];
      result_byte = ((result_byte >= '0') ? (result_byte - '0') : (result_byte)); // convert to bin in case of ascii
      service_response.data.push_back(result_byte);
      result_idx++;
      if (result_idx < sopasReplyBin.size() && sopasReplyBin[state_pos + result_idx] == ' ') // jump over ascii separator
        result_idx++;
    }
  }
  ROS_INFO_STREAM("SickScanServices: request: \"" << sopasCmd << "\"");
  ROS_INFO_STREAM("SickScanServices: response: \"" << sopasReplyString << "\" = \"" << DataDumper::binDataToAsciiString(sopasReplyBin.data(), sopasReplyBin.size()) << "\""
    << " (response.success=" << (int)(service_response.success) << ", response.data=" << DataDumper::binDataToAsciiString(service_response.data.data(), service_response.data.size()) << ")");

  return true;
}


/*!
* Callbacks for service messages.
* @param[in] service_request ros service request to lidar
* @param[out] service_response service response from lidar
* @return true on success, false in case of errors.
*/
bool sick_scan_xd::SickScanServices::serviceCbGetContaminationResult(sick_scan_srv::GetContaminationResultSrv::Request &service_request, sick_scan_srv::GetContaminationResultSrv::Response &service_response)
{
  std::string sopasCmd = std::string("sRN ContaminationResult");
  std::vector<unsigned char> sopasReplyBin;
  std::string sopasReplyString;

  service_response.success = false;
  service_response.warning = 0;
  service_response.error = 0;
  if(!sendSopasAndCheckAnswer(sopasCmd, sopasReplyBin, sopasReplyString))
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendSopasAndCheckAnswer failed on sending command\"" << sopasCmd << "\"");
    return false;
  }
  service_response.success = true;

  std::string response_str((char*)sopasReplyBin.data(), sopasReplyBin.size());
  std::size_t state_pos = response_str.find("ContaminationResult");
  int result_idx = 20;
  if (state_pos != std::string::npos && state_pos + result_idx < sopasReplyBin.size())
  {
    uint8_t result_byte = sopasReplyBin[state_pos + result_idx];
    result_byte = ((result_byte >= '0') ? (result_byte - '0') : (result_byte)); // convert to bin in case of ascii
    service_response.warning = result_byte;
    result_idx++;
    if (result_idx < sopasReplyBin.size() && sopasReplyBin[state_pos + result_idx] == ' ') // jump over ascii separator
      result_idx++;
    if (result_idx < sopasReplyBin.size())
    {
      result_byte = sopasReplyBin[state_pos + result_idx];
      result_byte = ((result_byte >= '0') ? (result_byte - '0') : (result_byte)); // convert to bin in case of ascii
      service_response.error = result_byte;
    }
  }
  ROS_INFO_STREAM("SickScanServices: request: \"" << sopasCmd << "\"");
  ROS_INFO_STREAM("SickScanServices: response: \"" << sopasReplyString << "\" = \"" << DataDumper::binDataToAsciiString(sopasReplyBin.data(), sopasReplyBin.size()) << "\""
    << " (response.success=" << (int)(service_response.success) << ", response.warning=" << (int)(service_response.warning) << ", response.error=" << (int)(service_response.error) << ")");

  return true;
}


/*!
 * Callback for service messages (LIDoutputstate, Request status change of monitoring fields on event).
 * Sends a cola telegram "sEN LIDoutputstate {0|1}" and receives the response from the lidar device.
 * @param[in] service_request ros service request to lidar
 * @param[out] service_response service response from lidar
 * @return true on success, false in case of errors.
 */
bool sick_scan_xd::SickScanServices::serviceCbLIDoutputstate(sick_scan_srv::LIDoutputstateSrv::Request &service_request, sick_scan_srv::LIDoutputstateSrv::Response &service_response)
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
bool sick_scan_xd::SickScanServices::sendAuthorization()
{
  // create a string like "sMN SetAccessMode 3 F4724744"
  std::string sopasCmd = "sMN SetAccessMode " + std::to_string(m_user_level) + " " + m_user_level_password;
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
bool sick_scan_xd::SickScanServices::sendRun()
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
 * Sends a multiScan136 command
 */
bool sick_scan_xd::SickScanServices::sendSopasCmdCheckResponse(const std::string& sopas_request, const std::string& expected_response)
{
  std::vector<unsigned char> sopasReplyBin;
  std::string sopasReplyString;
  if(!sendSopasAndCheckAnswer(sopas_request, sopasReplyBin, sopasReplyString))
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendSopasCmdCheckResponse() failed on sending command\"" << sopas_request << "\"");
    return false;
  }
  ROS_INFO_STREAM("SickScanServices::sendSopasCmdCheckResponse(): request: \"" << sopas_request << "\", response: \"" << sopasReplyString << "\"");
  if(sopasReplyString.find(expected_response) == std::string::npos)
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendSopasCmdCheckResponse(): request: \"" << sopas_request << "\", unexpected response: \"" << sopasReplyString << "\", \"" << expected_response << "\" not found");
    return false;
  }
  return true;
}

#if defined SCANSEGMENT_XD_SUPPORT && SCANSEGMENT_XD_SUPPORT > 0
static void printPicoScanImuDisabledWarning()
{
    ROS_WARN_STREAM("############################");
    ROS_WARN_STREAM("##                        ##");
    ROS_WARN_STREAM("##  IMU will be disabled  ##");
    ROS_WARN_STREAM("##                        ##");
    ROS_WARN_STREAM("############################");
    ROS_WARN_STREAM("## If you are using a picoScan150 w/o addons, disable the IMU by setting option \"imu_enable\" to \"False\" in your launchfile, or use sick_picoscan_single_echo.launch to avoid this error");
    ROS_WARN_STREAM("## If you are using a picoScan with IMU, check IMU settings with SOPAS Air");
}

/*!
* Sends the multiScan start commands "sWN ScanDataFormat", "sWN ScanDataPreformatting", "sWN ScanDataEthSettings", "sWN ScanDataEnable 1", "sMN LMCstartmeas", "sMN Run"
* @param[in] hostname IP address of multiScan136, default 192.168.0.1
* @param[in] port IP port of multiScan136, default 2115
* @param[in] scanner_type type of scanner, currently supported are multiScan136 and picoScan150
* @param[in] scandataformat ScanDataFormat: 1 for msgpack or 2 for compact scandata, default: 2 
* @param[in+out] imu_enable: Imu data transfer enabled
* @param[in] imu_udp_port: UDP port of imu data (if imu_enable is true)
* @param[in] check_udp_receiver_ip: check udp_receiver_ip by sending and receiving a udp test message
* @param[in] check_udp_receiver_port: udp port to check udp_receiver_ip
*/
bool sick_scan_xd::SickScanServices::sendMultiScanStartCmd(const std::string& hostname, int port, const std::string& scanner_type, int scandataformat, bool& imu_enable, int imu_udp_port, int performanceprofilenumber, bool check_udp_receiver_ip, int check_udp_receiver_port)
{
  // Check udp receiver ip address: hostname is expected to be a IPv4 address
  std::stringstream ip_stream(hostname);
  std::string ip_token;
  std::vector<std::string> ip_tokens;
  while (getline(ip_stream, ip_token, '.'))
  {
    ip_tokens.push_back(ip_token);
  }
  if (ip_tokens.size() != 4)
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendMultiScanStartCmd() failed: can't split ip address \"" << hostname << "\" into 4 tokens, check ip address.");
    ROS_ERROR_STREAM("## ERROR parsing ip address, check configuration of parameter udp_receiver_ip=\"" << hostname << "\" (launch file or commandline option).");
    ROS_ERROR_STREAM("## Parameter \"udp_receiver_ip\" should be the IPv4 address like 192.168.0.x of the system running sick_scan_xd.");
    return false;
  }
  // Check udp receiver ip address by sending and receiving a UDP test message
  std::string check_udp_receiver_msg = "sick_scan_xd udp test message verifying udp_receiver_ip";
  if (check_udp_receiver_ip && !check_udp_receiver_msg.empty())
  {
    sick_scansegment_xd::UdpPoster udp_loopback_poster(hostname, check_udp_receiver_port);
    std::string check_udp_receiver_response;
    if (!udp_loopback_poster.Post(check_udp_receiver_msg, check_udp_receiver_response) || check_udp_receiver_msg != check_udp_receiver_response)
    {
      ROS_ERROR_STREAM("## ERROR SickScanServices::sendMultiScanStartCmd() failed to send and receive a UDP test messsage to " << hostname << ":" << check_udp_receiver_port);
      ROS_ERROR_STREAM("## Check configuration of parameter udp_receiver_ip=\"" << hostname << "\" (launch file or commandline option).");
      ROS_ERROR_STREAM("## Parameter \"udp_receiver_ip\" should be the IPv4 address like 192.168.0.x of the system running sick_scan_xd.");
      return false;
    }
    else
    {
      ROS_DEBUG_STREAM("SickScanServices::sendMultiScanStartCmd(): UDP test message \"" << check_udp_receiver_response << "\" successfully received.");
      ROS_INFO_STREAM("SickScanServices::sendMultiScanStartCmd(): UDP test message successfully send to " << hostname << ":" << check_udp_receiver_port << ", parameter udp_receiver_ip=\"" << hostname << "\" is valid.");
    }
  }
  // Send sopas start sequence (ScanDataFormat, ScanDataEthSettings, ImuDataEthSettings, ScanDataEnable, ImuDataEnable)
  std::stringstream eth_settings_cmd, imu_eth_settings_cmd, scandataformat_cmd, performanceprofilenumber_cmd;
  scandataformat_cmd << "sWN ScanDataFormat " << scandataformat;
  if (performanceprofilenumber >= 0)
  {
    performanceprofilenumber_cmd << "sWN PerformanceProfileNumber " << std::uppercase << std::hex << performanceprofilenumber;
  }
  eth_settings_cmd << "sWN ScanDataEthSettings 1";
  imu_eth_settings_cmd << "sWN ImuDataEthSettings 1";
  for (int i = 0; i < ip_tokens.size(); i++)
  {
    eth_settings_cmd << " +";
    eth_settings_cmd << ip_tokens[i];
    imu_eth_settings_cmd << " +";
    imu_eth_settings_cmd << ip_tokens[i];
  }
  eth_settings_cmd << " +";
  eth_settings_cmd << port;
  imu_eth_settings_cmd << " +";
  imu_eth_settings_cmd << imu_udp_port;
  if (!sendSopasCmdCheckResponse(eth_settings_cmd.str(), "sWA ScanDataEthSettings")) // configure destination scan data output destination , f.e. "sWN ScanDataEthSettings 1 +192 +168 +0 +52 +2115" (ip 192.168.0.52 port 2115)
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendMultiScanStartCmd(): sendSopasCmdCheckResponse(\"sWN ScanDataEthSettings 1\") failed.");
    return false;
  }
  if (scandataformat != 1 && scandataformat != 2)
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendMultiscanStartCmd(): invalid scandataformat configuration, unsupported scandataformat=" << scandataformat << ", check configuration and use 1 for msgpack or 2 for compact data");
    return false;
  }
  if (!sendSopasCmdCheckResponse(scandataformat_cmd.str(), "sWA ScanDataFormat")) // set scan data output format to MSGPACK (1) or COMPACT (2)
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendMultiscanStartCmd(): sendSopasCmdCheckResponse(\"sWN ScanDataFormat 1\") failed.");
    return false;
  }
  if (performanceprofilenumber >= 0)
  {
    if (!sendSopasCmdCheckResponse(performanceprofilenumber_cmd.str(), "sWA PerformanceProfileNumber"))
    {
      ROS_ERROR_STREAM("## ERROR SickScanServices::sendMultiscanStartCmd(): sendSopasCmdCheckResponse(\"sWN PerformanceProfileNumber ..\") failed.");
      return false;
    }
  }
  if (scanner_type == SICK_SCANNER_SCANSEGMENT_XD_NAME && !sendSopasCmdCheckResponse("sWN ScanDataPreformatting 1", "sWA ScanDataPreformatting")) // ScanDataPreformatting for multiScan136 only
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendMultiscanStartCmd(): sendSopasCmdCheckResponse(\"sWN ScanDataPreformatting 1\") failed.");
    return false;
  }
  if (imu_enable && !sendSopasCmdCheckResponse(imu_eth_settings_cmd.str(), "sWA ImuDataEthSettings")) // imu data eth settings
  {
    if (scanner_type == SICK_SCANNER_PICOSCAN_NAME)
    {
      // picoScan150 ships in 2 variants, with and without IMU resp. IMU licence (picoScan150 w/o addons).
      // For picoScan150 w/o addons we disable the IMU, if SOPAS commands "ImuDataEthSettings" or "ImuDataEnable" failed.
      ROS_WARN_STREAM("## WARNING SickScanServices::sendMultiScanStartCmd(): sendSopasCmdCheckResponse(\"" << imu_eth_settings_cmd.str() << "\") failed.");
      printPicoScanImuDisabledWarning();
      imu_enable = false;
    }
    else
    {
      ROS_ERROR_STREAM("## ERROR SickScanServices::sendMultiScanStartCmd(): sendSopasCmdCheckResponse(\"" << imu_eth_settings_cmd.str() << "\") failed.");
    }
  }
  if (!sendRun())
  {
    return false;
  }
  if (!sendAuthorization())
  {
     return false;
  }
  if (!sendSopasCmdCheckResponse("sWN ScanDataEnable 1", "sWA ScanDataEnable")) // enable scan data output
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendMultiScanStartCmd(): sendSopasCmdCheckResponse(\"sWN ScanDataEnable 1\") failed.");
    return false;
  }
  if (imu_enable && !sendSopasCmdCheckResponse("sWN ImuDataEnable 1", "sWA ImuDataEnable")) // enable imu data transfer
  {
    if (scanner_type == SICK_SCANNER_PICOSCAN_NAME)
    {
      // picoScan150 ships in 2 variants, with and without IMU resp. IMU licence (picoScan150 w/o addons).
      // For picoScan150 w/o addons we disable the IMU, if SOPAS commands "ImuDataEthSettings" or "ImuDataEnable" failed.
      ROS_WARN_STREAM("## WARNING SickScanServices::sendMultiScanStartCmd(): sendSopasCmdCheckResponse(\"sWN ImuDataEnable 1\") failed.");
      printPicoScanImuDisabledWarning();
      imu_enable = false;
    }
    else
    {
      ROS_ERROR_STREAM("## ERROR SickScanServices::sendMultiScanStartCmd(): sendSopasCmdCheckResponse(\"sWN ImuDataEnable 1\") failed.");
    }
  }
  if (!sendRun())
  {
    return false;
  }
  if (!sendAuthorization())
  {
     return false;
  }
  if (!sendSopasCmdCheckResponse("sMN LMCstartmeas", "sAN LMCstartmeas")) // start measurement
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendMultiScanStartCmd(): sendSopasCmdCheckResponse(\"sMN LMCstartmeas\") failed.");
    return false;
  }
  return true;
}
#endif // SCANSEGMENT_XD_SUPPORT

#if defined SCANSEGMENT_XD_SUPPORT && SCANSEGMENT_XD_SUPPORT > 0
/*!
 * Sends the multiScan136 stop commands "sWN ScanDataEnable 0" and "sMN Run"
 */
bool sick_scan_xd::SickScanServices::sendMultiScanStopCmd(bool imu_enable)
{
  if (!sendSopasCmdCheckResponse("sWN ScanDataEnable 0", "sWA ScanDataEnable")) // disable scan data output
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendMultiScanStopCmd(): sendSopasCmdCheckResponse(\"sWN ScanDataEnable 0\") failed.");
    return false;
  }
  if (imu_enable && !sendSopasCmdCheckResponse("sWN ImuDataEnable 0", "sWA ImuDataEnable")) // disable imu data output
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::sendMultiScanStopCmd(): sendSopasCmdCheckResponse(\"sWN ImuDataEnable 0\") failed.");
    return false;
  }
  if (!sendRun())
  {
    return false;
  }
  return true;
}
#endif // SCANSEGMENT_XD_SUPPORT

union FLOAT_BYTE32_UNION
{
  uint8_t u8_bytes[4];
  uint32_t u32_bytes;
  int32_t i32_bytes;
  float value;
};

/*!
* Converts a hex string (hex_str: 4 byte hex value as string, little or big endian) to float.
* Check f.e. by https://www.h-schmidt.net/FloatConverter/IEEE754.html
* Examples:
* convertHexStringToFloat("C0490FF9", true) returns -3.14
* convertHexStringToFloat("3FC90FF9", true) returns +1.57
*/
float sick_scan_xd::SickScanServices::convertHexStringToFloat(const std::string& hex_str, bool hexStrIsBigEndian)
{
  FLOAT_BYTE32_UNION hex_buffer;
  if(hexStrIsBigEndian)
  {
    for(int m = 3, n = 1; n < hex_str.size(); n+=2, m--)
    {
      char hexval[4] = { hex_str[n-1], hex_str[n], '\0', '\0' };
      hex_buffer.u8_bytes[m] = (uint8_t)(std::strtoul(hexval, NULL, 16) & 0xFF);
    }
  }
  else
  {
    for(int m = 0, n = 1; n < hex_str.size(); n+=2, m++)
    {
      char hexval[4] = { hex_str[n-1], hex_str[n], '\0', '\0' };
      hex_buffer.u8_bytes[m] = (uint8_t)(std::strtoul(hexval, NULL, 16) & 0xFF);
    }
  }
  // ROS_DEBUG_STREAM("convertHexStringToFloat(" << hex_str << ", " << hexStrIsBigEndian << "): " << std::hex << hex_buffer.u32_bytes << " = " << std::fixed << hex_buffer.value);
  return hex_buffer.value;
}

/*!
* Converts a float value to hex string (hex_str: 4 byte hex value as string, little or big endian).
* Check f.e. by https://www.h-schmidt.net/FloatConverter/IEEE754.html
* Examples:
* convertFloatToHexString(-3.14, true) returns "C0490FDB"
* convertFloatToHexString(+1.57, true) returns "3FC90FF8"
*/
std::string sick_scan_xd::SickScanServices::convertFloatToHexString(float value, bool hexStrIsBigEndian)
{
  FLOAT_BYTE32_UNION hex_buffer;
  hex_buffer.value = value;
  std::stringstream hex_str;
  if(hexStrIsBigEndian)
  {
    for(int n = 3; n >= 0; n--)
      hex_str << std::setfill('0') << std::setw(2) << std::uppercase << std::hex << (int)(hex_buffer.u8_bytes[n]);
  }
  else
  {
    for(int n = 0; n < 4; n++)
      hex_str << std::setfill('0') << std::setw(2) << std::uppercase << std::hex << (int)(hex_buffer.u8_bytes[n]);
  }
  // ROS_DEBUG_STREAM("convertFloatToHexString(" << value << ", " << hexStrIsBigEndian << "): " << hex_str.str());
  return hex_str.str();
}

/*!
* Converts a hex string coded in 1/10000 deg (hex_str: 4 byte hex value as string, little or big endian) to an angle in [deg] (float).
*/
float sick_scan_xd::SickScanServices::convertHexStringToAngleDeg(const std::string& hex_str, bool hexStrIsBigEndian)
{
  char hex_str_8byte[9] = "00000000";
  for(int m=7,n=hex_str.size()-1; n >= 0; m--,n--)
    hex_str_8byte[m] = hex_str[n]; // fill with leading '0'
  FLOAT_BYTE32_UNION hex_buffer;
  if(hexStrIsBigEndian)
  {
    for(int m = 3, n = 1; n < 8; n+=2, m--)
    {
      char hexval[4] = { hex_str_8byte[n-1], hex_str_8byte[n], '\0', '\0' };
      hex_buffer.u8_bytes[m] = (uint8_t)(std::strtoul(hexval, NULL, 16) & 0xFF);
    }
  }
  else
  {
    for(int m = 0, n = 1; n < 8; n+=2, m++)
    {
      char hexval[4] = { hex_str_8byte[n-1], hex_str_8byte[n], '\0', '\0' };
      hex_buffer.u8_bytes[m] = (uint8_t)(std::strtoul(hexval, NULL, 16) & 0xFF);
    }
  }
  float angle_deg = (float)(hex_buffer.i32_bytes / 10000.0);
  // ROS_DEBUG_STREAM("convertHexStringToAngleDeg(" << hex_str << ", " << hexStrIsBigEndian << "): " << angle_deg << " [deg]");
  return angle_deg;
}

/*!
* Converts an angle in [deg] to hex string coded in 1/10000 deg (hex_str: 4 byte hex value as string, little or big endian).
*/
std::string sick_scan_xd::SickScanServices::convertAngleDegToHexString(float angle_deg, bool hexStrIsBigEndian)
{
  int32_t angle_val = (int32_t)std::round(angle_deg * 10000.0f);
  FLOAT_BYTE32_UNION hex_buffer;
  hex_buffer.i32_bytes = angle_val;
  std::stringstream hex_str;
  if(hexStrIsBigEndian)
  {
    for(int n = 3; n >= 0; n--)
      hex_str << std::setfill('0') << std::setw(2) << std::uppercase << std::hex << (int)(hex_buffer.u8_bytes[n]);
  }
  else
  {
    for(int n = 0; n < 4; n++)
      hex_str << std::setfill('0') << std::setw(2) << std::uppercase << std::hex << (int)(hex_buffer.u8_bytes[n]);
  }
  // ROS_DEBUG_STREAM("convertAngleDegToHexString(" << angle_deg << "  [deg], " << hexStrIsBigEndian << "): " << hex_str.str());
  return hex_str.str();
}

#if defined SCANSEGMENT_XD_SUPPORT && SCANSEGMENT_XD_SUPPORT > 0
/*!
* Sends the SOPAS command to query multiScan136 filter settings (FREchoFilter, LFPangleRangeFilter, host_LFPlayerFilter)
* @param[out] host_FREchoFilter FREchoFilter settings, default: 1, otherwise 0 for FIRST_ECHO (EchoCount=1), 1 for ALL_ECHOS (EchoCount=3), or 2 for LAST_ECHO (EchoCount=1)
* @param[out] host_LFPangleRangeFilter LFPangleRangeFilter settings, default: "0 -180.0 +180.0 -90.0 +90.0 1", otherwise "<enabled> <azimuth_start> <azimuth_stop> <elevation_start> <elevation_stop> <beam_increment>" with azimuth and elevation given in degree
* @param[out] host_LFPlayerFilter LFPlayerFilter settings, default: "0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1", otherwise  "<enabled> <layer0-enabled> <layer1-enabled> <layer2-enabled> ... <layer15-enabled>" with 1 for enabled and 0 for disabled
* @param[out] msgpack_validator_filter_settings; // filter settings for msgpack validator: required_echos, azimuth_start, azimuth_end. elevation_start, elevation_end, layer_filter
*/
bool sick_scan_xd::SickScanServices::queryMultiScanFiltersettings(int& host_FREchoFilter, std::string& host_LFPangleRangeFilter, std::string& host_LFPlayerFilter,
  sick_scansegment_xd::MsgpackValidatorFilterConfig& msgpack_validator_filter_settings, const std::string& scanner_type)
{
  std::vector<std::vector<unsigned char>> sopasRepliesBin;
  std::vector<std::string> sopasRepliesString;

  // Query FREchoFilter, LFPangleRangeFilter and LFPlayerFilter settings
  bool enableFREchoFilter = true, enableLFPangleRangeFilter = true, enableLFPlayerFilter = true;
  if (scanner_type == SICK_SCANNER_PICOSCAN_NAME) // LFPangleRangeFilter and LFPlayerFilter not supported by picoscan150
  {
    // enableLFPangleRangeFilter = false;
    enableLFPlayerFilter = false;
  }
  std::vector<std::string> sopasCommands;
  if (enableFREchoFilter)
    sopasCommands.push_back("FREchoFilter");
  if (enableLFPangleRangeFilter)
    sopasCommands.push_back("LFPangleRangeFilter");
  if (enableLFPlayerFilter)
    sopasCommands.push_back("LFPlayerFilter");
  for(int n = 0; n < sopasCommands.size(); n++)
  {
    std::string sopasRequest = "sRN " + sopasCommands[n];
    std::string sopasExpectedResponse = "sRA " +  sopasCommands[n];
    std::vector<unsigned char> sopasReplyBin;
    std::string sopasReplyString;
    if(!sendSopasAndCheckAnswer(sopasRequest, sopasReplyBin, sopasReplyString) || sopasReplyString.find(sopasExpectedResponse) == std::string::npos)
    {
      ROS_ERROR_STREAM("## ERROR SickScanServices::queryMultiScanFiltersettings(): sendSopasAndCheckAnswer(\"" << sopasRequest << "\") failed or unexpected response: \"" << sopasReplyString << "\", expected: \"" << sopasExpectedResponse << "\"");
      return false;
    }
    ROS_DEBUG_STREAM("SickScanServices::queryMultiScanFiltersettings(): request: \"" << sopasRequest << "\", response: \"" << sopasReplyString << "\"");
    sopasRepliesBin.push_back(sopasReplyBin);
    sopasRepliesString.push_back(sopasReplyString);
  }

  // Convert sopas answers
  std::vector<std::vector<std::string>> sopasTokens;
  for(int n = 0; n < sopasCommands.size(); n++)
  {
    std::string parameterString = sopasRepliesString[n].substr(4 + sopasCommands[n].size() + 1);
    std::vector<std::string> parameterToken;
    sick_scansegment_xd::util::parseVector(parameterString, parameterToken, ' ');
    sopasTokens.push_back(parameterToken);
    ROS_INFO_STREAM("SickScanServices::queryMultiScanFiltersettings(): " << sopasCommands[n] << ": \"" << parameterString << "\" = {" << sick_scansegment_xd::util::printVector(parameterToken, ",") << "}");
  }

  std::vector<float> multiscan_angles_deg;
  std::vector<int> layer_active_vector;
  for(int sopasCommandCnt = 0; sopasCommandCnt < sopasCommands.size(); sopasCommandCnt++)
  {
    const std::string& sopasCommand = sopasCommands[sopasCommandCnt];
    const std::vector<std::string>& sopasToken = sopasTokens[sopasCommandCnt];

    if (sopasCommand == "FREchoFilter") // Parse FREchoFilter
    {
      if(sopasToken.size() == 1)
      {
        host_FREchoFilter = std::stoi(sopasToken[0]);
      }
      else
      {
        ROS_ERROR_STREAM("## ERROR SickScanServices::queryMultiScanFiltersettings(): parse error in FREchoFilter");
        return false;
      }
    }

    if (sopasCommand == "LFPangleRangeFilter") // Parse LFPangleRangeFilter
    {
      if(sopasToken.size() == 6)
      {
        std::stringstream parameter;
        int filter_enabled = std::stoi(sopasToken[0]); // <enabled>
        parameter << filter_enabled;
        for(int n = 1; n < 5; n++) // <azimuth_start> <azimuth_stop> <elevation_start> <elevation_stop>
        {
          // float angle_deg = (convertHexStringToFloat(sopasToken[n], SCANSEGMENT_XD_SOPAS_ARGS_BIG_ENDIAN) * 180.0 / M_PI);
          float angle_deg = convertHexStringToAngleDeg(sopasToken[n], SCANSEGMENT_XD_SOPAS_ARGS_BIG_ENDIAN);
          parameter << " " << angle_deg;
          if(filter_enabled)
            multiscan_angles_deg.push_back(angle_deg);
        }
        parameter << " " << sopasToken[5]; // <beam_increment>
        host_LFPangleRangeFilter = parameter.str();
      }
      else
      {
        ROS_ERROR_STREAM("## ERROR SickScanServices::queryMultiScanFiltersettings(): parse error in LFPangleRangeFilter");
        return false;
      }
    }

    if (sopasCommand == "LFPlayerFilter") // Parse LFPlayerFilter
    {
      if(sopasToken.size() == 17)
      {
        std::stringstream parameter;
        int filter_enabled = std::stoi(sopasToken[0]); // <enabled>
        parameter << filter_enabled;
        for(int n = 1; n < sopasToken.size(); n++)
        {
          int layer_active = std::stoi(sopasToken[n]);
          if(filter_enabled)
            layer_active_vector.push_back(layer_active);
          parameter << " " << layer_active;
        }
        host_LFPlayerFilter = parameter.str();
      }
      else
      {
        ROS_ERROR_STREAM("## ERROR SickScanServices::queryMultiScanFiltersettings(): parse error in LFPlayerFilter");
        return false;
      }
    }
  }

  // Set filter settings for validation of msgpack data, i.e. set config.msgpack_validator_required_echos, config.msgpack_validator_azimuth_start, config.msgpack_validator_azimuth_end,
  // config.msgpack_validator_elevation_start, config.msgpack_validator_elevation_end, config.msgpack_validator_layer_filter according to the queried filter settings
  if(host_FREchoFilter == 0 || host_FREchoFilter == 2) // 0: FIRST_ECHO (EchoCount=1), 2: LAST_ECHO (EchoCount=1)
  {
    msgpack_validator_filter_settings.msgpack_validator_required_echos = { 0 }; // one echo with index 0
  }
  else if(host_FREchoFilter == 1) // 1: ALL_ECHOS (EchoCount=3)
  {
    msgpack_validator_filter_settings.msgpack_validator_required_echos = { 0, 1, 2 }; // three echos with index 0, 1, 2
  }
  else
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::queryMultiScanFiltersettings(): unexpected value of FREchoFilter = " << host_FREchoFilter
    << ", expected 0: FIRST_ECHO (EchoCount=1), 1: ALL_ECHOS (EchoCount=3) or 2: LAST_ECHO (EchoCount=1)");
    return false;
  }
  if(multiscan_angles_deg.size() == 4) // otherwise LFPangleRangeFilter disabled (-> use configured default values)
  {
    msgpack_validator_filter_settings.msgpack_validator_azimuth_start = (multiscan_angles_deg[0] * M_PI / 180);
    msgpack_validator_filter_settings.msgpack_validator_azimuth_end = (multiscan_angles_deg[1] * M_PI / 180);
    msgpack_validator_filter_settings.msgpack_validator_elevation_start = (multiscan_angles_deg[2] * M_PI / 180);
    msgpack_validator_filter_settings.msgpack_validator_elevation_end = (multiscan_angles_deg[3] * M_PI / 180);
  }
  if(layer_active_vector.size() == 16)  // otherwise LFPlayerFilter disabled (-> use configured default values)
  {
    msgpack_validator_filter_settings.msgpack_validator_layer_filter = layer_active_vector;
  }

  // Example: sopas.FREchoFilter = "1", sopas.LFPangleRangeFilter = "0 -180 180 -90.0002 90.0002 1", sopas.LFPlayerFilter = "0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"
  // msgpack_validator_required_echos = { 0 }, msgpack_validator_angles = { -3.14159 3.14159 -1.5708 1.5708 } [rad], msgpack_validator_layer_filter = { 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 }
  ROS_INFO_STREAM("SickScanServices::queryMultiScanFiltersettings(): sopas.FREchoFilter = \"" << host_FREchoFilter
    << "\", sopas.LFPangleRangeFilter = \"" << host_LFPangleRangeFilter
    << "\", sopas.LFPlayerFilter = \"" << host_LFPlayerFilter  << "\"");
  ROS_INFO_STREAM("SickScanServices::queryMultiScanFiltersettings(): msgpack_validator_required_echos = { " << sick_scansegment_xd::util::printVector(msgpack_validator_filter_settings.msgpack_validator_required_echos)
    << " }, msgpack_validator_angles = { " << msgpack_validator_filter_settings.msgpack_validator_azimuth_start << " " << msgpack_validator_filter_settings.msgpack_validator_azimuth_end
    << " " << msgpack_validator_filter_settings.msgpack_validator_elevation_start << " " << msgpack_validator_filter_settings.msgpack_validator_elevation_end
    << " } [rad], msgpack_validator_layer_filter = { " << sick_scansegment_xd::util::printVector(msgpack_validator_filter_settings.msgpack_validator_layer_filter)  << " }");
  return true;
}
#endif // SCANSEGMENT_XD_SUPPORT

#if defined SCANSEGMENT_XD_SUPPORT && SCANSEGMENT_XD_SUPPORT > 0
/*!
* Sends the SOPAS command to write multiScan136 filter settings (FREchoFilter, LFPangleRangeFilter, host_LFPlayerFilter)
* @param[in] host_FREchoFilter FREchoFilter settings, default: 1, otherwise 0 for FIRST_ECHO (EchoCount=1), 1 for ALL_ECHOS (EchoCount=3), or 2 for LAST_ECHO (EchoCount=1)
* @param[in] host_LFPangleRangeFilter LFPangleRangeFilter settings, default: "0 -180.0 +180.0 -90.0 +90.0 1", otherwise "<enabled> <azimuth_start> <azimuth_stop> <elevation_start> <elevation_stop> <beam_increment>" with azimuth and elevation given in degree
* @param[in] host_LFPlayerFilter LFPlayerFilter settings, default: "0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1", otherwise  "<enabled> <layer0-enabled> <layer1-enabled> <layer2-enabled> ... <layer15-enabled>" with 1 for enabled and 0 for disabled
* @param[in] host_LFPintervalFilter Optionally set LFPintervalFilter to "<enabled> <N>" with 1 for enabled and 0 for disabled and N to reduce output to every N-th scan
*/
bool sick_scan_xd::SickScanServices::writeMultiScanFiltersettings(int host_FREchoFilter, const std::string& host_LFPangleRangeFilter, const std::string& host_LFPlayerFilter, const std::string& host_LFPintervalFilter, const std::string& scanner_type)

{
  bool enableFREchoFilter = true, enableLFPangleRangeFilter = true, enableLFPlayerFilter = true, enableLFPintervalFilter = true;
  if (scanner_type == SICK_SCANNER_PICOSCAN_NAME) // LFPangleRangeFilter, LFPlayerFilter ant LFPintervalFilter not supported by picoscan150
  {
    // enableLFPangleRangeFilter = false;
    enableLFPlayerFilter = false;
    // enableLFPintervalFilter = false;
  }

  // Write FREchoFilter
  if(enableFREchoFilter && host_FREchoFilter >= 0) // otherwise not configured or supported
  {
    std::string sopasRequest = "sWN FREchoFilter " + std::to_string(host_FREchoFilter), sopasExpectedResponse = "sWA FREchoFilter";
    if (!sendSopasCmdCheckResponse(sopasRequest, sopasExpectedResponse))
    {
      ROS_ERROR_STREAM("## ERROR SickScanServices::writeMultiScanFiltersettings(): sendSopasCmdCheckResponse(\"" << sopasRequest << "\") failed.");
      return false;
    }
  }

  // Write LFPangleRangeFilter
  if(enableLFPangleRangeFilter && !host_LFPangleRangeFilter.empty()) // otherwise not configured or supported
  {
    // convert deg to rad and float to hex
    std::vector<std::string> parameter_token;
    sick_scansegment_xd::util::parseVector(host_LFPangleRangeFilter, parameter_token, ' ');
    if(parameter_token.size() != 6)
    {
      ROS_ERROR_STREAM("## ERROR SickScanServices::writeMultiScanFiltersettings(): can't split host_LFPangleRangeFilter = \"" << host_LFPangleRangeFilter << "\", expected 6 values separated by space");
      ROS_ERROR_STREAM("## ERROR SickScanServices::writeMultiScanFiltersettings() failed.");
      return false;
    }
    int filter_enabled = std::stoi(parameter_token[0]); // <enabled>
    std::vector<float> angle_deg;
    for(int n = 1; n < 5; n++)
      angle_deg.push_back(std::stof(parameter_token[n]));
    int beam_increment = std::stoi(parameter_token[5]); // <beam_increment>
    std::stringstream sopas_parameter;
    sopas_parameter << filter_enabled;
    for(int n = 0; n < angle_deg.size(); n++)
    {
      // sopas_parameter << " " << convertFloatToHexString(angle_deg[n] * M_PI / 180, SCANSEGMENT_XD_SOPAS_ARGS_BIG_ENDIAN);
      sopas_parameter << " " << convertAngleDegToHexString(angle_deg[n], SCANSEGMENT_XD_SOPAS_ARGS_BIG_ENDIAN);
    }
    sopas_parameter << " " << beam_increment;
    // Write LFPangleRangeFilter
    std::string sopasRequest = "sWN LFPangleRangeFilter " + sopas_parameter.str(), sopasExpectedResponse = "sWA LFPangleRangeFilter";
    if (!sendSopasCmdCheckResponse(sopasRequest, sopasExpectedResponse))
    {
      ROS_ERROR_STREAM("## ERROR SickScanServices::writeMultiScanFiltersettings(): sendSopasCmdCheckResponse(\"" << sopasRequest << "\") failed.");
      return false;
    }
  }

  // Write LFPlayerFilter
  if(enableLFPlayerFilter && !host_LFPlayerFilter.empty()) // otherwise not configured or supported
  {
    std::string sopasRequest = "sWN LFPlayerFilter " + host_LFPlayerFilter, sopasExpectedResponse = "sWA LFPlayerFilter";
    if (!sendSopasCmdCheckResponse(sopasRequest, sopasExpectedResponse))
    {
      ROS_ERROR_STREAM("## ERROR SickScanServices::writeMultiScanFiltersettings(): sendSopasCmdCheckResponse(\"" << sopasRequest << "\") failed.");
      return false;
    }
  }

  // Write LFPintervalFilter
  if(enableLFPintervalFilter && !host_LFPintervalFilter.empty()) // otherwise not configured or supported
  {
    std::string sopasRequest = "sWN LFPintervalFilter " + host_LFPintervalFilter, sopasExpectedResponse = "sWA LFPintervalFilter";
    if (!sendSopasCmdCheckResponse(sopasRequest, sopasExpectedResponse))
    {
      ROS_ERROR_STREAM("## ERROR SickScanServices::writeMultiScanFiltersettings(): sendSopasCmdCheckResponse(\"" << sopasRequest << "\") failed.");
      return false;
    }
  }

  // Apply the settings
  if (!sendSopasCmdCheckResponse("sMN Run", "sAN Run"))
  {
    ROS_ERROR_STREAM("## ERROR SickScanServices::writeMultiScanFiltersettings(): sendSopasCmdCheckResponse(\"sMN Run\") failed.");
    return false;
  }
  return true;
}
#endif // SCANSEGMENT_XD_SUPPORT

/*!
* Callbacks for service messages.
* @param[in] service_request ros service request to lidar
* @param[out] service_response service response from lidar
* @return true on success, false in case of errors.
*/
bool sick_scan_xd::SickScanServices::serviceCbSCdevicestate(sick_scan_srv::SCdevicestateSrv::Request &service_request, sick_scan_srv::SCdevicestateSrv::Response &service_response)
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

bool sick_scan_xd::SickScanServices::serviceCbSCreboot(sick_scan_srv::SCrebootSrv::Request &service_request, sick_scan_srv::SCrebootSrv::Response &service_response)
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

bool sick_scan_xd::SickScanServices::serviceCbSCsoftreset(sick_scan_srv::SCsoftresetSrv::Request &service_request, sick_scan_srv::SCsoftresetSrv::Response &service_response)
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

bool sick_scan_xd::SickScanServices::serviceCbSickScanExit(sick_scan_srv::SickScanExitSrv::Request &service_request, sick_scan_srv::SickScanExitSrv::Response &service_response)
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

#if __ROS_VERSION > 0
bool sick_scan_xd::SickScanServices::serviceCbFieldSetRead(sick_scan_srv::FieldSetReadSrv::Request &service_request, sick_scan_srv::FieldSetReadSrv::Response &service_response)
{
  SickScanFieldMonSingleton *fieldMon = SickScanFieldMonSingleton::getInstance();
  int field_set_selection_method = fieldMon->getFieldSelectionMethod();
  int active_field_set = fieldMon->getActiveFieldset();
  std::vector<unsigned char> sopasReply;
  int result1 = m_common_tcp->readFieldSetSelectionMethod(field_set_selection_method, sopasReply);
  int result2 = m_common_tcp->readActiveFieldSet(active_field_set, sopasReply);
  service_response.success = (result1 == ExitSuccess && result2 == ExitSuccess);
  service_response.field_set_selection_method = field_set_selection_method;
  service_response.active_field_set = active_field_set;
  return true;
}

bool sick_scan_xd::SickScanServices::serviceCbFieldSetWrite(sick_scan_srv::FieldSetWriteSrv::Request &service_request, sick_scan_srv::FieldSetWriteSrv::Response &service_response)
{
  int field_set_selection_method = service_request.field_set_selection_method_in;
  int active_field_set = service_request.active_field_set_in;
  std::vector<unsigned char> sopasReply;
  std::string sopasReplyString;
  int result1 = ExitSuccess, result2 = ExitSuccess;
  if (field_set_selection_method >= 0)
  {
    if(!sendAuthorization())
      result1 = ExitError;
    if(!sendSopasAndCheckAnswer("sMN LMCstopmeas", sopasReply, sopasReplyString))
      result1 = ExitError;
    result1 = m_common_tcp->writeFieldSetSelectionMethod(field_set_selection_method, sopasReply);
    if(!sendSopasAndCheckAnswer("sMN LMCstartmeas", sopasReply, sopasReplyString))
      result1 = ExitError;
    if(!sendSopasAndCheckAnswer("sMN Run", sopasReply, sopasReplyString))
      result1 = ExitError;
  }
  if (active_field_set >= 0)
  {
    result2 = m_common_tcp->writeActiveFieldSet(active_field_set, sopasReply);
  }
  int result3 = m_common_tcp->readFieldSetSelectionMethod(field_set_selection_method, sopasReply);
  int result4 = m_common_tcp->readActiveFieldSet(active_field_set, sopasReply);
  service_response.success = (result1 == ExitSuccess && result2 == ExitSuccess && result3 == ExitSuccess && result4 == ExitSuccess);
  service_response.field_set_selection_method = field_set_selection_method;
  service_response.active_field_set = active_field_set;
  service_response.success = true;
  return true;
}
#endif
