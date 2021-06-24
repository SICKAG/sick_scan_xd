/*
 * @brief Implementation of ROS services for sick_scan
 *
 * Copyright (C) 2021, Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2021, SICK AG, Waldkirch
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
 *     * Neither the name of Osnabrück University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

sick_scan::SickScanServices::SickScanServices(ros::NodeHandle* nh, sick_scan::SickScanCommonTcp* common_tcp, bool cola_binary)
: m_common_tcp(common_tcp), m_cola_binary(cola_binary)
{
    if(nh)
    {
        m_srv_server_ColaMsg = nh->advertiseService("ColaMsg",&sick_scan::SickScanServices::serviceCbColaMsg, this);
        m_srv_server_ECRChangeArr = nh->advertiseService("ECRChangeArr",&sick_scan::SickScanServices::serviceCbECRChangeArr, this);
        m_srv_server_LIDoutputstate = nh->advertiseService("LIDoutputstate",&sick_scan::SickScanServices::serviceCbLIDoutputstate, this);
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
bool sick_scan::SickScanServices::serviceCbColaMsg(sick_scan::ColaMsgSrv::Request &service_request, sick_scan::ColaMsgSrv::Response &service_response)
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
bool sick_scan::SickScanServices::serviceCbECRChangeArr(sick_scan::ECRChangeArrSrv::Request &service_request, sick_scan::ECRChangeArrSrv::Response &service_response)
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
bool sick_scan::SickScanServices::serviceCbLIDoutputstate(sick_scan::LIDoutputstateSrv::Request &service_request, sick_scan::LIDoutputstateSrv::Response &service_response)
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
