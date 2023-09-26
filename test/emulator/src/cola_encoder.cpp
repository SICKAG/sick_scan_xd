/*
 * @brief cola_encoder encodes service requests to cola telegrams, parses cola responses and
 * converts them to service responses.
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
#include "sick_scan/cola_encoder.h"

/*!
 * Converts the service request for service SickDevSetLidarConfigSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickDevSetLidarConfigSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickDevSetLidarConfigSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN DevSetLidarConfig";
  cola_ascii << std::showpos << " +" << service_request.index << " +" << service_request.minrange << " +" << service_request.maxrange << " " << service_request.minangle << " " << service_request.maxangle
    << " " << service_request.x << " " << service_request.y << " " << service_request.yaw 
    << std::noshowpos << " " << (service_request.upsidedown?1:0) 
    << " +" << service_request.ip.length() << " " << service_request.ip << " +" << service_request.port
    << " " << service_request.interfacetype << " " << service_request.maplayer << " " << (service_request.active?1:0);
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickDevSetLidarConfigSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickDevSetLidarConfigSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickDevSetLidarConfigSrv::Response& service_response)
{
  service_response.set = false;
  service_response.executed = false;
  if(cola_response.parameter.size() == 2)
  {
    service_response.set = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
    service_response.executed = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[1], false);
  }
  return service_response.set;
}

/*!
 * Converts the service request for service SickDevGetLidarConfigSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickDevGetLidarConfigSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickDevGetLidarConfigSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN DevGetLidarConfig";
  cola_ascii << std::showpos << " " << service_request.scannerindex;
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickDevGetLidarConfigSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickDevGetLidarConfigSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickDevGetLidarConfigSrv::Response& service_response)
{
  if(cola_response.parameter.size() == 13)
  {
    service_response.minrange = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[0], -1, 0U);
    service_response.maxrange = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[1], -1, 0U);
    service_response.minangle = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[2], -1, 0);
    service_response.maxangle = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[3], -1, 0);
    service_response.x = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[4], -1, 0);
    service_response.y = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[5], -1, 0);
    service_response.yaw = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[6], -1, 0);
    service_response.upsidedown = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[7], false);
    service_response.ip = cola_response.parameter[8];
    service_response.port = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[9], -1, 0U);
    service_response.interfacetype = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[10], -1, 0U);
    service_response.maplayer = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[11], -1, 0U);
    service_response.active = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[12], false);
    return true;
  }
  if(cola_response.parameter.size() >= 14)
  {
    service_response.minrange = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[0], -1, 0U);
    service_response.maxrange = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[1], -1, 0U);
    service_response.minangle = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[2], -1, 0);
    service_response.maxangle = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[3], -1, 0);
    service_response.x = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[4], -1, 0);
    service_response.y = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[5], -1, 0);
    service_response.yaw = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[6], -1, 0);
    service_response.upsidedown = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[7], false);
    int n = 8;
    int ip_strlen = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[n++], 10, 0U);
    service_response.ip = cola_response.parameter[n++];
    while (n + 4 < cola_response.parameter.size() && service_response.ip.length() < ip_strlen)
    {
        service_response.ip = service_response.ip + " " + cola_response.parameter[n++];
    }
    service_response.port = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[n++], -1, 0U);
    service_response.interfacetype = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[n++], -1, 0U);
    service_response.maplayer = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[n++], -1, 0U);
    service_response.active = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[n++], false);
    return true;
  }
  return false;
}

/*!
 * Converts the service request for service SickLocSetMapSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocSetMapSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocSetMapSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN LocSetMap +" << service_request.mapfilename.length() << " " << service_request.mapfilename;
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocSetMapSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocSetMapSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocSetMapSrv::Response& service_response)
{
  service_response.set = false;
  service_response.executed = false;
  if(cola_response.parameter.size() == 2)
  {
    service_response.set = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
    service_response.executed = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[1], false);
  }
  return service_response.set;
}

/*!
 * Converts the service request for service SickLocMapSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocMapSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocMapSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sRN LocMap";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocMapSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocMapSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocMapSrv::Response& service_response)
{
  service_response.success = false;
  if(cola_response.parameter.size() == 1)
  {
    service_response.map = cola_response.parameter[0];
    service_response.success = true;
  }
  if(cola_response.parameter.size() > 1)
  {
    int str_len = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[0], -1, 0U);
    service_response.map = cola_response.parameter[1];
    for (int n = 2; n < cola_response.parameter.size() && service_response.map.length() < str_len; n++)
    {
        service_response.map = service_response.map + " " + cola_response.parameter[n];
    }
    service_response.success = true;
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocMapStateSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocMapStateSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocMapStateSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sRN LocMapState";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocMapStateSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocMapStateSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocMapStateSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.mapstate = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
    service_response.success = true;
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocInitializePoseSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocInitializePoseSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocInitializePoseSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN LocInitializePose";
  cola_ascii << std::showpos << " " << service_request.x << " " << service_request.y << " " << service_request.yaw << " +" << service_request.sigmatranslation;
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocInitializePoseSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocInitializePoseSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocInitializePoseSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.success = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocInitialPoseSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocInitialPoseSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocInitialPoseSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sRN LocInitialPose";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocInitialPoseSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocInitialPoseSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocInitialPoseSrv::Response& service_response)
{
  service_response.success = false;
  if(cola_response.parameter.size() >= 4)
  {
    service_response.x = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[0], -1, 0);
    service_response.y = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[1], -1, 0);
    service_response.yaw = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[2], -1, 0);
    service_response.sigmatranslation = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[3], -1, 0U);
    service_response.success = true;
  }
  return service_response.success;
}


/*!
 * Converts the service request for service SickLocSetReflectorsForSupportActiveSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocSetReflectorsForSupportActiveSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocSetReflectorsForSupportActiveSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN LocSetReflectorsForSupportActive";
  cola_ascii << std::noshowpos << " " << (service_request.active?1:0);
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocSetReflectorsForSupportActiveSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocSetReflectorsForSupportActiveSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocSetReflectorsForSupportActiveSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.success = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocReflectorsForSupportActiveSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocReflectorsForSupportActiveSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocReflectorsForSupportActiveSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sRN LocReflectorsForSupportActive";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocReflectorsForSupportActiveSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocReflectorsForSupportActiveSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocReflectorsForSupportActiveSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.active = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
    service_response.success = true;
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocSetOdometryActiveSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocSetOdometryActiveSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocSetOdometryActiveSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN LocSetOdometryActive";
  cola_ascii << std::noshowpos << " " << (service_request.active?1:0);
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocSetOdometryActiveSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocSetOdometryActiveSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocSetOdometryActiveSrv::Response& service_response)
{
  service_response.set = false;
  service_response.executed = false;
  if(!cola_response.parameter.empty())
  {
    service_response.set = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
    service_response.executed = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[1], false);
  }
  return service_response.set;
}

/*!
 * Converts the service request for service SickLocOdometryActiveSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocOdometryActiveSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocOdometryActiveSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sRN LocOdometryActive";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocOdometryActiveSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocOdometryActiveSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocOdometryActiveSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.active = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
    service_response.success = true;
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocSetOdometryPortSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocSetOdometryPortSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocSetOdometryPortSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN LocSetOdometryPort";
  cola_ascii << " +" << service_request.port;
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocSetOdometryPortSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocSetOdometryPortSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocSetOdometryPortSrv::Response& service_response)
{
  service_response.set = false;
  if(!cola_response.parameter.empty())
  {
    service_response.set = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
    service_response.executed = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[1], false);
  }
  return service_response.set;
}

/*!
 * Converts the service request for service SickLocOdometryPortSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocOdometryPortSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocOdometryPortSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sRN LocOdometryPort";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocOdometryPortSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocOdometryPortSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocOdometryPortSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.port = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[0], -1, 0);
    service_response.success = true;
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocSetOdometryRestrictYMotionSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocSetOdometryRestrictYMotionSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocSetOdometryRestrictYMotionSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN LocSetOdometryRestrictYMotion";
  cola_ascii << std::noshowpos << " " << (service_request.active?1:0);
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocSetOdometryRestrictYMotionSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocSetOdometryRestrictYMotionSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocSetOdometryRestrictYMotionSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.success = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocOdometryRestrictYMotionSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocOdometryRestrictYMotionSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocOdometryRestrictYMotionSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sRN LocOdometryRestrictYMotion";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocOdometryRestrictYMotionSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocOdometryRestrictYMotionSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocOdometryRestrictYMotionSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.active = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
    service_response.success = true;
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocSetAutoStartActiveSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocSetAutoStartActiveSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocSetAutoStartActiveSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN LocSetAutoStartActive";
  cola_ascii << std::noshowpos << " " << (service_request.active?1:0);
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocSetAutoStartActiveSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocSetAutoStartActiveSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocSetAutoStartActiveSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.success = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocAutoStartActiveSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocAutoStartActiveSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocAutoStartActiveSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sRN LocAutoStartActive";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocAutoStartActiveSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocAutoStartActiveSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocAutoStartActiveSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.active = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
    service_response.success = true;
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocSetAutoStartSavePoseIntervalSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocSetAutoStartSavePoseIntervalSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocSetAutoStartSavePoseIntervalSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN LocSetAutoStartSavePoseInterval";
  cola_ascii << std::showpos << " " << service_request.interval;
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocSetAutoStartSavePoseIntervalSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocSetAutoStartSavePoseIntervalSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocSetAutoStartSavePoseIntervalSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.success = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocAutoStartSavePoseIntervalSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocAutoStartSavePoseIntervalSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocAutoStartSavePoseIntervalSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sRN LocAutoStartSavePoseInterval";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocAutoStartSavePoseIntervalSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocAutoStartSavePoseIntervalSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocAutoStartSavePoseIntervalSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.interval = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[0], -1, 0);
    service_response.success = true;
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocSetRingBufferRecordingActiveSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocSetRingBufferRecordingActiveSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocSetRingBufferRecordingActiveSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN LocSetRingBufferRecordingActive";
  cola_ascii << std::noshowpos << " " << (service_request.active?1:0);
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocSetRingBufferRecordingActiveSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocSetRingBufferRecordingActiveSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocSetRingBufferRecordingActiveSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.success = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocRingBufferRecordingActiveSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocRingBufferRecordingActiveSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocRingBufferRecordingActiveSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sRN LocRingBufferRecordingActive";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocRingBufferRecordingActiveSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocRingBufferRecordingActiveSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocRingBufferRecordingActiveSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.active = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
    service_response.success = true;
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickDevGetLidarIdentSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickDevGetLidarIdentSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickDevGetLidarIdentSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN DevGetLidarIdent";
  cola_ascii << std::showpos << " " << service_request.index;
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickDevGetLidarIdentSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickDevGetLidarIdentSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickDevGetLidarIdentSrv::Response& service_response)
{
  service_response.success = false;
  if(cola_response.parameter.size() == 1)
  {
    service_response.scannerident = cola_response.parameter[0];
    service_response.success = true;
  }
  if(cola_response.parameter.size() > 1)
  {
    int str_len = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[0], -1, 0U);
    service_response.scannerident = cola_response.parameter[1];
    for (int n = 2; n < cola_response.parameter.size() && service_response.scannerident.length() < str_len; n++)
    {
        service_response.scannerident = service_response.scannerident + " " + cola_response.parameter[n];
    }
    service_response.success = true;
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickDevGetLidarStateSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickDevGetLidarStateSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickDevGetLidarStateSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN DevGetLidarState";
  cola_ascii << std::showpos << " " << service_request.index;
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickDevGetLidarStateSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickDevGetLidarStateSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickDevGetLidarStateSrv::Response& service_response)
{
  service_response.success = false;
  if(cola_response.parameter.size() >= 3)
  {
    service_response.devicestatus = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[0], -1, 0);
    service_response.deviceconnected = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[1], -1, 0);
    service_response.receivingdata = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[2], -1, 0);
    service_response.success = true;
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickGetSoftwareVersionSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickGetSoftwareVersionSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickGetSoftwareVersionSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN GetSoftwareVersion";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickGetSoftwareVersionSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickGetSoftwareVersionSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickGetSoftwareVersionSrv::Response& service_response)
{
  service_response.success = false;
  if(cola_response.parameter.size() == 1)
  {
    service_response.version = cola_response.parameter[0];
    service_response.success = true;
  }
  if(cola_response.parameter.size() > 1)
  {
    int str_len = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[0], -1, 0U);
    service_response.version = cola_response.parameter[1];
    for (int n = 2; n < cola_response.parameter.size() && service_response.version.length() < str_len; n++)
    {
        service_response.version = service_response.version + " " + cola_response.parameter[n];
    }
    service_response.success = true;
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocAutoStartSavePoseSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocAutoStartSavePoseSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocAutoStartSavePoseSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN LocAutoStartSavePose";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocAutoStartSavePoseSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocAutoStartSavePoseSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocAutoStartSavePoseSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.success = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocForceUpdateSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocForceUpdateSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocForceUpdateSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN LocForceUpdate";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocForceUpdateSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocForceUpdateSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocForceUpdateSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.success = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocSaveRingBufferRecordingSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocSaveRingBufferRecordingSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocSaveRingBufferRecordingSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN LocSaveRingBufferRecording";
  cola_ascii << " +" << service_request.reason.length() << " " << service_request.reason;
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocSaveRingBufferRecordingSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocSaveRingBufferRecordingSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocSaveRingBufferRecordingSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.success = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocStartDemoMappingSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocStartDemoMappingSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocStartDemoMappingSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN LocStartDemoMapping";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocStartDemoMappingSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocStartDemoMappingSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocStartDemoMappingSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.success = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickReportUserMessageSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickReportUserMessageSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickReportUserMessageSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN ReportUserMessage";
  cola_ascii << " +" << service_request.usermessage.length() << " " << service_request.usermessage;
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickReportUserMessageSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickReportUserMessageSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickReportUserMessageSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.success = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickSavePermanentSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickSavePermanentSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickSavePermanentSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN SavePermanent";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickSavePermanentSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickSavePermanentSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickSavePermanentSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.success = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocResultPortSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocResultPortSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocResultPortSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sRN LocResultPort";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocResultPortSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocResultPortSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocResultPortSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.port = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[0], -1, 0);
    service_response.success = true;
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocResultModeSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocResultModeSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocResultModeSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sRN LocResultMode";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocResultModeSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocResultModeSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocResultModeSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.mode = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[0], -1, 0);
    service_response.success = true;
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocResultEndiannessSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocResultEndiannessSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocResultEndiannessSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sRN LocResultEndianness";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocResultEndiannessSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocResultEndiannessSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocResultEndiannessSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.endianness = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[0], -1, 0);
    service_response.success = true;
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocResultStateSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocResultStateSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocResultStateSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sRN LocResultState";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocResultStateSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocResultStateSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocResultStateSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.state = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[0], -1, 0);
    service_response.success = true;
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickLocResultPoseIntervalSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickLocResultPoseIntervalSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickLocResultPoseIntervalSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sRN LocResultPoseInterval";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickLocResultPoseIntervalSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickLocResultPoseIntervalSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickLocResultPoseIntervalSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.interval = sick_scan_xd::ColaParser::convertColaArg(cola_response.parameter[0], -1, 0);
    service_response.success = true;
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickDevSetIMUActiveSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickDevSetIMUActiveSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickDevSetIMUActiveSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sMN DevSetIMUActive";
  cola_ascii << std::noshowpos << " " << (service_request.active?1:0);
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickDevSetIMUActiveSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickDevSetIMUActiveSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickDevSetIMUActiveSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.success = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
  }
  return service_response.success;
}

/*!
 * Converts the service request for service SickDevIMUActiveSrv into a cola ascii telegram
 * @param[in] service_request ros request for service SickDevIMUActiveSrv
 * @return cola ascii telegram
 */
std::string sick_scan_xd::ColaEncoder::encodeServiceRequest(const sick_scan_xd::SickDevIMUActiveSrv::Request& service_request)
{
  std::stringstream cola_ascii;
  cola_ascii << "sRN DevIMUActive";
  return cola_ascii.str();
}

/*!
 * Parses a cola response and converts the arguments to a service response for service SickDevIMUActiveSrv
 * @param[in] cola_response cola ascii telegram (response from localization server)
 * @param[out] service_response converted response for service SickDevIMUActiveSrv
 * @return true on succes or false in case of parse errors
 */
bool sick_scan_xd::ColaEncoder::parseServiceResponse(const sick_scan_xd::SickLocColaTelegramMsg& cola_response, sick_scan_xd::SickDevIMUActiveSrv::Response& service_response)
{
  service_response.success = false;
  if(!cola_response.parameter.empty())
  {
    service_response.active = sick_scan_xd::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
    service_response.success = true;
  }
  return service_response.success;
}
