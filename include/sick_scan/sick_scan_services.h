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

#ifndef SICK_SCAN_SERVICES_H_
#define SICK_SCAN_SERVICES_H_

#include "sick_scan/sick_scan_common.h"
#include "sick_scan/sick_scan_common_tcp.h"
#include "sick_scan/ColaMsgSrv.h"
#include "sick_scan/ECRChangeArrSrv.h"
#include "sick_scan/LIDoutputstateSrv.h"

namespace sick_scan
{

  class SickScanServices
  {
  public:

    SickScanServices(ros::NodeHandle* nh = 0, sick_scan::SickScanCommonTcp* common_tcp = 0, bool cola_binary = true);

    virtual ~SickScanServices();

    /*!
     * Callback for service ColaMsg (ColaMsg, send a cola message to lidar).
     * @param[in] service_request ros service request to lidar
     * @param[out] service_response service response from lidar
     * @return true on success, false in case of errors.
     */
    bool serviceCbColaMsg(sick_scan::ColaMsgSrv::Request &service_request, sick_scan::ColaMsgSrv::Response &service_response);

    /*!
     * Callback for service messages (ECRChangeArr, Request status change of monitoring fields on event).
     * Sends a cola telegram "sEN ECRChangeArr {0|1}" and receives the response from the lidar device.
     * @param[in] service_request ros service request to lidar
     * @param[out] service_response service response from lidar
     * @return true on success, false in case of errors.
     */
    bool serviceCbECRChangeArr(sick_scan::ECRChangeArrSrv::Request &service_request, sick_scan::ECRChangeArrSrv::Response &service_response);

    /*!
     * Callback for service messages (LIDoutputstate, Request status change of monitoring fields on event).
     * Sends a cola telegram "sEN LIDoutputstate {0|1}" and receives the response from the lidar device.
     * @param[in] service_request ros service request to lidar
     * @param[out] service_response service response from lidar
     * @return true on success, false in case of errors.
     */
    bool serviceCbLIDoutputstate(sick_scan::LIDoutputstateSrv::Request &service_request, sick_scan::LIDoutputstateSrv::Response &service_response);

  protected:

    /*!
     * Sends a sopas command and returns the lidar reply.
     * @param[in] sopasCmd sopas command to send, f.e. "sEN ECRChangeArr 1"
     * @param[out] sopasReplyBin response from lidar incl. start/stop byte
     * @param[out] sopasReplyString sopasReplyBin converted to string
     * @return true on success, false in case of errors.
     */
    bool sendSopasAndCheckAnswer(const std::string& sopasCmd, std::vector<unsigned char>& sopasReplyBin, std::string& sopasReplyString);

    /*
     * Member data
     */

    bool m_cola_binary;                             ///< cola ascii or cola binary messages
    sick_scan::SickScanCommonTcp* m_common_tcp;     ///< common tcp handler
    ros::ServiceServer m_srv_server_ColaMsg;        ///< service "ColaMsg", &sick_scan::SickScanServices::serviceCbColaMsg
    ros::ServiceServer m_srv_server_ECRChangeArr;   ///< service "ECRChangeArr", &sick_scan::SickScanServices::serviceCbECRChangeArr
    ros::ServiceServer m_srv_server_LIDoutputstate; ///< service "LIDoutputstate", &sick_scan::SickScanServices::serviceCbLIDoutputstate

  }; /* class SickScanServices */

} /* namespace sick_scan */
#endif /* SICK_SCAN_SERVICES_H_ */
