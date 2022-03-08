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

#ifndef SICK_SCAN_SERVICES_H_
#define SICK_SCAN_SERVICES_H_

#include "sick_scan/sick_ros_wrapper.h"
#include "sick_scan/sick_scan_common.h"
#include "sick_scan/sick_scan_common_tcp.h"

namespace sick_scan
{

  class SickScanServices
  {
  public:

    SickScanServices(rosNodePtr nh = 0, sick_scan::SickScanCommonTcp* common_tcp = 0, bool cola_binary = true);

    virtual ~SickScanServices();

    /*!
     * Callback for service ColaMsg (ColaMsg, send a cola message to lidar).
     * @param[in] service_request ros service request to lidar
     * @param[out] service_response service response from lidar
     * @return true on success, false in case of errors.
     */
    bool serviceCbColaMsg(sick_scan_srv::ColaMsgSrv::Request &service_request, sick_scan_srv::ColaMsgSrv::Response &service_response);
    bool serviceCbColaMsgROS2(std::shared_ptr<sick_scan_srv::ColaMsgSrv::Request> service_request, std::shared_ptr<sick_scan_srv::ColaMsgSrv::Response> service_response) { return serviceCbColaMsg(*service_request, *service_response); }     

    /*!
     * Callback for service messages (ECRChangeArr, Request status change of monitoring fields on event).
     * Sends a cola telegram "sEN ECRChangeArr {0|1}" and receives the response from the lidar device.
     * @param[in] service_request ros service request to lidar
     * @param[out] service_response service response from lidar
     * @return true on success, false in case of errors.
     */
    bool serviceCbECRChangeArr(sick_scan_srv::ECRChangeArrSrv::Request &service_request, sick_scan_srv::ECRChangeArrSrv::Response &service_response);
    bool serviceCbECRChangeArrROS2(std::shared_ptr<sick_scan_srv::ECRChangeArrSrv::Request> service_request, std::shared_ptr<sick_scan_srv::ECRChangeArrSrv::Response> service_response) { return serviceCbECRChangeArr(*service_request, *service_response); }     

    /*!
     * Callback for service messages (LIDoutputstate, Request status change of monitoring fields on event).
     * Sends a cola telegram "sEN LIDoutputstate {0|1}" and receives the response from the lidar device.
     * @param[in] service_request ros service request to lidar
     * @param[out] service_response service response from lidar
     * @return true on success, false in case of errors.
     */
    bool serviceCbLIDoutputstate(sick_scan_srv::LIDoutputstateSrv::Request &service_request, sick_scan_srv::LIDoutputstateSrv::Response &service_response);
    bool serviceCbLIDoutputstateROS2(std::shared_ptr<sick_scan_srv::LIDoutputstateSrv::Request> service_request, std::shared_ptr<sick_scan_srv::LIDoutputstateSrv::Response> service_response) { return serviceCbLIDoutputstate(*service_request, *service_response); }     

    /*!
     * Callbacks for service messages.
     * @param[in] service_request ros service request to lidar
     * @param[out] service_response service response from lidar
     * @return true on success, false in case of errors.
     */

    bool serviceCbSCdevicestate(sick_scan_srv::SCdevicestateSrv::Request &service_request, sick_scan_srv::SCdevicestateSrv::Response &service_response);
    bool serviceCbSCdevicestateROS2(std::shared_ptr<sick_scan_srv::SCdevicestateSrv::Request> service_request, std::shared_ptr<sick_scan_srv::SCdevicestateSrv::Response> service_response) { return serviceCbSCdevicestate(*service_request, *service_response); }     

    bool serviceCbSCreboot(sick_scan_srv::SCrebootSrv::Request &service_request, sick_scan_srv::SCrebootSrv::Response &service_response);
    bool serviceCbSCrebootROS2(std::shared_ptr<sick_scan_srv::SCrebootSrv::Request> service_request, std::shared_ptr<sick_scan_srv::SCrebootSrv::Response> service_response) { return serviceCbSCreboot(*service_request, *service_response); }     

    bool serviceCbSCsoftreset(sick_scan_srv::SCsoftresetSrv::Request &service_request, sick_scan_srv::SCsoftresetSrv::Response &service_response);
    bool serviceCbSCsoftresetROS2(std::shared_ptr<sick_scan_srv::SCsoftresetSrv::Request> service_request, std::shared_ptr<sick_scan_srv::SCsoftresetSrv::Response> service_response) { return serviceCbSCsoftreset(*service_request, *service_response); }     

    bool serviceCbSickScanExit(sick_scan_srv::SickScanExitSrv::Request &service_request, sick_scan_srv::SickScanExitSrv::Response &service_response);
    bool serviceCbSickScanExitROS2(std::shared_ptr<sick_scan_srv::SickScanExitSrv::Request> service_request, std::shared_ptr<sick_scan_srv::SickScanExitSrv::Response> service_response) { return serviceCbSickScanExit(*service_request, *service_response); }     

  protected:

    /*!
    * Sends the SOPAS authorization command "sMN SetAccessMode 3 F4724744".
    */
    bool sendAuthorization();

    /*!
    * Sends the SOPAS command "sMN Run", which applies previous send settings
    */
    bool sendRun();

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
    std::string m_client_authorization_pw;
    rosServiceServer<sick_scan_srv::ColaMsgSrv> m_srv_server_ColaMsg;        ///< service "ColaMsg", &sick_scan::SickScanServices::serviceCbColaMsg
    rosServiceServer<sick_scan_srv::ECRChangeArrSrv> m_srv_server_ECRChangeArr;   ///< service "ECRChangeArr", &sick_scan::SickScanServices::serviceCbECRChangeArr
    rosServiceServer<sick_scan_srv::LIDoutputstateSrv> m_srv_server_LIDoutputstate; ///< service "LIDoutputstate", &sick_scan::SickScanServices::serviceCbLIDoutputstate
    rosServiceServer<sick_scan_srv::SCdevicestateSrv> m_srv_server_SCdevicestate; ///< service "SCdevicestate", &sick_scan::SickScanServices::serviceCbSCdevicestate
    rosServiceServer<sick_scan_srv::SCrebootSrv> m_srv_server_SCreboot; ///< service "SCreboot", &sick_scan::SickScanServices::serviceCbSCreboot
    rosServiceServer<sick_scan_srv::SCsoftresetSrv> m_srv_server_SCsoftreset; ///< service "SCsoftreset", &sick_scan::SickScanServices::serviceCbSCsoftreset
    rosServiceServer<sick_scan_srv::SickScanExitSrv> m_srv_server_SickScanExit; ///< service "SickScanExitSrv", &sick_scan::SickScanServices::serviceCbSickScanExit

  }; /* class SickScanServices */

} /* namespace sick_scan */
#endif /* SICK_SCAN_SERVICES_H_ */
