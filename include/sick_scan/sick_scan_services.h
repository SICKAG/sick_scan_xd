#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
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
#if defined SCANSEGMENT_XD_SUPPORT && SCANSEGMENT_XD_SUPPORT > 0
#include "sick_scansegment_xd/config.h"
#endif // SCANSEGMENT_XD_SUPPORT

namespace sick_scan_xd
{

  class SickScanServices
  {
  public:

    SickScanServices(rosNodePtr nh = 0, sick_scan_xd::SickScanCommonTcp* common_tcp = 0, ScannerBasicParam * lidar_param = 0);

    virtual ~SickScanServices();

    /*!
    * Sends the SOPAS authorization command "sMN SetAccessMode 3 F4724744".
    */
    bool sendAuthorization();

    /*!
     * Sends a multiScan or picoScan SOPAS command
     */
    bool sendSopasCmdCheckResponse(const std::string& sopas_request, const std::string& expected_response);

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
     * Callback for service messages (GetContaminationData, Read contamination indication data, LRS-4xxx only).
     * Sends a cola telegram "sRN ContaminationData" and receives the response from the lidar device.
     * @param[in] service_request ros service request to lidar
     * @param[out] service_response service response from lidar
     * @return true on success, false in case of errors.
     */
    bool serviceCbGetContaminationData(sick_scan_srv::GetContaminationDataSrv::Request &service_request, sick_scan_srv::GetContaminationDataSrv::Response &service_response);
    bool serviceCbGetContaminationDataROS2(std::shared_ptr<sick_scan_srv::GetContaminationDataSrv::Request> service_request, std::shared_ptr<sick_scan_srv::GetContaminationDataSrv::Response> service_response) { return serviceCbGetContaminationData(*service_request, *service_response); }

    /*!
     * Callback for service messages (GetContaminationResult, Read contamination indication result).
     * Sends a cola telegram "sRN ContaminationResult" and receives the response from the lidar device.
     * @param[in] service_request ros service request to lidar
     * @param[out] service_response service response from lidar
     * @return true on success, false in case of errors.
     */
    bool serviceCbGetContaminationResult(sick_scan_srv::GetContaminationResultSrv::Request &service_request, sick_scan_srv::GetContaminationResultSrv::Response &service_response);
    bool serviceCbGetContaminationResultROS2(std::shared_ptr<sick_scan_srv::GetContaminationResultSrv::Request> service_request, std::shared_ptr<sick_scan_srv::GetContaminationResultSrv::Response> service_response) { return serviceCbGetContaminationResult(*service_request, *service_response); }

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

#if __ROS_VERSION > 0
    bool serviceCbFieldSetRead(sick_scan_srv::FieldSetReadSrv::Request &service_request, sick_scan_srv::FieldSetReadSrv::Response &service_response);
    bool serviceCbFieldSetReadROS2(std::shared_ptr<sick_scan_srv::FieldSetReadSrv::Request> service_request, std::shared_ptr<sick_scan_srv::FieldSetReadSrv::Response> service_response) { return serviceCbFieldSetRead(*service_request, *service_response); }
    bool serviceCbFieldSetWrite(sick_scan_srv::FieldSetWriteSrv::Request &service_request, sick_scan_srv::FieldSetWriteSrv::Response &service_response);
    bool serviceCbFieldSetWriteROS2(std::shared_ptr<sick_scan_srv::FieldSetWriteSrv::Request> service_request, std::shared_ptr<sick_scan_srv::FieldSetWriteSrv::Response> service_response) { return serviceCbFieldSetWrite(*service_request, *service_response); }
#endif

#if defined SCANSEGMENT_XD_SUPPORT && SCANSEGMENT_XD_SUPPORT > 0
    /*!
     * Sends the multiScan start commands "sWN ScanDataFormat", "sWN ScanDataPreformatting", "sWN ScanDataEthSettings", "sWN ScanDataEnable 1", "sMN LMCstartmeas", "sMN Run"
     * @param[in] hostname IP address of multiScan136, default 192.168.0.1
     * @param[in] port IP port of multiScan136, default 2115
     * @param[in] scanner_type type of scanner, currently supported are multiScan136 and picoScan150
     * @param[in] scandataformat ScanDataFormat: 1 for msgpack or 2 for compact scandata, default: 1 
     * @param[in] imu_enable: Imu data transfer enabled
     * @param[in] imu_udp_port: UDP port of imu data (if imu_enable is true)
     * @param[in] check_udp_receiver_ip: check udp_receiver_ip by sending and receiving a udp test message
     * @param[in] check_udp_receiver_port: udp port to check udp_receiver_ip
     */
    bool sendMultiScanStartCmd(const std::string& hostname, int port, const std::string& scanner_type, int scandataformat, bool& imu_enable, int imu_udp_port, int performanceprofilenumber, bool check_udp_receiver_ip, int check_udp_receiver_port);

    /*!
     * Sends the multiScan stop commands "sWN ScanDataEnable 0" and "sMN Run"
     * @param[in] imu_enable: Imu data transfer enabled
     */
    bool sendMultiScanStopCmd(bool imu_enable);

    /*!
    * Sends the SOPAS command to query multiScan136 filter settings (FREchoFilter, LFPangleRangeFilter, host_LFPlayerFilter)
    * @param[out] host_FREchoFilter FREchoFilter settings, default: 1, otherwise 0 for FIRST_ECHO (EchoCount=1), 1 for ALL_ECHOS (EchoCount=3), or 2 for LAST_ECHO (EchoCount=1)
    * @param[out] host_LFPangleRangeFilter LFPangleRangeFilter settings, default: "0 -180.0 +180.0 -90.0 +90.0 1", otherwise "<enabled> <azimuth_start> <azimuth_stop> <elevation_start> <elevation_stop> <beam_increment>" with azimuth and elevation given in degree
    * @param[out] host_LFPlayerFilter LFPlayerFilter settings, default: "0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1", otherwise  "<enabled> <layer0-enabled> <layer1-enabled> <layer2-enabled> ... <layer15-enabled>" with 1 for enabled and 0 for disabled
    * @param[out] msgpack_validator_filter_settings; // filter settings for msgpack validator: required_echos, azimuth_start, azimuth_end. elevation_start, elevation_end, layer_filter
    */
    bool queryMultiScanFiltersettings(int& host_FREchoFilter, std::string& host_LFPangleRangeFilter, std::string& host_LFPlayerFilter, sick_scansegment_xd::MsgpackValidatorFilterConfig& msgpack_validator_filter_settings, const std::string& scanner_type);

    /*!
    * Sends the SOPAS command to write multiScan136 filter settings (FREchoFilter, LFPangleRangeFilter, host_LFPlayerFilter)
    * @param[in] host_FREchoFilter FREchoFilter settings, default: 1, otherwise 0 for FIRST_ECHO (EchoCount=1), 1 for ALL_ECHOS (EchoCount=3), or 2 for LAST_ECHO (EchoCount=1)
    * @param[in] host_LFPangleRangeFilter LFPangleRangeFilter settings, default: "0 -180.0 +180.0 -90.0 +90.0 1", otherwise "<enabled> <azimuth_start> <azimuth_stop> <elevation_start> <elevation_stop> <beam_increment>" with azimuth and elevation given in degree
    * @param[in] host_LFPlayerFilter LFPlayerFilter settings, default: "0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1", otherwise  "<enabled> <layer0-enabled> <layer1-enabled> <layer2-enabled> ... <layer15-enabled>" with 1 for enabled and 0 for disabled
    * @param[in] host_LFPintervalFilter Optionally set LFPintervalFilter to "<enabled> <N>" with 1 for enabled and 0 for disabled and N to reduce output to every N-th scan
    */
    bool writeMultiScanFiltersettings(int host_FREchoFilter, const std::string& host_LFPangleRangeFilter, const std::string& host_LFPlayerFilter, const std::string& host_LFPintervalFilter, const std::string& scanner_type);

#endif // defined SCANSEGMENT_XD_SUPPORT && SCANSEGMENT_XD_SUPPORT > 0

    /*!
     * Sends a sopas command and returns the lidar reply.
     * @param[in] sopasCmd sopas command to send, f.e. "sEN ECRChangeArr 1"
     * @param[out] sopasReplyBin response from lidar incl. start/stop byte
     * @param[out] sopasReplyString sopasReplyBin converted to string
     * @return true on success, false in case of errors.
     */
    bool sendSopasAndCheckAnswer(const std::string& sopasCmd, std::vector<unsigned char>& sopasReplyBin, std::string& sopasReplyString);

    /*!
    * Converts a hex string (hex_str: 4 byte hex value as string, little or big endian) to float.
    * Check f.e. by https://www.h-schmidt.net/FloatConverter/IEEE754.html
    * Examples:
    * convertHexStringToFloat("C0490FF9", true) returns -3.14
    * convertHexStringToFloat("3FC90FF9", true) returns +1.57
    */
    static float convertHexStringToFloat(const std::string& hex_str, bool hexStrIsBigEndian);

    /*!
    * Converts a float value to hex string (hex_str: 4 byte hex value as string, little or big endian).
    * Check f.e. by https://www.h-schmidt.net/FloatConverter/IEEE754.html
    * Examples:
    * convertFloatToHexString(-3.14, true) returns "C0490FDB"
    * convertFloatToHexString(+1.57, true) returns "3FC90FF8"
    */
    static std::string convertFloatToHexString(float value, bool hexStrIsBigEndian);

    /*!
    * Converts a hex string coded in 1/10000 deg (hex_str: 4 byte hex value as string, little or big endian) to an angle in [deg] (float).
    */
    static float convertHexStringToAngleDeg(const std::string& hex_str, bool hexStrIsBigEndian);

    /*!
    * Converts an angle in [deg] to hex string coded in 1/10000 deg (hex_str: 4 byte hex value as string, little or big endian).
    */
    static std::string convertAngleDegToHexString(float angle_deg, bool hexStrIsBigEndian);

  protected:

    /*!
    * Sends the SOPAS command "sMN Run", which applies previous send settings
    */
    bool sendRun();

    /*
     * Member data
     */

    bool m_cola_binary;                             ///< cola ascii or cola binary messages
    sick_scan_xd::SickScanCommonTcp* m_common_tcp;     ///< common tcp handler
    std::string m_user_level_password;
    int m_user_level;
    rosServiceServer<sick_scan_srv::ColaMsgSrv> m_srv_server_ColaMsg;        ///< service "ColaMsg", &sick_scan::SickScanServices::serviceCbColaMsg
    rosServiceServer<sick_scan_srv::ECRChangeArrSrv> m_srv_server_ECRChangeArr;   ///< service "ECRChangeArr", &sick_scan::SickScanServices::serviceCbECRChangeArr
    rosServiceServer<sick_scan_srv::GetContaminationDataSrv> m_srv_server_GetContaminationData; ///< service "GetContaminationData", &sick_scan::SickScanServices::serviceCbGetContaminationData (LRS-4xxx only)
    rosServiceServer<sick_scan_srv::GetContaminationResultSrv> m_srv_server_GetContaminationResult; ///< service "GetContaminationResult", &sick_scan::SickScanServices::serviceCbGetContaminationResult
    rosServiceServer<sick_scan_srv::LIDoutputstateSrv> m_srv_server_LIDoutputstate; ///< service "LIDoutputstate", &sick_scan::SickScanServices::serviceCbLIDoutputstate
    rosServiceServer<sick_scan_srv::SCdevicestateSrv> m_srv_server_SCdevicestate; ///< service "SCdevicestate", &sick_scan::SickScanServices::serviceCbSCdevicestate
    rosServiceServer<sick_scan_srv::SCrebootSrv> m_srv_server_SCreboot; ///< service "SCreboot", &sick_scan::SickScanServices::serviceCbSCreboot
    rosServiceServer<sick_scan_srv::SCsoftresetSrv> m_srv_server_SCsoftreset; ///< service "SCsoftreset", &sick_scan::SickScanServices::serviceCbSCsoftreset
    rosServiceServer<sick_scan_srv::SickScanExitSrv> m_srv_server_SickScanExit; ///< service "SickScanExitSrv", &sick_scan::SickScanServices::serviceCbSickScanExit
#if __ROS_VERSION > 0
    rosServiceServer<sick_scan_srv::FieldSetReadSrv> m_srv_server_FieldSetRead ;  ///< service "FieldSetReadSrv", &sick_scan::SickScanServices::serviceCbFieldSetRead
    rosServiceServer<sick_scan_srv::FieldSetWriteSrv> m_srv_server_FieldSetWrite; ///< service "FieldSetWriteSrv", &sick_scan::SickScanServices::serviceCbFieldSetWrite
#endif

  }; /* class SickScanServices */

} /* namespace sick_scan_xd */
#endif /* SICK_SCAN_SERVICES_H_ */
