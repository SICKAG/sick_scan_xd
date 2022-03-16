/*
 * Copyright (C) 2013, Osnabrueck University
 * Copyright (C) 2017, Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2017, SICK AG, Waldkirch
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
 *  Created on: 24.05.2012
 *
 *      Authors:
 *         Jochen Sprickerhof <jochen@sprickerhof.de>
 *         Martin Günther <mguenthe@uos.de>
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 * Based on the TiM communication example by SICK AG.
 *
 */

#ifndef SICK_SCAN_COMMON_H_
#define SICK_SCAN_COMMON_H_

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <vector>

#include <sick_scan/sick_ros_wrapper.h>

#include <thread>
#include <mutex>

#include <sick_scan/sick_scan_common_nw.h>
#include <sick_scan/helper/angle_compensator.h>

#include "sick_scan/sick_generic_parser.h"
#include "sick_scan/sick_scan_common_nw.h"

#include "sick_scan/sick_generic_field_mon.h"
#include "sick_scan/sick_scan_marker.h"

/*
** Default timeouts
*/
#define READ_TIMEOUT_MILLISEC_STARTUP   120000 // 120 sec read timeout during startup (sensor may be starting up, which can take up to 120 sec.) // TODO: move timeout to config
#define READ_TIMEOUT_MILLISEC_DEFAULT     5000 // 5 sec read timeout in operational mode (measurement mode) // TODO: move timeout to config
#define READ_TIMEOUT_MILLISEC_KILL_NODE 150000 // 150 sec pointcloud timeout, ros node will be killed if no point cloud published within the last 150 sec. // TODO: move timeout to config

void swap_endian(unsigned char *ptr, int numBytes);

namespace sick_scan
{

  class SickScanCommon
  {
  public:
    enum SOPAS_CMD
    {
      CMD_DEVICE_IDENT_LEGACY,
      CMD_DEVICE_IDENT,  // for MRS6124
      CMD_SERIAL_NUMBER,
      CMD_REBOOT,
      CMD_WRITE_EEPROM,
      CMD_FIRMWARE_VERSION,
      CMD_DEVICE_STATE,
      CMD_OPERATION_HOURS,
      CMD_POWER_ON_COUNT,
      CMD_LOCATION_NAME,
      CMD_ACTIVATE_STANDBY,
      CMD_SET_PARTICLE_FILTER,
      CMD_SET_MEAN_FILTER,
      CMD_ALIGNMENT_MODE,
      CMD_APPLICATION_MODE,
      CMD_APPLICATION_MODE_FIELD_ON,
      CMD_APPLICATION_MODE_FIELD_OFF,
      CMD_APPLICATION_MODE_RANGING_ON,
      CMD_SET_ACCESS_MODE_3,
      CMD_SET_ACCESS_MODE_3_SAFETY_SCANNER,
      CMD_SET_OUTPUT_RANGES,
      CMD_SET_OUTPUT_RANGES_NAV3,
      CMD_GET_OUTPUT_RANGES,
      CMD_RUN,
      CMD_SET_PARTIAL_SCAN_CFG,
      CMD_GET_PARTIAL_SCAN_CFG,
      CMD_GET_PARTIAL_SCANDATA_CFG,
      CMD_SET_PARTIAL_SCANDATA_CFG,
      CMD_STOP_SCANDATA,
      CMD_START_SCANDATA,
      CMD_START_RADARDATA,
      CMD_ACTIVATE_NTP_CLIENT,
      CMD_SET_NTP_INTERFACE_ETH,
      CMD_SET_ENCODER_MODE,
      CMD_SET_ENCODER_MODE_NO,
      CMD_SET_ENCODER_MODE_SI,
      CMD_SET_ENCODER_MODE_DP,
      CMD_SET_ENCODER_MODE_DL,
      CMD_SET_INCREMENTSOURCE_ENC,
      CMD_SET_3_4_TO_ENCODER,

      CMD_SET_ENOCDER_RES_1,
      CMD_SET_ENCODER_RES,

      CMD_START_IMU_DATA, // start of IMU data
      CMD_STOP_IMU_DATA, // start of IMU data

      // start of radar specific commands
      CMD_SET_TRANSMIT_RAWTARGETS_ON,  // transmit raw target for radar
      CMD_SET_TRANSMIT_RAWTARGETS_OFF, // do not transmit raw target for radar

      CMD_SET_TRANSMIT_OBJECTS_ON,  // transmit raw target for radar
      CMD_SET_TRANSMIT_OBJECTS_OFF, // do not transmit raw target for radar

      CMD_SET_TRACKING_MODE_0,  // set radar tracking mode to "BASIC"
      CMD_SET_TRACKING_MODE_1,  // set radar tracking mode to "TRAFFIC"

      CMD_LOAD_APPLICATION_DEFAULT, // load application default
      CMD_DEVICE_TYPE,
      CMD_ORDER_NUMBER,
      // end of radar specific commands
      CMD_START_MEASUREMENT,
      CMD_STOP_MEASUREMENT,
      CMD_SET_ECHO_FILTER,
      CMD_SET_NTP_UPDATETIME,
      CMD_SET_NTP_TIMEZONE,
      CMD_SET_IP_ADDR,
      CMD_SET_GATEWAY,
      CMD_SET_NTP_SERVER_IP_ADDR,
      CMD_SET_SCANDATACONFIGNAV, // "sMN mLMPsetscancfg ..."
      CMD_GET_SCANDATACONFIGNAV, // "sRN LMPscancfg"
      CMD_SEN_SCANDATACONFIGNAV, // "sEN LMPscancfg 1"
      CMD_GET_ANGLE_COMPENSATION_PARAM, // Angle Compensation Parameter for NAV lidar
      CMD_SET_TO_COLA_A_PROTOCOL,  //		sWN EIHstCola 1  // Cola B 	sWN EIHstCola 0  // Cola A
      CMD_SET_TO_COLA_B_PROTOCOL,  //
      CMD_GET_SAFTY_FIELD_CFG,// gets the safty fields cfg olny tim 7xxs supported at the moment

      CMD_SET_LFEREC_ACTIVE,       // activate LFErec messages, send "sEN LFErec 1"
      CMD_SET_LID_OUTPUTSTATE_ACTIVE,  // activate LIDoutputstate messages, send "sEN LIDoutputstate 1"
      CMD_SET_LID_INPUTSTATE_ACTIVE,  // activate LIDinputstate messages, send "sEN LIDinputstate 1"
      CMD_SET_SCAN_CFG_LIST, // "sMN mCLsetscancfglist %d", set scan config from list for NAX310  LD-OEM15xx LD-LRS36xx

      // ML: Add above new CMD-Identifier
      //
      //
      CMD_END // CMD_END is a tag for end of enum - never (re-)move it. It must be the last element.
    };
// --- START KEYWORD DEFINITIONS ---
#define PARAM_MIN_ANG "min_ang"
#define PARAM_MAX_ANG "max_ang"
#define PARAM_RES_ANG "res_ang"
// --- END KEYWORD DEFINITIONS ---


    SickScanCommon(rosNodePtr nh, SickGenericParser *parser);

    virtual ~SickScanCommon();

    int setParticleFilter(bool _active, int _particleThreshold);//actualy only 500 mm is working.
    /*! Changes the Identifier of a commandstr. to its expected answer counterpart
     *
     * @param requestStr sent request string
     * @return Expected answer
     */
    std::vector<std::string> generateExpectedAnswerString(const std::vector<unsigned char> requestStr);

    int sendSopasAndCheckAnswer(std::string request, std::vector<unsigned char> *reply, int cmdId);

    int sendSopasAndCheckAnswer(std::vector<unsigned char> request, std::vector<unsigned char> *reply, int cmdId);

    int sendSopasAorBgetAnswer(const std::string& request, std::vector<unsigned char> *reply, bool useBinaryCmd);

    ExitCode checkColaTypeAndSwitchToConfigured(bool useBinaryCmd);

    int setAligmentMode(int _AligmentMode);

    int setMeanFilter(bool _active, int _numberOfScans);

    int setApplicationMode(bool _active, int _mode); //0=RANG (Ranging) 1=FEVL (Field Application).
    int ActivateStandBy(void);

    bool testSettingIpAddress();

    bool testsetParticleFilter();

    bool testsetMeanFilter();

    bool testsetAligmentMode();

    bool testsetActivateStandBy();

    bool testsetApplicationMode();

    int getReadTimeOutInMs();

    void setReadTimeOutInMs(int timeOutInMs);

    int getProtocolType(void);

    void setProtocolType(SopasProtocol cola_dialect_id);

    virtual int init(rosNodePtr nh);

    int loopOnce(rosNodePtr nh);

    void check_angle_range(SickScanConfig &conf);

    void update_config(sick_scan::SickScanConfig &new_config, uint32_t level = 0);

#if defined USE_DYNAMIC_RECONFIGURE && __ROS_VERSION == 2
    rcl_interfaces::msg::SetParametersResult update_config_cb(const std::vector<rclcpp::Parameter> &parameters);
#endif

    double get_expected_frequency() const
    { return expectedFrequency_; }

    int convertAscii2BinaryCmd(const char *requestAscii, std::vector<unsigned char> *requestBinary);

    int init_cmdTables(rosNodePtr nh);

    /// Send a SOPAS command to the scanner that should cause a soft reset
    /**
     * \returns true if reboot command was accepted, false otherwise
     */
    virtual bool rebootScanner();

    /// Send a SOPAS command to the scanner that logs in the authorized client, changes the ip adress and the reboots the scanner
    /**
     * \param IpAdress new IP adress
     * \returns true if ip was changed and scanner is rebooting
     */
    bool changeIPandreboot(const std::string& IpAdress);

    SickScanCommonNw m_nw;

    SickScanConfig *getConfigPtr()
    {
      return (&config_);
    }

    /// Converts reply from sendSOPASCommand to string
    /**
     * \param [in] reply reply from sendSOPASCommand
     * \returns reply as string with special characters stripped out
     */
    std::string sopasReplyToString(const std::vector<unsigned char> &reply)
    {
      return replyToString(reply);
    }

    uint64_t getNanosecTimestampLastTcpMessageReceived(void) { return m_nw.getNanosecTimestampLastTcpMessageReceived(); } // Returns a timestamp in nanoseconds of the last received tcp message (or 0 if no message received)

    // move back to private
    /* FÜR MRS10000 brauchen wir einen Publish und eine NAchricht */
    // Should we publish laser or point cloud?
    // ros::Publisher cloud_pub_;
    rosPublisher<ros_sensor_msgs::PointCloud2> cloud_pub_;
    rosPublisher<ros_sensor_msgs::Imu> imuScan_pub_;
    rosPublisher<sick_scan_msg::Encoder> Encoder_pub;
    // sensor_msgs::PointCloud cloud_;
    ros_sensor_msgs::PointCloud2 cloud_;
    //////
    // Dynamic Reconfigure
    SickScanConfig config_;
  protected:
    virtual int init_device() = 0;

    virtual int init_scanner(rosNodePtr nh);

    virtual int stop_scanner(bool force_immediate_shutdown = false);

    virtual int close_device() = 0;

    /// Send a SOPAS command to the device and print out the response to the console.
    /**
     * \param [in] request the command to send.
     * \param [out] reply if not NULL, will be filled with the reply package to the command.
     * \param [in] cmdLen Length of the Comandstring in bytes used for Binary Mode only
     */
    virtual int sendSOPASCommand(const char *request, std::vector<unsigned char> *reply, int cmdLen, bool wait_for_reply = true) = 0;

    /// Converts a given SOPAS command from ascii to binary (in case of binary communication), sends sopas (ascii or binary) and returns the response (if wait_for_reply:=true)
    /**
     * \param [in] request the command to send.
     * \param [in] cmdLen Length of the Comandstring in bytes used for Binary Mode only
     */
    virtual int convertSendSOPASCommand(const std::string& sopas_ascii_request, std::vector<unsigned char> *reply, bool wait_for_reply = true);

    virtual int readWithTimeout(size_t timeout_ms, char *buffer, int buffer_size, int *bytes_read, const std::vector<std::string>& datagram_keywords) = 0;

    /// Read a datagram from the device.
    /**
     * \param [out] recvTimeStamp timestamp of received packet
     * \param [in] receiveBuffer data buffer to fill
     * \param [in] bufferSize max data size to write to buffer (result should be 0 terminated)
     * \param [out] actual_length the actual amount of data written
     * \param [in] isBinaryProtocol used Communication protocol True=Binary false=ASCII
     * \param [in] datagram_keywords keyword in returned datagram, e.g. { "LMDscandata" } to get scandata telegrams, or {} (empty vector) for next received datagram
     */
    virtual int get_datagram(rosNodePtr nh, rosTime &recvTimeStamp, unsigned char *receiveBuffer, int bufferSize, int *actual_length,
                             bool isBinaryProtocol, int *numberOfRemainingFifoEntries, const std::vector<std::string>& datagram_keywords) = 0;

    /// Converts reply from sendSOPASCommand to string
    /**
     * \param [in] reply reply from sendSOPASCommand
     * \returns reply as string with special characters stripped out
     */
    std::string replyToString(const std::vector<unsigned char> &reply);

    /**
    * \param [in] *vecArr to (unsigned) char buffer in big endian byte oder (MSB first)
    *
    * \returns    unsigned long value as interpretation of big endian long value
    */
    unsigned long convertBigEndianCharArrayToUnsignedLong(const unsigned char *vecArr);

    /**
    * \param [in] reply check reply whether is SOPAS-ASCII or SOPAS-Binary
    *
    * \returns    -1 if ascii otherwise the length of data content following offset 8
    */
    int checkForBinaryAnswer(const std::vector<unsigned char> *reply);

    /*!
    \brief check the identification string
    \param identStr string (got from sopas request)
    \return true, if this driver supports the scanner identified by the identification string
    */
    bool isCompatibleDevice(const std::string identStr) const;

    bool dumpDatagramForDebugging(unsigned char *buffer, int bufLen);

#ifdef USE_DIAGNOSTIC_UPDATER
    std::shared_ptr<diagnostic_updater::Updater> diagnostics_;
#endif

  private:
    SopasProtocol m_protocolId;
    // ROS
    rosPublisher<ros_sensor_msgs::LaserScan> pub_;
    rosPublisher<ros_std_msgs::String> datagram_pub_;
    bool publish_datagram_;

    rosPublisher<sick_scan_msg::LFErecMsg> lferec_pub_;
    bool publish_lferec_;
    rosPublisher<sick_scan_msg::LIDoutputstateMsg> lidoutputstate_pub_;
    bool publish_lidoutputstate_;
    SickScanMarker* cloud_marker_;

    // Diagnostics
#if defined USE_DIAGNOSTIC_UPDATER
#if __ROS_VERSION == 1
    diagnostic_updater::DiagnosedPublisher<ros_sensor_msgs::LaserScan> *diagnosticPub_;
#elif __ROS_VERSION == 2
    DiagnosedPublishAdapter<rosPublisher<ros_sensor_msgs::LaserScan>> *diagnosticPub_;
#else
    uint8_t* diagnosticPub_; // always 0
#endif
#else
    uint8_t* diagnosticPub_; // always 0
#endif
    double expectedFrequency_;


#if defined USE_DYNAMIC_RECONFIGURE && __ROS_VERSION == 1
    dynamic_reconfigure::Server<sick_scan::SickScanConfig> dynamic_reconfigure_server_;
#endif
    // Parser
    SickGenericParser *parser_;
    std::vector<std::string> sopasCmdVec;
    std::vector<std::string> sopasCmdMaskVec;
    std::vector<std::string> sopasReplyVec;
    std::vector<std::vector<unsigned char> > sopasReplyBinVec;
    std::vector<std::string> sopasReplyStrVec;
    std::vector<std::string> sopasCmdErrMsg;
    std::vector<int> sopasCmdChain;

    int outputChannelFlagId;

    bool checkForProtocolChangeAndMaybeReconnect(bool &useBinaryCmdNow);

    void setSensorIsRadar(bool _isRadar);

    bool getSensorIsRadar(void);

    bool setNewIpAddress(const std::string& ipNewIPAddr, bool useBinaryCmd);

    bool setNTPServerAndStart(const std::string& ipNewIPAddr, bool useBinaryCmd);

    int readTimeOutInMs;

    std::mutex sopasSendMutex; // mutex to lock sendSopasAndCheckAnswer

  private:
    bool sensorIsRadar;

    AngleCompensator *angleCompensator = NULL;

    double m_min_intensity; // Set range of LaserScan messages to infinity, if intensity < min_intensity (default: 0)

    //void getConfigUpdateParam(SickScanConfig & cfg);
    //void setConfigUpdateParam(SickScanConfig & cfg);

    int m_read_timeout_millisec_default;
    int m_read_timeout_millisec_startup;

    rosNodePtr m_nh;
  };

} /* namespace sick_scan */
#endif /* SICK_TIM3XX_COMMON_H_ */
