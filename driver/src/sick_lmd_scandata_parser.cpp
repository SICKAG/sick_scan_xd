/*
 * sick_lmd_scandata_parser parses result telegrams of type LMDscandata.
 *
 * Copyright (C) 2022, Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2022, SICK AG, Waldkirch
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
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 */

#include "softwarePLL.h"
#include <sick_scan/sick_scan_common.h>
#include <sick_scan/sick_lmd_scandata_parser.h>

#define deg2rad_const (0.017453292519943295769236907684886f)

namespace sick_scan_xd
{

    /** Increments the number of packets received in the SoftwarePLL */
    void incSoftwarePLLPacketReceived()
    {
      SoftwarePLL::instance().packets_received++;
      if (SoftwarePLL::instance().IsInitialized() == false)
      {
        if(SoftwarePLL::instance().packets_received <= 1)
        {
          ROS_INFO("Software PLL locking started, mapping ticks to system time.");
        }
        int packets_expected_to_drop = SoftwarePLL::instance().fifoSize - 1;
        SoftwarePLL::instance().packets_dropped++;
        size_t packets_dropped = SoftwarePLL::instance().packets_dropped;
        size_t packets_received = SoftwarePLL::instance().packets_received;
        if (packets_dropped < packets_expected_to_drop)
        {
          ROS_INFO_STREAM("" << packets_dropped << " / " << packets_expected_to_drop << " packets dropped. Software PLL not yet locked.");
        }
        else if (packets_dropped == packets_expected_to_drop)
        {
          ROS_INFO("Software PLL is ready and locked now!");
        }
        else if (packets_dropped > packets_expected_to_drop && packets_received > 0)
        {
          double drop_rate = (double)packets_dropped / (double)packets_received;
          ROS_WARN_STREAM("" << SoftwarePLL::instance().packets_dropped << " of " << SoftwarePLL::instance().packets_received << " packets dropped ("
            << std::fixed << std::setprecision(1) << (100*drop_rate) << " perc.), maxAbsDeltaTime=" << std::fixed << std::setprecision(3) << SoftwarePLL::instance().max_abs_delta_time);
          ROS_WARN_STREAM("More packages than expected were dropped!!\n"
                  "Check the network connection.\n"
                  "Check if the system time has been changed in a leap.\n"
                  "If the problems can persist, disable the software PLL with the option sw_pll_only_publish=False  !");
        }
      }
    }

    /** check angle values against +/- pi. 
        It is checked whether the angle is close to +pi or -pi. 
        In this case, the angle is minimally modified so that 
        the modified angle is safely within the interval [-pi,pi] 
        to avoid problems with angle wrapping. 
        If the angle value is modified, the function returns true else false.
      */
    bool check_near_plus_minus_pi(float *angle_val)
    {
      bool angle_slightly_modified = false;
      float pi_multiplier = *angle_val/M_PI;
      // float check_deviation_to_abs_one = fabs(pi_multiplier) - 1.0;
      // check for a small deviation
      // if (check_deviation_to_abs_one < 10.0 * FLT_EPSILON )
      if (pi_multiplier > 1.1 || pi_multiplier < -1.1)
      {
        ROS_WARN_STREAM("check_near_plus_minus_pi: min or max angle = " << *angle_val * 180 / M_PI << " degree, expected angle within -180 to +180 degree, check scan angle shift settings.");
      }
      else if (pi_multiplier > 1.0 - 10.0 * FLT_EPSILON || pi_multiplier < -1.0 + 10.0 * FLT_EPSILON)
      {
        float factor =  (*angle_val < 0.0) ? (-1.0) : (1.0);
        *angle_val = factor * (1.0 - FLT_EPSILON) * M_PI;
        angle_slightly_modified = true;  
      }
      else
      {
        angle_slightly_modified =false;
      }
      return(angle_slightly_modified);
    }

  /**
   * @brief Configures the angle parameters of a LaserScan message based on scanner metadata.
   *
   * This function calculates and sets the `angle_min`, `angle_max`, and `angle_increment` values
   * of a `sensor_msgs::LaserScan` message. It also handles scanner-specific adjustments such as
   * angle mirroring, special cases for known scanner models, and the time increment for scan points.
   *
   * @param[out] msg                        The LaserScan message to populate with angle parameters.
   * @param[in]  startAngle                 The start angle of the scan in degrees.
   * @param[in]  sizeOfSingleAngularStep    The angular resolution (step size) in degress.
   * @param[in]  numberOfItems              The number of scan points (data items) in the scan.
   * @param[in]  parser                     Pointer to the parser providing scanner-specific parameters.
   */
    void configureAngleParameters(ros_sensor_msgs::LaserScan& msg,
      double startAngle,
      double sizeOfSingleAngularStep,
      int numberOfItems,
      SickGenericParser* parser)
    {
      auto params = parser->getCurrentParamPtr();

      // Basic angle setup
      msg.angle_min = startAngle / 180.0 * M_PI + params->getScanAngleShift();
      msg.angle_increment = sizeOfSingleAngularStep / 180.0 * M_PI;
      msg.angle_max = msg.angle_min + (numberOfItems - 1) * msg.angle_increment;

      // Handle special time increment case (e.g., NAV-350)
      if (msg.time_increment == 0)
      {
        msg.time_increment = fabs(params->getNumberOfLayers() * msg.scan_time * msg.angle_increment / (2.0 * M_PI));
      }

      const std::string& scannerName = params->getScannerName();

      // Adjust angles for specific scanner models
      if (scannerName == SICK_SCANNER_NAV_31X_NAME ||
        scannerName == SICK_SCANNER_LRS_36x0_NAME ||
        scannerName == SICK_SCANNER_OEM_15XX_NAME)
      {
        msg.angle_min = static_cast<float>(-M_PI);
        msg.angle_max = static_cast<float>(+M_PI);
        msg.angle_increment *= -1.0;

        if (msg.angle_increment < 0.0)
        {
          // Ensure logical angle order when increment is negative
          msg.angle_min = static_cast<float>(+M_PI);
          msg.angle_max = static_cast<float>(-M_PI);
        }
      }
      else if (params->getScanMirroredAndShifted())
      {
        // Mirror the scan angles if required
        msg.angle_min *= -1.0;
        msg.angle_increment *= -1.0;
        msg.angle_max *= -1.0;
      }

      ROS_DEBUG_STREAM("process_dist: msg.angle_min=" << (msg.angle_min * 180.0 / M_PI)
        << ", msg.angle_max=" << (msg.angle_max * 180.0 / M_PI)
        << ", msg.angle_increment=" << (msg.angle_increment * 180.0 / M_PI));
      // << ", scaleFactor=" << scaleFactor
      // << ", scaleFactorOffset=" << scaleFactorOffset);

      // Wrap-around avoidance
      bool wrapAvoid = false;
      if (check_near_plus_minus_pi(&(msg.angle_min)) ||
        check_near_plus_minus_pi(&(msg.angle_max)))
      {
        wrapAvoid = true;
      }

      if (wrapAvoid)
      {
        msg.angle_increment = (msg.angle_max - msg.angle_min) / (numberOfItems - 1);
      }
    }


    /** Parse common result telegrams, i.e. parse telegrams of type LMDscandata received from the lidar */
    bool parseCommonBinaryResultTelegram(const uint8_t* receiveBuffer, int receiveBufferLength, short& elevAngleX200, double elevAngleTelegramValToDeg, double& elevationAngleInRad, rosTime& recvTimeStamp,
        bool config_sw_pll_only_publish, bool use_generation_timestamp, SickGenericParser* parser_, bool& FireEncoder, sick_scan_msg::Encoder& EncoderMsg, int& numEchos, 
        std::vector<float>& vang_vec, std::vector<float>& azimuth_vec, ros_sensor_msgs::LaserScan & msg)
    {
                  // bool lms1000_debug = true; // LMS-1000 diagnosis
                  elevAngleX200 = 0;  // signed short (F5 B2  -> Layer 24
                  // F5B2h -> -2638/200= -13.19°
                  int scanFrequencyX100 = 0;
                  double scanFrequency = 0.0;
                  long measurementFrequencyDiv100 = 0; // multiply with 100
                  int numOfEncoders = 0;
                  int numberOf16BitChannels = 0;
                  int numberOf8BitChannels = 0;
                  uint32_t SystemCountScan = 0;
                  static uint32_t lastSystemCountScan = 0;// this variable is used to ensure that only the first time stamp of an multi layer scann is used for PLL updating
                  uint32_t SystemCountTransmit = 0;
                  static uint32_t lastSystemCountTransmit = 0;

                  memcpy(&elevAngleX200, receiveBuffer + 50, 2);
                  swap_endian((unsigned char *) &elevAngleX200, 2);

                  // elevationAngleInRad = -elevAngleX200 / 200.0 * deg2rad_const;
                  elevationAngleInRad = -elevAngleX200 * elevAngleTelegramValToDeg * deg2rad_const; // MRS-6000: elevAngleTelegramValToDeg=1./200, MRS-1000: elevAngleTelegramValToDeg=1./100
                  // ROS_INFO_STREAM("LMDscandata: elevAngleX200=" << elevAngleX200 << " / " << (1/elevAngleTelegramValToDeg) << "  = " << (elevationAngleInRad / deg2rad_const) << " [deg]");
                  ROS_HEADER_SEQ(msg.header, elevAngleX200); // should be multiple of 0.625° starting with -2638 (corresponding to 13.19°)

                  // Time since start up in microseconds: Counting the time since power up the device; starting with 0. In the output telegram this is the time at the zero index before the measurement itself starts.
                  memcpy(&SystemCountScan, receiveBuffer + 0x26, 4); // 0x26 = 38 dec
                  swap_endian((unsigned char *) &SystemCountScan, 4);

                  // Time of transmission in microseconds: Time in μs when the complete scan is transmitted to the buffer for data output; starting with 0 at scanner bootup.
                  memcpy(&SystemCountTransmit, receiveBuffer + 0x2A, 4);
                  swap_endian((unsigned char *) &SystemCountTransmit, 4);

                  /*{
                    double system_count_scan_sec = SystemCountScan * 1e-6;
                    double system_count_transmit_sec = SystemCountTransmit * 1e-6;
                    ROS_INFO_STREAM("LMDscandata: SystemCountScan = " << system_count_scan_sec << " sec, SystemCountTransmit = " << system_count_transmit_sec << " sec, delta = " << 1.0e3*(system_count_transmit_sec - system_count_scan_sec) << " millisec.");
                  }*/
                  /* if(SystemCountScan < lastSystemCountScan)
                  {
                    ROS_WARN_STREAM("parseCommonBinaryResultTelegram(): SystemCountScan=" << SystemCountScan << " < lastSystemCountScan=" << lastSystemCountScan << ", SystemCountScan jump back");
                  }
                  if(SystemCountTransmit < lastSystemCountTransmit)
                  {
                    ROS_WARN_STREAM("parseCommonBinaryResultTelegram(): SystemCountTransmit=" << SystemCountTransmit << " < lastSystemCountTransmit=" << lastSystemCountTransmit << ", SystemCountTransmit jump back");
                  } */
                  uint32_t lidar_ticks = SystemCountScan;
                  if(use_generation_timestamp == 0)
                    lidar_ticks = SystemCountTransmit;
                  double timestampfloat = sec(recvTimeStamp) + nsec(recvTimeStamp) * 1e-9;
                  bool bRet;
                  if (SystemCountScan !=
                      lastSystemCountScan)// MRS 6000 sends 6 packets with same  SystemCountScan we should only update the pll once with this time stamp since the SystemCountTransmit are different and this will only increase jitter of the pll
                  {
                    bRet = SoftwarePLL::instance().updatePLL(sec(recvTimeStamp), nsec(recvTimeStamp), SystemCountTransmit, lidar_ticks);
                    lastSystemCountScan = SystemCountScan;
                  }
                  lastSystemCountTransmit = SystemCountTransmit;
                  // ROS_DEBUG_STREAM("recvTimeStamp before software-pll correction: " << recvTimeStamp);
                  rosTime tmp_time = recvTimeStamp;
                  uint32_t recvTimeStampSec = (uint32_t)sec(recvTimeStamp), recvTimeStampNsec = (uint32_t)nsec(recvTimeStamp);
                  bRet = SoftwarePLL::instance().getCorrectedTimeStamp(recvTimeStampSec, recvTimeStampNsec, lidar_ticks);

                  recvTimeStamp = rosTime(recvTimeStampSec, recvTimeStampNsec);
                  double timestampfloat_coor = sec(recvTimeStamp) + nsec(recvTimeStamp) * 1e-9;
                  double DeltaTime = timestampfloat - timestampfloat_coor;
                  // ROS_DEBUG_STREAM("recvTimeStamp after software-pll correction: " << recvTimeStamp);
                  //ROS_INFO("%F,%F,%u,%u,%F",timestampfloat,timestampfloat_coor,SystemCountTransmit,SystemCountScan,DeltaTime);
                  //TODO Handle return values
                  if (config_sw_pll_only_publish == true)
                  {
                    incSoftwarePLLPacketReceived();
                  }

#ifdef DEBUG_DUMP_ENABLED
                  double elevationAngleInDeg = elevationAngleInRad / deg2rad_const;
                  // DataDumper::instance().pushData((double)SystemCountScan, "LAYER", elevationAngleInDeg);
                  //DataDumper::instance().pushData((double)SystemCountScan, "LASESCANTIME", SystemCountScan);
                  //DataDumper::instance().pushData((double)SystemCountTransmit, "LASERTRANSMITTIME", SystemCountTransmit);
                  //DataDumper::instance().pushData((double)SystemCountScan, "LASERTRANSMITDELAY", debug_duration.toSec());
#endif

                  // byte 48 + 49: output status (0 0)
                  // byte 50 + 51: reserved

                  memcpy(&scanFrequencyX100, receiveBuffer + 52, 4);
                  swap_endian((unsigned char *) &scanFrequencyX100, 4);

                  memcpy(&measurementFrequencyDiv100, receiveBuffer + 56, 4);
                  swap_endian((unsigned char *) &measurementFrequencyDiv100, 4);


                  msg.scan_time = 1.0 / (scanFrequencyX100 / 100.0);

                  //due firmware inconsistency
                  if (measurementFrequencyDiv100 > 10000)
                  {
                    measurementFrequencyDiv100 /= 100;
                  }
                  if (measurementFrequencyDiv100 != 0)
                  {
                    msg.time_increment = 1.0 / (measurementFrequencyDiv100 * 100.0);
                  }
                  else
                  {
                    msg.time_increment = 0;
                  }
                  // timeIncrement = msg.time_increment;
                  msg.range_min = parser_->get_range_min();
                  msg.range_max = parser_->get_range_max();

                  memcpy(&numOfEncoders, receiveBuffer + 60, 2);
                  swap_endian((unsigned char *) &numOfEncoders, 2);
                  // ROS_DEBUG_STREAM("numOfEncoders = " << numOfEncoders);
                  int encoderDataOffset = 6 * numOfEncoders;
                  int32_t EncoderPosTicks[4] = {0};
                  int16_t EncoderSpeed[4] = {0};

                  if (numOfEncoders > 0 && numOfEncoders < 5)
                  {
                    FireEncoder = true;
                    for (int EncoderNum = 0; EncoderNum < numOfEncoders; EncoderNum++)
                    {
                      memcpy(&EncoderPosTicks[EncoderNum], receiveBuffer + 62 + EncoderNum * 6, 4);
                      swap_endian((unsigned char *) &EncoderPosTicks[EncoderNum], 4);
                      memcpy(&EncoderSpeed[EncoderNum], receiveBuffer + 66 + EncoderNum * 6, 2);
                      swap_endian((unsigned char *) &EncoderSpeed[EncoderNum], 2);
                    }
                  }
                  //TODO handle multi encoder with multiple encode msg or different encoder msg definition now using only first encoder
                  EncoderMsg.enc_position = EncoderPosTicks[0];
                  EncoderMsg.enc_speed = EncoderSpeed[0];
                  memcpy(&numberOf16BitChannels, receiveBuffer + 62 + encoderDataOffset, 2);
                  swap_endian((unsigned char *) &numberOf16BitChannels, 2);

                  int parseOff = 64 + encoderDataOffset;


                  char szChannel[255] = {0};
                  float scaleFactor = 1.0;
                  float scaleFactorOffset = 0.0;
                  int32_t startAngleDiv10000 = 1;
                  int32_t sizeOfSingleAngularStepDiv10000 = 1;
                  double startAngle = 0.0;
                  double sizeOfSingleAngularStep = 0.0;
                  short numberOfItems = 0;

                  //static int cnt = 0;
                  //cnt++;
                  // get number of 8 bit channels
                  // we must jump of the 16 bit data blocks including header ...
                  for (int i = 0; i < numberOf16BitChannels; i++)
                  {
                    int numberOfItems = 0x00;
                    memcpy(&numberOfItems, receiveBuffer + parseOff + 19, 2);
                    swap_endian((unsigned char *) &numberOfItems, 2);
                    parseOff += 21; // 21 Byte header followed by data entries
                    parseOff += numberOfItems * 2;
                  }

                  // now we can read the number of 8-Bit-Channels
                  memcpy(&numberOf8BitChannels, receiveBuffer + parseOff, 2);
                  swap_endian((unsigned char *) &numberOf8BitChannels, 2);

                  parseOff = 64 + encoderDataOffset;
                  enum datagram_parse_task
                  {
                    process_dist,
                    process_vang,
                    process_azimuth,
                    process_rssi,
                    process_idle
                  };
                  int rssiCnt = 0;
                  int vangleCnt = 0;
                  int azimuthCnt = 0;
                  int distChannelCnt = 0;

                  for (int processLoop = 0; processLoop < 2; processLoop++)
                  {
                    int totalChannelCnt = 0;


                    bool bCont = true;

                    datagram_parse_task task = process_idle;
                    bool parsePacket = true;
                    parseOff = 64 + encoderDataOffset;
                    bool processData = false;

                    if (processLoop == 0)
                    {
                      distChannelCnt = 0;
                      rssiCnt = 0;
                      vangleCnt = 0;
                      azimuthCnt = 0;
                      azimuth_vec.clear();
                    }

                    if (processLoop == 1)
                    {
                      processData = true;
                      numEchos = distChannelCnt;
                      msg.ranges.resize(numberOfItems * numEchos);
                      if (rssiCnt > 0)
                      {
                        msg.intensities.resize(numberOfItems * rssiCnt);
                      }
                      else
                      {
                      }
                      if (vangleCnt > 0) // should be 0 or 1
                      {
                        vang_vec.resize(numberOfItems * vangleCnt);
                      }
                      else
                      {
                        vang_vec.clear();
                      }
                      if (azimuthCnt > 0)
                      {
                        azimuth_vec.resize(numberOfItems * azimuthCnt);
                      }
                      // echoMask = (1 << numEchos) - 1;

                      // reset count. We will use the counter for index calculation now.
                      distChannelCnt = 0;
                      rssiCnt = 0;
                      vangleCnt = 0;
                      azimuthCnt = 0;

                    }

                    szChannel[6] = '\0';
                    scaleFactor = 1.0;
                    scaleFactorOffset = 0.0;
                    startAngleDiv10000 = 1;
                    sizeOfSingleAngularStepDiv10000 = 1;
                    startAngle = 0.0;
                    sizeOfSingleAngularStep = 0.0;
                    numberOfItems = 0;


#if 1 // prepared for multiecho parsing

                    bCont = true;
                    // try to get number of DIST and RSSI from binary data
                    task = process_idle;
                    do
                    {
                      task = process_idle;
                      int processDataLenValuesInBytes = 2;

                      if (totalChannelCnt == numberOf16BitChannels)
                      {
                        parseOff += 2; // jump of number of 8 bit channels- already parsed above
                      }

                      if (totalChannelCnt >= numberOf16BitChannels)
                      {
                        processDataLenValuesInBytes = 1; // then process 8 bit values ...
                      }
                      bCont = false;
                      strcpy(szChannel, "");

                      if (totalChannelCnt < (numberOf16BitChannels + numberOf8BitChannels))
                      {
                        szChannel[5] = '\0';
                        strncpy(szChannel, (const char *) receiveBuffer + parseOff, 5);
                      }
                      else
                      {
                        // all channels processed (16 bit and 8 bit channels)
                      }

                      if (strstr(szChannel, "DIST") == szChannel)
                      {
                        task = process_dist;
                        distChannelCnt++;
                        bCont = true;
                        numberOfItems = 0;
                        memcpy(&numberOfItems, receiveBuffer + parseOff + 19, 2);
                        swap_endian((unsigned char *) &numberOfItems, 2);
                      }
                      if (strstr(szChannel, "VANG") == szChannel) // MRS6000 transmits elevation angles on channel "VANG"
                      {
                        vangleCnt++;
                        task = process_vang;
                        bCont = true;
                        numberOfItems = 0;
                        memcpy(&numberOfItems, receiveBuffer + parseOff + 19, 2);
                        swap_endian((unsigned char *) &numberOfItems, 2);
                        vang_vec.resize(numberOfItems);
                      }
                      if (strstr(szChannel, "ANGL") == szChannel) // MRS1xxx and LMS1xxx transmit azimuth angles on channel "ANGL"
                      {
                        azimuthCnt++;
                        task = process_azimuth;
                        bCont = true;
                        numberOfItems = 0;
                        memcpy(&numberOfItems, receiveBuffer + parseOff + 19, 2);
                        swap_endian((unsigned char *) &numberOfItems, 2);
                        azimuth_vec.resize(numberOfItems);
                      }
                      if (strstr(szChannel, "RSSI") == szChannel)
                      {
                        task = process_rssi;
                        rssiCnt++;
                        bCont = true;
                        numberOfItems = 0;
                        // copy two byte value (unsigned short to  numberOfItems
                        memcpy(&numberOfItems, receiveBuffer + parseOff + 19, 2);
                        swap_endian((unsigned char *) &numberOfItems, 2); // swap
                      }
                      if (bCont)
                      {
                        scaleFactor = 0.0;
                        scaleFactorOffset = 0.0;
                        startAngleDiv10000 = 0;
                        sizeOfSingleAngularStepDiv10000 = 0;
                        numberOfItems = 0;

                        memcpy(&scaleFactor, receiveBuffer + parseOff + 5, 4);
                        memcpy(&scaleFactorOffset, receiveBuffer + parseOff + 9, 4);
                        memcpy(&startAngleDiv10000, receiveBuffer + parseOff + 13, 4);
                        memcpy(&sizeOfSingleAngularStepDiv10000, receiveBuffer + parseOff + 17, 2);
                        memcpy(&numberOfItems, receiveBuffer + parseOff + 19, 2);


                        swap_endian((unsigned char *) &scaleFactor, 4);
                        swap_endian((unsigned char *) &scaleFactorOffset, 4);
                        swap_endian((unsigned char *) &startAngleDiv10000, 4);
                        swap_endian((unsigned char *) &sizeOfSingleAngularStepDiv10000, 2);
                        swap_endian((unsigned char *) &numberOfItems, 2);

                        if(false) // if (lms1000_debug) // LMS-1000 diagnosis
                        {
                          ROS_DEBUG_STREAM("LMDscandata: lidar_scan_time=" << SystemCountScan << " microsec, lidar_transmit_time=" << SystemCountTransmit << " microsec, "
                            << "scan_frequency=" << (0.01 * scanFrequencyX100) << " Hz, measurement_frequency=" << measurementFrequencyDiv100 << " Hz,"
                            << "start_angle=" << (0.0001 * startAngleDiv10000) << " [deg], angular_step="<< (0.0001 * sizeOfSingleAngularStepDiv10000) << " [deg]");
                          // lms1000_debug = false;
                        } //

                        if (processData)
                        {
                          unsigned short *data = (unsigned short *) (receiveBuffer + parseOff + 21);

                          unsigned char *swapPtr = (unsigned char *) data;
                          // copy RSSI-Values +2 for 16-bit values +1 for 8-bit value
                          for (int i = 0;
                               i < numberOfItems * processDataLenValuesInBytes; i += processDataLenValuesInBytes)
                          {
                            if (processDataLenValuesInBytes == 1)
                            {
                            }
                            else
                            {
                              unsigned char tmp;
                              tmp = swapPtr[i + 1];
                              swapPtr[i + 1] = swapPtr[i];
                              swapPtr[i] = tmp;
                            }
                          }
                          int idx = 0;

                          switch (task)
                          {

                            case process_dist:
                            {
                              startAngle = startAngleDiv10000 / 10000.00;
                              sizeOfSingleAngularStep = sizeOfSingleAngularStepDiv10000 / 10000.0;
                              

                              configureAngleParameters(msg, startAngle, sizeOfSingleAngularStep, numberOfItems, parser_);
                              ROS_DEBUG_STREAM("process_dist: msg.angle_min=" << (msg.angle_min*180/M_PI) << ", msg.angle_max=" << (msg.angle_max*180/M_PI) << ", msg.angle_increment=" << (msg.angle_increment*180/M_PI) << ", scaleFactor=" << scaleFactor << ", scaleFactorOffset=" << scaleFactorOffset);

                              // Avoid 2*PI wrap around, if (msg.angle_max - msg.angle_min - 2*PI) is slightly above 0.0 due to floating point arithmetics
                              bool wrap_avoid = false;
                              bool ret = check_near_plus_minus_pi(&(msg.angle_min));
                              if (ret)
                              {
                                wrap_avoid = true;
                              }
                              ret = check_near_plus_minus_pi(&(msg.angle_max));
                              if (ret)
                              {
                                wrap_avoid = true; 
                              }

                              // in the case of slighlty modified min/max angles,
                              // we recalculate the angle_increment.
                              if (wrap_avoid)
                              {
                                msg.angle_increment = (msg.angle_max - msg.angle_min) / (numberOfItems - 1);
                              }
                              ROS_DEBUG_STREAM("process_dist: msg.angle_min=" << (msg.angle_min*180/M_PI) << ", msg.angle_max=" << (msg.angle_max*180/M_PI) << ", msg.angle_increment=" << (msg.angle_increment*180/M_PI) << ", scaleFactor=" << scaleFactor << ", scaleFactorOffset=" << scaleFactorOffset << ", " << (8*processDataLenValuesInBytes) << "-bit channel");

                              float *rangePtr = NULL;

                              if (numberOfItems > 0)
                              {
                                rangePtr = &msg.ranges[0];
                              }
                              float scaleFactor_001 = 0.001F * scaleFactor;// to avoid repeated multiplication
                              for (int i = 0; i < numberOfItems; i++)
                              {
                                idx = i + numberOfItems * (distChannelCnt - 1);
                                rangePtr[idx] = (float) data[i] * scaleFactor_001 + scaleFactorOffset;
#ifdef DEBUG_DUMP_ENABLED
                                if (distChannelCnt == 1)
                                {
                                  if (i == floor(numberOfItems / 2))
                                  {
                                    double curTimeStamp = SystemCountScan + i * msg.time_increment * 1E6;
                                    //DataDumper::instance().pushData(curTimeStamp, "DIST", rangePtr[idx]);
                                  }
                                }
#endif
                              }
                            }
                            break;
                            case process_rssi:
                            {
                              float *intensityPtr = NULL;

                              if (numberOfItems > 0)
                              {
                                intensityPtr = &msg.intensities[0];

                              }
                              for (int i = 0; i < numberOfItems; i++)
                              {
                                idx = i + numberOfItems * (rssiCnt - 1);
                                // we must select between 16 bit and 8 bit values
                                float rssiVal = 0.0;
                                if (processDataLenValuesInBytes == 2)
                                {
                                  rssiVal = (float) data[i];
                                }
                                else
                                {
                                  unsigned char *data8Ptr = (unsigned char *) data;
                                  rssiVal = (float) data8Ptr[i];
                                }
                                intensityPtr[idx] = rssiVal * scaleFactor + scaleFactorOffset;
                              }
                            }
                              break;

                            case process_vang: // elevation angles MRS6000
                              if (numberOfItems > 0)
                              {
                                float *vangPtr = &vang_vec[0]; // much faster, with vang_vec[i] each time the size will be checked
                                for (int i = 0; i < numberOfItems; i++)
                                {
                                  vangPtr[i] = (float) data[i] * scaleFactor + scaleFactorOffset;
                                }
                              }
                              break;

                            case process_azimuth: // azimuth angles MRS1xxx and LMS1xxx
                              if (numberOfItems > 0)
                              {
                                if (std::abs(scaleFactorOffset) <= FLT_EPSILON)
                                {
                                  // Note: If the firmware of a previous uncalibrated MRS1xxx or LMS1xxx has been upgraded to firmware version 2.3.0 (or newer) without calibrating the azimuth table,
                                  // the scaleFactorOffset is most likely 0.0, but the azimuth correction table is activated by default.
                                  // In this case, the following warnings is displayed according to requirements. To avoid the warning, parameter scandatacfg_azimuth_table can be set to value 0
                                  // (i.e. azimuth correction table deactivated) in the launchfile.
                                  ROS_WARN_STREAM("## WARNING parseCommonBinaryResultTelegram(): process_azimuth with unexpected scaleFactorOffset=" << scaleFactorOffset << ", expected value is not 0.0");
                                  ROS_WARN_STREAM("## WARNING parseCommonBinaryResultTelegram(): set parameter scandatacfg_azimuth_table=0 in the launchfile if the lidar has no calibrated azimuth table.");
                                }
                                float angle_min_rad = (1.0e-4f * startAngleDiv10000) * M_PI / 180.0 + parser_->getCurrentParamPtr()->getScanAngleShift();
                                float angle_inc_rad = (1.0e-4f * sizeOfSingleAngularStepDiv10000) * M_PI / 180.0;
                                float angle_max_rad = angle_min_rad + (numberOfItems - 1) * angle_inc_rad;
                                ROS_DEBUG_STREAM("process_dist: msg.angle_min=" << (msg.angle_min*180/M_PI) << ", msg.angle_max=" << (msg.angle_max*180/M_PI) << ", msg.angle_increment=" << (msg.angle_increment*180/M_PI));
                                ROS_DEBUG_STREAM("process_azimuth: angle_min=" << (angle_min_rad*180/M_PI) << ", angle_max=" << (angle_max_rad*180/M_PI) << ", angle_inc=" << (angle_inc_rad*180/M_PI) 
                                  << ", scaleFactor=" << scaleFactor << ", scaleFactorOffset=" << scaleFactorOffset << ", " << (8*processDataLenValuesInBytes) << "-bit channel");
                                // angular correction according to "LMS4000---Angular-correction.pptx" for scalefactor=1:
                                // azimuth = angle_min + (i * angle_inc) + (offset + data[i]) with 0 < i < numberOfItems
                                float *azimuthPtr = &azimuth_vec[0];
                                if (processDataLenValuesInBytes == 1)
                                {
                                  uint8_t* data_ptr = (uint8_t*) data;
                                  for (int i = 0; i < numberOfItems; i++)
                                    azimuthPtr[i] = (angle_min_rad + i * angle_inc_rad + (float)(1.0e-4 * M_PI / 180.0) * ((float)data_ptr[i] * scaleFactor + scaleFactorOffset));
                                }
                                else if (processDataLenValuesInBytes == 2)
                                {
                                  uint16_t* data_ptr = (uint16_t*) data;
                                  for (int i = 0; i < numberOfItems; i++)
                                    azimuthPtr[i] = (angle_min_rad + i * angle_inc_rad + (float)(1.0e-4 * M_PI / 180.0) * ((float)data_ptr[i] * scaleFactor + scaleFactorOffset));
                                }
                                // std::stringstream dbg_stream;
                                // for (int i = 0; i < numberOfItems; i++)
                                //   dbg_stream << " " << std::fixed << std::setprecision(3) << (azimuth_vec[i]*180/M_PI);
                                // ROS_DEBUG_STREAM("azimuth table: " << dbg_stream.str());
                              }
                              break;
                          }
                        }
                        parseOff += 21 + processDataLenValuesInBytes * numberOfItems;


                      }
                      totalChannelCnt++;
                    } while (bCont);
                  }
#endif

                  // double elevAngle = elevationAngleInRad; // elevAngleX200 / 200.0;
                  scanFrequency = scanFrequencyX100 / 100.0;

                  return true;
    }

} /* namespace sick_scan_xd */
