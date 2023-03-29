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

namespace sick_scan
{
    /* template<typename T> static void readFromBuffer(const uint8_t* receiveBuffer, int& pos, int receiveBufferLength, T& value)
    {
        if(pos + sizeof(value) < receiveBufferLength)
        {
            memcpy(&value, receiveBuffer + pos, sizeof(value));
            swap_endian((unsigned char *) &value, sizeof(value));
            pos += sizeof(value);
        }
        else
        {
            ROS_WARN_STREAM("readFromBuffer(): read pos = " << pos << " + sizeof(value) = " << sizeof(value) << " exceeds receiveBufferLength = " << receiveBufferLength);
        }
    } */


    /** Parse common result telegrams, i.e. parse telegrams of type LMDscandata received from the lidar */
    bool parseCommonBinaryResultTelegram(const uint8_t* receiveBuffer, int receiveBufferLength, short& elevAngleX200, double elevAngleTelegramValToDeg, double& elevationAngleInRad, rosTime& recvTimeStamp,
        bool config_sw_pll_only_publish, bool use_generation_timestamp, SickGenericParser* parser_, bool& FireEncoder, sick_scan_msg::Encoder& EncoderMsg, int& numEchos, 
        std::vector<float>& vang_vec, ros_sensor_msgs::LaserScan & msg)
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

                  double timestampfloat = sec(recvTimeStamp) + nsec(recvTimeStamp) * 1e-9;
                  bool bRet;
                  if (SystemCountScan !=
                      lastSystemCountScan)//MRS 6000 sends 6 packets with same  SystemCountScan we should only update the pll once with this time stamp since the SystemCountTransmit are different and this will only increase jitter of the pll
                  {
                    bRet = SoftwarePLL::instance().updatePLL(sec(recvTimeStamp), nsec(recvTimeStamp),
                                                             SystemCountTransmit);
                    lastSystemCountScan = SystemCountScan;
                  }
                  // ROS_DEBUG_STREAM("recvTimeStamp before software-pll correction: " << recvTimeStamp);
                  rosTime tmp_time = recvTimeStamp;
                  uint32_t recvTimeStampSec = (uint32_t)sec(recvTimeStamp), recvTimeStampNsec = (uint32_t)nsec(recvTimeStamp);
                  uint32_t lidar_ticks = SystemCountScan;
                  if(use_generation_timestamp == 0)
                    lidar_ticks = SystemCountTransmit;
                  bRet = SoftwarePLL::instance().getCorrectedTimeStamp(recvTimeStampSec, recvTimeStampNsec, lidar_ticks);

                  recvTimeStamp = rosTime(recvTimeStampSec, recvTimeStampNsec);
                  double timestampfloat_coor = sec(recvTimeStamp) + nsec(recvTimeStamp) * 1e-9;
                  double DeltaTime = timestampfloat - timestampfloat_coor;
                  // ROS_DEBUG_STREAM("recvTimeStamp after software-pll correction: " << recvTimeStamp);
                  //ROS_INFO("%F,%F,%u,%u,%F",timestampfloat,timestampfloat_coor,SystemCountTransmit,SystemCountScan,DeltaTime);
                  //TODO Handle return values
                  if (config_sw_pll_only_publish == true)
                  {
                    SoftwarePLL::instance().packets_received++;
                    if (bRet == false)
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
                      return false;
                    }
                  }

#ifdef DEBUG_DUMP_ENABLED
                  double elevationAngleInDeg = elevationAngleInRad / deg2rad_const;
                  // DataDumper::instance().pushData((double)SystemCountScan, "LAYER", elevationAngleInDeg);
                  //DataDumper::instance().pushData((double)SystemCountScan, "LASESCANTIME", SystemCountScan);
                  //DataDumper::instance().pushData((double)SystemCountTransmit, "LASERTRANSMITTIME", SystemCountTransmit);
                  //DataDumper::instance().pushData((double)SystemCountScan, "LASERTRANSMITDELAY", debug_duration.toSec());
#endif

                  /*
                  uint16_t u16_active_fieldset = 0;
                  memcpy(&u16_active_fieldset, receiveBuffer + 46, 2); // byte 46 + 47: input status (0 0), active fieldset
                  swap_endian((unsigned char *) &u16_active_fieldset, 2);
                  SickScanFieldMonSingleton *fieldMon = SickScanFieldMonSingleton::getInstance();
                  if(fieldMon)
                  {
                    fieldMon->setActiveFieldset(u16_active_fieldset & 0xFF);
                    ROS_INFO_STREAM("Binary scandata: active_fieldset = " << fieldMon->getActiveFieldset());
                  }
                  */
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
                    process_rssi,
                    process_idle
                  };
                  int rssiCnt = 0;
                  int vangleCnt = 0;
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
                      // echoMask = (1 << numEchos) - 1;

                      // reset count. We will use the counter for index calculation now.
                      distChannelCnt = 0;
                      rssiCnt = 0;
                      vangleCnt = 0;

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
                    bool doVangVecProc = false;
                    // try to get number of DIST and RSSI from binary data
                    task = process_idle;
                    do
                    {
                      task = process_idle;
                      doVangVecProc = false;
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
                      if (strstr(szChannel, "VANG") == szChannel)
                      {
                        vangleCnt++;
                        task = process_vang;
                        bCont = true;
                        numberOfItems = 0;
                        memcpy(&numberOfItems, receiveBuffer + parseOff + 19, 2);
                        swap_endian((unsigned char *) &numberOfItems, 2);

                        vang_vec.resize(numberOfItems);

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

                        /* if (lms1000_debug) // LMS-1000 diagnosis
                        {
                          ROS_INFO_STREAM("LMDscandata: lidar_scan_time=" << SystemCountScan << " microsec, lidar_transmit_time=" << SystemCountTransmit << " microsec, "
                            << "scan_frequency=" << (0.01 * scanFrequencyX100) << " Hz, measurement_frequency=" << measurementFrequencyDiv100 << " Hz,"
                            << "start_angle=" << (0.0001 * startAngleDiv10000) << " [deg], angular_step="<< (0.0001 * sizeOfSingleAngularStepDiv10000) << " [deg]");
                          lms1000_debug = false;
                        } */

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
                              sizeOfSingleAngularStep *= (M_PI / 180.0);

                              msg.angle_min = startAngle / 180.0 * M_PI + parser_->getCurrentParamPtr()->getScanAngleShift(); // msg.angle_min = startAngle / 180.0 * M_PI - M_PI / 2;
                              msg.angle_increment = sizeOfSingleAngularStep;
                              msg.angle_max = msg.angle_min + (numberOfItems - 1) * msg.angle_increment;

                              if(msg.time_increment == 0) // NAV-350
                              {
                                  msg.time_increment = fabs(parser_->getCurrentParamPtr()->getNumberOfLayers() * msg.scan_time * msg.angle_increment / (2.0 * M_PI));
                              }

                              if (parser_->getCurrentParamPtr()->getScannerName().compare(SICK_SCANNER_NAV_3XX_NAME) == 0)
                              {
                                msg.angle_min = (float)(-M_PI);
                                msg.angle_max = (float)(+M_PI);
                                msg.angle_increment *= -1.0;
                              }
                              else if (parser_->getCurrentParamPtr()->getScanMirroredAndShifted()) // i.e. for SICK_SCANNER_LRS_36x0_NAME and SICK_SCANNER_NAV_3XX_NAME
                              {
                                /* TODO: Check this ...
                                msg.angle_min -= (float)(M_PI / 2);
                                msg.angle_max -= (float)(M_PI / 2);
                                */

                                msg.angle_min *= -1.0;
                                msg.angle_increment *= -1.0;
                                msg.angle_max *= -1.0;

                              }
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
                                //XXX
                              }

                            }
                              break;
                            case process_rssi:
                            {
                              // Das muss vom Protokoll abgeleitet werden. !!!

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

                            case process_vang:
                              float *vangPtr = NULL;
                              if (numberOfItems > 0)
                              {
                                vangPtr = &vang_vec[0]; // much faster, with vang_vec[i] each time the size will be checked
                              }
                              for (int i = 0; i < numberOfItems; i++)
                              {
                                vangPtr[i] = (float) data[i] * scaleFactor + scaleFactorOffset;
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

} /* namespace sick_scan */
