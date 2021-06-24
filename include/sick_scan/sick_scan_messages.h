/*
 * @brief Implementation of ROS messages for sick_scan
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

#ifndef SICK_SCAN_MESSAGES_H_
#define SICK_SCAN_MESSAGES_H_

#include "sick_scan/sick_scan_common.h"
#include "sick_scan/sick_scan_common_tcp.h"
#include "sick_scan/LFErecMsg.h"
#include "sick_scan/LIDoutputstateMsg.h"

namespace sick_scan
{

  class SickScanMessages
  {
  public:

    SickScanMessages(ros::NodeHandle* nh = 0);

    virtual ~SickScanMessages();

    /*
     * @brief parses and converts a lidar LIDoutputstate message to a ros LIDoutputstate message
     * 
     * Example LIDoutputstate message from 20210111_sick_tim781s_mon_elephant.pcapng.json:
     * "tcp.description": ".......4sSN LIDoutputstate .................................E",
     * "tcp.payload": "02:02:02:02:00:00:00:34:73:53:4e:20:4c:49:44:6f:75:74:70:75:74:73:74:61:74:65:20:00:00:00:00:00:00:01:00:00:01:26:00:00:00:01:22:00:00:00:00:00:01:00:00:00:04:02:00:00:00:00:00:00:45"
     * 33 byte LIDoutputstate payload after "sSN LIDoutputstate ": "00:00:00:00:00:00:01:00:00:01:26:00:00:00:01:22:00:00:00:00:00:01:00:00:00:04:02:00:00:00:00:00:00"
     * 
     * @param[in] timeStamp timestamp on receiving the lidar message
     * @param[in] receiveBuffer byte array of lidar message
     * @param[in] receiveLength size of lidar message in byte
     * @param[in] useBinaryProtocol binary lidar message (true, Cola-B) or ascii lidar message (false, Cola-A)
     * @param[in] frame_id frame id of output message
     * @param[out] output_msg converted output message
     * 
     * @return true on success, false on error
     */
    static bool parseLIDoutputstateMsg(const ros::Time& timeStamp, uint8_t* receiveBuffer, int receiveLength, bool useBinaryProtocol, const std::string& frame_id, sick_scan::LIDoutputstateMsg& output_msg);

    /*
     * @brief parses and converts a lidar LFErec message to a ros LFErec message
     * 
     * Example LIDoutputstate message from 20210111_sick_tim781s_lferec_elephant.pcapng.json:
     * "tcp.description": "........sSN LFErec ........................0..............3...........................0..............4....P......................0..............35.....",
     * "tcp.payload": "02:02:02:02:00:00:00:8e:73:53:4e:20:4c:46:45:72:65:63:20:00:03:00:01:01:00:00:00:00:3f:80:00:00:00:00:00:00:00:00:0d:05:ff:f9:22:30:00:00:00:00:00:00:00:00:01:07:b2:01:01:08:33:2e:00:09:ac:90:00:01:02:00:00:00:00:3f:80:00:00:00:00:00:00:00:00:0d:05:ff:f9:22:30:01:00:00:00:00:00:00:00:01:07:b2:01:01:08:34:02:00:06:9f:50:00:01:03:00:00:00:00:3f:80:00:00:00:00:00:00:00:00:0d:05:ff:f9:22:30:01:00:00:00:00:00:00:00:01:07:b2:01:01:08:33:35:00:09:ac:90:ac"
     * 131 byte LFErec payload after "sSN LFErec ": "00:03:00:01:01:00:00:00:00:3f:80:00:00:00:00:00:00:00:00:0d:05:ff:f9:22:30:00:00:00:00:00:00:00:00:01:07:b2:01:01:08:33:2e:00:09:ac:90:00:01:02:00:00:00:00:3f:80:00:00:00:00:00:00:00:00:0d:05:ff:f9:22:30:01:00:00:00:00:00:00:00:01:07:b2:01:01:08:34:02:00:06:9f:50:00:01:03:00:00:00:00:3f:80:00:00:00:00:00:00:00:00:0d:05:ff:f9:22:30:01:00:00:00:00:00:00:00:01:07:b2:01:01:08:33:35:00:09:ac:90"
     * LFErec payload starts with 16 byte fields_number "00:03" -> 3 fields, each field has (131-2)/3 = 43 byte
     * Each field contains a field_index (1,2,3) and field_result (0: invalid/incorrect, 1: free/clear, 2: infringed)
     * Note: field indices are sorted in reverse order, i.e. with 2 fields configured, we have typically
     * output_msg.fields[0].field_index = 1, output_msg.fields[0].field_result_mrs = 0 (invalid)
     * output_msg.fields[1].field_index = 2, output_msg.fields[1].field_result_mrs = 1 or 2 (clear=1 or infringed=2)
     * output_msg.fields[2].field_index = 3, output_msg.fields[2].field_result_mrs = 1 or 2 (clear=1 or infringed=2)
     * 
     * @param[in] timeStamp timestamp on receiving the lidar message
     * @param[in] receiveBuffer byte array of lidar message
     * @param[in] receiveLength size of lidar message in byte
     * @param[in] useBinaryProtocol binary lidar message (true, Cola-B) or ascii lidar message (false, Cola-A)
     * @param[in] eval_field_logic USE_EVAL_FIELD_LMS5XX_LOGIC or USE_EVAL_FIELD_TIM7XX_LOGIC
     * @param[in] frame_id frame id of output message
     * @param[out] output_msg converted output message
     * 
     * @return true on success, false on error
     */
    static bool parseLFErecMsg(const ros::Time& timeStamp, uint8_t* receiveBuffer, int receiveLength, bool useBinaryProtocol, EVAL_FIELD_SUPPORT eval_field_logic, const std::string& frame_id, sick_scan::LFErecMsg& output_msg);

  protected:

    /*
     * Member data
     */

  }; /* class SickScanMessages */

} /* namespace sick_scan */
#endif /* SICK_SCAN_MESSAGES_H_ */
