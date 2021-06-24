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

#include "sick_scan/sick_scan_messages.h"

typedef uint8_t* byte_ptr;
template<typename T> static bool readBinaryBuffer(byte_ptr& buffer, int & bufferlen, T& value)
{
    if(sizeof(value) > bufferlen)
    {
        ROS_ERROR_STREAM("## ERROR SickScanMessages::readBinaryBuffer(): bufferlen=" << bufferlen << " byte, " << sizeof(value) << " byte required.");
        return false;
    }
    memcpy(&value, buffer, sizeof(value));
    swap_endian((unsigned char *) &value, sizeof(value));
    buffer += sizeof(value);
    bufferlen -= (int)sizeof(value);
    return true;
}

sick_scan::SickScanMessages::SickScanMessages(ros::NodeHandle* nh)
{
}

sick_scan::SickScanMessages::~SickScanMessages()
{
}

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
bool sick_scan::SickScanMessages::parseLIDoutputstateMsg(const ros::Time& timeStamp, uint8_t* receiveBuffer, int receiveLength, bool useBinaryProtocol, const std::string& frame_id, sick_scan::LIDoutputstateMsg& output_msg)
{
    if(useBinaryProtocol)
    {
        // parse and convert LIDoutputstate message
        int dbg_telegram_length = receiveLength;
        std::string dbg_telegram_hex_copy = DataDumper::binDataToAsciiString(receiveBuffer, receiveLength);
        int msg_start_idx = 27;  // 4 byte STX + 4 byte payload_length + 19 byte "sSN LIDoutputstate " = 27 byte
        int msg_parameter_length = receiveLength - msg_start_idx - 1; // start bytes + 1 byte CRC
        if(msg_parameter_length < 8) // at least 6 byte (version+system) + 2 byte (timestamp status)
        {
            ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLIDoutputstateMsg(): received " << receiveLength << " byte, expected at least 8 byte (" << __FILE__ << ":" << __LINE__ << ")");
            return false;
        }
        // The LIDoutputstate parameter may have 11 byte timestamp or not, i.e. the parameter format is either:
        // a) msg_parameter_length := (6 byte version+system) + number_of_output_states * (5 byte state+counter per output_state) + (2 byte 0x0000 no timestamp), or 
        // b) msg_parameter_length := (6 byte version+system) + number_of_output_states * (5 byte state+counter per output_state) + (2 byte 0x0001 timestamp available) + (11 byte timestamp)
        bool parameter_format_ok = false;
        // Check parameter format a (default, TiM781S: no timestamp)
        uint16_t timestamp_status = (receiveBuffer[receiveLength - 3] << 8) | (receiveBuffer[receiveLength - 2]); // 2 byte before receiveBuffer[receiveLength-1] = CRC
        int number_of_output_states = (msg_parameter_length - 6 - 2) / 5; // each output state has 1 byte state enum + 4 byte counter = 5 byte
        if(msg_parameter_length == 6 + number_of_output_states * 5 + 2 && timestamp_status == 0)
        {
            parameter_format_ok = true;
            ROS_DEBUG_STREAM("SickScanMessages::parseLIDoutputstateMsg(): " << number_of_output_states << " output states, no timestamp" );
        }
        else // Check parameter format b with timestamp
        {
            timestamp_status = (receiveBuffer[receiveLength - 14] << 8) | (receiveBuffer[receiveLength - 13]); // 2 byte before receiveBuffer[receiveLength-12] = (11 byte timestamp) + (1 byte CRC)
            number_of_output_states = (msg_parameter_length - 6 - 2 - 11) / 5; // each output state has 1 byte state enum + 4 byte counter = 5 byte
            if(msg_parameter_length == 6 + number_of_output_states * 5 + 2 + 11 && timestamp_status > 0)
            {
                parameter_format_ok = true;
                ROS_DEBUG_STREAM("SickScanMessages::parseLIDoutputstateMsg(): " << number_of_output_states << " output states and timestamp" );
            }
        }
        if(!parameter_format_ok)
        {
            ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLIDoutputstateMsg(): received " << receiveLength << " byte with invalid format (" << __FILE__ << ":" << __LINE__ << "): "
                << DataDumper::binDataToAsciiString(receiveBuffer, receiveLength));
            return false;
        }
        // Read 2 byte version + 2 byte system status
        receiveBuffer += msg_start_idx;
        receiveLength -= msg_start_idx;
        if( !readBinaryBuffer(receiveBuffer, receiveLength, output_msg.version_number)
         || !readBinaryBuffer(receiveBuffer, receiveLength, output_msg.system_counter))
        {
            ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLIDoutputstateMsg(): error parsing version_number and system_counter (" << __FILE__ << ":" << __LINE__ << ")");
            return false;
        }
        // Read N output states
        output_msg.output_state.reserve(number_of_output_states);
        output_msg.output_count.reserve(number_of_output_states);
        for(int state_cnt = 0; state_cnt < number_of_output_states; state_cnt++)
        {
            uint8_t output_state;
            uint32_t output_count;
            if( !readBinaryBuffer(receiveBuffer, receiveLength, output_state)
             || !readBinaryBuffer(receiveBuffer, receiveLength, output_count))
            {
                ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLIDoutputstateMsg(): error parsing version_number and system_counter (" << __FILE__ << ":" << __LINE__ << ")");
                return false;
            }
            if(output_state == 0 || output_state == 1) // 0: not active, 1: active, 2: not used
            {
                output_msg.output_state.push_back(output_state);
                output_msg.output_count.push_back(output_count);
            }
        }
        // Read timestamp state
        if( !readBinaryBuffer(receiveBuffer, receiveLength, output_msg.time_state))
        {
            ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLIDoutputstateMsg(): error parsing time_state (" << __FILE__ << ":" << __LINE__ << ")");
            return false;
        }
        if(output_msg.time_state != timestamp_status) // time_state and previously parsed timestamp_status must be identical
        {
            ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLIDoutputstateMsg(): time_state mismatch, received " << (int)output_msg.time_state << ", expected " << (int)timestamp_status << " (" << __FILE__ << ":" << __LINE__ << ")");
            return false;

        }
        // Read optional timestamp
        if(timestamp_status > 0)
        {
            if(!readBinaryBuffer(receiveBuffer, receiveLength, output_msg.year)
            || !readBinaryBuffer(receiveBuffer, receiveLength, output_msg.month)
            || !readBinaryBuffer(receiveBuffer, receiveLength, output_msg.day)
            || !readBinaryBuffer(receiveBuffer, receiveLength, output_msg.hour)
            || !readBinaryBuffer(receiveBuffer, receiveLength, output_msg.minute)
            || !readBinaryBuffer(receiveBuffer, receiveLength, output_msg.second)
            || !readBinaryBuffer(receiveBuffer, receiveLength, output_msg.microsecond))
            {
                ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLIDoutputstateMsg(): error parsing timestamp (" << __FILE__ << ":" << __LINE__ << ")");
                return false;
            }
        }
        output_msg.header.stamp = timeStamp;
        output_msg.header.seq = 0;
        output_msg.header.frame_id = frame_id;

        // Debug messages
        std::stringstream state_str;
        assert(output_msg.output_state.size() == output_msg.output_state.size());
        state_str << number_of_output_states << " output states received" << (timestamp_status?" with":", no") << " timestamp\n";
        for(int state_cnt = 0; state_cnt < output_msg.output_count.size(); state_cnt++)
            state_str << "state[" << state_cnt << "]: " << (uint32_t)output_msg.output_state[state_cnt] << ", count=" << (uint32_t)output_msg.output_count[state_cnt] << "\n";
        ROS_DEBUG_STREAM("SickScanMessages::parseLIDoutputstateMsg():\n"
            << dbg_telegram_length << " byte telegram: "  << dbg_telegram_hex_copy << "\n"
            << "version_number: " << (uint32_t)output_msg.version_number << ", system_counter: " << (uint32_t)output_msg.system_counter << "\n"
            << state_str.str()
            << "time state: " << (uint32_t)output_msg.time_state
            << ", date: " << std::setfill('0') << std::setw(4) << (uint32_t)output_msg.year << "-" << std::setfill('0') << std::setw(2) << (uint32_t)output_msg.month << "-" << std::setfill('0') << std::setw(2) << (uint32_t)output_msg.day
            << ", time: " << std::setfill('0') << std::setw(2) << (uint32_t)output_msg.hour << ":" << std::setfill('0') << std::setw(2) << (uint32_t)output_msg.minute << ":" << std::setfill('0') << std::setw(2) << (uint32_t)output_msg.second << "." << std::setfill('0') << std::setw(6) << (uint32_t)output_msg.microsecond);

        return true;
    }
    else
    {
        ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLIDoutputstateMsg not implemented for Cola-A messages(" << __FILE__ << ":" << __LINE__ << ")");        
    }
    return false;
}

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
bool sick_scan::SickScanMessages::parseLFErecMsg(const ros::Time& timeStamp, uint8_t* receiveBuffer, int receiveLength, bool useBinaryProtocol, EVAL_FIELD_SUPPORT eval_field_logic, const std::string& frame_id, sick_scan::LFErecMsg& output_msg)
{
    if(useBinaryProtocol)
    {
        uint8_t* msg_receiveBuffer = receiveBuffer;
        int msg_receiveLength = receiveLength;

        // parse and convert LFErec messages, see https://github.com/SICKAG/libsick_ldmrs/blob/master/src/sopas/LdmrsSopasLayer.cpp lines 1414 ff.
        int msg_start_idx = 19;  // 4 byte STX + 4 byte payload_length + 11 byte "sSN LFErec " = 19 byte
        receiveBuffer += msg_start_idx;
        receiveLength -= msg_start_idx;

        output_msg.fields_number = 0;
        if(!readBinaryBuffer(receiveBuffer, receiveLength, output_msg.fields_number) || output_msg.fields_number <= 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): parse error, fields number = " << output_msg.fields_number<< " should be greater 0 (" << __FILE__ << ":" << __LINE__ << ")");
            ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): " << msg_receiveLength << " byte received: " << DataDumper::binDataToAsciiString(msg_receiveBuffer, msg_receiveLength));
            return false;
        }
        output_msg.fields.reserve(output_msg.fields_number);
        for(int field_idx = 0; field_idx < output_msg.fields_number; field_idx++)
        {
            sick_scan::LFErecFieldMsg field_msg;
            if( !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.version_number)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.field_index)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.sys_count)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.dist_scale_factor)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.dist_scale_offset)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.angle_scale_factor)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.angle_scale_offset)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.field_result_mrs))
            {
                ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): parse error field " << field_idx << " (" << __FILE__ << ":" << __LINE__ << ")");
                ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): " << msg_receiveLength << " byte received: " << DataDumper::binDataToAsciiString(msg_receiveBuffer, msg_receiveLength));
                return false;
            }
            uint16_t dummy_byte[3] = {0};
            if(!readBinaryBuffer(receiveBuffer, receiveLength, dummy_byte[0])
            || !readBinaryBuffer(receiveBuffer, receiveLength, dummy_byte[1])
            || !readBinaryBuffer(receiveBuffer, receiveLength, dummy_byte[2]))
            {
                ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): parse error field " << field_idx << " (" << __FILE__ << ":" << __LINE__ << ")");
                ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): " << msg_receiveLength << " byte received: " << DataDumper::binDataToAsciiString(msg_receiveBuffer, msg_receiveLength));
                return false;
            }
            int additional_dummy_byte_length = 0;
            // if(eval_field_logic == USE_EVAL_FIELD_LMS5XX_LOGIC && field_msg.field_result_mrs == 2)
            //     additional_dummy_byte_length = (21 - 6);
            if(eval_field_logic == USE_EVAL_FIELD_LMS5XX_LOGIC && (dummy_byte[0] != 0 || dummy_byte[1] != 0 || dummy_byte[2] != 0))
                additional_dummy_byte_length = (21 - 6);
            receiveBuffer += additional_dummy_byte_length;
            receiveLength -= additional_dummy_byte_length;
            if(receiveLength < 0)
            {
                ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): parse error field " << field_idx << " (" << __FILE__ << ":" << __LINE__ << ")");
                ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): " << msg_receiveLength << " byte received: " << DataDumper::binDataToAsciiString(msg_receiveBuffer, msg_receiveLength));
                return false;
            }
            if(!readBinaryBuffer(receiveBuffer, receiveLength, field_msg.time_state))
            {
                ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): parse error field " << field_idx << " (" << __FILE__ << ":" << __LINE__ << ")");
                ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): " << msg_receiveLength << " byte received: " << DataDumper::binDataToAsciiString(msg_receiveBuffer, msg_receiveLength));
                return false;
            }
            if(field_msg.time_state)
            {
                if( !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.year)
                || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.month)
                || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.day)
                || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.hour)
                || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.minute)
                || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.second)
                || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.microsecond))
                {
                    ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): parse error field " << field_idx << " (" << __FILE__ << ":" << __LINE__ << ")");
                    ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): " << msg_receiveLength << " byte received: " << DataDumper::binDataToAsciiString(msg_receiveBuffer, msg_receiveLength));
                    return false;
                }
            }
            else
            {
                field_msg.year = 0;
                field_msg.month = 0;
                field_msg.day = 0;
                field_msg.hour = 0;
                field_msg.minute = 0;
                field_msg.second = 0;
                field_msg.microsecond = 0;
            }
            output_msg.fields.push_back(field_msg);
        }
        if(output_msg.fields.size() != output_msg.fields_number)
        {
            ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): parse error, fields number = " << output_msg.fields_number << ", but " << output_msg.fields.size() << " fields parsed (" << __FILE__ << ":" << __LINE__ << ")");
            ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): " << msg_receiveLength << " byte received: " << DataDumper::binDataToAsciiString(msg_receiveBuffer, msg_receiveLength));
            return false;
        }
        if(receiveLength != 1) // 1 byte CRC still in receiveBuffer
        {
            ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): parse error, " << receiveLength << " bytes in buffer after decoding all fields, should be 0 (" << __FILE__ << ":" << __LINE__ << ")");
            ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): " << msg_receiveLength << " byte received: " << DataDumper::binDataToAsciiString(msg_receiveBuffer, msg_receiveLength));
            return false;
        }

        output_msg.header.stamp = timeStamp;
        output_msg.header.seq = 0;
        output_msg.header.frame_id = frame_id;

        std::stringstream fields_str;
        for(int field_idx = 0; field_idx < output_msg.fields.size(); field_idx++)
        {
            sick_scan::LFErecFieldMsg& field_msg = output_msg.fields[field_idx];
            if(field_idx > 0)
                fields_str << "\n";
            fields_str << "field[" << field_idx << "]: idx=" << (uint32_t)field_msg.field_index 
                << ", dist_scale_factor=" << field_msg.dist_scale_factor
                << ", dist_scale_offset=" << field_msg.dist_scale_offset
                << ", angle_scale_factor=" << field_msg.angle_scale_factor
                << ", angle_scale_offset=" << field_msg.angle_scale_offset
                << ", field_result_mrs=" << (uint32_t)field_msg.field_result_mrs
                << ", time state: " << (uint32_t)field_msg.time_state
                << ", date: " << std::setfill('0') << std::setw(4) << (uint32_t)field_msg.year << "-" << std::setfill('0') << std::setw(2) << (uint32_t)field_msg.month << "-" << std::setfill('0') << std::setw(2) << (uint32_t)field_msg.day
                << ", time: " << std::setfill('0') << std::setw(2) << (uint32_t)field_msg.hour << ":" << std::setfill('0') << std::setw(2) << (uint32_t)field_msg.minute << ":" << std::setfill('0') << std::setw(2) << (uint32_t)field_msg.second 
                << "." << std::setfill('0') << std::setw(6) << (uint32_t)field_msg.microsecond;
        }
        ROS_DEBUG_STREAM("SickScanMessages::parseLFErecMsg(): LFErec with " << output_msg.fields.size() << " fields:\n" << fields_str.str());

        return true;
    }
    else
    {
        ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg not implemented for Cola-A messages (" << __FILE__ << ":" << __LINE__ << ")");        
    }
    return false;
}
