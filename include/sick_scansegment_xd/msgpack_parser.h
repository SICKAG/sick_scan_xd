#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * @brief msgpack_parser unpacks and parses msgpack data for the sick 3D lidar multiScan136.
 *
 * Usage example:
 *
 * std::ifstream msgpack_istream("polarscan_testdata_000.msg", std::ios::binary);
 * sick_scansegment_xd::ScanSegmentParserOutput msgpack_output;
 * sick_scansegment_xd::MsgPackParser::Parse(msgpack_istream, msgpack_output);
 *
 * sick_scansegment_xd::MsgPackParser::WriteCSV({ msgpack_output }, "polarscan_testdata_000.csv")
 *
 * for (int groupIdx = 0; groupIdx < msgpack_output.scandata.size(); groupIdx++)
 * {
 * 	 for (int echoIdx = 0; echoIdx < msgpack_output.scandata[groupIdx].size(); echoIdx++)
 * 	 {
 * 	   std::vector<sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint>& scanline = msgpack_output.scandata[groupIdx][echoIdx];
 * 	   std::cout << (groupIdx + 1) << ". group, " << (echoIdx + 1) << ". echo: ";
 * 	   for (int pointIdx = 0; pointIdx < scanline.size(); pointIdx++)
 * 	   {
 * 		  sick_scansegment_xd::ScanSegmentParserOutput::PointXYZI& point = scanline[pointIdx];
 * 		  std::cout << (pointIdx > 0 ? "," : "") << "(" << point.x << "," << point.y << "," << point.z << "," << point.i << ")";
 * 	   }
 * 	   std::cout << std::endl;
 * 	 }
 * }
 *
 * Copyright (C) 2020 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2020 SICK AG, Waldkirch
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
 *  Copyright 2020 SICK AG
 *  Copyright 2020 Ing.-Buero Dr. Michael Lehning
 *
 */
#ifndef __SICK_SCANSEGMENT_XD_MSGPACK_PARSER_H
#define __SICK_SCANSEGMENT_XD_MSGPACK_PARSER_H

#include "sick_scan/sick_ros_wrapper.h"
#include <sick_scan/sick_cloud_transform.h>
#include "sick_scan/sick_range_filter.h"
#include "sick_scansegment_xd/common.h"
#include "sick_scansegment_xd/fifo.h"
#include "sick_scansegment_xd/msgpack_validator.h"
#include "sick_scansegment_xd/scansegment_parser_output.h"

namespace sick_scansegment_xd
{
	/*
     * @brief class MsgPackParser unpacks and parses msgpack data for the sick 3D lidar multiScan136.
     */
	class MsgPackParser
	{
	public:

        /*
         * @brief reads a file in binary mode and returns all bytes.
         * @param[in] filepath input file incl. path
         * @param[out] list of bytes
         */
        static std::vector<uint8_t> ReadFile(const std::string& filepath);

        /*
         * @brief unpacks and parses msgpack data from a binary input stream.
         *
         * Usage example:
         *
         * std::vector<uint8_t> msgpack_data = sick_scansegment_xd::MsgPackParser::ReadFile("polarscan_testdata_000.msg");
         * sick_scansegment_xd::ScanSegmentParserOutput msgpack_output;
         * sick_scansegment_xd::MsgPackParser::Parse(msgpack_data, msgpack_output);
         * sick_scansegment_xd::MsgPackParser::WriteCSV({ msgpack_output }, "polarscan_testdata_000.csv")
         *
         * @param[in+out] msgpack_ifstream the binary input stream delivering the binary msgpack data
         * @param[in] msgpack_timestamp receive timestamp of msgpack_data
         * @param[in] add_transform_xyz_rpy Apply an additional transform to the cartesian pointcloud, default: "0,0,0,0,0,0" (i.e. no transform)
         * @param[out] result msgpack data converted to scanlines of type ScanSegmentParserOutput
         * @param[in+out] msgpack_validator_data_collector collects MsgPackValidatorData over N msgpacks
         * @param[in] msgpack_validator msgpack validation, see MsgPackValidator for details
         * @param[in] msgpack_validator_enabled true: check msgpack data for out of bounds and missing scan data, false: no msgpack validation
         * @param[in] discard_msgpacks_not_validated true: msgpacks are discarded if not validated, false: error message if a msgpack is not validated
         * @param[in] use_software_pll true (default): result timestamp from sensor ticks by software pll, false: result timestamp from msg receiving
         * @param[in] verbose true: enable debug output, false: quiet mode
         */
        static bool Parse(const std::vector<uint8_t>& msgpack_data, fifo_timestamp msgpack_timestamp, sick_scan_xd::SickCloudTransform& add_transform_xyz_rpy, ScanSegmentParserOutput& result, 
            sick_scansegment_xd::MsgPackValidatorData& msgpack_validator_data_collector, const sick_scansegment_xd::MsgPackValidator& msgpack_validator = sick_scansegment_xd::MsgPackValidator(), 
            bool msgpack_validator_enabled = false, bool discard_msgpacks_not_validated = false, bool use_software_pll = true, bool verbose = false);

		/*
		 * @brief unpacks and parses msgpack data from a binary input stream.
		 *
		 * Usage example:
		 * 
		 * std::ifstream msgpack_istream("polarscan_testdata_000.msg", std::ios::binary);
		 * sick_scansegment_xd::ScanSegmentParserOutput msgpack_output;
		 * sick_scansegment_xd::MsgPackParser::Parse(msgpack_istream, msgpack_output);
		 *
         * sick_scansegment_xd::MsgPackParser::WriteCSV({ msgpack_output }, "polarscan_testdata_000.csv")
         *
		 * for (int groupIdx = 0; groupIdx < msgpack_output.scandata.size(); groupIdx++)
		 * {
		 * 	 for (int echoIdx = 0; echoIdx < msgpack_output.scandata[groupIdx].size(); echoIdx++)
		 * 	 {
		 * 	   std::vector<sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint>& scanline = msgpack_output.scandata[groupIdx][echoIdx];
		 * 	   std::cout << (groupIdx + 1) << ". group, " << (echoIdx + 1) << ". echo: ";
		 * 	   for (int pointIdx = 0; pointIdx < scanline.size(); pointIdx++)
		 * 	   {
		 * 		  sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint& point = scanline[pointIdx];
		 * 		  std::cout << (pointIdx > 0 ? "," : "") << "(" << point.x << "," << point.y << "," << point.z << "," << point.i << ")";
		 * 	   }
		 * 	   std::cout << std::endl;
		 * 	 }
		 * }
		 *
		 * @param[in+out] msgpack_ifstream the binary input stream delivering the binary msgpack data
         * @param[in] msgpack_timestamp receive timestamp of msgpack_data
         * @param[in] add_transform_xyz_rpy Apply an additional transform to the cartesian pointcloud, default: "0,0,0,0,0,0" (i.e. no transform)
         * @param[out] result msgpack data converted to scanlines of type ScanSegmentParserOutput
         * @param[in+out] msgpack_validator_data_collector collects MsgPackValidatorData over N msgpacks
         * @param[in] msgpack_validator msgpack validation, see MsgPackValidator for details
         * @param[in] msgpack_validator_enabled true: check msgpack data for out of bounds and missing scan data, false: no msgpack validation
         * @param[in] discard_msgpacks_not_validated true: msgpacks are discarded if not validated, false: error message if a msgpack is not validated
         * @param[in] use_software_pll true (default): result timestamp from sensor ticks by software pll, false: result timestamp from msg receiving
         * @param[in] verbose true: enable debug output, false: quiet mode
         */
        static bool Parse(std::istream& msgpack_istream, fifo_timestamp msgpack_timestamp, sick_scan_xd::SickCloudTransform& add_transform_xyz_rpy, ScanSegmentParserOutput& result, 
            sick_scansegment_xd::MsgPackValidatorData& msgpack_validator_data_collector,
            const sick_scansegment_xd::MsgPackValidator& msgpack_validator = sick_scansegment_xd::MsgPackValidator(),
            bool msgpack_validator_enabled = false, bool discard_msgpacks_not_validated = false, 
            bool use_software_pll = true, bool verbose = false);

        /*
         * @brief Returns a hexdump of a msgpack. To get a well formatted json struct from a msgpack,
         * just paste the returned string to https://toolslick.com/conversion/data/messagepack-to-json
         */
        static std::string MsgpackToHexDump(const std::vector<uint8_t>& msgpack_data, bool pretty_print = true);
        
        /*
         * @brief exports msgpack data to csv file.
         *
         * Usage example:
         *
         * std::ifstream msgpack_istream("polarscan_testdata_000.msg", std::ios::binary);
         * sick_scansegment_xd::ScanSegmentParserOutput msgpack_output;
         * sick_scansegment_xd::MsgPackParser::Parse(msgpack_istream, msgpack_output);
         *
         * sick_scansegment_xd::MsgPackParser::WriteCSV({ msgpack_output }, "polarscan_testdata_000.csv")
         *
         * @param[in] result converted msgpack data, output from Parse function
         * @param[in] csvFile name of output csv file incl. optional file path
         * @param[in] overwrite_existing_file if overwrite_existing_file is true and csvFile already exists, the file will be overwritten. otherwise all results are appended to the file.
         */
        static bool WriteCSV(const std::vector<ScanSegmentParserOutput>& results, const std::string& csvFile, bool overwrite_existing_file);

        /*
         * @brief exports x, y, z, intensity, group, echo and message index of msgpack data.
         * @param[in] result converted msgpack data, output from Parse function
         * Note: All output vectors x, y, z, i, group_idx, echo_idx, msg_idx identical size, i.e. it's safe to 
         * assert(x.size() == y.size() && x.size() == z.size() && x.size() == i.size() && x.size() == group_idx.size() && echo_idx.size() == msg_idx.size());
         */
        static bool ExportXYZI(const std::vector<ScanSegmentParserOutput>& results, std::vector<float>& x, std::vector<float>& y, std::vector<float>& z, std::vector<float>& i, std::vector<int>& group_idx, std::vector<int>& echo_idx, std::vector<int>& msg_idx);

        /*
         * @brief return a timestamp of the current time (i.e. std::chrono::system_clock::now() formatted by "YYYY-MM-DD hh-mm-ss.msec").
         */
        static std::string Timestamp(const std::chrono::system_clock::time_point& now = std::chrono::system_clock::now());

        /*
         * @brief return a formatted timestamp "<sec>.<millisec>".
         * @param[in] sec second part of timestamp
         * @param[in] nsec nanosecond part of timestamp
         * @return "<sec>.<millisec>"
         */
        static std::string Timestamp(uint32_t sec, uint32_t nsec);

        /*
         * @brief Returns the tokenized integer of a msgpack key.
         * Example: MsgpackKeyToInt("data") returns 0x11.
         */
        // static int MsgpackKeyToInt(const std::string& key); // replaced by MsgpackKeyToInt preprocessor defines for performance 

        /*
         * @brief Returns the name of a tokenized msgpack key.
         * Example: MsgpackKeyToStr(0x11) returns "data".
         */
        // static std::string MsgpackKeyToStr(int key);

    protected:

        /*
         * @brief Counter for each message (each scandata decoded from msgpack data)
         */
        static int messageCount;
        static int telegramCount;

	};  // class MsgPackParser

}   // namespace sick_scansegment_xd
#endif // __SICK_SCANSEGMENT_XD_MSGPACK_PARSER_H
