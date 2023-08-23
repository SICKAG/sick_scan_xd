/*
 * @brief class ScanSegmentParserOutput is the output container for unpacked and converted msgpack and compact data for multiScan136 and picoScan.
 * In case of multiScan136, ScanSegmentParserOutput has 16 groups (layers), each group has 3 echos, each echo has a list of LidarPoint data in catesian coordinates
 * (x, y, z in meter and intensity). In case of picoScan, ScanSegmentParserOutput has 1 layer.
 *
 * Usage example for msgpack data:
 *
 * std::ifstream msgpack_istream("polarscan_testdata_000.msg", std::ios::binary);
 * sick_scansegment_xd::ScanSegmentParserOutput scansegment_output;
 * sick_scansegment_xd::MsgPackParser::Parse(msgpack_istream, scansegment_output);
 *
 * sick_scansegment_xd::MsgPackParser::WriteCSV({ scansegment_output }, "polarscan_testdata_000.csv")
 *
 * for (int groupIdx = 0; groupIdx < scansegment_output.scandata.size(); groupIdx++)
 * {
 * 	 for (int echoIdx = 0; echoIdx < scansegment_output.scandata[groupIdx].size(); echoIdx++)
 * 	 {
 * 	   std::vector<sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint>& scanline = scansegment_output.scandata[groupIdx][echoIdx];
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
#include "sick_scan/sick_ros_wrapper.h"
#include "sick_scansegment_xd/common.h"
#include "sick_scansegment_xd/scansegment_parser_output.h"

/*
* @brief Default constructor of class ScanSegmentParserOutput. 
* ScanSegmentParserOutput is the container for unpacked and converted msgpack and compact data for multiScan136 and picoScan.
* In case of multiScan136, ScanSegmentParserOutput has 16 groups (layers), each group has 3 echos, each echo has a list of LidarPoint data in catesian coordinates
* (x, y, z in meter and intensity). In case of picoScan, ScanSegmentParserOutput has 1 layer.
*/
sick_scansegment_xd::ScanSegmentParserOutput::ScanSegmentParserOutput() : timestamp(""), timestamp_sec(0), timestamp_nsec(0), segmentIndex(0), telegramCnt(0) 
{
}

/*
 * @brief return a formatted timestamp "<sec>.<millisec>".
 * @param[in] sec second part of timestamp
 * @param[in] nsec nanosecond part of timestamp
 * @return "<sec>.<millisec>"
 */
std::string sick_scansegment_xd::Timestamp(uint32_t sec, uint32_t nsec)
{
	std::stringstream timestamp;
	timestamp << sec << "." << std::setfill('0') << std::setw(6) << (nsec / 1000);
	return timestamp.str();
}

/*
 * @brief return a timestamp of the current time (i.e. std::chrono::system_clock::now() formatted by "YYYY-MM-DD hh-mm-ss.msec").
 */
std::string sick_scansegment_xd::Timestamp(const std::chrono::system_clock::time_point& now)
{
	std::time_t cur_time = std::chrono::system_clock::to_time_t(now);
	std::chrono::milliseconds milliseonds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
	struct tm local_time;
	localtime_s(&local_time, &cur_time);
	std::stringstream time_stream;
	time_stream << std::put_time(&local_time, "%F %T") << "." << std::setfill('0') << std::setw(3) << milliseonds.count();
	return time_stream.str();
}

