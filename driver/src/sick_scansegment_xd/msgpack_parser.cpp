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
 * 		  sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint& point = scanline[pointIdx];
 * 		  std::cout << (pointIdx > 0 ? "," : "") << "(" << point.x << "," << point.y << "," << point.z << "," << point.i << ")";
 * 	   }
 * 	   std::cout << std::endl;
 * 	 }
 * }
 *
 * Copyright (C) 2020,2021 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2020,2021 SICK AG, Waldkirch
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
 *  Copyright 2020,2021 SICK AG
 *  Copyright 2020,2021 Ing.-Buero Dr. Michael Lehning
 *
 */
#include <ctime>
#include <fstream>
#include <msgpack11.hpp>
#include "sick_scan/softwarePLL.h"
#include "sick_scansegment_xd/config.h"
#include "sick_scansegment_xd/msgpack_parser.h"

/** normalizes an angle to [ -PI , +PI ] */
static float normalizeAngle(float angle_rad)
{
	while (angle_rad > (float)(M_PI))
		angle_rad -= (float)(2.0 * M_PI);
	while (angle_rad < (float)(-M_PI))
		angle_rad += (float)(2.0 * M_PI);
	return angle_rad;
}

 /*
  * @brief Counter for each message (each scandata decoded from msgpack data)
  */
int sick_scansegment_xd::MsgPackParser::messageCount = 0;
int sick_scansegment_xd::MsgPackParser::telegramCount = 0;

/*
 * @brief Returns the tokenized integer of a msgpack key.
 * Example: MsgpackKeyToInt("data") returns 0x11.
 */
/* static int MsgpackKeyToInt(const std::string& key)
{
	static std::map<std::string, int> s_msgpack_keys = {
		{"class" , 0x10},
		{"data" , 0x11},
		{"numOfElems" , 0x12},
		{"elemSz" , 0x13},
		{"endian" , 0x14},
		{"elemTypes" , 0x15},
		{"little" , 0x30},
		{"float32" , 0x31},
		{"uint16" , 0x34},
		{"ChannelTheta" , 0x50},
		{"ChannelPhi" , 0x51},
		{"DistValues" , 0x52},
		{"RssiValues" , 0x53},
		{"PropertiesValues" , 0x54},
		{"Scan" , 0x70},
		{"TimestampStart" , 0x71},
		{"TimestampStop" , 0x72},
		{"ThetaStart" , 0x73},
		{"ThetaStop" , 0x74},
		{"ScanNumber" , 0x75},
		{"ModuleId" , 0x76},
		{"BeamCount" , 0x77},
		{"EchoCount" , 0x78},
		{"ScanSegment" , 0x90},
		{"SegmentCounter" , 0x91},
		{"FrameNumber" , 0x92},
		{"Availability" , 0x93},
		{"SenderId" , 0x94},
		{"SegmentSize" , 0x95},
		{"SegmentData" , 0x96},
		{"TelegramCounter" , 0xB0},
		{"TimestampTransmit" , 0xB1}
	};
	return s_msgpack_keys[key];
} */

/*
 * @brief Define msgpack keys by precompiler define (faster than calling sick_scansegment_xd::MsgpackKeyToInt)
 */
#define MsgpackKeyToInt_class             0x10 // sick_scansegment_xd::MsgpackKeyToInt("class")
#define MsgpackKeyToInt_data              0x11 // sick_scansegment_xd::MsgpackKeyToInt("data")
#define MsgpackKeyToInt_numOfElems        0x12 // sick_scansegment_xd::MsgpackKeyToInt("numOfElems")
#define MsgpackKeyToInt_elemSz            0x13 // sick_scansegment_xd::MsgpackKeyToInt("elemSz")
#define MsgpackKeyToInt_endian            0x14 // sick_scansegment_xd::MsgpackKeyToInt("endian")
#define MsgpackKeyToInt_elemTypes         0x15 // sick_scansegment_xd::MsgpackKeyToInt("elemTypes")
#define MsgpackKeyToInt_little            0x30 // sick_scansegment_xd::MsgpackKeyToInt("little")
#define MsgpackKeyToInt_float32           0x31 // sick_scansegment_xd::MsgpackKeyToInt("float32")
#define MsgpackKeyToInt_uint32            0x32 // sick_scansegment_xd::MsgpackKeyToInt("uint32")
#define MsgpackKeyToInt_uint8             0x33 // sick_scansegment_xd::MsgpackKeyToInt("uint8")
#define MsgpackKeyToInt_uint16            0x34 // sick_scansegment_xd::MsgpackKeyToInt("uint16")
#define MsgpackKeyToInt_ChannelTheta      0x50 // sick_scansegment_xd::MsgpackKeyToInt("ChannelTheta")
#define MsgpackKeyToInt_ChannelPhi        0x51 // sick_scansegment_xd::MsgpackKeyToInt("ChannelPhi")
#define MsgpackKeyToInt_DistValues        0x52 // sick_scansegment_xd::MsgpackKeyToInt("DistValues")
#define MsgpackKeyToInt_RssiValues        0x53 // sick_scansegment_xd::MsgpackKeyToInt("RssiValues")
#define MsgpackKeyToInt_PropertiesValues  0x54 // sick_scansegment_xd::MsgpackKeyToInt("PropertiesValues")
#define MsgpackKeyToInt_Scan              0x70 // sick_scansegment_xd::MsgpackKeyToInt("Scan")
#define MsgpackKeyToInt_TimestampStart    0x71 // sick_scansegment_xd::MsgpackKeyToInt("TimestampStart")
#define MsgpackKeyToInt_TimestampStop     0x72 // sick_scansegment_xd::MsgpackKeyToInt("TimestampStop")
#define MsgpackKeyToInt_ThetaStart        0x73 // sick_scansegment_xd::MsgpackKeyToInt("ThetaStart")
#define MsgpackKeyToInt_ThetaStop         0x74 // sick_scansegment_xd::MsgpackKeyToInt("ThetaStop")
#define MsgpackKeyToInt_ScanNumber        0x75 // sick_scansegment_xd::MsgpackKeyToInt("ScanNumber")
#define MsgpackKeyToInt_ModuleId          0x76 // sick_scansegment_xd::MsgpackKeyToInt("ModuleId")
#define MsgpackKeyToInt_BeamCount         0x77 // sick_scansegment_xd::MsgpackKeyToInt("BeamCount")
#define MsgpackKeyToInt_EchoCount         0x78 // sick_scansegment_xd::MsgpackKeyToInt("EchoCount")
#define MsgpackKeyToInt_ScanSegment       0x90 // sick_scansegment_xd::MsgpackKeyToInt("ScanSegment")
#define MsgpackKeyToInt_SegmentCounter    0x91 // sick_scansegment_xd::MsgpackKeyToInt("SegmentCounter")
#define MsgpackKeyToInt_FrameNumber       0x92 // sick_scansegment_xd::MsgpackKeyToInt("FrameNumber")
#define MsgpackKeyToInt_Availability      0x93 // sick_scansegment_xd::MsgpackKeyToInt("Availability")
#define MsgpackKeyToInt_SenderId          0x94 // sick_scansegment_xd::MsgpackKeyToInt("SenderId")
#define MsgpackKeyToInt_SegmentSize       0x95 // sick_scansegment_xd::MsgpackKeyToInt("SegmentSize")
#define MsgpackKeyToInt_SegmentData       0x96 // sick_scansegment_xd::MsgpackKeyToInt("SegmentData")
#define MsgpackKeyToInt_LayerId           0xA0 // sick_scansegment_xd::MsgpackKeyToInt("LayerId")
#define MsgpackKeyToInt_TelegramCounter   0xB0 // sick_scansegment_xd::MsgpackKeyToInt("TelegramCounter")
#define MsgpackKeyToInt_TimestampTransmit 0xB1 // sick_scansegment_xd::MsgpackKeyToInt("TimestampTransmit")
#define MsgpackKeyToInt_MaxValue          0xB2 // max allowed value of a msgpack key

/*
 * @brief static defined key values of type msgpack11::MsgPack, avoids new and delete for each call to msgpack11::MsgPack::object::find()
 */
class MsgPackKeyValues
{
public:
	MsgPackKeyValues()
	{
		values = std::vector<msgpack11::MsgPack>(MsgpackKeyToInt_MaxValue);
		for (int n = 0; n < MsgpackKeyToInt_MaxValue; n++)
			values[n] = msgpack11::MsgPack(n);
	}
	std::vector<msgpack11::MsgPack> values;
};
static MsgPackKeyValues s_msgpack_keys;

/*
 * @brief Returns the name of a tokenized msgpack key.
 * Example: MsgpackKeyToStr(0x11) returns "data".
 */
/* static std::string MsgpackKeyToStr(int key)
{
	static std::map<int, std::string> s_msgpack_keys = {
		{0x10 , "class"},
		{0x11 , "data"},
		{0x12 , "numOfElems"},
		{0x13 , "elemSz"},
		{0x14 , "endian"},
		{0x15 , "elemTypes"},
		{0x30 , "little"},
		{0x31 , "float32"},
		{0x34 , "uint16"},
		{0x50 , "ChannelTheta"},
		{0x51 , "ChannelPhi"},
		{0x52 , "DistValues"},
		{0x53 , "RssiValues"},
		{0x54 , "PropertiesValues"},
		{0x70 , "Scan"},
		{0x71 , "TimestampStart"},
		{0x72 , "TimestampStop"},
		{0x73 , "ThetaStart"},
		{0x74 , "ThetaStop"},
		{0x75 , "ScanNumber"},
		{0x76 , "ModuleId"},
		{0x77 , "BeamCount"},
		{0x78 , "EchoCount"},
		{0x90 , "ScanSegment"},
		{0x91 , "SegmentCounter"},
		{0x92 , "FrameNumber"},
		{0x93 , "Availability"},
		{0x94 , "SenderId"},
		{0x95 , "SegmentSize"},
		{0x96 , "SegmentData"},
		{0xB0 , "TelegramCounter"},
		{0xB1 , "TimestampTransmit"}
	};
	return s_msgpack_keys[key];
} */

/*
 * Prints MsgPack raw data to string (debug and development only)
 */
static std::string printMsgPack(const msgpack11::MsgPack& msg)
{
	std::stringstream s;
	if (msg.is_number())
		s << msg.number_value();
	if (msg.is_string())
		s << "\"" << msg.string_value() << "\"";
	if (msg.is_bool())
		s << (msg.bool_value() ? "true" : "false");
	if (!msg.array_items().empty())
	{
		s << "array[";
		for (int n = 0; n < msg.array_items().size(); n++)
			s << (n > 0 ? "," : "") << printMsgPack(msg.array_items()[n]);
		s << "]";
	}
	if (!msg.binary_items().empty())
	{
		s << "binary[";
		for (int n = 0; n < msg.binary_items().size(); n++)
			s << (n > 0 ? "," : "") << printMsgPack(msg.binary_items()[n]);
		s << "]";
	}
	if (!msg.object_items().empty())
	{
		s << "object{";
		int n = 0;
		for (msgpack11::MsgPack::object::const_iterator iter_item = msg.object_items().cbegin(); iter_item != msg.object_items().cend(); iter_item++, n++)
		{
			s << (n > 0 ? "," : "") << "\"" << printMsgPack(iter_item->first) << "\":\"" << printMsgPack(iter_item->second) << "\"";
		}
		s << "}";
	}
	return s.str();
}

/*
 * @brief class MsgPackElement is a quadruple of MsgPack
 * for data, size, type and endianess of a MsgPack element.
 * Note: For performance reasons, the pointer to the objects
 * are stored. The caller has to insure objects are allocated
 * properly while accessing MsgPackElement members.
 */
class MsgPackElement
{
public:
	MsgPackElement() {}
	MsgPackElement(const msgpack11::MsgPack::object& object_items)
	{
		data = &object_items.find(s_msgpack_keys.values[MsgpackKeyToInt_data])->second;
		elemSz = &object_items.find(s_msgpack_keys.values[MsgpackKeyToInt_elemSz])->second;
		endian = &object_items.find(s_msgpack_keys.values[MsgpackKeyToInt_endian])->second;
		elemTypes = &object_items.find(s_msgpack_keys.values[MsgpackKeyToInt_elemTypes])->second;
		if (elemTypes->is_array())
			elemTypes = &elemTypes->array_items()[0];
	}
	const msgpack11::MsgPack* data;
	const msgpack11::MsgPack* elemSz;
	const msgpack11::MsgPack* elemTypes;
	const msgpack11::MsgPack* endian;
};

/*
 * @brief class MsgPackToFloat32VectorConverter decodes a MsgPackElement into an array of float data.
 */
class MsgPackToFloat32VectorConverter
{
public:
	MsgPackToFloat32VectorConverter() {}
	MsgPackToFloat32VectorConverter(const MsgPackElement& msgpack, bool dstIsBigEndian)
	{
		union FLOAT_4BYTE_UNION
		{
			uint8_t u8_bytes[4];
			uint32_t u32_bytes;
			float value;
		};

		union UINT_2BYTE_UNION
		{
			uint8_t u8_bytes[2];
			uint16_t u16_bytes;
		};

		assert(msgpack.data && msgpack.elemSz && msgpack.elemTypes && msgpack.endian
			&& msgpack.elemSz->is_number()
			&& msgpack.data->binary_items().size() > 0
			&& ((msgpack.data->binary_items().size()) % (msgpack.elemSz->int_value())) == 0);
		assert((msgpack.elemSz->int_value() == 4 && msgpack.elemTypes->int_value() == MsgpackKeyToInt_float32)
			|| (msgpack.elemSz->int_value() == 2 && msgpack.elemTypes->int_value() == MsgpackKeyToInt_uint16));

		bool srcIsBigEndian = (msgpack.endian->string_value() == "big");
		const msgpack11::MsgPack::binary& binary_items = msgpack.data->binary_items();
		int elem_size = msgpack.elemSz->int_value();
		int binary_size = (int)(binary_items.size());
		m_data.reserve(binary_size / elem_size);
		if (msgpack.elemSz->int_value() == 4 && msgpack.elemTypes->int_value() == MsgpackKeyToInt_float32) // Decode 4 bytes as float
		{
			FLOAT_4BYTE_UNION elem_buffer;
			if (srcIsBigEndian == dstIsBigEndian) // src and dst have identical endianess: reinterprete 4 bytes as float
			{
				for (int n = 0; n < binary_size; n += 4)
				{
					elem_buffer.u32_bytes = *((uint32_t*)(&binary_items[n]));
					m_data.push_back(elem_buffer.value);
				}
			}
			else // src and dst have different endianess: reorder 4 bytes and interprete as float
			{
				for (int n = 0; n < binary_size; n += 4)
				{
					elem_buffer.u8_bytes[3] = binary_items[n + 0];
					elem_buffer.u8_bytes[2] = binary_items[n + 1];
					elem_buffer.u8_bytes[1] = binary_items[n + 2];
					elem_buffer.u8_bytes[0] = binary_items[n + 3];
					m_data.push_back(elem_buffer.value);
				}
			}
		}
		else if (msgpack.elemSz->int_value() == 2 && msgpack.elemTypes->int_value() == MsgpackKeyToInt_uint16) // Decode 2 bytes as uint16 and convert to float
		{
			UINT_2BYTE_UNION elem_buffer;
			if (srcIsBigEndian == dstIsBigEndian) // src and dst have identical endianess: reinterprete 2 bytes as uint16 and convert to float
			{
				for (int n = 0; n < binary_size; n += 2)
				{
					elem_buffer.u16_bytes = *((uint16_t*)(&binary_items[n]));
					m_data.push_back((float)elem_buffer.u16_bytes);
				}
			}
			else // src and dst have different endianess: reorder 2 bytes (uint16) and convert to float
			{
				for (int n = 0; n < binary_size; n += 2)
				{
					elem_buffer.u8_bytes[1] = binary_items[n + 0];
					elem_buffer.u8_bytes[0] = binary_items[n + 1];
					m_data.push_back((float)elem_buffer.u16_bytes);
				}
			}
		}
		else
		{
			std::cerr << "## ERROR MsgPackToFloat32VectorConverter: invalid or unsupported elemSz or elemTypes:" << std::endl
				<< "    msgpack.data = " << (msgpack.data ? printMsgPack(*msgpack.data) : "NULL") << std::endl
				<< "    msgpack.elemSz = " << (msgpack.elemSz ? printMsgPack(*msgpack.elemSz) : "NULL") << std::endl
				<< "    msgpack.elemTypes = " << (msgpack.elemTypes ? printMsgPack(*msgpack.elemTypes) : "NULL") << std::endl
				<< "    msgpack.endian = " << (msgpack.endian ? printMsgPack(*msgpack.endian) : "NULL") << std::endl;
		}
	}
	std::string print(void)
	{
		std::stringstream s;
		if (!m_data.empty())
		{
			s << m_data[0];
			for(int n = 1; n < m_data.size(); n++)
				s << "," << m_data[n];
		}
		return s.str();
	}
    float rad2deg(float angle) const { return angle * (float)(180.0 / M_PI); }
	std::string printRad2Deg(void)
	{
		std::stringstream s;
		if (!m_data.empty())
		{
			s << rad2deg(m_data[0]);
			for(int n = 1; n < m_data.size(); n++)
				s << "," << rad2deg(m_data[n]);
		}
		return s.str();
	}
	std::vector<float>& data(void)
	{
		return m_data;
	}
protected:
	std::vector<float> m_data;
};

/*
 * @brief reads a file in binary mode and returns all bytes.
 * @param[in] filepath input file incl. path
 * @param[out] list of bytes
 */
std::vector<uint8_t> sick_scansegment_xd::MsgPackParser::ReadFile(const std::string& filepath)
{
	std::ifstream file_istream(filepath, std::ios::binary);
	if (file_istream.is_open())
	    return std::vector<uint8_t>((std::istreambuf_iterator<char>(file_istream)), std::istreambuf_iterator<char>());
	return std::vector<uint8_t>();
}

/*
 * @brief Returns a hexdump of a msgpack. To get a well formatted json struct from a msgpack,
 * just paste the returned string to https://toolslick.com/conversion/data/messagepack-to-json
 */
std::string sick_scansegment_xd::MsgPackParser::MsgpackToHexDump(const std::vector<uint8_t>& msgpack_data, bool pretty_print)
{
	std::stringstream msgpack_hexdump;
	for (size_t n = 0; n < msgpack_data.size(); n++)
	{
		msgpack_hexdump << std::setfill('0') << std::setw(2) << std::hex << (int)(msgpack_data[n] & 0xFF);
		if(pretty_print)
		{
			if ((n % 20) == 19)
				msgpack_hexdump << std::endl;
			else
				msgpack_hexdump << " ";
		}
	}
	return msgpack_hexdump.str();
}

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
bool sick_scansegment_xd::MsgPackParser::Parse(const std::vector<uint8_t>& msgpack_data, fifo_timestamp msgpack_timestamp, 
    sick_scan_xd::SickCloudTransform& add_transform_xyz_rpy, ScanSegmentParserOutput& result,
    sick_scansegment_xd::MsgPackValidatorData& msgpack_validator_data_collector, const sick_scansegment_xd::MsgPackValidator& msgpack_validator,
	bool msgpack_validator_enabled, bool discard_msgpacks_not_validated,
	bool use_software_pll, bool verbose)
{
	// To debug, print and visual msgpack_data, just paste hex dump to
	// https://toolslick.com/conversion/data/messagepack-to-json
	// to get a well formatted json struct.
	// std::string msgpack_hexdump = MsgpackToHexDump(msgpack_data, false);
	// std::ofstream msgpack_dumpfile("msgpack-debug.msgpack.hex");
	// if(msgpack_dumpfile.is_open())
	//     msgpack_dumpfile << msgpack_hexdump;
	// std::string msgpack_hexdump = MsgpackToHexDump(msgpack_data, true);
	// std::cout << std::endl << "MsgPack hexdump: " << std::endl << msgpack_hexdump << std::endl << std::endl;
	std::string msgpack_string((char*)msgpack_data.data(), msgpack_data.size());
	std::istringstream msgpack_istream(msgpack_string);
	return Parse(msgpack_istream, msgpack_timestamp, add_transform_xyz_rpy, result, msgpack_validator_data_collector, msgpack_validator, msgpack_validator_enabled, discard_msgpacks_not_validated, use_software_pll, verbose);
}

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
 * @param[in] discard_msgpacks_not_validated true: msgpacks are discarded if not validated, false: error message if a msgpack is not validated
 * @param[in] msgpack_validator msgpack validation, see MsgPackValidator for details
 * @param[in] msgpack_validator_enabled true: check msgpack data for out of bounds and missing scan data, false: no msgpack validation
 * @param[in+out] msgpack_validator_data_collector collects MsgPackValidatorData over N msgpacks
 * @param[in] use_software_pll true (default): result timestamp from sensor ticks by software pll, false: result timestamp from msg receiving
 * @param[in] verbose true: enable debug output, false: quiet mode
 */
bool sick_scansegment_xd::MsgPackParser::Parse(std::istream& msgpack_istream, fifo_timestamp msgpack_timestamp, 
	sick_scan_xd::SickCloudTransform& add_transform_xyz_rpy, ScanSegmentParserOutput& result,
    sick_scansegment_xd::MsgPackValidatorData& msgpack_validator_data_collector, 
	const sick_scansegment_xd::MsgPackValidator& msgpack_validator,
	bool msgpack_validator_enabled, bool discard_msgpacks_not_validated,
	bool use_software_pll, bool verbose)
{
	int64_t systemtime_nanoseconds = msgpack_timestamp.time_since_epoch().count();
	uint32_t systemtime_sec = (uint32_t)(systemtime_nanoseconds / 1000000000);  // seconds part of timestamp
	uint32_t systemtime_nsec = (uint32_t)(systemtime_nanoseconds % 1000000000); // nanoseconds part of timestamp
	result.timestamp = sick_scansegment_xd::Timestamp(systemtime_sec, systemtime_nsec); // Timestamp(std::chrono::system_clock::now()); // default timestamp: msgpack receive time, overwritten by timestamp from msgpack data
	result.timestamp_sec = systemtime_sec;
	result.timestamp_nsec = systemtime_nsec;
	int32_t segment_idx = messageCount++; // default value: counter for each message (each scandata decoded from msgpack data), overwritten by msgpack data
	int32_t telegram_cnt = telegramCount++; // default value: counter for each message (each scandata decoded from msgpack data), overwritten by msgpack data
	msgpack11::MsgPack msg_unpacked;
	try
	{
		// Unpack the binary msgpack data
		std::string msg_parse_error;
		msg_unpacked = msgpack11::MsgPack::parse(msgpack_istream, msg_parse_error);
		if (!msg_parse_error.empty())
		{
			ROS_ERROR_STREAM("## ERROR msgpack11::MsgPack::parse(): " << msg_parse_error);
			return false;
		}
		// std::cout << printMsgPack(msg_unpacked) << std::endl << std::endl;
	}
	catch(const std::exception & exc)
	{
		ROS_ERROR_STREAM("## ERROR msgpack11::MsgPack::parse(): exception " << exc.what());
		return false;
	}

	// Get endianess of the system (destination target)
	bool dstIsBigEndian = sick_scansegment_xd::Config::SystemIsBigEndian();

	// Parse the unpacked msgpack data, see sick_scansegment_xd/python/polarscan_reader_test/polarscan_receiver_test.py for multiScan136 message format
    // and https://github.com/SICKAG/msgpack11/blob/master/msgpack11.hpp or https://github.com/SICKAG/msgpack11/blob/master/example.cpp
	// for details about decoding and paring MsgPack data.
	try
	{
		sick_scansegment_xd::MsgPackValidatorData msgpack_validator_data;
		// std::cout << "root_object_items: " << printMsgPack(msg_unpacked.object_items()) << std::endl;
		msgpack11::MsgPack::object::const_iterator root_data_iter = msg_unpacked.object_items().find(s_msgpack_keys.values[MsgpackKeyToInt_data]);
		if (root_data_iter == msg_unpacked.object_items().end())
		{
			ROS_WARN_STREAM("## ERROR MsgPackParser::Parse(): \"data\" not found");
			return false;
		}
		const msgpack11::MsgPack& root_data = root_data_iter->second;
		msgpack11::MsgPack::object::const_iterator group_data_iter = root_data.object_items().find(s_msgpack_keys.values[MsgpackKeyToInt_SegmentData]);
		msgpack11::MsgPack::object::const_iterator timestamp_data_iter = root_data.object_items().find(s_msgpack_keys.values[MsgpackKeyToInt_TimestampTransmit]);
		if (group_data_iter == root_data.object_items().end() || timestamp_data_iter == root_data.object_items().end())
		{
			ROS_WARN_STREAM("## ERROR MsgPackParser::Parse(): \"SegmentData\" and/or \"TimestampTransmit\" not found");
			return false;
		}
		const msgpack11::MsgPack& group_data = group_data_iter->second;
		const msgpack11::MsgPack& timestamp_data = timestamp_data_iter->second;
		if (use_software_pll && timestamp_data.is_number())
		{
			// result.timestamp = std::to_string(timestamp_data.int64_value());
			// Calculate system time from sensor ticks using SoftwarePLL
			// result.timestamp = std::to_string(timestamp_data.int64_value());
			SoftwarePLL& software_pll = SoftwarePLL::instance();
			uint32_t curtick = timestamp_data.int32_value();
			software_pll.updatePLL(systemtime_sec, systemtime_nsec, curtick, curtick);
			if (software_pll.IsInitialized())
			{
				uint32_t pll_sec = 0, pll_nsec = 0;
				software_pll.getCorrectedTimeStamp(pll_sec, pll_nsec, curtick);
				result.timestamp = sick_scansegment_xd::Timestamp(pll_sec, pll_nsec);
				result.timestamp_sec = pll_sec;
				result.timestamp_nsec = pll_nsec;
				if (verbose)
					ROS_INFO_STREAM("MsgPackParser::Parse(): sensor_ticks=" << curtick << ", system_time=" << sick_scansegment_xd::Timestamp(systemtime_sec, systemtime_nsec) << " sec, timestamp=" << result.timestamp << " sec");
			}
			else if (verbose)
				ROS_INFO_STREAM("MsgPackParser::Parse(): sensor_ticks=" << curtick << ", system_time=" << sick_scansegment_xd::Timestamp(systemtime_sec, systemtime_nsec) << " sec, SoftwarePLL not yet initialized");
		}
		msgpack11::MsgPack::object::const_iterator segment_counter_iter = root_data.object_items().find(s_msgpack_keys.values[MsgpackKeyToInt_SegmentCounter]);
		if (segment_counter_iter == root_data.object_items().end())
		{
			ROS_WARN_STREAM("## ERROR MsgPackParser::Parse(): \"SegmentCounter\" not found");
			return false;
		}
		const msgpack11::MsgPack& segment_idx_data = segment_counter_iter->second;
		segment_idx = segment_idx_data.int32_value();

		msgpack11::MsgPack::object::const_iterator telegram_counter_iter = root_data.object_items().find(s_msgpack_keys.values[MsgpackKeyToInt_TelegramCounter]);
		if (telegram_counter_iter != root_data.object_items().end())
		{
			const msgpack11::MsgPack& telegram_cnt_data = telegram_counter_iter->second;
			telegramCount = telegram_cnt_data.int32_value();
			telegram_cnt = telegramCount;
		}

		// std::cout << "root_data: " << printMsgPack(root_data) << std::endl << "root_data.array_items().size(): " << root_data.array_items().size() << ", root_data.object_items().size(): " << root_data.object_items().size() << std::endl;
		// std::cout << "group_data.array_items().size(): " << group_data.array_items().size() << ", group_data.object_items().size(): " << group_data.object_items().size() << std::endl;
		result.scandata.reserve(group_data.array_items().size());
		for (int groupIdx = 0; groupIdx < group_data.array_items().size(); groupIdx++)
		{
			// Get ChannelPhi, ChannelTheta, DistValues and RssiValues for each group
			const msgpack11::MsgPack& groupMsg = group_data.array_items()[groupIdx];
			// std::cout << "groupMsg: " << printMsgPack(groupMsg) << std::endl << "groupMsg.array_items().size(): " << groupMsg.array_items().size() << ", groupMsg.object_items().size(): " << groupMsg.object_items().size() << std::endl;
			msgpack11::MsgPack::object::const_iterator dataMsg = groupMsg.object_items().find(s_msgpack_keys.values[MsgpackKeyToInt_data]);
			if (dataMsg == groupMsg.object_items().end())
			{
				continue; // ok if missing
			}
			msgpack11::MsgPack::object::const_iterator echoCountMsg = dataMsg->second.object_items().find(s_msgpack_keys.values[MsgpackKeyToInt_EchoCount]);
			msgpack11::MsgPack::object::const_iterator channelPhiMsg = dataMsg->second.object_items().find(s_msgpack_keys.values[MsgpackKeyToInt_ChannelPhi]);
			msgpack11::MsgPack::object::const_iterator channelThetaMsg = dataMsg->second.object_items().find(s_msgpack_keys.values[MsgpackKeyToInt_ChannelTheta]);
			msgpack11::MsgPack::object::const_iterator distValuesMsg = dataMsg->second.object_items().find(s_msgpack_keys.values[MsgpackKeyToInt_DistValues]);
			msgpack11::MsgPack::object::const_iterator rssiValuesMsg = dataMsg->second.object_items().find(s_msgpack_keys.values[MsgpackKeyToInt_RssiValues]);
			msgpack11::MsgPack::object::const_iterator timestampStartMsg = dataMsg->second.object_items().find(s_msgpack_keys.values[MsgpackKeyToInt_TimestampStart]);
			msgpack11::MsgPack::object::const_iterator timestampStopMsg = dataMsg->second.object_items().find(s_msgpack_keys.values[MsgpackKeyToInt_TimestampStop]);
			msgpack11::MsgPack::object::const_iterator propertiesMsg = dataMsg->second.object_items().find(s_msgpack_keys.values[MsgpackKeyToInt_PropertiesValues]);
			if (echoCountMsg == dataMsg->second.object_items().end() ||
				channelPhiMsg == dataMsg->second.object_items().end() || channelThetaMsg == dataMsg->second.object_items().end() ||
				distValuesMsg == dataMsg->second.object_items().end() || rssiValuesMsg == dataMsg->second.object_items().end() ||
				timestampStartMsg == dataMsg->second.object_items().end() || timestampStopMsg == dataMsg->second.object_items().end())
			{
				ROS_WARN_STREAM("## ERROR MsgPackParser::Parse(): Entries in data segment missing");
				continue;
			}
			uint32_t u32TimestampStart = timestampStartMsg->second.uint32_value();
			uint32_t u32TimestampStop = timestampStopMsg->second.uint32_value();
			uint32_t u32TimestampStart_sec = 0, u32TimestampStart_nsec = 0;
			uint32_t u32TimestampStop_sec = 0, u32TimestampStop_nsec = 0;
			if (use_software_pll && SoftwarePLL::instance().IsInitialized())
			{
				SoftwarePLL& software_pll = SoftwarePLL::instance();
				software_pll.getCorrectedTimeStamp(u32TimestampStart_sec, u32TimestampStart_nsec, u32TimestampStart);
				software_pll.getCorrectedTimeStamp(u32TimestampStop_sec, u32TimestampStop_nsec, u32TimestampStop);
			}

			// Get data, elemSz, elemTypes and endian for each MsgPack object
			MsgPackElement channelPhiMsgElement(channelPhiMsg->second.object_items());
			MsgPackElement channelThetaMsgElement(channelThetaMsg->second.object_items());
			std::vector<MsgPackElement> distValuesDataMsg(distValuesMsg->second.array_items().size());
			std::vector<MsgPackElement> rssiValuesDataMsg(rssiValuesMsg->second.array_items().size());
			for (int n = 0; n < distValuesMsg->second.array_items().size(); n++)
				distValuesDataMsg[n] = MsgPackElement(distValuesMsg->second.array_items()[n].object_items());
			for (int n = 0; n < rssiValuesMsg->second.array_items().size(); n++)
				rssiValuesDataMsg[n] = MsgPackElement(rssiValuesMsg->second.array_items()[n].object_items());
      
			// Get optional property values
			std::vector<std::vector<uint8_t>> propertyValues; // uint8_t property = propertyValues[echoIdx][pointIdx]
			if (propertiesMsg != dataMsg->second.object_items().end()) // property values available
			{
				propertyValues = std::vector<std::vector<uint8_t>>(propertiesMsg->second.array_items().size());
				for (int n = 0; n < propertiesMsg->second.array_items().size(); n++)
				{
					const MsgPackElement& propertyMsgPackElement = MsgPackElement(propertiesMsg->second.array_items()[n].object_items());
					propertyValues[n] = std::vector<uint8_t>(propertyMsgPackElement.data->binary_items().size(), 0);
					if (propertyMsgPackElement.elemSz->int_value() == 1 && propertyMsgPackElement.elemTypes->int_value() == MsgpackKeyToInt_uint8 && propertyMsgPackElement.data->binary_items().size() > 0)
					{
						for(int m = 0; m < propertyValues[n].size(); m++)
						{
							propertyValues[n][m] = propertyMsgPackElement.data->binary_items()[m];
						}
					}
					else
					{
						ROS_WARN_STREAM("## ERROR MsgPackParser::Parse(): invalid property array");
					}
				}
			}

			// Convert all data to float values
			int iEchoCount = echoCountMsg->second.int32_value();
			MsgPackToFloat32VectorConverter channelPhi(channelPhiMsgElement, dstIsBigEndian);
			MsgPackToFloat32VectorConverter channelTheta(channelThetaMsgElement, dstIsBigEndian);
			std::vector<MsgPackToFloat32VectorConverter> distValues(distValuesDataMsg.size());
			std::vector<MsgPackToFloat32VectorConverter> rssiValues(rssiValuesDataMsg.size());
			for (int n = 0; n < distValuesDataMsg.size(); n++)
				distValues[n] = MsgPackToFloat32VectorConverter(distValuesDataMsg[n], dstIsBigEndian);
			for (int n = 0; n < rssiValuesDataMsg.size(); n++)
				rssiValues[n] = MsgPackToFloat32VectorConverter(rssiValuesDataMsg[n], dstIsBigEndian);
			assert(channelPhi.data().size() == 1 && channelTheta.data().size() > 0 && distValues.size() == iEchoCount && rssiValues.size() == iEchoCount);

      // Check optional propertyValues: if available, we expect as many properties as we have points
			int numProperties = propertyValues.size();
			for (int n = 0; n < numProperties; n++)
			{
				// ROS_DEBUG_STREAM("MsgPackParser::Parse(): " << (distValues[n].data().size()) << " dist values, " << (rssiValues[n].data().size()) << " rssi values, " << (propertyValues[n].size()) << " property values (" << (n+1) << ". echo)");
				if (propertyValues[n].size() != distValues[n].data().size())
				{
					ROS_WARN_STREAM("## ERROR MsgPackParser::Parse(): invalid property values for echo " << n
						<< ", property size = " << propertyValues[n].size()
						<< ", dist size = " << distValues[n].data().size());
				}
			}

			// Convert to cartesian coordinates
			result.scandata.push_back(sick_scansegment_xd::ScanSegmentParserOutput::Scangroup());
			result.scandata.back().timestampStart_sec = u32TimestampStart_sec;
			result.scandata.back().timestampStart_nsec = u32TimestampStart_nsec;
			result.scandata.back().timestampStop_sec = u32TimestampStop_sec;
			result.scandata.back().timestampStop_nsec = u32TimestampStop_nsec;
			iEchoCount = std::min((int)distValuesDataMsg.size(), iEchoCount);
			iEchoCount = std::min((int)rssiValuesDataMsg.size(), iEchoCount);
			std::vector<sick_scansegment_xd::ScanSegmentParserOutput::Scanline>& groupData = result.scandata.back().scanlines;
			groupData.reserve(iEchoCount);
			int iPointCount = (int)channelTheta.data().size();
			// Precompute sin and cos values of azimuth and elevation
			float elevation = -channelPhi.data()[0]; // elevation must be negated, a positive pitch-angle yields negative z-coordinates
			float cos_elevation = std::cos(elevation);
			float sin_elevation = std::sin(elevation);
			std::vector<float> cos_azimuth(iPointCount);
			std::vector<float> sin_azimuth(iPointCount);
			std::vector<uint64_t> lut_lidar_timestamp_microsec(iPointCount);
			for (int pointIdx = 0; pointIdx < iPointCount; pointIdx++)
			{
				float azimuth = channelTheta.data()[pointIdx] + add_transform_xyz_rpy.azimuthOffset();
				cos_azimuth[pointIdx] = std::cos(azimuth);
				sin_azimuth[pointIdx] = std::sin(azimuth);
				lut_lidar_timestamp_microsec[pointIdx] = ((pointIdx * (u32TimestampStop - u32TimestampStart)) / (iPointCount - 1)) + u32TimestampStart;
				// if (pointIdx > 0)
				//     SCANSEGMENT_XD_DEBUG_STREAM("azimuth[" << pointIdx << "] = " << (azimuth * 180 / M_PI) << " [deg], delta_azimuth = " << ((channelTheta.data[pointIdx] - channelTheta.data[pointIdx-1]) * 180 / M_PI) << " [deg]");
			}                 
			for (int echoIdx = 0; echoIdx < iEchoCount; echoIdx++)
			{
				assert(iPointCount == channelTheta.data().size() && iPointCount == distValues[echoIdx].data().size() && iPointCount == rssiValues[echoIdx].data().size());
				groupData.push_back(sick_scansegment_xd::ScanSegmentParserOutput::Scanline());
				sick_scansegment_xd::ScanSegmentParserOutput::Scanline& scanline = groupData.back();
				scanline.points.reserve(iPointCount);
				int numPropertiesForThisEcho = propertyValues[echoIdx].size();  // number of properties for this echo index
				for (int pointIdx = 0; pointIdx < iPointCount; pointIdx++)
				{
     			    

					float dist = 0.001f * distValues[echoIdx].data()[pointIdx]; // convert distance to meter
					float intensity = rssiValues[echoIdx].data()[pointIdx];
					float x = dist * cos_azimuth[pointIdx] * cos_elevation;
					float y = dist * sin_azimuth[pointIdx] * cos_elevation;
					float z = dist * sin_elevation;

					uint8_t reflectorbit = 0;
					if (pointIdx < numPropertiesForThisEcho)  // grab reflector bit from properties for this echo and point
					{
					    reflectorbit |= ((propertyValues[echoIdx][pointIdx]) & 0x01); // reflector bit is set, if a reflector is detected on any number of echos
					}
					add_transform_xyz_rpy.applyTransform(x, y, z);
					float azimuth = channelTheta.data()[pointIdx];
					float azimuth_norm = normalizeAngle(azimuth);
					if (msgpack_validator_enabled)
					{
						msgpack_validator_data.update(echoIdx, segment_idx, azimuth_norm, elevation);
						msgpack_validator_data_collector.update(echoIdx, segment_idx, azimuth_norm, elevation);
					}
					uint64_t lidar_timestamp_microsec = lut_lidar_timestamp_microsec[pointIdx];
					scanline.points.push_back(sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint(x, y, z, intensity, dist, azimuth, elevation, groupIdx, echoIdx, pointIdx, lidar_timestamp_microsec, reflectorbit));
				}
			}

			// debug output
			if (verbose)
			{
				ROS_INFO_STREAM((groupIdx + 1) << ". group: EchoCount = " << iEchoCount);
				ROS_INFO_STREAM((groupIdx + 1) << ". group: phi (elevation, rad) = [" << channelPhi.print() << "], " << channelPhi.data().size() << " element");
				ROS_INFO_STREAM((groupIdx + 1) << ". group: phi (elevation, deg) = [" << channelPhi.printRad2Deg() << "], " << channelPhi.data().size() << " element");
				ROS_INFO_STREAM((groupIdx + 1) << ". group: theta (azimuth, rad) = [" << channelTheta.print() << "], " << channelTheta.data().size() << " elements");
				ROS_INFO_STREAM((groupIdx + 1) << ". group: theta (azimuth, deg) = [" << channelTheta.printRad2Deg() << "], " << channelTheta.data().size() << " elements");
				ROS_INFO_STREAM((groupIdx + 1) << ". group: timestampStart = " << u32TimestampStart << " = " << sick_scansegment_xd::Timestamp(u32TimestampStart_sec, u32TimestampStart_nsec));
				ROS_INFO_STREAM((groupIdx + 1) << ". group: timestampStop = " << u32TimestampStop << " = " << sick_scansegment_xd::Timestamp(u32TimestampStop_sec, u32TimestampStop_nsec));
				for (int n = 0; n < distValues.size(); n++)
					ROS_INFO_STREAM((groupIdx + 1) << ". group: dist[" << n << "] = [" << distValues[n].print() << "], " << distValues[n].data().size() << " elements");
				for (int n = 0; n < rssiValues.size(); n++)
					ROS_INFO_STREAM((groupIdx + 1) << ". group: rssi[" << n << "] = [" << rssiValues[n].print() << "], " << rssiValues[n].data().size() << " elements");
				// std::cout << (groupIdx + 1) << ". group: ChannelPhiMsg.data = " << printMsgPack(*channelPhiMsgElement.data) << std::endl;
				// std::cout << (groupIdx + 1) << ". group: ChannelPhiMsg.elemSz = " << printMsgPack(*channelPhiMsgElement.elemSz) << std::endl;
				// std::cout << (groupIdx + 1) << ". group: ChannelPhiMsg.elemType = " << printMsgPack(*channelPhiMsgElement.elemTypes) << std::endl;
				// std::cout << (groupIdx + 1) << ". group: ChannelPhiMsg.endian = " << printMsgPack(*channelPhiMsgElement.endian) << std::endl;
				// std::cout << (groupIdx + 1) << ". group: ChannelThetaMsg.data = " << printMsgPack(*channelThetaMsgElement.data) << std::endl;
				// std::cout << (groupIdx + 1) << ". group: ChannelThetaMsg.elemSz = " << printMsgPack(*channelThetaMsgElement.elemSz) << std::endl;
				// std::cout << (groupIdx + 1) << ". group: ChannelThetaMsg.elemType = " << printMsgPack(*channelThetaMsgElement.elemTypes) << std::endl;
				// std::cout << (groupIdx + 1) << ". group: ChannelThetaMsg.endian = " << printMsgPack(*channelThetaMsgElement.endian) << std::endl;
				// for (int n = 0; n < distValuesDataMsg.size(); n++)
				// {
				// 	 std::cout << (groupIdx + 1) << ". group: DistValuesDataMsg[" << n << "].data = " << printMsgPack(*distValuesDataMsg[n].data) << std::endl;
				// 	 std::cout << (groupIdx + 1) << ". group: DistValuesDataMsg[" << n << "].elemSz = " << printMsgPack(*distValuesDataMsg[n].elemSz) << std::endl;
				// 	 std::cout << (groupIdx + 1) << ". group: DistValuesDataMsg[" << n << "].elemTypes = " << printMsgPack(*distValuesDataMsg[n].elemTypes) << std::endl;
				// 	 std::cout << (groupIdx + 1) << ". group: DistValuesDataMsg[" << n << "].endian = " << printMsgPack(*distValuesDataMsg[n].endian) << std::endl;
				// }
				// for (int n = 0; n < rssiValuesDataMsg.size(); n++)
				// {
				//	 std::cout << (groupIdx + 1) << ". group: RssiValuesDataMsg[" << n << "].data = " << printMsgPack(*rssiValuesDataMsg[n].data) << std::endl;
				//	 std::cout << (groupIdx + 1) << ". group: RssiValuesDataMsg[" << n << "].elemSz = " << printMsgPack(*rssiValuesDataMsg[n].elemSz) << std::endl;
				//	 std::cout << (groupIdx + 1) << ". group: RssiValuesDataMsg[" << n << "].elemTypes = " << printMsgPack(*rssiValuesDataMsg[n].elemTypes) << std::endl;
				//	 std::cout << (groupIdx + 1) << ". group: RssiValuesDataMsg[" << n << "].endian = " << printMsgPack(*rssiValuesDataMsg[n].endian) << std::endl;
				// }
				ROS_INFO_STREAM("");
			}
		}
    	// msgpack validation
		if(msgpack_validator_enabled)
		{
			if (verbose)
			{
				std::vector<std::string> messages = msgpack_validator_data.print();
				for(int n = 0; n < messages.size(); n++)
					ROS_INFO_STREAM(messages[n]);
			}
			if (msgpack_validator.validateNotOutOfBound(msgpack_validator_data) == false)
			{
				if (discard_msgpacks_not_validated)
				{
					ROS_ERROR_STREAM("## ERROR MsgPackParser::Parse(): msgpack out of bounds validation failed, pointcloud discarded");
					return false;
				}
				ROS_ERROR_STREAM("## ERROR MsgPackParser::Parse(): msgpack out of bounds validation failed");
			}
			else if (verbose)
			{
				ROS_INFO_STREAM("msgpack out of bounds validation passed.");
			}
		}
	}
	catch (const std::exception& exc)
	{
		ROS_ERROR_STREAM("## ERROR msgpack11::MsgPack::parse(): exception " << exc.what());
		return false;
	}
	result.segmentIndex = segment_idx;
	result.telegramCnt = telegram_cnt;
	return true;
}

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
bool sick_scansegment_xd::MsgPackParser::WriteCSV(const std::vector<ScanSegmentParserOutput>& results, const std::string& csvFile, bool overwrite_existing_file)
{
	if (results.empty())
		return false;
	bool write_header = false;
	std::ios::openmode openmode = std::ios::app;
	if (overwrite_existing_file || results[0].segmentIndex == 0 || !sick_scansegment_xd::FileReadable(csvFile))
	{
		write_header = true; // Write a csv header once for all new csv files
		openmode = std::ios::trunc; // Create a new file in overwrite mode or at the first message, otherwise append all further messages
	}
	std::ofstream csv_ostream(csvFile, openmode);
	if (!csv_ostream.is_open())
	{
		ROS_ERROR_STREAM("## ERRORMsgPackParser::WriteCSV(): can't open output file \"" << csvFile << "\" for writing.");
		return false;
	}
	if (write_header)
	{
		csv_ostream << "SegmentIndex;               Timestamp;    GroupIdx;     EchoIdx;    PointIdx;           X;           Y;           Z;       Range;     Azimuth;   Elevation;   Intensity" << std::endl;
	}
	for (int msgCnt = 0; msgCnt < results.size(); msgCnt++)
	{
		const ScanSegmentParserOutput& result = results[msgCnt];
		for (int groupIdx = 0; groupIdx < result.scandata.size(); groupIdx++)
		{
			for (int echoIdx = 0; echoIdx < result.scandata[groupIdx].scanlines.size(); echoIdx++)
			{
				const std::vector<sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint>& scanline = result.scandata[groupIdx].scanlines[echoIdx].points;
				for (int pointIdx = 0; pointIdx < scanline.size(); pointIdx++)
				{
					const sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint& point = scanline[pointIdx];
					csv_ostream << std::setw(12) << result.segmentIndex;
					csv_ostream << ";" << std::setw(24) << result.timestamp;
					csv_ostream << ";" << std::setw(12) << point.groupIdx;
					csv_ostream << ";" << std::setw(12) << point.echoIdx;
					csv_ostream << ";" << std::setw(12) << point.pointIdx;
					csv_ostream << ";" << std::setw(12) << std::fixed << std::setprecision(3) << point.x;
					csv_ostream << ";" << std::setw(12) << std::fixed << std::setprecision(3) << point.y;
					csv_ostream << ";" << std::setw(12) << std::fixed << std::setprecision(3) << point.z;
					csv_ostream << ";" << std::setw(12) << std::fixed << std::setprecision(3) << point.range;
					csv_ostream << ";" << std::setw(12) << std::fixed << std::setprecision(8) << point.azimuth;
					csv_ostream << ";" << std::setw(12) << std::fixed << std::setprecision(8) << point.elevation;
					csv_ostream << ";" << std::setw(12) << std::fixed << std::setprecision(3) << point.i;
					csv_ostream << ";" << std::setw(24) << point.lidar_timestamp_microsec;
					csv_ostream << std::endl;
				}
			}
		}
	}
	return true;
}

/*
 * @brief exports x, y, z, intensity, group, echo and message index of msgpack data.
 * @param[in] result converted msgpack data, output from Parse function
 * Note: All output vectors x, y, z, i, group_idx, echo_idx, msg_idx identical size, i.e. it's safe to
 * assert(x.size() == y.size() && x.size() == z.size() && x.size() == i.size() && x.size() == group_idx.size() && echo_idx.size() == msg_idx.size());
 */
bool sick_scansegment_xd::MsgPackParser::ExportXYZI(const std::vector<ScanSegmentParserOutput>& results, std::vector<float>& x, std::vector<float>& y, std::vector<float>& z, std::vector<float>& i, std::vector<int>& group_idx, std::vector<int>& echo_idx, std::vector<int>& msg_idx)
{
	if (results.empty())
		return false;
	size_t data_length = 0;
	for (int groupIdx = 0; groupIdx < results[0].scandata.size(); groupIdx++)
	{
		for (int echoIdx = 0; echoIdx < results[0].scandata[groupIdx].scanlines.size(); echoIdx++)
		{
			data_length += results[0].scandata[groupIdx].scanlines[echoIdx].points.size();
		}
	}
	x.reserve(data_length);
	y.reserve(data_length);
	z.reserve(data_length);
	i.reserve(data_length);
	group_idx.reserve(data_length);
	echo_idx.reserve(data_length);
	msg_idx.reserve(data_length);
	for (int msgCnt = 0; msgCnt < results.size(); msgCnt++)
	{
		const ScanSegmentParserOutput& result = results[msgCnt];
		for (int groupIdx = 0; groupIdx < result.scandata.size(); groupIdx++)
		{
			for (int echoIdx = 0; echoIdx < result.scandata[groupIdx].scanlines.size(); echoIdx++)
			{
				const std::vector<sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint>& scanline = result.scandata[groupIdx].scanlines[echoIdx].points;
				for (int pointIdx = 0; pointIdx < scanline.size(); pointIdx++)
				{
					const sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint& point = scanline[pointIdx];
					x.push_back(point.x);
					y.push_back(point.y);
					z.push_back(point.z);
					i.push_back(point.i);
					group_idx.push_back(point.groupIdx);
					echo_idx.push_back(point.echoIdx);
					msg_idx.push_back(result.segmentIndex);
				}
			}
		}
	}
	return true;

}
