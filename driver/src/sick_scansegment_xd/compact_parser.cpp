/*
 * @brief compact_parser parses and convertes scandata in compact format
 *
 * Copyright (C) 2020,2021,2022,2023,2024,2025 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2020,2021,2022,2023,2024,2025 SICK AG, Waldkirch
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
 *  Copyright 2025 SICK AG
 *  Copyright 2025 Ing.-Buero Dr. Michael Lehning
 *
 */
#include "sick_scan/softwarePLL.h"
#include "sick_scansegment_xd/compact_parser.h"
#include "sick_scansegment_xd/config.h"
#include "sick_scansegment_xd/udp_receiver.h"

union COMPACT_4BYTE_UNION
{
  uint8_t u8_bytes[4];
  uint16_t u16_bytes[2];
  uint32_t u32_bytes[1];
  float f32_val[1];
};

union COMPACT_8BYTE_UNION
{
  uint8_t u8_bytes[8];
  uint16_t u16_bytes[4];
  uint32_t u32_bytes[2];
  uint64_t u64_bytes[1];
  float f32_val[2];
  double f64_val[1];
};

typedef enum ReadBeamAzimOrderEnum
{
    READ_BEAM_AZIM,
    READ_BEAM_PROP
} ReadBeamAzimOrder;

template <typename T> static inline T readUnsigned(const uint8_t* scandata, uint32_t* byte_cnt)
{
#if TARGET_IS_LITTLE_ENDIAN // src and dst have identical endianess
    *byte_cnt += sizeof(T);
    return (*((T*)scandata));
#else // src and dst have different endianess: reorder bytes
    COMPACT_8BYTE_UNION buffer;
    buffer.u64_bytes[0] = 0;
    for (int n = 0, m = sizeof(T) - 1; n < sizeof(T); n++,m--)
    {
        buffer.u8_bytes[n] = scandata[m];
    }
    *byte_cnt += sizeof(T);
    return (T)(buffer.u64_bytes[0]);
#endif
}

static inline float readFloat32(const uint8_t* scandata, uint32_t* byte_cnt)
{
#if TARGET_IS_LITTLE_ENDIAN // src and dst have identical endianess
    *byte_cnt += sizeof(float);
    return (*((float*)scandata));
#else // src and dst have different endianess: reorder bytes
    COMPACT_4BYTE_UNION buffer;
    buffer.u32_bytes[0] = 0;
    for (int n = 0, m = sizeof(float) - 1; n < sizeof(float); n++,m--)
    {
        buffer.u8_bytes[n] = scandata[m];
    }
    *byte_cnt += 4;
    return buffer.f32_val[0];
#endif
}

static inline bool endOfBuffer(uint32_t byte_cnt, size_t bytes_to_read, uint32_t num_bytes)
{
    return ((byte_cnt) + (bytes_to_read) > (num_bytes));
}

static void print_warning(const std::string& err_msg, int line_number, double print_rate = 1)
{
  static std::map<int, std::chrono::system_clock::time_point> last_error_printed;
  static std::map<int, size_t> error_cnt;
  if (error_cnt[line_number] == 0 || std::chrono::duration<double>(std::chrono::system_clock::now() - last_error_printed[line_number]).count() > 1/print_rate)
  {
    if (error_cnt[line_number] <= 1)
      ROS_WARN_STREAM(err_msg);
    else
      ROS_WARN_STREAM(err_msg << " (warning repeated " << error_cnt[line_number] << " times)");
    last_error_printed[line_number] = std::chrono::system_clock::now();
    error_cnt[line_number] = 0;
  }
  error_cnt[line_number] += 1;
}

#define CHECK_MODULE_SIZE(metadata, byte_required, byte_cnt, bytes_to_read, module_size, name, line_number) \
if (((byte_required) = (byte_cnt) + (bytes_to_read)) > (module_size))                                       \
{                                                                                                           \
    std::stringstream err_msg;                                                                              \
    err_msg << "## WARNING CompactDataParser::ParseModuleMetaData(): module_size=" << (module_size) << ", "   \
        << (byte_required) << " bytes required to read " << (name);                                         \
    print_warning(err_msg.str(), line_number);                                                                \
    return (metadata);                                                                                      \
}

/** returns a human readable description of the imu  data */
std::string sick_scansegment_xd::CompactImuData::to_string() const
{
    std::stringstream description;
    if (valid)
    {
      description << "acceleration_x:" << std::fixed << std::setprecision(1) << acceleration_x;
      description << ", acceleration_y:" << std::fixed << std::setprecision(1) << acceleration_y;
      description << ", acceleration_z:" << std::fixed << std::setprecision(1) << acceleration_z;
      description << ", angular_velocity_x:" << std::fixed << std::setprecision(1) << angular_velocity_x;
      description << ", angular_velocity_y:" << std::fixed << std::setprecision(1) << angular_velocity_y;
      description << ", angular_velocity_z:" << std::fixed << std::setprecision(1) << angular_velocity_z;
      description << ", orientation_w:" << std::fixed << std::setprecision(1) << orientation_w;
      description << ", orientation_x:" << std::fixed << std::setprecision(1) << orientation_x;
      description << ", orientation_y:" << std::fixed << std::setprecision(1) << orientation_y;
      description << ", orientation_z:" << std::fixed << std::setprecision(1) << orientation_z;
    }
    return description.str();
}


/** returns a human readable description of the header data */
std::string sick_scansegment_xd::CompactDataHeader::to_string() const
{
    std::stringstream description;
    description << "commandId:" << commandId;
    description << ", telegramVersion:" << telegramVersion;
    description << ", timeStampTransmit:" << timeStampTransmit;
    if (isImu())
    {
      description << ", IMU, " << imudata.to_string();
    }
    else
    {
      description << ", telegramCounter:" << telegramCounter;
      description << ", sizeModule0:" << sizeModule0;
    }
  return description.str();
}

/** returns a human readable description of the module metadata */
std::string sick_scansegment_xd::CompactModuleMetaData::to_string() const
{
    std::stringstream description;
    description << "SegmentCounter:" << SegmentCounter;
    description << ", FrameNumber:" << FrameNumber;
    description << ", SenderId:" << SenderId;
    description << ", NumberOfLinesInModule:" << NumberOfLinesInModule;
    description << ", NumberOfBeamsPerScan:" << NumberOfBeamsPerScan;
    description << ", NumberOfEchosPerBeam:" << NumberOfEchosPerBeam;
    description << ", TimeStampStart:[";
    for(int n = 0; n < TimeStampStart.size(); n++)
        description << (n>0?",":"") << TimeStampStart[n];
    description << "], TimeStampStop:[";
    for(int n = 0; n < TimeStampStop.size(); n++)
        description << (n>0?",":"") << TimeStampStop[n];
    description << "], Phi:[";
    for(int n = 0; n < Phi.size(); n++)
        description << (n>0?",":"") << Phi[n];
    description << "], ThetaStart:[";
    for(int n = 0; n < ThetaStart.size(); n++)
        description << (n>0?",":"") << ThetaStart[n];
    description << "], ThetaStop:[";
    for(int n = 0; n < ThetaStop.size(); n++)
        description << (n>0?",":"") << ThetaStop[n];
    description << "], DistanceScalingFactor:" << DistanceScalingFactor;
    description << ", NextModuleSize:" << NextModuleSize;
    description << ", Availability:" << (int)Availability;
    description << ", DataContentEchos:" << (int)DataContentEchos;
    description << ", DataContentBeams:" << (int)DataContentBeams;
    description << ", valid:" << (int)valid;
    return description.str();
}

/** returns a human readable description of the module measurement data */
std::string sick_scansegment_xd::CompactModuleMeasurementData::to_string() const
{
    std::stringstream description;
    for(int group_idx = 0; group_idx < scandata.size(); group_idx++)
    {
        description << (group_idx > 0 ? "," : "") << "scandata[" << group_idx << "]=[";
        const std::vector<ScanSegmentParserOutput::Scanline>& scanlines = scandata[group_idx].scanlines;
        for(int line_idx = 0; line_idx < scanlines.size(); line_idx++)
        {
            description << (line_idx > 0 ? "," : "") << "scanline[" << line_idx << "]=[";
            for(int point_idx = 0; point_idx < scanlines[line_idx].points.size(); point_idx++)
            {
                const ScanSegmentParserOutput::LidarPoint& point = scanlines[line_idx].points[point_idx];
                description << (point_idx > 0 ? "," : "") << "(";
                description << point.x << "," << point.y << "," << point.z << "," << point.i << ",";
                description << point.range << "," << point.azimuth << "," << point.elevation << ",";
                description << point.groupIdx << "," << point.echoIdx << "," << point.pointIdx << ")";
            }
            description << "]" << (scanlines.size() > 1 ? "\n": "");
        }
        description << "]" << (scandata.size() > 1 ? "\n": "");
    }
    description << ", valid:" << (int)valid;
    return description.str();
}

/*
* @brief parses scandata header in compact format.
* @param[in] scandata 32 byte scandata header
*/
sick_scansegment_xd::CompactDataHeader sick_scansegment_xd::CompactDataParser::ParseHeader(const uint8_t* scandata)
{
    uint32_t byte_cnt = 0;
    sick_scansegment_xd::CompactDataHeader header;
    header.commandId = readUnsigned<uint32_t>(scandata + byte_cnt, &byte_cnt);         // Telegram type, expected value: 1
    if (header.commandId == 1) // scan data in compact format
    {
      header.telegramCounter = readUnsigned<uint64_t>(scandata + byte_cnt, &byte_cnt);   // Incrementing telegram counter, starting with 1, number of telegrams since power up
      header.timeStampTransmit = readUnsigned<uint64_t>(scandata + byte_cnt, &byte_cnt); // Sensor timestamp in microseconds since 1.1.1970 00:00 UTC
      header.telegramVersion = readUnsigned<uint32_t>(scandata + byte_cnt, &byte_cnt);   // Telegram version, expected value: 4
      header.sizeModule0 = readUnsigned<uint32_t>(scandata + byte_cnt, &byte_cnt);       // Size of first module in byte
    }
    else if (header.commandId == 2) // imu data in compact format, all values in little endian
    {
      header.telegramCounter = 0; // not transmitted
      header.sizeModule0 = 0; // no further payload
      header.telegramVersion = readUnsigned<uint32_t>(scandata + byte_cnt, &byte_cnt);   // Telegram version, expected value: 1
      header.imudata.acceleration_x = readFloat32(scandata + byte_cnt, &byte_cnt); // 4 bytes float in m/s^2, acceleration along the x-axis including gravity
      header.imudata.acceleration_y = readFloat32(scandata + byte_cnt, &byte_cnt); // 4 bytes float in m/s^2, acceleration along the y-axis including gravity
      header.imudata.acceleration_z = readFloat32(scandata + byte_cnt, &byte_cnt); // 4 bytes float in m/s^2, acceleration along the z-axis including gravity
      header.imudata.angular_velocity_x = readFloat32(scandata + byte_cnt, &byte_cnt); // 4 bytes float in rad/s
      header.imudata.angular_velocity_y = readFloat32(scandata + byte_cnt, &byte_cnt); // 4 bytes float in rad/s
      header.imudata.angular_velocity_z = readFloat32(scandata + byte_cnt, &byte_cnt); // 4 bytes float in rad/s
      header.imudata.orientation_w = readFloat32(scandata + byte_cnt, &byte_cnt); // 4 bytes float, orientation quaternion w
      header.imudata.orientation_x = readFloat32(scandata + byte_cnt, &byte_cnt); // 4 bytes float, orientation quaternion x
      header.imudata.orientation_y = readFloat32(scandata + byte_cnt, &byte_cnt); // 4 bytes float, orientation quaternion y
      header.imudata.orientation_z = readFloat32(scandata + byte_cnt, &byte_cnt); // 4 bytes float, orientation quaternion z
      header.timeStampTransmit = readUnsigned<uint64_t>(scandata + byte_cnt, &byte_cnt); // Sensor timestamp in microseconds since 1.1.1970 00:00 UTC
      header.imudata.valid = true;
    }
    else
    {
      ROS_WARN_STREAM("CompactDataParser::ParseHeader: header.commandId = " << header.commandId << " not supported");
    }
    /* Create dummy IMU data, TEST ONLY **
    static int dummy_imu_data_cnt = 0;
    if (((++dummy_imu_data_cnt) % 2) == 0)
    {
      uint64_t timeStampTransmit = header.timeStampTransmit;
      header = sick_scansegment_xd::CompactDataHeader();
      header.commandId = 2;
      header.telegramVersion = 1;
      header.imudata.acceleration_x = 1;
      header.imudata.acceleration_y = 2;
      header.imudata.acceleration_z = 3;
      header.imudata.angular_velocity_x = 4;
      header.imudata.angular_velocity_y = 5;
      header.imudata.angular_velocity_z = 6;
      header.imudata.orientation_w = 7;
      header.imudata.orientation_x = 8;
      header.imudata.orientation_y = 9;
      header.imudata.orientation_z = 10;
      header.timeStampTransmit = timeStampTransmit;
      header.imudata.valid = true;
    }
    ** End of dummy IMU data */
    return header;
}

/*
* @brief Parses module meta data in compact format.
* @param[in] scandata received bytes
* @param[in] module_size Number of bytes to parse
* @param[in] telegramVersion compact format version, currently version 3 and 4 supported
* @param[out] module_metadata_size number of bytes parsed (i.e. size of meta data in bytes if successful, i.e. if metadata.valid == true)
*/
sick_scansegment_xd::CompactModuleMetaData sick_scansegment_xd::CompactDataParser::ParseModuleMetaData(const uint8_t* scandata, uint32_t module_size, uint32_t telegramVersion, uint32_t& module_metadata_size)
{
    uint32_t byte_cnt = 0, byte_required = 0;
    sick_scansegment_xd::CompactModuleMetaData metadata;
    // metadata.valid flag is false and becomes true after successful parsing
    module_metadata_size = 0;
    // SegmentCounter
    CHECK_MODULE_SIZE(metadata, byte_required, byte_cnt, sizeof(uint64_t), module_size, "SegmentCounter", __LINE__);
    metadata.SegmentCounter = readUnsigned<uint64_t>(scandata + byte_cnt, &byte_cnt);           // Incrementing segment counter
    // FrameNumber
    CHECK_MODULE_SIZE(metadata, byte_required, byte_cnt, sizeof(uint64_t), module_size, "FrameNumber", __LINE__);
    metadata.FrameNumber = readUnsigned<uint64_t>(scandata + byte_cnt, &byte_cnt);              // Number of frames since power on
    // SenderId
    CHECK_MODULE_SIZE(metadata, byte_required, byte_cnt, sizeof(uint32_t), module_size, "SenderId", __LINE__);
    metadata.SenderId = readUnsigned<uint32_t>(scandata + byte_cnt, &byte_cnt);                 // The serial number of the device
    // NumberOfLinesInModule
    CHECK_MODULE_SIZE(metadata, byte_required, byte_cnt, sizeof(uint32_t), module_size, "NumberOfLinesInModule", __LINE__);
    metadata.NumberOfLinesInModule = readUnsigned<uint32_t>(scandata + byte_cnt, &byte_cnt);    // Number of layers in this module
    if (metadata.NumberOfLinesInModule > 16)
    {
        std::stringstream err_msg;
        err_msg << "## WARNING CompactDataParser::ParseModuleMetaData(): unexpected NumberOfLinesInModule=" << metadata.NumberOfLinesInModule;
        print_warning(err_msg.str(), __LINE__);
    }
    // NumberOfBeamsPerScan
    CHECK_MODULE_SIZE(metadata, byte_required, byte_cnt, sizeof(uint32_t), module_size, "NumberOfBeamsPerScan", __LINE__);
    metadata.NumberOfBeamsPerScan = readUnsigned<uint32_t>(scandata + byte_cnt, &byte_cnt);     // Number of beams per layer (all layers have the same number of beams)
    // NumberOfEchosPerBeam
    CHECK_MODULE_SIZE(metadata, byte_required, byte_cnt, sizeof(uint32_t), module_size, "NumberOfEchosPerBeam", __LINE__);
    metadata.NumberOfEchosPerBeam = readUnsigned<uint32_t>(scandata + byte_cnt, &byte_cnt);     // Number of echos per beams
    // TimeStampStart
    CHECK_MODULE_SIZE(metadata, byte_required, byte_cnt, metadata.NumberOfLinesInModule * sizeof(uint64_t), module_size, "TimeStampStart", __LINE__);
    metadata.TimeStampStart.reserve(metadata.NumberOfLinesInModule);
    for(uint32_t cnt = 0; cnt < metadata.NumberOfLinesInModule; cnt++)  // Array of timestamps of the first beam in a scan in microseconds, number of elements is numberOfLinesInModule
    {
        metadata.TimeStampStart.push_back(readUnsigned<uint64_t>(scandata + byte_cnt, &byte_cnt));
    }
    // TimeStampStop
    CHECK_MODULE_SIZE(metadata, byte_required, byte_cnt, metadata.NumberOfLinesInModule * sizeof(uint64_t), module_size, "TimeStampStop", __LINE__);
    metadata.TimeStampStop.reserve(metadata.NumberOfLinesInModule);
    for(uint32_t cnt = 0; cnt < metadata.NumberOfLinesInModule; cnt++)  // Array of timestamps of the last beam in a scan in microseconds, number of elements is numberOfLinesInModule
    {
        metadata.TimeStampStop.push_back(readUnsigned<uint64_t>(scandata + byte_cnt, &byte_cnt));
    }
    // Phi
    CHECK_MODULE_SIZE(metadata, byte_required, byte_cnt, metadata.NumberOfLinesInModule * sizeof(float), module_size, "Phi", __LINE__);
    metadata.Phi.reserve(metadata.NumberOfLinesInModule);
    for(uint32_t cnt = 0; cnt < metadata.NumberOfLinesInModule; cnt++)  // Array of elevation angles in radians for each layer in this module, number of elements is numberOfLinesInModule
    {
        float val = readFloat32(scandata + byte_cnt, &byte_cnt);
        metadata.Phi.push_back(val);
    }
    // ThetaStart
    CHECK_MODULE_SIZE(metadata, byte_required, byte_cnt, metadata.NumberOfLinesInModule * sizeof(float), module_size, "ThetaStart", __LINE__);
    metadata.ThetaStart.reserve(metadata.NumberOfLinesInModule);
    for(uint32_t cnt = 0; cnt < metadata.NumberOfLinesInModule; cnt++)  // Array of azimuth angles in radians for first beam of each layer in this module, number of elements is numberOfLinesInModule
    {
        float val = readFloat32(scandata + byte_cnt, &byte_cnt);
        metadata.ThetaStart.push_back(val);
    }
    // ThetaStop
    CHECK_MODULE_SIZE(metadata, byte_required, byte_cnt, metadata.NumberOfLinesInModule * sizeof(float), module_size, "ThetaStop", __LINE__);
    metadata.ThetaStop.reserve(metadata.NumberOfLinesInModule);
    for(uint32_t cnt = 0; cnt < metadata.NumberOfLinesInModule; cnt++)  // Array of azimuth angles in radians for last beam of each layer in this module, number of elements is numberOfLinesInModule
    {
        float val = readFloat32(scandata + byte_cnt, &byte_cnt);
        metadata.ThetaStop.push_back(val);
    }
    // DistanceScalingFactor for 16-bit distance values: distance_mm = DistanceScalingFactor * distance_16bit_sensor
    if (telegramVersion == 3)
    {
        metadata.DistanceScalingFactor = 1;
    }
    else if (telegramVersion == 4)
    {
        CHECK_MODULE_SIZE(metadata, byte_required, byte_cnt, sizeof(float), module_size, "DistanceScalingFactor", __LINE__);
        metadata.DistanceScalingFactor = readFloat32(scandata + byte_cnt, &byte_cnt);
    }
    else
    {
        ROS_ERROR_STREAM("## ERROR CompactDataParser::ParseModuleMetaData(): telegramVersion=" << telegramVersion << " not supported");
        return metadata;
    }
    // NextModuleSize
    CHECK_MODULE_SIZE(metadata, byte_required, byte_cnt, sizeof(uint32_t), module_size, "NextModuleSize", __LINE__);
    metadata.NextModuleSize = readUnsigned<uint32_t>(scandata + byte_cnt, &byte_cnt);           // Size of the next module (or 0 if this module is the last one)
    // Availability
    CHECK_MODULE_SIZE(metadata, byte_required, byte_cnt, sizeof(uint8_t), module_size, "Availability", __LINE__);
    metadata.Availability = readUnsigned<uint8_t>(scandata + byte_cnt, &byte_cnt);              // Reserved for futher use (flag indication distortion in scan data)
    // DataContentEchos
    CHECK_MODULE_SIZE(metadata, byte_required, byte_cnt, sizeof(uint8_t), module_size, "DataContentEchos", __LINE__);
    metadata.DataContentEchos = readUnsigned<uint8_t>(scandata + byte_cnt, &byte_cnt);          // Bitarray: (DataContentEchos & 0x01) == 0x01 if distance values available, (DataContentEchos & 0x02) == 0x02 if RSSI values available, (DataContentEchos & 0x03) == 0x03: distance and RSSI values available
    // DataContentBeams
    CHECK_MODULE_SIZE(metadata, byte_required, byte_cnt, sizeof(uint8_t), module_size, "DataContentBeams", __LINE__);
    metadata.DataContentBeams = readUnsigned<uint8_t>(scandata + byte_cnt, &byte_cnt);          // Bitarray: (DataContentBeams & 0x01) == 0x01 if beam properties available, (DataContentBeams & 0x02) == 0x02 if azimuth angle per beeam available, (DataContentBeams & 0x03) == 0x03: both available
    // reserved
    CHECK_MODULE_SIZE(metadata, byte_required, byte_cnt, sizeof(uint8_t), module_size, "reserved", __LINE__);
    metadata.reserved = readUnsigned<uint8_t>(scandata + byte_cnt, &byte_cnt);                  // Reserved for 32 bit alignment
    // valid flag set true after successful parsing
    metadata.valid = true;
    module_metadata_size = byte_cnt;
    return metadata;
}

static std::vector<int> s_layer_elevation_table_mdeg; // Optional elevation LUT in mdeg for layers in compact format, s_layer_elevation_table_mdeg[layer_idx] := ideal elevation in mdeg

/*
* @brief Sets the elevation in mdeg for layers in compact format.
* @param[in] layer_elevation_table_mdeg layer_elevation_table_mdeg[layer_idx] := ideal elevation in mdeg
*/
void sick_scansegment_xd::CompactDataParser::SetLayerElevationTable(const std::vector<int>& layer_elevation_table_mdeg)
{
  s_layer_elevation_table_mdeg = layer_elevation_table_mdeg;
}

/*
* @brief Return a layer-id from a given elevation angle. See compact scanformat documention:
* The line/layer index in the figure below is not a layer id according to layer numbering for multi layer sensors.
* Therefore this functions returns a layer-id from the elevation angle in rad.
* @param[in] layer_elevation_rad layer_elevation in radians
* @return layer-id
*/
int sick_scansegment_xd::CompactDataParser::GetLayerIDfromElevation(float layer_elevation_rad) // layer_elevation in radians
{
  int layer_elevation_mdeg = (int)std::lround(layer_elevation_rad * 180000 / M_PI);
  if (!s_layer_elevation_table_mdeg.empty())
  {
    int layer_idx = 0;
    int elevation_dist = std::abs(layer_elevation_mdeg - s_layer_elevation_table_mdeg[layer_idx]);
    for(int n = 1; n < s_layer_elevation_table_mdeg.size(); n++)
    {
      int dist = std::abs(layer_elevation_mdeg - s_layer_elevation_table_mdeg[n]);
      if (elevation_dist > dist)
      {
        elevation_dist = dist;
        layer_idx = n;
      }
      else
      {
        break;
      }
    }
    return layer_idx;
  }
  else
  {
    static std::map<int,int> elevation_layerid_map;
    if (elevation_layerid_map.find(layer_elevation_mdeg) == elevation_layerid_map.end())
    {
      elevation_layerid_map[layer_elevation_mdeg] = elevation_layerid_map.size() + 1; // Add new layer
      int layerid = 0;
      for(std::map<int,int>::iterator iter_layerid_map = elevation_layerid_map.begin(); iter_layerid_map != elevation_layerid_map.end(); iter_layerid_map++)
        iter_layerid_map->second = layerid++; // Resort by ascending elevation
    }
    return elevation_layerid_map[layer_elevation_mdeg];
  }
}

/*
* @brief Return the typical (default) elevation of a given layer index
* @param[in] layer_idx layer index
* @return layer elevation in degree
*/
float sick_scansegment_xd::CompactDataParser::GetElevationDegFromLayerIdx(int layer_idx)
{
  if (layer_idx >= 0 && layer_idx < s_layer_elevation_table_mdeg.size())
  {
    return 0.001f * s_layer_elevation_table_mdeg[layer_idx];
  }
  return 0;
}


/*
* @brief Parses module measurement data in compact format.
* @param[in] payload binary payload
* @param[in] num_bytes size of binary payload in bytes
* @param[in] meta_data module metadata with measurement properties
* @param[out] measurement_data parsed and converted module measurement data
* @return true on success, false on error
*/
bool sick_scansegment_xd::CompactDataParser::ParseModuleMeasurementData(const uint8_t* payload, uint32_t num_bytes, const sick_scansegment_xd::CompactDataHeader& compact_header,
  const sick_scansegment_xd::CompactModuleMetaData& meta_data, float azimuth_offset, sick_scansegment_xd::CompactModuleMeasurementData& measurement_data)
{
  measurement_data = sick_scansegment_xd::CompactModuleMeasurementData();
  measurement_data.valid = false;
  if (meta_data.NumberOfLinesInModule < 1 ||
    meta_data.NumberOfEchosPerBeam < 1 ||
    meta_data.NumberOfBeamsPerScan < 1)
  {
    ROS_ERROR_STREAM("CompactDataParser::ParseModuleMeasurementData(): invalid meta_data: { " << meta_data.to_string() << " }");
    return false;
  }
  bool dist_available = ((meta_data.DataContentEchos & 0x01) == 0x01);
  bool rssi_available = ((meta_data.DataContentEchos & 0x02) == 0x02);
  bool beam_prop_available = ((meta_data.DataContentBeams & 0x01) == 0x01);
  bool beam_azim_available = ((meta_data.DataContentBeams & 0x02) == 0x02);
  float dist_scale_factor = meta_data.DistanceScalingFactor;

  uint32_t num_layers = meta_data.NumberOfLinesInModule;
  uint32_t num_echos = meta_data.NumberOfEchosPerBeam;
  measurement_data.scandata = std::vector<ScanSegmentParserOutput::Scangroup>(num_layers);

  ROS_DEBUG_STREAM("CompactDataParser::ParseModuleMeasurementData(): num_bytes=" << num_bytes << ", num_layers=" << num_layers
    << ", num_points=" << meta_data.NumberOfBeamsPerScan << ", num_echos=" << num_echos << ", dist_available=" << dist_available
    << ", rssi_available=" << rssi_available << ", beam_prop_available=" << beam_prop_available
    << ", beam_azim_available=" << beam_azim_available);
  // Prepare output data
  std::vector<float> lut_layer_elevation(num_layers);
  std::vector<float> lut_layer_azimuth_start(num_layers);
  std::vector<float> lut_layer_azimuth_stop(num_layers);
  std::vector<float> lut_layer_azimuth_delta(num_layers);
  std::vector<uint64_t> lut_layer_lidar_timestamp_microsec_start(num_layers);
  std::vector<uint64_t> lut_layer_lidar_timestamp_microsec_stop(num_layers);
  std::vector<float> lut_sin_elevation(num_layers);
  std::vector<float> lut_cos_elevation(num_layers);
  std::vector<int> lut_groupIdx(num_layers);

  for (uint32_t layer_idx = 0; layer_idx < num_layers; layer_idx++)
  {
    uint32_t layer_timeStamp_start_sec = (meta_data.TimeStampStart[layer_idx] / 1000000);
    uint32_t layer_timeStamp_start_nsec = 1000 * (meta_data.TimeStampStart[layer_idx] % 1000000);
    uint32_t layer_timeStamp_stop_sec = (meta_data.TimeStampStop[layer_idx] / 1000000);
    uint32_t layer_timeStamp_stop_nsec = 1000 * (meta_data.TimeStampStop[layer_idx] % 1000000);
    measurement_data.scandata[layer_idx].timestampStart_sec = layer_timeStamp_start_sec;
    measurement_data.scandata[layer_idx].timestampStart_nsec = layer_timeStamp_start_nsec;
    measurement_data.scandata[layer_idx].timestampStop_sec = layer_timeStamp_stop_sec;
    measurement_data.scandata[layer_idx].timestampStop_nsec = layer_timeStamp_stop_nsec;
    measurement_data.scandata[layer_idx].scanlines = std::vector<ScanSegmentParserOutput::Scanline>(num_echos);
    for (uint32_t echo_idx = 0; echo_idx < num_echos; echo_idx++)
    {
      measurement_data.scandata[layer_idx].scanlines[echo_idx].points = std::vector<ScanSegmentParserOutput::LidarPoint>(meta_data.NumberOfBeamsPerScan);
    }
    lut_layer_elevation[layer_idx] = -meta_data.Phi[layer_idx]; // elevation must be negated, a positive pitch-angle yields negative z-coordinates (compare to MsgPackParser::Parse in msgpack_parser.cpp)
    lut_layer_azimuth_start[layer_idx] = meta_data.ThetaStart[layer_idx];
    lut_layer_azimuth_stop[layer_idx] = meta_data.ThetaStop[layer_idx];
    lut_layer_azimuth_delta[layer_idx] = (lut_layer_azimuth_stop[layer_idx] - lut_layer_azimuth_start[layer_idx]) / (float)(std::max(1, (int)meta_data.NumberOfBeamsPerScan - 1));
    lut_layer_lidar_timestamp_microsec_start[layer_idx] = meta_data.TimeStampStart[layer_idx];
    lut_layer_lidar_timestamp_microsec_stop[layer_idx] = meta_data.TimeStampStop[layer_idx];    
    lut_sin_elevation[layer_idx] = std::sin(lut_layer_elevation[layer_idx]);
    lut_cos_elevation[layer_idx] = std::cos(lut_layer_elevation[layer_idx]);
    lut_groupIdx[layer_idx] = GetLayerIDfromElevation(meta_data.Phi[layer_idx]);
  }
  // Parse scan data
  uint32_t byte_cnt = 0;
  for (uint32_t point_idx = 0; point_idx < meta_data.NumberOfBeamsPerScan; point_idx++)
  {
    for (uint32_t layer_idx = 0; layer_idx < num_layers; layer_idx++)
    {
      float layer_elevation = lut_layer_elevation[layer_idx];
      float layer_azimuth_start = lut_layer_azimuth_start[layer_idx];
      float layer_azimuth_stop = lut_layer_azimuth_stop[layer_idx];
      float layer_azimuth_delta = lut_layer_azimuth_delta[layer_idx];
      uint64_t lidar_timestamp_microsec_start = lut_layer_lidar_timestamp_microsec_start[layer_idx];
      uint64_t lidar_timestamp_microsec_stop = lut_layer_lidar_timestamp_microsec_stop[layer_idx];
      uint64_t lidar_timestamp_microsec = ((point_idx * (lidar_timestamp_microsec_stop - lidar_timestamp_microsec_start)) / (meta_data.NumberOfBeamsPerScan - 1)) + lidar_timestamp_microsec_start;
      float sin_elevation = lut_sin_elevation[layer_idx];
      float cos_elevation = lut_cos_elevation[layer_idx];
      int groupIdx = lut_groupIdx[layer_idx];
      std::vector<ScanSegmentParserOutput::LidarPoint> points(num_echos);
      uint8_t beam_property = 0;
      float azimuth = 0;
      for (uint32_t echo_idx = 0; echo_idx < num_echos; echo_idx++)
      {
        if (dist_available)
        {
          if (endOfBuffer(byte_cnt, sizeof(uint16_t), num_bytes)) // if (byte_cnt + sizeof(uint16_t) > num_bytes)
          {
            ROS_ERROR_STREAM("## ERROR CompactDataParser::ParseModuleMeasurementData(" << __LINE__ << "): byte_cnt=" << byte_cnt << ", num_bytes=" << num_bytes << ", layer " << layer_idx << " of " << num_layers
              << ", point " << point_idx << " of " << meta_data.NumberOfBeamsPerScan << ", echo " << echo_idx << " of " << num_echos);
            return false;
          }
          points[echo_idx].range = (dist_scale_factor * (float)readUnsigned<uint16_t>(payload + byte_cnt, &byte_cnt)) / 1000.0f;
        }
        if (rssi_available)
        {
          if (endOfBuffer(byte_cnt, sizeof(uint16_t), num_bytes)) // if (byte_cnt + sizeof(uint16_t) > num_bytes)
          {
            ROS_ERROR_STREAM("## ERROR CompactDataParser::ParseModuleMeasurementData(" << __LINE__ << "): byte_cnt=" << byte_cnt << ", num_bytes=" << num_bytes << ", layer " << layer_idx << " of " << num_layers
              << ", point " << point_idx << " of " << meta_data.NumberOfBeamsPerScan << ", echo " << echo_idx << " of " << num_echos);
            return false;
          }
          points[echo_idx].i = (float)readUnsigned<uint16_t>(payload + byte_cnt, &byte_cnt);
        }
      }
      std::vector<ReadBeamAzimOrderEnum> azim_prop_order;
      if (compact_header.telegramVersion == 3) // for backward compatibility only
      {
        azim_prop_order = { READ_BEAM_AZIM, READ_BEAM_PROP }; // telegramVersion 3: 2 byte azimuth + 1 byte property
      }
      else if (compact_header.telegramVersion == 4)
      {
        azim_prop_order = { READ_BEAM_PROP, READ_BEAM_AZIM }; // telegramVersion 4 (default): 1 byte property + 2 byte azimuth
      }
      else
      {
        ROS_ERROR_STREAM("## ERROR CompactDataParser::ParseModuleMeasurementData(" << __LINE__ << "): telegramVersion=" << compact_header.telegramVersion << " not supported");
        return false;
      }
      for (int azim_prop_cnt = 0; azim_prop_cnt < 2; azim_prop_cnt++)
      {
        if (azim_prop_order[azim_prop_cnt] == READ_BEAM_AZIM)
        {
          if (beam_azim_available)
          {
            if (endOfBuffer(byte_cnt, sizeof(uint16_t), num_bytes)) // if (byte_cnt + sizeof(uint16_t) > num_bytes)
            {
              ROS_ERROR_STREAM("## ERROR CompactDataParser::ParseModuleMeasurementData(" << __LINE__ << "): byte_cnt=" << byte_cnt << ", num_bytes=" << num_bytes << ", layer " << layer_idx << " of " << num_layers
                << ", point " << point_idx << " of " << meta_data.NumberOfBeamsPerScan);
              return false;
            }
            azimuth = ((float)readUnsigned<uint16_t>(payload + byte_cnt, &byte_cnt) - 16384.0f) / 5215.0f + azimuth_offset;
          }
          else
          {
            azimuth = layer_azimuth_start + point_idx * layer_azimuth_delta + azimuth_offset;
          }
        }
        if (azim_prop_order[azim_prop_cnt] == READ_BEAM_PROP)
        {
          if (beam_prop_available)
          {
            if (byte_cnt + sizeof(uint8_t) > num_bytes)
            {
              ROS_ERROR_STREAM("## ERROR CompactDataParser::ParseModuleMeasurementData(" << __LINE__ << "): byte_cnt=" << byte_cnt << ", num_bytes=" << num_bytes << ", layer " << layer_idx << " of " << num_layers
                << ", point " << point_idx << " of " << meta_data.NumberOfBeamsPerScan);
              return false;
            }
            beam_property = readUnsigned<uint8_t>(payload + byte_cnt, &byte_cnt);
          }
        }
      }
      float sin_azimuth = std::sin(azimuth);
      float cos_azimuth = std::cos(azimuth);
      // std::stringstream s;
      // s << "Measurement[" << layer_idx << "," << point_idx << "]=(";
      // for(uint32_t echo_idx = 0; echo_idx < num_echos; echo_idx++)
      //     s << points[echo_idx].range << "," << points[echo_idx].i << ",";
      // s << (int)beam_property << "," << azimuth << ")";
      // ROS_DEBUG_STREAM("" << s.str());
      // Flag to ensure the reflector bit is only set once (on the last valid echo)
      bool reflectorBitSet = false;

      // Check if this beam is marked as a reflector (bit 0 set)
      bool isReflector = (beam_property & 0x01);

      // Iterate over all echoes in reverse (from last to first)
      for (int32_t echo_idx = static_cast<int32_t>(num_echos) - 1; echo_idx >= 0; --echo_idx)
      {
        ScanSegmentParserOutput::LidarPoint& pt = points[echo_idx];

        // Assign basic point metadata
        pt.azimuth = azimuth;
        pt.elevation = layer_elevation;
        double range = pt.range;

        // Convert polar coordinates to Cartesian
        pt.x = range * cos_azimuth * cos_elevation;
        pt.y = range * sin_azimuth * cos_elevation;
        pt.z = range * sin_elevation;

        // Assign additional metadata
        pt.echoIdx = echo_idx;
        pt.groupIdx = groupIdx;
        pt.pointIdx = point_idx;
        pt.lidar_timestamp_microsec = lidar_timestamp_microsec;

        // If this is the first valid echo from the end and a reflector is present,
        // set the reflector bit
        if (!reflectorBitSet && isReflector && range > 1e-6)
        {
          pt.reflectorbit |= 0x01;
          reflectorBitSet = true;
        }

        // Store the processed point in the scan structure
        measurement_data.scandata[layer_idx].scanlines[echo_idx].points[point_idx] = pt;
      }
    }
  }
  if (byte_cnt != num_bytes)
  {
    ROS_ERROR_STREAM("## ERROR CompactDataParser::ParseModuleMeasurementData(" << __LINE__ << "): byte_cnt=" << byte_cnt << ", num_bytes=" << num_bytes);
    return false;
  }
  measurement_data.valid = true;
  return measurement_data.valid;
}

/*
* @brief Parses a scandata segment in compact format.
* @param[in] payload binary payload
* @param[in] bytes_received size of binary payload in bytes
* @param[out] segment_data parsed segment data (or 0 if not required)
* @param[out] payload_length_bytes parsed number of bytes
* @param[out] num_bytes_required  min number of bytes required for successful parsing
* @param[in] azimuth_offset optional offset in case of additional coordinate transform (default: 0)
* @param[in] verbose > 0: print debug messages (default: 0)
*/
bool sick_scansegment_xd::CompactDataParser::ParseSegment(const uint8_t* payload, size_t bytes_received, sick_scansegment_xd::CompactSegmentData* segment_data,
    uint32_t& payload_length_bytes, uint32_t& num_bytes_required , float azimuth_offset, int verbose)
{
    // Read 32 byte compact data header
    const uint32_t header_size_bytes = 32;
    const std::vector<uint8_t> msg_start_seq = { 0x02, 0x02,  0x02,  0x02 };
    sick_scansegment_xd::CompactDataHeader compact_header;
    if(bytes_received >= 32)
    {
        compact_header = sick_scansegment_xd::CompactDataParser::ParseHeader(payload + msg_start_seq.size());
        if (verbose > 0)
        {
            ROS_INFO_STREAM("CompactDataParser::ParseSegment(): compact header = { " << compact_header.to_string() << " }");
        }
    }
    else
    {
        if (verbose > 0)
        {
            ROS_INFO_STREAM("CompactDataParser::ParseSegment(): " << bytes_received << " bytes received (compact), at least " << (msg_start_seq.size() + 32) << " bytes required for compact data header");
        }
        payload_length_bytes = 0;
        num_bytes_required  = msg_start_seq.size() + 32;
        return false;
    }
    if (segment_data)
    {
        segment_data->segmentHeader = compact_header;
        segment_data->segmentModules.clear();
    }
    if (compact_header.commandId == 2) // imu data in compact format have always 64 byte payload, payload length is not coded in the header
    {
        payload_length_bytes = 60; // i.e. 64 byte excl. 4 byte CRC
        num_bytes_required  = 64;  // 64 byte incl. 4 byte CRC
        return compact_header.imudata.valid;
    }
    // Read compact data modules
    uint32_t module_size = compact_header.sizeModule0;
    uint32_t module_offset = header_size_bytes;
    payload_length_bytes = header_size_bytes;
    num_bytes_required  = header_size_bytes;
    bool success = true;
    while (module_size > 0)
    {
        if (module_offset +  module_size > bytes_received)
        {
            if (verbose > 0)
            {
                ROS_INFO_STREAM("CompactDataParser::ParseSegment(): " << bytes_received << " bytes received (compact), at least " << (module_offset +  module_size) << " bytes required for module meta data");
            }
            payload_length_bytes = 0;
            num_bytes_required  = module_offset +  module_size;
            return false;
        }
        uint32_t module_metadata_size = 0;
        sick_scansegment_xd::CompactModuleMetaData module_meta_data = sick_scansegment_xd::CompactDataParser::ParseModuleMetaData(payload + module_offset, module_size, compact_header.telegramVersion, module_metadata_size);
        if (verbose > 0)
        {
            ROS_INFO_STREAM("CompactDataParser::ParseSegment(): module meta data = { " << module_meta_data.to_string() << " }");
        }
        if (module_meta_data.valid != true || module_size < module_metadata_size)
        {
            std::stringstream err_msg;
            err_msg << "## WARNING CompactDataParser::ParseSegment(): " << bytes_received << " bytes received (compact), CompactDataParser::ParseModuleMetaData() failed";
            print_warning(err_msg.str(), __LINE__);
            payload_length_bytes = 0;
            num_bytes_required  = module_offset +  module_size;
            return false;
        }
        if (segment_data)
        {
            sick_scansegment_xd::CompactModuleData segment_module;
            segment_module.moduleMetadata = module_meta_data;
            sick_scansegment_xd::CompactDataParser::ParseModuleMeasurementData(payload + module_offset + module_metadata_size, module_size - module_metadata_size, compact_header, module_meta_data, azimuth_offset, segment_module.moduleMeasurement);
            if (verbose > 0)
            {
                ROS_INFO_STREAM("CompactDataParser::ParseSegment(): module measurement data = { " << segment_module.moduleMeasurement.to_string() << " }");
            }
            if (segment_module.moduleMeasurement.valid)
            {
                segment_data->segmentModules.push_back(segment_module);
            }
            else
            {
                ROS_ERROR_STREAM("## ERROR CompactDataParser::ParseSegment(): CompactDataParser::ParseModuleMeasurementData() failed, parsed data = { " << segment_module.moduleMeasurement.to_string() << " }");
                success = false;
            }
        }
        module_offset += module_size;
        payload_length_bytes += module_size;
        num_bytes_required  = payload_length_bytes;
        module_size = module_meta_data.NextModuleSize;
    }
    if (segment_data && verbose > 0)
    {
        ROS_INFO_STREAM("CompactDataParser: " << segment_data->segmentModules.size() << " modules");
        for(int module_idx = 0; module_idx < segment_data->segmentModules.size(); module_idx++)
        {
            const std::vector<ScanSegmentParserOutput::Scangroup>& scandata = segment_data->segmentModules[module_idx].moduleMeasurement.scandata;
            ROS_INFO_STREAM("    CompactDataParser (module " << module_idx << "): " << scandata.size() << " groups");
            for(int group_idx = 0; group_idx < scandata.size(); group_idx++)
            {
                ROS_INFO_STREAM("        CompactDataParser (module " << module_idx << ", group " << group_idx << "): " << scandata[group_idx].scanlines.size() << " lines");
                for(int line_idx = 0; line_idx < scandata[group_idx].scanlines.size(); line_idx++)
                {
                    const std::vector<ScanSegmentParserOutput::LidarPoint>& points = scandata[group_idx].scanlines[line_idx].points;
                    ROS_INFO_STREAM("        CompactDataParser (module " << module_idx << ", group " << group_idx << ", line " << line_idx << "): " << points.size() << " points");
                    for(int point_idx = 0; point_idx < points.size(); point_idx++)
                    {
                        ROS_INFO_STREAM("        [" << points[point_idx].x << "," << points[point_idx].y << "," << points[point_idx].z << "," << points[point_idx].range << "," << points[point_idx].azimuth << "," << points[point_idx].elevation << "," << points[point_idx].groupIdx << "," << points[point_idx].pointIdx << "]");
                    }
                }
            }
        }
    }
    return success;
}

#define EXPORT_MEASUREMENT_AZIMUTH_ACCELERATION_CSV 0 // Measurement of IMU latency (development only): Export ticks (imu resp. lidar timestamp in micro seconds), imu acceleration and lidar max azimuth of board cube
#if EXPORT_MEASUREMENT_AZIMUTH_ACCELERATION_CSV
/*
* @brief Returns the max azimuth aperture (i.e. max - min azimuth) of all scan points within a cube, i.e. the max azimuth aperture of all points p
*        with x_min <= p.x <= x_max && y_min <= p.y <= y_max && z_min <= p.z <= z_max && p.azimuth >= azimuth_min && p.azimuth <= azimuth_max
*/
static bool getMaxAzimuthApertureWithinCube(const std::vector<sick_scansegment_xd::ScanSegmentParserOutput::Scangroup>& scandata,
  float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, float azimuth_min, float azimuth_max,
  double& azimuth_aperture, uint64_t& timestamp_microsec)
{
  bool success = false;
  azimuth_aperture = 0;
  timestamp_microsec = 0;
  for(int group_idx = 0; group_idx < scandata.size(); group_idx++)
  {
    for(int line_idx = 0; line_idx < scandata[group_idx].scanlines.size(); line_idx++)
    {
      uint64_t timestampStart_microsec = (uint64_t)scandata[group_idx].timestampStart_sec * 1000000UL + (uint64_t)scandata[group_idx].timestampStart_nsec / 1000;
      uint64_t timestampStop_microsec  = (uint64_t)scandata[group_idx].timestampStop_sec  * 1000000UL + (uint64_t)scandata[group_idx].timestampStop_nsec  / 1000;
      double point_azi_min = FLT_MAX, point_azi_max = -FLT_MAX;
      for(int point_idx = 0; point_idx < scandata[group_idx].scanlines[line_idx].points.size(); point_idx++)
      {
        const sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint& p = scandata[group_idx].scanlines[line_idx].points[point_idx];
        if (p.x >= x_min && p.x <= x_max && p.y >= y_min && p.y <= y_max && p.z >= z_min && p.z<= z_max && p.azimuth >= azimuth_min && p.azimuth <= azimuth_max)
        {
          point_azi_min = std::min(point_azi_min, (double)p.azimuth);
          point_azi_max = std::max(point_azi_max, (double)p.azimuth);
        }
      }
      if (point_azi_max > point_azi_min && azimuth_aperture < point_azi_max) // (point_azi_max - point_azi_min))
      {
        azimuth_aperture = point_azi_max; // - point_azi_min;
        timestamp_microsec = timestampStart_microsec;
        success = true;
      }
    }
  }
  return success;
}
#endif // EXPORT_MEASUREMENT_AZIMUTH_ACCELERATION_CSV

/*
* @brief Parses a scandata segment in compact format.
* @param[in] parser_config configuration and settings for multiScan and picoScan parser
* @param[in] segment_data binary segment data in compact format
* @param[in] system_timestamp receive timestamp of segment_data (system time)
* @param[in] add_transform_xyz_rpy Optionally apply an additional transform to the cartesian pointcloud, default: "0,0,0,0,0,0" (i.e. no transform)
* @param[out] result scandata converted to ScanSegmentParserOutput
* @param[in] use_software_pll true (default): result timestamp from sensor ticks by software pll, false: result timestamp from msg receiving
* @param[in] verbose true: enable debug output, false: quiet mode
*/
bool sick_scansegment_xd::CompactDataParser::Parse(const ScanSegmentParserConfig& parser_config, const std::vector<uint8_t>& payload, fifo_timestamp system_timestamp, sick_scan_xd::SickCloudTransform& add_transform_xyz_rpy,
    ScanSegmentParserOutput& result, bool use_software_pll, bool verbose)
{
    // Parse segment data
    sick_scansegment_xd::CompactSegmentData segment_data;
    uint32_t payload_length_bytes = 0, num_bytes_required  = 0;
    if (!sick_scansegment_xd::CompactDataParser::ParseSegment(payload.data(), payload.size(), &segment_data, payload_length_bytes, num_bytes_required , add_transform_xyz_rpy.azimuthOffset()))
    {
        ROS_ERROR_STREAM("## ERROR CompactDataParser::Parse(): CompactDataParser::ParseSegment() failed, payload = " << sick_scansegment_xd::UdpReceiver::ToHexString(payload, payload.size()));
        return false;
    }
    // Convert segment data to ScanSegmentParserOutput
    sick_scansegment_xd::CompactDataHeader& segmentHeader = segment_data.segmentHeader;
    result.scandata.clear();
    result.imudata = segment_data.segmentHeader.imudata;
    result.segmentIndex = 0;
    result.telegramCnt = segmentHeader.telegramCounter;
    for (int module_idx = 0; module_idx < segment_data.segmentModules.size(); module_idx++)
    {
        sick_scansegment_xd::CompactModuleMetaData& moduleMetadata = segment_data.segmentModules[module_idx].moduleMetadata;
        sick_scansegment_xd::CompactModuleMeasurementData& moduleMeasurement = segment_data.segmentModules[module_idx].moduleMeasurement;
        if (!moduleMeasurement.valid)
        {
            ROS_ERROR_STREAM("## ERROR CompactDataParser::Parse(): invalid moduleMeasurement (ignored)");
            continue;
        }
        for (int measurement_idx = 0; measurement_idx < moduleMeasurement.scandata.size(); measurement_idx++)
        {
            ScanSegmentParserOutput::Scangroup& scandata = moduleMeasurement.scandata[measurement_idx];
            if (scandata.timestampStart_sec == 0)
                ROS_WARN_STREAM("## WARNING CompactDataParser::Parse(): scandata.timestampStart_sec=" << scandata.timestampStart_sec << ", compact_parser.cpp:" << __LINE__);
            // Apply optional range filter and optional transform
            for(int line_idx = 0; line_idx < scandata.scanlines.size(); line_idx++)
            {
                std::vector<ScanSegmentParserOutput::LidarPoint>& points_in = scandata.scanlines[line_idx].points;
                std::vector<ScanSegmentParserOutput::LidarPoint> points_out;
                points_out.reserve(points_in.size());
                for(int point_idx = 0; point_idx < points_in.size(); point_idx++)
                {
                    add_transform_xyz_rpy.applyTransform(points_in[point_idx].x, points_in[point_idx].y, points_in[point_idx].z);
                    points_out.push_back(points_in[point_idx]);
                }
                scandata.scanlines[line_idx].points = points_out;
            }
            // result.scandata.push_back(scandata);
            // Reorder lidar points by layer id (groupIdx) and echoIdx (identical to the msgpack scandata)
            // result.scandata[groupIdx] = all scandata of layer <groupIdx> appended to one scanline
            for(int line_idx = 0; line_idx < scandata.scanlines.size(); line_idx++)
            {
               std::vector<ScanSegmentParserOutput::LidarPoint>& points = scandata.scanlines[line_idx].points;
                for(int point_idx = 0; point_idx < points.size(); point_idx++)
                {
                    ScanSegmentParserOutput::LidarPoint& point = points[point_idx];
                    int groupIdx = point.groupIdx;
                    int echoIdx = point.echoIdx;
                    while(result.scandata.size() <= groupIdx)
                    {
                      result.scandata.push_back(ScanSegmentParserOutput::Scangroup());
                      result.scandata.back().timestampStart_sec = scandata.timestampStart_sec;
                      result.scandata.back().timestampStart_nsec = scandata.timestampStart_nsec;
                      result.scandata.back().timestampStop_sec = scandata.timestampStop_sec;
                      result.scandata.back().timestampStop_nsec = scandata.timestampStop_nsec;
                    }
                    while(result.scandata[groupIdx].scanlines.size() <= echoIdx)
                    {
                      result.scandata[groupIdx].scanlines.push_back(ScanSegmentParserOutput::Scanline());
                    }
                    if (result.scandata[groupIdx].scanlines[echoIdx].points.empty())
                    {
                        result.scandata[groupIdx].scanlines[echoIdx].points.reserve(points.size());
                    }
                    result.scandata[groupIdx].scanlines[echoIdx].points.push_back(point);
                }
            }
        }
        if (module_idx == 0)
        {
            result.segmentIndex = moduleMetadata.SegmentCounter;
        }
        else if (result.segmentIndex != moduleMetadata.SegmentCounter)
        {
            ROS_WARN_STREAM("## WARNING CompactDataParser::Parse(): different SegmentCounter in module 0 and module " << module_idx << " currently not supported,"
                << " scandata of segment " << moduleMetadata.SegmentCounter << " appended to segment " << result.segmentIndex);
        }
    }
    if (result.scandata.empty() && !result.imudata.valid)
    {
        ROS_ERROR_STREAM("## ERROR CompactDataParser::Parse(): CompactDataParser::ParseSegment() failed (no scandata found)");
        return false;
    }
    // Convert timestamp from sensor time to system time
    uint64_t sensor_timeStamp = segmentHeader.timeStampTransmit; // Sensor timestamp in microseconds since 1.1.1970 00:00 UTC
    if (result.imudata.valid)
        sensor_timeStamp -= parser_config.imu_latency_microsec;
    if (!result.scandata.empty())
        sensor_timeStamp = (uint64_t)result.scandata[0].timestampStart_sec * 1000000UL + (uint64_t)result.scandata[0].timestampStart_nsec / 1000; // i.e. start of scan in microseconds
    result.timestamp_sec = (sensor_timeStamp / 1000000);
    result.timestamp_nsec= 1000 * (sensor_timeStamp % 1000000);
    if (use_software_pll)
    {
        if (sensor_timeStamp == 0 && !result.scandata.empty())
        {
            sensor_timeStamp = (uint64_t)result.scandata[0].timestampStart_sec * 1000000UL + (uint64_t)result.scandata[0].timestampStart_nsec / 1000; // i.e. start of scan in microseconds
            ROS_WARN_STREAM("## WARNING CompactDataParser::Parse(): segmentHeader.timeStampTransmit=" << segmentHeader.timeStampTransmit 
                << ", result.scandata[0].timestampStart_sec=" << result.scandata[0].timestampStart_sec << ", result.scandata[0].timestampStart_nsec=" << result.scandata[0].timestampStart_nsec 
                << ", sensor_timeStamp=" << sensor_timeStamp << ", compact_parser.cpp:" << __LINE__);
        }
        else if(sensor_timeStamp == 0)
        {
            ROS_WARN_STREAM("## WARNING CompactDataParser::Parse(): segmentHeader.timeStampTransmit=" << segmentHeader.timeStampTransmit << ", sensor_timeStamp=" << sensor_timeStamp << ", compact_parser.cpp:" << __LINE__);
        }
        SoftwarePLL& software_pll = SoftwarePLL::instance();
        int64_t systemtime_nanoseconds = system_timestamp.time_since_epoch().count();
        uint32_t systemtime_sec = (uint32_t)(systemtime_nanoseconds / 1000000000);  // seconds part of system timestamp
        uint32_t systemtime_nsec = (uint32_t)(systemtime_nanoseconds % 1000000000); // nanoseconds part of system timestamp
        software_pll.updatePLL(systemtime_sec, systemtime_nsec, segmentHeader.timeStampTransmit, sensor_timeStamp);
        if (software_pll.IsInitialized())
        {
            uint32_t pll_sec = 0, pll_nsec = 0;
            software_pll.getCorrectedTimeStamp(pll_sec, pll_nsec, sensor_timeStamp);
            result.timestamp_sec = pll_sec;
            result.timestamp_nsec = pll_nsec;
        }
    }
    result.timestamp = sick_scansegment_xd::Timestamp(result.timestamp_sec, result.timestamp_nsec);

#if EXPORT_MEASUREMENT_AZIMUTH_ACCELERATION_CSV // Measurement of IMU latency (development only): Export ticks (imu resp. lidar timestamp in micro seconds), imu acceleration and lidar max azimuth of board cube
    ROS_INFO_STREAM("CompactDataParser::Parse(): header = " << segmentHeader.to_string() << ", system timestamp = " << result.timestamp);
    if (result.imudata.valid) // Export imu acceleration
    {
    	std::ofstream csv_ostream("/tmp/imu_latency.csv", std::ios::app);
      csv_ostream << segmentHeader.timeStampTransmit << ";" << ";" << std::fixed << std::setprecision(3) << (result.imudata.acceleration_z) << "\n";
    }
    else // Export lidar azimuth of calibration board
    {
      uint64_t timestamp_microsec_azi = 0;
      double azimuth_aperture = 0;
      if (getMaxAzimuthApertureWithinCube(result.scandata, 0.5f, 1.5f, -1.0f, +1.0f, -1.0f, +1.0f, -M_PI, +M_PI, azimuth_aperture, timestamp_microsec_azi))
      {
        std::ofstream csv_ostream("/tmp/imu_latency.csv", std::ios::app);
        csv_ostream << timestamp_microsec_azi << ";" << std::fixed << std::setprecision(3) << (azimuth_aperture * 180 / M_PI) << ";" << "\n";
      }
    }
#endif // EXPORT_MEASUREMENT_AZIMUTH_ACCELERATION_CSV

    return true;
}
