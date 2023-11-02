#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * @brief compact_parser parses and convertes scandata in compact format
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
 *  Copyright 2020 SICK AG
 *  Copyright 2020 Ing.-Buero Dr. Michael Lehning
 *
 */
#ifndef __SICK_SCANSEGMENT_XD_COMPACT_PARSER_H
#define __SICK_SCANSEGMENT_XD_COMPACT_PARSER_H

#include "sick_scan/sick_ros_wrapper.h"
#include <sick_scan/sick_cloud_transform.h>
#include "sick_scan/sick_range_filter.h"
#include "sick_scansegment_xd/common.h"
#include "sick_scansegment_xd/fifo.h"
#include "sick_scansegment_xd/scansegment_parser_output.h"


namespace sick_scansegment_xd
{
    /*
    * @brief class CompactDataHeader is a container for the header of scandata in compact format
    */
    class CompactDataHeader
    {
    public:
        uint32_t commandId = 0;           // Telegram type, expected value: 1 for scan data, 2 for imu data
        uint64_t telegramCounter = 0;     // Incrementing telegram counter, starting with 1, number of telegrams since power on
        uint64_t timeStampTransmit = 0;   // Sensor timestamp in microseconds since 1.1.1970 00:00 UTC
        uint32_t telegramVersion = 0;     // Telegram version, expected value: 3
        uint32_t sizeModule0 = 0;         // Size of first module in byte
        CompactImuData imudata;               // IMU data in case of commandId == 2;
        bool isImu() const { return commandId == 2 && imudata.valid; } // true if the telegram is imu data
        std::string to_string() const;    // returns a human readable description of the header data
    }; // class CompactDataHeader

    /*
    * @brief class CompactModuleMetaData is a container for the metadata of scandata module in compact format
    */
    class CompactModuleMetaData
    {
    public:
        uint64_t SegmentCounter = 0;           // Incrementing segment counter
        uint64_t FrameNumber = 0;              // Number of frames since power on
        uint32_t SenderId = 0;                 // The serial number of the device
        uint32_t NumberOfLinesInModule = 0;    // Number of layers in this module
        uint32_t NumberOfBeamsPerScan = 0;     // Number of beams per layer (all layers have the same number of beams)
        uint32_t NumberOfEchosPerBeam = 0;     // Number of echos per beams
        std::vector<uint64_t> TimeStampStart;  // Array of timestamps of the first beam in a scan in microseconds, number of elements is numberOfLinesInModule
        std::vector<uint64_t> TimeStampStop;   // Array of timestamps of the last beam in a scan in microseconds, number of elements is numberOfLinesInModule
        std::vector<float> Phi;                // Array of elevation angles in radians for each layer in this module, number of elements is numberOfLinesInModule
        std::vector<float> ThetaStart;         // Array of azimuth angles in radians for first beam of each layer in this module, number of elements is numberOfLinesInModule
        std::vector<float> ThetaStop;          // Array of azimuth angles in radians for last beam of each layer in this module, number of elements is numberOfLinesInModule
        float DistanceScalingFactor = 1;       // Scaling factor for 16-bit distance values: distance_mm = DistanceScalingFactor * distance_16bit_sensor
        uint32_t NextModuleSize = 0;           // Size of the next module (or 0 if this module is the last one)
        uint8_t Availability = 0;              // Reserved for futher use (flag indication distortion in scan data)
        uint8_t DataContentEchos = 0;          // Bitarray: (DataContentEchos & 0x01) == 0x01 if distance values available, (DataContentEchos & 0x02) == 0x02 if RSSI values available, (DataContentEchos & 0x03) == 0x03: distance and RSSI values available
        uint8_t DataContentBeams = 0;          // Bitarray: (DataContentBeams & 0x01) == 0x01 if beam properties available, (DataContentBeams & 0x02) == 0x02 if azimuth angle per beam available, (DataContentBeams & 0x03) == 0x03: both available
        uint8_t reserved = 0;                  // Reserved for 32 bit alignment
        bool valid = false;                    // valid flag set true after successful parsing
        std::string to_string() const;         // returns a human readable description of the module metadata
    }; // class CompactModuleMetaData

    /*
    * @brief class CompactModuleMeasurementData is a container for measurement data of a module in compact format
    */
    class CompactModuleMeasurementData
    {
    public:
        std::vector<ScanSegmentParserOutput::Scangroup> scandata;  // measurement data converted to ScanSegmentParserOutput scanlines
        bool valid = false;                                    // valid flag set true after successful parsing
        std::string to_string() const;                         // returns a human readable description of the module measurement data
    }; // class CompactModuleMeasurementData

    /*
    * @brief class CompactModuleData is a container for meta and measurement data of a module in compact format
    */
    class CompactModuleData
    {
    public:
        CompactModuleMetaData moduleMetadata;
        CompactModuleMeasurementData moduleMeasurement;
    }; // class CompactModuleData

    /*
    * @brief class CompactSegmentData is a container for segment data in compact format
    */
    class CompactSegmentData
    {
    public:
        CompactDataHeader segmentHeader;
        std::vector<CompactModuleData> segmentModules;
    }; // class CompactSegmentData

    /*
    * @brief class CompactDataParser parses scandata in compact format
    */
    class CompactDataParser
    {
    public:
    
        /*
        * @brief Parses scandata header in compact format.
        * @param[in] scandata 32 byte scandata header
        */
        static CompactDataHeader ParseHeader(const uint8_t* scandata);

        /*
        * @brief Parses module meta data in compact format.
        * @param[in] scandata received bytes
        * @param[in] module_size Number of bytes to parse
        * @param[in] telegramVersion compact format version, currently version 3 and 4 supported
        * @param[out] module_metadata_size number of bytes parsed (i.e. size of meta data in bytes if successful, i.e. if metadata.valid == true)
        */
        static CompactModuleMetaData ParseModuleMetaData(const uint8_t* scandata, uint32_t module_size, uint32_t telegramVersion, uint32_t& module_metadata_size);

        /*
        * @brief Parses module measurement data in compact format.
        * @param[in] payload binary payload
        * @param[in] num_bytes size of binary payload in bytes
        * @param[in] meta_data module metadata with measurement properties
        * @param[out] measurement_data parsed and converted module measurement data
        * @return true on success, false on error
        */
        static bool ParseModuleMeasurementData(const uint8_t* payload, uint32_t num_bytes, const sick_scansegment_xd::CompactDataHeader& compact_header, 
            const sick_scansegment_xd::CompactModuleMetaData& meta_data, float azimuth_offset, sick_scansegment_xd::CompactModuleMeasurementData& measurement_data);

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
        static bool ParseSegment(const uint8_t* payload, size_t bytes_received, sick_scansegment_xd::CompactSegmentData* segment_data,
            uint32_t& payload_length_bytes, uint32_t& num_bytes_required , float azimuth_offset = 0, int verbose = 0);

        /*
        * @brief Parses a scandata segment in compact format.
        * @param[in] parser_config configuration and settings for multiScan and picoScan parser
        * @param[in] payload binary segment data in compact format
        * @param[in] system_timestamp receive timestamp of segment_data (system time)
        * @param[in] add_transform_xyz_rpy Optionally apply an additional transform to the cartesian pointcloud, default: "0,0,0,0,0,0" (i.e. no transform)
        * @param[out] result scandata converted to ScanSegmentParserOutput
        * @param[in] use_software_pll true (default): result timestamp from sensor ticks by software pll, false: result timestamp from msg receiving
        * @param[in] verbose true: enable debug output, false: quiet mode
        */
        static bool Parse(const ScanSegmentParserConfig& parser_config, const std::vector<uint8_t>& payload, fifo_timestamp system_timestamp, sick_scan_xd::SickCloudTransform& add_transform_xyz_rpy, 
            ScanSegmentParserOutput& result, bool use_software_pll = true, bool verbose = false);

        /*
        * @brief Sets the elevation in mdeg for layers in compact format.
        * @param[in] layer_elevation_table_mdeg layer_elevation_table_mdeg[layer_idx] := ideal elevation in mdeg
        */
        static void SetLayerElevationTable(const std::vector<int>& layer_elevation_table_mdeg);

        /*
        * @brief Return a layer-id from a given elevation angle. See compact scanformat documention:
        * The line/layer index in the figure below is not a layer id according to layer numbering for multi layer sensors.
        * Therefore this functions returns a layer-id from the elevation angle in rad.
        * @param[in] layer_elevation_rad layer_elevation in radians
        * @return layer-id
        */
        static int GetLayerIDfromElevation(float layer_elevation_rad);

        /*
        * @brief Return the typical (default) elevation of a given layer index
        * @param[in] layer_idx layer index
        * @return layer elevation in degree
        */
        static float GetElevationDegFromLayerIdx(int layer_idx);

    }; // class CompactDataParser

} // namespace sick_scansegment_xd
#endif // __SICK_SCANSEGMENT_XD_COMPACT_PARSER_H
