/*
 * @brief msgpack_converter runs a background thread to unpack and parses msgpack data for the sick 3D lidar multiScan136.
 * msgpack_converter pops binary msgpack data from an input fifo, converts the data to scanlines using MsgPackParser::Parse()
 * and pushes the ScanSegmentParserOutput to an output fifo.
 *
 * Usage example:
 *
 * sick_scansegment_xd::UdpReceiver udp_receiver;
 * udp_receiver.Init("127.0.0.1", 2115, -1, true);
 * sick_scansegment_xd::MsgPackConverter msgpack_converter(udp_receiver.Fifo(), -1, true);
 * msgpack_converter.Start()
 * udp_receiver.Start();
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
#include "sick_scansegment_xd/config.h"
#include "sick_scansegment_xd/compact_parser.h"
#include "sick_scansegment_xd/msgpack_converter.h"
#include "sick_scansegment_xd/udp_receiver.h"

/*
 * @brief Default constructor.
 */
sick_scansegment_xd::MsgPackConverter::MsgPackConverter() : m_verbose(false), m_input_fifo(0), m_scandataformat(1), m_output_fifo(0), m_converter_thread(0), m_run_converter_thread(false)

{
}

/*
 * @brief Initializing constructor
 * @param[in] parser_config configuration and settings for multiScan and picoScan parser
 * @param[in] add_transform_xyz_rpy Apply an additional transform to the cartesian pointcloud, default: "0,0,0,0,0,0" (i.e. no transform)
 * @param[in] input_fifo input fifo buffering udp packages
 * @param[in] scandataformat ScanDataFormat: 1 for msgpack or 2 for compact scandata, default: 1
 * @param[in] msgpack_output_fifolength max. output fifo length (-1: unlimited, default: 20 for buffering 1 second at 20 Hz), elements will be removed from front if number of elements exceeds the fifo_length
 * @param[in] verbose true: enable debug output, false: quiet mode (default)
 */
sick_scansegment_xd::MsgPackConverter::MsgPackConverter(const ScanSegmentParserConfig& parser_config, const sick_scan_xd::SickCloudTransform& add_transform_xyz_rpy, sick_scansegment_xd::PayloadFifo* input_fifo, int scandataformat, int msgpack_output_fifolength, bool verbose)
    : m_parser_config(parser_config), m_verbose(verbose), m_active(false), m_input_fifo(input_fifo), m_scandataformat(scandataformat), m_converter_thread(0), m_run_converter_thread(false), m_msgpack_validator_enabled(false), m_discard_msgpacks_not_validated(false)
{
    m_output_fifo = new sick_scansegment_xd::Fifo<ScanSegmentParserOutput>(msgpack_output_fifolength);
    m_add_transform_xyz_rpy = add_transform_xyz_rpy;
}

/*
 * @brief Default destructor.
 */
sick_scansegment_xd::MsgPackConverter::~MsgPackConverter()
{
    Close();
}

/*
 * @brief Starts a background thread, pops msgpack data packages from input fifo, converts them
 * and pushes ScanSegmentParserOutput data to the output fifo.
 */
bool sick_scansegment_xd::MsgPackConverter::Start(void)
{
    m_run_converter_thread = true;
    m_converter_thread = new std::thread(&sick_scansegment_xd::MsgPackConverter::Run, this);
    return true;
}

/*
 * @brief Stops the converter thread
 */
void sick_scansegment_xd::MsgPackConverter::Close(void)
{
    m_run_converter_thread = false;
    if (m_output_fifo)
    {
        m_output_fifo->Shutdown();
    }
    if (m_converter_thread)
    {
        if (m_converter_thread->joinable())
            m_converter_thread->join();
        delete m_converter_thread;
        m_converter_thread = 0;
    }
    if (m_output_fifo)
    {
        delete m_output_fifo;
        m_output_fifo = 0;
    }
}

/*
 * @brief Configures msgpack validation, see MsgPackValidator for details
 * @param[in] msgpack_validator the msgpack validator
 * @param[in] msgpack_validator_enabled true: check msgpack data for out of bounds and missing scan data, false: no msgpack validation
 * @param[in] discard_msgpacks_not_validated true: msgpacks are discarded if scan data out of bounds detected, false: error message if a msgpack is not validated
 * @param[in] msgpack_validator_check_missing_scandata_interval check msgpack for missing scandata after collecting N msgpacks, default: N = 12 segments. Increase this value to tolerate udp packet drops. Use 12 to check each full scan.
 */
void sick_scansegment_xd::MsgPackConverter::SetValidator(sick_scansegment_xd::MsgPackValidator& msgpack_validator, bool msgpack_validator_enabled, bool discard_msgpacks_not_validated, int msgpack_validator_check_missing_scandata_interval)
{
    m_msgpack_validator = msgpack_validator;
    m_msgpack_validator_enabled = msgpack_validator_enabled;
    m_discard_msgpacks_not_validated = discard_msgpacks_not_validated;
    m_msgpack_validator_check_missing_scandata_interval = msgpack_validator_check_missing_scandata_interval;
}


/*
 * @brief Thread callback, runs the converter. Pops msgpack data from the input fifo, converts them und pushes ScanSegmentParserOutput data to the output fifo.
 */
bool sick_scansegment_xd::MsgPackConverter::Run(void)
{
    if (!m_input_fifo || !m_output_fifo)
    {
        ROS_ERROR_STREAM("## ERROR MsgPackConverter::Run(): MsgPackConverter not initialized.");
        return false;
    }
    try
    {
        sick_scansegment_xd::MsgPackValidatorData msgpack_validator_data_collector;
        for (size_t msgpack_cnt = 1; m_run_converter_thread; msgpack_cnt++)
        {
            std::vector<uint8_t> input_payload;
            fifo_timestamp input_timestamp;
            size_t input_counter = 0;
            if (m_input_fifo->Pop(input_payload, input_timestamp, input_counter))
            {
                try
                {
                    if (!m_active)
                    {
                        continue; // still in sopas initialization sequence
                    }
                    sick_scansegment_xd::ScanSegmentParserOutput msgpack_output;
                    bool parse_success = false;
                    if (m_scandataformat == SCANDATA_MSGPACK)
                    {
                        parse_success = sick_scansegment_xd::MsgPackParser::Parse(input_payload, input_timestamp, m_add_transform_xyz_rpy, msgpack_output, msgpack_validator_data_collector, 
                            m_msgpack_validator, m_msgpack_validator_enabled, m_discard_msgpacks_not_validated, true, m_verbose);
                    }
                    else if (m_scandataformat == SCANDATA_COMPACT)
                    {
                        parse_success = sick_scansegment_xd::CompactDataParser::Parse(m_parser_config, input_payload, input_timestamp, m_add_transform_xyz_rpy, msgpack_output);
                    }
                    else
                    {
                        ROS_ERROR_STREAM("## ERROR MsgPackConverter::Run(): invalid scandataformat configuration, unsupported scandataformat=" << m_scandataformat
                            << ", check configuration and use " << SCANDATA_MSGPACK << " for msgpack or " << SCANDATA_COMPACT << " for compact data");
                    }
                    if (parse_success)
                    {
                        size_t fifo_length = m_output_fifo->Push(msgpack_output, input_timestamp, input_counter);
                        if (m_verbose)
                        {
                            ROS_INFO_STREAM("MsgPackConverter::Run(): " << m_input_fifo->Size() << " messages in input fifo, " << fifo_length << " messages in output fifo.");
                        }
                    }
                    else
                    {
                        ROS_ERROR_STREAM("## ERROR MsgPackConverter::Run(): msgpack parse error");
                        if (m_verbose)
                        {
                            ROS_ERROR_STREAM("## ERROR MsgPackConverter::Run(): MsgPackParser::Parse() failed on " << input_payload.size() << " byte input data: " << sick_scansegment_xd::UdpReceiver::ToPrintableString(input_payload, input_payload.size()));
                        }
                    }
                    if (m_msgpack_validator_enabled) // validate msgpack data
                    {
                        if (m_msgpack_validator.validateNotOutOfBound(msgpack_validator_data_collector) == false)
                            ROS_ERROR_STREAM("## ERROR MsgPackConverter::Run(): msgpack out of bounds validation failed");
                        else if (m_verbose)
                            ROS_INFO_STREAM("MsgPackConverter::Run(): msgpack validation passed (no scandata out of bounds)");
                        if(msgpack_cnt >= m_msgpack_validator_check_missing_scandata_interval)
                        {
                            if (m_msgpack_validator.validateNoMissingScandata(msgpack_validator_data_collector) == false)
                                ROS_ERROR_STREAM("## ERROR MsgPackConverter::Run(): msgpack validation failed (scandata missing)");
                            else if (m_verbose)
                                ROS_INFO_STREAM("MsgPackConverter::Run(): msgpack validation passed (no scandata missing)");
                            msgpack_cnt = 0; // reset counter for next interval
                            msgpack_validator_data_collector = sick_scansegment_xd::MsgPackValidatorData(); // reset collected msgpack data
                        }
                    }
                }
                catch (std::exception & e)
                {
                    ROS_ERROR_STREAM("## ERROR MsgPackConverter::Run(): parse error " << e.what());
                }
            }
        }
        m_run_converter_thread = false;
        return true;
    }
    catch (std::exception & e)
    {
        ROS_ERROR_STREAM("## ERROR MsgPackConverter::Run(): " << e.what());
    }
    m_run_converter_thread = false;
    return false;
}
