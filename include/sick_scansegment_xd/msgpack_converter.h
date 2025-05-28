#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
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
#ifndef __SICK_SCANSEGMENT_XD_MSGPACK_CONVERTER_H
#define __SICK_SCANSEGMENT_XD_MSGPACK_CONVERTER_H

#include "sick_scan/sick_ros_wrapper.h"
#include <sick_scan/sick_cloud_transform.h>
#include "sick_scan/sick_range_filter.h"
#include "sick_scansegment_xd/common.h"
#include "sick_scansegment_xd/fifo.h"
#include "sick_scansegment_xd/msgpack_parser.h"
#include "sick_scansegment_xd/msgpack_validator.h"

namespace sick_scansegment_xd
{
	/*
     * @brief class MsgPackConverter runs a background thread to unpack and parses msgpack data for the sick 3D lidar multiScan136.
     * msgpack_converter pops binary msgpack data from an input fifo, converts the data to scanlines using MsgPackParser::Parse()
     * and pushes the ScanSegmentParserOutput to an output fifo.
     */
	class MsgPackConverter
	{
	public:

        /*
         * @brief Default constructor.
         */
        MsgPackConverter();

        /*
         * @brief Initializing constructor
         * @param[in] parser_config configuration and settings for multiScan and picoScan parser
         * @param[in] add_transform_xyz_rpy Apply an additional transform to the cartesian pointcloud, default: "0,0,0,0,0,0" (i.e. no transform)
         * @param[in] input_fifo input fifo buffering udp packages
         * @param[in] scandataformat ScanDataFormat: 1 for msgpack or 2 for compact scandata, default: 2
         * @param[in] msgpack_output_fifolength max. output fifo length (-1: unlimited, default: 20 for buffering 1 second at 20 Hz), elements will be removed from front if number of elements exceeds the fifo_length
         * @param[in] verbose true: enable debug output, false: quiet mode (default)
         */
         MsgPackConverter(const ScanSegmentParserConfig& parser_config, const sick_scan_xd::SickCloudTransform& add_transform_xyz_rpy, sick_scansegment_xd::PayloadFifo* input_fifo, int scandataformat = 2, int msgpack_output_fifolength = 20, bool verbose = false);

        /*
         * @brief Default destructor.
         */
        ~MsgPackConverter();

        /*
         * @brief Starts a background thread, pops msgpack data packages from input fifo, converts them
         * and pushes ScanSegmentParserOutput data to the output fifo.
         */
        bool Start(void);

        /*
         * @brief Stops the converter thread 
         */
        void Close(void);

        /*
         * @brief Configures msgpack validation, see MsgPackValidator for details
         * @param[in] msgpack_validator the msgpack validator
         * @param[in] msgpack_validator_enabled true: check msgpack data for out of bounds and missing scan data, false: no msgpack validation
         * @param[in] discard_msgpacks_not_validated true: msgpacks are discarded if scan data out of bounds detected, false: error message if a msgpack is not validated
         * @param[in] msgpack_validator_check_missing_scandata_interval check msgpack for missing scandata after collecting N msgpacks, default: N = 12 segments. Increase this value to tolerate udp packet drops. Use 12 to check each full scan.
         */
        void SetValidator(sick_scansegment_xd::MsgPackValidator& msgpack_validator, bool msgpack_validator_enabled, bool discard_msgpacks_not_validated, int msgpack_validator_check_missing_scandata_interval); 

        /*
         * @brief Returns the output fifo storing the multiScan136 scanlines.
         */
        sick_scansegment_xd::Fifo<ScanSegmentParserOutput>* Fifo(void) { return m_output_fifo; }

        /*
        * Activates resp. deactivates telegram parsing
        */
        virtual void SetActive(bool active)
        {
            m_active = active;
        }


   protected:

       /*
        * @brief Thread callback, runs the converter. Pops msgpack data from the input fifo, converts them und pushes ScanSegmentParserOutput data to the output fifo.
        */
       bool Run(void);

       /*
        * Configuration and parameter
        */
       bool m_verbose = false;                                  // true: enable debug output, false: quiet mode (default)
       ScanSegmentParserConfig m_parser_config;                 // configuration and settings for multiScan and picoScan parser
       bool m_active = false;                                   // start inactive and run telegram parsing AFTER initialization completed
       
        /*
         * Member data to run the converter
         */
       PayloadFifo* m_input_fifo = 0;                               // input fifo for msgpack data
       int m_scandataformat = 2;                                    // ScanDataFormat: 1 for msgpack or 2 for compact scandata, default: 1
       sick_scansegment_xd::Fifo<ScanSegmentParserOutput>* m_output_fifo = 0;  // output fifo for ScanSegmentParserOutput data converted from  msgpack data
       std::thread* m_converter_thread = 0;                         // background thread to convert msgpack to ScanSegmentParserOutput data
       bool m_run_converter_thread = false;                         // flag to start and stop the udp converter thread
       bool m_msgpack_validator_enabled = false;                    // true: check msgpack data for out of bounds and missing scan data, false: no msgpack validation
       sick_scansegment_xd::MsgPackValidator m_msgpack_validator;   // msgpack validation, see MsgPackValidator for details
       bool m_discard_msgpacks_not_validated;                       // true: msgpacks are discarded if scan data out of bounds detected, false: error message if a msgpack is not validated
       int m_msgpack_validator_check_missing_scandata_interval;     // check msgpack for missing scandata after collecting N msgpacks, default: N = 12 segments. Increase this value to tolerate udp packet drops. Use 12 to check each full scan.
       sick_scan_xd::SickCloudTransform m_add_transform_xyz_rpy;    // Apply an additional transform to the cartesian pointcloud, default: "0,0,0,0,0,0" (i.e. no transform)
	};  // class MsgPackConverter

}   // namespace sick_scansegment_xd
#endif // __SICK_SCANSEGMENT_XD_MSGPACK_CONVERTER_H
