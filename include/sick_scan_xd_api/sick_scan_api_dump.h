/*
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

#ifndef __SICK_SCAN_API_DUMP_H_INCLUDED
#define __SICK_SCAN_API_DUMP_H_INCLUDED

#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

/*
** Optional message verification for test and development:
**   0 = disabled (default), 
**   1 = dump all messages exported by sick_scan API
** For verification of messages exported by the sick_scan API:
**   - Set VERIFY_API_MESSAGES 1
**   - Run sick_scan_api_test in ROS-1 against lidar or emulator
**   - Compare the dumped messages in the logfolder ("./log"):
**     * files api_*_impl.log are generated in api_impl,
**     * files api_*_test.log are received via sick_scan_api and converted back to ros messages
**     * files api_*_impl.log and api_*_test.log should be identical
*/
#define VERIFY_API_MESSAGES 0 // Optional message verification for test and development only (default: 0)

#if VERIFY_API_MESSAGES

#define DUMP_API_POINTCLOUD_MESSAGE(postfix,msg)           SickScanApiDump::dumpMessage("log", "api_cartesian_cloud", postfix, msg);
#define DUMP_API_IMU_MESSAGE(postfix,msg)                  SickScanApiDump::dumpMessage("log", "api_imu", postfix, msg);
#define DUMP_API_LFEREC_MESSAGE(postfix,msg)               SickScanApiDump::dumpMessage("log", "api_lferec", postfix, msg);
#define DUMP_API_LIDOUTPUTSTATE_MESSAGE(postfix,msg)       SickScanApiDump::dumpMessage("log", "api_lidoutputstate", postfix, msg);
#define DUMP_API_RADARSCAN_MESSAGE(postfix,msg)            SickScanApiDump::dumpMessage("log", "api_radarscan", postfix, msg);
#define DUMP_API_LDMRSOBJECTARRAY_MESSAGE(postfix,msg)     SickScanApiDump::dumpMessage("log", "api_ldmrsobjects", postfix, msg);
#define DUMP_API_VISUALIZATIONMARKER_MESSAGE(postfix,msg)  SickScanApiDump::dumpMessage("log", "api_marker", postfix, msg);

#else

#define DUMP_API_POINTCLOUD_MESSAGE(postfix,msg)
#define DUMP_API_IMU_MESSAGE(postfix,msg)
#define DUMP_API_LFEREC_MESSAGE(postfix,msg)
#define DUMP_API_LIDOUTPUTSTATE_MESSAGE(postfix,msg)
#define DUMP_API_RADARSCAN_MESSAGE(postfix,msg)
#define DUMP_API_LDMRSOBJECTARRAY_MESSAGE(postfix,msg)
#define DUMP_API_VISUALIZATIONMARKER_MESSAGE(postfix,msg)

#endif

class SickScanApiDump
{
public:
    /** Shortcut to dump a message to file, output file is named "<logfolder>/<prefix>_<timestamp_millisec>_<postfix>.log" */
    template <class MsgType> static void dumpMessage(const std::string& logfolder, const std::string& prefix, const std::string& postfix, const MsgType& msg)
    {
        uint64_t timestamp_millisec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        std::stringstream filepath;
        filepath << logfolder << "/" << prefix << "_" << std::setfill('0') << std::setw(16) << timestamp_millisec << "_" << postfix << ".log";
        std::ofstream fout(filepath.str(), std::fstream::out);
        if (fout.is_open())
        {
            fout << msg << std::endl;
            fout.close();
        }
    }

}; // SickScanApiDump

#endif // __SICK_SCAN_API_DUMP_H_INCLUDED
