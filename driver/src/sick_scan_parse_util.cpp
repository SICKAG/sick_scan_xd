/*
* Copyright (C) 2017, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2017, SICK AG, Waldkirch
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
 *  Created on: 24.05.2012
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 * Utility functions for parsing scanner specific sopas requests and responses
 *
 */
#include <iomanip>
#include <sick_scan/sick_scan_common.h>
#include <sick_scan/sick_scan_parse_util.h>

#if 0  // debugging and unittest
class SickScanParseUtilUnittest
{
public:
    SickScanParseUtilUnittest()
    {
        runUnittest();
    }
    void runUnittest()
    {
        std::string sopas_reply = "sRA LMPscancfg \\x00\\x00\\x03\\x20\\x00\\x01\\x00\\x00\\x09\\xc4\\x00\\x00\\x00\\x00\\x00\\x36\\xee\\x80\\x00\\x00\\x09\\xc4\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x09\\xc4\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x09\\xc4\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00";
        sick_scan_xd::SickScanParseUtil::LMPscancfg scancfg;
        if (!sick_scan_xd::SickScanParseUtil::SopasToLMPscancfg(sopas_reply, scancfg))
            std::cerr << "## ERROR SickScanParseUtil::SopasToLMPscancfg failed: " << scancfg.print() << std::endl;
        else
            std::cout << "SickScanParseUtil::SopasToLMPscancfg success: " << scancfg.print() << std::endl;
        std::string sopas_cmd;
        if (!sick_scan_xd::SickScanParseUtil::LMPscancfgToSopas(scancfg, sopas_cmd))
            std::cerr << "## ERROR SickScanParseUtil::LMPscancfgToSopas failed: \"" << sopas_cmd << "\"" << std::endl;
        else
            std::cout << "SickScanParseUtil::LMPscancfgToSopas success: \"" << sopas_cmd << "\"" << std::endl;
    }
};
static SickScanParseUtilUnittest selftester = SickScanParseUtilUnittest();
#endif  // debugging and unittest

// returns the given angle in rad normalized to -PI ... +PI
double sick_scan_xd::normalizeAngleRad(double angle_rad, double angle_min, double angle_max)
{
while(angle_rad > angle_max)
    angle_rad -= (2 * M_PI);
while(angle_rad < angle_min)
    angle_rad += (2 * M_PI);
return angle_rad;
}

// Converts a string to a 6D pose x,y,z,roll,pitch,yaw in [m] resp. [rad]
std::vector<float> sick_scan_xd::parsePose(const std::string& pose_xyz_rpy_str)
{
    std::istringstream config_stream(pose_xyz_rpy_str);
    std::string config_arg;
    std::vector<float> config_values;
    while (getline(config_stream, config_arg, ','))
    {
        // ROS-2 interpretes parameter values configured by "ros2 param set <node> <arg> <value>" as yaml content, 
        // but does not remove yaml escape characters required for negative numbers. Any '\\' is removed here.
        std::string::size_type n = 0;
        while ((n = config_arg.find('\\', n)) != std::string::npos)
            config_arg.replace( n, 1, "");
        try
        {
            float arg_value = std::stof(config_arg);
            config_values.push_back(arg_value);
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM("## ERROR sick_scan_xd::parsePose(): parse error in string \"" << pose_xyz_rpy_str << "\", arg=\"" << config_arg << "\", exception " << e.what());
        }
    }
    return config_values;
}

template<typename T> static bool convertBin(const std::string& sopas_string, size_t& offset, T& value)
{
    value = 0;
    if (sopas_string.size() >= offset + 4 * sizeof(value))
    {
        for (int byte_cnt = 0; byte_cnt < sizeof(value); byte_cnt++)
        {
            std::string hex_str = sopas_string.substr(offset + 4 * byte_cnt + 2, 2);
            unsigned long cur_byte = std::stoul(hex_str, nullptr, 16);
            value = (value << 8) | (cur_byte & 0xFF);
        }
        offset += 4 * sizeof(value);
        return true;
    }
    else
    {
        ROS_WARN_STREAM("## ERROR in SickScanParseUtil::convertBin(\"" << sopas_string << "\", offset=" << offset << ", sizeof(value)=" << sizeof(value) << "): value not readable, reached end of string");
    }
    return false;
}

template<typename T> static std::string convertHex(T value)
{
    std::stringstream s;
    for (int byte_cnt = (int)sizeof(value) - 1; byte_cnt >= 0; byte_cnt--)
    {
        s << "\\x" << std::setfill('0') << std::setw(2) << std::hex << ((value >> 8 * byte_cnt) & 0xFF);
    }
    return s.str();
}

std::string sick_scan_xd::SickScanParseUtil::LMPscancfg::print() const
{
    std::stringstream scancfg_msg;
    scancfg_msg << "scan_frequency=" << scan_frequency << ", active_sector_cnt=" << active_sector_cnt;
    for (int sector_cnt = 0; sector_cnt < sector_cfg.size(); sector_cnt++)
    {
        scancfg_msg << ", angular_resolution[" << sector_cnt << "]=" << sector_cfg[sector_cnt].angular_resolution
            << ", start_angle[" << sector_cnt << "]=" << sector_cfg[sector_cnt].start_angle
            << ", stop_angle[" << sector_cnt << "]=" << sector_cfg[sector_cnt].stop_angle;
    }
    return scancfg_msg.str();
}

/*
* @brief Parse the sopas reply to "sRN LMPscancfg" and convert to LMPscancfg
*
* LRS-36x1 reply to "sRN LMPscancfg" (Cola-B example): "sRA LMPscancfg \x00\x00\x03\x20\x00\x01\x00\x00\x09\xc4\x00\x00\x00\x00\x00\x36\xee\x80\x00\x00\x09\xc4\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x09\xc4\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x09\xc4\x00\x00\x00\x00\x00\x00\x00\x00"
*/
bool sick_scan_xd::SickScanParseUtil::SopasToLMPscancfg(const std::string& sopas_reply, sick_scan_xd::SickScanParseUtil::LMPscancfg& scancfg)
{
    scancfg = sick_scan_xd::SickScanParseUtil::LMPscancfg();
    size_t offset = 0;
    // Check for LMPscancfg or mLMPsetscancfg reply
    std::vector<std::string> sopas_reply_ids = { "sRA LMPscancfg " , "sAN mLMPsetscancfg " };
    if (strncmp(sopas_reply.data(), sopas_reply_ids[0].data(), sopas_reply_ids[0].size()) == 0)
    {
        offset = sopas_reply_ids[0].size();
    }
    else if (strncmp(sopas_reply.data(), sopas_reply_ids[1].data(), sopas_reply_ids[1].size()) == 0)
    {
        // Read 1 byte status code after "sAN mLMPsetscancfg "
        offset = sopas_reply_ids[1].size(); 
        uint8_t status_code = 0;
        convertBin(sopas_reply, offset, status_code);
        if (status_code != 0)
            ROS_WARN_STREAM("SickScanParseUtil::SopasToLMPscancfg(): status code " << (uint32_t)(status_code) << " in reply: \"" << sopas_reply << "\" indicates an ERROR");
    }
    else
    {
        ROS_WARN_STREAM("## ERROR in SickScanParseUtil::SopasToLMPscancfg: \"" << sopas_reply << "\" not supported");
        return false;
    }
    bool success = convertBin(sopas_reply, offset, scancfg.scan_frequency); // scan frequency in 1/100 Hz
    success = success && convertBin(sopas_reply, offset, scancfg.active_sector_cnt); // number of active sectors
    int max_sector_cnt = 4; // scancfg.active_sector_cnt // always 4 sectors are transmitted by sopas
    scancfg.sector_cfg.reserve(max_sector_cnt);
    for (int sector_cnt = 0; success == true && sector_cnt < max_sector_cnt && offset < sopas_reply.length(); sector_cnt++)
    {
        sick_scan_xd::SickScanParseUtil::LMPscancfgSector scancfg_sector;
        success = success && convertBin(sopas_reply, offset, scancfg_sector.angular_resolution); // angular resolution in 1/10000 deg
        success = success && convertBin(sopas_reply, offset, scancfg_sector.start_angle); // start angle in 1/10000 deg
        success = success && convertBin(sopas_reply, offset, scancfg_sector.stop_angle); // stop angle in 1/10000 deg
        scancfg.sector_cfg.push_back(scancfg_sector);
    }
    if (!success)
    {
        ROS_WARN_STREAM("## ERROR in LMPscancfg reply: \"" << sopas_reply << "\"");
        ROS_WARN_STREAM("## SickScanParseUtil::SopasToLMPscancfg(): convertBin() failed with " << scancfg.print());
        scancfg = sick_scan_xd::SickScanParseUtil::LMPscancfg();
    }
    else
    {
        ROS_INFO_STREAM("LMPscancfg reply: \"" << sopas_reply << "\"");
        ROS_INFO_STREAM("LMPscancfg: { " << scancfg.print() << " }");
    }
    return success;
}

/*
* @brief Convert LMPscancfg to sopas request "sMN mLMPsetscancfg ..."
* Example: "\x02sMN mLMPsetscancfg +2000 +1 +7500 +3600000 0 +2500 0 0 +2500 0 0 +2500 0 0\x03"

*/
bool sick_scan_xd::SickScanParseUtil::LMPscancfgToSopas(const sick_scan_xd::SickScanParseUtil::LMPscancfg& scancfg, std::string& sopas_cmd)
{
    sopas_cmd = "";
    std::stringstream sopas_hex;
    sopas_hex << "\x02sMN mLMPsetscancfg ";
    sopas_hex << convertHex(scancfg.scan_frequency);
    sopas_hex << convertHex(scancfg.active_sector_cnt);
    for (int sector_cnt = 0; sector_cnt < scancfg.sector_cfg.size(); sector_cnt++)
    {
        sopas_hex << convertHex(scancfg.sector_cfg[sector_cnt].angular_resolution);
        sopas_hex << convertHex(scancfg.sector_cfg[sector_cnt].start_angle);
        sopas_hex << convertHex(scancfg.sector_cfg[sector_cnt].stop_angle);
    }
    sopas_hex << "\x03";
    sopas_cmd = sopas_hex.str();
    return true;
}
