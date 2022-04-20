/*
 * @brief mrs100_curl implements a rest-api to post start and stop commands to the multiScan136.
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
 */
#include "sick_lidar3d/common.h"
#include "sick_lidar3d/json_parser.h"
#include "sick_lidar3d/mrs100_curl.h"

#include <curl/curl.h>

static const std::string crown_api = "/api/";

 /*
 ** @brief curl write callback for response data from server. Since we expect json strings, we can just append the response data to the output string.
 */
static size_t curlWriteCb(void* contents, size_t size, size_t nmemb, void* userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

/*
** @brief Shortens multiple consecutive spaces to one space
*/
static std::string shortenSpaces(const std::string& src)
{
    std::string dst;
    dst.reserve(src.length());
    bool last_char_is_space = false;
    for (int n = 0; n < src.length(); n++)
    {
        if (!std::isspace(src[n]))
        {
            dst.push_back(src[n]);
            last_char_is_space = false;

        }
        else if (!last_char_is_space)
        {
            dst.push_back(' ');
            last_char_is_space = true;
        }
    }
    return dst;
}

/*
 * @brief Posts a message via REST-API using libcurl
 */
static bool post(CURL* curl, const std::string& url, const std::string& fields, std::string& response)
{
    CURLcode curl_res;
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, fields.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlWriteCb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_res = curl_easy_perform(curl);
    bool success = (curl_res == CURLE_OK);
    if(success)
    {
#ifndef WIN32
        // Expected response: {"header":{"status":0,"message":"Ok"},"data":{"ErrorCode":0}}
        std::map<std::string, sick_lidar3d::JsonValue> json_response = sick_lidar3d::JsonParser::parseRestResponseData(response);
        int response_error_code = json_response["/data/ErrorCode"].toInt();
        if (response_error_code != 0)
        {
            LIDAR3D_ERROR_STREAM("## ERROR MRS100CURL::postStart(): message \"" << url << "\" with fields \"" << fields << "\" posted, response: \"" << shortenSpaces(response) << "\", error_code: " << response_error_code << ", expected error_code 0");
            success = false;
        }
        else
        {
            LIDAR3D_INFO_STREAM("MRS100CURL::postStart(): message \"" << url << "\" with fields \"" << fields << "\" posted, response: \"" << shortenSpaces(response) << "\", error_code: " << response_error_code);
        }
#endif // WIN32
    }
    else
    {
        LIDAR3D_ERROR_STREAM("## ERROR MRS100CURL::postStart(): Failed to post message \"" << url << "\" with fields \"" << fields << "\", response=\"" << shortenSpaces(response) << "\", CURLcode=" << curl_res);
    }
    return success;
}

/*
 * @brief Posts the start command to multiScan136
 * @param[in] mrs100_ip ip address of a mrs100, f.e. "127.0.0.1" (localhost, loopback) for an emulator or "192.168.0.1" for multiScan136
 * @param[in] mrs100_dst_ip UDP destination IP address (ip address of udp receiver)
 * @param[in] udp_port ip port, f.e. 2115 (default port for multiScan136 emulator)
 */
bool sick_lidar3d::MRS100CURL::postStart(const std::string& mrs100_ip, const std::string& mrs100_dst_ip, int udp_port)
{
    bool okay = true;
    std::string response, request, fields;
    CURL* curl = curl_easy_init();
    if (!curl)
    {
        LIDAR3D_ERROR_STREAM("## ERROR :MRS100CURL::postStart(): curl initialization error");
        return false;
    }
    /*
    request = std::string("http://") + mrs100_ip + crown_api + "setIpAddress";
    fields = std::string("{\"args\": {\"ipAddress\": \"") + mrs100_dst_ip + "\"}}";
    if (!post(curl, request, fields, response))
        okay = false;
    
    request = std::string("http://") + mrs100_ip + crown_api + "setPort";
    fields = std::string("{\"args\": {\"port\": ") + std::to_string(udp_port) + "}}";
    if (!post(curl, request, fields, response))
        okay = false;
    */
    request = std::string("http://") + mrs100_ip + crown_api + "mStartMeasure";
    fields = "";
    if (!post(curl, request, fields, response))
        okay = false;

    curl_easy_cleanup(curl);
    if (!okay)
        LIDAR3D_ERROR_STREAM("## ERROR :MRS100CURL::postStart() failed");
    return okay;
}

/*
 * @brief Posts the stop command to mrs100
 * @param[in] ip ip address of a mrs100, f.e. "127.0.0.1" (localhost, loopback) for an emulator or "192.168.0.1" for multiScan136
 * @param[in] udp_port ip port, f.e. 2115 (default port for multiScan136 emulator)
 */
bool sick_lidar3d::MRS100CURL::postStop(const std::string& ip, int udp_port)
{
    bool okay = true;;
    std::string response, request, fields;
    CURL* curl = curl_easy_init();
    if (!curl)
    {
        LIDAR3D_ERROR_STREAM("## ERROR :MRS100CURL::postStop(): curl initialization error");
        return false;
    }

    request = std::string("http://") + ip + crown_api + "mStopMeasure";
    fields = "";
    if (!post(curl, request, fields, response))
        okay = false;

    curl_easy_cleanup(curl);
    if (!okay)
        LIDAR3D_ERROR_STREAM("## ERROR :MRS100CURL::postStop() failed");
    return okay;
}
