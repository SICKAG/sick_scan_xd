/*
 * @brief json_parser wraps json parsing and conversion using jsoncpp.
 *
 * Copyright (C) 2021 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2021 SICK AG, Waldkirch
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
 *  Copyright 2021 SICK AG
 *  Copyright 2021 Ing.-Buero Dr. Michael Lehning
 *
 */
#ifdef _MSC_VER
#include <json/json.h>
#else
#include <jsoncpp/json/json.h>
#endif

#include "sick_lidar3d/common.h"
#include "sick_lidar3d/json_parser.h"

bool sick_lidar3d::JsonValue::toBool(void) const
{
    bool val = false;
    switch (m_type)
    {
    case sick_lidar3d::JsonValue::BOOL:
        val = m_b_val;
        break;
    case sick_lidar3d::JsonValue::INT:
        val = (m_i_val != 0);
        break;
    case sick_lidar3d::JsonValue::STRING:
        val = (m_s_val.size() > 0 && (m_s_val[0] == '1' || m_s_val[0] == 'T' || m_s_val[0] == 't'));
        break;
    default:
        break;
    }
    return val;
}

int64_t sick_lidar3d::JsonValue::toInt(void) const
{
    int64_t val = 0;
    switch (m_type)
    {
    case sick_lidar3d::JsonValue::BOOL:
        val = (m_b_val ? 1 : 0);
        break;
    case sick_lidar3d::JsonValue::INT:
        val = m_i_val;
        break;
    case sick_lidar3d::JsonValue::DOUBLE:
        val = (int64_t)m_d_val;
        break;
    case sick_lidar3d::JsonValue::STRING:
        val = std::strtoll(m_s_val.c_str(), 0, 0);
        break;
    default:
        break;
    }
    return val;
}

double sick_lidar3d::JsonValue::toDouble(void) const
{
    double val = 0;
    switch (m_type)
    {
    case sick_lidar3d::JsonValue::BOOL:
        val = (m_b_val ? 1 : 0);
        break;
    case sick_lidar3d::JsonValue::INT:
        val = (double)m_i_val;
        break;
    case sick_lidar3d::JsonValue::DOUBLE:
        val = m_d_val;
        break;
    case sick_lidar3d::JsonValue::STRING:
        val = (double)std::strtold(m_s_val.c_str(), 0);
        break;
    default:
        break;
    }
    return val;
}

std::string sick_lidar3d::JsonValue::toString(void) const
{
    std::string val("");
    switch (m_type)
    {
    case sick_lidar3d::JsonValue::BOOL:
        val = (m_b_val ? "true" : "false");
        break;
    case sick_lidar3d::JsonValue::INT:
        val = std::to_string(m_i_val);
        break;
    case sick_lidar3d::JsonValue::DOUBLE:
        val = std::to_string(m_d_val);
        break;
    case sick_lidar3d::JsonValue::STRING:
        val = m_s_val;
        break;
    default:
        break;
    }
    return val;
}

sick_lidar3d::JsonValue::Type sick_lidar3d::JsonValue::type(void) const
{
    return m_type;
}

std::string sick_lidar3d::JsonValue::typeString(void) const
{
    std::string val("invalid");
    switch (m_type)
    {
    case sick_lidar3d::JsonValue::BOOL:
        val = "bool";
        break;
    case sick_lidar3d::JsonValue::INT:
        val = "int";
        break;
    case sick_lidar3d::JsonValue::DOUBLE:
        val = "double";
        break;
    case sick_lidar3d::JsonValue::STRING:
        val = "string";
        break;
    default:
        break;
    }
    return val;
}

/*
** @brief Recursive parsing of all json values.
*/
static void parseJsonRecursive(const std::string& key, Json::Value& json_value, std::map<std::string, sick_lidar3d::JsonValue>& key_value_pairs)
{
    if (json_value.type() == Json::ValueType::intValue)
    {
        key_value_pairs[key] = sick_lidar3d::JsonValue((int64_t)json_value.asInt64());
    }
    else if (json_value.type() == Json::ValueType::uintValue)
    {
        key_value_pairs[key] = sick_lidar3d::JsonValue((int64_t)json_value.asUInt64());
    }
    else if (json_value.type() == Json::ValueType::realValue)
    {
        key_value_pairs[key] = sick_lidar3d::JsonValue(json_value.asDouble());
    }
    else if (json_value.type() == Json::ValueType::stringValue)
    {
        key_value_pairs[key] = sick_lidar3d::JsonValue(json_value.asString());
    }
    else if (json_value.type() == Json::ValueType::booleanValue)
    {
        key_value_pairs[key] = sick_lidar3d::JsonValue(json_value.asBool());
    }
    else if (json_value.type() == Json::ValueType::arrayValue)
    {
        for (Json::Value::ArrayIndex json_idx = 0; json_idx != json_value.size(); json_idx++)
        {
            parseJsonRecursive(key + "/" + std::to_string(json_idx), json_value[json_idx], key_value_pairs);
        }
    }
    else if (json_value.type() == Json::ValueType::objectValue)
    {
        Json::Value::Members members = json_value.getMemberNames();
        for (Json::Value::ArrayIndex json_idx = 0; json_idx != members.size(); json_idx++)
        {
            std::string member_name = members[json_idx];
            parseJsonRecursive(key + "/" + member_name, json_value[member_name], key_value_pairs);
        }
    }
}

/*
** @brief Parses the response data of a http GET or POST request and returns a map of key-value pairs.
** Example: json_values = parseRestResponseData("{'header': {'status': 0, 'message': 'Ok'}, 'data': {'success': True}}")
** returns a map json_values["success"] := "True" resp. toBool(json_values["success"]) == true.
**
** @param json_msg json response from SIM localization server
** @param verbose if verbose>0: print key-value pairs, otherwise silent except for error messages
**
** @return map of key-value pairs
*/
std::map<std::string, sick_lidar3d::JsonValue> sick_lidar3d::JsonParser::parseRestResponseData(const std::string& json_msg, int verbose)
{
    std::map<std::string, JsonValue> key_value_pairs;
    Json::Reader json_reader;
    std::istringstream json_istream(json_msg);
    Json::Value json_root;

    json_reader.parse(json_istream, json_root);
    parseJsonRecursive("", json_root, key_value_pairs);

    for (std::map<std::string, JsonValue>::const_iterator iter = key_value_pairs.cbegin(); verbose && iter != key_value_pairs.cend(); iter++)
        LIDAR3D_INFO_STREAM("json_map[\"" << iter->first << "\"] : " << iter->second.toString() << " (type " << iter->second.typeString() << ")");

    return key_value_pairs;
}
