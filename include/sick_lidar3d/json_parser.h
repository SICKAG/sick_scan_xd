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
#ifndef __SICK_LIDAR3D_JSON_PARSER_H_INCLUDED
#define __SICK_LIDAR3D_JSON_PARSER_H_INCLUDED

#include <map>
#include <string>

namespace sick_lidar3d
{
    /*
    ** @brief class JsonValue represents a type (bool, int, double or string) and a value
    */
    class JsonValue
    {
    public:
        enum Type { INVALID = 0, BOOL, INT, DOUBLE, STRING };
        JsonValue() : m_type(INVALID), m_b_val(false), m_i_val(0), m_d_val(0), m_s_val("") {}
        JsonValue(bool val) : m_type(BOOL), m_b_val(val), m_i_val(0), m_d_val(0), m_s_val("") {}
        JsonValue(int64_t val) : m_type(INT), m_b_val(0), m_i_val(val), m_d_val(0), m_s_val("") {}
        JsonValue(double val) : m_type(DOUBLE), m_b_val(0), m_i_val(0), m_d_val(val), m_s_val("") {}
        JsonValue(const std::string& val) : m_type(STRING), m_b_val(0), m_i_val(0), m_d_val(0), m_s_val(val) {}
        bool toBool(void) const;
        int64_t toInt(void) const;
        double toDouble(void) const;
        std::string toString(void) const;
        Type type(void) const;
        std::string typeString(void) const;
    protected:
        Type m_type;
        bool m_b_val;
        int64_t m_i_val;
        double m_d_val;
        std::string m_s_val;
    };

    /*
    ** @brief class JsonParser wraps json parsing and conversion using jsoncpp
    */
    class JsonParser
    {
    public:

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
        static std::map<std::string, JsonValue> parseRestResponseData(const std::string& json_msg, int verbose = 0);

    }; // class JsonParser

} // namespace sick_lidar3d
#endif // __SICK_LIDAR3D_JSON_PARSER_H_INCLUDED
