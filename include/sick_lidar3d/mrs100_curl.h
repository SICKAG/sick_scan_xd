/*
 * @brief mrs100_curl implements a rest-api to post start and stop commands to the multiScan136.
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
#ifndef __SICK_LIDAR3D_MRS_CURL_H
#define __SICK_LIDAR3D_MRS_CURL_H

#include <string>

namespace sick_lidar3d
{
    class MRS100CURL
    {
    public:

        /*
         * @brief Posts the start command to multiScan136
         * @param[in] mrs100_ip ip address of a multiScan136, f.e. "127.0.0.1" (localhost, loopback) for an emulator or "192.168.0.1" for multiScan136
         * @param[in] mrs100_dst_ip UDP destination IP address ()
         * @param[in] udp_port ip port, f.e. 2115 (default port for multiScan136 emulator)
         */
        static bool postStart(const std::string& mrs100_ip = "192.168.0.1", const std::string& mrs100_dst_ip = "", int udp_port = 2115);

        /*
         * @brief Posts the stop command to multiScan136
         * @param[in] ip ip address of a multiScan136, f.e. "127.0.0.1" (localhost, loopback) for an emulator or "192.168.0.1" for multiScan136
         * @param[in] udp_port ip port, f.e. 2115 (default port for multiScan136 emulator)
         */
        static bool postStop(const std::string& ip = "192.168.0.1", int udp_port = 2115);

    };  // class MRS100CURL

}   // namespace sick_lidar3d
#endif // __SICK_LIDAR3D_MRS_CURL_H
