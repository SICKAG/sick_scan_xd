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
#ifndef __SICK_GENERIC_CALLBACK_H_INCLUDED
#define __SICK_GENERIC_CALLBACK_H_INCLUDED

#include <string>
#include <vector>

#include <sick_scan/sick_ros_wrapper.h>

namespace sick_scan
{
    typedef void* Handle;

    typedef void(* PointCloud2Callback)(Handle handle, const ros_sensor_msgs::PointCloud2* msg);
    void addPointcloudListener(Handle handle, PointCloud2Callback listener);
    void notifyPointcloudListener(Handle handle, const sensor_msgs::msg::PointCloud2* msg);

    /*
    *  Callback template for registration and deregistration of callbacks incl. notification of listeners
    */
    template<class MsgType> class SickCallbackHandler
    {
    public:
        
        typedef void(* callbackFunctionPtr)(Handle handle, const MsgType* msg);

        void addListener(Handle handle, callbackFunctionPtr listener)
        {
            m_listeners[handle].push_back(listener);
        }

        void notifyListener(Handle handle, const MsgType* msg)
        {
            std::vector<callbackFunctionPtr> & listeners = m_listeners[handle];
            for(int n = 0; n < listeners.size(); n++)
            {
                if (listeners[n])
                {
                    listeners[n](handle, msg);
                }
            }
        }

    protected:

        std::map<Handle, std::vector<callbackFunctionPtr>> m_listeners;

    };  // class SickCallbackHandler

}   // namespace sick_scan
#endif // __SICK_GENERIC_CALLBACK_H_INCLUDED
