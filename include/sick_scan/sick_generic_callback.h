#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
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

// forward declaration of SickLdmrsObjectArray required for LdmrsObjectArray listener
#if __ROS_VERSION == 2 // ROS-2 (Linux or Windows)
#include <sick_scan/msg/sick_ldmrs_object_array.hpp>
#else
#include <sick_scan/SickLdmrsObjectArray.h>
#endif

namespace sick_scan
{
    struct PointCloud2withEcho
    {
        PointCloud2withEcho(const ros_sensor_msgs::PointCloud2* msg = 0, int32_t _num_echos = 0, int32_t _segment_idx = 0) : pointcloud(*msg), num_echos(_num_echos), segment_idx(_segment_idx) {}
        ros_sensor_msgs::PointCloud2 pointcloud; // ROS PointCloud2
        int32_t num_echos;                       // number of echos
        int32_t segment_idx;                     // segment index (or -1 if pointcloud contains data from multiple segments)
    };

    typedef void(* PointCloud2Callback)(rosNodePtr handle, const PointCloud2withEcho* msg);
    typedef void(* ImuCallback)(rosNodePtr handle, const ros_sensor_msgs::Imu* msg);
    typedef void(* LIDoutputstateCallback)(rosNodePtr handle, const sick_scan_msg::LIDoutputstateMsg* msg);
    typedef void(* LFErecCallback)(rosNodePtr handle, const sick_scan_msg::LFErecMsg* msg);
    typedef void(* SickLdmrsObjectArrayCallback)(rosNodePtr handle, const sick_scan_msg::SickLdmrsObjectArray* msg);
    typedef void(* RadarScanCallback)(rosNodePtr handle, const sick_scan_msg::RadarScan* msg);
    typedef void(* VisualizationMarkerCallback)(rosNodePtr handle, const ros_visualization_msgs::MarkerArray* msg);

    void addCartesianPointcloudListener(rosNodePtr handle, PointCloud2Callback listener);
    void notifyCartesianPointcloudListener(rosNodePtr handle, const PointCloud2withEcho* msg);
    void removeCartesianPointcloudListener(rosNodePtr handle, PointCloud2Callback listener);

    void addPolarPointcloudListener(rosNodePtr handle, PointCloud2Callback listener);
    void notifyPolarPointcloudListener(rosNodePtr handle, const PointCloud2withEcho* msg);
    void removePolarPointcloudListener(rosNodePtr handle, PointCloud2Callback listener);

    void addImuListener(rosNodePtr handle, ImuCallback listener);
    void notifyImuListener(rosNodePtr handle, const ros_sensor_msgs::Imu* msg);
    void removeImuListener(rosNodePtr handle, ImuCallback listener);

    void addLIDoutputstateListener(rosNodePtr handle, LIDoutputstateCallback listener);
    void notifyLIDoutputstateListener(rosNodePtr handle, const sick_scan_msg::LIDoutputstateMsg* msg);
    void removeLIDoutputstateListener(rosNodePtr handle, LIDoutputstateCallback listener);

    void addLFErecListener(rosNodePtr handle, LFErecCallback listener);
    void notifyLFErecListener(rosNodePtr handle, const sick_scan_msg::LFErecMsg* msg);
    void removeLFErecListener(rosNodePtr handle, LFErecCallback listener);
    
    void addLdmrsObjectArrayListener(rosNodePtr handle, SickLdmrsObjectArrayCallback listener);
    void notifyLdmrsObjectArrayListener(rosNodePtr handle, const sick_scan_msg::SickLdmrsObjectArray* msg);
    void removeLdmrsObjectArrayListener(rosNodePtr handle, SickLdmrsObjectArrayCallback listener);

    void addRadarScanListener(rosNodePtr handle, RadarScanCallback listener);
    void notifyRadarScanListener(rosNodePtr handle, const sick_scan_msg::RadarScan* msg);
    void removeRadarScanListener(rosNodePtr handle, RadarScanCallback listener);

    void addVisualizationMarkerListener(rosNodePtr handle, VisualizationMarkerCallback listener);
    void notifyVisualizationMarkerListener(rosNodePtr handle, const ros_visualization_msgs::MarkerArray* msg);
    void removeVisualizationMarkerListener(rosNodePtr handle, VisualizationMarkerCallback listener);

    /*
    *  Callback template for registration and deregistration of callbacks incl. notification of listeners
    */
    template<typename HandleType, class MsgType> class SickCallbackHandler
    {
    public:
        
        typedef void(* callbackFunctionPtr)(HandleType handle, const MsgType* msg);

        void addListener(HandleType handle, callbackFunctionPtr listener)
        {
            if (listener)
            {
                m_listeners[handle].push_back(listener);
            }
        }

        void notifyListener(HandleType handle, const MsgType* msg)
        {
            std::list<callbackFunctionPtr> & listeners = m_listeners[handle];
            for(typename std::list<callbackFunctionPtr>::iterator iter_listener = listeners.begin(); iter_listener != listeners.end(); iter_listener++)
            {
                if (*iter_listener)
                {
                    (*iter_listener)(handle, msg);
                }
            }
        }

        void removeListener(HandleType handle, callbackFunctionPtr listener)
        {
            std::list<callbackFunctionPtr> & listeners = m_listeners[handle];
            for(typename std::list<callbackFunctionPtr>::iterator iter_listener = listeners.begin(); iter_listener != listeners.end(); )
            {
                if (*iter_listener == listener)
                {
                    iter_listener = listeners.erase(iter_listener);
                }
                else
                {
                    iter_listener++;
                }
            }
        }

        void clear()
        {
            m_listeners.clear();
        }

    protected:

        std::map<HandleType, std::list<callbackFunctionPtr>> m_listeners;

    };  // class SickCallbackHandler

}   // namespace sick_scan
#endif // __SICK_GENERIC_CALLBACK_H_INCLUDED
