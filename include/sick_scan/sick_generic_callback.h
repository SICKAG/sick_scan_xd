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

#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

#include <sick_scan/sick_ros_wrapper.h>
#include <sick_scan/sick_nav_scandata.h>

// forward declaration of SickLdmrsObjectArray required for LdmrsObjectArray listener
#if __ROS_VERSION == 2 // ROS-2 (Linux or Windows)
#include <sick_scan_xd/msg/sick_ldmrs_object_array.hpp>
#else
#include <sick_scan_xd/SickLdmrsObjectArray.h>
#endif

namespace sick_scan_xd
{
    struct PointCloud2withEcho
    {
        PointCloud2withEcho() {}
        PointCloud2withEcho(const ros_sensor_msgs::PointCloud2* msg, int32_t _num_echos, int32_t _segment_idx, const std::string& _topic) 
            : num_echos(_num_echos), segment_idx(_segment_idx), topic(_topic)
        {
            pointcloud = ((msg) ? (*msg) : (ros_sensor_msgs::PointCloud2()));
        }
        ros_sensor_msgs::PointCloud2 pointcloud; // ROS PointCloud2
        int32_t num_echos = 0;                   // number of echos
        int32_t segment_idx = 0;                 // segment index (or -1 if pointcloud contains data from multiple segments)
        std::string topic;                       // ros topic this pointcloud is published
    };

    typedef void(* PointCloud2Callback)(rosNodePtr handle, const PointCloud2withEcho* msg);
    typedef void(* ImuCallback)(rosNodePtr handle, const ros_sensor_msgs::Imu* msg);
    typedef void(* LIDoutputstateCallback)(rosNodePtr handle, const sick_scan_msg::LIDoutputstateMsg* msg);
    typedef void(* LFErecCallback)(rosNodePtr handle, const sick_scan_msg::LFErecMsg* msg);
    typedef void(* SickLdmrsObjectArrayCallback)(rosNodePtr handle, const sick_scan_msg::SickLdmrsObjectArray* msg);
    typedef void(* RadarScanCallback)(rosNodePtr handle, const sick_scan_msg::RadarScan* msg);
    typedef void(* VisualizationMarkerCallback)(rosNodePtr handle, const ros_visualization_msgs::MarkerArray* msg);
    typedef void(* NAV350mNPOSDataCallback)(rosNodePtr handle, const NAV350mNPOSData* msg);

    void addCartesianPointcloudListener(rosNodePtr handle, PointCloud2Callback listener);
    void notifyCartesianPointcloudListener(rosNodePtr handle, const PointCloud2withEcho* msg);
    void removeCartesianPointcloudListener(rosNodePtr handle, PointCloud2Callback listener);
    bool isCartesianPointcloudListenerRegistered(rosNodePtr handle, PointCloud2Callback listener);

    void addPolarPointcloudListener(rosNodePtr handle, PointCloud2Callback listener);
    void notifyPolarPointcloudListener(rosNodePtr handle, const PointCloud2withEcho* msg);
    void removePolarPointcloudListener(rosNodePtr handle, PointCloud2Callback listener);
    bool isPolarPointcloudListenerRegistered(rosNodePtr handle, PointCloud2Callback listener);

    void addImuListener(rosNodePtr handle, ImuCallback listener);
    void notifyImuListener(rosNodePtr handle, const ros_sensor_msgs::Imu* msg);
    void removeImuListener(rosNodePtr handle, ImuCallback listener);
    bool isImuListenerRegistered(rosNodePtr handle, ImuCallback listener);
    const std::string& getImuTopic();
    void setImuTopic(const std::string& topic);

    void addLIDoutputstateListener(rosNodePtr handle, LIDoutputstateCallback listener);
    void notifyLIDoutputstateListener(rosNodePtr handle, const sick_scan_msg::LIDoutputstateMsg* msg);
    void removeLIDoutputstateListener(rosNodePtr handle, LIDoutputstateCallback listener);
    bool isLIDoutputstateListenerRegistered(rosNodePtr handle, LIDoutputstateCallback listener);
    const std::string& getLIDoutputstateTopic();
    void setLIDoutputstateTopic(const std::string& topic);

    void addLFErecListener(rosNodePtr handle, LFErecCallback listener);
    void notifyLFErecListener(rosNodePtr handle, const sick_scan_msg::LFErecMsg* msg);
    void removeLFErecListener(rosNodePtr handle, LFErecCallback listener);
    bool isLFErecListenerRegistered(rosNodePtr handle, LFErecCallback listener);
    const std::string& getLFErecTopic();
    void setLFErecTopic(const std::string& topic);
    
    void addLdmrsObjectArrayListener(rosNodePtr handle, SickLdmrsObjectArrayCallback listener);
    void notifyLdmrsObjectArrayListener(rosNodePtr handle, const sick_scan_msg::SickLdmrsObjectArray* msg);
    void removeLdmrsObjectArrayListener(rosNodePtr handle, SickLdmrsObjectArrayCallback listener);
    bool isLdmrsObjectArrayListenerRegistered(rosNodePtr handle, SickLdmrsObjectArrayCallback listener);

    void addRadarScanListener(rosNodePtr handle, RadarScanCallback listener);
    void notifyRadarScanListener(rosNodePtr handle, const sick_scan_msg::RadarScan* msg);
    void removeRadarScanListener(rosNodePtr handle, RadarScanCallback listener);
    bool isRadarScanListenerRegistered(rosNodePtr handle, RadarScanCallback listener);
    const std::string& getRadarScanTopic();
    void setRadarScanTopic(const std::string& topic);

    void addVisualizationMarkerListener(rosNodePtr handle, VisualizationMarkerCallback listener);
    void notifyVisualizationMarkerListener(rosNodePtr handle, const ros_visualization_msgs::MarkerArray* msg);
    void removeVisualizationMarkerListener(rosNodePtr handle, VisualizationMarkerCallback listener);
    bool isVisualizationMarkerListenerRegistered(rosNodePtr handle, VisualizationMarkerCallback listener);
    const std::string& getVisualizationMarkerTopic();
    void setVisualizationMarkerTopic(const std::string& topic);

    void addNavPoseLandmarkListener(rosNodePtr handle, NAV350mNPOSDataCallback listener);
    void notifyNavPoseLandmarkListener(rosNodePtr handle, NAV350mNPOSData* navdata);
    void removeNavPoseLandmarkListener(rosNodePtr handle, NAV350mNPOSDataCallback listener);
    bool isNavPoseLandmarkListenerRegistered(rosNodePtr handle, NAV350mNPOSDataCallback listener);

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
                std::unique_lock<std::mutex> lock(m_listeners_mutex);
                m_listeners[handle].push_back(listener);
            }
        }

        void notifyListener(HandleType handle, const MsgType* msg)
        {
            std::list<callbackFunctionPtr> listeners = getListener(handle);
            for(typename std::list<callbackFunctionPtr>::iterator iter_listener = listeners.begin(); iter_listener != listeners.end(); iter_listener++)
            {
                if (*iter_listener)
                {
                    (*iter_listener)(handle, msg);
                }
            }
        }

        void notifyListener(const MsgType* msg)
        {
            std::vector<HandleType> handle_list;
            {
                std::unique_lock<std::mutex> lock(m_listeners_mutex);
                for(typename std::map<HandleType, std::list<callbackFunctionPtr>>::iterator iter_listeners = m_listeners.begin(); iter_listeners != m_listeners.end(); iter_listeners++)
                    handle_list.push_back(iter_listeners->first);
            }
            for(int n = 0; n < handle_list.size(); n++)
            {
                notifyListener(handle_list[n], msg);
            }
        }

        void removeListener(HandleType handle, callbackFunctionPtr listener)
        {
            std::unique_lock<std::mutex> lock(m_listeners_mutex);
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

        bool isListenerRegistered(HandleType handle, callbackFunctionPtr listener)
        {
            if (listener)
            {
                std::unique_lock<std::mutex> lock(m_listeners_mutex);
                std::list<callbackFunctionPtr> & listeners = m_listeners[handle];
                for(typename std::list<callbackFunctionPtr>::iterator iter_listener = listeners.begin(); iter_listener != listeners.end(); iter_listener++)
                {
                    if (*iter_listener == listener)
                        return true;
                }
            }
            return false;
        }

        const std::string& getTopic()
        {
            return m_topic;
        }
        
        void setTopic(const std::string& topic)
        {
            m_topic = topic;
        }

        void clear()
        {
            std::unique_lock<std::mutex> lock(m_listeners_mutex);
            m_listeners.clear();
        }

    protected:

        std::list<callbackFunctionPtr> getListener(HandleType handle)
        {
            std::unique_lock<std::mutex> lock(m_listeners_mutex);
            return m_listeners[handle];
        }

        std::map<HandleType, std::list<callbackFunctionPtr>> m_listeners; // list of listeners
        std::mutex m_listeners_mutex; // mutex to protect access to m_listeners
        std::string m_topic; // ros topic the message is published

    };  // class SickCallbackHandler

    /*
    *  Utility template to wait for a message
    */
    template<typename HandleType, class MsgType> class SickWaitForMessageHandler
    {
    public:

        typedef SickWaitForMessageHandler<HandleType, MsgType>* SickWaitForMessageHandlerPtr;

        bool waitForNextMessage(MsgType& msg, double timeout_sec)
        {
            uint64_t timeout_microsec = std::max<uint64_t>((uint64_t)(1), (uint64_t)(timeout_sec * 1.0e6));
            std::chrono::system_clock::time_point wait_end_time = std::chrono::system_clock::now() + std::chrono::microseconds(timeout_microsec);
            std::unique_lock<std::mutex> lock(m_message_mutex);
            m_message_valid = false;
            while(m_running && rosOk() && !m_message_valid)
            {
                if (m_message_cond.wait_until(lock, wait_end_time) == std::cv_status::timeout || std::chrono::system_clock::now() >= wait_end_time)
                    break;
            }
            if (m_message_valid)
            {
                msg = m_message;
            }
            return m_message_valid;
        }

        static void messageCallback(HandleType node, const MsgType* msg)
        {
            if (msg)
            {
                std::unique_lock<std::mutex> lock(s_wait_for_message_handler_mutex);
                for(typename std::list<SickWaitForMessageHandlerPtr>::iterator iter_handler = s_wait_for_message_handler_list.begin(); iter_handler != s_wait_for_message_handler_list.end(); iter_handler++)
                {
                    if (*iter_handler)
                        (*iter_handler)->message_callback(node, msg);
                }
            }
        }

        static void addWaitForMessageHandlerHandler(SickWaitForMessageHandlerPtr handler)
        {
            std::unique_lock<std::mutex> lock(s_wait_for_message_handler_mutex);
            s_wait_for_message_handler_list.push_back(handler);
        }

        static void removeWaitForMessageHandlerHandler(SickWaitForMessageHandlerPtr handler)
        {
            std::unique_lock<std::mutex> lock(s_wait_for_message_handler_mutex);
            for(typename std::list<SickWaitForMessageHandlerPtr>::iterator iter_handler = s_wait_for_message_handler_list.begin(); iter_handler != s_wait_for_message_handler_list.end(); )
            {
                if (*iter_handler == handler)
                    iter_handler = s_wait_for_message_handler_list.erase(iter_handler);
                else
                    iter_handler++;
            }
        }

        static void shutdown()
        {
            std::unique_lock<std::mutex> lock(s_wait_for_message_handler_mutex);
            for (typename std::list<SickWaitForMessageHandlerPtr>::iterator iter_handler = s_wait_for_message_handler_list.begin(); iter_handler != s_wait_for_message_handler_list.end(); iter_handler++)
            {
                if ((*iter_handler))
                    (*iter_handler)->signal_shutdown();
            }
        }

    protected:

        void signal_shutdown(void)
        {
          std::unique_lock<std::mutex> lock(m_message_mutex);
          m_running = false;
          m_message_cond.notify_all();
        }

        void message_callback(HandleType node, const MsgType* msg)
        {
            if (msg)
            {
                ROS_INFO_STREAM("SickScanApiWaitEventHandler::message_callback(): message recceived");
                std::unique_lock<std::mutex> lock(m_message_mutex);
                if (m_running && rosOk())
                {
                    m_message = *msg;
                    m_message_valid = true;
                }
                m_message_cond.notify_all();
            }
        }

        bool m_running = true;                    // set false to signal shutdown
        bool m_message_valid = false;             // becomes true after message has been received
        MsgType m_message;                        // the received message
        std::mutex m_message_mutex;               // mutex to protect access to m_message
        std::condition_variable m_message_cond;   // condition to wait for resp. notify when a message is received

        static std::list<SickWaitForMessageHandler<HandleType, MsgType>*> s_wait_for_message_handler_list; // list of all instances of SickWaitForMessageHandler
        static std::mutex s_wait_for_message_handler_mutex; // mutex to protect access to s_wait_for_message_handler_list
    };  // class SickWaitForMessageHandler

    typedef SickWaitForMessageHandler<rosNodePtr, sick_scan_xd::PointCloud2withEcho>   WaitForCartesianPointCloudMessageHandler;
    typedef SickWaitForMessageHandler<rosNodePtr, sick_scan_xd::PointCloud2withEcho>   WaitForPolarPointCloudMessageHandler;
    typedef SickWaitForMessageHandler<rosNodePtr, ros_sensor_msgs::Imu>                WaitForImuMessageHandler;
    typedef SickWaitForMessageHandler<rosNodePtr, sick_scan_msg::LFErecMsg>            WaitForLFErecMessageHandler;
    typedef SickWaitForMessageHandler<rosNodePtr, sick_scan_msg::LIDoutputstateMsg>    WaitForLIDoutputstateMessageHandler;
    typedef SickWaitForMessageHandler<rosNodePtr, sick_scan_msg::RadarScan>            WaitForRadarScanMessageHandler;
    typedef SickWaitForMessageHandler<rosNodePtr, sick_scan_msg::SickLdmrsObjectArray> WaitForLdmrsObjectArrayMessageHandler;
    typedef SickWaitForMessageHandler<rosNodePtr, ros_visualization_msgs::MarkerArray> WaitForVisualizationMarkerMessageHandler;
    typedef SickWaitForMessageHandler<rosNodePtr, sick_scan_xd::NAV350mNPOSData>       WaitForNAVPOSDataMessageHandler;

}   // namespace sick_scan_xd
#endif // __SICK_GENERIC_CALLBACK_H_INCLUDED
