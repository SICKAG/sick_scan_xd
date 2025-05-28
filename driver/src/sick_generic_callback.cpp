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
#include <sick_scan/sick_generic_callback.h>

static sick_scan_xd::SickCallbackHandler<rosNodePtr,sick_scan_xd::PointCloud2withEcho>   s_cartesian_poincloud_callback_handler;
static sick_scan_xd::SickCallbackHandler<rosNodePtr,sick_scan_xd::PointCloud2withEcho>   s_polar_poincloud_callback_handler;
static sick_scan_xd::SickCallbackHandler<rosNodePtr,ros_sensor_msgs::Imu>                s_imu_callback_handler;
static sick_scan_xd::SickCallbackHandler<rosNodePtr,sick_scan_msg::LIDoutputstateMsg>    s_lidoutputstate_callback_handler;
static sick_scan_xd::SickCallbackHandler<rosNodePtr,sick_scan_msg::LFErecMsg>            s_lferec_callback_handler;
static sick_scan_xd::SickCallbackHandler<rosNodePtr,sick_scan_msg::SickLdmrsObjectArray> s_ldmrsobjectarray_callback_handler;
static sick_scan_xd::SickCallbackHandler<rosNodePtr,sick_scan_msg::RadarScan>            s_radarscan_callback_handler;
static sick_scan_xd::SickCallbackHandler<rosNodePtr,ros_visualization_msgs::MarkerArray> s_visualizationmarker_callback_handler;
static sick_scan_xd::SickCallbackHandler<rosNodePtr,sick_scan_xd::NAV350mNPOSData>       s_navposelandmark_callback_handler;

namespace sick_scan_xd
{
    void addCartesianPointcloudListener(rosNodePtr handle, PointCloud2Callback listener)
    {
        s_cartesian_poincloud_callback_handler.addListener(handle, listener);
	}

    void notifyCartesianPointcloudListener(rosNodePtr handle, const sick_scan_xd::PointCloud2withEcho* msg)
    {
        s_cartesian_poincloud_callback_handler.notifyListener(handle, msg);
	}

    void removeCartesianPointcloudListener(rosNodePtr handle, PointCloud2Callback listener)
    {
        s_cartesian_poincloud_callback_handler.removeListener(handle, listener);
	}

    bool isCartesianPointcloudListenerRegistered(rosNodePtr handle, PointCloud2Callback listener)
    {
        return s_cartesian_poincloud_callback_handler.isListenerRegistered(handle, listener);
	}

    void addPolarPointcloudListener(rosNodePtr handle, PointCloud2Callback listener)
    {
        s_polar_poincloud_callback_handler.addListener(handle, listener);
	}

    void notifyPolarPointcloudListener(rosNodePtr handle, const sick_scan_xd::PointCloud2withEcho* msg)
    {
        s_polar_poincloud_callback_handler.notifyListener(handle, msg);
	}

    void removePolarPointcloudListener(rosNodePtr handle, PointCloud2Callback listener)
    {
        s_polar_poincloud_callback_handler.removeListener(handle, listener);
	}

    bool isPolarPointcloudListenerRegistered(rosNodePtr handle, PointCloud2Callback listener)
    {
        return s_polar_poincloud_callback_handler.isListenerRegistered(handle, listener);
	}

    void addImuListener(rosNodePtr handle, ImuCallback listener)
    {
        s_imu_callback_handler.addListener(handle, listener);
	}

    void notifyImuListener(rosNodePtr handle, const ros_sensor_msgs::Imu* msg)
    {
        s_imu_callback_handler.notifyListener(handle, msg);
	}

    void removeImuListener(rosNodePtr handle, ImuCallback listener)
    {
        s_imu_callback_handler.removeListener(handle, listener);
	}

    bool isImuListenerRegistered(rosNodePtr handle, ImuCallback listener)
    {
        return s_imu_callback_handler.isListenerRegistered(handle, listener);
	}

    const std::string& getImuTopic()
    {
        return s_imu_callback_handler.getTopic();
    }
    
    void setImuTopic(const std::string& topic)
    {
        s_imu_callback_handler.setTopic(topic);
    }

    void addLIDoutputstateListener(rosNodePtr handle, LIDoutputstateCallback listener)
    {
        s_lidoutputstate_callback_handler.addListener(handle, listener);
	}
    
    void notifyLIDoutputstateListener(rosNodePtr handle, const sick_scan_msg::LIDoutputstateMsg* msg)
    {
        s_lidoutputstate_callback_handler.notifyListener(handle, msg);
	}
    
    void removeLIDoutputstateListener(rosNodePtr handle, LIDoutputstateCallback listener)
    {
        s_lidoutputstate_callback_handler.removeListener(handle, listener);
	}

    bool isLIDoutputstateListenerRegistered(rosNodePtr handle, LIDoutputstateCallback listener)
    {
        return s_lidoutputstate_callback_handler.isListenerRegistered(handle, listener);
	}

    const std::string& getLIDoutputstateTopic()
    {
        return s_lidoutputstate_callback_handler.getTopic();
    }
    
    void setLIDoutputstateTopic(const std::string& topic)
    {
        s_lidoutputstate_callback_handler.setTopic(topic);
    }

    void addLFErecListener(rosNodePtr handle, LFErecCallback listener)
    {
        s_lferec_callback_handler.addListener(handle, listener);
	}
    
    void notifyLFErecListener(rosNodePtr handle, const sick_scan_msg::LFErecMsg* msg)
    {
        s_lferec_callback_handler.notifyListener(handle, msg);
	}
    
    void removeLFErecListener(rosNodePtr handle, LFErecCallback listener)
    {
        s_lferec_callback_handler.removeListener(handle, listener);
	}
    
    bool isLFErecListenerRegistered(rosNodePtr handle, LFErecCallback listener)
    {
        return s_lferec_callback_handler.isListenerRegistered(handle, listener);
	}

    const std::string& getLFErecTopic()
    {
        return s_lferec_callback_handler.getTopic();
    }
    
    void setLFErecTopic(const std::string& topic)
    {
        s_lferec_callback_handler.setTopic(topic);
    }

    void addLdmrsObjectArrayListener(rosNodePtr handle, SickLdmrsObjectArrayCallback listener)
    {
        s_ldmrsobjectarray_callback_handler.addListener(handle, listener);
	}
    
    void notifyLdmrsObjectArrayListener(rosNodePtr handle, const sick_scan_msg::SickLdmrsObjectArray* msg)
    {
        s_ldmrsobjectarray_callback_handler.notifyListener(handle, msg);
	}
    
    void removeLdmrsObjectArrayListener(rosNodePtr handle, SickLdmrsObjectArrayCallback listener)
    {
        s_ldmrsobjectarray_callback_handler.removeListener(handle, listener);
	}

    bool isLdmrsObjectArrayListenerRegistered(rosNodePtr handle, SickLdmrsObjectArrayCallback listener)
    {
        return s_ldmrsobjectarray_callback_handler.isListenerRegistered(handle, listener);
	}

    void addRadarScanListener(rosNodePtr handle, RadarScanCallback listener)
    {
        s_radarscan_callback_handler.addListener(handle, listener);
	}
    
    void notifyRadarScanListener(rosNodePtr handle, const sick_scan_msg::RadarScan* msg)
    {
        s_radarscan_callback_handler.notifyListener(handle, msg);
	}
    
    void removeRadarScanListener(rosNodePtr handle, RadarScanCallback listener)
    {
        s_radarscan_callback_handler.removeListener(handle, listener);
	}

    bool isRadarScanListenerRegistered(rosNodePtr handle, RadarScanCallback listener)
    {
        return s_radarscan_callback_handler.isListenerRegistered(handle, listener);
	}

    const std::string& getRadarScanTopic()
    {
        return s_radarscan_callback_handler.getTopic();
    }
    
    void setRadarScanTopic(const std::string& topic)
    {
        s_radarscan_callback_handler.setTopic(topic);
    }

    void addVisualizationMarkerListener(rosNodePtr handle, VisualizationMarkerCallback listener)
    {
        s_visualizationmarker_callback_handler.addListener(handle, listener);
	}

    void notifyVisualizationMarkerListener(rosNodePtr handle, const ros_visualization_msgs::MarkerArray* msg)
    {
        s_visualizationmarker_callback_handler.notifyListener(handle, msg);
	}

    void removeVisualizationMarkerListener(rosNodePtr handle, VisualizationMarkerCallback listener)
    {
        s_visualizationmarker_callback_handler.removeListener(handle, listener);
	}

    bool isVisualizationMarkerListenerRegistered(rosNodePtr handle, VisualizationMarkerCallback listener)
    {
        return s_visualizationmarker_callback_handler.isListenerRegistered(handle, listener);
	}

    const std::string& getVisualizationMarkerTopic()
    {
        return s_visualizationmarker_callback_handler.getTopic();
    }

    void setVisualizationMarkerTopic(const std::string& topic)
    {
        s_visualizationmarker_callback_handler.setTopic(topic);
    }

    void addNavPoseLandmarkListener(rosNodePtr handle, NAV350mNPOSDataCallback listener)
    {
        s_navposelandmark_callback_handler.addListener(handle, listener);
	}

    void notifyNavPoseLandmarkListener(rosNodePtr handle, NAV350mNPOSData* msg)
    {
        s_navposelandmark_callback_handler.notifyListener(handle, msg);
	}

    void removeNavPoseLandmarkListener(rosNodePtr handle, NAV350mNPOSDataCallback listener)
    {
        s_navposelandmark_callback_handler.removeListener(handle, listener);
	}

    bool isNavPoseLandmarkListenerRegistered(rosNodePtr handle, NAV350mNPOSDataCallback listener)
    {
        return s_navposelandmark_callback_handler.isListenerRegistered(handle, listener);
	}

}   // namespace sick_scan_xd
