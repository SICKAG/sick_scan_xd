/*
 * @brief sim_loc_utils contains a collection of utility functions for SIM Localization.
 *
 * Copyright (C) 2019 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2019 SICK AG, Waldkirch
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
 *  Copyright 2019 SICK AG
 *  Copyright 2019 Ing.-Buero Dr. Michael Lehning
 *
 */
#ifndef __SIM_LOC_UTILS_H_INCLUDED
#define __SIM_LOC_UTILS_H_INCLUDED

#include <math.h>
#include <string>
#include <sstream>
#include <vector>
#include <boost/thread.hpp>

#include "sick_scan/ros_wrapper.h"

namespace sick_scan
{
  /*!
   * class SetGet implements setter and getter template functions for thread-protected set and get of a value.
   */
  template<typename ElementType, typename MutexType = boost::mutex> class SetGet
  {
  public:
  
    /*! Constructor with optional initialization of the value */
    SetGet(const ElementType & value = ElementType()) : m_value(value) {}
  
    /*! Sets the value threadsafe */
    void set(const ElementType & value)
    {
      boost::lock_guard<MutexType> value_lockguard(m_value_mutex);
      m_value = value;
    }
  
    /*! Returns the value threadsafe */
    ElementType get(void)
    {
      boost::lock_guard<MutexType> value_lockguard(m_value_mutex);
      return m_value;
    }

  protected:
  
    /** member variables */
    ElementType m_value; ///< protected value
    MutexType m_value_mutex; ///< mutex to protect value
  };
  
  /*!
   * Shortcut for SetGet<uint32_t>
   */
  class SetGet32 : public SetGet<uint32_t>
  {
  public:
    /*! Constructor with optional initialization of  value */
    SetGet32(const uint32_t & value = 0) : SetGet<uint32_t,boost::mutex>(value) {}
    /*! Increments and returns the value */
    uint32_t inc(void)
    {
      boost::lock_guard<boost::mutex> value_lockguard(m_value_mutex);
      return ++m_value;
    }
  };
  
  /*!
   * class Utils implements utility functions for SIM Localization.
   */
  class Utils
  {
  public:

    /*!
     * Converts and returns binary data to hex string
     * @param[in] binary_data binary input data
     * @return hex string
     */
    static std::string toHexString(const std::vector<uint8_t> & binary_data);

    /*!
     * Converts and returns binary data to ascii string with non-ascii data represented as "\x<hexvalue>"
     * @param[in] binary_data binary input data
     * @return hex string
     */
    static std::string toAsciiString(const uint8_t* binary_data, int length);
  
    /*!
     * Shortcut to replace linefeeds by colon-separators
     */
    static void flattenString(std::string & s);
    
    /*!
     * Shortcut to print a type in flatten format, replacing linefeeds by colon-separators
     */
    template <typename T> static std::string flattenToString(const T & x)
    {
      std::stringstream s;
      s << x;
      std::string out(s.str());
      flattenString(out);
      return out;
    }
  
    /** Shortcut to print (msg_header.stamp.sec,msg_header.stamp.nsec) on ROS1 resp. (msg_header.stamp.sec,msg_header.stamp.nanosec) on ROS2 */
    static std::string flattenToString(const std_msgs::Header* header)
    {
      std::stringstream s;
      s << "header.stamp: " << header->stamp.sec << "." << header->stamp.NSEC;
      return s.str();
    }
    
    /*!
     * Shortcut to print a SickLocColaTelegramMsg in flatten format, replacing linefeeds by colon-separators
     */
    static std::string flattenToString(const sick_scan::SickLocColaTelegramMsg & cola_telegram)
    {
      std::stringstream s;
      s << flattenToString(&cola_telegram.header)
        << ", command_type: " << cola_telegram.command_type
        << ", command_name: " << cola_telegram.command_name << ", parameter: [";
      for(size_t n = 0; n < cola_telegram.parameter.size(); n++)
        s << (n>0?",":"") << cola_telegram.parameter[n];
      s << "]";
      std::string out(s.str());
      flattenString(out);
      return out;
    }

    /*!
     * Shortcut to print a SickLocColaTelegramMsg in flatten format, replacing linefeeds by colon-separators
     */
    static std::string flattenToString(const sick_scan::SickLocResultPortTelegramMsg & telegram)
    {
      
      std::stringstream s;
      s << flattenToString(&telegram.header)
        << ", telegram_payload: [posex:" << telegram.telegram_payload.posex << ",posey:" << telegram.telegram_payload.posey << ",poseyaw:" << telegram.telegram_payload.poseyaw
        << ",scancounter:" << telegram.telegram_payload.scancounter << ",timestamp:" << telegram.telegram_payload.timestamp << ",quality:" << (int)(telegram.telegram_payload.quality&0xFF)
        << ",covariancex:" << telegram.telegram_payload.covariancex << ",covariancey:" << telegram.telegram_payload.covariancey << ",covarianceyaw:" << telegram.telegram_payload.covarianceyaw
        << ",outliersratio:" << telegram.telegram_payload.outliersratio << ",errorcode:" << telegram.telegram_payload.errorcode
        << "], telegram_checksum: " << telegram.telegram_trailer.checksum;
      std::string out(s.str());
      flattenString(out);
      return out;
    }

    /*!
     * Compares two objects by streaming them to strings.
     * Returns true, if string representation of x and y is identical,
     * otherwise false.
     */
    template <typename T> static bool identicalByStream(const T & x, const T & y)
    {
      std::stringstream sx, sy;
      sx << x;
      sy << y;
      return sx.str() == sy.str();
    }
  
    /*!
     * Compares two vectors by streaming them to strings.
     * Returns true, if string representation of x and y is identical,
     * otherwise false.
     */
    template <typename T> static bool identicalByStream(const std::vector<T> & x, const std::vector<T> & y)
    {
      if(x.size() == y.size())
      {
        for(size_t n = 0; n < x.size(); n++)
          if(!identicalByStream(x[n], y[n]))
            return false;
        return true;
      }
      return false;
    }

    /*!
     * Compares two SickLocResultPortTelegramMsgs by streaming them to strings.
     * Returns true, if string representation of x and y is identical,
     * otherwise false. The message headers with timestamp and frame_id are ignored.
     */
    static bool identicalByStream(const SickLocResultPortTelegramMsg & x, const SickLocResultPortTelegramMsg & y)
    {
      #if defined __ROS_VERSION && __ROS_VERSION == 1
      return identicalByStream(x.telegram_header, y.telegram_header)
        && identicalByStream(x.telegram_payload, y.telegram_payload)
        && identicalByStream(x.telegram_trailer, y.telegram_trailer);
      #elif defined __ROS_VERSION && __ROS_VERSION == 2
      return x.telegram_header == y.telegram_header && x.telegram_payload == y.telegram_payload && x.telegram_trailer == y.telegram_trailer;
      //return x == y;
      #else
      return false;
      #endif
    }

    /*!
     * Compares two SickLocColaTelegramMsg by streaming them to strings.
     * Returns true, if string representation of x and y is identical,
     * otherwise false. The message headers with timestamp and frame_id are ignored.
     */
    static bool identicalByStream(const SickLocColaTelegramMsg & x, const SickLocColaTelegramMsg & y)
    {
      return x.command_name == y.command_name && x.command_type == y.command_type && identicalByStream(x.parameter, y.parameter);
    }
  
    /*!
     * Returns the normalized angle, i.e. the angle in the range -PI to +PI.
     * @param[in] angle angle in radians
     * @return normalized angle in radians
     */
    static double normalizeAngle(double angle)
    {
      while(angle > M_PI)
        angle -= (2.0 * M_PI);
      while(angle < -M_PI)
        angle += (2.0 * M_PI);
      return angle;
    }
  
  }; // class Utils
  
} // namespace sick_scan
#endif // __SIM_LOC_UTILS_H_INCLUDED
