#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * @brief msgpack_validator validates received msgpacks against
 * the multiScan136 filter settings (FREchoFilter, LFPangleRangeFilter,
 * and LFPlayerFilter).
 *
 * If msgpacks with scan data out of filter settings (i.e. echo,
 * azimuth or segment not configured), an error message is printed
 * and the msgpack is discarded.
 *
 * By default, the msgpack_validator checks msgpacks against the full
 * range, i.e. all echos, azimuth_start=-PI, azimuth_end=+PI,
 * elevation_start=-PI/2, elevation_end=+PI/2 and all segments.
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
#ifndef __SICK_SCANSEGMENT_XD_MSGPACK_VALIDATOR_H
#define __SICK_SCANSEGMENT_XD_MSGPACK_VALIDATOR_H

#ifndef _USE_MATH_DEFINES // to ensure that M_PI is defined
#define _USE_MATH_DEFINES
#endif
#include <math.h>
#include <map>
#include <string>
#include <vector>

#include "sick_scan/sick_ros_wrapper.h"
#include "sick_scansegment_xd/common.h"

namespace sick_scansegment_xd
{
	/*
     * @brief class MsgPackValidatorData collects echo_idx, azimuth, elevation and segment_idx
     *        during msgpack parsing for msgpack validation.
     */
	class MsgPackValidatorData
	{
	public:

          /*
          * @brief Default constructor.
          */
          MsgPackValidatorData();

          /*
          * @brief Default destructor.
          */
          ~MsgPackValidatorData();

          /*
          * @brief Updates the azimuth histogram
          */
          void update(int echo_idx, int segment_idx, float azimuth, float elevation);

          /*
          * @brief Returns the resolution of azimuth histogram in degree (i.e. 1.0 degree)
          */
          float getAzimuthHistogramResolutionDeg(void) const;

          /*
          * @brief Returns the resolution of azimuth histogram in radians (i.e. (PI/180) * 1.0 degree)
          */
          float getAzimuthHistogramResolutionRad(void) const;

          /*
          * @brief Returns the resolution of elevation histogram in degree (i.e. 1.0 degree)
          */
          float getElevationHistogramResolutionDeg(void) const;

          /*
          * @brief Returns the resolution of elevation histogram in degree (i.e. (PI/180) * 1.0 degree)
          */
          float getElevationHistogramResolutionRad(void) const;

          /*
          * @brief Returns the azimuth histogram as human readable string
          */
          std::vector<std::string> print(void) const;

          // Histogram of azimuth angles: azimuth_count = AzimuthHistogram[azimuth_idx]
          typedef std::map<int, int> AzimuthHistogram; 

          // AzimuthHistogram per elevation: azimuth_count = AzimuthHistogramPerElevation[elevation_idx][azimuth_idx]
          typedef std::map<int, AzimuthHistogram> AzimuthHistogramPerElevation; 

          // AzimuthHistogramPerElevation per segment: azimuth_count = AzimuthHistogramPerElevationPerSegment[segment_idx][elevation_idx][azimuth_idx]
          typedef std::map<int, AzimuthHistogramPerElevation> AzimuthHistogramPerElevationPerSegment; 

          // AzimuthHistogramPerElevationPerSegment per per echo: azimuth_count = AzimuthHistogramPerElevationPerSegmentPerEcho[echo_idx][segment_idx][elevation_idx][azimuth_idx]
          typedef std::map<int, AzimuthHistogramPerElevationPerSegment> AzimuthHistogramPerElevationPerSegmentPerEcho; 

          // Returns the Histogram of azimuth angles: azimuth_count = getHistogram()[echo_idx][segment_idx][elevationToInt(elevation)][azimuthToInt(azimuth)]
          const AzimuthHistogramPerElevationPerSegmentPerEcho& getHistogram(void) const { return m_azimuth_histogram; }

          // Converts the azimuth index of the azimuth histogram to the azimuth angle in radians
          float azimuthIndexToRad(int azimuth_idx) const { return intToAzimuth(azimuth_idx); }

          // Converts the azimuth index of the azimuth histogram to the azimuth angle in degree
          float azimuthIndexToDeg(int azimuth_idx) const { return rad2deg(intToAzimuth(azimuth_idx)); }

          // Converts the elevation index of the histogram to the elevation angle in radians
          float elevationIndexToRad(int elevation_idx) const { return intToElevation(elevation_idx); }

          // Converts the elevation index of the histogram to the elevation angle in degree
          float elevationIndexToDeg(int elevation_idx) const { return rad2deg(intToElevation(elevation_idx)); }

          // Converts azimuth angle in rad to the azimuth index of the histogram
          int azimuthRadToIndex(float azimuth_rad) const { return azimuthToInt(azimuth_rad); }

          // Converts elevation angle in rad to the elevation index of the histogram
          int elevationRadToIndex(float elevation_rad) const { return elevationToInt(elevation_rad); }

   protected:

        #define AzimuthHistogramResolution (1.0f) // Histogram of azimuth angles in 1.0 degrees
        #define ElevationHistogramResolution (1.0f) // Histogram of elevation angles in 1.0 degrees

        float deg2rad(float angle) const { return angle * (float)(M_PI / 180.0); }
        float rad2deg(float angle) const { return angle * (float)(180.0 / M_PI); }

        /** Converts an azimuth angle in radians to an integer in 0.5 degree resolution */
        int azimuthToInt(float azimuth_rad) const { return (int)std::round(rad2deg(azimuth_rad) / AzimuthHistogramResolution); }
        float intToAzimuth(int azimuth_idx) const { return  deg2rad(azimuth_idx * AzimuthHistogramResolution); }

        /** Converts an elevation angle in radians to an integer in 0.5 degree resolution */
        int elevationToInt(float elevation_rad) const { return (int)std::round(rad2deg(elevation_rad) / ElevationHistogramResolution); }
        float intToElevation(int elevation_idx) const { return  deg2rad(elevation_idx * ElevationHistogramResolution); }

        /*
         * Member data
         */

        // Histogram of azimuth angles: azimuth_count = m_azimuth_histogram[echo_idx][segment_idx][elevationToInt(elevation)][azimuthToInt(azimuth)]
        AzimuthHistogramPerElevationPerSegmentPerEcho m_azimuth_histogram;

	};  // class MsgPackValidatorData

	/*
     * @brief class MsgPackValidator validates received msgpacks against
     * the multiScan136 filter settings (FREchoFilter, LFPangleRangeFilter,
     * and LFPlayerFilter).
     *
     * If msgpacks with scan data out of filter settings (i.e. echo,
     * azimuth or segment not configured), an error message is printed
     * and the msgpack is discarded.
     *
     * By default, the msgpack_validator checks msgpacks against the full
     * range, i.e. all echos, azimuth_start=-PI, azimuth_end=+PI,
     * elevation_start=-PI/2, elevation_end=+PI/2 and all segments.
     */
	class MsgPackValidator
	{
	public:

        /*
         * @brief Default constructor, initializes full range validation (all echos, azimuth_start=-PI, azimuth_end=+PI,
         *        elevation_start=-PI/2, elevation_end=+PI/2, 12 segments)
         *
         * @param[in] echos indizes of expected echos, default: all echos required
         * @param[in] azimuth_start start azimuth in radians, default: -PI
         * @param[in] azimuth_end end azimuth in radians, default: +PI
         * @param[in] elevation_start start elevation in radians, default: -PI/2
         * @param[in] elevation_end end elevation in radians, default: +PI/2
         * @param[in] segments indizes of expected segments, default: 12 segments
         * @param[in] layer_filter 0 (deactivated) or 1 (activated) for each layer, default: all 16 layers activated
         * @param[in] verbose 0: print error messages, 1: print error and informational messages, 2: print error and all messages
         */
         MsgPackValidator(const std::vector<int>& echos = { 0, 1, 2 }, 
             float azimuth_start = -M_PI, float azimuth_end = M_PI, 
             float elevation_start = -M_PI/2.0, float elevation_end = M_PI/2.0, 
             const std::vector<int>& segments = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 },
             const std::vector<int>& layer_filter = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
             int verbose = 0);

        /*
         * @brief Default destructor.
         */
        ~MsgPackValidator();

        /*
         * @brief Validates a received msgpack against the configured limits, i.e. checks that echo, segment and azimuth are
         *        within their configured range.
         *
         * @param[in] data_received echos, azimuth, elevation and segments collected during msgpack parsing
         *
         * @return true if validation passed successfully, false otherwise.
         */
        bool validateNotOutOfBound(const MsgPackValidatorData& data_received) const;

        /*
         * @brief Validates a received msgpack, i.e. checks that the required number of echos, segments and azimuth values have been received.
         *
         * @param[in] msgpack_data_collected echos, azimuth, elevation and segments collected during msgpack parsing
         *
         * @return true if validation passed successfully, false otherwise.
         */
        bool validateNoMissingScandata(const MsgPackValidatorData& msgpack_data_collected) const;

     protected:

          /** prints a vector as a flat list */
          template<typename T> std::string listVector(const std::vector<T> & vec) const
          {
               std::stringstream s;
               for(auto iter = vec.cbegin(); iter != vec.cend(); iter++)
                   s << (iter == vec.cbegin() ? "" : ", ") << (*iter);
               return s.str();
          }

          /** Returns the number of azimuth angles counted in the histogram for a given echo, segment, elevation and azimuth */
          int getAzimuthHistogramCount(const MsgPackValidatorData::AzimuthHistogramPerElevationPerSegmentPerEcho& azimuth_histogram, int echo_idx, int segment_idx, int elevation_idx, int azimuth_idx) const;

          void printMissingHistogramData(const std::vector<std::string>& messages) const;

          /*
          * Member data for msgpack validataion
          */
          std::vector<int> m_echos_required;    // indizes of expected echos, default: all echos required
          float m_azimuth_start;                // start azimuth in radians, default: -PI
          float m_azimuth_end;                  // azimuth in radians, default: +PI
          float m_elevation_start;              // start elevation in radians, default: -PI/2
          float m_elevation_end;                // end elevation in radians, default: +PI/2
          std::vector<int> m_valid_segments;    // indizes of valid segments, default: all segments, index 0 to 11
          std::vector<int> m_layer_filter;      // layer_filter, 0 (deactivated) or 1 (activated) for each layer, default: all 16 layers activated
          int m_verbose;                        // 0: print error messages, 1: print error and informational messages, 2: print error and all messages

	};  // class MsgPackValidator

}   // namespace sick_scansegment_xd
#endif // __SICK_SCANSEGMENT_XD_MSGPACK_VALIDATOR_H
