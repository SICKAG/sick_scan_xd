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
 *
 */
#include <float.h>

#include "sick_scansegment_xd/msgpack_validator.h"

/*
 * @brief Default constructor. Class MsgPackValidatorData collects echo_idx, azimuth, elevation and segment_idx
 *        during msgpack parsing for msgpack validation.
 */
sick_scansegment_xd::MsgPackValidatorData::MsgPackValidatorData()
{
}

/*
 * @brief Default destructor.
 */
sick_scansegment_xd::MsgPackValidatorData::~MsgPackValidatorData()
{
}

/*
 * @brief Updates the azimuth histogram
 */
void sick_scansegment_xd::MsgPackValidatorData::update(int echo_idx, int segment_idx, float azimuth, float elevation) 
{ 
    m_azimuth_histogram[echo_idx][segment_idx][elevationToInt(elevation)][azimuthToInt(azimuth)] += 1;
}

/*
 * @brief Returns the resolution of azimuth histogram in degree (i.e. 1.0 degree)
 */
float sick_scansegment_xd::MsgPackValidatorData::getAzimuthHistogramResolutionDeg(void) const
{ 
    return AzimuthHistogramResolution;
}

/*
 * @brief Returns the resolution of azimuth histogram in radians (i.e. (PI/180) * 1.0 degree)
 */
float sick_scansegment_xd::MsgPackValidatorData::getAzimuthHistogramResolutionRad(void) const
{ 
    return deg2rad(AzimuthHistogramResolution);
}

/*
 * @brief Returns the resolution of elevation histogram in degree (i.e. 1.0 degree)
 */
float sick_scansegment_xd::MsgPackValidatorData::getElevationHistogramResolutionDeg(void) const
{ 
    return ElevationHistogramResolution;
}

/*
 * @brief Returns the resolution of elevation histogram in degree (i.e. (PI/180) * 1.0 degree)
 */
float sick_scansegment_xd::MsgPackValidatorData::getElevationHistogramResolutionRad(void) const
{ 
    return deg2rad(ElevationHistogramResolution);
}

/*
 * @brief Returns the current data values as human readable string
 */
std::vector<std::string> sick_scansegment_xd::MsgPackValidatorData::print(void) const
{
    std::vector<std::string> messages;
    // Loop over Histogram of azimuth angles: azimuth_count = m_azimuth_histogram[echo_idx][segment_idx][elevationToInt(elevation)][azimuthToInt(azimuth)]
    for(AzimuthHistogramPerElevationPerSegmentPerEcho::const_iterator iter_echos = m_azimuth_histogram.cbegin(); iter_echos != m_azimuth_histogram.cend(); iter_echos++)
    {
        int echo_idx = iter_echos->first;
        const AzimuthHistogramPerElevationPerSegment& azimuth_histogram_elevation_segment = iter_echos->second;
        for(AzimuthHistogramPerElevationPerSegment::const_iterator iter_segment = azimuth_histogram_elevation_segment.cbegin(); iter_segment != azimuth_histogram_elevation_segment.cend(); iter_segment++)
        {
            int segment_idx = iter_segment->first;
            const AzimuthHistogramPerElevation& azimuth_histogram_elevation = iter_segment->second;
            for(AzimuthHistogramPerElevation::const_iterator iter_elevation = azimuth_histogram_elevation.cbegin(); iter_elevation != azimuth_histogram_elevation.cend(); iter_elevation++)
            {
                float elevation_rad = intToElevation(iter_elevation->first);
                const AzimuthHistogram& azimuth_histogram = iter_elevation->second;
                float azimuth_min = azimuth_histogram.empty() ? 0 : FLT_MAX;
                float azimuth_max = azimuth_histogram.empty() ? 0 : -FLT_MAX;
                for(AzimuthHistogram::const_iterator iter_azimuth = azimuth_histogram.cbegin(); iter_azimuth != azimuth_histogram.cend(); iter_azimuth++)
                {
                    float azimuth_rad = intToAzimuth(iter_azimuth->first);
                    int azimuth_cnt = iter_azimuth->second;
                    if(azimuth_cnt > 0)
                    {
                        azimuth_min = std::min(azimuth_min, azimuth_rad);
                        azimuth_max = std::max(azimuth_max, azimuth_rad);
                    }
                }
                std::stringstream s;
                s << "MsgPackValidatorData[echo=" << echo_idx << "][segment=" << segment_idx << "][elevation=" << rad2deg(elevation_rad) << "]: azimuth=[" << rad2deg(azimuth_min) << ", " << rad2deg(azimuth_max) << "] [deg]" ;
                messages.push_back(s.str());
            }
        }
    }
    return messages;
}

/*
 * @brief Default constructor, initializes full range validation
 *        (all echos, azimuth_start=-PI, azimuth_end=+PI,
 *        elevation_start=-PI/2, elevation_end=+PI/2, all segments)
 *
 *        class MsgPackValidator validates received msgpacks against
 *        the multiScan136 filter settings (FREchoFilter, LFPangleRangeFilter,
 *        and LFPlayerFilter).
 *
 *        If msgpacks with scan data out of filter settings (i.e. echo,
 *        azimuth or segment not configured), an error message is printed
 *        and the msgpack is discarded.
 *
 *        By default, the msgpack_validator checks msgpacks against the full
 *        range, i.e. all echos, azimuth_start=-PI, azimuth_end=+PI,
 *        elevation_start=-PI/2, elevation_end=+PI/2 and all segments.
 *
 * @param[in] echos indizes of expected echos, default: all echos required
 * @param[in] azimuth_start start azimuth in radians, default: -PI
 * @param[in] azimuth_end end azimuth in radians, default: +PI
 * @param[in] elevation_start start elevation in radians, default: -PI/2
 * @param[in] elevation_end end elevation in radians, default: +PI/2
 * @param[in] segments indizes of expected segments, default: all segments
 * @param[in] layer_filter 0 (deactivated) or 1 (activated) for each layer, default: all 16 layers activated
 * @param[in] verbose 0: print error messages, 1: print error and informational messages, 2: print error and all messages
 */
sick_scansegment_xd::MsgPackValidator::MsgPackValidator(const std::vector<int>& echos, float azimuth_start, float azimuth_end, float elevation_start, float elevation_end, 
    const std::vector<int>& segments, const std::vector<int>& layer_filter, int verbose)
: m_echos_required(echos), m_azimuth_start(azimuth_start), m_azimuth_end(azimuth_end), m_elevation_start(elevation_start), m_elevation_end(elevation_end), 
m_valid_segments(segments), m_layer_filter(layer_filter), m_verbose(verbose)
{
}

/*
 * @brief Default destructor.
 */
sick_scansegment_xd::MsgPackValidator::~MsgPackValidator()
{
}

/*
 * @brief Validates a received msgpack against the configured limits, i.e. checks that echo, segment and azimuth are
 *        within their configured range.
 *
 * @param[in] data_received echos, azimuth, elevation and segments collected during msgpack parsing
 *
 * @return true if validation passed successfully, false otherwise.
 */
bool sick_scansegment_xd::MsgPackValidator::validateNotOutOfBound(const MsgPackValidatorData& msgpack_data_received) const
{
    // loop over histogram of azimuth angles: azimuth_count = getHistogram()[echo_idx][segment_idx][elevationToInt(elevation)][azimuthToInt(azimuth)]
    // and check against configured limits
    bool success = true;
    const MsgPackValidatorData::AzimuthHistogramPerElevationPerSegmentPerEcho& azimuth_histogram = msgpack_data_received.getHistogram();
    for(MsgPackValidatorData::AzimuthHistogramPerElevationPerSegmentPerEcho::const_iterator iter_echos = azimuth_histogram.cbegin(); iter_echos != azimuth_histogram.cend(); iter_echos++)
    {
        int echo_idx = iter_echos->first;
        const MsgPackValidatorData::AzimuthHistogramPerElevationPerSegment& azimuth_histogram_elevation_segment = iter_echos->second;
        for(MsgPackValidatorData::AzimuthHistogramPerElevationPerSegment::const_iterator iter_segment = azimuth_histogram_elevation_segment.cbegin(); iter_segment != azimuth_histogram_elevation_segment.cend(); iter_segment++)
        {
            int segment_idx = iter_segment->first;
            const MsgPackValidatorData::AzimuthHistogramPerElevation& azimuth_histogram_elevation = iter_segment->second;
            for(MsgPackValidatorData::AzimuthHistogramPerElevation::const_iterator iter_elevation = azimuth_histogram_elevation.cbegin(); iter_elevation != azimuth_histogram_elevation.cend(); iter_elevation++)
            {
                int elevation_idx = iter_elevation->first;
                float elevation_rad = msgpack_data_received.elevationIndexToRad(elevation_idx);
                const MsgPackValidatorData::AzimuthHistogram& azimuth_histogram = iter_elevation->second;
                std::vector<float> out_of_bounds_azimuth_angles;
                for(MsgPackValidatorData::AzimuthHistogram::const_iterator iter_azimuth = azimuth_histogram.cbegin(); iter_azimuth != azimuth_histogram.cend(); iter_azimuth++)
                {
                    float azimuth_rad = msgpack_data_received.azimuthIndexToRad(iter_azimuth->first);
                    int azimuth_cnt = iter_azimuth->second;
                    if(azimuth_cnt > 0)
                    {
                        if (std::find(m_echos_required.begin(), m_echos_required.end(), echo_idx) == m_echos_required.end())
                        {
                            success = false;
			                ROS_WARN_STREAM("## WARNING MsgPackValidator: echo = " << echo_idx << " unexpected (expected echo: [ " 
                                << listVector(m_echos_required) << " ]");
                            break;
                        }
                        if (std::find(m_valid_segments.begin(), m_valid_segments.end(), segment_idx) == m_valid_segments.end())
                        {
                            success = false;
			                ROS_WARN_STREAM("## WARNING MsgPackValidator: segment = " << segment_idx << " (echo " << echo_idx << ") unexpected (valid segments: [ " 
                                << listVector(m_valid_segments) << " ]");
                            break;
                        }
                        if (elevation_rad < m_elevation_start || elevation_rad > m_elevation_end)
                        {
                            success = false;
			                ROS_WARN_STREAM("## WARNING MsgPackValidator: elevation = " << (elevation_rad * 180.0 / M_PI) << " deg (echo " << echo_idx << ", segment " << segment_idx 
                                << ") out of limits [ " << (m_elevation_start * 180.0 / M_PI) << ", " << (m_elevation_end * 180.0 / M_PI) << " ] deg.");
                            break;
                        }
                        if (azimuth_rad < m_azimuth_start || azimuth_rad > m_azimuth_end)
                        {
                            success = false;
                            out_of_bounds_azimuth_angles.push_back(azimuth_rad * (float)(180.0 / M_PI));
                        }
                    }
                }
                if(!out_of_bounds_azimuth_angles.empty())
                {
			        ROS_WARN_STREAM("## WARNING MsgPackValidator: azimuth angles = [ " << listVector(out_of_bounds_azimuth_angles) << " ] deg (echo " << echo_idx << ", segment " << segment_idx << ", elevation " 
                        << (elevation_rad * 180.0 / M_PI) << " deg) out of limits [ " << (m_azimuth_start * 180.0 / M_PI) << ", " << (m_azimuth_end * 180.0 / M_PI) << " ] deg.");
                }
            }
        }
    }
    if (!success)
        ROS_WARN_STREAM("## WARNING MsgPackValidator::validateNotOutOfBound() finished with error.");
    else if (m_verbose > 1)
        ROS_INFO_STREAM("MsgPackValidator::validateNotOutOfBound() finished successful.");
    return success;
}

/** Returns the number of azimuth angles counted in the histogram for a given echo, segment, elevation and azimuth */
int sick_scansegment_xd::MsgPackValidator::getAzimuthHistogramCount(const MsgPackValidatorData::AzimuthHistogramPerElevationPerSegmentPerEcho& azimuth_histogram,
    int echo_idx, int segment_idx, int elevation_idx, int azimuth_idx) const
{
    MsgPackValidatorData::AzimuthHistogramPerElevationPerSegmentPerEcho::const_iterator collected_echo = azimuth_histogram.find(echo_idx);
    if (collected_echo != azimuth_histogram.cend())
    {
        int collected_echo_idx = collected_echo->first;
        const MsgPackValidatorData::AzimuthHistogramPerElevationPerSegment::const_iterator collected_segment = collected_echo->second.find(segment_idx);
        if (collected_segment != collected_echo->second.cend())
        {
            int collected_segment_idx = collected_segment->first;
            const MsgPackValidatorData::AzimuthHistogramPerElevation::const_iterator collected_elevation = collected_segment->second.find(elevation_idx);
            if (collected_elevation != collected_segment->second.cend())
            {
                int collected_elevation_idx = collected_elevation->first;
                const MsgPackValidatorData::AzimuthHistogram::const_iterator collected_azimuth = collected_elevation->second.find(azimuth_idx);
                if (collected_azimuth != collected_elevation->second.cend())
                {
                    int collected_azimuth_idx = collected_azimuth->first;
                    int collected_azimuth_hist_cnt = collected_azimuth->second;
                    if(collected_echo_idx == echo_idx && collected_segment_idx == segment_idx && collected_elevation_idx == elevation_idx && collected_azimuth_idx == azimuth_idx)
                    {
                        return collected_azimuth_hist_cnt; // given echo, segment, elevation and azimuth found in histogram, return histogram counter
                    }
                }
            }
        }

    }
    return 0; // given echo, segment, elevation and azimuth not found in histogram
}

void sick_scansegment_xd::MsgPackValidator::printMissingHistogramData(const std::vector<std::string>& messages) const
{
    if (m_verbose > 1)
    {
        for (int msg_cnt = 0; msg_cnt < messages.size(); msg_cnt++)
            ROS_WARN_STREAM("## " << messages[msg_cnt]);
    }
}

/*
 * @brief Validates a received msgpack, i.e. checks that the required number of echos, segments and azimuth values have been received.
 *
 * @param[in] msgpack_data_collected echos, azimuth, elevation and segments collected during msgpack parsing
 *
 * @return true if validation passed successfully, false otherwise.
 */
bool sick_scansegment_xd::MsgPackValidator::validateNoMissingScandata(const MsgPackValidatorData& msgpack_data_collected) const
{
    // Idea to validate mspack data for completeness:
    // 1. Received azimuth angles are collected in a histogram over some period for each elevation
    // 2. The expected area (azimuth range) covered by azimuth angles is known from configuration
    // Validation: Check that the expected area is covered by stepping through the histogram in steps of e.g. 1 degree.
    // Example: [-180°,+30] degree is the expected area (azimuth range) given by configuration. 
    // Thus the histogram is checked for azimuth angles -180, -179, -178, ...., 29, 30 [deg]
    // The azimuth histogram is checked with a given resolution. The check is passed, if the histogram counter is > 0 
    // for each azimuth angle within the expected range.

    bool success = true;
    bool azimuth_values_missing = false;
    bool elevation_values_missing = false;
    const MsgPackValidatorData::AzimuthHistogramPerElevationPerSegmentPerEcho& azimuth_histogram = msgpack_data_collected.getHistogram();
    
    // Check all required echos
    for (std::vector<int>::const_iterator iter_echo = m_echos_required.cbegin(); iter_echo != m_echos_required.cend(); iter_echo++)
    {
        int echo_idx = (*iter_echo);
        // Check if required echo found in histogram
        MsgPackValidatorData::AzimuthHistogramPerElevationPerSegmentPerEcho::const_iterator collected_echo = azimuth_histogram.find(echo_idx);
        if (collected_echo == azimuth_histogram.cend() || collected_echo->first != echo_idx || collected_echo->second.empty())
        {
            ROS_WARN_STREAM("## WARNING MsgPackValidator::validateNoMissingScandata() failed: no scan data found in echo " << echo_idx << ":");
            printMissingHistogramData(msgpack_data_collected.print());
            return false;
        }

        // Check all required segments and collect azimuth histograms over all segments
        MsgPackValidatorData::AzimuthHistogramPerElevation collected_azimuth_histogram_per_elevation;
        for (std::vector<int>::const_iterator iter_segment = m_valid_segments.cbegin(); iter_segment != m_valid_segments.cend(); iter_segment++)
        {
            int segment_idx = (*iter_segment);
            const MsgPackValidatorData::AzimuthHistogramPerElevationPerSegment::const_iterator collected_segment = collected_echo->second.find(segment_idx);
            // Check if required segment found in histogram
            if (collected_segment == collected_echo->second.cend() || collected_segment->first != segment_idx || collected_segment->second.empty())
            {
                /* ROS_WARN_STREAM("## WARNING MsgPackValidator::validateNoMissingScandata() failed: no scan data found in segment " << segment_idx << " (echo " << echo_idx << "):");
                printMissingHistogramData(msgpack_data_collected.print());
                return false; */
                continue;
            }
            // Collect azimuth histograms over all segments
            for(MsgPackValidatorData::AzimuthHistogramPerElevation::const_iterator collected_elevation = collected_segment->second.cbegin(); collected_elevation != collected_segment->second.cend(); collected_elevation++)
            {
                int collected_elevation_idx = collected_elevation->first;
                for(MsgPackValidatorData::AzimuthHistogram::const_iterator collected_azimuth = collected_elevation->second.cbegin(); collected_azimuth != collected_elevation->second.cend(); collected_azimuth++)
                {
                    int collected_azimuth_idx = collected_azimuth->first;
                    int collected_azimuth_hist_cnt = collected_azimuth->second;
                    collected_azimuth_histogram_per_elevation[collected_elevation_idx][collected_azimuth_idx] += collected_azimuth_hist_cnt;
                }
            }
        }

        // Check required layers
        int num_layer_required = 0;
        for (int layer_idx = 0; layer_idx < m_layer_filter.size(); layer_idx++)
        {
            if(m_layer_filter[layer_idx])
                num_layer_required++;
        }
        int num_layer_collected = 0;
        for(MsgPackValidatorData::AzimuthHistogramPerElevation::const_iterator collected_elevation = collected_azimuth_histogram_per_elevation.begin(); collected_elevation != collected_azimuth_histogram_per_elevation.end(); collected_elevation++)
        {
            num_layer_collected++;
        }
        if(num_layer_required != num_layer_collected)
        {
            ROS_WARN_STREAM("## WARNING MsgPackValidator::validateNoMissingScandata() failed: " << num_layer_required << " layer expected, but " << num_layer_collected << " layer collected.");
            printMissingHistogramData(msgpack_data_collected.print());
            return false;
        }

        // Check required azimuth angles in all layer
        for(MsgPackValidatorData::AzimuthHistogramPerElevation::iterator collected_elevation = collected_azimuth_histogram_per_elevation.begin(); collected_elevation != collected_azimuth_histogram_per_elevation.end(); collected_elevation++)
        {
            int elevation_idx = collected_elevation->first;
            MsgPackValidatorData::AzimuthHistogram& collected_azimuth_histogram = collected_elevation->second;
            if (collected_azimuth_histogram.empty())
            {
                // no scan data for this elevation
                if (m_verbose > 1)
                    ROS_WARN_STREAM("## WARNING MsgPackValidator::validateNoMissingScandata() failed: no scan data found for elevation " << msgpack_data_collected.elevationIndexToDeg(elevation_idx) 
                        << " [deg] (echo " << echo_idx << ")");
                elevation_values_missing = true;
                success = false;
                continue;
            }
            // Check all required azimuth angles
            float azimuth_res = msgpack_data_collected.getAzimuthHistogramResolutionRad();
            int azimuth_idx_start = msgpack_data_collected.azimuthRadToIndex(m_azimuth_start + azimuth_res);
            int azimuth_idx_end = msgpack_data_collected.azimuthRadToIndex(m_azimuth_end - azimuth_res);
            for (int azimuth_idx = azimuth_idx_start + 1; azimuth_idx < azimuth_idx_end; azimuth_idx++)
            {
                if(collected_azimuth_histogram[azimuth_idx] <= 0)
                {
                    // no scan data for this azimuth
                    if (m_verbose > 1)
                        ROS_WARN_STREAM("## WARNING MsgPackValidator::validateNoMissingScandata() failed: no scan data found for azimuth " << msgpack_data_collected.azimuthIndexToDeg(azimuth_idx) 
                            << " [deg] (echo " << echo_idx << ", elevation " << msgpack_data_collected.elevationIndexToDeg(elevation_idx) << ")");
                    azimuth_values_missing = true;
                    success = false;
                }
                // else
                // {
                //     ROS_INFO_STREAM("elevation " << msgpack_data_collected.elevationIndexToDeg(elevation_idx) << ", azimuth " << msgpack_data_collected.azimuthIndexToDeg(azimuth_idx) 
                //         << " [deg] echo " << echo_idx <<  ": azimuth_hist_cnt = " << collected_azimuth_histogram[azimuth_idx]);
                // }
            }
        }
    }

    if(!success)
    {
        ROS_WARN_STREAM("## WARNING MsgPackValidator::validateNoMissingScandata(): msgpack validation for missing scan data finished with error " << (elevation_values_missing ? "elevation values missing ":"") << (azimuth_values_missing ? "azimuth values missing " : "") );
        printMissingHistogramData(msgpack_data_collected.print());
    }
    else if (m_verbose > 0)
    {
        ROS_INFO_STREAM("MsgPackValidator::validateNoMissingScandata() finished successful.");
    }
    return success;
}
