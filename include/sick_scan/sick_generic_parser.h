#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * Copyright (C) 2013, Osnabrueck University
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
 *  Created on: 14.11.2013
 *
 *      Author: Martin Günther <mguenthe@uos.de>
 *
 */

#ifndef SICK_GENERIC_PARSER_H_
#define SICK_GENERIC_PARSER_H_

#include <string>

// List of supported laser scanner and radar scanner
#define SICK_SCANNER_LMS_1XXX_NAME "sick_lms_1xxx"
#define SICK_SCANNER_MRS_1XXX_NAME "sick_mrs_1xxx"
#define SICK_SCANNER_TIM_240_NAME "sick_tim_240"
#define SICK_SCANNER_TIM_5XX_NAME "sick_tim_5xx"
#define SICK_SCANNER_TIM_7XX_NAME "sick_tim_7xx"
#define SICK_SCANNER_TIM_7XXS_NAME "sick_tim_7xxS"
#define SICK_SCANNER_LMS_5XX_NAME "sick_lms_5xx"
#define SICK_SCANNER_LMS_1XX_NAME "sick_lms_1xx"
#define SICK_SCANNER_MRS_6XXX_NAME "sick_mrs_6xxx"
#define SICK_SCANNER_LMS_4XXX_NAME "sick_lms_4xxx"
#define SICK_SCANNER_RMS_XXXX_NAME "sick_rms_xxxx" // supports RMS-1xxx and RMS-2xxx
#define SICK_SCANNER_NAV_31X_NAME "sick_nav_31x"
#define SICK_SCANNER_NAV_350_NAME "sick_nav_350"
#define SICK_SCANNER_NAV_2XX_NAME "sick_nav_2xx"
#define SICK_SCANNER_TIM_4XX_NAME "sick_tim_4xx"
#define SICK_SCANNER_LRS_4XXX_NAME "sick_lrs_4xxx"
#define SICK_SCANNER_LRS_36x0_NAME "sick_lrs_36x0"
#define SICK_SCANNER_LRS_36x1_NAME "sick_lrs_36x1"
#define SICK_SCANNER_OEM_15XX_NAME "sick_oem_15xx"
#define SICK_SCANNER_SCANSEGMENT_XD_NAME "sick_multiscan" // "sick_scansegment_xd", multiScan136
#define SICK_SCANNER_PICOSCAN_NAME "sick_picoscan" // picoScan150, part of sick_scansegment_xd family

#include "abstract_parser.h"

#include "sick_scan/sick_scan_common.h"
#include "sick_scan/sick_range_filter.h"
#include "sick_scan/dataDumper.h"
// namespace sensor_msgs
namespace sick_scan_xd
{
  enum EVAL_FIELD_SUPPORT // type of eval field support:
  {
    EVAL_FIELD_UNSUPPORTED = 0,        // Lidar does not support eval field (default)
    USE_EVAL_FIELD_TIM7XX_LOGIC,       // eval fields supported by TiM7xx and TiM7xxS
    USE_EVAL_FIELD_LMS5XX_LOGIC,       // eval fields supported by LMS5XX
    USE_EVAL_FIELD_LMS5XX_UNSUPPORTED, // eval fields not supported by LMS5XX
    USE_EVAL_FIELD_NUM                 // max number of eval field support types
  };

  enum RADAR_TYPE_ENUM
  {
    NO_RADAR = 0,        // sensor is a not a radar (default)
    RADAR_1D = 1,        // sensor is a 1D radar
    RADAR_3D = 2         // sensor is a 3D radar
  };


  class ScannerBasicParam
  {
  public:
    void setScannerName(std::string _s);

    std::string getScannerName(void) const;

    bool isOneOfScannerNames(const std::vector<std::string>& scanner_names) const;

    void setNumberOfLayers(int _layerNum);

    int getNumberOfLayers(void);

    void setNumberOfShots(int _shots);

    int getNumberOfShots(void);

    void setNumberOfMaximumEchos(int _maxEchos);

    int getNumberOfMaximumEchos(void);

    void setAngularDegreeResolution(double _res);

    void setElevationDegreeResolution(double _elevRes);

    double getElevationDegreeResolution(void);

    double getAngularDegreeResolution(void);

    double getExpectedFrequency(void);

    bool getDeviceIsRadar(void);
    RADAR_TYPE_ENUM getDeviceRadarType(void);

    bool getTrackingModeSupported(void);
    void setTrackingModeSupported(bool _trackingModeSupported);

    bool getUseBinaryProtocol(void);

    void setScanMirroredAndShifted(bool _scanMirroredAndShifted);

    bool getScanMirroredAndShifted();
    void setScanAngleShift(double _scanAngleShift);//for NAV310 should be changed in only mirrord in comibantion with new scanangelshift param

    double getScanAngleShift();

    void setImuEnabled(bool _imuEnabled);

    bool getImuEnabled();

    void setUseBinaryProtocol(bool _useBinary);

    void setDeviceIsRadar(RADAR_TYPE_ENUM _radar_type);

    void setIntensityResolutionIs16Bit(bool _IntensityResolutionIs16Bit);

    bool getIntensityResolutionIs16Bit(void);

    void setExpectedFrequency(double _freq);

    ScannerBasicParam();

    void setUseSafetyPasWD(bool _useSafetyPasWD);

    bool getUseSafetyPasWD();

    void setEncoderMode(int8_t _EncoderMode);

    int8_t getEncoderMode();

    EVAL_FIELD_SUPPORT getUseEvalFields();

    void setUseEvalFields(EVAL_FIELD_SUPPORT _useEvalFields);

    int getMaxEvalFields(void);
    
    void setMaxEvalFields(int _maxEvalFields);

    void setRectEvalFieldAngleRefPointOffsetRad(float _rectFieldAngleRefPointOffsetRad);
    float getRectEvalFieldAngleRefPointOffsetRad(void);

    void setUseScancfgList (bool _useScancfgList );

    bool getUseScancfgList();

    void setUseWriteOutputRanges(bool _useWriteOutputRanges);

    bool getUseWriteOutputRanges();

    void setWaitForReady(bool _waitForReady);

    bool getWaitForReady();

    void setFREchoFilterAvailable(bool _frEchoFilterAvailable);

    bool getFREchoFilterAvailable(void);

    void setScandatacfgAzimuthTableSupported(bool _scandatacfgAzimuthTableSupported);
    bool getScandatacfgAzimuthTableSupported(void) const;

  private:
    std::string scannerName;
    int numberOfLayers;
    int numberOfShots;
    int numberOfMaximumEchos;
    double elevationDegreeResolution;
    double angleDegressResolution;
    double expectedFrequency;
    double scanAngleShift;
    bool useBinaryProtocol;
    bool IntensityResolutionIs16Bit;
    RADAR_TYPE_ENUM deviceRadarType = NO_RADAR;
    bool trackingModeSupported = false;
    bool useSafetyPasWD;
    int8_t encoderMode;
    bool CartographerCompatibility;
    bool scanMirroredAndShifted;
    bool imuEnabled;
    EVAL_FIELD_SUPPORT useEvalFields;
    int maxEvalFields;
    float rectFieldAngleRefPointOffsetRad;
    bool useScancfgList;
    bool useWriteOutputRanges;
    bool waitForReady;
    bool frEchoFilterAvailable = false;
    bool scandatacfgAzimuthTableSupported = false;
  };


  class SickGenericParser : public AbstractParser
  {
  public:
    SickGenericParser(std::string scannerType);

    virtual ~SickGenericParser();

    virtual int parse_datagram(char *datagram, size_t datagram_length, SickScanConfig &config,
                               ros_sensor_msgs::LaserScan &msg, int &numEchos, int &echoMask);


    bool checkScanTiming(float time_increment, float scan_time, float angle_increment, float tol);

    void set_range_min(float min);

    void set_range_max(float max);

    float get_range_min(void);

    float get_range_max(void);

    void set_range_filter_config(RangeFilterResultHandling range_filter_handling);
    RangeFilterResultHandling get_range_filter_config(void) const;

    void set_time_increment(float time);

    float get_time_increment(void);

    void setScannerType(std::string s);

    std::string getScannerType(void);

    int lookUpForAllowedScanner(std::string scannerName);

    void setCurrentParamPtr(ScannerBasicParam *_ptr);

    ScannerBasicParam *getCurrentParamPtr();


    int checkForDistAndRSSI(std::vector<char *> &fields, int expected_number_of_data, int &distNum, int &rssiNum,
                            std::vector<float> &distVal, std::vector<float> &rssiVal, int &distMask);


  private:
    float override_range_min_, override_range_max_;
    float override_time_increment_;
    std::string scannerType;
    std::vector<std::string> allowedScannerNames;
    std::vector<ScannerBasicParam> basicParams;
    ScannerBasicParam *currentParamSet = 0;
    RangeFilterResultHandling m_range_filter_handling = RANGE_FILTER_DEACTIVATED;
  };

} /* namespace sick_scan_xd */
#endif /* SICK_GENERIC_PARSER_H_ */
