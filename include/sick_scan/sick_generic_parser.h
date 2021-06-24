/*
 * Copyright (C) 2013, Osnabrück University
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
 *     * Neither the name of Osnabrück University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
#define SICK_SCANNER_RMS_3XX_NAME "sick_rms_3xx"
#define SICK_SCANNER_NAV_3XX_NAME "sick_nav_3xx"
#define SICK_SCANNER_NAV_2XX_NAME "sick_nav_2xx"
#define SICK_SCANNER_TIM_4XX_NAME "sick_tim_4xx"
#include "abstract_parser.h"

#include "sensor_msgs/LaserScan.h"
#include "sick_scan/sick_scan_common.h"
#include "sick_scan/dataDumper.h"
// namespace sensor_msgs
namespace sick_scan
{
  enum EVAL_FIELD_SUPPORT // type of eval field support:
  {
    EVAL_FIELD_UNSUPPORTED = 0,        // Lidar does not support eval field (default)
    USE_EVAL_FIELD_TIM7XX_LOGIC,       // eval fields supported by TiM7xx and TiM7xxS
    USE_EVAL_FIELD_LMS5XX_LOGIC,       // eval fields supported by LMS5XX
    USE_EVAL_FIELD_LMS5XX_UNSUPPORTED, // eval fields not supported by LMS5XX
    USE_EVAL_FIELD_NUM                 // max number of eval field support types
  };

  class ScannerBasicParam
  {
  public:
    void setScannerName(std::string _s);

    std::string getScannerName(void);

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

    bool getUseBinaryProtocol(void);

    void setScanMirroredAndShifted(bool _scanMirroredAndShifted);

    bool getScanMirroredAndShifted();

    void setUseBinaryProtocol(bool _useBinary);

    void setDeviceIsRadar(bool _deviceIsRadar);

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

  private:
    std::string scannerName;
    int numberOfLayers;
    int numberOfShots;
    int numberOfMaximumEchos;
    double elevationDegreeResolution;
    double angleDegressResolution;
    double expectedFrequency;
    bool useBinaryProtocol;
    bool IntensityResolutionIs16Bit;
    bool deviceIsRadar;
    bool useSafetyPasWD;
    int8_t encoderMode;
    bool CartographerCompatibility;
    bool scanMirroredAndShifted;
    EVAL_FIELD_SUPPORT useEvalFields;
    int maxEvalFields;
  };


  class SickGenericParser : public AbstractParser
  {
  public:
    SickGenericParser(std::string scannerType);

    virtual ~SickGenericParser();

    virtual int parse_datagram(char *datagram, size_t datagram_length, SickScanConfig &config,
                               sensor_msgs::LaserScan &msg, int &numEchos, int &echoMask);


    void checkScanTiming(float time_increment, float scan_time, float angle_increment, float tol);

    void set_range_min(float min);

    void set_range_max(float max);

    float get_range_min(void);

    float get_range_max(void);

    void set_time_increment(float time);

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
    ScannerBasicParam *currentParamSet;
  };

} /* namespace sick_scan */
#endif /* SICK_GENERIC_PARSER_H_ */
