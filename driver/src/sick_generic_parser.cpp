/**
* \file
* \brief Laser Scanner Parser
* Copyright (C) 2013, Osnabrueck University
* Copyright (C) 2017, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2017, SICK AG, Waldkirch
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
*  Last modified: 05th Nov 2019
*
*      Authors:
*              Michael Lehning <michael.lehning@lehning.de>
*              Jochen Sprickerhof <jochen@sprickerhof.de>
*              Martin Günther <mguenthe@uos.de>
*
* Based on the TiM communication example by SICK AG.
*
*/

#ifdef _MSC_VER
#pragma warning(disable: 4267)
#endif

//#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include <math.h>
#include "sick_scan/sick_scan_common.h"
#include "sick_scan/sick_ros_wrapper.h"
#include "sick_scan/sick_lmd_scandata_parser.h"

namespace sick_scan_xd
{
  using namespace std;

  /*!
  \brief Setting name (type) of scanner

  \param _s name of scanner
  \sa getScannerName
  */
  void ScannerBasicParam::setScannerName(std::string _s)
  {
    scannerName = _s;
  }

  /*!
  \brief Getting name (type) of scanner

  \return Name of scanner
  \sa setScannerName
  */
  std::string ScannerBasicParam::getScannerName() const
  {
    return (scannerName);
  }

  /*!
  \brief Returns true, if the scanner name (type) is found int a given list of scanner names
  */
  bool ScannerBasicParam::isOneOfScannerNames(const std::vector<std::string>& scanner_names) const
  {
    for(int n = 0; n < scanner_names.size(); n++)
    {
      if (getScannerName().compare(scanner_names[n]) == 0)
        return true;
    }
    return false;
  }


  /*!
  \brief Setting number of scanner layers (depending of scanner type/family)

  \param _layerNum of scanner layers (e.g. 1 for TiM5xx and 24 for MRS6124
  \sa getNumberOfLayers
  */
  void ScannerBasicParam::setNumberOfLayers(int _layerNum)
  {
    numberOfLayers = _layerNum;
  }

  /*!
  \brief Getting number of scanner layers

  \return Number of scanners layer (e.g. 1 for TiM5xx and 24 for MRS6124)
  \sa setNumberOfLayers
  */
  int ScannerBasicParam::getNumberOfLayers(void)
  {
    return (numberOfLayers);

  }

  /*!
  \brief Set number of shots per scan

  \param _shots of shots per scan (for one layer)
  \sa getNumberOfLayers
  */
  void ScannerBasicParam::setNumberOfShots(int _shots)
  {
    numberOfShots = _shots;
  }

  /*!
  \brief Get number of shots per scan

  \return Number of shots per scan (for one layer)
  \sa getNumberOfLayers
  */
  int ScannerBasicParam::getNumberOfShots(void)
  {
    return (numberOfShots);
  }

  /*!
  \brief Set number of maximum echoes for this laser scanner type

  \param _maxEchos of max echoes
  \sa getNumberOfMaximumEchos
  */
  void ScannerBasicParam::setNumberOfMaximumEchos(int _maxEchos)
  {
    this->numberOfMaximumEchos = _maxEchos;
  }


  /*!
  \brief Get number of maximum echoes for this laser scanner type

  \return Number of max echoes
  \sa setNumberOfMaximumEchos
  */
  int ScannerBasicParam::getNumberOfMaximumEchos(void)
  {
    return (numberOfMaximumEchos);
  }

  /*!
  \brief Set pointer to corresponding parameter object to the parser

  \param _ptr to parameter object
  \sa getCurrentParamPtr
  */
  void SickGenericParser::setCurrentParamPtr(ScannerBasicParam *_ptr)
  {
    currentParamSet = _ptr;
  }


  /*!
  \brief Set angular resolution in degrees
  \param _res resolution in degress (NOT rad) between each shot
  \sa getAngularDegreeResolution
  */
  void ScannerBasicParam::setAngularDegreeResolution(double _res)
  {
    angleDegressResolution = _res;
  }

  /*!
  \brief Get angular resolution in degress

  \return angle resolution in degress (NOT rad) between each shot
  */
  double ScannerBasicParam::getAngularDegreeResolution(void)
  {
    return (angleDegressResolution);
  }

  /*!
  \brief set expected scan frequency
  \param _freq scan frequency in [Hz]
  \sa getExpectedFrequency
  */
  void ScannerBasicParam::setExpectedFrequency(double _freq)
  {
    expectedFrequency = _freq;
  }

  /*!
  \brief get expected scan frequency

  \return expected scan frequency in [Hz]
  \sa setExpectedFrequency
  */
  double ScannerBasicParam::getExpectedFrequency()
  {
    return (expectedFrequency);
  }


  /*!
  \brief set angular resolution in VERTICAL direction for multilayer scanner
  \param _elevRes resolution in degree
  \sa getElevationDegreeResolution
  */
  void ScannerBasicParam::setElevationDegreeResolution(double _elevRes)
  {
    this->elevationDegreeResolution = _elevRes;
  }


  /*!
  \brief get angular resolution in VERTICAL direction for multilayer scanner
  \return elevation resolution in degree
  \sa setElevationDegreeResolution
  */
  double ScannerBasicParam::getElevationDegreeResolution()
  {
    return (this->elevationDegreeResolution);
  }

  /*!
  \brief flag to decide between usage of ASCII-sopas or BINARY-sopas
  \param _useBinary: True for binary, False for ASCII
  \sa getUseBinaryProtocol
  */
  void ScannerBasicParam::setUseBinaryProtocol(bool _useBinary)
  {
    this->useBinaryProtocol = _useBinary;
  }

  bool ScannerBasicParam::getImuEnabled()
  {
    return this->imuEnabled;
  }
  void ScannerBasicParam::setImuEnabled(bool _imuEnabled)
  {
    this->imuEnabled = _imuEnabled;
  }
  /*!
  \brief flag to mark the device as radar (instead of laser scanner)
  \param _deviceIsRadar: false for laserscanner, true for radar (like rms_xxxx)
  \sa getDeviceIsRadar
  */
  void ScannerBasicParam::setDeviceIsRadar(RADAR_TYPE_ENUM _radar_type)
  {
    deviceRadarType = _radar_type;
  }

  /*!
  \brief flag to mark the device as radar (instead of laser scanner)
  \param _deviceIsRadar: false for laserscanner, true for radar (like rms_xxxx)
  \sa getDeviceIsRadar
  */
  bool ScannerBasicParam::getDeviceIsRadar(void)
  {
    return (deviceRadarType != NO_RADAR);
  }

  RADAR_TYPE_ENUM ScannerBasicParam::getDeviceRadarType(void)
  {
    return deviceRadarType;
  }

  /*!
  \brief set/get flag to mark the radar device supports selection of tracking modes.
  By default, true for all radar devices except RMS-1xxx, otherwise false.
  */
bool ScannerBasicParam::getTrackingModeSupported(void)
{
  return (getDeviceIsRadar() && trackingModeSupported);
}
void ScannerBasicParam::setTrackingModeSupported(bool _trackingModeSupported)
{
  trackingModeSupported = _trackingModeSupported;
}


  /*!
\brief flag to mark mirroring of rotation direction
\param _scanMirrored: false for normal mounting true for up side down or NAV 310
\sa setScanMirrored
*/
  void ScannerBasicParam::setScanMirroredAndShifted(bool _scannMirroredAndShifted)
  {
    scanMirroredAndShifted = _scannMirroredAndShifted;
  }

  /*!
  \brief flag to mark mirroring of rotation direction
  \param _scanMirrored:  false for normal mounting true for up side down or NAV 310
  \sa getScanMirrored
  */
  bool ScannerBasicParam::getScanMirroredAndShifted(void)
  {
    return (scanMirroredAndShifted);
  }

  /*!
  \brief flag to decide between usage of ASCII-sopas or BINARY-sopas
  \return _useBinary: True for binary, False for ASCII
  \sa getUseBinaryProtocol
  */
  bool ScannerBasicParam::getUseBinaryProtocol(void)
  {
    return (this->useBinaryProtocol);
  }

  /*!
  \brief Set the RSSI Value length
  \param _useBinary: Boolean value: True=16 Bit False=8Bit
  \sa getUseBinaryProtocol
  */
  void ScannerBasicParam::setIntensityResolutionIs16Bit(bool _IntensityResolutionIs16Bit)
  {
    this->IntensityResolutionIs16Bit = _IntensityResolutionIs16Bit;
  }

  /*!
  \brief Get the RSSI Value length
  \return Boolean value: True=16 Bit False=8Bit
  \sa setUseBinaryProtocol
  */
  bool ScannerBasicParam::getIntensityResolutionIs16Bit(void)
  {
    return (IntensityResolutionIs16Bit);
  }

  /*!
  \brief flag to mark the device uses the safety scanner password
  \param  _useSafetyPasWD: false for normal scanners true for safety scanners
  \sa setUseSafetyPasWD
  */
  void ScannerBasicParam::setUseSafetyPasWD(bool _useSafetyPasWD)
  {
    this->useSafetyPasWD = _useSafetyPasWD;
  }

  /*!
  \brief flag to mark the device uses the safety scanner password
  \reutrn Bool true for safety password false for normal password
  \sa getUseSafetyPasWD
  */
  bool ScannerBasicParam::getUseSafetyPasWD()
  {
    return (useSafetyPasWD);
  }

  EVAL_FIELD_SUPPORT ScannerBasicParam::getUseEvalFields()
  {
    return this->useEvalFields;
  }

  void ScannerBasicParam::setUseEvalFields(EVAL_FIELD_SUPPORT _useEvalFields)
  {
    this->useEvalFields = _useEvalFields;
  }

  int ScannerBasicParam::getMaxEvalFields(void)
  {
    return this->maxEvalFields;
  }

  void ScannerBasicParam::setMaxEvalFields(int _maxEvalFields)
  {
    this->maxEvalFields = _maxEvalFields;
  }

  void ScannerBasicParam::setRectEvalFieldAngleRefPointOffsetRad(float _rectFieldAngleRefPointOffsetRad)
  {
    this->rectFieldAngleRefPointOffsetRad = _rectFieldAngleRefPointOffsetRad;
  }

  float ScannerBasicParam::getRectEvalFieldAngleRefPointOffsetRad(void)
  {
    return this->rectFieldAngleRefPointOffsetRad;
  }

  void ScannerBasicParam::setUseScancfgList (bool _useScancfgList )
  {
    this->useScancfgList = _useScancfgList;
  }
  bool ScannerBasicParam::getUseScancfgList()
  {
    return this->useScancfgList;
  }

  void ScannerBasicParam::setUseWriteOutputRanges(bool _useWriteOutputRanges)
  {
      this->useWriteOutputRanges = _useWriteOutputRanges;
  }

  bool ScannerBasicParam::getUseWriteOutputRanges()
  {
      return this->useWriteOutputRanges;
  }

  void ScannerBasicParam::setWaitForReady(bool _waitForReady)
  {
    this->waitForReady = _waitForReady;
  }
  bool ScannerBasicParam::getWaitForReady()
  {
    return this->waitForReady;
  }

  void ScannerBasicParam::setFREchoFilterAvailable(bool _frEchoFilterAvailable)
   {
    this->frEchoFilterAvailable = _frEchoFilterAvailable;
  }

  bool ScannerBasicParam::getFREchoFilterAvailable(void)
  {
    return this->frEchoFilterAvailable;
  }

  void ScannerBasicParam::setScandatacfgAzimuthTableSupported(bool _scandatacfgAzimuthTableSupported)
   {
    this->scandatacfgAzimuthTableSupported = _scandatacfgAzimuthTableSupported;
  }

  bool ScannerBasicParam::getScandatacfgAzimuthTableSupported(void) const
  {
    return this->scandatacfgAzimuthTableSupported;
  }


  /*!
  \brief Construction of parameter object

  */
  ScannerBasicParam::ScannerBasicParam()
  : numberOfLayers(0), numberOfShots(0), numberOfMaximumEchos(0), elevationDegreeResolution(0), angleDegressResolution(0), expectedFrequency(0),
     useBinaryProtocol(false), IntensityResolutionIs16Bit(false), deviceRadarType(NO_RADAR), useSafetyPasWD(false), encoderMode(0),
     CartographerCompatibility(false), scanMirroredAndShifted(false), useEvalFields(EVAL_FIELD_UNSUPPORTED), maxEvalFields(0), rectFieldAngleRefPointOffsetRad(0),
     imuEnabled (false), scanAngleShift(0), useScancfgList(false), useWriteOutputRanges(false)
  {
    this->elevationDegreeResolution = 0.0;
    this->setUseBinaryProtocol(false);
  }

  /*!
\brief Prama for encoder mode
\param _EncoderMode: -1 Use for Scanners WO Encoder 00 disabled 01 single increment 02 direction recognition phase 03 direction recognition level
\sa setEncoderMode
*/
  void ScannerBasicParam::setEncoderMode(int8_t _encoderMode)
  {
    this->encoderMode = _encoderMode;
  }

  /*!
  /*!
\brief Getter-Method for encoder mode
\return EncoderMode:-1 Use for Scanners WO Encoder  00 disabled 01 single increment 02 direction recognition phase 03 direction recognition level
\sa setEncoderMode
  */
  int8_t ScannerBasicParam::getEncoderMode()
  {
    return (encoderMode);
  }

  void ScannerBasicParam::setScanAngleShift(double _scanAngleShift)
  {
    this->scanAngleShift = _scanAngleShift;
  }

  double ScannerBasicParam::getScanAngleShift()
  {
     return this->scanAngleShift;
  }

  /*!
  \brief Construction of parser object
   \param _scanType Type of the Laserscanner

  */
  SickGenericParser::SickGenericParser(std::string _scanType) :
      AbstractParser(),
      override_range_min_((float) 0.05),
      override_range_max_((float) 100.0),
      override_time_increment_((float) -1.0)
  {
    setScannerType(_scanType);
    allowedScannerNames.push_back(SICK_SCANNER_MRS_1XXX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_TIM_240_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_TIM_5XX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_TIM_7XX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_TIM_7XXS_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_LMS_5XX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_LMS_1XX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_LMS_1XXX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_MRS_6XXX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_LMS_4XXX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_LRS_4XXX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_RMS_XXXX_NAME); // Radar scanner
    allowedScannerNames.push_back(SICK_SCANNER_NAV_31X_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_NAV_350_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_NAV_2XX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_TIM_4XX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_LRS_36x0_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_LRS_36x1_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_OEM_15XX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_SCANSEGMENT_XD_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_PICOSCAN_NAME);
    basicParams.resize(allowedScannerNames.size()); // resize to number of supported scanner types
    for (int i = 0; i <
                    (int) basicParams.size(); i++) // set specific parameter for each scanner type - scanner type is identified by name
    {
      basicParams[i].setDeviceIsRadar(NO_RADAR); // (false); // Default
      basicParams[i].setTrackingModeSupported(false); // Default
      basicParams[i].setScannerName(allowedScannerNames[i]);  // set scanner type for this parameter object
      basicParams[i].setScandatacfgAzimuthTableSupported(false);

      if (basicParams[i].getScannerName().compare(SICK_SCANNER_MRS_1XXX_NAME) ==
          0)  // MRS1000 - 4 layer, 1101 shots per scan
      {
        basicParams[i].setNumberOfMaximumEchos(3);
        basicParams[i].setNumberOfLayers(4);
        basicParams[i].setNumberOfShots(1101);
        basicParams[i].setAngularDegreeResolution(0.25);
        basicParams[i].setElevationDegreeResolution(2.5); // in [degree]
        basicParams[i].setExpectedFrequency(50.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(NO_RADAR); // (false); // Default
        basicParams[i].setTrackingModeSupported(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setImuEnabled(true);// Activate Imu for MRS1000
        basicParams[i].setScanAngleShift(-M_PI/2);
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
        basicParams[i].setRectEvalFieldAngleRefPointOffsetRad(0);
        basicParams[i].setUseScancfgList(false);
        basicParams[i].setUseWriteOutputRanges(true); // default: use "sWN LMPoutputRange" if scan configuration not set by ScanCfgList-entry
        basicParams[i].setWaitForReady(true);
        basicParams[i].setFREchoFilterAvailable(true);
        basicParams[i].setScandatacfgAzimuthTableSupported(true);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_LMS_1XXX_NAME) ==
          0)  // LMS1000 - 4 layer, 1101 shots per scan
      {
        basicParams[i].setNumberOfMaximumEchos(3);
        basicParams[i].setNumberOfLayers(4);
        basicParams[i].setNumberOfShots(1101);
        basicParams[i].setAngularDegreeResolution(1.00);  // 0.25° wurde nicht unterstuetzt. (SFA 4)
        basicParams[i].setElevationDegreeResolution(0.0); // in [degree]
        basicParams[i].setExpectedFrequency(50.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(NO_RADAR); // (false); // Default
        basicParams[i].setTrackingModeSupported(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setImuEnabled(false);// Default
        basicParams[i].setScanAngleShift(-M_PI/2);
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
        basicParams[i].setRectEvalFieldAngleRefPointOffsetRad(0);
        basicParams[i].setUseScancfgList(false);
        basicParams[i].setUseWriteOutputRanges(true); // default: use "sWN LMPoutputRange" if scan configuration not set by ScanCfgList-entry
        basicParams[i].setWaitForReady(true);
        basicParams[i].setFREchoFilterAvailable(false);
        basicParams[i].setScandatacfgAzimuthTableSupported(true);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_TIM_240_NAME) ==
          0) // TIM_5xx - 1 Layer, max. 811 shots per scan
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(241); // [-120 deg, 120 deg]
        basicParams[i].setAngularDegreeResolution(1.00000);
        basicParams[i].setExpectedFrequency(15.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(NO_RADAR); // (false); // Default
        basicParams[i].setTrackingModeSupported(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setImuEnabled(false);// Default
        basicParams[i].setScanAngleShift(0);
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
        basicParams[i].setRectEvalFieldAngleRefPointOffsetRad(0);
        basicParams[i].setUseScancfgList(false);
        basicParams[i].setUseWriteOutputRanges(true); // default: use "sWN LMPoutputRange" if scan configuration not set by ScanCfgList-entry
        basicParams[i].setWaitForReady(false);
        basicParams[i].setFREchoFilterAvailable(false);
        basicParams[i].setScandatacfgAzimuthTableSupported(false);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_TIM_5XX_NAME) ==
          0) // TIM_5xx - 1 Layer, max. 811 shots per scan
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(811);
        basicParams[i].setAngularDegreeResolution(0.3333);
        basicParams[i].setExpectedFrequency(15.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(NO_RADAR); // (false); // Default
        basicParams[i].setTrackingModeSupported(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setImuEnabled(false);// Default
        basicParams[i].setScanAngleShift(-M_PI/2);
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
        basicParams[i].setRectEvalFieldAngleRefPointOffsetRad(0);
        basicParams[i].setUseScancfgList(false);
        basicParams[i].setUseWriteOutputRanges(true); // default: use "sWN LMPoutputRange" if scan configuration not set by ScanCfgList-entry
        basicParams[i].setWaitForReady(false);
        basicParams[i].setFREchoFilterAvailable(false);
        basicParams[i].setScandatacfgAzimuthTableSupported(false);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_LMS_4XXX_NAME) == 0) // LMS_4xxx - 1 Layer, 600 Hz
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(841);//
        basicParams[i].setAngularDegreeResolution(0.0833);//
        basicParams[i].setExpectedFrequency(600.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(NO_RADAR); // (false); // Default
        basicParams[i].setTrackingModeSupported(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setImuEnabled(false);// Default
        basicParams[i].setScanAngleShift(-M_PI/2);
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
        basicParams[i].setRectEvalFieldAngleRefPointOffsetRad(0);
        basicParams[i].setUseScancfgList(false);
        basicParams[i].setUseWriteOutputRanges(true); // default: use "sWN LMPoutputRange" if scan configuration not set by ScanCfgList-entry
        basicParams[i].setWaitForReady(false);
        basicParams[i].setFREchoFilterAvailable(false);
        basicParams[i].setScandatacfgAzimuthTableSupported(false);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_TIM_7XX_NAME) == 0) // TIM_7xx - 1 Layer Scanner
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(811);
        basicParams[i].setAngularDegreeResolution(0.3333);
        basicParams[i].setExpectedFrequency(15.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(NO_RADAR); // (false); // Default
        basicParams[i].setTrackingModeSupported(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setImuEnabled(false);// Default
        basicParams[i].setScanAngleShift(-M_PI/2);
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(USE_EVAL_FIELD_TIM7XX_LOGIC);
        basicParams[i].setMaxEvalFields(48);
        basicParams[i].setRectEvalFieldAngleRefPointOffsetRad(0);
        basicParams[i].setUseScancfgList(false);
        basicParams[i].setUseWriteOutputRanges(true); // default: use "sWN LMPoutputRange" if scan configuration not set by ScanCfgList-entry
        basicParams[i].setWaitForReady(false);
        basicParams[i].setFREchoFilterAvailable(false);
        basicParams[i].setScandatacfgAzimuthTableSupported(false);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_TIM_7XXS_NAME) == 0) // TIM_7xxS - 1 layer Safety Scanner
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(811);
        basicParams[i].setAngularDegreeResolution(0.3333);
        basicParams[i].setExpectedFrequency(15.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(NO_RADAR); // (false); // Default
        basicParams[i].setTrackingModeSupported(false); // Default
        basicParams[i].setUseSafetyPasWD(true); // Safety scanner
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setImuEnabled(false);// Default
        basicParams[i].setScanAngleShift(-M_PI/2);
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(USE_EVAL_FIELD_TIM7XX_LOGIC);
        basicParams[i].setMaxEvalFields(48);
        basicParams[i].setRectEvalFieldAngleRefPointOffsetRad(0);
        basicParams[i].setUseScancfgList(false);
        basicParams[i].setUseWriteOutputRanges(true); // default: use "sWN LMPoutputRange" if scan configuration not set by ScanCfgList-entry
        basicParams[i].setWaitForReady(false);
        basicParams[i].setFREchoFilterAvailable(false);
        basicParams[i].setScandatacfgAzimuthTableSupported(false);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_LMS_5XX_NAME) == 0) // LMS_5xx - 1 Layer
      {
        basicParams[i].setNumberOfMaximumEchos(5); // (1) LMS sends up to 5 echos
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(381);
        basicParams[i].setAngularDegreeResolution(0.5);
        basicParams[i].setExpectedFrequency(15.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(NO_RADAR); // (false); // Default
        basicParams[i].setTrackingModeSupported(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setImuEnabled(false);// Default
        basicParams[i].setScanAngleShift(-M_PI/2);
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(USE_EVAL_FIELD_LMS5XX_LOGIC);
        basicParams[i].setMaxEvalFields(30);
        basicParams[i].setRectEvalFieldAngleRefPointOffsetRad(0);
        basicParams[i].setUseScancfgList(false);
        basicParams[i].setUseWriteOutputRanges(true); // default: use "sWN LMPoutputRange" if scan configuration not set by ScanCfgList-entry
        basicParams[i].setWaitForReady(false);
        basicParams[i].setFREchoFilterAvailable(true); // LMS uses echo filter settings to configure number of echos: "sWN FREchoFilter N" with N=0: first echo, N=1: all echos, N=2: last echo
        basicParams[i].setScandatacfgAzimuthTableSupported(false);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_LMS_1XX_NAME) == 0) // LMS_1xx - 1 Layer
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(1081);
        basicParams[i].setAngularDegreeResolution(0.5);
        basicParams[i].setExpectedFrequency(25.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(NO_RADAR); // (false); // Default
        basicParams[i].setTrackingModeSupported(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setImuEnabled(false);// Default
        basicParams[i].setScanAngleShift(-M_PI/2);
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(USE_EVAL_FIELD_LMS5XX_LOGIC);
        basicParams[i].setMaxEvalFields(30);
        basicParams[i].setRectEvalFieldAngleRefPointOffsetRad((float)(-45 * M_PI / 180.0));
        basicParams[i].setUseScancfgList(false);
        basicParams[i].setUseWriteOutputRanges(true); // default: use "sWN LMPoutputRange" if scan configuration not set by ScanCfgList-entry
        basicParams[i].setWaitForReady(true); // changed from false to true, see comment in sick_lms1xx.launch
        basicParams[i].setFREchoFilterAvailable(false);
        basicParams[i].setScandatacfgAzimuthTableSupported(false);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_LRS_36x0_NAME) == 0) //
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(7200);
        basicParams[i].setAngularDegreeResolution(0.25); // (0.5);
        basicParams[i].setExpectedFrequency(8.0); // (15.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(NO_RADAR); // (false); // Default
        basicParams[i].setTrackingModeSupported(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setImuEnabled(false);// Default
        basicParams[i].setScanAngleShift(+M_PI/2); // (+M_PI/2);
        basicParams[i].setScanMirroredAndShifted(true); // (true);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);// TODO Check this
        basicParams[i].setMaxEvalFields(30);
        basicParams[i].setRectEvalFieldAngleRefPointOffsetRad(0);
        basicParams[i].setUseScancfgList(true);
        basicParams[i].setUseWriteOutputRanges(false); // default: use "sWN LMPoutputRange" if scan configuration not set by ScanCfgList-entry
        basicParams[i].setWaitForReady(true);
        basicParams[i].setFREchoFilterAvailable(false);
        basicParams[i].setScandatacfgAzimuthTableSupported(false);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_LRS_36x1_NAME) == 0) //
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(7200);
        basicParams[i].setAngularDegreeResolution(0.5);
        basicParams[i].setExpectedFrequency(15.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(NO_RADAR); // (false); // Default
        basicParams[i].setTrackingModeSupported(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setImuEnabled(false);// Default
        basicParams[i].setScanAngleShift(-M_PI);
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);// TODO Check this
        basicParams[i].setMaxEvalFields(30);
        basicParams[i].setRectEvalFieldAngleRefPointOffsetRad(0);
        basicParams[i].setUseScancfgList(true);
        basicParams[i].setUseWriteOutputRanges(false); // default: use "sWN LMPoutputRange" if scan configuration not set by ScanCfgList-entry
        basicParams[i].setWaitForReady(true);
        basicParams[i].setFREchoFilterAvailable(false);
        basicParams[i].setScandatacfgAzimuthTableSupported(false);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_MRS_6XXX_NAME) == 0) //
      {
        basicParams[i].setNumberOfMaximumEchos(5);
        basicParams[i].setNumberOfLayers(24);
        basicParams[i].setNumberOfShots(925);
        basicParams[i].setAngularDegreeResolution(0.13);
        basicParams[i].setElevationDegreeResolution(1.25); // in [degree]
        basicParams[i].setExpectedFrequency(50.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(NO_RADAR); // (false); // Default
        basicParams[i].setTrackingModeSupported(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setImuEnabled(false);// Default
        basicParams[i].setScanAngleShift(-M_PI/2);
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
        basicParams[i].setRectEvalFieldAngleRefPointOffsetRad(0);
        basicParams[i].setUseScancfgList(false);
        basicParams[i].setUseWriteOutputRanges(true); // default: use "sWN LMPoutputRange" if scan configuration not set by ScanCfgList-entry
        basicParams[i].setWaitForReady(false);
        basicParams[i].setFREchoFilterAvailable(true);
        basicParams[i].setScandatacfgAzimuthTableSupported(false);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_LRS_4XXX_NAME) == 0) // LRS_4XXX - 1 Layer
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(7201);
        basicParams[i].setAngularDegreeResolution(0.05);
        basicParams[i].setExpectedFrequency(12.5);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(NO_RADAR); // (false); // Default
        basicParams[i].setTrackingModeSupported(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setImuEnabled(false);// Default
        basicParams[i].setScanAngleShift(0);
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
        basicParams[i].setRectEvalFieldAngleRefPointOffsetRad(0);
        basicParams[i].setUseScancfgList(true); // false // LRS4000 sets scan rate and angular resolution set by "sMN mCLsetscancfglist <mode>"
        basicParams[i].setUseWriteOutputRanges(true); // false // LRS4000 sets the scan configuration by both "sMN mCLsetscancfglist <mode>" AND "sWN LMPoutputRange" (default: use "sWN LMPoutputRange" if scan configuration not set by ScanCfgList-entry)
        basicParams[i].setWaitForReady(false);
        basicParams[i].setFREchoFilterAvailable(true); // (false) // LRS4XXX uses echo filter settings to configure 1 echo, use filter_echos = 0 (first echo) for LRS4xxx
        basicParams[i].setScandatacfgAzimuthTableSupported(false);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_RMS_XXXX_NAME) == 0) // Radar
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(0); // for radar scanner
        basicParams[i].setNumberOfShots(65);
        basicParams[i].setAngularDegreeResolution(0.00);
        basicParams[i].setElevationDegreeResolution(0.00); // in [degree]
        basicParams[i].setExpectedFrequency(0.00);
        basicParams[i].setUseBinaryProtocol(false); // use ASCII-Protocol
        basicParams[i].setDeviceIsRadar(RADAR_3D); // Default: Device is a 3D radar (RMS-1xxx switched to 1D radar after device type query)
        basicParams[i].setTrackingModeSupported(true); // Default: tracking mode enabled (not supported by RMS-1xxx, disabled for RMS-1 after device type query)
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setImuEnabled(false);// Default
        basicParams[i].setScanAngleShift(0);
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
        basicParams[i].setRectEvalFieldAngleRefPointOffsetRad(0);
        basicParams[i].setUseScancfgList(false);
        basicParams[i].setUseWriteOutputRanges(true); // default: use "sWN LMPoutputRange" if scan configuration not set by ScanCfgList-entry
        basicParams[i].setWaitForReady(false);
        basicParams[i].setFREchoFilterAvailable(false);
        basicParams[i].setScandatacfgAzimuthTableSupported(false);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_NAV_31X_NAME) == 0) // Nav 3xx
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(2880);
        basicParams[i].setAngularDegreeResolution(0.750);
        basicParams[i].setExpectedFrequency(55.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(NO_RADAR); // (false); // Default
        basicParams[i].setTrackingModeSupported(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setImuEnabled(false);// Default
        basicParams[i].setScanAngleShift(0);
        basicParams[i].setScanMirroredAndShifted(true);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
        basicParams[i].setRectEvalFieldAngleRefPointOffsetRad(0);
        basicParams[i].setUseScancfgList(true);
        basicParams[i].setUseWriteOutputRanges(false); // default: use "sWN LMPoutputRange" if scan configuration not set by ScanCfgList-entry
        basicParams[i].setWaitForReady(true);
        basicParams[i].setFREchoFilterAvailable(false);
        basicParams[i].setScandatacfgAzimuthTableSupported(false);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_NAV_350_NAME) == 0)
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(1440);
        basicParams[i].setAngularDegreeResolution(0.250);
        basicParams[i].setExpectedFrequency(8.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(NO_RADAR); // (false);
        basicParams[i].setTrackingModeSupported(false);
        basicParams[i].setUseSafetyPasWD(false);
        basicParams[i].setEncoderMode(-1);
        basicParams[i].setImuEnabled(false);
        basicParams[i].setScanAngleShift(-M_PI);
        basicParams[i].setScanMirroredAndShifted(true);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
        basicParams[i].setRectEvalFieldAngleRefPointOffsetRad(0);
        basicParams[i].setUseScancfgList(true);
        basicParams[i].setUseWriteOutputRanges(false); // default: use "sWN LMPoutputRange" if scan configuration not set by ScanCfgList-entry
        basicParams[i].setWaitForReady(false);
        basicParams[i].setFREchoFilterAvailable(false);
        basicParams[i].setScandatacfgAzimuthTableSupported(false);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_OEM_15XX_NAME) == 0) // OEM 15xx
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(2880);
        basicParams[i].setAngularDegreeResolution(0.25); // (0.750);
        basicParams[i].setExpectedFrequency(8.0); // (55.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(NO_RADAR); // (false); // Default
        basicParams[i].setTrackingModeSupported(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setImuEnabled(false);// Default
        basicParams[i].setScanAngleShift(0); // (0);
        basicParams[i].setScanMirroredAndShifted(true); // (false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
        basicParams[i].setRectEvalFieldAngleRefPointOffsetRad(0);
        basicParams[i].setUseScancfgList(true);
        basicParams[i].setUseWriteOutputRanges(false); // default: use "sWN LMPoutputRange" if scan configuration not set by ScanCfgList-entry
        basicParams[i].setWaitForReady(true);
        basicParams[i].setFREchoFilterAvailable(false);
        basicParams[i].setScandatacfgAzimuthTableSupported(false);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_NAV_2XX_NAME) == 0) // NAV_2xx - 1 Layer
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(1081);
        basicParams[i].setAngularDegreeResolution(0.5);
        basicParams[i].setExpectedFrequency(25.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(NO_RADAR); // (false); // Default
        basicParams[i].setTrackingModeSupported(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setImuEnabled(false);// Default
        basicParams[i].setScanAngleShift(0);
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
        basicParams[i].setRectEvalFieldAngleRefPointOffsetRad(0);
        basicParams[i].setUseScancfgList(false);
        basicParams[i].setUseWriteOutputRanges(true); // default: use "sWN LMPoutputRange" if scan configuration not set by ScanCfgList-entry
        basicParams[i].setWaitForReady(false);
        basicParams[i].setFREchoFilterAvailable(false);
        basicParams[i].setScandatacfgAzimuthTableSupported(false);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_TIM_4XX_NAME) == 0) // TiM433 and TiM443
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(721);
        basicParams[i].setAngularDegreeResolution(0.33333333333);
        basicParams[i].setExpectedFrequency(15.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(NO_RADAR); // (false); // Default
        basicParams[i].setTrackingModeSupported(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setImuEnabled(false);// Default
        basicParams[i].setScanAngleShift(-M_PI/2);
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
        basicParams[i].setRectEvalFieldAngleRefPointOffsetRad(0);
        basicParams[i].setUseScancfgList(false);
        basicParams[i].setUseWriteOutputRanges(true); // default: use "sWN LMPoutputRange" if scan configuration not set by ScanCfgList-entry
        basicParams[i].setWaitForReady(false);
        basicParams[i].setFREchoFilterAvailable(false);
        basicParams[i].setScandatacfgAzimuthTableSupported(false);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_SCANSEGMENT_XD_NAME) == 0 
      || basicParams[i].getScannerName().compare(SICK_SCANNER_PICOSCAN_NAME) == 0)
      {
        // SCANSEGMENT_XD (Multiscan 136, picoscan150) handled by msgpack_converter and msgpack_exporter
      }
    }

    int scannerIdx = lookUpForAllowedScanner(scannerType);

    if (scannerIdx == -1)  // find index of parameter set - derived from scanner type name
    {
      ROS_ERROR("Scanner not supported.\n");
      throw new std::string("scanner type " + scannerType + " not supported.");
    }
    else
    {
      currentParamSet = &(basicParams[scannerIdx]);
    }
  }

  /*!
  \brief Gets Pointer to parameter object
  \return Pointer to parameter object holding information about protocol usage and scanner type specific parameter
  */
  ScannerBasicParam *SickGenericParser::getCurrentParamPtr()
  {
    return (currentParamSet);
  }

  /*!
  \brief checks the given scannerName/scannerType of validity
  \param scannerName as string (e.g. "tim_5xx")
  \return index of found scanner. -1 corresponds to "not found"
  */
  int SickGenericParser::lookUpForAllowedScanner(std::string scannerName)
  {
    int iRet = -1;
    for (int i = 0; i < (int) allowedScannerNames.size(); i++)
    {
      if (allowedScannerNames[i].compare(scannerName) == 0)
      {
        return (i);
      }
    }

    return (iRet);
  }

  /*!
  \brief Destructor of parser
  \sa Constructor SickGenericParser
  */
  SickGenericParser::~SickGenericParser()
  {
  }

  /*!
  \brief check for DIST and RSSI-entries in the datagram. Helper routine for parser

  \param fields: String entries holding the information
  \param expected_number_of_data: Warning, if the number of found entries does not correspond to this entries
  \param distNum: Number of found DIST-entries
  \param rssiNum: Number of found RSSI-entries
  \param distVal: parsed istance values
  \param rssiVal: parsed RSSI-values
  \param distMask: Bit-Masking holds the information of found DIST-entries (e.g. DIST1 -> Bit 0, DIST2 -> BIT 1 and so on)
  \return Errorcode
  \sa parse_datagram
  */
  int SickGenericParser::checkForDistAndRSSI(std::vector<char *> &fields, int expected_number_of_data, int &distNum,
                                             int &rssiNum, std::vector<float> &distVal, std::vector<float> &rssiVal,
                                             int &distMask)
  {
    int iRet = ExitSuccess;
    distNum = 0;
    rssiNum = 0;
    int baseOffset = 20;

    distMask = 0;
    // More in depth checks: check data length and RSSI availability
    // 25: Number of data (<= 10F)
    unsigned short int number_of_data = 0;
    if (strstr(fields[baseOffset], "DIST") != fields[baseOffset]) // First initial check
    {
      ROS_WARN_STREAM("Field 20 of received data does not start with DIST (is: " << std::string(fields[20]) << ". Unexpected data, ignoring scan\n");
      return ExitError;
    }

    int offset = 20;
    do
    {
      bool distFnd = false;
      bool rssiFnd = false;
      if (strlen(fields[offset]) == 5)
      {
        if (strstr(fields[offset], "DIST") == fields[offset])
        {
          distFnd = true;
          distNum++;
          int distId = -1;
          if (1 == sscanf(fields[offset], "DIST%d", &distId))
          {
            distMask |= (1 << (distId - 1)); // set bit regarding to id
          }
        }
        if (strstr(fields[offset], "RSSI") == fields[offset])
        {
          rssiNum++;
          rssiFnd = true;
        }
      }
      if (rssiFnd || distFnd)
      {
        offset += 5;
        if (offset >= (int) fields.size())
        {
          ROS_WARN("Missing RSSI or DIST data");
          return ExitError;
        }
        number_of_data = 0;
        sscanf(fields[offset], "%hx", &number_of_data);
        if (number_of_data != expected_number_of_data)
        {
          ROS_WARN("number of dist or rssi values mismatching.");
          return ExitError;
        }
        offset++;
        // Here is the first value
        for (int i = 0; i < number_of_data; i++)
        {
          if (distFnd)
          {
            unsigned short iRange;
            float range;
            sscanf(fields[offset + i], "%hx", &iRange);
            range = iRange / 1000.0f;
            distVal.push_back(range);
          }
          else
          {
            unsigned short iRSSI;
            sscanf(fields[offset + i], "%hx", &iRSSI);
            rssiVal.push_back((float) iRSSI);
          }
        }
        offset += number_of_data;
      }
      else
      {
        offset++; // necessary????
      }
    } while (offset < (int) fields.size());

    return (iRet);
  }


  bool SickGenericParser::checkScanTiming(float time_increment, float scan_time, float angle_increment, float tol)
  {
    if (this->getCurrentParamPtr()->getNumberOfLayers() > 1)
    {
      return true;
    }

    float expected_time_increment = (float)
        fabs(this->getCurrentParamPtr()->getNumberOfLayers() * scan_time * angle_increment / (2.0 * M_PI));//If the direction of rotation is reversed, i.e. negative angle increment, a negative scan time results. This does not makes sense, therefore the absolute value is calculated.
    if (fabs(expected_time_increment - time_increment) > 0.00001)
    {
#if defined __ROS_VERSION && __ROS_VERSION == 1
      ROS_WARN_THROTTLE(60,
                        "The time_increment, scan_time and angle_increment values reported by the scanner are inconsistent! "
                        "Expected time_increment: %.9f, reported time_increment: %.9f "
                        "(time_increment=%.9f, scan_time=%.9f, angle_increment=%.9f). "
                        "Check angle shift settings. Perhaps you should set the parameter time_increment to the expected value. This message will print every 60 seconds.",
                        expected_time_increment, time_increment, time_increment, scan_time, angle_increment*180.0/M_PI);
#else
        static rosTime last_message_time(0);
        if ((rosTimeNow() - last_message_time) > rosDurationFromSec(60))
        {
          last_message_time = rosTimeNow();
          ROS_WARN_STREAM(
              "The time_increment, scan_time and angle_increment values reported by the scanner are inconsistent! "
              << "Expected time_increment: " << expected_time_increment << ", reported time_increment:" << time_increment << " "
              << "(time_increment=" << time_increment << ", scan_time=" << scan_time << ", angle_increment=" << (angle_increment * 180.0 / M_PI) << "). "
              << "Check angle shift settings. Perhaps you should set the parameter time_increment to the expected value. This message will print every 60 seconds.");
        }
#endif
      return false;
    }
    // ROS_DEBUG_STREAM("SickGenericParser::checkScanTiming(time_increment=" << time_increment << ", scan_time=" << scan_time << ", angle_increment=" << (angle_increment*180.0/M_PI)
    //     << "): expected_time_increment=" << expected_time_increment);
    return true;
  };


  /*!
\brief Parsing Ascii datagram
\param datagram: Pointer to datagram data
\param datagram_length: Number of bytes in datagram
\param config: Pointer to Configdata
\param msg: Holds result of Parsing
\param numEchos: Number of DIST-blocks found in message
\param echoMask: Mask corresponding to DIST-block-identifier
\return set_range_max
*/
  int SickGenericParser::parse_datagram(char *datagram, size_t datagram_length, SickScanConfig &config,
                                        ros_sensor_msgs::LaserScan &msg, int &numEchos, int &echoMask)
  {
    // echoMask introduced to get a workaround for cfg bug using MRS1104
    bool dumpData = false;
    int verboseLevel = 0; // for low level debugging only

    int HEADER_FIELDS = 32;
    char *cur_field;
    size_t count;
    int scannerIdx = lookUpForAllowedScanner(getScannerType());

    // Reserve sufficient space
    std::vector<char *> fields;
    fields.reserve(datagram_length / 2);

    // ----- only for debug output
    std::vector<char> datagram_copy_vec;
    datagram_copy_vec.resize(datagram_length + 1); // to avoid using malloc. destructor frees allocated mem.
    char *datagram_copy = &(datagram_copy_vec[0]);

    if (verboseLevel > 0)
    {
      sick_scan_xd::SickScanCommon::dumpDatagramForDebugging((unsigned char *)datagram, datagram_length, true);
    }

    strncpy(datagram_copy, datagram, datagram_length); // datagram will be changed by strtok
    datagram_copy[datagram_length] = 0;

    // ----- tokenize
    count = 0;
    cur_field = strtok(datagram, " ");

    while (cur_field != NULL)
    {
      fields.push_back(cur_field);
      //std::cout << cur_field << std::endl;
      cur_field = strtok(NULL, " ");
    }

    //std::cout << fields[27] << std::endl;

    count = fields.size();


    if (verboseLevel > 0)
    {
      sick_scan_xd::SickScanCommon::dumpDatagramForDebugging((unsigned char *)fields.data(), fields.size(), true);
    }

    // Validate header. Total number of tokens is highly unreliable as this may
    // change when you change the scanning range or the device name using SOPAS ET
    // tool. The header remains stable, however.
    if (count < HEADER_FIELDS)
    {
      ROS_WARN_STREAM("received less fields than minimum fields (actual: " << (int)count << ", minimum: " << (int)HEADER_FIELDS << "), ignoring scan\n");
      ROS_WARN_STREAM("are you using the correct node? (124 --> sick_tim310_1130000m01, > 33 --> sick_tim551_2050001, 580 --> sick_tim310s01, 592 --> sick_tim310)\n");
      // ROS_DEBUG("received message was: %s", datagram_copy);
      return ExitError;
    }

    if (basicParams[scannerIdx].getNumberOfLayers() == 1)
    {
      if (strcmp(fields[15], "0"))
      {
          ROS_WARN_STREAM("Field 15 of received data is not equal to 0 (" << std::string(fields[15]) << "). Unexpected data, ignoring scan\n");
        return ExitError;
      }
    }
    else // fields[15] enthält keine "0"
    {

      //other layers are here on alternate values
      // ROS_WARN("Field 15 of received data is not equal to 0 (%s). Unexpected data, ignoring scan", fields[15]);
      // return ExitError;
    }
    if (strcmp(fields[20], "DIST1"))
    {
      ROS_WARN_STREAM("Field 20 of received data is not equal to DIST1i (" << std::string(fields[20]) << "). Unexpected data, ignoring scan\n");
      return ExitError;
    }

    // More in depth checks: check data length and RSSI availability
    // 25: Number of data (<= 10F)
    unsigned short int number_of_data = 0;
    sscanf(fields[25], "%hx", &number_of_data);

    int numOfExpectedShots = basicParams[scannerIdx].getNumberOfShots();
    if (number_of_data < 1 || number_of_data > numOfExpectedShots)
    {
      ROS_WARN_STREAM("Data length is outside acceptable range 1-" << numOfExpectedShots << " (" << number_of_data << "). Ignoring scan");
      return ExitError;
    }
    if (count < HEADER_FIELDS + number_of_data)
    {
        ROS_WARN_STREAM("Less fields than expected for " << number_of_data << " data points (" << count << "). Ignoring scan");
      return ExitError;
    }
    ROS_DEBUG_STREAM("Number of data: " << number_of_data);

    // Calculate offset of field that contains indicator of whether or not RSSI data is included
    size_t rssi_idx = 26 + number_of_data;
    bool rssi = false;
    if (strcmp(fields[rssi_idx], "RSSI1") == 0)
    {
      rssi = true;
    }
    unsigned short int number_of_rssi_data = 0;
    if (rssi)
    {
      sscanf(fields[rssi_idx + 5], "%hx", &number_of_rssi_data);

      // Number of RSSI data should be equal to number of data
      if (number_of_rssi_data != number_of_data)
      {
        ROS_WARN_STREAM("Number of RSSI data is lower than number of range data (" << number_of_data  << " vs " << number_of_rssi_data  << ")");
        return ExitError;
      }

      // Check if the total length is still appropriate.
      // RSSI data size = number of RSSI readings + 6 fields describing the data
      if (count < HEADER_FIELDS + number_of_data + number_of_rssi_data + 6)
      {
        ROS_WARN_STREAM("Less fields than expected for " << number_of_data << " data points (" << count << "). Ignoring scan");
        return ExitError;
      }

      if (strcmp(fields[rssi_idx], "RSSI1"))
      {
        ROS_WARN_STREAM("Field " << rssi_idx + 1 << " of received data is not equal to RSSI1 (" << fields[rssi_idx + 1] << "). Unexpected data, ignoring scan");
      }
    }

    short layer = 0;
    if (basicParams[scannerIdx].getNumberOfLayers() > 1)
    {
      sscanf(fields[15], "%hx", &layer);
      ROS_HEADER_SEQ(msg.header, layer);
    }
    // ----- read fields into msg
    msg.header.frame_id = config.frame_id;
    // evtl. debug stream benutzen
    // ROS_DEBUG("publishing with frame_id %s", config.frame_id.c_str());

    rosTime start_time = rosTimeNow(); // will be adjusted in the end


    /*

     */
    // <STX> (\x02)
    // 0: Type of command (SN)
    // 1: Command (LMDscandata)
    // 2: Firmware version number (1)
    // 3: Device number (1)
    // 4: Serial number (eg. B96518)
    // 5 + 6: Device Status (0 0 = ok, 0 1 = error)
    // 7: Telegram counter (eg. 99)
    // 8: Scan counter (eg. 9A)
    // 9: Time since startup (eg. 13C8E59)
    // 10: Time of transmission (eg. 13C9CBE)
    // 11 + 12: Input status (0 0), active fieldset
    // 13 + 14: Output status (8 0)
    // 15: Reserved Byte A (0)

    // 16: Scanning Frequency (5DC)
    unsigned short scanning_freq = -1;
    sscanf(fields[16], "%hx", &scanning_freq);
    msg.scan_time = 1.0f / (scanning_freq / 100.0f);
    // ROS_DEBUG("hex: %s, scanning_freq: %d, scan_time: %f", fields[16], scanning_freq, msg.scan_time);

    // 17: Measurement Frequency (36)
    unsigned short measurement_freq = -1;
    sscanf(fields[17], "%hx", &measurement_freq);
    msg.time_increment = 1.0f / (measurement_freq * 100.0f);
    if (override_time_increment_ > 0.0)
    {
      // Some lasers may report incorrect measurement frequency
      msg.time_increment = override_time_increment_;
    }
    // ROS_DEBUG("measurement_freq: %d, time_increment: %f", measurement_freq, msg.time_increment);

    // 18: Number of encoders (0)
    // 19: Number of 16 bit channels (1)
    // 20: Measured data contents (DIST1)

    // 21: Scaling factor (3F800000)
    // ignored for now (is always 1.0):
    //      unsigned int scaling_factor_int = -1;
    //      sscanf(fields[21], "%x", &scaling_factor_int);
    //
    //      float scaling_factor = reinterpret_cast<float&>(scaling_factor_int);
    //      // ROS_DEBUG("hex: %s, scaling_factor_int: %d, scaling_factor: %f", fields[21], scaling_factor_int, scaling_factor);

    // 22: Scaling offset (00000000) -- always 0
    // 23: Starting angle (FFF92230)
    int starting_angle = -1;
    sscanf(fields[23], "%x", &starting_angle);
   

    // 24: Angular step width (2710)
    unsigned short angular_step_width = -1;
    sscanf(fields[24], "%hx", &angular_step_width);

    double startAngleInDeg = (starting_angle / 10000.0);
    double sizeOfSingleAngularStepInDeg = (angular_step_width / 10000.0);

    SickGenericParser tmpParser(basicParams[scannerIdx].getScannerName());
    sick_scan_xd::configureAngleParameters(msg,
      startAngleInDeg,
      sizeOfSingleAngularStepInDeg,
      number_of_data,
      &tmpParser);
    // 25: Number of data (<= 10F)
    // This is already determined above in number_of_data
    int indexMin = 0;
    int distNum = 0;
    int rssiNum = 0;


    checkForDistAndRSSI(fields, number_of_data, distNum, rssiNum, msg.ranges, msg.intensities, echoMask);
    if (config.intensity)
    {
      if (rssiNum > 0)
      {

      }
      else
      {
        ROS_WARN("Intensity parameter is enabled, but the scanner is not configured to send RSSI values! ");
      }
    }
    numEchos = distNum;

    // 26 + n: RSSI data included
    // IF RSSI not included:
    //   26 + n + 1 .. 26 + n + 3 = unknown (but seems to be [0, 1, B] always)
    //   26 + n + 4 .. count - 4 = device label
    //   count - 3 .. count - 1 = unknown (but seems to be 0 always)
    //   <ETX> (\x03)

    msg.range_min = override_range_min_;
    msg.range_max = override_range_max_;

    if (basicParams[scannerIdx].getNumberOfLayers() > 1)
    {
      char szDummy[255] = {0};
      sprintf(szDummy, "%s_%+04d", config.frame_id.c_str(), layer); // msg.header.seq := layer
      msg.header.frame_id = szDummy;
    }
    // ----- adjust start time
    // - last scan point = now  ==>  first scan point = now - number_of_data * time increment
#ifndef _MSC_VER  // TIMING in Simulation not correct
    double duration_sec = number_of_data * msg.time_increment
        + indexMin * msg.time_increment // shift forward to time of first published scan point
        + config.time_offset; // add time offset (to account for USB latency etc.)
    msg.header.stamp = start_time - rosDurationFromSec(duration_sec); // rosDuration().fromSec(number_of_data * msg.time_increment);

#endif
    // ----- consistency check

    this->checkScanTiming(msg.time_increment, msg.scan_time, msg.angle_increment, 0.00001f);
    return ExitSuccess;
  }


  /*!
  \brief Setting minimum range
  \param min range in [m]
  \sa set_range_max
  */
  void SickGenericParser::set_range_min(float min)
  {
    override_range_min_ = min;
  }

  /*!
  \brief Setting maximum range
  \param max range in [m]
  \sa set_range_min
  */
  void SickGenericParser::set_range_max(float max)
  {
    override_range_max_ = max;
  }


  /*!
   \brief Getting maximum range
   \return range in [m]
   \sa set_range_max
   */
  float SickGenericParser::get_range_max(void)
  {
    return (override_range_max_);
  }

  /*!
  \brief Getting minimum range
  \return range in [m]
  \sa set_range_min
  */
  float SickGenericParser::get_range_min(void)
  {
    return (override_range_min_);
  }

  /*!
   \brief Set range filter handling (range filter deactivated, drop, set to nan, etc.pp.)
   */
  void SickGenericParser::set_range_filter_config(RangeFilterResultHandling range_filter_handling)
  {
    m_range_filter_handling = range_filter_handling;
  }

  /*!
   \brief Get range filter handling (range filter deactivated, drop, set to nan, etc.pp.)
   */
  RangeFilterResultHandling SickGenericParser::get_range_filter_config(void) const
  {
    return m_range_filter_handling;
  }

  /*!
\brief setting time increment between shots

\param time increment
*/
  void SickGenericParser::set_time_increment(float time)
  {
    override_time_increment_ = time;
  }

  /*!
\brief getting time increment between shots

\param time increment
*/
  float SickGenericParser::get_time_increment(void)
  {
      return override_time_increment_;
  }

  /*!
  \brief setting scannertype

  \param _scannerType
  \sa getScannerType
  */
  void SickGenericParser::setScannerType(std::string _scannerType)
  {
    scannerType = _scannerType;
  }

  /*!
  \brief getting scannertype

  \return scannerType
  */
  std::string SickGenericParser::getScannerType(void)
  {
    return (scannerType);

  }

} /* namespace sick_scan_xd */
