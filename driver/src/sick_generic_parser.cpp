/**
* \file
* \brief Laser Scanner Parser
* Copyright (C) 2013, Osnabrück University
* Copyright (C) 2017, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2017, SICK AG, Waldkirch
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

#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include <math.h>
#include <sick_scan/sick_scan_common.h>
#include <ros/ros.h>

#ifdef _MSC_VER
#include "sick_scan/rosconsole_simu.hpp"
#endif

namespace sick_scan
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
  std::string ScannerBasicParam::getScannerName()
  {
    return (scannerName);
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

  /*!
  \brief flag to mark the device as radar (instead of laser scanner)
  \param _deviceIsRadar: false for laserscanner, true for radar (like rms_3xx)
  \sa getDeviceIsRadar
  */
  void ScannerBasicParam::setDeviceIsRadar(bool _deviceIsRadar)
  {
    deviceIsRadar = _deviceIsRadar;
  }

  /*!
  \brief flag to mark the device as radar (instead of laser scanner)
  \param _deviceIsRadar: false for laserscanner, true for radar (like rms_3xx)
  \sa getDeviceIsRadar
  */
  bool ScannerBasicParam::getDeviceIsRadar(void)
  {
    return (deviceIsRadar);
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


  /*!
  \brief Construction of parameter object

  */
  ScannerBasicParam::ScannerBasicParam()
  : numberOfLayers(0), numberOfShots(0), numberOfMaximumEchos(0), elevationDegreeResolution(0), angleDegressResolution(0), expectedFrequency(0),
     useBinaryProtocol(false), IntensityResolutionIs16Bit(false), deviceIsRadar(false), useSafetyPasWD(false), encoderMode(0),
     CartographerCompatibility(false), scanMirroredAndShifted(false), useEvalFields(EVAL_FIELD_UNSUPPORTED), maxEvalFields(0)
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
    allowedScannerNames.push_back(SICK_SCANNER_RMS_3XX_NAME); // Radar scanner
    allowedScannerNames.push_back(SICK_SCANNER_NAV_3XX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_NAV_2XX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_TIM_4XX_NAME);
    basicParams.resize(allowedScannerNames.size()); // resize to number of supported scanner types
    for (int i = 0; i <
                    (int) basicParams.size(); i++) // set specific parameter for each scanner type - scanner type is identified by name
    {
      basicParams[i].setDeviceIsRadar(false); // Default
      basicParams[i].setScannerName(allowedScannerNames[i]);  // set scanner type for this parameter object

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
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
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
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
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
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
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
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_LMS_4XXX_NAME) == 0) // LMS_4xxx - 1 Layer, 600 Hz
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(841);//
        basicParams[i].setAngularDegreeResolution(0.0833);//
        basicParams[i].setExpectedFrequency(600.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_TIM_7XX_NAME) == 0) // TIM_7xx - 1 Layer Scanner
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(811);
        basicParams[i].setAngularDegreeResolution(0.3333);
        basicParams[i].setExpectedFrequency(15.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(USE_EVAL_FIELD_TIM7XX_LOGIC);
        basicParams[i].setMaxEvalFields(48);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_TIM_7XXS_NAME) == 0) // TIM_7xxS - 1 layer Safety Scanner
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(811);
        basicParams[i].setAngularDegreeResolution(0.3333);
        basicParams[i].setExpectedFrequency(15.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(true); // Safety scanner
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(USE_EVAL_FIELD_TIM7XX_LOGIC);
        basicParams[i].setMaxEvalFields(48);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_LMS_5XX_NAME) == 0) // LMS_5xx - 1 Layer
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(381);
        basicParams[i].setAngularDegreeResolution(0.5);
        basicParams[i].setExpectedFrequency(15.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(USE_EVAL_FIELD_LMS5XX_LOGIC);
        basicParams[i].setMaxEvalFields(30);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_LMS_1XX_NAME) == 0) // LMS_1xx - 1 Layer
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(1081);
        basicParams[i].setAngularDegreeResolution(0.5);
        basicParams[i].setExpectedFrequency(25.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(USE_EVAL_FIELD_LMS5XX_LOGIC);
        basicParams[i].setMaxEvalFields(30);
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
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
      }

      if (basicParams[i].getScannerName().compare(SICK_SCANNER_RMS_3XX_NAME) == 0) // Radar
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(0); // for radar scanner
        basicParams[i].setNumberOfShots(65);
        basicParams[i].setAngularDegreeResolution(0.00);
        basicParams[i].setElevationDegreeResolution(0.00); // in [degree]
        basicParams[i].setExpectedFrequency(0.00);
        basicParams[i].setUseBinaryProtocol(false); // use ASCII-Protocol
        basicParams[i].setDeviceIsRadar(true); // Device is a radar
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_NAV_3XX_NAME) == 0) // Nav 3xx
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(2880);
        basicParams[i].setAngularDegreeResolution(0.750);
        basicParams[i].setExpectedFrequency(55.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setScanMirroredAndShifted(true);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_NAV_2XX_NAME) == 0) // NAV_2xx - 1 Layer
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(1081);
        basicParams[i].setAngularDegreeResolution(0.5);
        basicParams[i].setExpectedFrequency(25.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_TIM_4XX_NAME) == 0) // TiM433 and TiM443
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(721);
        basicParams[i].setAngularDegreeResolution(0.33333333333);
        basicParams[i].setExpectedFrequency(15.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
        basicParams[i].setScanMirroredAndShifted(false);
        basicParams[i].setUseEvalFields(EVAL_FIELD_UNSUPPORTED);
        basicParams[i].setMaxEvalFields(0);
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
      ROS_WARN("Field 20 of received data does not start with DIST (is: %s). Unexpected data, ignoring scan",
               fields[20]);
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
            range = iRange / 1000.0;
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


  void SickGenericParser::checkScanTiming(float time_increment, float scan_time, float angle_increment, float tol)
  {
    if (this->getCurrentParamPtr()->getNumberOfLayers() > 1)
    {
      return;
    }

    float expected_time_increment =
        fabs(this->getCurrentParamPtr()->getNumberOfLayers() * scan_time * angle_increment / (2.0 * M_PI));//If the direction of rotation is reversed, i.e. negative angle increment, a negative scan time results. This does not makes sense, therefore the absolute value is calculated.
    if (fabs(expected_time_increment - time_increment) > 0.00001)
    {
      ROS_WARN_THROTTLE(60,
                        "The time_increment, scan_time and angle_increment values reported by the scanner are inconsistent! "
                        "Expected time_increment: %.9f, reported time_increment: %.9f. "
                        "Perhaps you should set the parameter time_increment to the expected value. This message will print every 60 seconds.",
                        expected_time_increment, time_increment);
    }
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
                                        sensor_msgs::LaserScan &msg, int &numEchos, int &echoMask)
  {
    // echoMask introduced to get a workaround for cfg bug using MRS1104
		// ros::NodeHandle tmpParam("~");
    bool dumpData = false;
    int verboseLevel = 0;
		// tmpParam.getParam("verboseLevel", verboseLevel);

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
      ROS_WARN("Verbose LEVEL activated. Only for DEBUG.");
    }

    if (verboseLevel > 0)
    {
      static int cnt = 0;
      char szDumpFileName[511] = {0};
      char szDir[255] = {0};
#ifdef _MSC_VER
      strcpy(szDir,"C:\\temp\\");
#else
      strcpy(szDir, "/tmp/");
#endif
      sprintf(szDumpFileName, "%stmp%06d.bin", szDir, cnt);
      bool isBinary = this->getCurrentParamPtr()->getUseBinaryProtocol();
      if (isBinary)
      {
        FILE *ftmp;
        ftmp = fopen(szDumpFileName, "wb");
        if (ftmp != NULL)
        {
          fwrite(datagram, datagram_length, 1, ftmp);
          fclose(ftmp);
        }
      }
      cnt++;
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
      static int cnt = 0;
      char szDumpFileName[511] = {0};
      char szDir[255] = {0};
#ifdef _MSC_VER
      strcpy(szDir,"C:\\temp\\");
#else
      strcpy(szDir, "/tmp/");
#endif
      sprintf(szDumpFileName, "%stmp%06d.txt", szDir, cnt);
      ROS_WARN("Verbose LEVEL activated. Only for DEBUG.");
      FILE *ftmp;
      ftmp = fopen(szDumpFileName, "w");
      if (ftmp != NULL)
      {
        int i;
        for (i = 0; i < count; i++)
        {
          fprintf(ftmp, "%3d: %s\n", i, fields[i]);
        }
        fclose(ftmp);
      }
      cnt++;
    }

    // Validate header. Total number of tokens is highly unreliable as this may
    // change when you change the scanning range or the device name using SOPAS ET
    // tool. The header remains stable, however.
    if (count < HEADER_FIELDS)
    {
      ROS_WARN(
          "received less fields than minimum fields (actual: %d, minimum: %d), ignoring scan", (int) count,
          HEADER_FIELDS);
      ROS_WARN(
          "are you using the correct node? (124 --> sick_tim310_1130000m01, > 33 --> sick_tim551_2050001, 580 --> sick_tim310s01, 592 --> sick_tim310)");
      // ROS_DEBUG("received message was: %s", datagram_copy);
      return ExitError;
    }

    if (basicParams[scannerIdx].getNumberOfLayers() == 1)
    {
      if (strcmp(fields[15], "0"))
      {
        ROS_WARN("Field 15 of received data is not equal to 0 (%s). Unexpected data, ignoring scan", fields[15]);
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
      ROS_WARN("Field 20 of received data is not equal to DIST1i (%s). Unexpected data, ignoring scan", fields[20]);
      return ExitError;
    }

    // More in depth checks: check data length and RSSI availability
    // 25: Number of data (<= 10F)
    unsigned short int number_of_data = 0;
    sscanf(fields[25], "%hx", &number_of_data);

    int numOfExpectedShots = basicParams[scannerIdx].getNumberOfShots();
    if (number_of_data < 1 || number_of_data > numOfExpectedShots)
    {
      ROS_WARN("Data length is outside acceptable range 1-%d (%d). Ignoring scan", numOfExpectedShots, number_of_data);
      return ExitError;
    }
    if (count < HEADER_FIELDS + number_of_data)
    {
      ROS_WARN("Less fields than expected for %d data points (%zu). Ignoring scan", number_of_data, count);
      return ExitError;
    }
    ROS_DEBUG("Number of data: %d", number_of_data);

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
        ROS_WARN("Number of RSSI data is lower than number of range data (%d vs %d", number_of_data,
                 number_of_rssi_data);
        return ExitError;
      }

      // Check if the total length is still appropriate.
      // RSSI data size = number of RSSI readings + 6 fields describing the data
      if (count < HEADER_FIELDS + number_of_data + number_of_rssi_data + 6)
      {
        ROS_WARN("Less fields than expected for %d data points (%zu). Ignoring scan", number_of_data, count);
        return ExitError;
      }

      if (strcmp(fields[rssi_idx], "RSSI1"))
      {
        ROS_WARN("Field %zu of received data is not equal to RSSI1 (%s). Unexpected data, ignoring scan", rssi_idx + 1,
                 fields[rssi_idx + 1]);
      }
    }

    if (basicParams[scannerIdx].getNumberOfLayers() > 1)
    {
      short layer = -1;
      sscanf(fields[15], "%hx", &layer);
      msg.header.seq = layer;
    }
    // ----- read fields into msg
    msg.header.frame_id = config.frame_id;
    // evtl. debug stream benutzen
    // ROS_DEBUG("publishing with frame_id %s", config.frame_id.c_str());

    ros::Time start_time = ros::Time::now(); // will be adjusted in the end


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
    /*
    unsigned short u16_active_fieldset = 0;
    if(sscanf(fields[12], "%hx", &u16_active_fieldset) == 1)
    {
        SickScanFieldMonSingleton *fieldMon = SickScanFieldMonSingleton::getInstance();
        if(fieldMon)
        {
          fieldMon->setActiveFieldset(u16_active_fieldset & 0xFF);
          ROS_INFO("Scandata: active_fieldset = %d", fieldMon->getActiveFieldset());
        }
    }
    */
    // 13 + 14: Output status (8 0)
    // 15: Reserved Byte A (0)

    // 16: Scanning Frequency (5DC)
    unsigned short scanning_freq = -1;
    sscanf(fields[16], "%hx", &scanning_freq);
    msg.scan_time = 1.0 / (scanning_freq / 100.0);
    // ROS_DEBUG("hex: %s, scanning_freq: %d, scan_time: %f", fields[16], scanning_freq, msg.scan_time);

    // 17: Measurement Frequency (36)
    unsigned short measurement_freq = -1;
    sscanf(fields[17], "%hx", &measurement_freq);
    msg.time_increment = 1.0 / (measurement_freq * 100.0);
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
    msg.angle_min = (starting_angle / 10000.0) / 180.0 * M_PI - M_PI / 2;
    // ROS_DEBUG("starting_angle: %d, angle_min: %f", starting_angle, msg.angle_min);

    // 24: Angular step width (2710)
    unsigned short angular_step_width = -1;
    sscanf(fields[24], "%hx", &angular_step_width);
    msg.angle_increment = (angular_step_width / 10000.0) / 180.0 * M_PI;
    msg.angle_max = msg.angle_min + (number_of_data - 1) * msg.angle_increment;

    // 25: Number of data (<= 10F)
    // This is already determined above in number_of_data
    int index_min = 0;

#if 1  // neuer Ansatz
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
        ROS_WARN_ONCE("Intensity parameter is enabled, but the scanner is not configured to send RSSI values! "
                      "Please read the section 'Enabling intensity (RSSI) output' here: http://wiki.ros.org/sick_tim.");
      }
    }
    numEchos = distNum;
#endif
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
      sprintf(szDummy, "%s_%+04d", config.frame_id.c_str(), msg.header.seq);
      msg.header.frame_id = szDummy;
    }
    // ----- adjust start time
    // - last scan point = now  ==>  first scan point = now - number_of_data * time increment
#ifndef _MSC_VER  // TIMING in Simulation not correct
    msg.header.stamp = start_time - ros::Duration().fromSec(number_of_data * msg.time_increment);

    // - shift forward to time of first published scan point
    msg.header.stamp += ros::Duration().fromSec((double) index_min * msg.time_increment);

    // - add time offset (to account for USB latency etc.)
    msg.header.stamp += ros::Duration().fromSec(config.time_offset);
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
\brief setting time increment between shots

\param time increment
*/
  void SickGenericParser::set_time_increment(float time)
  {
    override_time_increment_ = time;
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

} /* namespace sick_scan */
