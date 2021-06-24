/**
* \file
* \brief Laser Scanner Entry Point
*
* Copyright (C) 2020, 2019,2018,2017, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2020, 2019,2018,2017, SICK AG, Waldkirch
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
*  Last modified: 9th June 2020
*
*      Authors:
*         Michael Lehning <michael.lehning@lehning.de>
*         Jochen Sprickerhof <jochen@sprickerhof.de>
*         Martin Günther <mguenthe@uos.de>
*
* Based on the TiM communication example by SICK AG.
*
*
*
*  Copyright 2018/2019 SICK AG
*  Copyright 2018/2019 Ing.-Büro Dr. Michael Lehning



*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sick_scan/sick_generic_laser.h"
#include "sick_scan/dataDumper.h"

#include "sick_scan/helper/angle_compensator.h"
#ifdef _MSC_VER
#include "sick_scan/rosconsole_simu.hpp"
#endif

#define MAX_NAME_LEN (1024)

// 001.001.000 Switch to multithreaded processing of data
// 001.001.001: Documentation added
// 001.002.001: Bug in bin_scanf fixed (number of counted arguments was always 1)
// 001.002.002: MRS1xxx/LMS1xxx - legacy device ident cmd. changed to new device ident cmd
// 001.002.003: MRS1xxx/LMS1xxx - support of hector_slam integrated
// 001.002.004: RMS3xx - profiling and radar support optimized
// 001.002.005: Startup process changed to state machine
// 001.002.006: Signal handler for ctrl-c added
// 001.002.007: Fix for multi echo handling with lookup table
// 001.002.008: IMU Parser structure added
// 001.002.009: Application setting modified for MRS1104
// 001.002.010: First version of IMU parser
// 001.002.011: Ip Address change added, bugfixing
// 001.003.000: Jan 2019  release 0.0.14
// 001.003.001: Jan 2019  release 0.0.14 address handling for ip v4 parsing fixed
// 001.003.002: Feb 2019 Fixing and optimizing console output
// 001.003.016: Feb 2019 Profiling+instructions, Caching of Ros-Params
// 001.003.017: May 2019 stability issues, scan rate and angular resolution settings added
// 001.003.018: May 2019 LMS1000 Min/Max angel settings added and tested
// 001.003.020: May 2019 Bloom process prepared
// 001.003.022: Sep 2019 LMS 4xxx added
// 001.004.000: Oct 2019 ENCODER Support added
// 001.004.001: Nov 2019 IMU Timestamp Fix
// 1.5.0: 2019-11-20: Radar implementation moved to a singleton
// 1.5.1: 2020-03-04: First support of TiM240 prototype
// 1.5.3: 2020-03-19: Fixes for LMS1xx
// 1.5.4: 2020-03-26: Fixes for 16 bit resolution flag
// 1.5.5: 2020-04-01: MRS6xxx check
// 1.5.5: 2020-05-14: NAV 2xx support
// 1.7.0: 2020-06-01: TiM443 added
// 1.7.1: 2020-06-04: NAV 2xx angle correction added
// 1.7.2: 2020-06-09: TiM433 added and launch file info for TiM4xx added
// 1.7.3: 2020-06-10: NAV 3xx angle correction added
// 1.7.4: 2020-06-10: NAV 3xx angle correction improved
// 1.7.5: 2020-06-25: Preparing for Release Noetic
// 1.7.6: 2020-07-14: NAV310 handling optimized (angle calculation and compensation), barebone quaterion to euler
// 1.7.7: 2020-07-21: barebone quaterion to euler
#define SICK_GENERIC_MAJOR_VER "1"
#define SICK_GENERIC_MINOR_VER "10"
#define SICK_GENERIC_PATCH_LEVEL "1"

#include <algorithm> // for std::min


std::string getVersionInfo();

/*!
\brief Startup routine - if called with no argmuments we assume debug session.
       Set scanner name variable by parsing for "__name:=". This will be changed in the future
	   by setting a parameter. Calls mainGenericLaser after parsing.

\param argc: Number of Arguments
\param argv: Argument variable
\return exit-code
\sa mainGenericLaser
*/
int main(int argc, char **argv)
{

  DataDumper::instance().writeToFileNameWhenBufferIsFull("/tmp/sickscan_debug.csv");
  char nameId[] = "__name:=";
  char nameVal[MAX_NAME_LEN] = {0};
  char **argv_tmp; // argv_tmp[0][0] argv_tmp[0] identisch ist zu (*argv_tmp)
  int argc_tmp;
  std::string scannerName = "????";

  // sick_scan::SickScanImu::imuParserTest();

  argc_tmp = argc;
  argv_tmp = argv;

  const int MAX_STR_LEN = 1024;
  char nameTagVal[MAX_STR_LEN] = {0};
  char logTagVal[MAX_STR_LEN] = {0};
  char internalDebugTagVal[MAX_STR_LEN] = {0};
  char sensorEmulVal[MAX_STR_LEN] = {0};

  if (argc == 1) // just for testing without calling by roslaunch
  {
    // recommended call for internal debugging as an example: __name:=sick_rms_320 __internalDebug:=1
    // strcpy(nameTagVal, "__name:=sick_rms_3xx");  // sick_rms_320 -> radar
    strcpy(nameTagVal, "__name:=sick_tim_5xx");  // sick_rms_320 -> radar
    strcpy(logTagVal, "__log:=/tmp/tmp.log");
    strcpy(internalDebugTagVal, "__internalDebug:=1");
    // strcpy(sensorEmulVal, "__emulSensor:=1");
    strcpy(sensorEmulVal, "__emulSensor:=0");
    argc_tmp = 5;
    argv_tmp = (char **) malloc(sizeof(char *) * argc_tmp);

    argv_tmp[0] = argv[0];
    argv_tmp[1] = nameTagVal;
    argv_tmp[2] = logTagVal;
    argv_tmp[3] = internalDebugTagVal;
    argv_tmp[4] = sensorEmulVal;

  }
  //
  std::string versionInfo = "sick_generic_caller V. ";
  versionInfo += std::string(SICK_GENERIC_MAJOR_VER) + '.';
  versionInfo += std::string(SICK_GENERIC_MINOR_VER) + '.';
  versionInfo += std::string(SICK_GENERIC_PATCH_LEVEL);

  setVersionInfo(versionInfo);
  ROS_INFO("%s", versionInfo.c_str());
  for (int i = 0; i < argc_tmp; i++)
  {
    if (strstr(argv_tmp[i], nameId) == argv_tmp[i])
    {
      strcpy(nameVal, argv_tmp[i] + strlen(nameId));
      scannerName = nameVal;
    }
    ROS_INFO("Program arguments: %s", argv_tmp[i]);
  }

  int result = mainGenericLaser(argc_tmp, argv_tmp, scannerName);
  return result;

}
