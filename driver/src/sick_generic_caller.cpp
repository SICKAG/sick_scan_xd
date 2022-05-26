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
*  Copyright 2018/2019/2020/2021 SICK AG
*  Copyright 2018/2019/2020/2021 Ing.-Büro Dr. Michael Lehning



*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include "sick_scan/sick_generic_laser.h"
#include "sick_scan/dataDumper.h"
#include "sick_scan/helper/angle_compensator.h"

#define MAX_NAME_LEN (1024)

#define SICK_GENERIC_MAJOR_VER "2"
#define SICK_GENERIC_MINOR_VER "6"
#define SICK_GENERIC_PATCH_LEVEL "7"

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

#if defined __ROS_VERSION && __ROS_VERSION == 2
  // Pass command line arguments to rclcpp.
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  //node_options.automatically_declare_initial_parameters(true);
  rosNodePtr node = rclcpp::Node::make_shared("sick_scan", "", node_options);
#else
  ros::init(argc, argv, scannerName, ros::init_options::NoSigintHandler);  // scannerName holds the node-name
  // signal(SIGINT, rosSignalHandler);
  ros::NodeHandle nh("~");
  rosNodePtr node = &nh;
#endif
  signal(SIGINT, rosSignalHandler);

  ROS_INFO_STREAM(versionInfo << "\n");
  for (int i = 0; i < argc_tmp; i++)
  {
    if (strstr(argv_tmp[i], nameId) == argv_tmp[i])
    {
      strcpy(nameVal, argv_tmp[i] + strlen(nameId));
      scannerName = nameVal;
    }
    ROS_INFO_STREAM("Program argument " << (i+1) << ": " << argv_tmp[i] << "\n");
  }

  int result = 0;
  try
  {
    // result = mainGenericLaser(argc_tmp, argv_tmp, scannerName, node);
    if (!startGenericLaser(argc_tmp, argv_tmp, scannerName, node, &result))
    {
      ROS_ERROR_STREAM("## ERROR in sick_generic_caller::main(): startGenericLaser() failed, could not start generic laser event loop");
    }
    else
    {
      rosSpin(node);
    }
    stopScannerAndExit();
  }
  catch(const std::exception& e)
  {
    ROS_ERROR_STREAM("## ERROR in ick_generic_caller::main(): exception " << e.what());
  }
  catch(...)
  {
    ROS_ERROR_STREAM("## ERROR in ick_generic_caller::main(): unknown exception ");
  }

  return result;

}
