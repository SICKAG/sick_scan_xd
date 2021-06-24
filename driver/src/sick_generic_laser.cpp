/**
* \file
* \brief Laser Scanner Main Handling
* Copyright (C) 2013,     Osnabrück University
* Copyright (C) 2017,2018 Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2017,2018 SICK AG, Waldkirch
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
*  Last modified: 21st Aug 2018
*
*      Authors:
*         Michael Lehning <michael.lehning@lehning.de>
*         Jochen Sprickerhof <jochen@sprickerhof.de>
*         Martin Günther <mguenthe@uos.de>
*
*
*/

#ifdef _MSC_VER
#define _WIN32_WINNT 0x0501
#pragma warning(disable: 4996)
#pragma warning(disable: 4267)
#endif

#ifndef _MSC_VER


#endif

#include <sick_scan/sick_scan_common_tcp.h>

#include <sick_scan/sick_generic_parser.h>
#include <sick_scan/sick_generic_laser.h>
#include <sick_scan/sick_scan_services.h>


#ifdef _MSC_VER
#include "sick_scan/rosconsole_simu.hpp"
#endif
#define _USE_MATH_DEFINES

#include <math.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

static bool isInitialized = false;
static sick_scan::SickScanCommonTcp *s = NULL;
static std::string versionInfo = "???";

void setVersionInfo(std::string _versionInfo)
{
  versionInfo = _versionInfo;
}

std::string getVersionInfo()
{

  return (versionInfo);
}

enum NodeRunState
{
  scanner_init, scanner_run, scanner_finalize
};

NodeRunState runState = scanner_init;  //


/*!
\brief splitting expressions like <tag>:=<value> into <tag> and <value>
\param [In] tagVal: string expression like <tag>:=<value>
\param [Out] tag: Tag after Parsing
\param [Ozt] val: Value after Parsing
\return Result of matching process (true: matching expression found, false: no match found)
*/

bool getTagVal(std::string tagVal, std::string &tag, std::string &val)
{
  bool ret = false;
  std::size_t pos;
  pos = tagVal.find(":=");
  tag = "";
  val = "";
  if (pos == std::string::npos)
  {
    ret = false;
  }
  else
  {
    tag = tagVal.substr(0, pos);
    val = tagVal.substr(pos + 2);
    ret = true;
  }
  return (ret);
}


void my_handler(int signalRecv)
{
  ROS_INFO("Caught signal %d\n", signalRecv);
  ROS_INFO("good bye");
  ROS_INFO("You are leaving the following version of this node:");
  ROS_INFO("%s", getVersionInfo().c_str());
  if (s != NULL)
  {
    if (isInitialized)
    {
      s->stopScanData();
    }

    runState = scanner_finalize;
  }
  ros::shutdown();
}

/*!
\brief Internal Startup routine.
\param argc: Number of Arguments
\param argv: Argument variable
\param nodeName name of the ROS-node
\return exit-code
\sa main
*/
int mainGenericLaser(int argc, char **argv, std::string nodeName)
{
  std::string tag;
  std::string val;


  bool doInternalDebug = false;
  bool emulSensor = false;
  for (int i = 0; i < argc; i++)
  {
    std::string s = argv[i];
    if (getTagVal(s, tag, val))
    {
      if (tag.compare("__internalDebug") == 0)
      {
        int debugState = 0;
        sscanf(val.c_str(), "%d", &debugState);
        if (debugState > 0)
        {
          doInternalDebug = true;
        }
      }
      if (tag.compare("__emulSensor") == 0)
      {
        int dummyState = 0;
        sscanf(val.c_str(), "%d", &dummyState);
        if (dummyState > 0)
        {
          emulSensor = true;
        }
      }
    }
  }

  ros::init(argc, argv, nodeName, ros::init_options::NoSigintHandler);  // scannerName holds the node-name
  signal(SIGINT, my_handler);

  ros::NodeHandle nhPriv("~");


  std::string scannerName;
  if (false == nhPriv.getParam("scanner_type", scannerName))
  {
    ROS_ERROR("cannot find parameter ""scanner_type"" in the param set. Please specify scanner_type.");
    ROS_ERROR("Try to set %s as fallback.\n", nodeName.c_str());
    scannerName = nodeName;
  }


  if (doInternalDebug)
  {
#ifdef _MSC_VER
    nhPriv.setParam("name", scannerName);
    rossimu_settings(nhPriv);  // just for tiny simulations under Visual C++
#else
    nhPriv.setParam("hostname", "192.168.0.4");
    nhPriv.setParam("imu_enable", true);
    nhPriv.setParam("cloud_topic", "pt_cloud");
#endif
  }

// check for TCP - use if ~hostname is set.
  bool useTCP = false;
  std::string hostname;
  if (nhPriv.getParam("hostname", hostname))
  {
    useTCP = true;
  }
  bool changeIP = false;
  std::string sNewIp;
  if (nhPriv.getParam("new_IP_address", sNewIp))
  {
    changeIP = true;
  }
  std::string port;
  nhPriv.param<std::string>("port", port, "2112");

  int timelimit;
  nhPriv.param("timelimit", timelimit, 5);

  bool subscribe_datagram;
  int device_number;
  nhPriv.param("subscribe_datagram", subscribe_datagram, false);
  nhPriv.param("device_number", device_number, 0);


  sick_scan::SickGenericParser *parser = new sick_scan::SickGenericParser(scannerName);

  double param;
  char colaDialectId = 'A'; // A or B (Ascii or Binary)

  if (nhPriv.getParam("range_min", param))
  {
    parser->set_range_min(param);
  }
  if (nhPriv.getParam("range_max", param))
  {
    parser->set_range_max(param);
  }
  if (nhPriv.getParam("time_increment", param))
  {
    parser->set_time_increment(param);
  }

  /*
   *  Check, if parameter for protocol type is set
   */
  bool use_binary_protocol = true;
  if (true == nhPriv.getParam("emul_sensor", emulSensor))
  {
    ROS_INFO("Found emul_sensor overwriting default settings. Emulation: %s", emulSensor ? "True" : "False");
  }
  if (true == nhPriv.getParam("use_binary_protocol", use_binary_protocol))
  {
    ROS_INFO("Found sopas_protocol_type param overwriting default protocol:");
    if (use_binary_protocol == true)
    {
      ROS_INFO("Binary protocol activated");
    }
    else
    {
      if (parser->getCurrentParamPtr()->getNumberOfLayers() > 4)
      {
        nhPriv.setParam("sopas_protocol_type", true);
        use_binary_protocol = true;
        ROS_WARN("This scanner type does not support ASCII communication.\n"
                 "Binary communication has been activated.\n"
                 "The parameter \"sopas_protocol_type\" has been set to \"True\".");
      }
      else
      {
        ROS_INFO("ASCII protocol activated");
      }
    }
    parser->getCurrentParamPtr()->setUseBinaryProtocol(use_binary_protocol);
  }


  if (parser->getCurrentParamPtr()->getUseBinaryProtocol())
  {
    colaDialectId = 'B';
  }
  else
  {
    colaDialectId = 'A';
  }

  bool start_services = false;
  sick_scan::SickScanServices* services = 0;
  int result = sick_scan::ExitError;

  sick_scan::SickScanConfig cfg;

  while (ros::ok())
  {
    switch (runState)
    {
      case scanner_init:
        ROS_INFO("Start initialising scanner [Ip: %s] [Port: %s]", hostname.c_str(), port.c_str());
        // attempt to connect/reconnect
        delete s;  // disconnect scanner
        if (useTCP)
        {
          s = new sick_scan::SickScanCommonTcp(hostname, port, timelimit, parser, colaDialectId);
        }
        else
        {
          ROS_ERROR("TCP is not switched on. Probably hostname or port not set. Use roslaunch to start node.");
          exit(-1);
        }


        if (emulSensor)
        {
          s->setEmulSensor(true);
        }
        result = s->init();

        // Start ROS services
        if (true == nhPriv.getParam("start_services", start_services) && true == start_services)
        {
            services = new sick_scan::SickScanServices(&nhPriv, s, parser->getCurrentParamPtr()->getUseBinaryProtocol());
            ROS_INFO("SickScanServices: ros services initialized");
        }

        isInitialized = true;
        signal(SIGINT, SIG_DFL); // change back to standard signal handler after initialising
        if (result == sick_scan::ExitSuccess) // OK -> loop again
        {
          if (changeIP)
          {
            runState = scanner_finalize;
          }


          runState = scanner_run; // after initialising switch to run state
        }
        else
        {
          runState = scanner_init; // If there was an error, try to restart scanner

        }
        break;

      case scanner_run:
        if (result == sick_scan::ExitSuccess) // OK -> loop again
        {
          ros::spinOnce();
          result = s->loopOnce();
        }
        else
        {
          runState = scanner_finalize; // interrupt
        }
      case scanner_finalize:
        break; // ExitError or similiar -> interrupt while-Loop
      default:
        ROS_ERROR("Invalid run state in main loop");
        break;
    }
  }
  if(services)
  {
    delete services;
    services = 0;
  }
  if (s != NULL)
  {
    delete s; // close connnect
  }
  if (parser != NULL)
  {
    delete parser; // close parser
  }
  return result;

}