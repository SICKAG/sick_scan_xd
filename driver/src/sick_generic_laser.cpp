/**
* \file
* \brief Laser Scanner Main Handling
* Copyright (C) 2013,     Osnabrueck University
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
//#define _WIN32_WINNT 0x0501
#pragma warning(disable: 4996)
#pragma warning(disable: 4267)
#endif

#include <sick_scan/sick_ros_wrapper.h>
#if defined LDMRS_SUPPORT && LDMRS_SUPPORT > 0
#include <sick_scan/ldmrs/sick_ldmrs_node.h>
#endif
#if defined SCANSEGMENT_XD_SUPPORT && SCANSEGMENT_XD_SUPPORT > 0
#include "sick_scansegment_xd/scansegment_threads.h"
#endif
#include <sick_scan/sick_scan_common_tcp.h>
#include <sick_scan/sick_generic_parser.h>
#include <sick_scan/sick_generic_laser.h>
#include <sick_scan/sick_scan_services.h>
#include <sick_scan/sick_generic_monitoring.h>

#include "launchparser.h"
#if __ROS_VERSION != 1 // launchparser for native Windows/Linux and ROS-2
#define USE_LAUNCHPARSER // settings and parameter by LaunchParser
#endif

#define _USE_MATH_DEFINES

#include <math.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#define SICK_GENERIC_MAJOR_VER "3"
#define SICK_GENERIC_MINOR_VER "1"
#define SICK_GENERIC_PATCH_LEVEL "0"

#define DELETE_PTR(p) if(p){delete(p);p=0;}

static bool isInitialized = false;
static sick_scan_xd::SickScanCommonTcp *s_scanner = NULL;
static std::string versionInfo = std::string(SICK_GENERIC_MAJOR_VER) + '.' + std::string(SICK_GENERIC_MINOR_VER) + '.' + std::string(SICK_GENERIC_PATCH_LEVEL);
static bool s_shutdownSignalReceived = false;

void setVersionInfo(std::string _versionInfo)
{
  versionInfo = _versionInfo;
}

std::string getVersionInfo()
{

  return (versionInfo);
}

void mainGenericLaserInternal(int argc, char **argv, std::string nodeName, rosNodePtr nhPriv, bool do_ros_spin, int & exit_code);

class GenericLaserCallable
{
public:
  GenericLaserCallable(int _argc, char** _argv, std::string _nodeName, rosNodePtr _nhPriv, int* _exit_code)
  : argc(_argc), argv(_argv), nodeName(_nodeName), nhPriv(_nhPriv), exit_code(_exit_code)
  {
    generic_laser_thread = new std::thread(&GenericLaserCallable::mainGenericLaserCb, this);
  }
  void mainGenericLaserCb(void)
  {
    mainGenericLaserInternal(argc, argv, nodeName, nhPriv, false, *exit_code);
  }
  void join(void)
  {
    if(generic_laser_thread && generic_laser_thread->joinable())
    {
      generic_laser_thread->join();
    }
  }
  int argc;
  char** argv;
  std::string nodeName;
  rosNodePtr nhPriv;
  int* exit_code;
  std::thread* generic_laser_thread;
};

static GenericLaserCallable* s_generic_laser_thread = 0;

static NodeRunState runState = scanner_init;

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

bool stopScannerAndExit(bool force_immediate_shutdown)
{
  bool success = true;
  if (s_scanner != NULL)
  {
    if (isInitialized)
    {
      success = s_scanner->stopScanData(force_immediate_shutdown);
    }
    runState = scanner_finalize;
  }
  joinGenericLaser();
  return success;
}

bool shutdownSignalReceived()
{
 return s_shutdownSignalReceived;
}

void rosSignalHandler(int signalRecv)
{
  ROS_INFO_STREAM("Caught signal " << signalRecv << "\n");
  ROS_INFO_STREAM("good bye\n");
  ROS_INFO_STREAM("You are leaving the following version of this node:\n");
  ROS_INFO_STREAM(getVersionInfo() << "\n");
  s_shutdownSignalReceived = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  stopScannerAndExit(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "rosSignalHandler exit." << std::endl;
  rosShutdown();
}


inline bool ends_with(std::string const &value, std::string const &ending)
{
  if (ending.size() > value.size())
  { return false; }
  return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

/*!
\brief Parses an optional launchfile and sets all parameters.
       This function is used at startup to enable system independant parameter handling
       for native Linux/Windows, ROS-1 and ROS-2. Parameter are overwritten by optional
       commandline arguments
\param argc: Number of commandline arguments
\param argv: commandline arguments
\param nodeName name of the ROS-node
\return exit-code
\sa main
*/
bool parseLaunchfileSetParameter(rosNodePtr nhPriv, int argc, char **argv)
{
  std::string tag;
  std::string val;
  int launchArgcFileIdx = -1;
  for (int n = 1; n < argc; n++)
  {
    std::string extKey = ".launch";
    std::string argv_str = argv[n];
    if (ends_with(argv_str, extKey))
    {
      launchArgcFileIdx = n;
      std::vector<std::string> tagList, typeList, valList;
      LaunchParser launchParser;
      bool ret = launchParser.parseFile(argv_str, tagList, typeList, valList);
      if (ret == false)
      {
        ROS_INFO_STREAM("Cannot parse launch file (check existence and content): >>>" << argv_str << "<<<\n");
        exit(-1);
      }
      for (size_t i = 0; i < tagList.size(); i++)
      {
        printf("%-30s %-10s %-20s\n", tagList[i].c_str(), typeList[i].c_str(), valList[i].c_str());
        if(typeList[i] == "bool" && !valList[i].empty())
          rosSetParam(nhPriv, tagList[i], (bool)(valList[i][0] == '1' || valList[i][0] == 't' || valList[i][0] == 'T'));
        else if(typeList[i] == "int" && !valList[i].empty())
          rosSetParam(nhPriv, tagList[i], (int)std::stoi(valList[i]));
        else if(typeList[i] == "float" && !valList[i].empty())
          rosSetParam(nhPriv, tagList[i], (float)std::stof(valList[i]));
        else if(typeList[i] == "double" && !valList[i].empty())
          rosSetParam(nhPriv, tagList[i], (double)std::stod(valList[i]));
        else // parameter type "string"
          rosSetParam(nhPriv, tagList[i], valList[i]);
      }
    }
  }

  for (int n = 1; n < argc; n++)
  {
    std::string argv_str = argv[n];

    // Ignore all arguments after and including --ros-args
    if (argv_str == "--ros-args") {
      break;
    }

    if (getTagVal(argv_str, tag, val))
    {
        rosSetParam(nhPriv, tag, val);
    }
    else
    {
      if (launchArgcFileIdx != n)
      {
          ROS_ERROR_STREAM("## ERROR parseLaunchfileSetParameter(): Tag-Value setting not valid. Use pattern: <tag>:=<value>  (e.g. hostname:=192.168.0.4) (Check the entry: " << argv_str << ")\n");
          return false;
      }
    }
  }
  return true;
}

/*!
\brief Internal Startup routine.
\param argc: Number of Arguments
\param argv: Argument variable
\param nodeName name of the ROS-node
\param nhPriv ros node handle
\param exit_code exit-code
\sa main
*/
void mainGenericLaserInternal(int argc, char **argv, std::string nodeName, rosNodePtr nhPriv, bool do_ros_spin, int & exit_code)
{
  std::string tag;
  std::string val;

  exit_code = sick_scan_xd::ExitSuccess;
  bool doInternalDebug = false;
  bool emulSensor = false;
  for (int i = 0; i < argc; i++)
  {
    std::string argv_str = argv[i];
    if (getTagVal(argv_str, tag, val))
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

#ifdef USE_LAUNCHPARSER
  if(!parseLaunchfileSetParameter(nhPriv, argc, argv))
  {
    ROS_ERROR_STREAM("## ERROR sick_generic_laser: parseLaunchfileSetParameter() failed, aborting\n");
    exit_code = sick_scan_xd::ExitError;
    exit(-1);
  }
#endif

  std::string scannerName;
  rosDeclareParam(nhPriv, "scanner_type", scannerName);
  rosGetParam(nhPriv, "scanner_type", scannerName);
  if (false == rosGetParam(nhPriv, "scanner_type", scannerName) || scannerName.empty())
  {
    ROS_ERROR_STREAM("cannot find parameter ""scanner_type"" in the param set. Please specify scanner_type.");
    ROS_ERROR_STREAM("Try to set " << nodeName << " as fallback.\n");
    scannerName = nodeName;
  }

  std::string cloud_topic = "cloud";
  rosDeclareParam(nhPriv, "hostname", "192.168.0.4");
  rosDeclareParam(nhPriv, "imu_enable", true);
  rosDeclareParam(nhPriv, "cloud_topic", cloud_topic);
  if (doInternalDebug)
  {
#ifdef ROSSIMU
      nhPriv->setParam("name", scannerName);
    rossimu_settings(*nhPriv);  // just for tiny simulations under Visual C++
#else
      rosSetParam(nhPriv, "hostname", "192.168.0.4");
      rosSetParam(nhPriv, "imu_enable", true);
      rosSetParam(nhPriv, "cloud_topic", "cloud");
#endif
  }
  rosGetParam(nhPriv, "cloud_topic", cloud_topic);


// check for TCP - use if ~hostname is set.
  bool useTCP = false;
  std::string hostname;
  if (rosGetParam(nhPriv, "hostname", hostname))
  {
    useTCP = true;
  }
  bool changeIP = false;
  std::string sNewIp;
  rosDeclareParam(nhPriv, "new_IP_address", sNewIp);
  if (rosGetParam(nhPriv, "new_IP_address", sNewIp) && !sNewIp.empty())
  {
    changeIP = true;
  }
  std::string port = "2112";
  rosDeclareParam(nhPriv, "port", port);
  rosGetParam(nhPriv, "port", port);

  int timelimit = 5;
  rosDeclareParam(nhPriv, "timelimit", timelimit);
  rosGetParam(nhPriv, "timelimit", timelimit);

  bool subscribe_datagram = false;
  rosDeclareParam(nhPriv, "subscribe_datagram", subscribe_datagram);
  rosGetParam(nhPriv, "subscribe_datagram", subscribe_datagram);

  int device_number = 0;
  rosDeclareParam(nhPriv, "device_number", device_number);
  rosGetParam(nhPriv, "device_number", device_number);

  std::string frame_id = "cloud";
  rosDeclareParam(nhPriv, "frame_id", frame_id);
  rosGetParam(nhPriv, "frame_id", frame_id);

  if(scannerName == "sick_ldmrs")
  {
#if defined LDMRS_SUPPORT && LDMRS_SUPPORT > 0
    ROS_INFO("Initializing LDMRS...");
    sick_scan_xd::SickLdmrsNode ldmrs;
    exit_code = ldmrs.init(nhPriv, hostname, frame_id);
    if(exit_code != sick_scan_xd::ExitSuccess)
    {
      ROS_ERROR("LDMRS initialization failed.");
      exit_code = sick_scan_xd::ExitError;
      return;
    }
    ROS_INFO("LDMRS initialized.");
    // Run event loop
    // rosSpin(nhPriv);
    while(rosOk()) // return after signal, while rosSpin runs in sick_generic_caller
    {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    exit_code = sick_scan_xd::ExitSuccess;
    return;
#else
    ROS_ERROR("LDMRS not supported. Please build sick_scan_xd with option LDMRS_SUPPORT");
    exit_code = sick_scan_xd::ExitError;
    return;
#endif
  }

  if(scannerName == SICK_SCANNER_SCANSEGMENT_XD_NAME || scannerName == SICK_SCANNER_PICOSCAN_NAME)
  {
#if defined SCANSEGMENT_XD_SUPPORT && SCANSEGMENT_XD_SUPPORT > 0
    exit_code = sick_scansegment_xd::run(nhPriv, scannerName);
    std::cout << "sick_generic_laser: sick_scansegment_xd finished with " << (exit_code == sick_scan_xd::ExitSuccess ? "success" : "ERROR") << std::endl;
    return;
#else
    ROS_ERROR_STREAM("SCANSEGMENT_XD_SUPPORT deactivated, " << scannerName << " not supported. Please build sick_scan_xd with option SCANSEGMENT_XD_SUPPORT");
    exit_code = sick_scan_xd::ExitError;
    return;
#endif
  }

  sick_scan_xd::SickGenericParser *parser = new sick_scan_xd::SickGenericParser(scannerName);

  char colaDialectId = 'A'; // A or B (Ascii or Binary)

  float range_min = parser->get_range_min();
  rosDeclareParam(nhPriv, "range_min", range_min);
  if (rosGetParam(nhPriv, "range_min", range_min))
  {
    parser->set_range_min(range_min);
  }
  float range_max = parser->get_range_max();
  rosDeclareParam(nhPriv, "range_max", range_max);
  if (rosGetParam(nhPriv, "range_max", range_max))
  {
    parser->set_range_max(range_max);
  }
  int range_filter_handling = parser->get_range_filter_config();
  rosDeclareParam(nhPriv, "range_filter_handling", range_filter_handling);
  if (rosGetParam(nhPriv, "range_filter_handling", range_filter_handling))
  {
    parser->set_range_filter_config((sick_scan_xd::RangeFilterResultHandling)range_filter_handling);
  }
  ROS_INFO_STREAM("Range filter configuration: range_min=" << range_min << ", range_max=" << range_max << ", range_filter_handling=" << range_filter_handling);

  float time_increment = parser->get_time_increment();
  rosDeclareParam(nhPriv, "time_increment", time_increment);
  if (rosGetParam(nhPriv, "time_increment", time_increment))
  {
    parser->set_time_increment(time_increment);
  }

  /*
   *  Check, if parameter for protocol type is set
   */
  bool use_binary_protocol = true;
  rosDeclareParam(nhPriv, "emul_sensor", emulSensor);
  if (true == rosGetParam(nhPriv, "emul_sensor", emulSensor))
  {
    ROS_INFO_STREAM("Found emul_sensor overwriting default settings. Emulation:" << (emulSensor ? "True" : "False"));
  }
  rosDeclareParam(nhPriv, "use_binary_protocol", use_binary_protocol);
  if (true == rosGetParam(nhPriv, "use_binary_protocol", use_binary_protocol))
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
          rosSetParam(nhPriv, "sopas_protocol_type", true);
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

  sick_scan_xd::SickScanMonitor* scan_msg_monitor = 0;
  sick_scan_xd::PointCloudMonitor* pointcloud_monitor = 0;
  bool message_monitoring_enabled = true;
  int read_timeout_millisec_default = READ_TIMEOUT_MILLISEC_DEFAULT;
  int read_timeout_millisec_startup = READ_TIMEOUT_MILLISEC_STARTUP;
  int read_timeout_millisec_kill_node = READ_TIMEOUT_MILLISEC_KILL_NODE;
  rosDeclareParam(nhPriv, "message_monitoring_enabled", message_monitoring_enabled);
  rosGetParam(nhPriv, "message_monitoring_enabled", message_monitoring_enabled);
  rosDeclareParam(nhPriv, "read_timeout_millisec_default", read_timeout_millisec_default);
  rosGetParam(nhPriv, "read_timeout_millisec_default", read_timeout_millisec_default);
  rosDeclareParam(nhPriv, "read_timeout_millisec_startup", read_timeout_millisec_startup);
  rosGetParam(nhPriv, "read_timeout_millisec_startup", read_timeout_millisec_startup);
  rosDeclareParam(nhPriv, "read_timeout_millisec_kill_node", read_timeout_millisec_kill_node);
  rosGetParam(nhPriv, "read_timeout_millisec_kill_node", read_timeout_millisec_kill_node);
  int message_monitoring_read_timeout_millisec = read_timeout_millisec_default;
  if(message_monitoring_enabled)
  {
    scan_msg_monitor = new sick_scan_xd::SickScanMonitor(message_monitoring_read_timeout_millisec);
#if __ROS_VERSION > 0 // point cloud monitoring in Linux-ROS
    if (read_timeout_millisec_kill_node > 0)
    {
      pointcloud_monitor = new sick_scan_xd::PointCloudMonitor();
      bool pointcloud_monitor_started = pointcloud_monitor->startPointCloudMonitoring(nhPriv, read_timeout_millisec_kill_node, cloud_topic);
      ROS_INFO_STREAM("PointCloudMonitor" << (pointcloud_monitor_started?" ":" NOT ") << "started.");
    }
    else
    {
      ROS_INFO_STREAM("PointCloudMonitor deactivated due to configuration read_timeout_millisec_kill_node=" << read_timeout_millisec_kill_node <<", pointcloud will not be monitored for timeout errors.");
    }
#endif
  }

  bool start_services = true;
  sick_scan_xd::SickScanServices* services = 0;
  exit_code = sick_scan_xd::ExitError;

  //sick_scan_xd::SickScanConfig cfg;
  //std::chrono::system_clock::time_point timestamp_rosOk = std::chrono::system_clock::now();

  while (rosOk() && runState != scanner_finalize)
  {
    //if (rosOk())
    //  timestamp_rosOk = std::chrono::system_clock::now();
    //else if (std::chrono::duration<double>(std::chrono::system_clock::now() - timestamp_rosOk).count() > 2 * 1000) // 2 seconds timeout to stop the scanner
    //  runState = scanner_finalize;

    switch (runState)
    {
      case scanner_init:
        ROS_INFO_STREAM("Start initialising scanner [Ip: " << hostname  << "] [Port:" << port << "]");
        // attempt to connect/reconnect
        DELETE_PTR(s_scanner);  // disconnect scanner
        if (useTCP)
        {
          s_scanner = new sick_scan_xd::SickScanCommonTcp(hostname, port, timelimit, nhPriv, parser, colaDialectId);
        }
        else
        {
          ROS_ERROR("TCP is not switched on. Probably hostname or port not set. Use roslaunch to start node.");
          exit(-1);
        }

        if (emulSensor)
        {
          s_scanner->setEmulSensor(true);
        }
        exit_code = s_scanner->init(nhPriv);
        if (exit_code == sick_scan_xd::ExitError || exit_code == sick_scan_xd::ExitFatal)
        {
		      ROS_ERROR("## ERROR in mainGenericLaser: init failed, retrying..."); // ROS_ERROR("init failed, shutting down");
          continue;
        }

        // Start ROS services
        rosDeclareParam(nhPriv, "start_services", start_services);
        rosGetParam(nhPriv, "start_services", start_services);
        if (true == start_services)
        {
            services = new sick_scan_xd::SickScanServices(nhPriv, s_scanner, parser->getCurrentParamPtr());
            ROS_INFO("SickScanServices: ros services initialized");
        }

        isInitialized = true;
        // signal(SIGINT, SIG_DFL); // change back to standard signal handler after initialising

        if (exit_code == sick_scan_xd::ExitSuccess) // OK -> loop again
        {
          if (changeIP)
          {
            runState = scanner_finalize;
          }
          runState = scanner_run; // after initialising switch to run state
#if __ROS_VERSION > 0
          ROS_INFO_STREAM("Setup completed, sick_scan_xd is up and running. Pointcloud is published on topic \"" << cloud_topic << "\"");
#else
          ROS_INFO("Setup completed, sick_scan_xd is up and running.");
#endif
        }
        else
        {
          runState = scanner_init; // If there was an error, try to restart scanner
        }
        break;

      case scanner_run:
        if (exit_code == sick_scan_xd::ExitSuccess) // OK -> loop again
        {
          if(do_ros_spin)
          {
            rosSpinOnce(nhPriv);
          }
          exit_code = s_scanner->loopOnce(nhPriv);

          if(scan_msg_monitor && message_monitoring_enabled) // Monitor scanner messages
          {
            exit_code = scan_msg_monitor->checkStateReinitOnError(nhPriv, runState, s_scanner, parser, services);
            if(exit_code != sick_scan_xd::ExitSuccess) // scanner re-init failed after read timeout or tcp error
            {
              ROS_ERROR("## ERROR in sick_generic_laser main loop: read timeout, scanner re-init failed");
            }
          }
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
  printf("sick_generic_laser: leaving main loop...");

  if(pointcloud_monitor)
    pointcloud_monitor->stopPointCloudMonitoring();
  DELETE_PTR(scan_msg_monitor);
  DELETE_PTR(pointcloud_monitor);
  DELETE_PTR(services);
  DELETE_PTR(s_scanner); // close connnect
  DELETE_PTR(parser); // close parser
  return;
}

/*!
\brief Runs mainGenericLaser non-blocking in a new thread
\param argc: Number of Arguments
\param argv: Argument variable
\param nodeName name of the ROS-node
\param nhPriv ros node handle
\param exit_code exit-code
\return true if mainGenericLaser is running, false otherwise
\sa mainGenericLaser
*/
bool startGenericLaser(int argc, char **argv, std::string nodeName, rosNodePtr nhPriv, int* exit_code)
{
  if (s_generic_laser_thread == 0)
  {
    runState = scanner_init;
    s_generic_laser_thread = new GenericLaserCallable(argc, argv, nodeName, nhPriv, exit_code);
  }
  return (s_generic_laser_thread != 0);
}

/*!
\brief Waits until all GenericLaser jobs finished.
*/
void joinGenericLaser(void)
{
  if (s_generic_laser_thread != 0)
  {
    s_generic_laser_thread->join();
    delete s_generic_laser_thread;
    s_generic_laser_thread = 0;
  }
}

/*!
\brief Internal Startup routine.
\param argc: Number of Arguments
\param argv: Argument variable
\param nodeName name of the ROS-node
\return exit-code
\sa main
*/
int mainGenericLaser(int argc, char **argv, std::string nodeName, rosNodePtr nhPriv)
{
  int result;
  mainGenericLaserInternal(argc, argv, nodeName, nhPriv, true, result);
  return result;
}

// Send odometry data to NAV350
#include "softwarePLL.h"
#include "sick_scan_api.h"
#include "sick_scan/sick_nav_scandata_parser.h"
int32_t SickScanApiNavOdomVelocityImpl(SickScanApiHandle apiHandle, SickScanNavOdomVelocityMsg* src_msg) // odometry data in nav coordinates
{
  if(s_scanner)
  {
    sick_scan_msg::NAVOdomVelocity nav_msg;
    nav_msg.vel_x = src_msg->vel_x;
    nav_msg.vel_y = src_msg->vel_y;
    nav_msg.omega = src_msg->omega;
    nav_msg.timestamp = src_msg->timestamp;
    nav_msg.coordbase = src_msg->coordbase;
    s_scanner->messageCbNavOdomVelocity(nav_msg);
    return SICK_SCAN_API_SUCCESS;
  }
  return SICK_SCAN_API_ERROR;
}
int32_t SickScanApiOdomVelocityImpl(SickScanApiHandle apiHandle, SickScanOdomVelocityMsg* src_msg) // odometry data in system coordinates
{
  if(s_scanner && s_scanner->getCurrentParamPtr() && SoftwarePLL::instance().IsInitialized())
  {
    sick_scan_msg::NAVOdomVelocity nav_msg;
    nav_msg.vel_x = src_msg->vel_x;
    nav_msg.vel_y = src_msg->vel_y;
    double angle_shift = -1.0 * s_scanner->getCurrentParamPtr()->getScanAngleShift();
    sick_scan_xd::rotateXYbyAngleOffset(nav_msg.vel_x, nav_msg.vel_y, angle_shift); // Convert to velocity in lidar coordinates in m/s
    nav_msg.omega = src_msg->omega; // angular velocity in radians/s
    nav_msg.coordbase = 0; // 0 = local coordinate system of the NAV350
    SoftwarePLL::instance().convSystemtimeToLidarTimestamp(src_msg->timestamp_sec, src_msg->timestamp_nsec, nav_msg.timestamp);
    s_scanner->messageCbNavOdomVelocity(nav_msg);
    return SICK_SCAN_API_SUCCESS;
  }
  else
  {
    ROS_WARN_STREAM("## ERROR SickScanCommon::messageCbRosOdom(): SoftwarePLL not yet ready, timestamp can not be converted from system time to lidar time, odometry message ignored.");
  }
  return SICK_SCAN_API_ERROR;
}
