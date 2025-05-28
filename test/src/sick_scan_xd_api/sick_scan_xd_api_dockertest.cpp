#include <csignal>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#ifdef _MSC_VER
#include <conio.h>
#include <json/json.h>
#else
// #include <ncurses.h>
#include <jsoncpp/json/json.h>
#endif

#include "sick_scan_api.h"
#include "sick_scan_api_dump.h"
#include "sick_scan_api_converter.h"

#include <stdio.h>
#include <algorithm>
#include <cassert>
#include <cstring>

class ApiDockerTestConfig
{
public:
   std::string json_export_file = "";
   std::string imu_topic = "";
};

static Json::Value s_json_pointcloud_messages;
static Json::Value s_json_imu_messages;
static ApiDockerTestConfig s_config;
static bool s_shutdown_signal_received = false;

void exportJsonMessages()
{
  if (!s_config.json_export_file.empty())
  {
    Json::Value json_root;
    if (s_json_pointcloud_messages.size() > 0)
      json_root["RefPointcloudMsg"] = s_json_pointcloud_messages;
    if (s_json_imu_messages.size() > 0)
      json_root["RefImuMsg"] = s_json_imu_messages;
    std::ofstream fs_json(s_config.json_export_file);
    fs_json << json_root;
    std::cout << "sick_scan_xd_api_dockertest: exported messages to json file \"" << s_config.json_export_file << "\"" << std::endl;
  }
}

// signal handler to close docker test after SIGINT and SIGKILL
void signalHandler(int signalRecv)
{
  printf("sick_scan_xd_api_dockertest: caught signal %d, aborting ...\n", signalRecv);
  s_shutdown_signal_received = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  exportJsonMessages();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  printf("sick_scan_xd_api_dockertest: exit (line %d)\n", __LINE__);
}

// Exit with error message
static void exitOnError(const char* msg, int32_t error_code)
{
	printf("## ERROR sick_scan_xd_api_dockertest: %s, error code %d\n", msg, error_code);
	exit(EXIT_FAILURE);
}

// Convert a pointcloud message to json
static void apiMessageToJson(const SickScanPointCloudMsg* msg, Json::Value& json_msg)
{
  json_msg["frame_id"] = msg->header.frame_id;
  json_msg["width"] = msg->width;
  json_msg["height"] = msg->height;
  json_msg["point_step"] = msg->point_step;
  json_msg["row_step"] = msg->row_step;
  Json::Value json_fields;
  for (int field_cnt = 0; field_cnt < msg->fields.size; field_cnt++)
  {
    Json::Value json_field;
    json_field["name"] = msg->fields.buffer[field_cnt].name;
    json_field["offset"] = msg->fields.buffer[field_cnt].offset;
    json_field["datatype"] = msg->fields.buffer[field_cnt].datatype;
    json_field["count"] = msg->fields.buffer[field_cnt].count;
    json_fields.append(json_field);
  }
  json_msg["fields"] = json_fields;
  std::stringstream hex_data_str;
  for (int data_cnt = 0; data_cnt < msg->data.size; data_cnt++)
  {
    hex_data_str << std::setfill('0') << std::setw(2) << std::hex << (int)(msg->data.buffer[data_cnt]);
  }
  json_msg["data"] = hex_data_str.str();
}

// Convert an imu message to json
static void apiMessageToJson(const SickScanImuMsg* msg, Json::Value& json_msg)
{
  json_msg["frame_id"] = msg->header.frame_id;
  json_msg["orientation"].resize(4);
  json_msg["angular_velocity"].resize(3);
  json_msg["linear_acceleration"].resize(3);
  json_msg["orientation_covariance"].resize(9);
  json_msg["angular_velocity_covariance"].resize(9);
  json_msg["linear_acceleration_covariance"].resize(9);
  json_msg["orientation"][0] = msg->orientation.x;
  json_msg["orientation"][1] = msg->orientation.y;
  json_msg["orientation"][2] = msg->orientation.z;
  json_msg["orientation"][3] = msg->orientation.w;
  json_msg["angular_velocity"][0] = msg->angular_velocity.x;
  json_msg["angular_velocity"][1] = msg->angular_velocity.y;
  json_msg["angular_velocity"][2] = msg->angular_velocity.z;
  json_msg["linear_acceleration"][0] = msg->linear_acceleration.x;
  json_msg["linear_acceleration"][1] = msg->linear_acceleration.y;
  json_msg["linear_acceleration"][2] = msg->linear_acceleration.z;
  for(int n = 0; n < 9; n++)
  {
    json_msg["orientation_covariance"][n] = msg->orientation_covariance[n];
    json_msg["angular_velocity_covariance"][n] = msg->angular_velocity_covariance[n];
    json_msg["linear_acceleration_covariance"][n] = msg->linear_acceleration_covariance[n];
  }
}

// Prints a json value to string, replacing all '\n', '\r' and '\t' by space ' '
static std::string jsonToOneLineString(const Json::Value& json_msg)
{
  std::stringstream json_str;
  json_str << json_msg;
  std::string json_print = json_str.str();
  std::replace(json_print.begin(), json_print.end(), '\n', ' ');
  std::replace(json_print.begin(), json_print.end(), '\r', ' ');
  std::replace(json_print.begin(), json_print.end(), '\t', ' ');
  return json_print;
}

// Append to temporary json logfile
static void appendJsonMsgToLogfile(const std::string& type, const std::string& topic, const Json::Value& json_msg)
{
  if (!s_config.json_export_file.empty())
  {
    Json::Value json_log_msg, json_log_msg_topic;
    json_log_msg_topic[topic] = json_msg;
    json_log_msg[type] = json_log_msg_topic;
    std::string json_log_str = jsonToOneLineString(json_log_msg);
    std::string json_log_file = s_config.json_export_file + ".log";
    std::ofstream fs_json_log(json_log_file, std::ios_base::app);
    fs_json_log << json_log_str << std::endl;
  }
}

// Example callback for cartesian pointcloud messages, converts and publishes a SickScanPointCloudMsg to sensor_msgs::PointCloud2 on ROS-1
static void apiTestCartesianPointCloudMsgCallback(SickScanApiHandle apiHandle, const SickScanPointCloudMsg* msg)
{
	printf("[Info]: apiTestCartesianPointCloudMsgCallback(apiHandle:%p): %dx%d pointcloud callback...\n", apiHandle, msg->width, msg->height);
  Json::Value json_msg;
  apiMessageToJson(msg, json_msg);
  s_json_pointcloud_messages[msg->topic].append(json_msg);
  appendJsonMsgToLogfile("RefPointcloudMsg", msg->topic, json_msg);
}

// Example callback for polar pointcloud messages, converts and publishes a SickScanPointCloudMsg to sensor_msgs::PointCloud2 on ROS-1
static void apiTestPolarPointCloudMsgCallback(SickScanApiHandle apiHandle, const SickScanPointCloudMsg* msg)
{
	printf("[Info]: apiTestPolarPointCloudMsgCallback(apiHandle:%p): %dx%d pointcloud callback...\n", apiHandle, msg->width, msg->height);
  Json::Value json_msg;
  apiMessageToJson(msg, json_msg);
  s_json_pointcloud_messages[msg->topic].append(json_msg);
  appendJsonMsgToLogfile("RefPointcloudMsg", msg->topic, json_msg);
}

// Example callback for imu messages
static void apiTestImuMsgCallback(SickScanApiHandle apiHandle, const SickScanImuMsg* msg)
{
	printf("[Info]: apiTestImuMsgCallback(apiHandle:%p): Imu message, orientation=(%.6f,%.6f,%.6f,%.6f), angular_velocity=(%.6f,%.6f,%.6f), linear_acceleration=(%.6f,%.6f,%.6f)\n",
	    apiHandle, msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w,
      msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.y,
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  Json::Value json_msg;
  apiMessageToJson(msg, json_msg);
  std::string topic = msg->topic;
  if (!s_config.imu_topic.empty())
    topic = s_config.imu_topic; // imu topic overwritten by commandline option
  s_json_imu_messages[topic].append(json_msg);
  appendJsonMsgToLogfile("RefImuMsg", topic, json_msg);
}

// Example callback for RadarScan messages
static void apiTestRadarScanMsgCallback(SickScanApiHandle apiHandle, const SickScanRadarScan* msg)
{
  printf("[Info]: apiTestRadarScanMsgCallback(apiHandle:%p): RadarScan message %dx%d\n", apiHandle, msg->targets.width, msg->targets.height);
  Json::Value json_msg;
  apiMessageToJson(&msg->targets, json_msg);
  std::string topic = "/cloud_radar_track";
  s_json_pointcloud_messages[topic].append(json_msg);
  appendJsonMsgToLogfile("RefPointcloudMsg", topic, json_msg);
}

// sick_scan_api_test main: Initialize, receive and process lidar messages via sick_scan_xd API.
void sick_scan_api_test_main(int argc, char** argv)
{
  int32_t ret = SICK_SCAN_API_SUCCESS;
  SickScanApiHandle apiHandle = 0;

  if ((apiHandle = SickScanApiCreate(argc, argv)) == 0)
    exitOnError("SickScanApiCreate failed", -1);

  // Initialize a lidar and starts message receiving and processing
  if ((ret = SickScanApiInitByCli(apiHandle, argc, argv)) != SICK_SCAN_API_SUCCESS)
    exitOnError("SickScanApiInitByCli failed", ret);

  // Register a callback for PointCloud messages
  if ((ret = SickScanApiRegisterCartesianPointCloudMsg(apiHandle, apiTestCartesianPointCloudMsgCallback)) != SICK_SCAN_API_SUCCESS)
    exitOnError("SickScanApiRegisterCartesianPointCloudMsg failed", ret);
  if ((ret = SickScanApiRegisterPolarPointCloudMsg(apiHandle, apiTestPolarPointCloudMsgCallback)) != SICK_SCAN_API_SUCCESS)
    exitOnError("SickScanApiRegisterCartesianPointCloudMsg failed", ret);

  // Register a callback for Imu messages
  if ((ret = SickScanApiRegisterImuMsg(apiHandle, apiTestImuMsgCallback)) != SICK_SCAN_API_SUCCESS)
    exitOnError("SickScanApiRegisterImuMsg failed", ret);

  // Register a callback for RadarScan messages
  if ((ret = SickScanApiRegisterRadarScanMsg(apiHandle, apiTestRadarScanMsgCallback)) != SICK_SCAN_API_SUCCESS)
    exitOnError("SickScanApiRegisterRadarScanMsg failed", ret);

  // Run main loop
  while (!s_shutdown_signal_received)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // Cleanup and exit
  printf("sick_scan_xd_api_dockertest finishing...\n");
  SickScanApiDeregisterCartesianPointCloudMsg(apiHandle, apiTestCartesianPointCloudMsgCallback);
  SickScanApiDeregisterPolarPointCloudMsg(apiHandle, apiTestPolarPointCloudMsgCallback);
  SickScanApiDeregisterImuMsg(apiHandle, apiTestImuMsgCallback);
  SickScanApiDeregisterRadarScanMsg(apiHandle, apiTestRadarScanMsgCallback);

  if ((ret = SickScanApiClose(apiHandle)) != SICK_SCAN_API_SUCCESS)
    exitOnError("SickScanApiClose failed", ret);
  if ((ret = SickScanApiRelease(apiHandle)) != SICK_SCAN_API_SUCCESS)
    exitOnError("SickScanApiRelease failed", ret);
}

// sick_scan_api_test main: Initialize, receive and process lidar messages via sick_scan_xd API.
int main(int argc, char** argv)
{
  int32_t ret = SICK_SCAN_API_SUCCESS;
  for (int n = 0; n < argc; n++)
  {
    printf("%s%s", (n > 0 ? " " : ""), argv[n]);
    if (strncmp(argv[n], "imu_topic:=", 11) == 0)
      s_config.imu_topic = argv[n] + 11;
    if (strncmp(argv[n], "_jsonfile:=", 11) == 0)
      s_config.json_export_file = argv[n] + 11;
  }
  signal(SIGINT, signalHandler);  // SIGINT = 2, Ctrl-C or kill -2
  signal(SIGTERM, signalHandler); // SIGTERM = 15, default kill level
  printf("\nsick_scan_xd_api_dockertest started\n");

#ifdef _MSC_VER
  std::string sick_scan_api_lib = "sick_scan_xd_shared_lib.dll";
  std::vector<std::string> search_library_path = { "", "build/Debug/", "build_win64/Debug/", "src/build/Debug/", "src/build_win64/Debug/", "src/sick_scan_xd/build/Debug/", "src/sick_scan_xd/build_win64/Debug/", "./", "../" };
#else
  std::string sick_scan_api_lib = "libsick_scan_xd_shared_lib.so";
  std::vector<std::string> search_library_path = { "", "build/", "build_linux/", "src/build/", "src/build_linux/", "src/sick_scan_xd/build/", "src/sick_scan_xd/build_linux/", "./", "../" };
#endif
  ret = SICK_SCAN_API_NOT_LOADED;
  for(int search_library_cnt = 0; search_library_cnt < search_library_path.size(); search_library_cnt++)
  {
    std::string libfilepath = search_library_path[search_library_cnt] + sick_scan_api_lib;
    if ((ret = SickScanApiLoadLibrary(libfilepath.c_str())) == SICK_SCAN_API_SUCCESS)
    {
      printf("sick_scan_xd library \"%s\" loaded successfully\n", libfilepath.c_str());
      break;
    }
  }
  if (ret != SICK_SCAN_API_SUCCESS)
    exitOnError("SickScanApiLoadLibrary failed", ret);

  // Initialize and run sick_scan_xd_api_dockertest
  sick_scan_api_test_main(argc, argv);

  // Unload and exit
  printf("sick_scan_xd_api_dockertest finishing...\n");
  if ((ret = SickScanApiUnloadLibrary()) != SICK_SCAN_API_SUCCESS)
    exitOnError("SickScanApiUnloadLibrary failed", ret);
  printf("sick_scan_xd_api_dockertest finished successfully\n");
  exit(EXIT_SUCCESS);
}
