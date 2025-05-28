#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#ifdef _MSC_VER
#include <conio.h>
#else
// #include <ncurses.h>
#endif

#include "sick_scan_api.h"
#include "sick_scan_api_dump.h"
#include "sick_scan_api_converter.h"

#if __ROS_VERSION == 1
std::string ros_api_cloud_topic = "api_cloud";
std::string ros_api_cloud_polar_topic = "api_cloud_polar";
std::string ros_api_visualizationmarker_topic = "marker";
ros::Publisher ros_api_cloud_publisher;
ros::Publisher ros_api_cloud_polar_publisher;
ros::Publisher ros_api_visualizationmarker_publisher;
#else
#include <stdio.h>
#include <algorithm>
#include <cassert>
#include <cstring>

const float IMAGE_SCALE = 250.0f;
const float IMAGE_OFFSET = 2.0f;
const int IMAGE_WIDTH = 1000;
const int IMAGE_HEIGHT = 1000;

#include "toojpeg.h"
static FILE* foutJpg = 0;
#endif

#if __ROS_VERSION != 1
// jpeg callback, just writes one byte
void jpegOutputCallback(unsigned char oneByte)
{
  assert(foutJpg != 0);
  fwrite(&oneByte, 1, 1, foutJpg);
}

/**
 * @brief Clamps a value between a minimum and a maximum limit.
 *
 * Returns the input value if it is within the [min_val, max_val] range.
 * Otherwise, returns the nearest boundary value.
 *
 * @tparam T  Type of the input value (must support comparison operators).
 * @param val       The value to be clamped.
 * @param min_val   The minimum permissible value.
 * @param max_val   The maximum permissible value.
 * @return          The clamped value within [min_val, max_val].
 *
 * @note If min_val > max_val, behavior is undefined.
 */
template <typename T>
T clamp(T val, T min_val, T max_val) {
  return std::max(min_val, std::min(val, max_val));
}

/**
 * @brief Converts an intensity value into a pseudo-color RGB value using a jet colormap approximation.
 *
 * This function maps a given intensity value (typically in the range [0, 65535]) to an RGB color using
 * a color ramp similar to the MATLAB/Jet colormap. The output color starts from blue for low intensities
 * and transitions through green to red for high intensities.
 *
 * @param[in] intensity  The intensity value to be mapped to a color (expected range: 0.0 to 65535.0).
 * @param[out] r         Reference to the red channel (0–255) to be set by the function.
 * @param[out] g         Reference to the green channel (0–255) to be set by the function.
 * @param[out] b         Reference to the blue channel (0–255) to be set by the function.
 *
 * @note This function clamps the normalized intensity to [0.0, 1.0] before processing.
 *       The resulting color is a visualization aid, not physically meaningful.
 */
static void intensity_to_rgb(float intensity, uint8_t& r, uint8_t& g, uint8_t& b)
{
  // Normalize intensity (0 to 65535) ? (0 to 1)
  float normalized = intensity / 65535.0f;

  if (normalized < 0.0f) normalized = 0.0f;
  if (normalized > 1.0f) normalized = 1.0f;

  // Map to a rainbow (jet) colormap approximation
  float fourValue = 4.0f * normalized;

  float rf = fourValue - 1.5f;
  float gf = fourValue - 0.5f;
  float bf = fourValue + 0.5f;

  // Compute Red component
  if (rf <= 0.0f) r = 0;
  else if (rf >= 1.0f) r = 255;
  else r = static_cast<uint8_t>(255.0f * rf);

  // Compute Green component
  if (gf <= 0.0f) g = 0;
  else if (gf >= 1.0f) g = 255;
  else g = static_cast<uint8_t>(255.0f * gf);

  // Compute Blue component
  if (bf <= 0.0f) b = 0;
  else if (bf >= 1.0f) b = 255;
  else b = static_cast<uint8_t>(255.0f * bf);

  // Reverse segments for descending slopes:
  if (fourValue > 2.5f)
    r = static_cast<uint8_t>(255 - r);
  if (fourValue > 1.5f && fourValue <= 3.5f)
    g = static_cast<uint8_t>(255 - g);
  if (fourValue > 0.5f && fourValue <= 2.5f)
    b = static_cast<uint8_t>(255 - b);
}


// Simple plot function for pointcloud data, just demonstrates how to use a SickScanPointCloudMsg
static void plotPointcloudToJpeg(const std::string& jpegfilepath, const SickScanPointCloudMsg& msg)
{
  constexpr int img_width = IMAGE_WIDTH, img_height = IMAGE_HEIGHT;
  // Get offsets for x, y, z, intensity values
  SickScanPointFieldMsg* msg_fields_buffer = (SickScanPointFieldMsg*)msg.fields.buffer;
  int field_offset_x = -1, field_offset_y = -1, field_offset_z = -1, field_offset_intensity = -1;
  for (int n = 0; n < msg.fields.size; n++)
  {
    if (strcmp(msg_fields_buffer[n].name, "x") == 0 && msg_fields_buffer[n].datatype == SICK_SCAN_POINTFIELD_DATATYPE_FLOAT32)
      field_offset_x = msg_fields_buffer[n].offset;
    else if (strcmp(msg_fields_buffer[n].name, "y") == 0 && msg_fields_buffer[n].datatype == SICK_SCAN_POINTFIELD_DATATYPE_FLOAT32)
      field_offset_y = msg_fields_buffer[n].offset;
    else if (strcmp(msg_fields_buffer[n].name, "z") == 0 && msg_fields_buffer[n].datatype == SICK_SCAN_POINTFIELD_DATATYPE_FLOAT32)
      field_offset_z = msg_fields_buffer[n].offset;
    else if ((strcmp(msg_fields_buffer[n].name, "intensity") == 0 ||
      strcmp(msg_fields_buffer[n].name, "i") == 0) &&
      msg_fields_buffer[n].datatype == SICK_SCAN_POINTFIELD_DATATYPE_FLOAT32)
    {
      field_offset_intensity = msg_fields_buffer[n].offset;
    }
  }
  assert(field_offset_x >= 0 && field_offset_y >= 0 && field_offset_z >= 0);
  // Create an image with 250 pixel/meter, max. +/-2 meter

  uint8_t* img_pixel = (uint8_t*)calloc(3 * img_width * img_height, sizeof(uint8_t)); // allocate 3 byte RGB array
  // Plot all points in pointcloud
  for (uint32_t row_idx = 0; row_idx < msg.height; ++row_idx)
  {
    const uint8_t* row_ptr = msg.data.buffer + row_idx * msg.row_step;
    for (uint32_t col_idx = 0; col_idx < msg.width; ++col_idx)
    {
      const uint8_t* point_ptr = row_ptr + col_idx * msg.point_step;

      const float point_x = *reinterpret_cast<const float*>(point_ptr + field_offset_x);
      const float point_y = *reinterpret_cast<const float*>(point_ptr + field_offset_y);
      // const float point_z = *reinterpret_cast<const float*>(point_ptr + field_offset_z); // unused

      float point_intensity = 0.0f;
      if (field_offset_intensity >= 0)
      {
        point_intensity = *reinterpret_cast<const float*>(point_ptr + field_offset_intensity);
      }

      const int img_x = static_cast<int>(IMAGE_SCALE * (-point_y + 2.0f));
      const int img_y = static_cast<int>(IMAGE_SCALE * (-point_x + 2.0f));

      if (img_x >= 0 && img_x < img_width && img_y >= 0 && img_y < img_height)
      {
        uint8_t r, g, b;
        intensity_to_rgb(point_intensity, r, g, b);

        const int img_index = 3 * (img_y * img_width + img_x);
        img_pixel[img_index + 0] = r;
        img_pixel[img_index + 1] = g;
        img_pixel[img_index + 2] = b;
      }
    }
  }
  // Write image to jpeg-file
  std::string jpeg_filename = jpegfilepath;
#ifdef _MSC_VER
  std::replace(jpeg_filename.begin(), jpeg_filename.end(), '/', '\\');
#else
  std::replace(jpeg_filename.begin(), jpeg_filename.end(), '\\', '/');
#endif
  std::string jpeg_filename_tmp = jpeg_filename + "_tmp";
  foutJpg = fopen(jpeg_filename_tmp.c_str(), "wb");
  if (foutJpg)
  {
    TooJpeg::writeJpeg(jpegOutputCallback, img_pixel, img_width, img_height, true, 99);
    fclose(foutJpg);
#ifdef _MSC_VER
    _unlink(jpegfilepath.c_str());
    rename(jpeg_filename_tmp.c_str(), jpegfilepath.c_str());
#else
    rename(jpeg_filename_tmp.c_str(), jpegfilepath.c_str());
#endif
  }
  free(img_pixel);
}
#endif

// Exit with error message
static void exitOnError(const char* msg, int32_t error_code)
{
  printf("## ERROR sick_scan_xd_api_test: %s, error code %d\n", msg, error_code);
  exit(EXIT_FAILURE);
}

// Example callback for cartesian pointcloud messages, converts and publishes a SickScanPointCloudMsg to sensor_msgs::PointCloud2 on ROS-1
static void apiTestCartesianPointCloudMsgCallback(SickScanApiHandle apiHandle, const SickScanPointCloudMsg* msg)
{
  printf("[Info]: apiTestCartesianPointCloudMsgCallback(apiHandle:%p): %dx%d pointcloud callback...\n", apiHandle, msg->width, msg->height);
#if __ROS_VERSION == 1
  sensor_msgs::PointCloud2 pointcloud = SickScanApiConverter::convertPointCloudMsg(*msg);
  ros_api_cloud_publisher.publish(pointcloud);
  ROS_INFO_STREAM("apiTestCartesianPointCloudMsgCallback(apiHandle:" << apiHandle << "): published " << pointcloud.width << "x" << pointcloud.height << " pointcloud on topic \"" << ros_api_cloud_topic << "\"");
  DUMP_API_POINTCLOUD_MESSAGE("test", pointcloud);
#else
  plotPointcloudToJpeg("/tmp/sick_scan_api_demo.jpg", *msg);
#endif
}

// Example callback for polar pointcloud messages, converts and publishes a SickScanPointCloudMsg to sensor_msgs::PointCloud2 on ROS-1
static void apiTestPolarPointCloudMsgCallback(SickScanApiHandle apiHandle, const SickScanPointCloudMsg* msg)
{
  printf("[Info]: apiTestPolarPointCloudMsgCallback(apiHandle:%p): %dx%d pointcloud callback...\n", apiHandle, msg->width, msg->height);
#if __ROS_VERSION == 1
  sensor_msgs::PointCloud2 pointcloud = SickScanApiConverter::convertPolarPointCloudMsg(*msg);
  ros_api_cloud_polar_publisher.publish(pointcloud);
  ROS_INFO_STREAM("apiTestPolarPointCloudMsgCallback(apiHandle:" << apiHandle << "): published " << pointcloud.width << "x" << pointcloud.height << " pointcloud on topic \"" << ros_api_cloud_polar_topic << "\"");
#endif
}

// Example callback for imu messages
static void apiTestImuMsgCallback(SickScanApiHandle apiHandle, const SickScanImuMsg* msg)
{
  printf("[Info]: apiTestImuMsgCallback(apiHandle:%p): Imu message, orientation=(%.6f,%.6f,%.6f,%.6f), angular_velocity=(%.6f,%.6f,%.6f), linear_acceleration=(%.6f,%.6f,%.6f)\n",
    apiHandle, msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w,
    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.y,
    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
#if __ROS_VERSION == 1
  sensor_msgs::Imu ros_msg = SickScanApiConverter::convertImuMsg(*msg);
  DUMP_API_IMU_MESSAGE("test", ros_msg);
#endif
}

// Example callback for lferec messages
static void apiTestLFErecMsgCallback(SickScanApiHandle apiHandle, const SickScanLFErecMsg* msg)
{
  printf("[Info]: apiTestLFErecMsgCallback(apiHandle:%p): LFErec message, %d fields\n", apiHandle, (int)msg->fields_number);
#if __ROS_VERSION == 1
  sick_scan_xd::LFErecMsg ros_msg = SickScanApiConverter::convertLFErecMsg(*msg);
  DUMP_API_LFEREC_MESSAGE("test", ros_msg);
#endif
}

// Example callback for LIDoutputstate messages
static void apiTestLIDoutputstateMsgCallback(SickScanApiHandle apiHandle, const SickScanLIDoutputstateMsg* msg)
{
  printf("[Info]: apiTestLIDoutputstateMsgCallback(apiHandle:%p): LIDoutputstate message, state=(%d,%d,%d,%d,%d,%d,%d,%d), count=(%d,%d,%d,%d,%d,%d,%d,%d)\n", apiHandle,
    (int)msg->output_state[0], (int)msg->output_state[1], (int)msg->output_state[2], (int)msg->output_state[3], (int)msg->output_state[4], (int)msg->output_state[5], (int)msg->output_state[6], (int)msg->output_state[7],
    (int)msg->output_state[0], (int)msg->output_count[1], (int)msg->output_count[2], (int)msg->output_count[3], (int)msg->output_count[4], (int)msg->output_count[5], (int)msg->output_count[6], (int)msg->output_count[7]);
#if __ROS_VERSION == 1
  sick_scan_xd::LIDoutputstateMsg ros_msg = SickScanApiConverter::convertLIDoutputstateMsg(*msg);
  DUMP_API_LIDOUTPUTSTATE_MESSAGE("test", ros_msg);
#endif
}

// Example callback for RadarScan messages
static void apiTestRadarScanMsgCallback(SickScanApiHandle apiHandle, const SickScanRadarScan* msg)
{
  printf("[Info]: apiTestRadarScanMsgCallback(apiHandle:%p): RadarScan message, %d targets, %d objects\n", apiHandle, (int)(msg->targets.width * msg->targets.height), (int)msg->objects.size);
#if __ROS_VERSION == 1
  sick_scan_xd::RadarScan ros_msg = SickScanApiConverter::convertRadarScanMsg(*msg);
  if (ros_msg.targets.width * ros_msg.targets.height > 0)
    ros_api_cloud_publisher.publish(ros_msg.targets);
  sensor_msgs::PointCloud2 ros_pointcloud = SickScanApiConverter::convertRadarObjectsToPointCloud(msg->header, &ros_msg.objects[0], ros_msg.objects.size());
  if (ros_pointcloud.width * ros_pointcloud.height > 0)
    ros_api_cloud_polar_publisher.publish(ros_pointcloud);
  DUMP_API_RADARSCAN_MESSAGE("test", ros_msg);
#endif
}

// Example callback for LdmrsObjectArray messages
static void apiTestLdmrsObjectArrayCallback(SickScanApiHandle apiHandle, const SickScanLdmrsObjectArray* msg)
{
  printf("[Info]: apiTestLdmrsObjectArrayCallback(apiHandle:%p): LdmrsObjectArray message, %d objects\n", apiHandle, (int)msg->objects.size);
#if __ROS_VERSION == 1
  sick_scan_xd::SickLdmrsObjectArray ros_msg = SickScanApiConverter::convertLdmrsObjectArray(*msg);
  DUMP_API_LDMRSOBJECTARRAY_MESSAGE("test", ros_msg);
#endif
}

// Example callback for VisualizationMarker messages
static void apiTestVisualizationMarkerMsgCallback(SickScanApiHandle apiHandle, const SickScanVisualizationMarkerMsg* msg)
{
  printf("[Info]: apiTestVisualizationMarkerMsgCallback(apiHandle:%p): VisualizationMarker message, %d objects\n", apiHandle, (int)msg->markers.size);
#if __ROS_VERSION == 1
  visualization_msgs::MarkerArray ros_msg = SickScanApiConverter::convertVisualizationMarkerMsg(*msg);
  DUMP_API_VISUALIZATIONMARKER_MESSAGE("test", ros_msg);
  if (ros_msg.markers.size() > 0)
    ros_api_visualizationmarker_publisher.publish(ros_msg);
#endif
}

// Example callback for NAV350 Pose- and Landmark messages
static void apiTestNavPoseLandmarkMsgCallback(SickScanApiHandle apiHandle, const SickScanNavPoseLandmarkMsg* msg)
{
  printf("[Info]: apiTestNavPoseLandmarkMsgCallback(apiHandle:%p): pose_x=%f, pose_y=%f, yaw=%f, %d reflectors\n", apiHandle, msg->pose_x, msg->pose_y, msg->pose_yaw, (int)msg->reflectors.size);
}

// Example callback for diagnostic messages
static void apiTestDiagnosticMsgCallback(SickScanApiHandle apiHandle, const SickScanDiagnosticMsg* msg)
{
  if (msg->status_code == 1) // status_code defined in SICK_DIAGNOSTIC_STATUS: WARN=1
    printf("[WARN]: apiTestDiagnosticMsgCallback(apiHandle:%p): status_code = %d (WARNING), status_message = \"%s\"\n", apiHandle, msg->status_code, msg->status_message);
  else if (msg->status_code == 2) // status_code defined in SICK_DIAGNOSTIC_STATUS: ERROR=2
    printf("[ERROR]: apiTestDiagnosticMsgCallback(apiHandle:%p): status_code = %d (ERROR), status_message = \"%s\"\n", apiHandle, msg->status_code, msg->status_message);
  else
    printf("[Info]: apiTestDiagnosticMsgCallback(apiHandle:%p): status_code = %d, status_message = \"%s\"\n", apiHandle, msg->status_code, msg->status_message);
  int32_t status_code = -1;
  char message_buffer[1024] = "";
  if (SickScanApiGetStatus(apiHandle, &status_code, message_buffer, (int32_t)sizeof(message_buffer)) == SICK_SCAN_API_SUCCESS)
  {
    printf("[Info]: SickScanApiGetStatus(apiHandle:%p): status_code = %d, message = \"%s\"\n", apiHandle, status_code, message_buffer);
  }
  else
  {
    printf("[ERROR]: SickScanApiGetStatus(apiHandle:%p) failed\n", apiHandle);
  }
}

// Example callback for diagnostic messages
static void apiTestLogMsgCallback(SickScanApiHandle apiHandle, const SickScanLogMsg* msg)
{
  if (msg->log_level == 2) // log_level defined in ros::console::levels: Warn=2
    printf("[WARN]: apiTestLogMsgCallback(apiHandle:%p): log_level = %d (WARNING), log_message = %s\n", apiHandle, msg->log_level, msg->log_message);
  else if (msg->log_level >= 3) // log_level defined in ros::console::levels: Error=3, Fatal=4
    printf("[ERROR]: apiTestLogMsgCallback(apiHandle:%p): log_level = %d (ERROR), log_message = %s\n", apiHandle, msg->log_level, msg->log_message);
  else if (false) // debugging
    printf("[Info]: apiTestLogMsgCallback(apiHandle:%p): log_level = %d, log_message = %s\n", apiHandle, msg->log_level, msg->log_message);
}

// Receive lidar message by SickScanApiWaitNext-functions ("message polling")
static void runSickScanApiTestWaitNext(SickScanApiHandle* apiHandle, bool* run_flag)
{
  double wait_next_message_timeout = 0.1; // wait max. 0.1 seconds for the next message (otherwise SickScanApiWaitNext-function return with timeout)
  SickScanPointCloudMsg pointcloud_msg;
  SickScanImuMsg imu_msg;
  SickScanLFErecMsg lferec_msg;
  SickScanLIDoutputstateMsg lidoutputstate_msg;
  SickScanRadarScan radarscan_msg;
  SickScanLdmrsObjectArray ldmrsobjectarray_msg;
  SickScanVisualizationMarkerMsg visualizationmarker_msg;
  SickScanNavPoseLandmarkMsg navposelandmark_msg;
  SickScanOdomVelocityMsg odom_msg;
  odom_msg.vel_x = +1.0f;
  odom_msg.vel_y = -1.0f;
  odom_msg.omega = 0.5f;
  odom_msg.timestamp_sec = 12345;
  odom_msg.timestamp_nsec = 6789;
  SickScanNavOdomVelocityMsg navodom_msg;
  navodom_msg.vel_x = +1.0f;
  navodom_msg.vel_y = -1.0f;
  navodom_msg.omega = 0.5f;
  navodom_msg.timestamp = 123456789;
  navodom_msg.coordbase = 0;
  while (run_flag && *run_flag)
  {
    // Get/poll the next cartesian PointCloud message
    int32_t ret = SickScanApiWaitNextCartesianPointCloudMsg(*apiHandle, &pointcloud_msg, wait_next_message_timeout);
    if (ret == SICK_SCAN_API_SUCCESS)
      apiTestCartesianPointCloudMsgCallback(*apiHandle, &pointcloud_msg);
    else if (ret != SICK_SCAN_API_SUCCESS && ret != SICK_SCAN_API_TIMEOUT)
      printf("## ERROR sick_scan_xd_api_test: SickScanApiWaitNextCartesianPointCloudMsg failed\n");
    SickScanApiFreePointCloudMsg(*apiHandle, &pointcloud_msg);

    // Get/poll the next polar PointCloud message
    ret = SickScanApiWaitNextPolarPointCloudMsg(*apiHandle, &pointcloud_msg, wait_next_message_timeout);
    if (ret == SICK_SCAN_API_SUCCESS)
      apiTestPolarPointCloudMsgCallback(*apiHandle, &pointcloud_msg);
    else if (ret != SICK_SCAN_API_SUCCESS && ret != SICK_SCAN_API_TIMEOUT)
      printf("## ERROR sick_scan_xd_api_test: SickScanApiWaitNextPolarPointCloudMsg failed\n");
    SickScanApiFreePointCloudMsg(*apiHandle, &pointcloud_msg);

    // Get/poll the next Imu message
    ret = SickScanApiWaitNextImuMsg(*apiHandle, &imu_msg, wait_next_message_timeout);
    if (ret == SICK_SCAN_API_SUCCESS)
      apiTestImuMsgCallback(*apiHandle, &imu_msg);
    else if (ret != SICK_SCAN_API_SUCCESS && ret != SICK_SCAN_API_TIMEOUT)
      printf("## ERROR sick_scan_xd_api_test: SickScanApiWaitNextImuMsg failed\n");
    SickScanApiFreeImuMsg(*apiHandle, &imu_msg);

    // Get/poll the next LFErec message
    ret = SickScanApiWaitNextLFErecMsg(*apiHandle, &lferec_msg, wait_next_message_timeout);
    if (ret == SICK_SCAN_API_SUCCESS)
      apiTestLFErecMsgCallback(*apiHandle, &lferec_msg);
    else if (ret != SICK_SCAN_API_SUCCESS && ret != SICK_SCAN_API_TIMEOUT)
      printf("## ERROR sick_scan_xd_api_test: SickScanApiWaitNextLFErecMsg failed\n");
    SickScanApiFreeLFErecMsg(*apiHandle, &lferec_msg);

    // Get/poll the next LIDoutputstate message
    ret = SickScanApiWaitNextLIDoutputstateMsg(*apiHandle, &lidoutputstate_msg, wait_next_message_timeout);
    if (ret == SICK_SCAN_API_SUCCESS)
      apiTestLIDoutputstateMsgCallback(*apiHandle, &lidoutputstate_msg);
    else if (ret != SICK_SCAN_API_SUCCESS && ret != SICK_SCAN_API_TIMEOUT)
      printf("## ERROR sick_scan_xd_api_test: SickScanApiWaitNextLIDoutputstateMsg failed\n");
    SickScanApiFreeLIDoutputstateMsg(*apiHandle, &lidoutputstate_msg);

    // Get/poll the next RadarScan message
    ret = SickScanApiWaitNextRadarScanMsg(*apiHandle, &radarscan_msg, wait_next_message_timeout);
    if (ret == SICK_SCAN_API_SUCCESS)
      apiTestRadarScanMsgCallback(*apiHandle, &radarscan_msg);
    else if (ret != SICK_SCAN_API_SUCCESS && ret != SICK_SCAN_API_TIMEOUT)
      printf("## ERROR sick_scan_xd_api_test: SickScanApiWaitNextRadarScanMsg failed\n");
    SickScanApiFreeRadarScanMsg(*apiHandle, &radarscan_msg);

    // Get/poll the next LdmrsObjectArray message
    ret = SickScanApiWaitNextLdmrsObjectArrayMsg(*apiHandle, &ldmrsobjectarray_msg, wait_next_message_timeout);
    if (ret == SICK_SCAN_API_SUCCESS)
      apiTestLdmrsObjectArrayCallback(*apiHandle, &ldmrsobjectarray_msg);
    else if (ret != SICK_SCAN_API_SUCCESS && ret != SICK_SCAN_API_TIMEOUT)
      printf("## ERROR sick_scan_xd_api_test: SickScanApiWaitNextLdmrsObjectArrayMsg failed\n");
    SickScanApiFreeLdmrsObjectArrayMsg(*apiHandle, &ldmrsobjectarray_msg);

    // Get/poll the next VisualizationMarker message
    ret = SickScanApiWaitNextVisualizationMarkerMsg(*apiHandle, &visualizationmarker_msg, wait_next_message_timeout);
    if (ret == SICK_SCAN_API_SUCCESS)
      apiTestVisualizationMarkerMsgCallback(*apiHandle, &visualizationmarker_msg);
    else if (ret != SICK_SCAN_API_SUCCESS && ret != SICK_SCAN_API_TIMEOUT)
      printf("## ERROR sick_scan_xd_api_test: SickScanApiWaitNextVisualizationMarkerMsg failed\n");
    SickScanApiFreeVisualizationMarkerMsg(*apiHandle, &visualizationmarker_msg);

    // Get/poll the next NAV350 Pose- and Landmark message
    ret = SickScanApiWaitNextNavPoseLandmarkMsg(*apiHandle, &navposelandmark_msg, wait_next_message_timeout);
    if (ret == SICK_SCAN_API_SUCCESS)
      apiTestNavPoseLandmarkMsgCallback(*apiHandle, &navposelandmark_msg);
    else if (ret != SICK_SCAN_API_SUCCESS && ret != SICK_SCAN_API_TIMEOUT)
      printf("## ERROR sick_scan_xd_api_test: SickScanApiWaitNextNavPoseLandmarkMsg failed\n");
    SickScanApiFreeNavPoseLandmarkMsg(*apiHandle, &navposelandmark_msg);

    // Send NAV350 odom message example
    // ret = SickScanApiNavOdomVelocityMsg(*apiHandle, &navodom_msg);
    // ret = SickScanApiOdomVelocityMsg(*apiHandle, &odom_msg);
  }
}

// sick_scan_api_test main: Initialize, receive and process lidar messages via sick_scan_xd API.
int sick_scan_api_test_main(int argc, char** argv, const std::string& sick_scan_args, bool polling)
{
  int32_t ret = SICK_SCAN_API_SUCCESS;
  SickScanApiHandle apiHandle = 0;

  if ((apiHandle = SickScanApiCreate(argc, argv)) == 0)
    exitOnError("SickScanApiCreate failed", -1);

  // Initialize a lidar and starts message receiving and processing
#if __ROS_VERSION == 1
  if ((ret = SickScanApiInitByLaunchfile(apiHandle, sick_scan_args.c_str())) != SICK_SCAN_API_SUCCESS)
    exitOnError("SickScanApiInitByLaunchfile failed", ret);
#else
  if ((ret = SickScanApiInitByCli(apiHandle, argc, argv)) != SICK_SCAN_API_SUCCESS)
    exitOnError("SickScanApiInitByCli failed", ret);
#endif

  bool run_polling = polling;
  std::thread* run_polling_thread = 0;
  if (polling) // Receive lidar message by SickScanApiWaitNext-functions running in a background thread ("message polling")
  {
    run_polling_thread = new std::thread(runSickScanApiTestWaitNext, &apiHandle, &run_polling);
  }
  else
  {
    // Register a callback for PointCloud messages
    if ((ret = SickScanApiRegisterCartesianPointCloudMsg(apiHandle, apiTestCartesianPointCloudMsgCallback)) != SICK_SCAN_API_SUCCESS)
      exitOnError("SickScanApiRegisterCartesianPointCloudMsg failed", ret);
    if ((ret = SickScanApiRegisterPolarPointCloudMsg(apiHandle, apiTestPolarPointCloudMsgCallback)) != SICK_SCAN_API_SUCCESS)
      exitOnError("SickScanApiRegisterCartesianPointCloudMsg failed", ret);

    // Register a callback for Imu messages
    if ((ret = SickScanApiRegisterImuMsg(apiHandle, apiTestImuMsgCallback)) != SICK_SCAN_API_SUCCESS)
      exitOnError("SickScanApiRegisterImuMsg failed", ret);

    // Register a callback for LFErec messages
    if ((ret = SickScanApiRegisterLFErecMsg(apiHandle, apiTestLFErecMsgCallback)) != SICK_SCAN_API_SUCCESS)
      exitOnError("SickScanApiRegisterLFErecMsg failed", ret);

    // Register a callback for LIDoutputstate messages
    if ((ret = SickScanApiRegisterLIDoutputstateMsg(apiHandle, apiTestLIDoutputstateMsgCallback)) != SICK_SCAN_API_SUCCESS)
      exitOnError("SickScanApiRegisterLIDoutputstateMsg failed", ret);

    // Register a callback for RadarScan messages
    if ((ret = SickScanApiRegisterRadarScanMsg(apiHandle, apiTestRadarScanMsgCallback)) != SICK_SCAN_API_SUCCESS)
      exitOnError("SickScanApiRegisterRadarScanMsg failed", ret);

    // Register a callback for LdmrsObjectArray messages
    if ((ret = SickScanApiRegisterLdmrsObjectArrayMsg(apiHandle, apiTestLdmrsObjectArrayCallback)) != SICK_SCAN_API_SUCCESS)
      exitOnError("SickScanApiRegisterLdmrsObjectArrayMsg failed", ret);

    // Register a callback for VisualizationMarker messages
    if ((ret = SickScanApiRegisterVisualizationMarkerMsg(apiHandle, apiTestVisualizationMarkerMsgCallback)) != SICK_SCAN_API_SUCCESS)
      exitOnError("SickScanApiRegisterVisualizationMarkerMsg failed", ret);

    // Register a callback for NAV350 Pose- and Landmark messages messages
    if ((ret = SickScanApiRegisterNavPoseLandmarkMsg(apiHandle, apiTestNavPoseLandmarkMsgCallback)) != SICK_SCAN_API_SUCCESS)
      exitOnError("SickScanApiRegisterVisualizationSickScanApiRegisterNavPoseLandmarkMsgMarkerMsg failed", ret);

    // Register a callback for diagnostic messages (notification in case of changed status, e.g. after errors)
    if ((ret = SickScanApiRegisterDiagnosticMsg(apiHandle, apiTestDiagnosticMsgCallback)) != SICK_SCAN_API_SUCCESS)
      exitOnError("SickScanApiRegisterDiagnosticMsg failed", ret);

    // Register a callback for log messages (all informational and error messages)
    if ((ret = SickScanApiRegisterLogMsg(apiHandle, apiTestLogMsgCallback)) != SICK_SCAN_API_SUCCESS)
      exitOnError("SickScanApiRegisterLogMsg failed", ret);

  }

  // Run main loop
  int user_key = 0;
  char sopas_response_buffer[1024] = { 0 };
  while (true)
  {
#if __ROS_VERSION == 1
    ros::spin();
#elif __ROS_VERSION == 0 && defined _MSC_VER
    while (_kbhit() == 0)
    {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      printf("sick_scan_xd_api_test running. Press ENTER to exit or r for re-initialization\n");
    }
    user_key = _getch();
#else
    user_key = getchar();
    getchar();
#endif
    printf("sick_scan_xd_api_test: user_key = '%c' (%d)\n", (char)user_key, user_key);
    const char* sopas_request = 0;
    if (user_key == 's' || user_key == 'S') // Send sopas command "sRN SCdevicestate", sopas response: "sRA SCdevicestate \x01"
      sopas_request = "sRN SCdevicestate";
    // else if (user_key == 'c' || user_key == 'C') // Send sopas command "sRN ContaminationResult" supported by MRS-1000, LMS-1000, multiScan, sopas response: "sRA ContaminationResult \x00\x00"
    //   sopas_request = "sRN ContaminationResult";
    if (sopas_request) // Send sopas command and continue
    {
      if (SickScanApiSendSOPAS(apiHandle, sopas_request, &sopas_response_buffer[0], (int32_t)sizeof(sopas_response_buffer)) != SICK_SCAN_API_SUCCESS)
        printf("## WARNING sick_scan_xd_api_test: SickScanApiSendSOPAS(\"%s\") failed\n", sopas_request);
      else
        printf("sick_scan_xd_api_test: SickScanApiSendSOPAS(\"%s\") succeeded: response = \"%s\"\n\n", sopas_request, sopas_response_buffer);
    }
    else
    {
      break;
    }
  }

  // Cleanup and exit
  printf("sick_scan_xd_api_test finishing...\n");
  if (polling)
  {
    run_polling = false;
    if ((ret = SickScanApiClose(apiHandle)) != SICK_SCAN_API_SUCCESS)
      exitOnError("SickScanApiClose failed", ret);
    if (run_polling_thread->joinable())
      run_polling_thread->join();
    delete run_polling_thread;
    run_polling_thread = 0;
  }
  else
  {
    SickScanApiDeregisterCartesianPointCloudMsg(apiHandle, apiTestCartesianPointCloudMsgCallback);
    SickScanApiDeregisterPolarPointCloudMsg(apiHandle, apiTestPolarPointCloudMsgCallback);
    SickScanApiDeregisterImuMsg(apiHandle, apiTestImuMsgCallback);
    SickScanApiDeregisterLFErecMsg(apiHandle, apiTestLFErecMsgCallback);
    SickScanApiDeregisterLIDoutputstateMsg(apiHandle, apiTestLIDoutputstateMsgCallback);
    SickScanApiDeregisterRadarScanMsg(apiHandle, apiTestRadarScanMsgCallback);
    SickScanApiDeregisterLdmrsObjectArrayMsg(apiHandle, apiTestLdmrsObjectArrayCallback);
    SickScanApiDeregisterVisualizationMarkerMsg(apiHandle, apiTestVisualizationMarkerMsgCallback);
    SickScanApiDeregisterNavPoseLandmarkMsg(apiHandle, apiTestNavPoseLandmarkMsgCallback);
    if ((ret = SickScanApiClose(apiHandle)) != SICK_SCAN_API_SUCCESS)
      exitOnError("SickScanApiClose failed", ret);
  }
  if ((ret = SickScanApiRelease(apiHandle)) != SICK_SCAN_API_SUCCESS)
    exitOnError("SickScanApiRelease failed", ret);
  if (!polling)
  {
    SickScanApiDeregisterDiagnosticMsg(apiHandle, apiTestDiagnosticMsgCallback);
    SickScanApiDeregisterLogMsg(apiHandle, apiTestLogMsgCallback);
  }

  return user_key;
}

// sick_scan_api_test main: Initialize, receive and process lidar messages via sick_scan_xd API.
int main(int argc, char** argv)
{
  int32_t ret = SICK_SCAN_API_SUCCESS;
  std::string sick_scan_args;
  bool polling = false;

#if __ROS_VERSION == 1
  sick_scan_args = "./src/sick_scan_xd/launch/sick_tim_7xx.launch"; // example launch file
  for (int n = 0; n < argc; n++)
  {
    printf("%s%s", (n > 0 ? " " : ""), argv[n]);
    if (strncmp(argv[n], "_sick_scan_args:=", 17) == 0)
      sick_scan_args = argv[n] + 17;
    if (strncmp(argv[n], "_polling:=", 10) == 0 && atoi(argv[n] + 10) > 0)
      polling = true;
  }
  ros::init(argc, argv, "sick_scan_xd_api_test");
  ros::NodeHandle nh("~");
  ros_api_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>(ros_api_cloud_topic, 10);
  ros_api_cloud_polar_publisher = nh.advertise<sensor_msgs::PointCloud2>(ros_api_cloud_polar_topic, 10);
  ros_api_visualizationmarker_publisher = nh.advertise<visualization_msgs::MarkerArray>(ros_api_visualizationmarker_topic, 10);
#endif
  printf("\nsick_scan_xd_api_test started\n");

#ifdef _MSC_VER
  std::string sick_scan_api_lib = "sick_scan_xd_shared_lib.dll";
  std::vector<std::string> search_library_path = { "", "build/Debug/", "build_win64/Debug/", "src/build/Debug/", "src/build_win64/Debug/", "src/sick_scan_xd/build/Debug/", "src/sick_scan_xd/build_win64/Debug/", "./", "../" };
#else
  std::string sick_scan_api_lib = "libsick_scan_xd_shared_lib.so";
  std::vector<std::string> search_library_path = { "", "build/", "build_linux/", "src/build/", "src/build_linux/", "src/sick_scan_xd/build/", "src/sick_scan_xd/build_linux/", "./", "../" };
#endif
  ret = SICK_SCAN_API_NOT_LOADED;
  for (int search_library_cnt = 0; search_library_cnt < search_library_path.size(); search_library_cnt++)
  {
    std::string libfilepath = search_library_path[search_library_cnt] + sick_scan_api_lib;
    if ((ret = SickScanApiLoadLibrary(libfilepath.c_str())) == SICK_SCAN_API_SUCCESS)
    {
      printf("sick_scan_xd library \"%s\" loaded successfully\n", libfilepath.c_str());
      sick_scan_api_lib = libfilepath;
      break;
    }
  }
  if (ret != SICK_SCAN_API_SUCCESS)
    exitOnError("SickScanApiLoadLibrary failed", ret);

  // (Re-)Initialize and run sick_scan_xd_api_test
  int user_key = 0;
  do
  {
    user_key = sick_scan_api_test_main(argc, argv, sick_scan_args, polling);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    printf("sick_scan_xd_api_test finished with user key '%c' (%d), re-initialize and repeat sick_scan_xd_api_test ...\n", (char)user_key, user_key);
  } while (user_key == 'R' || user_key == 'r');

  // Unload and exit
  printf("sick_scan_xd_api_test finishing...\n");
  if ((ret = SickScanApiUnloadLibrary()) != SICK_SCAN_API_SUCCESS)
    exitOnError("SickScanApiUnloadLibrary failed", ret);
  printf("sick_scan_xd_api_test finished successfully\n");

  if (false) // Optional test: reload, initialize and run sick_scan_xd_api_test
  {
    printf("\nsick_scan_xd_api_test: reload %s\n", sick_scan_api_lib.c_str());
    if ((ret = SickScanApiLoadLibrary(sick_scan_api_lib.c_str())) != SICK_SCAN_API_SUCCESS)
      exitOnError("SickScanApiLoadLibrary failed", ret);
    printf("\nsick_scan_xd_api_test: restart\n");
    int user_key = 0;
    do
    {
      user_key = sick_scan_api_test_main(argc, argv, sick_scan_args, polling);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      printf("sick_scan_xd_api_test finished with user key '%c' (%d), re-initialize and repeat sick_scan_xd_api_test ...\n", (char)user_key, user_key);
    } while (user_key == 'R' || user_key == 'r');
    printf("sick_scan_xd_api_test finishing...\n");
    if ((ret = SickScanApiUnloadLibrary()) != SICK_SCAN_API_SUCCESS)
      exitOnError("SickScanApiUnloadLibrary failed", ret);
    printf("sick_scan_xd_api_test finished successfully\n");
  }

  exit(EXIT_SUCCESS);
}
