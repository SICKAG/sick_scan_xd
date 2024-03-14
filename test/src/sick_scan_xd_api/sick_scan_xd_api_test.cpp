#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <thread>
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
#include "toojpeg.h"
static FILE *foutJpg = 0;
#endif

#if __ROS_VERSION != 1
// jpeg callback, just writes one byte
void jpegOutputCallback(unsigned char oneByte)
{
    assert(foutJpg != 0);
    fwrite(&oneByte, 1, 1, foutJpg);
}
// Simple plot function for pointcloud data, just demonstrates how to use a SickScanPointCloudMsg
static void plotPointcloudToJpeg(const std::string& jpegfilepath, const SickScanPointCloudMsg& msg)
{
    // Get offsets for x, y, z, intensity values
    SickScanPointFieldMsg* msg_fields_buffer = (SickScanPointFieldMsg*)msg.fields.buffer;
    int field_offset_x = -1, field_offset_y = -1, field_offset_z = -1, field_offset_intensity = -1;
    for(int n = 0; n < msg.fields.size; n++)
    {
        if (strcmp(msg_fields_buffer[n].name, "x") == 0 && msg_fields_buffer[n].datatype == SICK_SCAN_POINTFIELD_DATATYPE_FLOAT32)
            field_offset_x = msg_fields_buffer[n].offset;
        else if (strcmp(msg_fields_buffer[n].name, "y") == 0 && msg_fields_buffer[n].datatype == SICK_SCAN_POINTFIELD_DATATYPE_FLOAT32)
            field_offset_y = msg_fields_buffer[n].offset;
        else if (strcmp(msg_fields_buffer[n].name, "z") == 0 && msg_fields_buffer[n].datatype == SICK_SCAN_POINTFIELD_DATATYPE_FLOAT32)
            field_offset_z = msg_fields_buffer[n].offset;
        else if (strcmp(msg_fields_buffer[n].name, "intensity") == 0 && msg_fields_buffer[n].datatype == SICK_SCAN_POINTFIELD_DATATYPE_FLOAT32)
            field_offset_intensity = msg_fields_buffer[n].offset;
    }
	assert(field_offset_x >= 0 && field_offset_y >= 0 && field_offset_z >= 0);
	// Create an image with 250 pixel/meter, max. +/-2 meter
	int img_width = 250 * 4, img_height = 250 * 4;
	uint8_t* img_pixel = (uint8_t*)calloc(3 * img_width * img_height, sizeof(uint8_t)); // allocate 3 byte RGB array
	// Plot all points in pointcloud
    for (int row_idx = 0; row_idx < (int)msg.height; row_idx++)
    {
        for (int col_idx = 0; col_idx < (int)msg.width; col_idx++)
        {
            // Get cartesian point coordinates
            int polar_point_offset = row_idx * msg.row_step + col_idx * msg.point_step;
            float point_x = *((float*)(msg.data.buffer + polar_point_offset + field_offset_x));
            float point_y = *((float*)(msg.data.buffer + polar_point_offset + field_offset_y));
            float point_z = *((float*)(msg.data.buffer + polar_point_offset + field_offset_z));
            float point_intensity = 0;
            if (field_offset_intensity >= 0)
                point_intensity = *((float*)(msg.data.buffer + polar_point_offset + field_offset_intensity));
			// Convert point coordinates in meter to image coordinates in pixel
			int img_x = (int)(250.0f * (-point_y + 2.0f)); // img_x := -pointcloud.y
			int img_y = (int)(250.0f * (-point_x + 2.0f)); // img_y := -pointcloud.x
			if (img_x >= 0 && img_x < img_width && img_y >= 0 && img_y < img_height) // point within the image area
			{
				img_pixel[3 * img_y * img_width + 3 * img_x + 0] = 255; // R
				img_pixel[3 * img_y * img_width + 3 * img_x + 1] = 255; // G
				img_pixel[3 * img_y * img_width + 3 * img_x + 2] = 255; // B
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
  else if (msg->status_code == 1) // status_code defined in SICK_DIAGNOSTIC_STATUS: ERROR=2
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
  else
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
	while(run_flag && *run_flag)
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

  // Cleanup and exit
  printf("sick_scan_xd_api_test finishing...\n");
  if (polling)
  {
    run_polling = false;
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
  }
  if ((ret = SickScanApiClose(apiHandle)) != SICK_SCAN_API_SUCCESS)
    exitOnError("SickScanApiClose failed", ret);
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
  const char* sick_scan_api_lib = "sick_scan_xd_shared_lib.dll";
#else
  const char* sick_scan_api_lib = "libsick_scan_xd_shared_lib.so";
#endif
  if ((ret = SickScanApiLoadLibrary(sick_scan_api_lib)) != SICK_SCAN_API_SUCCESS)
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
  exit(EXIT_SUCCESS);
}





