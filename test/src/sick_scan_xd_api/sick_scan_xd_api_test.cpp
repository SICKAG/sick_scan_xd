#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <thread>

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
    sick_scan::LFErecMsg ros_msg = SickScanApiConverter::convertLFErecMsg(*msg);
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
    sick_scan::LIDoutputstateMsg ros_msg = SickScanApiConverter::convertLIDoutputstateMsg(*msg);
    DUMP_API_LIDOUTPUTSTATE_MESSAGE("test", ros_msg);
#endif
}

// Example callback for RadarScan messages
static void apiTestRadarScanMsgCallback(SickScanApiHandle apiHandle, const SickScanRadarScan* msg)
{	
	printf("[Info]: apiTestRadarScanMsgCallback(apiHandle:%p): RadarScan message, %d targets, %d objects\n", apiHandle, (int)(msg->targets.width * msg->targets.height), (int)msg->objects.size);
#if __ROS_VERSION == 1
    sick_scan::RadarScan ros_msg = SickScanApiConverter::convertRadarScanMsg(*msg);
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
    sick_scan::SickLdmrsObjectArray ros_msg = SickScanApiConverter::convertLdmrsObjectArray(*msg);
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

// Receive lidar message by SickScanApiWaitNext-functions ("message polling")
static void runSickScanApiWaitNext(SickScanApiHandle* apiHandle, bool* run_flag)
{
	double wait_next_message_timeout = 0.1; // wait max. 0.1 seconds for the next message (otherwise SickScanApiWaitNext-function return with timeout)
    SickScanPointCloudMsg pointcloud_msg;
	SickScanVisualizationMarkerMsg visualizationmarker_msg;
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

		// Get/poll the next VisualizationMarker message
		ret = SickScanApiWaitNextVisualizationMarkerMsg(*apiHandle, &visualizationmarker_msg, wait_next_message_timeout);
		if (ret == SICK_SCAN_API_SUCCESS)
            apiTestVisualizationMarkerMsgCallback(*apiHandle, &visualizationmarker_msg);
		else if (ret != SICK_SCAN_API_SUCCESS && ret != SICK_SCAN_API_TIMEOUT)
			printf("## ERROR sick_scan_xd_api_test: SickScanApiWaitNextVisualizationMarkerMsg failed\n");
		SickScanApiFreeVisualizationMarkersg(*apiHandle, &visualizationmarker_msg);
	}
}

// sick_scan_api_test main: Initialize, receive and process lidar messages via sick_scan_xd API.
int main(int argc, char** argv)
{
	int32_t ret = SICK_SCAN_API_SUCCESS;
	SickScanApiHandle apiHandle = 0;
    std::stringstream cli_params;
	bool polling = false;

#if __ROS_VERSION == 1
    std::string sick_scan_args = "./src/sick_scan_xd/launch/sick_tim_7xx.launch"; // example launch file
    for(int n = 0; n < argc; n++)
	{
        printf("%s%s", (n > 0 ? " ": ""), argv[n]);
		if (strncmp(argv[n],"_sick_scan_args:=", 17) == 0)
		    sick_scan_args = argv[n] + 17;
		if (strncmp(argv[n],"_polling:=", 10) == 0 && atoi(argv[n] + 10) > 0)
		    polling = true;
	}
    ros::init(argc, argv, "sick_scan_xd_api_test");
    ros::NodeHandle nh("~");
	ros_api_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>(ros_api_cloud_topic, 10);
	ros_api_cloud_polar_publisher = nh.advertise<sensor_msgs::PointCloud2>(ros_api_cloud_polar_topic, 10);
	ros_api_visualizationmarker_publisher = nh.advertise<visualization_msgs::MarkerArray>(ros_api_visualizationmarker_topic, 10);
#endif
	printf("\nsick_scan_xd_api_test started\n");

    const char* sick_scan_api_lib = "libsick_scan_shared_lib.so";
    if((ret = SickScanApiLoadLibrary(sick_scan_api_lib)) != SICK_SCAN_API_SUCCESS)
	    exitOnError("SickScanApiLoadLibrary failed", ret);

    if((apiHandle = SickScanApiCreate(argc, argv)) == 0)
	    exitOnError("SickScanApiCreate failed", -1);

    // Initialize a lidar and starts message receiving and processing
#if __ROS_VERSION == 1
    if((ret = SickScanApiInitByLaunchfile(apiHandle, sick_scan_args.c_str())) != SICK_SCAN_API_SUCCESS)
	    exitOnError("SickScanApiInitByLaunchfile failed", ret);
#else	 
    if((ret = SickScanApiInitByCli(apiHandle, argc, argv)) != SICK_SCAN_API_SUCCESS)
	    exitOnError("SickScanApiInitByCli failed", ret);
#endif	 

	bool run_polling = polling;
	std::thread* run_polling_thread = 0;
    if (polling) // Receive lidar message by SickScanApiWaitNext-functions running in a background thread ("message polling")
	{
		run_polling_thread = new std::thread(runSickScanApiWaitNext, &apiHandle, &run_polling);
	}
	else
	{
		// Register a callback for PointCloud messages
		if((ret = SickScanApiRegisterCartesianPointCloudMsg(apiHandle, apiTestCartesianPointCloudMsgCallback)) != SICK_SCAN_API_SUCCESS)
			exitOnError("SickScanApiRegisterCartesianPointCloudMsg failed", ret);
		if((ret = SickScanApiRegisterPolarPointCloudMsg(apiHandle, apiTestPolarPointCloudMsgCallback)) != SICK_SCAN_API_SUCCESS)
			exitOnError("SickScanApiRegisterCartesianPointCloudMsg failed", ret);

		// Register a callback for Imu messages
		if((ret = SickScanApiRegisterImuMsg(apiHandle, apiTestImuMsgCallback)) != SICK_SCAN_API_SUCCESS)
			exitOnError("SickScanApiRegisterImuMsg failed", ret);

		// Register a callback for LFErec messages
		if((ret = SickScanApiRegisterLFErecMsg(apiHandle, apiTestLFErecMsgCallback)) != SICK_SCAN_API_SUCCESS)
			exitOnError("SickScanApiRegisterLFErecMsg failed", ret);

		// Register a callback for LIDoutputstate messages
		if((ret = SickScanApiRegisterLIDoutputstateMsg(apiHandle, apiTestLIDoutputstateMsgCallback)) != SICK_SCAN_API_SUCCESS)
			exitOnError("SickScanApiRegisterLIDoutputstateMsg failed", ret);

		// Register a callback for RadarScan messages
		if((ret = SickScanApiRegisterRadarScanMsg(apiHandle, apiTestRadarScanMsgCallback)) != SICK_SCAN_API_SUCCESS)
			exitOnError("SickScanApiRegisterRadarScanMsg failed", ret);

		// Register a callback for LdmrsObjectArray messages
		if((ret = SickScanApiRegisterLdmrsObjectArrayMsg(apiHandle, apiTestLdmrsObjectArrayCallback)) != SICK_SCAN_API_SUCCESS)
			exitOnError("SickScanApiRegisterLdmrsObjectArrayMsg failed", ret);

		// Register a callback for VisualizationMarker messages
		if((ret = SickScanApiRegisterVisualizationMarkerMsg(apiHandle, apiTestVisualizationMarkerMsgCallback)) != SICK_SCAN_API_SUCCESS)
			exitOnError("SickScanApiRegisterVisualizationMarkerMsg failed", ret);
	}

    // Run main loop
#if __ROS_VERSION == 1
    ros::spin();
#else	 
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
	}
    if((ret = SickScanApiClose(apiHandle)) != SICK_SCAN_API_SUCCESS)
	    exitOnError("SickScanApiClose failed", ret);
    if((ret = SickScanApiRelease(apiHandle)) != SICK_SCAN_API_SUCCESS)
	    exitOnError("SickScanApiRelease failed", ret);
    if((ret = SickScanApiUnloadLibrary()) != SICK_SCAN_API_SUCCESS)
	    exitOnError("SickScanApiUnloadLibrary failed", ret);
	printf("sick_scan_xd_api_test finished successfully\n");
	exit(EXIT_SUCCESS);
}
