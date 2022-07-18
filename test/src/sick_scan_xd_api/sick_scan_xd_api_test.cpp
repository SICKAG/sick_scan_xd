#include <stdio.h>
#include <stdlib.h>
#include <sstream>

#include "sick_scan_api.h"
#include "sick_scan_api_converter.h"

#if __ROS_VERSION == 1
std::string ros_api_cloud_topic = "api_cloud";
std::string ros_api_cloud_polar_topic = "api_cloud_polar";
ros::Publisher ros_api_cloud_publisher;
ros::Publisher ros_api_cloud_polar_publisher;
#endif

static void exitOnError(const char* msg, int32_t error_code)
{
	printf("## ERROR sick_scan_xd_api_test: %s, error code %d\n", msg, error_code);
	exit(EXIT_FAILURE);
}

// typedef void(* SickScanPointCloudMsgCallback)(SickScanApiHandle apiHandle, const SickScanPointCloudMsg* msg);

// Callback for cartesian pointcloud messages, converts and publishes a SickScanPointCloudMsg to sensor_msgs::PointCloud2 on ROS-1
static void apiTestCartesianPointCloudMsgCallback(SickScanApiHandle apiHandle, const SickScanPointCloudMsg* msg)
{	
	printf("[Info]: apiTestCartesianPointCloudMsgCallback(apiHandle:%p): %dx%d pointcloud callback...\n", apiHandle, msg->width, msg->height);
#if __ROS_VERSION == 1
    sensor_msgs::PointCloud2 pointcloud = SickScanApiConverter::convertPointCloudMsg(*msg);
	ros_api_cloud_publisher.publish(pointcloud);
	ROS_INFO_STREAM("apiTestCartesianPointCloudMsgCallback(apiHandle:" << apiHandle << "): published " << pointcloud.width << "x" << pointcloud.height << " pointcloud on topic \"" << ros_api_cloud_topic << "\"");
#endif
}

// Callback for polar pointcloud messages, converts and publishes a SickScanPointCloudMsg to sensor_msgs::PointCloud2 on ROS-1
static void apiTestPolarPointCloudMsgCallback(SickScanApiHandle apiHandle, const SickScanPointCloudMsg* msg)
{	
	printf("[Info]: apiTestPolarPointCloudMsgCallback(apiHandle:%p): %dx%d pointcloud callback...\n", apiHandle, msg->width, msg->height);
#if __ROS_VERSION == 1
    sensor_msgs::PointCloud2 pointcloud = SickScanApiConverter::convertPolarPointCloudMsg(*msg);
	ros_api_cloud_polar_publisher.publish(pointcloud);
	ROS_INFO_STREAM("apiTestPolarPointCloudMsgCallback(apiHandle:" << apiHandle << "): published " << pointcloud.width << "x" << pointcloud.height << " pointcloud on topic \"" << ros_api_cloud_polar_topic << "\"");
#endif
}

// Callback for imu messages
static void apiTestImMsgCallback(SickScanApiHandle apiHandle, const SickScanImuMsg* msg)
{	
	printf("[Info]: apiTestImMsgCallback(apiHandle:%p): Imu message, orientation=(%.6f,%.6f,%.6f,%.6f), angular_velocity=(%.6f,%.6f,%.6f), linear_acceleration=(%.6f,%.6f,%.6f)\n", 
	    apiHandle, msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w, 
        msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.y, 
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
}

int main(int argc, char** argv)
{
	int32_t ret = SICK_SCAN_API_SUCCESS;
	SickScanApiHandle apiHandle = 0;
    std::stringstream cli_params;

#if __ROS_VERSION == 1
    std::string sick_scan_args = "./src/sick_scan_xd/launch/sick_tim_7xx.launch"; // example launch file
    for(int n = 0; n < argc; n++)
	{
        printf("%s%s", (n > 0 ? " ": ""), argv[n]);
		if (strncmp(argv[n],"_sick_scan_args:=", 17) == 0)
		    sick_scan_args = argv[n] + 17;
	}
    ros::init(argc, argv, "sick_scan_xd_api_test");
    ros::NodeHandle nh("~");
	ros_api_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>(ros_api_cloud_topic, 10);
	ros_api_cloud_polar_publisher = nh.advertise<sensor_msgs::PointCloud2>(ros_api_cloud_polar_topic, 10);
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

    // Register a callback for PointCloud messages
    if((ret = SickScanApiRegisterCartesianPointCloudMsg(apiHandle, apiTestCartesianPointCloudMsgCallback)) != SICK_SCAN_API_SUCCESS)
	    exitOnError("SickScanApiRegisterCartesianPointCloudMsg failed", ret);
    if((ret = SickScanApiRegisterPolarPointCloudMsg(apiHandle, apiTestPolarPointCloudMsgCallback)) != SICK_SCAN_API_SUCCESS)
	    exitOnError("SickScanApiRegisterCartesianPointCloudMsg failed", ret);

    // Register a callback for Imu messages
    if((ret = SickScanApiRegisterImuMsg(apiHandle, apiTestImMsgCallback)) != SICK_SCAN_API_SUCCESS)
	    exitOnError("SickScanApiRegisterImuMsg failed", ret);

    // Run main loop
#if __ROS_VERSION == 1
    ros::spin();
#else	 
	getchar();
#endif	 

    // Cleanup and exit
	printf("sick_scan_xd_api_test finishing...\n");
    if((ret = SickScanApiDeregisterCartesianPointCloudMsg(apiHandle, apiTestCartesianPointCloudMsgCallback)) != SICK_SCAN_API_SUCCESS)
	    exitOnError("SickScanApiDeregisterCartesianPointCloudMsg failed", ret);
    if((ret = SickScanApiDeregisterPolarPointCloudMsg(apiHandle, apiTestPolarPointCloudMsgCallback)) != SICK_SCAN_API_SUCCESS)
	    exitOnError("SickScanApiDeregisterPolarPointCloudMsg failed", ret);
    if((ret = SickScanApiDeregisterImuMsg(apiHandle, apiTestImMsgCallback)) != SICK_SCAN_API_SUCCESS)
	    exitOnError("SickScanApiDeregisterImuMsg failed", ret);
    if((ret = SickScanApiClose(apiHandle)) != SICK_SCAN_API_SUCCESS)
	    exitOnError("SickScanApiClose failed", ret);
    if((ret = SickScanApiRelease(apiHandle)) != SICK_SCAN_API_SUCCESS)
	    exitOnError("SickScanApiRelease failed", ret);
    if((ret = SickScanApiUnloadLibrary()) != SICK_SCAN_API_SUCCESS)
	    exitOnError("SickScanApiUnloadLibrary failed", ret);
	printf("sick_scan_xd_api_test finished successfully\n");
	exit(EXIT_SUCCESS);
}
