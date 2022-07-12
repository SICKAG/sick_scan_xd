#include <signal.h>
#include <iomanip>
#include <vector>
#include <string>
#include <sstream>

#include "sick_scan_api.h"
#include "sick_scan/sick_generic_laser.h"
#include <sick_scan/sick_generic_callback.h>

static std::string s_scannerName = "sick_scan";
static std::map<SickScanApiHandle,std::string> s_api_caller;
static std::vector<void*> s_malloced_resources;


static SickScanApiHandle castNodeToApiHandle(rosNodePtr node)
{
    return ((SickScanApiHandle)(&(*node))); // return ((SickScanApiHandle)node);
}

static rosNodePtr castApiHandleToNode(SickScanApiHandle apiHandle)
{
    return (*((rosNodePtr*)&apiHandle)); // return ((rosNodePtr)apiHandle);
}

/*
*  Message converter
*/
static SickScanPointCloudMsg convertPointCloudMsg(const ros_sensor_msgs::PointCloud2& msg)
{
    SickScanPointCloudMsg export_msg;
    memset(&export_msg, 0, sizeof(export_msg));
    // Copy header and pointcloud dimension
    ROS_HEADER_SEQ(export_msg.header, msg.header.seq); // export_msg.header.seq = msg.header.seq;
    export_msg.header.timestamp_sec = sec(msg.header.stamp); // msg.header.stamp.sec;
    export_msg.header.timestamp_nsec = nsec(msg.header.stamp); // msg.header.stamp.nsec;
    strncpy(export_msg.header.frame_id, msg.header.frame_id.c_str(), sizeof(export_msg.header.frame_id) - 2);
    export_msg.width = msg.width;
    export_msg.height = msg.height;
    export_msg.is_bigendian = msg.is_bigendian;
    export_msg.is_dense = msg.is_dense;
    export_msg.point_step = msg.point_step;
    export_msg.row_step = msg.row_step;
    // Copy field descriptions
    int num_fields = msg.fields.size();
    std::vector<SickScanPointFieldMsg> export_fields(num_fields);
    for(int n = 0; n < num_fields; n++)
    {
        SickScanPointFieldMsg export_field;
        memset(&export_field, 0, sizeof(export_field));
        strncpy(export_field.name, msg.fields[n].name.c_str(), sizeof(export_field.name) - 2);
        export_field.offset = msg.fields[n].offset;
        export_field.datatype = msg.fields[n].datatype;
        export_field.count = msg.fields[n].count;
        export_fields[n] = export_field;
    }
    export_msg.fields.buffer = (SickScanPointFieldMsg*)malloc(num_fields * sizeof(SickScanPointFieldMsg));
    if (export_msg.fields.buffer != 0)
    {
        export_msg.fields.size = num_fields;
        export_msg.fields.capacity = num_fields;
        memcpy(export_msg.fields.buffer, export_fields.data(), num_fields * sizeof(SickScanPointFieldMsg));
    }
    // Copy pointcloud data
    export_msg.data.buffer = (uint8_t*)malloc(msg.row_step * msg.height);
    if (export_msg.data.buffer != 0)
    {
        export_msg.data.size = msg.row_step * msg.height;
        export_msg.data.capacity = msg.row_step * msg.height;
        memcpy(export_msg.data.buffer, msg.data.data(), msg.row_step * msg.height);
    }
    // Return converted pointcloud
    return export_msg;
}

static void freePointCloudMsg(SickScanPointCloudMsg& export_msg)
{
    if (export_msg.fields.buffer != 0)
        free(export_msg.fields.buffer);
    if (export_msg.data.buffer != 0)
        free(export_msg.data.buffer);
    memset(&export_msg, 0, sizeof(export_msg));
}

/*
*  Callback handler
*/
static sick_scan::SickCallbackHandler<SickScanApiHandle,SickScanPointCloudMsg> s_callback_handler_pointcloud_messages;

static void pointcloud_callback(rosNodePtr node, const ros_sensor_msgs::PointCloud2* msg)
{
    ROS_INFO_STREAM("api_impl pointcloud_callback: PointCloud2 message, " << msg->width << "x" << msg->height << " points");
    // Convert ros_sensor_msgs::PointCloud2 message to SickScanPointCloudMsg and export (i.e. notify all listeners)
    SickScanPointCloudMsg export_msg = convertPointCloudMsg(*msg);
    SickScanApiHandle apiHandle = castNodeToApiHandle(node);
    s_callback_handler_pointcloud_messages.notifyListener(apiHandle, &export_msg);
    freePointCloudMsg(export_msg);
}

/*
*  Functions to initialize and close the API and a lidar
*/

/*
*  Create an instance of sick_scan_xd api.
*  Optional commandline arguments argc, argv identical to sick_generic_caller.
*  Call SickScanApiInitByLaunchfile or SickScanApiInitByCli to process a lidar.
*/
SickScanApiHandle SickScanApiCreate(int argc, char** argv)
{
    std::string versionInfo = std::string("sick_scan_api V. ") + getVersionInfo();
    setVersionInfo(versionInfo);

#if defined __ROS_VERSION && __ROS_VERSION == 2
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    rosNodePtr node = rclcpp::Node::make_shared("sick_scan", "", node_options);
    SickScanApiHandle apiHandle = castNodeToApiHandle(node);
#else
    ros::init(argc, argv, s_scannerName, ros::init_options::NoSigintHandler);
    SickScanApiHandle apiHandle = new ros::NodeHandle("~");
#endif

    signal(SIGINT, rosSignalHandler);
    ROS_INFO_STREAM(versionInfo);

    if (argc > 0 && argv != 0 && argv[0] != 0)
        s_api_caller[apiHandle] = argv[0];
    return apiHandle;
}

// Release and free all resources of a handle; the handle is invalid after SickScanApiRelease
int32_t SickScanApiRelease(SickScanApiHandle apiHandle)
{
    for(int n = 0; n < s_malloced_resources.size(); n++)
        free(s_malloced_resources[n]);
    s_malloced_resources.clear();
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Initializes a lidar by launchfile and starts message receiving and processing
int32_t SickScanApiInitByLaunchfile(SickScanApiHandle apiHandle, const char* launchfile_args)
{
    if (apiHandle == 0)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiInitByLaunchfile(" << launchfile_args << "): invalid apiHandle");
        return SICK_SCAN_API_NOT_INITIALIZED;
    }
    // Split launchfile_args by spaces
    ROS_INFO_STREAM("SickScanApiInitByLaunchfile: launchfile_args = \"" << launchfile_args << "\"");
    std::istringstream args_stream(launchfile_args);
    std::string arg;
    std::vector<std::string> args;
    while (getline(args_stream, arg, ' ' ))
        args.push_back(arg);
    // Convert to argc, argv
    int argc = args.size() + 1;
    char** argv = (char**)malloc(argc * sizeof(char*));
    s_malloced_resources.push_back(argv);
    argv[0] = (char*)malloc(s_api_caller[apiHandle].size() + 1);
    strcpy(argv[0], s_api_caller[apiHandle].c_str());
    s_malloced_resources.push_back(argv[0]);
    for(int n = 1; n < argc; n++)
    {
        argv[n] = (char*)malloc(strlen(args[n-1].c_str()) + 1);
        strcpy(argv[n], args[n-1].c_str());
        s_malloced_resources.push_back(argv[n]);
    }
    // Init using SickScanApiInitByCli
    int32_t ret = SickScanApiInitByCli(apiHandle, argc, argv);
    return ret;
}

// Initializes a lidar by commandline arguments and starts message receiving and processing
int32_t SickScanApiInitByCli(SickScanApiHandle apiHandle, int argc, char** argv)
{
    if (apiHandle == 0)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiInitByCli(): invalid apiHandle");
        return SICK_SCAN_API_NOT_INITIALIZED;
    }
    std::stringstream cli_params;
    for(int n = 0; n < argc; n++)
        cli_params << (n > 0 ? " ": "")  << argv[n];
    ROS_INFO_STREAM("SickScanApiInitByCli: " << cli_params.str());
    
    // Start sick_scan event loop
    int exit_code = 0;
    rosNodePtr node = castApiHandleToNode(apiHandle);
    if (!startGenericLaser(argc, argv, s_scannerName, node, &exit_code) || exit_code != sick_scan::ExitSuccess)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiInitByCli(): startGenericLaser() failed, could not start generic laser event loop");
        return SICK_SCAN_API_ERROR;
    }
    return SICK_SCAN_API_SUCCESS;
}

// Stops message receiving and processing and closes a lidar
int32_t SickScanApiClose(SickScanApiHandle apiHandle)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

/*
*  Registration / deregistration of message callbacks
*/

// Register / deregister a callback for cartesian PointCloud messages, pointcloud in cartesian coordinates with fields x, y, z, intensity
int32_t SickScanApiRegisterCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback)
{
    if (apiHandle == 0)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterCartesianPointCloudMsg(): invalid apiHandle");
        return SICK_SCAN_API_NOT_INITIALIZED;
    }
    s_callback_handler_pointcloud_messages.addListener(apiHandle, callback);
    rosNodePtr node = castApiHandleToNode(apiHandle);
    sick_scan::addPointcloudListener(node, pointcloud_callback);
    return SICK_SCAN_API_SUCCESS;
}
int32_t SickScanApiDeregisterCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Register / deregister a callback for polar PointCloud messages, pointcloud in polar coordinates with fields range, azimuth, elevation, intensity
int32_t SickScanApiRegisterPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiDeregisterPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Register / deregister a callback for Imu messages
int32_t SickScanApiRegisterImuMsg(SickScanApiHandle apiHandle, SickScanImuMsgCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiDeregisterImuMsg(SickScanApiHandle apiHandle, SickScanImuMsgCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Register / deregister a callback for SickScanLFErecMsg messages
int32_t SickScanApiRegisterLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsgCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiDeregisterLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsgCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Register / deregister a callback for SickScanLIDoutputstateMsg messages
int32_t SickScanApiRegisterLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsgCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiDeregisterLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsgCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Register / deregister a callback for SickScanRadarScan messages
int32_t SickScanApiRegisterRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScanCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiDeregisterRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScanCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Register / deregister a callback for SickScanLdmrsObjectArray messages
int32_t SickScanApiRegisterLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArrayCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiDeregisterLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArrayCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

/*
*  Polling functions
*/

// Wait for and return the next cartesian resp. polar PointCloud messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg, double timeout_sec)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiWaitNextPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg, double timeout_sec)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiFreePolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Wait for and return the next Imu messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextImuMsg(SickScanApiHandle apiHandle, SickScanImuMsg* msg, double timeout_sec)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiFreeImuMsg(SickScanApiHandle apiHandle, SickScanImuMsg* msg)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Wait for and return the next LFErec messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsg* msg, double timeout_sec)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiFreeLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsg* msg)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Wait for and return the next LIDoutputstate messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsg* msg, double timeout_sec)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiFreeLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsg* msg)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Wait for and return the next RadarScan messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScan* msg, double timeout_sec)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiFreeRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScan* msg)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Wait for and return the next LdmrsObjectArray messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArray* msg, double timeout_sec)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiFreeLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArray* msg)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
