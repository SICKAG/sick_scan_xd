#include <exception>
#include <iomanip>
#include <memory>
#include <signal.h>
#include <sstream>
#include <string>
#include <vector>

#include "softwarePLL.h"
#include "sick_scan_api.h"
#include "sick_scan_api_dump.h"
#include "sick_scan/sick_generic_laser.h"
#include "sick_scan/sick_generic_callback.h"
#include "sick_scan/sick_scan_logging.h"

template <typename HandleType, class MsgType> std::list<sick_scan_xd::SickWaitForMessageHandler<HandleType, MsgType>*> sick_scan_xd::SickWaitForMessageHandler<HandleType, MsgType>::s_wait_for_message_handler_list;
template <typename HandleType, class MsgType> std::mutex sick_scan_xd::SickWaitForMessageHandler<HandleType, MsgType>::s_wait_for_message_handler_mutex;

static std::string s_scannerName = "sick_scan";
static std::map<SickScanApiHandle,std::string> s_api_caller;
static int s_argc = 0;
static char** s_argv = 0; // deep copy of argv commandline arguments
static std::vector<void*> s_malloced_resources;
static sick_scan_xd::SickCallbackHandler<SickScanApiHandle,SickScanPointCloudMsg>          s_callback_handler_cartesian_pointcloud_messages;
static sick_scan_xd::SickCallbackHandler<SickScanApiHandle,SickScanPointCloudMsg>          s_callback_handler_polar_pointcloud_messages;
static sick_scan_xd::SickCallbackHandler<SickScanApiHandle,SickScanImuMsg>                 s_callback_handler_imu_messages;
static sick_scan_xd::SickCallbackHandler<SickScanApiHandle,SickScanLFErecMsg>              s_callback_handler_lferec_messages;
static sick_scan_xd::SickCallbackHandler<SickScanApiHandle,SickScanLIDoutputstateMsg>      s_callback_handler_lidoutputstate_messages;
static sick_scan_xd::SickCallbackHandler<SickScanApiHandle,SickScanRadarScan>              s_callback_handler_radarscan_messages;
static sick_scan_xd::SickCallbackHandler<SickScanApiHandle,SickScanLdmrsObjectArray>       s_callback_handler_ldmrsobjectarray_messages;
static sick_scan_xd::SickCallbackHandler<SickScanApiHandle,SickScanVisualizationMarkerMsg> s_callback_handler_visualizationmarker_messages;
static sick_scan_xd::SickCallbackHandler<SickScanApiHandle,SickScanNavPoseLandmarkMsg>     s_callback_handler_navposelandmark_messages;
static sick_scan_xd::SickCallbackHandler<SickScanApiHandle,SickScanDiagnosticMsg>          s_callback_handler_diagnostic_messages;
static sick_scan_xd::SickCallbackHandler<SickScanApiHandle,SickScanLogMsg>                 s_callback_handler_log_messages;

#if __ROS_VERSION == 2 // workaround for missing imu quaternion operator << in ROS2
#   define ROS_VECTOR3D_TO_STREAM(msg)   ((msg).x) << "," << ((msg).y) << "," << ((msg).z)
#   define ROS_QUATERNION_TO_STREAM(msg) ((msg).x) << "," << ((msg).y) << "," << ((msg).z) << "," << ((msg).w)
#else
#   define ROS_VECTOR3D_TO_STREAM(msg) (msg)
#   define ROS_QUATERNION_TO_STREAM(msg) (msg)
#endif

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

static SickScanPointCloudMsg convertPointCloudMsg(const sick_scan_xd::PointCloud2withEcho& msg_with_echo)
{
    SickScanPointCloudMsg export_msg;
    memset(&export_msg, 0, sizeof(export_msg));
    // Copy header and pointcloud dimension
    const ros_sensor_msgs::PointCloud2& msg = msg_with_echo.pointcloud;
    ROS_HEADER_SEQ(export_msg.header, msg.header.seq); // export_msg.header.seq = msg.header.seq;
    export_msg.header.timestamp_sec = sec(msg.header.stamp); // msg.header.stamp.sec;
    export_msg.header.timestamp_nsec = nsec(msg.header.stamp); // msg.header.stamp.nsec;
    strncpy(export_msg.header.frame_id, msg.header.frame_id.c_str(), sizeof(export_msg.header.frame_id) - 2);
    strncpy(export_msg.topic, msg_with_echo.topic.c_str(), sizeof(export_msg.topic) - 2);   
    export_msg.width = msg.width;
    export_msg.height = msg.height;
    export_msg.is_bigendian = msg.is_bigendian;
    export_msg.is_dense = msg.is_dense;
    export_msg.point_step = msg.point_step;
    export_msg.row_step = msg.row_step;
    export_msg.num_echos = msg_with_echo.num_echos;
    export_msg.segment_idx = msg_with_echo.segment_idx;
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

static SickScanImuMsg convertImuMsg(const ros_sensor_msgs::Imu& src_msg, const std::string& topic = sick_scan_xd::getImuTopic())
{
    SickScanImuMsg dst_msg;
    memset(&dst_msg, 0, sizeof(dst_msg));
    // Copy header
    ROS_HEADER_SEQ(dst_msg.header, src_msg.header.seq);
    dst_msg.header.timestamp_sec = sec(src_msg.header.stamp);
    dst_msg.header.timestamp_nsec = nsec(src_msg.header.stamp);
    strncpy(dst_msg.header.frame_id, src_msg.header.frame_id.c_str(), sizeof(dst_msg.header.frame_id) - 2);
    strncpy(dst_msg.topic, topic.c_str(), sizeof(dst_msg.topic) - 2);   
    // Copy imu data
    dst_msg.orientation.x = src_msg.orientation.x;
    dst_msg.orientation.y = src_msg.orientation.y;
    dst_msg.orientation.z = src_msg.orientation.z;
    dst_msg.orientation.w = src_msg.orientation.w;
    for(int n = 0; n < 9; n++)
        dst_msg.orientation_covariance[n] = src_msg.orientation_covariance[n];
    dst_msg.angular_velocity.x = src_msg.angular_velocity.x;
    dst_msg.angular_velocity.y = src_msg.angular_velocity.y;
    dst_msg.angular_velocity.z = src_msg.angular_velocity.z;
    for(int n = 0; n < 9; n++)
        dst_msg.angular_velocity_covariance[n] = src_msg.angular_velocity_covariance[n];
    dst_msg.linear_acceleration.x = src_msg.linear_acceleration.x;
    dst_msg.linear_acceleration.y = src_msg.linear_acceleration.y;
    dst_msg.linear_acceleration.z = src_msg.linear_acceleration.z;
    for(int n = 0; n < 9; n++)
        dst_msg.linear_acceleration_covariance[n] = src_msg.linear_acceleration_covariance[n];
    return dst_msg;
}

static void freeImuMsg(SickScanImuMsg& msg)
{
    memset(&msg, 0, sizeof(msg));
}

static SickScanLFErecMsg convertLFErecMsg(const sick_scan_msg::LFErecMsg& src_msg, const std::string& topic = sick_scan_xd::getLFErecTopic())
{
    SickScanLFErecMsg dst_msg;
    memset(&dst_msg, 0, sizeof(dst_msg));
    // Copy header
    ROS_HEADER_SEQ(dst_msg.header, src_msg.header.seq);
    dst_msg.header.timestamp_sec = sec(src_msg.header.stamp);
    dst_msg.header.timestamp_nsec = nsec(src_msg.header.stamp);
    strncpy(dst_msg.header.frame_id, src_msg.header.frame_id.c_str(), sizeof(dst_msg.header.frame_id) - 2);
    strncpy(dst_msg.topic, topic.c_str(), sizeof(dst_msg.topic) - 2);   
    // Copy LFErec data
    int max_fields_number = (int)(sizeof(dst_msg.fields) / sizeof(dst_msg.fields[0]));
    dst_msg.fields_number = ((src_msg.fields_number < max_fields_number) ? src_msg.fields_number : max_fields_number);
    for(int n = 0; n < dst_msg.fields_number; n++)
    {
        dst_msg.fields[n].version_number = src_msg.fields[n].version_number;
        dst_msg.fields[n].field_index = src_msg.fields[n].field_index;
        dst_msg.fields[n].sys_count = src_msg.fields[n].sys_count;
        dst_msg.fields[n].dist_scale_factor = src_msg.fields[n].dist_scale_factor;
        dst_msg.fields[n].dist_scale_offset = src_msg.fields[n].dist_scale_offset;
        dst_msg.fields[n].angle_scale_factor = src_msg.fields[n].angle_scale_factor;
        dst_msg.fields[n].angle_scale_offset = src_msg.fields[n].angle_scale_offset;
        dst_msg.fields[n].field_result_mrs = src_msg.fields[n].field_result_mrs;
        dst_msg.fields[n].time_state = src_msg.fields[n].time_state;
        dst_msg.fields[n].year = src_msg.fields[n].year;
        dst_msg.fields[n].month = src_msg.fields[n].month;
        dst_msg.fields[n].day = src_msg.fields[n].day;
        dst_msg.fields[n].hour = src_msg.fields[n].hour;
        dst_msg.fields[n].minute = src_msg.fields[n].minute;
        dst_msg.fields[n].second = src_msg.fields[n].second;
        dst_msg.fields[n].microsecond = src_msg.fields[n].microsecond;
    }
    return dst_msg;
}

static void freeLFErecMsg(SickScanLFErecMsg& msg)
{
    memset(&msg, 0, sizeof(msg));
}

static SickScanLIDoutputstateMsg convertLIDoutputstateMsg(const sick_scan_msg::LIDoutputstateMsg& src_msg, const std::string& topic = sick_scan_xd::getLIDoutputstateTopic())
{
    SickScanLIDoutputstateMsg dst_msg;
    memset(&dst_msg, 0, sizeof(dst_msg));
    // Copy header
    ROS_HEADER_SEQ(dst_msg.header, src_msg.header.seq);
    dst_msg.header.timestamp_sec = sec(src_msg.header.stamp);
    dst_msg.header.timestamp_nsec = nsec(src_msg.header.stamp);
    strncpy(dst_msg.header.frame_id, src_msg.header.frame_id.c_str(), sizeof(dst_msg.header.frame_id) - 2);
    strncpy(dst_msg.topic, topic.c_str(), sizeof(dst_msg.topic) - 2);   
    // Copy LIDoutputstate data
    dst_msg.version_number = src_msg.version_number;
    dst_msg.system_counter = src_msg.system_counter;
    int max_states = (int)(sizeof(dst_msg.output_state) / sizeof(dst_msg.output_state[0]));
    int max_counts = (int)(sizeof(dst_msg.output_count) / sizeof(dst_msg.output_count[0]));
    for(int n = 0; n < src_msg.output_state.size() && n < max_states; n++)
        dst_msg.output_state[n] = src_msg.output_state[n];
    for(int n = 0; n < src_msg.output_count.size() && n < max_counts; n++)
        dst_msg.output_count[n] = src_msg.output_count[n];
    dst_msg.time_state = src_msg.time_state;
    dst_msg.year = src_msg.year;
    dst_msg.month = src_msg.month;
    dst_msg.day = src_msg.day;
    dst_msg.hour = src_msg.hour;
    dst_msg.minute = src_msg.minute;
    dst_msg.second = src_msg.second;
    dst_msg.microsecond = src_msg.microsecond;
    return dst_msg;
}

static void freeLIDoutputstateMsg(SickScanLIDoutputstateMsg& msg)
{
    memset(&msg, 0, sizeof(msg));
}

static SickScanRadarScan convertRadarScanMsg(const sick_scan_msg::RadarScan& src_msg, const std::string& topic = sick_scan_xd::getRadarScanTopic())
{
    SickScanRadarScan dst_msg;
    memset(&dst_msg, 0, sizeof(dst_msg));
    // Copy header
    ROS_HEADER_SEQ(dst_msg.header, src_msg.header.seq);
    dst_msg.header.timestamp_sec = sec(src_msg.header.stamp);
    dst_msg.header.timestamp_nsec = nsec(src_msg.header.stamp);
    strncpy(dst_msg.header.frame_id, src_msg.header.frame_id.c_str(), sizeof(dst_msg.header.frame_id) - 2);
    strncpy(dst_msg.topic, topic.c_str(), sizeof(dst_msg.topic) - 2);   
    // Copy radarpreheader data
    dst_msg.radarpreheader.uiversionno = src_msg.radarpreheader.uiversionno;
    dst_msg.radarpreheader.uiident = src_msg.radarpreheader.radarpreheaderdeviceblock.uiident;
    dst_msg.radarpreheader.udiserialno = src_msg.radarpreheader.radarpreheaderdeviceblock.udiserialno;
    dst_msg.radarpreheader.bdeviceerror = src_msg.radarpreheader.radarpreheaderdeviceblock.bdeviceerror;
    dst_msg.radarpreheader.bcontaminationwarning = src_msg.radarpreheader.radarpreheaderdeviceblock.bcontaminationwarning;
    dst_msg.radarpreheader.bcontaminationerror = src_msg.radarpreheader.radarpreheaderdeviceblock.bcontaminationerror;
    dst_msg.radarpreheader.uitelegramcount = src_msg.radarpreheader.radarpreheaderstatusblock.uitelegramcount;
    dst_msg.radarpreheader.uicyclecount = src_msg.radarpreheader.radarpreheaderstatusblock.uicyclecount;
    dst_msg.radarpreheader.udisystemcountscan = src_msg.radarpreheader.radarpreheaderstatusblock.udisystemcountscan;
    dst_msg.radarpreheader.udisystemcounttransmit = src_msg.radarpreheader.radarpreheaderstatusblock.udisystemcounttransmit;
    dst_msg.radarpreheader.uiinputs = src_msg.radarpreheader.radarpreheaderstatusblock.uiinputs;
    dst_msg.radarpreheader.uioutputs = src_msg.radarpreheader.radarpreheaderstatusblock.uioutputs;
    dst_msg.radarpreheader.uicycleduration = src_msg.radarpreheader.radarpreheadermeasurementparam1block.uicycleduration;
    dst_msg.radarpreheader.uinoiselevel = src_msg.radarpreheader.radarpreheadermeasurementparam1block.uinoiselevel;
    dst_msg.radarpreheader.numencoder = src_msg.radarpreheader.radarpreheaderarrayencoderblock.size();
    int max_encpositions = (int)(sizeof(dst_msg.radarpreheader.udiencoderpos) / sizeof(dst_msg.radarpreheader.udiencoderpos[0]));
    int max_encspeedvals = (int)(sizeof(dst_msg.radarpreheader.iencoderspeed) / sizeof(dst_msg.radarpreheader.iencoderspeed[0]));
    dst_msg.radarpreheader.numencoder = ((dst_msg.radarpreheader.numencoder < max_encpositions) ? dst_msg.radarpreheader.numencoder : max_encpositions);
    dst_msg.radarpreheader.numencoder = ((dst_msg.radarpreheader.numencoder < max_encspeedvals) ? dst_msg.radarpreheader.numencoder : max_encspeedvals);
    for(int n = 0; n < dst_msg.radarpreheader.numencoder; n++)
    {
        dst_msg.radarpreheader.udiencoderpos[n] = src_msg.radarpreheader.radarpreheaderarrayencoderblock[n].udiencoderpos;
        dst_msg.radarpreheader.iencoderspeed[n] = src_msg.radarpreheader.radarpreheaderarrayencoderblock[n].iencoderspeed;
    }
    // Copy radar target pointcloud data
    sick_scan_xd::PointCloud2withEcho targets_with_echo(&src_msg.targets, 1, 0, "radar");
    dst_msg.targets = convertPointCloudMsg(targets_with_echo);
    // Copy radar object data
    dst_msg.objects.size = src_msg.objects.size();
    dst_msg.objects.capacity = dst_msg.objects.size;
    dst_msg.objects.buffer = (SickScanRadarObject*)malloc(dst_msg.objects.capacity * sizeof(SickScanRadarObject));
    if (!dst_msg.objects.buffer)
    {
        dst_msg.objects.size = 0;
        dst_msg.objects.capacity = 0;
    }
    for(int n = 0; n < dst_msg.objects.size; n++)
    {
        dst_msg.objects.buffer[n].id = src_msg.objects[n].id;
        dst_msg.objects.buffer[n].tracking_time_sec = sec(src_msg.objects[n].tracking_time);
        dst_msg.objects.buffer[n].tracking_time_nsec = nsec(src_msg.objects[n].tracking_time);
        dst_msg.objects.buffer[n].last_seen_sec = sec(src_msg.objects[n].last_seen);
        dst_msg.objects.buffer[n].last_seen_nsec = nsec(src_msg.objects[n].last_seen);
        dst_msg.objects.buffer[n].velocity_linear.x = src_msg.objects[n].velocity.twist.linear.x;
        dst_msg.objects.buffer[n].velocity_linear.y = src_msg.objects[n].velocity.twist.linear.y;
        dst_msg.objects.buffer[n].velocity_linear.z = src_msg.objects[n].velocity.twist.linear.y;
        dst_msg.objects.buffer[n].velocity_angular.x = src_msg.objects[n].velocity.twist.angular.x;
        dst_msg.objects.buffer[n].velocity_angular.y = src_msg.objects[n].velocity.twist.angular.y;
        dst_msg.objects.buffer[n].velocity_angular.z = src_msg.objects[n].velocity.twist.angular.y;
        for(int m = 0; m < 36; m++)
            dst_msg.objects.buffer[n].velocity_covariance[m] = src_msg.objects[n].velocity.covariance[m];
        dst_msg.objects.buffer[n].bounding_box_center_position.x = src_msg.objects[n].bounding_box_center.position.x;
        dst_msg.objects.buffer[n].bounding_box_center_position.y = src_msg.objects[n].bounding_box_center.position.y;
        dst_msg.objects.buffer[n].bounding_box_center_position.z = src_msg.objects[n].bounding_box_center.position.z;
        dst_msg.objects.buffer[n].bounding_box_center_orientation.x = src_msg.objects[n].bounding_box_center.orientation.x;
        dst_msg.objects.buffer[n].bounding_box_center_orientation.y = src_msg.objects[n].bounding_box_center.orientation.y;
        dst_msg.objects.buffer[n].bounding_box_center_orientation.z = src_msg.objects[n].bounding_box_center.orientation.z;
        dst_msg.objects.buffer[n].bounding_box_center_orientation.w = src_msg.objects[n].bounding_box_center.orientation.w;
        dst_msg.objects.buffer[n].bounding_box_size.x = src_msg.objects[n].bounding_box_size.x;
        dst_msg.objects.buffer[n].bounding_box_size.y = src_msg.objects[n].bounding_box_size.y;
        dst_msg.objects.buffer[n].bounding_box_size.z = src_msg.objects[n].bounding_box_size.z;
        dst_msg.objects.buffer[n].object_box_center_position.x = src_msg.objects[n].object_box_center.pose.position.x;
        dst_msg.objects.buffer[n].object_box_center_position.y = src_msg.objects[n].object_box_center.pose.position.y;
        dst_msg.objects.buffer[n].object_box_center_position.z = src_msg.objects[n].object_box_center.pose.position.z;
        dst_msg.objects.buffer[n].object_box_center_orientation.x = src_msg.objects[n].object_box_center.pose.orientation.x;
        dst_msg.objects.buffer[n].object_box_center_orientation.y = src_msg.objects[n].object_box_center.pose.orientation.y;
        dst_msg.objects.buffer[n].object_box_center_orientation.z = src_msg.objects[n].object_box_center.pose.orientation.z;
        dst_msg.objects.buffer[n].object_box_center_orientation.w = src_msg.objects[n].object_box_center.pose.orientation.w;
        for(int m = 0; m < 36; m++)
            dst_msg.objects.buffer[n].object_box_center_covariance[m] = src_msg.objects[n].object_box_center.covariance[m];
        dst_msg.objects.buffer[n].object_box_size.x = src_msg.objects[n].object_box_size.x;
        dst_msg.objects.buffer[n].object_box_size.y = src_msg.objects[n].object_box_size.y;
        dst_msg.objects.buffer[n].object_box_size.z = src_msg.objects[n].object_box_size.z;
        dst_msg.objects.buffer[n].contour_points.size = src_msg.objects[n].contour_points.size();
        dst_msg.objects.buffer[n].contour_points.capacity = dst_msg.objects.buffer[n].contour_points.size;
        dst_msg.objects.buffer[n].contour_points.buffer = (SickScanVector3Msg*)malloc(dst_msg.objects.buffer[n].contour_points.capacity * sizeof(SickScanVector3Msg));
        if (!dst_msg.objects.buffer[n].contour_points.buffer)
        {
            dst_msg.objects.buffer[n].contour_points.size = 0;
            dst_msg.objects.buffer[n].contour_points.capacity = 0;
        }
        for(int m = 0; m < dst_msg.objects.buffer[n].contour_points.size; m++)
        {
            dst_msg.objects.buffer[n].contour_points.buffer[m].x = src_msg.objects[n].contour_points[m].x;
            dst_msg.objects.buffer[n].contour_points.buffer[m].y = src_msg.objects[n].contour_points[m].y;
            dst_msg.objects.buffer[n].contour_points.buffer[m].z = src_msg.objects[n].contour_points[m].z;

        }
    }
    return dst_msg;
}

static void freeRadarScanMsg(SickScanRadarScan& msg)
{
    freePointCloudMsg(msg.targets);
    for(int n = 0; n < msg.objects.size; n++)
        free(msg.objects.buffer[n].contour_points.buffer);
    free(msg.objects.buffer);
    memset(&msg, 0, sizeof(msg));
}

static SickScanLdmrsObjectArray convertLdmrsObjectArrayMsg(const sick_scan_msg::SickLdmrsObjectArray& src_msg)
{
    SickScanLdmrsObjectArray dst_msg;
    memset(&dst_msg, 0, sizeof(dst_msg));
    // Copy header
    ROS_HEADER_SEQ(dst_msg.header, src_msg.header.seq);
    dst_msg.header.timestamp_sec = sec(src_msg.header.stamp);
    dst_msg.header.timestamp_nsec = nsec(src_msg.header.stamp);
    strncpy(dst_msg.header.frame_id, src_msg.header.frame_id.c_str(), sizeof(dst_msg.header.frame_id) - 2);
    // Copy ldmrs objects
    dst_msg.objects.size = src_msg.objects.size();
    dst_msg.objects.capacity = dst_msg.objects.size;
    dst_msg.objects.buffer = (SickScanLdmrsObject*)malloc(dst_msg.objects.capacity * sizeof(SickScanLdmrsObject));
    if (!dst_msg.objects.buffer)
    {
        dst_msg.objects.size = 0;
        dst_msg.objects.capacity = 0;
    }
    for(int n = 0; n < dst_msg.objects.size; n++)
    {
        dst_msg.objects.buffer[n].id = src_msg.objects[n].id;
        dst_msg.objects.buffer[n].tracking_time_sec = sec(src_msg.objects[n].tracking_time);
        dst_msg.objects.buffer[n].tracking_time_nsec = nsec(src_msg.objects[n].tracking_time);
        dst_msg.objects.buffer[n].last_seen_sec = sec(src_msg.objects[n].last_seen);
        dst_msg.objects.buffer[n].last_seen_nsec = nsec(src_msg.objects[n].last_seen);
        dst_msg.objects.buffer[n].velocity_linear.x = src_msg.objects[n].velocity.twist.linear.x;
        dst_msg.objects.buffer[n].velocity_linear.y = src_msg.objects[n].velocity.twist.linear.y;
        dst_msg.objects.buffer[n].velocity_linear.z = src_msg.objects[n].velocity.twist.linear.y;
        dst_msg.objects.buffer[n].velocity_angular.x = src_msg.objects[n].velocity.twist.angular.x;
        dst_msg.objects.buffer[n].velocity_angular.y = src_msg.objects[n].velocity.twist.angular.y;
        dst_msg.objects.buffer[n].velocity_angular.z = src_msg.objects[n].velocity.twist.angular.y;
        for(int m = 0; m < 36; m++)
            dst_msg.objects.buffer[n].velocity_covariance[m] = src_msg.objects[n].velocity.covariance[m];
        dst_msg.objects.buffer[n].bounding_box_center_position.x = src_msg.objects[n].bounding_box_center.position.x;
        dst_msg.objects.buffer[n].bounding_box_center_position.y = src_msg.objects[n].bounding_box_center.position.y;
        dst_msg.objects.buffer[n].bounding_box_center_position.z = src_msg.objects[n].bounding_box_center.position.z;
        dst_msg.objects.buffer[n].bounding_box_center_orientation.x = src_msg.objects[n].bounding_box_center.orientation.x;
        dst_msg.objects.buffer[n].bounding_box_center_orientation.y = src_msg.objects[n].bounding_box_center.orientation.y;
        dst_msg.objects.buffer[n].bounding_box_center_orientation.z = src_msg.objects[n].bounding_box_center.orientation.z;
        dst_msg.objects.buffer[n].bounding_box_center_orientation.w = src_msg.objects[n].bounding_box_center.orientation.w;
        dst_msg.objects.buffer[n].bounding_box_size.x = src_msg.objects[n].bounding_box_size.x;
        dst_msg.objects.buffer[n].bounding_box_size.y = src_msg.objects[n].bounding_box_size.y;
        dst_msg.objects.buffer[n].bounding_box_size.z = src_msg.objects[n].bounding_box_size.z;
        dst_msg.objects.buffer[n].object_box_center_position.x = src_msg.objects[n].object_box_center.pose.position.x;
        dst_msg.objects.buffer[n].object_box_center_position.y = src_msg.objects[n].object_box_center.pose.position.y;
        dst_msg.objects.buffer[n].object_box_center_position.z = src_msg.objects[n].object_box_center.pose.position.z;
        dst_msg.objects.buffer[n].object_box_center_orientation.x = src_msg.objects[n].object_box_center.pose.orientation.x;
        dst_msg.objects.buffer[n].object_box_center_orientation.y = src_msg.objects[n].object_box_center.pose.orientation.y;
        dst_msg.objects.buffer[n].object_box_center_orientation.z = src_msg.objects[n].object_box_center.pose.orientation.z;
        dst_msg.objects.buffer[n].object_box_center_orientation.w = src_msg.objects[n].object_box_center.pose.orientation.w;
        for(int m = 0; m < 36; m++)
            dst_msg.objects.buffer[n].object_box_center_covariance[m] = src_msg.objects[n].object_box_center.covariance[m];
        dst_msg.objects.buffer[n].object_box_size.x = src_msg.objects[n].object_box_size.x;
        dst_msg.objects.buffer[n].object_box_size.y = src_msg.objects[n].object_box_size.y;
        dst_msg.objects.buffer[n].object_box_size.z = src_msg.objects[n].object_box_size.z;
        dst_msg.objects.buffer[n].contour_points.size = src_msg.objects[n].contour_points.size();
        dst_msg.objects.buffer[n].contour_points.capacity = dst_msg.objects.buffer[n].contour_points.size;
        dst_msg.objects.buffer[n].contour_points.buffer = (SickScanVector3Msg*)malloc(dst_msg.objects.buffer[n].contour_points.capacity * sizeof(SickScanVector3Msg));
        if (!dst_msg.objects.buffer[n].contour_points.buffer)
        {
            dst_msg.objects.buffer[n].contour_points.size = 0;
            dst_msg.objects.buffer[n].contour_points.capacity = 0;
        }
        for(int m = 0; m < dst_msg.objects.buffer[n].contour_points.size; m++)
        {
            dst_msg.objects.buffer[n].contour_points.buffer[m].x = src_msg.objects[n].contour_points[m].x;
            dst_msg.objects.buffer[n].contour_points.buffer[m].y = src_msg.objects[n].contour_points[m].y;
            dst_msg.objects.buffer[n].contour_points.buffer[m].z = src_msg.objects[n].contour_points[m].z;

        }
    }
    return dst_msg;
}

static void freeLdmrsObjectArrayMsg(SickScanLdmrsObjectArray& msg)
{
    for(int n = 0; n < msg.objects.size; n++)
        free(msg.objects.buffer[n].contour_points.buffer);
    free(msg.objects.buffer);
    memset(&msg, 0, sizeof(msg));
}

static SickScanVisualizationMarkerMsg convertVisualizationMarkerMsg(const ros_visualization_msgs::MarkerArray& src_msg, const std::string& topic = sick_scan_xd::getVisualizationMarkerTopic())
{
    SickScanVisualizationMarkerMsg dst_msg;
    memset(&dst_msg, 0, sizeof(dst_msg));
    strncpy(dst_msg.topic, topic.c_str(), sizeof(dst_msg.topic) - 2);   
    if (src_msg.markers.size() > 0)
    {
        // Copy markers
        dst_msg.markers.size = src_msg.markers.size();
        dst_msg.markers.capacity = dst_msg.markers.size;
        dst_msg.markers.buffer = (SickScanVisualizationMarker*)malloc(dst_msg.markers.capacity * sizeof(SickScanVisualizationMarker));
        if (!dst_msg.markers.buffer)
        {
            dst_msg.markers.size = 0;
            dst_msg.markers.capacity = 0;
        }
        for(int n = 0; n < dst_msg.markers.size; n++)
        {
            const ros_visualization_msgs::Marker& src_marker = src_msg.markers[n];
            SickScanVisualizationMarker& dst_marker = dst_msg.markers.buffer[n];
            memset(&dst_marker, 0, sizeof(dst_marker));
            // Copy header
            ROS_HEADER_SEQ(dst_marker.header, src_marker.header.seq);
            dst_marker.header.timestamp_sec = sec(src_marker.header.stamp);
            dst_marker.header.timestamp_nsec = nsec(src_marker.header.stamp);
            strncpy(dst_marker.header.frame_id, src_marker.header.frame_id.c_str(), sizeof(dst_marker.header.frame_id) - 2);
            // Copy data
            strncpy(dst_marker.ns, src_marker.ns.c_str(), sizeof(dst_marker.ns) - 2);
            dst_marker.id = src_marker.id;
            dst_marker.type = src_marker.type;
            dst_marker.action = src_marker.action;
            dst_marker.pose_position.x = src_marker.pose.position.x;
            dst_marker.pose_position.y = src_marker.pose.position.y;
            dst_marker.pose_position.z = src_marker.pose.position.z;
            dst_marker.pose_orientation.x = src_marker.pose.orientation.x;
            dst_marker.pose_orientation.y = src_marker.pose.orientation.y;
            dst_marker.pose_orientation.z = src_marker.pose.orientation.z;
            dst_marker.pose_orientation.w = src_marker.pose.orientation.w;
            dst_marker.scale.x = src_marker.scale.x;
            dst_marker.scale.y = src_marker.scale.y;
            dst_marker.scale.z = src_marker.scale.z;
            dst_marker.color.r = src_marker.color.r;
            dst_marker.color.g = src_marker.color.g;
            dst_marker.color.b = src_marker.color.b;
            dst_marker.color.a = src_marker.color.a;
            dst_marker.lifetime_sec = sec(src_marker.lifetime);
            dst_marker.lifetime_nsec = nsec(src_marker.lifetime);
            dst_marker.frame_locked = src_marker.frame_locked;
            strncpy(dst_marker.text, src_marker.text.c_str(), sizeof(dst_marker.text) - 2);
            strncpy(dst_marker.mesh_resource, src_marker.mesh_resource.c_str(), sizeof(dst_marker.mesh_resource) - 2);
            dst_marker.mesh_use_embedded_materials = src_marker.mesh_use_embedded_materials;
            dst_marker.points.size = src_marker.points.size();
            dst_marker.points.capacity = dst_marker.points.size;
            dst_marker.points.buffer = (SickScanVector3Msg*)malloc(dst_marker.points.capacity * sizeof(SickScanVector3Msg));
            if (!dst_marker.points.buffer)
            {
                dst_marker.points.size = 0;
                dst_marker.points.capacity = 0;
            }
            for(int m = 0; m < dst_marker.points.size; m++)
            {
                dst_marker.points.buffer[m].x = src_marker.points[m].x;
                dst_marker.points.buffer[m].y = src_marker.points[m].y;
                dst_marker.points.buffer[m].z = src_marker.points[m].z;
            }
            dst_marker.colors.size = src_marker.colors.size();
            dst_marker.colors.capacity = dst_marker.colors.size;
            dst_marker.colors.buffer = (SickScanColorRGBA*)malloc(dst_marker.colors.capacity * sizeof(SickScanColorRGBA));
            if (!dst_marker.colors.buffer)
            {
                dst_marker.colors.size = 0;
                dst_marker.colors.capacity = 0;
            }
            for(int m = 0; m < dst_marker.colors.size; m++)
            {
                dst_marker.colors.buffer[m].r = src_marker.colors[m].r;
                dst_marker.colors.buffer[m].g = src_marker.colors[m].g;
                dst_marker.colors.buffer[m].b = src_marker.colors[m].b;
                dst_marker.colors.buffer[m].a = src_marker.colors[m].a;
            }
        }
    }
    return dst_msg;
}

static void freeVisualizationMarkerMsg(SickScanVisualizationMarkerMsg& msg)
{
    for(int n = 0; n < msg.markers.size; n++)
    {
        free(msg.markers.buffer[n].points.buffer);
        free(msg.markers.buffer[n].colors.buffer);
    }
    free(msg.markers.buffer);
    memset(&msg, 0, sizeof(msg));
}

/*
*  Callback handler
*/

static void cartesian_pointcloud_callback(rosNodePtr node, const sick_scan_xd::PointCloud2withEcho* msg)
{
    ROS_DEBUG_STREAM("api_impl cartesian_pointcloud_callback: PointCloud2 message, " << msg->pointcloud.width << "x" << msg->pointcloud.height << " points");
    DUMP_API_POINTCLOUD_MESSAGE("impl", msg->pointcloud);
    // Convert ros_sensor_msgs::PointCloud2 message to SickScanPointCloudMsg and export (i.e. notify all listeners)
    SickScanPointCloudMsg export_msg = convertPointCloudMsg(*msg);
    SickScanApiHandle apiHandle = castNodeToApiHandle(node);
    s_callback_handler_cartesian_pointcloud_messages.notifyListener(apiHandle, &export_msg);
    freePointCloudMsg(export_msg);
}

static void polar_pointcloud_callback(rosNodePtr node, const sick_scan_xd::PointCloud2withEcho* msg)
{
    ROS_DEBUG_STREAM("api_impl polar_pointcloud_callback: PointCloud2 message, " << msg->pointcloud.width << "x" << msg->pointcloud.height << " points");
    // Convert ros_sensor_msgs::PointCloud2 message to SickScanPointCloudMsg and export (i.e. notify all listeners)
    SickScanPointCloudMsg export_msg = convertPointCloudMsg(*msg);
    SickScanApiHandle apiHandle = castNodeToApiHandle(node);
    s_callback_handler_polar_pointcloud_messages.notifyListener(apiHandle, &export_msg);
    freePointCloudMsg(export_msg);
}

static void imu_callback(rosNodePtr node, const ros_sensor_msgs::Imu* msg)
{
    // ROS_DEBUG_STREAM("api_impl lferec_callback: Imu message = {" << (*msg) << "}");
    DUMP_API_IMU_MESSAGE("impl", *msg);
    ROS_DEBUG_STREAM("api_impl imu_callback: Imu message, orientation={" << ROS_QUATERNION_TO_STREAM(msg->orientation) << "}, angular_velocity={" << ROS_VECTOR3D_TO_STREAM(msg->angular_velocity) << "}, linear_acceleration={" << ROS_VECTOR3D_TO_STREAM(msg->linear_acceleration) << "}");
    // Convert ros_sensor_msgs::PointCloud2 message to SickScanPointCloudMsg and export (i.e. notify all listeners)
    SickScanImuMsg export_msg = convertImuMsg(*msg);
    SickScanApiHandle apiHandle = castNodeToApiHandle(node);
    s_callback_handler_imu_messages.notifyListener(apiHandle, &export_msg);
    freeImuMsg(export_msg);
}

static void lferec_callback(rosNodePtr node, const sick_scan_msg::LFErecMsg* msg)
{
    // ROS_DEBUG_STREAM("api_impl lferec_callback: LFErec message = {" << (*msg) << "}");
    DUMP_API_LFEREC_MESSAGE("impl", *msg);
    std::stringstream field_info;
    for(int n = 0; n  < msg->fields_number; n++)
    {
        field_info << ", field " << (int)(msg->fields[n].field_index) << ": (";
        if (msg->fields[n].field_result_mrs == 1)
            field_info << "free,";
        else if (msg->fields[n].field_result_mrs == 2)
            field_info << "infringed,";
        else
            field_info << "invalid,";
        field_info << msg->fields[n].dist_scale_factor << "," << msg->fields[n].dist_scale_offset << "," << msg->fields[n].angle_scale_factor << "," << msg->fields[n].angle_scale_offset << ")";
    }
    ROS_DEBUG_STREAM("api_impl lferec_callback: LFErec message, " << msg->fields_number << " fields" << field_info.str());
    SickScanLFErecMsg export_msg = convertLFErecMsg(*msg);
    SickScanApiHandle apiHandle = castNodeToApiHandle(node);
    s_callback_handler_lferec_messages.notifyListener(apiHandle, &export_msg);
    freeLFErecMsg(export_msg);
}

static void lidoutputstate_callback(rosNodePtr node, const sick_scan_msg::LIDoutputstateMsg* msg)
{
    // ROS_DEBUG_STREAM("api_impl lidoutputstate_callback: LIDoutputstate message = {" << (*msg) << "}");
    DUMP_API_LIDOUTPUTSTATE_MESSAGE("impl", *msg);
    std::stringstream state_info;
    state_info << ", outputstate=(";
    for(int n = 0; n  < msg->output_state.size(); n++)
        state_info << (n > 0 ? ",": "") << (int)(msg->output_state[n]);
    state_info << "), outputcount=(";
    for(int n = 0; n  < msg->output_count.size(); n++)
        state_info << (n > 0 ? ",": "") << (int)(msg->output_count[n]);
    state_info << ")";
    ROS_DEBUG_STREAM("api_impl lidoutputstate_callback: LIDoutputstate message" << state_info.str());
    SickScanLIDoutputstateMsg export_msg = convertLIDoutputstateMsg(*msg);
    SickScanApiHandle apiHandle = castNodeToApiHandle(node);
    s_callback_handler_lidoutputstate_messages.notifyListener(apiHandle, &export_msg);
    freeLIDoutputstateMsg(export_msg);
}

static void radarscan_callback(rosNodePtr node, const sick_scan_msg::RadarScan* msg)
{
    // ROS_DEBUG_STREAM("api_impl radarscan_callback: RadarScan message = {" << (*msg) << "}");
    DUMP_API_RADARSCAN_MESSAGE("impl", *msg);
    ROS_DEBUG_STREAM("api_impl radarscan_callback: " << (msg->targets.width * msg->targets.height) << " targets, "  << msg->objects.size() << " objects");
    SickScanRadarScan export_msg = convertRadarScanMsg(*msg);
    SickScanApiHandle apiHandle = castNodeToApiHandle(node);
    s_callback_handler_radarscan_messages.notifyListener(apiHandle, &export_msg);
    freeRadarScanMsg(export_msg);
}

static void ldmrsobjectarray_callback(rosNodePtr node, const sick_scan_msg::SickLdmrsObjectArray* msg)
{
    // ROS_DEBUG_STREAM("api_impl ldmrsobjectarray_callback: LdmrsObjectArray message = {" << (*msg) << "}");
    DUMP_API_LDMRSOBJECTARRAY_MESSAGE("impl", *msg);
    ROS_DEBUG_STREAM("api_impl ldmrsobjectarray_callback: " << msg->objects.size() << " objects");
    SickScanLdmrsObjectArray export_msg = convertLdmrsObjectArrayMsg(*msg);
    SickScanApiHandle apiHandle = castNodeToApiHandle(node);
    s_callback_handler_ldmrsobjectarray_messages.notifyListener(apiHandle, &export_msg);
    freeLdmrsObjectArrayMsg(export_msg);
}

static void visualizationmarker_callback(rosNodePtr node, const ros_visualization_msgs::MarkerArray* msg)
{
    // ROS_DEBUG_STREAM("api_impl visualizationmarker_callback: MarkerArray message = {" << (*msg) << "}");
    DUMP_API_VISUALIZATIONMARKER_MESSAGE("impl", *msg);
    std::stringstream marker_info;
    for(int n = 0; n < msg->markers.size(); n++)
        marker_info << ", marker " << msg->markers[n].id << ": pos=(" << msg->markers[n].pose.position.x << "," << msg->markers[n].pose.position.y << "," << msg->markers[n].pose.position.z << ")";
    ROS_DEBUG_STREAM("api_impl visualizationmarker_callback: " << msg->markers.size() << " markers" << marker_info.str());
    SickScanVisualizationMarkerMsg export_msg = convertVisualizationMarkerMsg(*msg);
    SickScanApiHandle apiHandle = castNodeToApiHandle(node);
    s_callback_handler_visualizationmarker_messages.notifyListener(apiHandle, &export_msg);
    freeVisualizationMarkerMsg(export_msg);
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
    try
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
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiCreate(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiCreate(): unknown exception ");
    }
    return 0;
}

// Release and free all resources of a handle; the handle is invalid after SickScanApiRelease
int32_t SickScanApiRelease(SickScanApiHandle apiHandle)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiRelease(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_api_caller[apiHandle].clear();
        s_callback_handler_cartesian_pointcloud_messages.clear();
        s_callback_handler_polar_pointcloud_messages.clear();
        s_callback_handler_imu_messages.clear();
        s_callback_handler_lferec_messages.clear();
        s_callback_handler_lidoutputstate_messages.clear();
        s_callback_handler_radarscan_messages.clear();
        s_callback_handler_ldmrsobjectarray_messages.clear();
        s_callback_handler_visualizationmarker_messages.clear();
        s_callback_handler_navposelandmark_messages.clear();
        for(int n = 0; n < s_malloced_resources.size(); n++)
            free(s_malloced_resources[n]);
        s_malloced_resources.clear();
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRelease(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRelease(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}

// Initializes a lidar by launchfile and starts message receiving and processing
int32_t SickScanApiInitByLaunchfile(SickScanApiHandle apiHandle, const char* launchfile_args)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiInitByLaunchfile(" << launchfile_args << "): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        // Split launchfile_args by spaces
        ROS_INFO_STREAM("SickScanApiInitByLaunchfile: launchfile_args = \"" << launchfile_args << "\"");
        std::string args_string(launchfile_args);
        std::string arg;
        std::vector<std::string> args;
        std::string endToken = ".launch";
        std::size_t pos = args_string.find(endToken);
        std::string filepath = args_string.substr(0, pos + endToken.length());
        args_string.erase(0, pos + endToken.length() + 1);
        args.push_back(filepath);
        while ((pos = args_string.find(' ')) != std::string::npos) 
        {
          arg = args_string.substr(0, pos);
          args.push_back(arg);
          args_string.erase(0, pos + 1);
        }
        args.push_back(args_string);
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
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiInitByLaunchfile(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiInitByLaunchfile(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}

// Initializes a lidar by commandline arguments and starts message receiving and processing
int32_t SickScanApiInitByCli(SickScanApiHandle apiHandle, int argc, char** argv)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiInitByCli(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        // Create a deep copy of argv
        s_argc = argc;
        s_argv = (char**)malloc(argc * sizeof(char*));
        std::stringstream cli_params;
        for(int n = 0; n < argc; n++)
        {
            s_argv[n] = (char*)malloc((strlen(argv[n]) + 1) * sizeof(char));
            strcpy(s_argv[n],argv[n]);
            cli_params << (n > 0 ? " ": "")  << argv[n];
        }
        ROS_INFO_STREAM("SickScanApiInitByCli: " << cli_params.str());
        
        // Start sick_scan event loop
        int exit_code = 0;
        rosNodePtr node = castApiHandleToNode(apiHandle);
        try
        {
            if (!startGenericLaser(s_argc, s_argv, s_scannerName, node, &exit_code) || exit_code != sick_scan_xd::ExitSuccess)
            {
                ROS_ERROR_STREAM("## ERROR SickScanApiInitByCli(): startGenericLaser() failed, could not start generic laser event loop");
                return SICK_SCAN_API_ERROR;
            }
            return SICK_SCAN_API_SUCCESS;
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiInitByCli(): exception " << e.what());
        }
        try
        {
            ROS_WARN_STREAM("SickScanApiInitByCli(): running sick_generic_laser in main thread ...");
            exit_code = mainGenericLaser(s_argc, s_argv, s_scannerName, node);
            if (exit_code != sick_scan_xd::ExitSuccess)
            {
                ROS_ERROR_STREAM("## ERROR SickScanApiInitByCli(): mainGenericLaser() failed");
                return SICK_SCAN_API_ERROR;
            }
            return SICK_SCAN_API_SUCCESS;
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiInitByCli(): exception " << e.what());
        }
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiInitByCli(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiInitByCli(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}

// Stops message receiving and processing and closes a lidar
int32_t SickScanApiClose(SickScanApiHandle apiHandle)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiClose(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        // stopScannerAndExit(true);
        rosSignalHandler(SIGINT); // Send Ctrl-C for gentle shutdown
        sick_scan_xd::WaitForCartesianPointCloudMessageHandler::shutdown();
        sick_scan_xd::WaitForPolarPointCloudMessageHandler::shutdown();
        sick_scan_xd::WaitForImuMessageHandler::shutdown();
        sick_scan_xd::WaitForLFErecMessageHandler::shutdown();
        sick_scan_xd::WaitForLIDoutputstateMessageHandler::shutdown();
        sick_scan_xd::WaitForRadarScanMessageHandler::shutdown();
        sick_scan_xd::WaitForLdmrsObjectArrayMessageHandler::shutdown();
        sick_scan_xd::WaitForVisualizationMarkerMessageHandler::shutdown();
        sick_scan_xd::WaitForNAVPOSDataMessageHandler::shutdown();
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiClose(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiClose(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}

/*
*  Registration / deregistration of message callbacks
*/

// Register / deregister a callback for cartesian PointCloud messages, pointcloud in cartesian coordinates with fields x, y, z, intensity
int32_t SickScanApiRegisterCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiRegisterCartesianPointCloudMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_cartesian_pointcloud_messages.addListener(apiHandle, callback);
        rosNodePtr node = castApiHandleToNode(apiHandle);
        sick_scan_xd::addCartesianPointcloudListener(node, cartesian_pointcloud_callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterCartesianPointCloudMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterCartesianPointCloudMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}
int32_t SickScanApiDeregisterCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterCartesianPointCloudMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_cartesian_pointcloud_messages.removeListener(apiHandle, callback);
        rosNodePtr node = castApiHandleToNode(apiHandle);
        sick_scan_xd::removeCartesianPointcloudListener(node, cartesian_pointcloud_callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterCartesianPointCloudMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterCartesianPointCloudMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}

// Register / deregister a callback for polar PointCloud messages, pointcloud in polar coordinates with fields range, azimuth, elevation, intensity
int32_t SickScanApiRegisterPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiRegisterPolarPointCloudMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_polar_pointcloud_messages.addListener(apiHandle, callback);
        rosNodePtr node = castApiHandleToNode(apiHandle);
        sick_scan_xd::addPolarPointcloudListener(node, polar_pointcloud_callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterPolarPointCloudMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterPolarPointCloudMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}
int32_t SickScanApiDeregisterPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterPolarPointCloudMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_polar_pointcloud_messages.removeListener(apiHandle, callback);
        rosNodePtr node = castApiHandleToNode(apiHandle);
        sick_scan_xd::removePolarPointcloudListener(node, polar_pointcloud_callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterPolarPointCloudMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterPolarPointCloudMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}

// Register / deregister a callback for Imu messages
int32_t SickScanApiRegisterImuMsg(SickScanApiHandle apiHandle, SickScanImuMsgCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiRegisterImuMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_imu_messages.addListener(apiHandle, callback);
        rosNodePtr node = castApiHandleToNode(apiHandle);
        sick_scan_xd::addImuListener(node, imu_callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterImuMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterImuMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}
int32_t SickScanApiDeregisterImuMsg(SickScanApiHandle apiHandle, SickScanImuMsgCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterImuMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_imu_messages.removeListener(apiHandle, callback);
        rosNodePtr node = castApiHandleToNode(apiHandle);
        sick_scan_xd::removeImuListener(node, imu_callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterImuMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterImuMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}

// Register / deregister a callback for SickScanLFErecMsg messages
int32_t SickScanApiRegisterLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsgCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiRegisterLFErecMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_lferec_messages.addListener(apiHandle, callback);
        rosNodePtr node = castApiHandleToNode(apiHandle);
        sick_scan_xd::addLFErecListener(node, lferec_callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterLFErecMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterLFErecMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}
int32_t SickScanApiDeregisterLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsgCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterLFErecMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_lferec_messages.removeListener(apiHandle, callback);
        rosNodePtr node = castApiHandleToNode(apiHandle);
        sick_scan_xd::removeLFErecListener(node, lferec_callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterLFErecMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterLFErecMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}

// Register / deregister a callback for SickScanLIDoutputstateMsg messages
int32_t SickScanApiRegisterLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsgCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiRegisterLIDoutputstateMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_lidoutputstate_messages.addListener(apiHandle, callback);
        rosNodePtr node = castApiHandleToNode(apiHandle);
        sick_scan_xd::addLIDoutputstateListener(node, lidoutputstate_callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterLIDoutputstateMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterLIDoutputstateMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}
int32_t SickScanApiDeregisterLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsgCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterLIDoutputstateMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_lidoutputstate_messages.removeListener(apiHandle, callback);
        rosNodePtr node = castApiHandleToNode(apiHandle);
        sick_scan_xd::removeLIDoutputstateListener(node, lidoutputstate_callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterLIDoutputstateMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterLIDoutputstateMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}

// Register / deregister a callback for SickScanRadarScan messages
int32_t SickScanApiRegisterRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScanCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiRegisterRadarScanMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_radarscan_messages.addListener(apiHandle, callback);
        rosNodePtr node = castApiHandleToNode(apiHandle);
        sick_scan_xd::addRadarScanListener(node, radarscan_callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterRadarScanMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterRadarScanMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}
int32_t SickScanApiDeregisterRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScanCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterRadarScanMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_radarscan_messages.removeListener(apiHandle, callback);
        rosNodePtr node = castApiHandleToNode(apiHandle);
        sick_scan_xd::removeRadarScanListener(node, radarscan_callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterRadarScanMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterRadarScanMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}

// Register / deregister a callback for SickScanLdmrsObjectArray messages
int32_t SickScanApiRegisterLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArrayCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiRegisterLdmrsObjectArrayMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_ldmrsobjectarray_messages.addListener(apiHandle, callback);
        rosNodePtr node = castApiHandleToNode(apiHandle);
        sick_scan_xd::addLdmrsObjectArrayListener(node, ldmrsobjectarray_callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterLdmrsObjectArrayMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterLdmrsObjectArrayMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}
int32_t SickScanApiDeregisterLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArrayCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterLdmrsObjectArrayMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_ldmrsobjectarray_messages.removeListener(apiHandle, callback);
        rosNodePtr node = castApiHandleToNode(apiHandle);
        sick_scan_xd::removeLdmrsObjectArrayListener(node, ldmrsobjectarray_callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterLdmrsObjectArrayMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterLdmrsObjectArrayMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}

// Register / deregister a callback for VisualizationMarker messages
int32_t SickScanApiRegisterVisualizationMarkerMsg(SickScanApiHandle apiHandle, SickScanVisualizationMarkerCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiRegisterVisualizationMarkerMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_visualizationmarker_messages.addListener(apiHandle, callback);
        rosNodePtr node = castApiHandleToNode(apiHandle);
        sick_scan_xd::addVisualizationMarkerListener(node, visualizationmarker_callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterVisualizationMarkerMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterVisualizationMarkerMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}
int32_t SickScanApiDeregisterVisualizationMarkerMsg(SickScanApiHandle apiHandle, SickScanVisualizationMarkerCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterVisualizationMarkerMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_visualizationmarker_messages.removeListener(apiHandle, callback);
        rosNodePtr node = castApiHandleToNode(apiHandle);
        sick_scan_xd::removeVisualizationMarkerListener(node, visualizationmarker_callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterVisualizationMarkerMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterVisualizationMarkerMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}

/*
*  Functions for diagnostic and logging
*/
// Register a callback for diagnostic messages (notification in case of changed status, e.g. after errors)
int32_t SickScanApiRegisterDiagnosticMsg(SickScanApiHandle apiHandle, SickScanDiagnosticMsgCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiRegisterDiagnosticMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_diagnostic_messages.addListener(apiHandle, callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterDiagnosticMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterDiagnosticMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}

// Deregister a callback for diagnostic messages (notification in case of changed status, e.g. after errors)
int32_t SickScanApiDeregisterDiagnosticMsg(SickScanApiHandle apiHandle, SickScanDiagnosticMsgCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterDiagnosticMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_diagnostic_messages.removeListener(apiHandle, callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterDiagnosticMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterDiagnosticMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}

// Register a callback for log messages (all informational and error messages)
int32_t SickScanApiRegisterLogMsg(SickScanApiHandle apiHandle, SickScanLogMsgCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiRegisterLogMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_log_messages.addListener(apiHandle, callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterLogMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterLogMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}

// Deregister a callback for log messages (all informational and error messages)
int32_t SickScanApiDeregisterLogMsg(SickScanApiHandle apiHandle, SickScanLogMsgCallback callback)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterLogMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_log_messages.removeListener(apiHandle, callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterLogMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterLogMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}

// Query current status and status message
int32_t SickScanApiGetStatus(SickScanApiHandle apiHandle, int32_t* status_code, char* message_buffer, int32_t message_buffer_size)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiGetStatus(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        SICK_DIAGNOSTIC_STATUS diagnostic_code = SICK_DIAGNOSTIC_STATUS::WARN;
        std::string diagnostic_message;
        getDiagnosticStatus(diagnostic_code, diagnostic_message);
        int32_t len = std::min<int32_t>(message_buffer_size, (int32_t)diagnostic_message.length() + 1);
        *status_code = diagnostic_code;
        strncpy(message_buffer, diagnostic_message.c_str(), len);
        message_buffer[len-1] = '\0';
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiGetStatus(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiGetStatus(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}

// Sends a SOPAS command like "sRN SCdevicestate" or "sRN ContaminationResult" and returns the lidar response
int32_t SickScanApiSendSOPAS(SickScanApiHandle apiHandle, const char* sopas_command, char* sopas_response_buffer, int32_t response_buffer_size)
{
  try
  {
    if (apiHandle == 0)
    {
      ROS_ERROR_STREAM("## ERROR SickScanApiSendSOPAS(): invalid apiHandle");
      return SICK_SCAN_API_NOT_INITIALIZED;
    }
    std::string sopas_ascii_request = sopas_command;
    std::string sopas_response;
    if (!convertSendSOPASCommand(sopas_ascii_request, sopas_response, true))
    {
      ROS_ERROR_STREAM("## ERROR SickScanApiSendSOPAS(): convertSendSOPASCommand(\"" << sopas_ascii_request << "\") failed");
      return SICK_SCAN_API_ERROR;
    }
    if (sopas_response.length() >= response_buffer_size)
    {
      ROS_WARN_STREAM("## ERROR SickScanApiSendSOPAS(\"" << sopas_ascii_request << "\"): response_buffer_size " << response_buffer_size << " too small, response \"" << sopas_response << "\" requires at least " << (sopas_response.length() + 1) << " bytes, response truncated");
    }
    strncpy(sopas_response_buffer, sopas_response.c_str(), response_buffer_size - 1);
    sopas_response_buffer[response_buffer_size - 1] = '\0';
    return SICK_SCAN_API_SUCCESS;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("## ERROR SickScanApiSendSOPAS(): exception " << e.what());
  }
  catch (...)
  {
    ROS_ERROR_STREAM("## ERROR SickScanApiSendSOPAS(): unknown exception ");
  }
  return SICK_SCAN_API_ERROR;
}

// Set verbose level 0=DEBUG, 1=INFO, 2=WARN, 3=ERROR, 4=FATAL or 5=QUIET (equivalent to ros::console::levels),
// i.e. print messages on console above the given verbose level.
// Default verbose level is 1 (INFO), i.e. print informational, warnings and error messages.
int32_t SickScanApiSetVerboseLevel(SickScanApiHandle apiHandle, int32_t verbose_level)
{
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiSetVerboseLevel(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        setVerboseLevel(verbose_level);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiSetVerboseLevel(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiSetVerboseLevel(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}

// Returns the current verbose level 0=DEBUG, 1=INFO, 2=WARN, 3=ERROR, 4=FATAL or 5=QUIET. Default verbose level is 1 (INFO)
int32_t SickScanApiGetVerboseLevel(SickScanApiHandle apiHandle)
{
    int32_t verbose_level = 1;
    try
    {
        if (apiHandle == 0)
            ROS_ERROR_STREAM("## ERROR getVerboseLevel(): invalid apiHandle");
        verbose_level = getVerboseLevel();
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR getVerboseLevel(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR getVerboseLevel(): unknown exception ");
    }
    return verbose_level;
}

// Notifies all registered log message listener, i.e. all registered listener callbacks are called for all messages of type INFO, WARN, ERROR or FATAL 
void notifyLogMessageListener(int msg_level, const std::string& message)
{
    SickScanLogMsg msg;
    msg.log_level = msg_level;
    msg.log_message = (char*)calloc(message.length() + 1, sizeof(char));
    strncpy(msg.log_message, message.c_str(), message.length());
    s_callback_handler_log_messages.notifyListener(&msg);
    free(msg.log_message);
    // std::cout << "SICK_LOG_MESSAGE " << msg_level << ": \"" << message << "\""<< std::endl;
}

// Notifies all registered listener about a new diagnostic status
void notifyDiagnosticListener(SICK_DIAGNOSTIC_STATUS status_code, const std::string& status_message)
{
    SickScanDiagnosticMsg msg;
    msg.status_code = status_code;
    msg.status_message = (char*)calloc(status_message.length() + 1, sizeof(char));
    strncpy(msg.status_message, status_message.c_str(), status_message.length());
    s_callback_handler_diagnostic_messages.notifyListener(&msg);
    free(msg.status_message);
    // std::cout << "SICK_DIAGNOSTIC_STATUS " << status_code << ": \"" << status_message << "\""<< std::endl;
}

/*
*  Polling functions
*/

// Wait for and return the next cartesian resp. polar PointCloud messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg, double timeout_sec)
{
    int32_t ret_val = SICK_SCAN_API_ERROR;
    try
    {
        memset(msg, 0, sizeof(*msg));
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextCartesianPointCloudMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        if (!rosOk())
        {
          ROS_WARN_STREAM("SickScanApiWaitNext closing or uninitialized");
          return SICK_SCAN_API_NOT_INITIALIZED;
        }
        rosNodePtr node = castApiHandleToNode(apiHandle);
        if (!sick_scan_xd::isCartesianPointcloudListenerRegistered(node, sick_scan_xd::WaitForCartesianPointCloudMessageHandler::messageCallback))
            sick_scan_xd::addCartesianPointcloudListener(node, sick_scan_xd::WaitForCartesianPointCloudMessageHandler::messageCallback); // registrate static SickWaitForMessageHandler callback once
        sick_scan_xd::WaitForCartesianPointCloudMessageHandler wait_message_handler;
        sick_scan_xd::WaitForCartesianPointCloudMessageHandler::addWaitForMessageHandlerHandler(&wait_message_handler);
        sick_scan_xd::PointCloud2withEcho ros_msg;
        if (wait_message_handler.waitForNextMessage(ros_msg, timeout_sec) 
            && ros_msg.pointcloud.width * ros_msg.pointcloud.height > 0
            && ros_msg.pointcloud.fields.size() >= 3
            && ros_msg.pointcloud.fields[0].name == "x"
            && ros_msg.pointcloud.fields[1].name == "y"
            && ros_msg.pointcloud.fields[2].name == "z")
        {
            // ros_sensor_msgs::PointCloud2 message received, convert to SickScanPointCloudMsg
            ROS_INFO_STREAM("SickScanApiWaitNextCartesianPointCloudMsg: PointCloud2 message, " << ros_msg.pointcloud.width << "x" << ros_msg.pointcloud.height << " points");
            *msg = convertPointCloudMsg(ros_msg);
            ret_val = SICK_SCAN_API_SUCCESS;
        }
        else
        {
            ret_val = SICK_SCAN_API_TIMEOUT;
        }
        sick_scan_xd::WaitForCartesianPointCloudMessageHandler::removeWaitForMessageHandlerHandler(&wait_message_handler);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextCartesianPointCloudMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextCartesianPointCloudMsg(): unknown exception ");
    }
    return ret_val;
}
int32_t SickScanApiWaitNextPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg, double timeout_sec)
{
    int32_t ret_val = SICK_SCAN_API_ERROR;
    try
    {
        memset(msg, 0, sizeof(*msg));
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextPolarPointCloudMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        if (!rosOk())
        {
          ROS_WARN_STREAM("SickScanApiWaitNext closing or uninitialized");
          return SICK_SCAN_API_NOT_INITIALIZED;
        }
        rosNodePtr node = castApiHandleToNode(apiHandle);
        if (!sick_scan_xd::isPolarPointcloudListenerRegistered(node, sick_scan_xd::WaitForPolarPointCloudMessageHandler::messageCallback))
            sick_scan_xd::addPolarPointcloudListener(node, sick_scan_xd::WaitForPolarPointCloudMessageHandler::messageCallback); // registrate static SickWaitForMessageHandler callback once
        sick_scan_xd::WaitForPolarPointCloudMessageHandler wait_message_handler;
        sick_scan_xd::WaitForPolarPointCloudMessageHandler::addWaitForMessageHandlerHandler(&wait_message_handler);
        sick_scan_xd::PointCloud2withEcho ros_msg;
        if (wait_message_handler.waitForNextMessage(ros_msg, timeout_sec) 
            && ros_msg.pointcloud.width * ros_msg.pointcloud.height > 0
            && ros_msg.pointcloud.fields.size() >= 3
            && ros_msg.pointcloud.fields[0].name == "range"
            && ros_msg.pointcloud.fields[1].name == "azimuth"
            && ros_msg.pointcloud.fields[2].name == "elevation")
        {
            // ros_sensor_msgs::PointCloud2 message received, convert to SickScanPointCloudMsg
            ROS_INFO_STREAM("SickScanApiWaitNextPolarPointCloudMsg: PointCloud2 message, " << ros_msg.pointcloud.width << "x" << ros_msg.pointcloud.height << " points");
            *msg = convertPointCloudMsg(ros_msg);
            ret_val = SICK_SCAN_API_SUCCESS;
        }
        else
        {
            ret_val = SICK_SCAN_API_TIMEOUT;
        }
        sick_scan_xd::WaitForPolarPointCloudMessageHandler::removeWaitForMessageHandlerHandler(&wait_message_handler);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextPolarPointCloudMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextPolarPointCloudMsg(): unknown exception ");
    }
    return ret_val;
}
int32_t SickScanApiFreePointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg)
{
    if(apiHandle && msg)
    {
        freePointCloudMsg(*msg);
        return SICK_SCAN_API_SUCCESS;
    }
    return SICK_SCAN_API_NOT_INITIALIZED;
}

// Wait for and return the next Imu messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextImuMsg(SickScanApiHandle apiHandle, SickScanImuMsg* msg, double timeout_sec)
{
    int32_t ret_val = SICK_SCAN_API_ERROR;
    try
    {
        memset(msg, 0, sizeof(*msg));
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextImuMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        if (!rosOk())
        {
          ROS_WARN_STREAM("SickScanApiWaitNext closing or uninitialized");
          return SICK_SCAN_API_NOT_INITIALIZED;
        }
        rosNodePtr node = castApiHandleToNode(apiHandle);
        if (!sick_scan_xd::isImuListenerRegistered(node, sick_scan_xd::WaitForImuMessageHandler::messageCallback))
            sick_scan_xd::addImuListener(node, sick_scan_xd::WaitForImuMessageHandler::messageCallback);
        sick_scan_xd::WaitForImuMessageHandler wait_message_handler;
        sick_scan_xd::WaitForImuMessageHandler::addWaitForMessageHandlerHandler(&wait_message_handler);
        ros_sensor_msgs::Imu ros_msg;
        if (wait_message_handler.waitForNextMessage(ros_msg, timeout_sec))
        {
            // ros_sensor_msgs::PointCloud2 message received, convert to SickScanPointCloudMsg
            ROS_INFO_STREAM("SickScanApiWaitNextImuMsg: Imu message");
            *msg = convertImuMsg(ros_msg);
            ret_val = SICK_SCAN_API_SUCCESS;
        }
        else
        {
            ret_val = SICK_SCAN_API_TIMEOUT;
        }
        sick_scan_xd::WaitForImuMessageHandler::removeWaitForMessageHandlerHandler(&wait_message_handler);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextImuMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextImuMsg(): unknown exception ");
    }
    return ret_val;
}
int32_t SickScanApiFreeImuMsg(SickScanApiHandle apiHandle, SickScanImuMsg* msg)
{
    if(apiHandle && msg)
    {
        freeImuMsg(*msg);
        return SICK_SCAN_API_SUCCESS;
    }
    return SICK_SCAN_API_NOT_INITIALIZED;
}

// Wait for and return the next LFErec messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsg* msg, double timeout_sec)
{
    int32_t ret_val = SICK_SCAN_API_ERROR;
    try
    {
        memset(msg, 0, sizeof(*msg));
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextLFErecMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        if (!rosOk())
        {
          ROS_WARN_STREAM("SickScanApiWaitNext closing or uninitialized");
          return SICK_SCAN_API_NOT_INITIALIZED;
        }
        rosNodePtr node = castApiHandleToNode(apiHandle);
        if (!sick_scan_xd::isLFErecListenerRegistered(node, sick_scan_xd::WaitForLFErecMessageHandler::messageCallback))
            sick_scan_xd::addLFErecListener(node, sick_scan_xd::WaitForLFErecMessageHandler::messageCallback);
        sick_scan_xd::WaitForLFErecMessageHandler wait_message_handler;
        sick_scan_xd::WaitForLFErecMessageHandler::addWaitForMessageHandlerHandler(&wait_message_handler);
        sick_scan_msg::LFErecMsg ros_msg;
        if (wait_message_handler.waitForNextMessage(ros_msg, timeout_sec) && ros_msg.fields_number > 0)
        {
            // ros_sensor_msgs::PointCloud2 message received, convert to SickScanPointCloudMsg
            ROS_INFO_STREAM("SickScanApiWaitNextLFErecMsg: LFErec message, " << ros_msg.fields_number << " fields");
            *msg = convertLFErecMsg(ros_msg);
            ret_val = SICK_SCAN_API_SUCCESS;
        }
        else
        {
            ret_val = SICK_SCAN_API_TIMEOUT;
        }
        sick_scan_xd::WaitForLFErecMessageHandler::removeWaitForMessageHandlerHandler(&wait_message_handler);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextLFErecMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextLFErecMsg(): unknown exception ");
    }
    return ret_val;
}
int32_t SickScanApiFreeLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsg* msg)
{
    if(apiHandle && msg)
    {
        freeLFErecMsg(*msg);
        return SICK_SCAN_API_SUCCESS;
    }
    return SICK_SCAN_API_NOT_INITIALIZED;
}

// Wait for and return the next LIDoutputstate messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsg* msg, double timeout_sec)
{
    int32_t ret_val = SICK_SCAN_API_ERROR;
    try
    {
        memset(msg, 0, sizeof(*msg));
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextLIDoutputstateMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        if (!rosOk())
        {
          ROS_WARN_STREAM("SickScanApiWaitNext closing or uninitialized");
          return SICK_SCAN_API_NOT_INITIALIZED;
        }
        rosNodePtr node = castApiHandleToNode(apiHandle);
        if (!sick_scan_xd::isLIDoutputstateListenerRegistered(node, sick_scan_xd::WaitForLIDoutputstateMessageHandler::messageCallback))
            sick_scan_xd::addLIDoutputstateListener(node, sick_scan_xd::WaitForLIDoutputstateMessageHandler::messageCallback);
        sick_scan_xd::WaitForLIDoutputstateMessageHandler wait_message_handler;
        sick_scan_xd::WaitForLIDoutputstateMessageHandler::addWaitForMessageHandlerHandler(&wait_message_handler);
        sick_scan_msg::LIDoutputstateMsg ros_msg;
        if (wait_message_handler.waitForNextMessage(ros_msg, timeout_sec) && ros_msg.output_state.size() + ros_msg.output_count.size() > 0)
        {
            // ros_sensor_msgs::PointCloud2 message received, convert to SickScanPointCloudMsg
            ROS_INFO_STREAM("SickScanApiWaitNextLIDoutputstateMsg: LIDoutputstate message, " << ros_msg.output_state.size() << " states, " << ros_msg.output_count.size() << " counters");
            *msg = convertLIDoutputstateMsg(ros_msg);
            ret_val = SICK_SCAN_API_SUCCESS;
        }
        else
        {
            ret_val = SICK_SCAN_API_TIMEOUT;
        }
        sick_scan_xd::WaitForLIDoutputstateMessageHandler::removeWaitForMessageHandlerHandler(&wait_message_handler);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextLIDoutputstateMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextLIDoutputstateMsg(): unknown exception ");
    }
    return ret_val;
}
int32_t SickScanApiFreeLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsg* msg)
{
    if(apiHandle && msg)
    {
        freeLIDoutputstateMsg(*msg);
        return SICK_SCAN_API_SUCCESS;
    }
    return SICK_SCAN_API_NOT_INITIALIZED;
}

// Wait for and return the next RadarScan messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScan* msg, double timeout_sec)
{
    int32_t ret_val = SICK_SCAN_API_ERROR;
    try
    {
        memset(msg, 0, sizeof(*msg));
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextRadarScanMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        if (!rosOk())
        {
          ROS_WARN_STREAM("SickScanApiWaitNext closing or uninitialized");
          return SICK_SCAN_API_NOT_INITIALIZED;
        }
        rosNodePtr node = castApiHandleToNode(apiHandle);
        if (!sick_scan_xd::isRadarScanListenerRegistered(node, sick_scan_xd::WaitForRadarScanMessageHandler::messageCallback))
            sick_scan_xd::addRadarScanListener(node, sick_scan_xd::WaitForRadarScanMessageHandler::messageCallback);
        sick_scan_xd::WaitForRadarScanMessageHandler wait_message_handler;
        sick_scan_xd::WaitForRadarScanMessageHandler::addWaitForMessageHandlerHandler(&wait_message_handler);
        sick_scan_msg::RadarScan ros_msg;
        if (wait_message_handler.waitForNextMessage(ros_msg, timeout_sec) && ros_msg.targets.width * ros_msg.targets.height + ros_msg.objects.size() > 0)
        {
            // ros_sensor_msgs::PointCloud2 message received, convert to SickScanPointCloudMsg
            ROS_INFO_STREAM("SickScanApiWaitNextRadarScanMsg: RadarScan message, " << (ros_msg.targets.width * ros_msg.targets.height) << " targets, " << ros_msg.objects.size() << " objects");
            *msg = convertRadarScanMsg(ros_msg);
            ret_val = SICK_SCAN_API_SUCCESS;
        }
        else
        {
            ret_val = SICK_SCAN_API_TIMEOUT;
        }
        sick_scan_xd::WaitForRadarScanMessageHandler::removeWaitForMessageHandlerHandler(&wait_message_handler);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextRadarScanMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextRadarScanMsg(): unknown exception ");
    }
    return ret_val;
}
int32_t SickScanApiFreeRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScan* msg)
{
    if(apiHandle && msg)
    {
        freeRadarScanMsg(*msg);
        return SICK_SCAN_API_SUCCESS;
    }
    return SICK_SCAN_API_NOT_INITIALIZED;
}

// Wait for and return the next LdmrsObjectArray messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArray* msg, double timeout_sec)
{
    int32_t ret_val = SICK_SCAN_API_ERROR;
    try
    {
        memset(msg, 0, sizeof(*msg));
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextLdmrsObjectArrayMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        if (!rosOk())
        {
          ROS_WARN_STREAM("SickScanApiWaitNext closing or uninitialized");
          return SICK_SCAN_API_NOT_INITIALIZED;
        }
        rosNodePtr node = castApiHandleToNode(apiHandle);
        if (!sick_scan_xd::isLdmrsObjectArrayListenerRegistered(node, sick_scan_xd::WaitForLdmrsObjectArrayMessageHandler::messageCallback))
            sick_scan_xd::addLdmrsObjectArrayListener(node, sick_scan_xd::WaitForLdmrsObjectArrayMessageHandler::messageCallback);
        sick_scan_xd::WaitForLdmrsObjectArrayMessageHandler wait_message_handler;
        sick_scan_xd::WaitForLdmrsObjectArrayMessageHandler::addWaitForMessageHandlerHandler(&wait_message_handler);
        sick_scan_msg::SickLdmrsObjectArray ros_msg;
        if (wait_message_handler.waitForNextMessage(ros_msg, timeout_sec) && ros_msg.objects.size() > 0)
        {
            // ros_sensor_msgs::PointCloud2 message received, convert to SickScanPointCloudMsg
            ROS_INFO_STREAM("SickScanApiWaitNextLdmrsObjectArrayMsg: LdmrsObjectArray message, " << ros_msg.objects.size() << " objects");
            *msg = convertLdmrsObjectArrayMsg(ros_msg);
            ret_val = SICK_SCAN_API_SUCCESS;
        }
        else
        {
            ret_val = SICK_SCAN_API_TIMEOUT;
        }
        sick_scan_xd::WaitForLdmrsObjectArrayMessageHandler::removeWaitForMessageHandlerHandler(&wait_message_handler);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextLdmrsObjectArrayMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextLdmrsObjectArrayMsg(): unknown exception ");
    }
    return ret_val;
}
int32_t SickScanApiFreeLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArray* msg)
{
    if(apiHandle && msg)
    {
        freeLdmrsObjectArrayMsg(*msg);
        return SICK_SCAN_API_SUCCESS;
    }
    return SICK_SCAN_API_NOT_INITIALIZED;
}

// Wait for and return the next VisualizationMarker message. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextVisualizationMarkerMsg(SickScanApiHandle apiHandle, SickScanVisualizationMarkerMsg* msg, double timeout_sec)
{
    int32_t ret_val = SICK_SCAN_API_ERROR;
    try
    {
        memset(msg, 0, sizeof(*msg));
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextVisualizationMarkerMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        if (!rosOk())
        {
          ROS_WARN_STREAM("SickScanApiWaitNext closing or uninitialized");
          return SICK_SCAN_API_NOT_INITIALIZED;
        }
        rosNodePtr node = castApiHandleToNode(apiHandle);
        if (!sick_scan_xd::isVisualizationMarkerListenerRegistered(node, sick_scan_xd::WaitForVisualizationMarkerMessageHandler::messageCallback))
            sick_scan_xd::addVisualizationMarkerListener(node, sick_scan_xd::WaitForVisualizationMarkerMessageHandler::messageCallback);
        sick_scan_xd::WaitForVisualizationMarkerMessageHandler wait_message_handler;
        sick_scan_xd::WaitForVisualizationMarkerMessageHandler::addWaitForMessageHandlerHandler(&wait_message_handler);
        ros_visualization_msgs::MarkerArray ros_msg;
        if (wait_message_handler.waitForNextMessage(ros_msg, timeout_sec) && ros_msg.markers.size() > 0)
        {
            // ros_sensor_msgs::PointCloud2 message received, convert to SickScanPointCloudMsg
            ROS_INFO_STREAM("SickScanApiWaitNextVisualizationMarkerMsg: VisualizationMarker message, " << ros_msg.markers.size() << " markers");
            *msg = convertVisualizationMarkerMsg(ros_msg);
            ret_val = SICK_SCAN_API_SUCCESS;
        }
        else
        {
            ret_val = SICK_SCAN_API_TIMEOUT;
        }
        sick_scan_xd::WaitForVisualizationMarkerMessageHandler::removeWaitForMessageHandlerHandler(&wait_message_handler);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextVisualizationMarkerMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextVisualizationMarkerMsg(): unknown exception ");
    }
    return ret_val;
}
int32_t SickScanApiFreeVisualizationMarkerMsg(SickScanApiHandle apiHandle, SickScanVisualizationMarkerMsg* msg)
{
    if(apiHandle && msg)
    {
        freeVisualizationMarkerMsg(*msg);
        return SICK_SCAN_API_SUCCESS;
    }
    return SICK_SCAN_API_NOT_INITIALIZED;
}

/*
** NAV-350 support and messages
*/

static SickScanNavPoseLandmarkMsg convertNAV350mNPOSData(const sick_scan_xd::NAV350mNPOSData& src_msg)
{
    SickScanNavPoseLandmarkMsg dst_msg;
    memset(&dst_msg, 0, sizeof(dst_msg));
    dst_msg.pose_valid = src_msg.poseDataValid;
    dst_msg.pose_nav_x = src_msg.poseData.x;
    dst_msg.pose_nav_y = src_msg.poseData.y;
    dst_msg.pose_nav_phi = src_msg.poseData.phi;
    dst_msg.pose_opt_valid = src_msg.poseData.optPoseDataValid;
    dst_msg.pose_opt_output_mode = src_msg.poseData.optPoseData.outputMode;
    dst_msg.pose_opt_timestamp = src_msg.poseData.optPoseData.timestamp;
    dst_msg.pose_opt_mean_dev = src_msg.poseData.optPoseData.meanDev;
    dst_msg.pose_opt_nav_mode = src_msg.poseData.optPoseData.navMode;
    dst_msg.pose_opt_info_state = src_msg.poseData.optPoseData.infoState;
    dst_msg.pose_opt_quant_used_reflectors = src_msg.poseData.optPoseData.quantUsedReflectors;
    if (dst_msg.pose_valid > 0)
        sick_scan_xd::convertNAVCartPos3DtoROSPos3D(dst_msg.pose_nav_x, dst_msg.pose_nav_y, dst_msg.pose_nav_phi, dst_msg.pose_x, dst_msg.pose_y, dst_msg.pose_yaw, src_msg.angleOffset);
    if (dst_msg.pose_opt_valid > 0 && SoftwarePLL::instance().IsInitialized())
        SoftwarePLL::instance().getCorrectedTimeStamp(dst_msg.pose_timestamp_sec, dst_msg.pose_timestamp_nsec, dst_msg.pose_opt_timestamp);

    if (src_msg.landmarkDataValid && src_msg.landmarkData.reflectors.size() > 0)
    {
        dst_msg.reflectors.buffer = (SickScanNavReflector*)malloc(src_msg.landmarkData.reflectors.size() * sizeof(SickScanNavReflector));
        if (dst_msg.reflectors.buffer)
        {
            dst_msg.reflectors.size = src_msg.landmarkData.reflectors.size();
            dst_msg.reflectors.capacity = src_msg.landmarkData.reflectors.size();
            for(int reflector_cnt = 0; reflector_cnt < src_msg.landmarkData.reflectors.size(); reflector_cnt++)
            {
                const sick_scan_xd::NAV350ReflectorData* src_reflector = &src_msg.landmarkData.reflectors[reflector_cnt];
                SickScanNavReflector* dst_reflector = &dst_msg.reflectors.buffer[reflector_cnt];
                dst_reflector->cartesian_valid = src_reflector->cartesianDataValid;
                dst_reflector->cartesian_x = src_reflector->cartesianData.x;
                dst_reflector->cartesian_y = src_reflector->cartesianData.y;
                dst_reflector->polar_valid = src_reflector->polarDataValid;
                dst_reflector->polar_dist = src_reflector->polarData.dist;
                dst_reflector->polar_phi = src_reflector->polarData.phi;
                dst_reflector->opt_valid = src_reflector->optReflectorDataValid;
                dst_reflector->opt_local_id = src_reflector->optReflectorData.localID;
                dst_reflector->opt_global_id = src_reflector->optReflectorData.globalID;
                dst_reflector->opt_type = src_reflector->optReflectorData.type;
                dst_reflector->opt_subtype = src_reflector->optReflectorData.subType;
                dst_reflector->opt_quality = src_reflector->optReflectorData.quality;
                dst_reflector->opt_timestamp = src_reflector->optReflectorData.timestamp;
                dst_reflector->opt_size = src_reflector->optReflectorData.size;
                dst_reflector->opt_hitcount = src_reflector->optReflectorData.hitCount;
                dst_reflector->opt_meanecho = src_reflector->optReflectorData.meanEcho;
                dst_reflector->opt_startindex = src_reflector->optReflectorData.startIndex;
                dst_reflector->opt_endindex = src_reflector->optReflectorData.endIndex;
                dst_reflector->pos_valid = src_reflector->cartesianDataValid;
                if (src_reflector->cartesianDataValid)
                    sick_scan_xd::convertNAVCartPos2DtoROSPos2D(src_reflector->cartesianData.x, src_reflector->cartesianData.y, dst_reflector->pos_x, dst_reflector->pos_y, src_msg.angleOffset);
                if (src_reflector->optReflectorDataValid > 0 && SoftwarePLL::instance().IsInitialized())
                    SoftwarePLL::instance().getCorrectedTimeStamp(dst_reflector->opt_timestamp_sec, dst_reflector->opt_timestamp_nsec, src_reflector->optReflectorData.timestamp);
            }
        }
    }
    return dst_msg;
}
static void freeNavPoseLandmarkMsg(SickScanNavPoseLandmarkMsg& msg)
{
    free(msg.reflectors.buffer);
    memset(&msg, 0, sizeof(msg));
}
static void nav_pose_landmark_callback(rosNodePtr node, const sick_scan_xd::NAV350mNPOSData* msg)
{
    ROS_DEBUG_STREAM("api_impl nav_pose_landmark_callback: NAV350mNPOSData message");
    SickScanNavPoseLandmarkMsg export_msg = convertNAV350mNPOSData(*msg);
    SickScanApiHandle apiHandle = castNodeToApiHandle(node);
    s_callback_handler_navposelandmark_messages.notifyListener(apiHandle, &export_msg);
    freeNavPoseLandmarkMsg(export_msg);
}
int32_t SickScanApiRegisterNavPoseLandmarkMsg(SickScanApiHandle apiHandle, SickScanNavPoseLandmarkCallback callback)
{   
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiRegisterNavPoseLandmarkMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_navposelandmark_messages.addListener(apiHandle, callback);
        rosNodePtr node = castApiHandleToNode(apiHandle);
        sick_scan_xd::addNavPoseLandmarkListener(node, nav_pose_landmark_callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterNavPoseLandmarkMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiRegisterNavPoseLandmarkMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}
int32_t SickScanApiDeregisterNavPoseLandmarkMsg(SickScanApiHandle apiHandle, SickScanNavPoseLandmarkCallback callback)
{   
    try
    {
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterNavPoseLandmarkMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        s_callback_handler_navposelandmark_messages.removeListener(apiHandle, callback);
        rosNodePtr node = castApiHandleToNode(apiHandle);
        sick_scan_xd::removeNavPoseLandmarkListener(node, nav_pose_landmark_callback);
        return SICK_SCAN_API_SUCCESS;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterNavPoseLandmarkMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiDeregisterNavPoseLandmarkMsg(): unknown exception ");
    }
    return SICK_SCAN_API_ERROR;
}
int32_t SickScanApiWaitNextNavPoseLandmarkMsg(SickScanApiHandle apiHandle, SickScanNavPoseLandmarkMsg* msg, double timeout_sec)
{   
    int32_t ret_val = SICK_SCAN_API_ERROR;
    try
    {
        memset(msg, 0, sizeof(*msg));
        if (apiHandle == 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextNavPoseLandmarkMsg(): invalid apiHandle");
            return SICK_SCAN_API_NOT_INITIALIZED;
        }
        rosNodePtr node = castApiHandleToNode(apiHandle);
        if (!sick_scan_xd::isNavPoseLandmarkListenerRegistered(node, sick_scan_xd::WaitForNAVPOSDataMessageHandler::messageCallback))
            sick_scan_xd::addNavPoseLandmarkListener(node, sick_scan_xd::WaitForNAVPOSDataMessageHandler::messageCallback);
        sick_scan_xd::WaitForNAVPOSDataMessageHandler wait_message_handler;
        sick_scan_xd::WaitForNAVPOSDataMessageHandler::addWaitForMessageHandlerHandler(&wait_message_handler);
        sick_scan_xd::NAV350mNPOSData navdata_msg;
        if (wait_message_handler.waitForNextMessage(navdata_msg, timeout_sec) && (navdata_msg.poseDataValid > 0 || navdata_msg.landmarkDataValid > 0))
        {
            ROS_INFO_STREAM("SickScanApiWaitNextNavPoseLandmarkMsg: NAV350mNPOSData message");
            *msg = convertNAV350mNPOSData(navdata_msg);
            ret_val = SICK_SCAN_API_SUCCESS;
        }
        else
        {
            ret_val = SICK_SCAN_API_TIMEOUT;
        }
        sick_scan_xd::WaitForNAVPOSDataMessageHandler::removeWaitForMessageHandlerHandler(&wait_message_handler);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextNavPoseLandmarkMsg(): exception " << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("## ERROR SickScanApiWaitNextNavPoseLandmarkMsg(): unknown exception ");
    }
    return ret_val;
}
int32_t SickScanApiFreeNavPoseLandmarkMsg(SickScanApiHandle apiHandle, SickScanNavPoseLandmarkMsg* msg)
{   
    if(apiHandle && msg)
    {
        freeNavPoseLandmarkMsg(*msg);
        return SICK_SCAN_API_SUCCESS;
    }
    return SICK_SCAN_API_NOT_INITIALIZED;
}
// Send odometry data to NAV350
int32_t SickScanApiNavOdomVelocityImpl(SickScanApiHandle apiHandle, SickScanNavOdomVelocityMsg* msg);
int32_t SickScanApiOdomVelocityImpl(SickScanApiHandle apiHandle, SickScanOdomVelocityMsg* src_msg);
int32_t SickScanApiNavOdomVelocityMsg(SickScanApiHandle apiHandle, SickScanNavOdomVelocityMsg* msg)
{
    return SickScanApiNavOdomVelocityImpl(apiHandle, msg);
}
int32_t SickScanApiOdomVelocityMsg(SickScanApiHandle apiHandle, SickScanOdomVelocityMsg* msg)
{
    return SickScanApiOdomVelocityImpl(apiHandle, msg);
}
