/*
* Copyright (C) 2022, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2022, SICK AG, Waldkirch
* All rights reserved.
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
*      Authors:
*         Michael Lehning <michael.lehning@lehning.de>
*
*/

#ifndef __SICK_SCAN_API_H_INCLUDED
#define __SICK_SCAN_API_H_INCLUDED

/*
* This file declares the functions and datatypes of the sick_scan_xd C-API.
* See doc/sick_scan_api/sick_scan_api.md for further information.
*/

#include <stdint.h>

#ifdef  __cplusplus
extern "C" {
#endif

#ifdef _MSC_VER
#define SICK_SCAN_API_DECLSPEC_EXPORT __declspec(dllexport)
#else
#define SICK_SCAN_API_DECLSPEC_EXPORT __attribute__ ((visibility ("default")))
#endif

#ifndef SICK_SCAN_XD_API_CALLING_CONVENTION
#define SICK_SCAN_XD_API_CALLING_CONVENTION
#endif

/*
*  Message definitions
*/

typedef struct SickScanHeaderType // equivalent to ros::std_msgs::Header
{
  uint32_t seq;            // sequence ID: consecutively increasing ID
  uint32_t timestamp_sec;  // seconds part of message timestamps: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
  uint32_t timestamp_nsec; // nanoseconds part of message timestamps: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
  char frame_id[256];      // Frame this data is associated with
} SickScanHeader;

typedef struct SickScanUint8ArrayType // Array of 8 bit values, which can be serialized and imported in C, C++ or Python
{
  uint64_t capacity; // Number of allocated elements, i.e. max. number of elements in buffer, allocated buffer size is capacity * sizeof(uint8_t)
  uint64_t size;     // Number of currently used elements in the buffer
  uint8_t* buffer;   // Memory, data in plain order (buffer == 0, if size == 0 && capacity == 0, otherwise allocated memory), allocation/deallocation always managed by the caller.
} SickScanUint8Array;

enum SickScanNativeDataType // This message holds the description of one point entry in the PointCloud2 message format, equivalent to type enum im ros::sensor_msgs::PointField
{
  SICK_SCAN_POINTFIELD_DATATYPE_INT8    = 1,
  SICK_SCAN_POINTFIELD_DATATYPE_UINT8   = 2,
  SICK_SCAN_POINTFIELD_DATATYPE_INT16   = 3,
  SICK_SCAN_POINTFIELD_DATATYPE_UINT16  = 4,
  SICK_SCAN_POINTFIELD_DATATYPE_INT32   = 5,
  SICK_SCAN_POINTFIELD_DATATYPE_UINT32  = 6,
  SICK_SCAN_POINTFIELD_DATATYPE_FLOAT32 = 7,
  SICK_SCAN_POINTFIELD_DATATYPE_FLOAT64 = 8
};

typedef struct SickScanPointFieldMsgType // equivalent to ros::sensor_msgs::PointField
{
  // SickScanPointFieldArray is an array of SickScanPointFieldMsg, which defines the structure of the binary data of a SickScanPointCloudMsg.
  // SickScanPointFieldMsg for pointclouds in cartesian coordinates with fields (x, y, z, intensity):
  //     [ SickScanPointFieldMsg(name="x", offset=0, datatype=FLOAT32, count=1),
  //       SickScanPointFieldMsg(name="y", offset=4, datatype=FLOAT32, count=1),
  //       SickScanPointFieldMsg(name="z", offset=8, datatype=FLOAT32, count=1),
  //       SickScanPointFieldMsg(name="intensity", offset=12, datatype=FLOAT32, count=1) ]
  // SickScanPointFieldMsg for pointclouds in polar coordinates with fields (range, azimuth, elevation, intensity):
  //     [ SickScanPointFieldMsg(name="range", offset=0, datatype=FLOAT32, count=1),
  //       SickScanPointFieldMsg(name="azimuth", offset=4, datatype=FLOAT32, count=1),
  //       SickScanPointFieldMsg(name="elevation", offset=8, datatype=FLOAT32, count=1),
  //       SickScanPointFieldMsg(name="intensity", offset=12, datatype=FLOAT32, count=1) ]
  char name[256];     // Name of field (max. length 256 characters)
  uint32_t offset;    // Offset from start of point struct
  uint8_t  datatype;  // Datatype enumeration, see SickScanNativeDataType above
  uint32_t  count;    // How many elements in the field
} SickScanPointFieldMsg;

typedef struct SickScanPointFieldArrayType // Array of SickScanPointFieldMsg, which can be serialized and imported in C, C++ or python
{
  uint64_t capacity; // Number of allocated elements, i.e. max. number of elements in buffer, allocated buffer size is capacity * sizeof(SickScanPointFieldMsg)
  uint64_t size;     // Number of currently used elements in the buffer
  SickScanPointFieldMsg* buffer;  // Memory, data in plain order and system endianess (buffer == 0, if size == 0 && capacity == 0, otherwise allocated memory), allocation/deallocation always managed by the caller.
} SickScanPointFieldArray;

typedef struct SickScanPointCloudMsgType // equivalent to ros::std_msgs::PointCloud2
{
  // SickScanPointCloudMsg contains the polar or cartesian pointcloud data.
  // A SickScanPointCloudMsg in cartesian coordinates has fields (x, y, z, intensity).
  // A SickScanPointCloudMsg in polar coordinates has fields (range, azimuth, elevation, intensity).
  // Note: The pointcloud contains always <num_echos> echos. Invalid echos are filled with 0 values in the data array.
  SickScanHeader header;          // message timestamp
  uint32_t height;                // 2D structure of the point cloud. If the cloud is unordered, height is 1
  uint32_t width;                 // and width is the length of the point cloud.
  SickScanPointFieldArray fields; // Describes the channels and their layout in the binary data blob.
  uint8_t   is_bigendian;         // Is this data bigendian?
  uint32_t  point_step;           // Length of a point in bytes
  uint32_t  row_step;             // Length of a row in bytes
  SickScanUint8Array data;        // Actual point data, size is (row_step*height)
  uint8_t is_dense;               // True if there are no invalid points
  int32_t num_echos;              // number of echos
  int32_t segment_idx;            // segment index (or -1 if pointcloud contains data from multiple segments)
  char topic[256];                // ros topic this pointcloud is published
} SickScanPointCloudMsg;

typedef struct SickScanVector3MsgType // equivalent to geometry_msgs/Vector3
{
  double x;
  double y;
  double z;
} SickScanVector3Msg;

typedef struct SickScanQuaternionMsgType // equivalent to geometry_msgs/Quaternion
{
  double x;
  double y;
  double z;
  double w;
} SickScanQuaternionMsg;

typedef struct SickScanPointArrayType // Array of SickScanVector3Msg, which can be serialized and imported in C, C++ or python
{
  uint64_t capacity; // Number of allocated elements, i.e. max. number of elements in buffer, allocated buffer size is capacity * sizeof(SickScanVector3Msg)
  uint64_t size;     // Number of currently used elements in the buffer
  SickScanVector3Msg* buffer;  // Memory, data in plain order and system endianess (buffer == 0, if size == 0 && capacity == 0, otherwise allocated memory), allocation/deallocation always managed by the caller.
} SickScanPointArray;

typedef struct SickScanImuMsgType // equivalent to ros sensor_msgs::Imu
{
  SickScanHeader header;                       // message timestamp
  SickScanQuaternionMsg orientation;
  double orientation_covariance[9];            // Row major about x, y, z axes
  SickScanVector3Msg angular_velocity;
  double angular_velocity_covariance[9];       // Row major about x, y, z axes
  SickScanVector3Msg linear_acceleration;
  double linear_acceleration_covariance[9];    // Row major x, y z
  char topic[256];                             // ros topic this messages is published
} SickScanImuMsg;

typedef struct SickScanLFErecFieldMsgType // equivalent to LFErecFieldMsg.msg
{
  uint16_t version_number;
  uint8_t field_index;
  uint32_t sys_count;
  float dist_scale_factor;
  float dist_scale_offset;
  uint32_t angle_scale_factor;
  int32_t angle_scale_offset;
  uint8_t field_result_mrs;     // field result is 0 (invalid/incorrect), 1 (free/clear) or 2 (infringed)
  uint16_t time_state;          // No time data: 0, Time data: 1
  uint16_t year;                // f.e. 2021
  uint8_t month;                // 1 ... 12
  uint8_t day;                  // 1 ... 31
  uint8_t hour;                 // 0 ... 23
  uint8_t minute;               // 0 ... 59
  uint8_t second;               // 0 ... 59
  uint32_t microsecond;         // 0 ... 999999
} SickScanLFErecFieldMsg;

typedef struct SickScanLFErecMsgType // equivalent to LFErecMsg.msg
{
  SickScanHeader header;             // message timestamp
  uint16_t fields_number;            // number of valid fields
  SickScanLFErecFieldMsg fields[3];  // max. 3 valid fields
  char topic[256];                   // ros topic this messages is published
} SickScanLFErecMsg;

typedef struct SickScanLIDoutputstateMsgType // equivalent to LIDoutputstateMsg.msg
{
  SickScanHeader header;        // message timestamp
  uint16_t version_number;      // Status code version number
  uint32_t system_counter;      // Status code system counter (time in microsec since power up, max. 71 min then starting from 0 again)
  uint8_t output_state[8];      // array of max. 8 output states, each state with value 0 (not active), 1 (active) or 2 (not used)
  uint32_t output_count[8];     // array of max. 8 output counter
  uint16_t time_state;          // No time data: 0, Time data: 1 (sensortime from the last change of min. one of the outputs)
  uint16_t year;                // f.e. 2021
  uint8_t month;                // 1 ... 12
  uint8_t day;                  // 1 ... 31
  uint8_t hour;                 // 0 ... 23
  uint8_t minute;               // 0 ... 59
  uint8_t second;               // 0 ... 59
  uint32_t microsecond;         // 0 ... 999999
  char topic[256];              // ros topic this messages is published
} SickScanLIDoutputstateMsg;

typedef struct SickScanRadarPreHeaderType // equivalent to RadarPreHeader.msg
{
  uint16_t uiversionno;             // version number
  // sick_scan/RadarPreHeaderDeviceBlock:
  uint32_t uiident;                 // Logical number of the device"
  uint32_t udiserialno;             // Serial number of the device
  uint8_t bdeviceerror;             // State of the device
  uint8_t bcontaminationwarning;    // Contamination Warning
  uint8_t bcontaminationerror;      // Contamination Error
  // sick_scan/RadarPreHeaderStatusBlock:
  uint32_t uitelegramcount;         // telegram number
  uint32_t uicyclecount;            // "scan number"
  uint32_t udisystemcountscan;      // system time since power on of scan gen. [us]
  uint32_t udisystemcounttransmit;  // system time since power on of scan transmission [us]
  uint16_t uiinputs;                // state of digital inputs
  uint16_t uioutputs;               // state of digital outputs
  // sick_scan/RadarPreHeaderMeasurementParam1Block:
  uint32_t uicycleduration;         // Time in microseconds to detect this items
  uint32_t uinoiselevel;            // [1/100dB]
  // sick_scan/RadarPreHeaderEncoderBlock[]:
  uint16_t numencoder;              // number of valid encoders (0)
  uint32_t udiencoderpos[3];        // array of max. 3 encoder position [inc]
  int16_t iencoderspeed[3];         // array of max. 3 encoder speed [inc/mm]
} SickScanRadarPreHeader;

typedef struct SickScanRadarObjectType // equivalent to RadarObject.msg
{
  int32_t id;
  uint32_t tracking_time_sec;  // seconds part of tracking_time (since when the object is tracked): seconds (stamp_secs) since epoch
  uint32_t tracking_time_nsec; // nanoseconds part of tracking_time (since when the object is tracked): nanoseconds since stamp_secs
  uint32_t last_seen_sec;      // seconds part of last_seen timestamp: seconds (stamp_secs) since epoch
  uint32_t last_seen_nsec;     // nanoseconds part of last_seen timestamp: nanoseconds since stamp_secs
  // geometry_msgs/TwistWithCovariance velocity
  SickScanVector3Msg velocity_linear;
  SickScanVector3Msg velocity_angular;
  double velocity_covariance[36]; // Row-major representation of the 6x6 covariance matrix (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
  // geometry_msgs/Pose bounding_box_center
  SickScanVector3Msg bounding_box_center_position;
  SickScanQuaternionMsg bounding_box_center_orientation;
  // geometry_msgs/Vector3 bounding_box_size
  SickScanVector3Msg bounding_box_size;
  // geometry_msgs/PoseWithCovariance object_box_center
  SickScanVector3Msg object_box_center_position;
  SickScanQuaternionMsg object_box_center_orientation;
  double object_box_center_covariance[36]; // Row-major representation of the 6x6 covariance matrix (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
  // geometry_msgs/Vector3 object_box_size
  SickScanVector3Msg object_box_size;
  // geometry_msgs/Point[] contour_points
  SickScanPointArray contour_points;
} SickScanRadarObject;

typedef struct SickScanRadarObjectArrayType // Array of SickScanRadarObject
{
  uint64_t capacity; // Number of allocated elements, i.e. max. number of elements in buffer, allocated buffer size is capacity * sizeof(SickScanRadarObject)
  uint64_t size;     // Number of currently used elements in the buffer
  SickScanRadarObject* buffer;  // Memory, data in plain order and system endianess (buffer == 0, if size == 0 && capacity == 0, otherwise allocated memory), allocation/deallocation always managed by the caller.
} SickScanRadarObjectArray;

typedef struct SickScanRadarScanType // equivalent to RadarScan.msg
{
  SickScanHeader header;                 // message timestamp
  SickScanRadarPreHeader radarpreheader; // RadarPreHeader.msg
  SickScanPointCloudMsg targets;         // sensor_msgs/PointCloud2
  SickScanRadarObjectArray objects;      // Array of RadarObject.msg
  char topic[256]; // ros topic this messages is published
} SickScanRadarScan;

typedef SickScanRadarObject SickScanLdmrsObject; // equivalent to SickLdmrsObject.msg

typedef struct SickScanLdmrsObjectBufferType // Array of SickScanLdmrsObject
{
  uint64_t capacity; // Number of allocated elements, i.e. max. number of elements in buffer, allocated buffer size is capacity * sizeof(SickScanLdmrsObject)
  uint64_t size;     // Number of currently used elements in the buffer
  SickScanLdmrsObject* buffer;  // Memory, data in plain order and system endianess (buffer == 0, if size == 0 && capacity == 0, otherwise allocated memory), allocation/deallocation always managed by the caller.
} SickScanLdmrsObjectBuffer;

typedef struct SickScanLdmrsObjectArrayType // equivalent to SickLdmrsObjectArray.msg
{
  SickScanHeader header;                 // message timestamp
  SickScanLdmrsObjectBuffer objects;     // Array of SickScanLdmrsObjects
} SickScanLdmrsObjectArray;

typedef struct SickScanColorRGBAType // equivalent to std_msgs::ColorRGBA
{
  float r;
  float g;
  float b;
  float a;
} SickScanColorRGBA;

typedef struct SickScanColorRGBAArrayType // Array of SickScanColorRGBA, which can be serialized and imported in C, C++ or python
{
  uint64_t capacity; // Number of allocated elements, i.e. max. number of elements in buffer, allocated buffer size is capacity * sizeof(SickScanColorRGBA)
  uint64_t size;     // Number of currently used elements in the buffer
  SickScanColorRGBA* buffer;  // Memory, data in plain order and system endianess (buffer == 0, if size == 0 && capacity == 0, otherwise allocated memory), allocation/deallocation always managed by the caller.
} SickScanColorRGBAArray;

typedef struct SickScanVisualizationMarkerType // equivalent to visualization_msgs::Marker
{
  SickScanHeader header;                       // message timestamp
  char ns[1024];                               // Namespace to place this object in... used in conjunction with id to create a unique name for the object
  int32_t id;                                  // object ID useful in conjunction with the namespace for manipulating and deleting the object later
  int32_t type;                                // Type of object
  int32_t action;                              // 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
  SickScanVector3Msg pose_position;            // Pose of the object (positional part)
  SickScanQuaternionMsg pose_orientation;      // Pose of the object (rotational part)
  SickScanVector3Msg scale;                    // Scale of the object 1,1,1 means default (usually 1 meter square)
  SickScanColorRGBA color;                     // Color [0.0-1.0]
  uint32_t lifetime_sec;                       // How long the object should last before being automatically deleted.  0 means forever (seconds part)
  uint32_t lifetime_nsec;                      // How long the object should last before being automatically deleted.  0 means forever (nanoseconds part)
  uint8_t frame_locked;                        // boolean, If this marker should be frame-locked, i.e. retransformed into its frame every timestep
  SickScanPointArray points;                   // Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
  SickScanColorRGBAArray colors;               // Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...). Number of colors must either be 0 or equal to the number of points. NOTE: alpha is not yet used
  char text[1024];                             // NOTE: only used for text markers
  char mesh_resource[1024];                    // NOTE: only used for MESH_RESOURCE markers
  uint8_t mesh_use_embedded_materials;         // boolean
} SickScanVisualizationMarker;

typedef struct SickScanVisualizationMarkerBufferType // Array of SickScanVisualizationMarkers
{
  uint64_t capacity; // Number of allocated elements, i.e. max. number of elements in buffer, allocated buffer size is capacity * sizeof(SickScanVisualizationMarker)
  uint64_t size;     // Number of currently used elements in the buffer
  SickScanVisualizationMarker* buffer;  // Memory, data in plain order and system endianess (buffer == 0, if size == 0 && capacity == 0, otherwise allocated memory), allocation/deallocation always managed by the caller.
} SickScanVisualizationMarkerBuffer;

typedef struct SickScanVisualizationMarkerMsgType // equivalent to visualization_msgs::MarkerArray
{
  SickScanVisualizationMarkerBuffer markers;      // Array of SickScanVisualizationMarkers
  char topic[256];                                // ros topic this messages is published
} SickScanVisualizationMarkerMsg;

typedef struct SickScanNavReflectorType       // NAV-350 reflector
{
  uint16_t pos_valid;
  float pos_x;                            // reflector x-position in m, if pos_valid > 0
  float pos_y;                            // reflector y-position in m, if pos_valid > 0
  uint16_t cartesian_valid;
  int32_t cartesian_x;                    // cartesian x in mm, if cartesian_valid > 0
  int32_t cartesian_y;                    // cartesian y in mm, if cartesian_valid > 0
  uint16_t polar_valid;
  uint32_t polar_dist;                    // polar dist in mm, if polar_valid > 0
  uint32_t polar_phi;                     // polar phi in mdeg, if polar_valid > 0
  uint16_t opt_valid;
  // Optional reflector data, if opt_valid > 0
  uint16_t opt_local_id;
  uint16_t opt_global_id;
  uint8_t opt_type;
  uint16_t opt_subtype;
  uint16_t opt_quality;
  uint32_t opt_timestamp;                 // lidar timestamp in milliseconds
  uint16_t opt_size;
  uint16_t opt_hitcount;
  uint16_t opt_meanecho;
  uint16_t opt_startindex;
  uint16_t opt_endindex;
  uint32_t opt_timestamp_sec;             // timestamp converted to system time (seconds part, 0 if timestamp not valid)
  uint32_t opt_timestamp_nsec;            // timestamp converted to system time (nanoseconds part, 0 if timestamp not valid)
} SickScanNavReflector;

typedef struct SickScanNavReflectorBufferType // Array of SickScanNavReflectors
{
  uint64_t capacity;                          // Number of allocated elements, i.e. max. number of elements in buffer, allocated buffer size is capacity * sizeof(SickScanNavReflector)
  uint64_t size;                              // Number of currently used elements in the buffer
  SickScanNavReflector* buffer;               // Memory, data in plain order and system endianess (buffer == 0, if size == 0 && capacity == 0, otherwise allocated memory), allocation/deallocation always managed by the caller.
} SickScanNavReflectorBuffer;

typedef struct SickScanNavPoseLandmarkMsgType // NAV-350 pose and landmark message
{
  uint16_t pose_valid;
  // NAV pose, if pose_valid > 0:
  float pose_x;                           // x-position in ros coordinates in m
  float pose_y;                           // y-position in ros coordinates in m
  float pose_yaw;                         // yaw angle in ros coordinates in radians
  uint32_t pose_timestamp_sec;            // timestamp of pose converted to system time (seconds part, 0 if timestamp not valid)
  uint32_t pose_timestamp_nsec;           // timestamp of pose converted to system time (nanoseconds part, 0 if timestamp not valid)
  int32_t pose_nav_x;                     // x-position in lidar coordinates in mm
  int32_t pose_nav_y;                     // y-position in lidar coordinates in mm
  uint32_t pose_nav_phi;                  // orientation in lidar coordinates in 0 ... 360000 mdeg
  uint16_t pose_opt_valid;
  // Optional NAV pose data, if pose_opt_valid > 0:
  uint8_t pose_opt_output_mode;
  uint32_t pose_opt_timestamp;            // lidar timestamp in milliseconds
  int32_t pose_opt_mean_dev;
  uint8_t pose_opt_nav_mode;
  uint32_t pose_opt_info_state ;
  uint8_t pose_opt_quant_used_reflectors;
  // NAV reflectors:
  SickScanNavReflectorBuffer reflectors;      // Array of SickScanNavReflectors
} SickScanNavPoseLandmarkMsg;

typedef struct SickScanNavOdomVelocityMsgType // NAV350 velocity/odometry data, see NAVOdomVelocity.msg
{
  float vel_x;        // x-component of velocity in the coordinate system defined by coordbase (i.e. in lidar coordinate for coordbase=0) in m/s, -32.0 ... +32.0 m/s
  float vel_y;        // y-component of velocity in the coordinate system defined by coordbase (i.e. in lidar coordinate for coordbase=0) in m/s, -32.0 ... +32.0 m/s
  float omega;        // angular velocity of the NAV350 in radians/s, -2*PI ... +2*PI rad/s
  uint32_t timestamp; // timestamp of the Velocity vector related to the NAV350 clock
  uint8_t coordbase;  // coordinate system of the velocity vector (local or global), 0 = local coordinate system of the NAV350, 1 = absolute coordinate system
} SickScanNavOdomVelocityMsg;

typedef struct SickScanOdomVelocityMsgType // Velocity/odometry data in system coordinates and time
{
  float vel_x;             // x-component of velocity in ros coordinates in m/s
  float vel_y;             // y-component of velocity in ros coordinates in m/s
  float omega;             // angular velocity in radians/s
  uint32_t timestamp_sec;  // seconds part of system timestamp of the odometry data
  uint32_t timestamp_nsec; // nanoseconds part of system timestamp of the odometry data
} SickScanOdomVelocityMsg;

typedef struct SickScanLogMsgType // general log message
{
  int32_t log_level; // log_level defined in ros::console::levels: Info=1, Warn=2, Error=3, Fatal=4
  char* log_message; // log message
} SickScanLogMsg;

typedef struct SickScanDiagnosticMsgType // general diagnostic message
{
  int32_t status_code; // status_code defined in SICK_DIAGNOSTIC_STATUS: OK=0 (normal operation), WARN=1 (warning), ERROR=2 (error, should not occure), INIT=3 (initialization after startup or reconnection), EXIT=4 (sick_scan_xd exiting)
  char* status_message; // diagnostic message
} SickScanDiagnosticMsg;

/*
*  Callback declarations
*/

typedef void* SickScanApiHandle;
typedef void(SICK_SCAN_XD_API_CALLING_CONVENTION * SickScanPointCloudMsgCallback)(SickScanApiHandle apiHandle, const SickScanPointCloudMsg* msg);
typedef void(SICK_SCAN_XD_API_CALLING_CONVENTION * SickScanImuMsgCallback)(SickScanApiHandle apiHandle, const SickScanImuMsg* msg);
typedef void(SICK_SCAN_XD_API_CALLING_CONVENTION * SickScanLFErecMsgCallback)(SickScanApiHandle apiHandle, const SickScanLFErecMsg* msg);
typedef void(SICK_SCAN_XD_API_CALLING_CONVENTION * SickScanLIDoutputstateMsgCallback)(SickScanApiHandle apiHandle, const SickScanLIDoutputstateMsg* msg);
typedef void(SICK_SCAN_XD_API_CALLING_CONVENTION * SickScanRadarScanCallback)(SickScanApiHandle apiHandle, const SickScanRadarScan* msg);
typedef void(SICK_SCAN_XD_API_CALLING_CONVENTION * SickScanLdmrsObjectArrayCallback)(SickScanApiHandle apiHandle, const SickScanLdmrsObjectArray* msg);
typedef void(SICK_SCAN_XD_API_CALLING_CONVENTION * SickScanVisualizationMarkerCallback)(SickScanApiHandle apiHandle, const SickScanVisualizationMarkerMsg* msg);
typedef void(SICK_SCAN_XD_API_CALLING_CONVENTION * SickScanNavPoseLandmarkCallback)(SickScanApiHandle apiHandle, const SickScanNavPoseLandmarkMsg* msg);
typedef void(SICK_SCAN_XD_API_CALLING_CONVENTION * SickScanLogMsgCallback)(SickScanApiHandle apiHandle, const SickScanLogMsg* msg);
typedef void(SICK_SCAN_XD_API_CALLING_CONVENTION * SickScanDiagnosticMsgCallback)(SickScanApiHandle apiHandle, const SickScanDiagnosticMsg* msg);

/*
*  Functions to initialize and close the API and a lidar
*/

// Load sick_scan_xd api library (dll or so file)
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiLoadLibrary(const char* library_filepath);

// Unload sick_scan_xd api library
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiUnloadLibrary();

/*
*  Create an instance of sick_scan_xd api.
*  Optional commandline arguments argc, argv identical to sick_generic_caller.
*  Call SickScanApiInitByLaunchfile or SickScanApiInitByCli to process a lidar.
*/
SICK_SCAN_API_DECLSPEC_EXPORT SickScanApiHandle SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiCreate(int argc, char** argv);

// Release and free all resources of a handle; the handle is invalid after SickScanApiRelease
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRelease(SickScanApiHandle apiHandle);

// Initializes a lidar by launchfile and starts message receiving and processing
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiInitByLaunchfile(SickScanApiHandle apiHandle, const char* launchfile_args);

// Initializes a lidar by commandline arguments and starts message receiving and processing
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiInitByCli(SickScanApiHandle apiHandle, int argc, char** argv);

// Stops message receiving and processing and closes a lidar
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiClose(SickScanApiHandle apiHandle);

/*
*  Registration / deregistration of message callbacks
*/

// Register / deregister a callback for cartesian PointCloud messages, pointcloud in cartesian coordinates with fields x, y, z, intensity
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback);

// Register / deregister a callback for polar PointCloud messages, pointcloud in polar coordinates with fields range, azimuth, elevation, intensity
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback);

// Register / deregister a callback for Imu messages
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterImuMsg(SickScanApiHandle apiHandle, SickScanImuMsgCallback callback);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterImuMsg(SickScanApiHandle apiHandle, SickScanImuMsgCallback callback);

// Register / deregister a callback for SickScanLFErecMsg messages
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsgCallback callback);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsgCallback callback);

// Register / deregister a callback for SickScanLIDoutputstateMsg messages
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsgCallback callback);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsgCallback callback);

// Register / deregister a callback for SickScanRadarScan messages
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScanCallback callback);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScanCallback callback);

// Register / deregister a callback for SickScanLdmrsObjectArray messages
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArrayCallback callback);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArrayCallback callback);

// Register / deregister a callback for VisualizationMarker messages
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterVisualizationMarkerMsg(SickScanApiHandle apiHandle, SickScanVisualizationMarkerCallback callback);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterVisualizationMarkerMsg(SickScanApiHandle apiHandle, SickScanVisualizationMarkerCallback callback);

// Register / deregister a callback for SickScanNavPoseLandmark messages
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterNavPoseLandmarkMsg(SickScanApiHandle apiHandle, SickScanNavPoseLandmarkCallback callback);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterNavPoseLandmarkMsg(SickScanApiHandle apiHandle, SickScanNavPoseLandmarkCallback callback);

/*
*  Functions for diagnostic and logging
*/

// Register / deregister a callback for diagnostic messages (notification in case of changed status, e.g. after errors)
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterDiagnosticMsg(SickScanApiHandle apiHandle, SickScanDiagnosticMsgCallback callback);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterDiagnosticMsg(SickScanApiHandle apiHandle, SickScanDiagnosticMsgCallback callback);

// Register / deregister a callback for log messages (all informational and error messages)
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterLogMsg(SickScanApiHandle apiHandle, SickScanLogMsgCallback callback);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterLogMsg(SickScanApiHandle apiHandle, SickScanLogMsgCallback callback);

// Query current status and status message
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiGetStatus(SickScanApiHandle apiHandle, int32_t* status_code, char* message_buffer, int32_t message_buffer_size);

// Sends a SOPAS command like "sRN SCdevicestate" or "sRN ContaminationResult" and returns the lidar response
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiSendSOPAS(SickScanApiHandle apiHandle, const char* sopas_command, char* sopas_response_buffer, int32_t response_buffer_size);

// Set verbose level 0=DEBUG, 1=INFO, 2=WARN, 3=ERROR, 4=FATAL or 5=QUIET (equivalent to ros::console::levels),
// i.e. print messages on console above the given verbose level.
// Default verbose level is 1 (INFO), i.e. print informational, warnings and error messages.
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiSetVerboseLevel(SickScanApiHandle apiHandle, int32_t verbose_level);

// Returns the current verbose level 0=DEBUG, 1=INFO, 2=WARN, 3=ERROR, 4=FATAL or 5=QUIET. Default verbose level is 1 (INFO)
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiGetVerboseLevel(SickScanApiHandle apiHandle);

/*
*  Polling functions
*/

// Wait for and return the next cartesian resp. polar PointCloud message. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiWaitNextCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg, double timeout_sec);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiWaitNextPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg, double timeout_sec);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiFreePointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg);

// Wait for and return the next Imu message. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiWaitNextImuMsg(SickScanApiHandle apiHandle, SickScanImuMsg* msg, double timeout_sec);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiFreeImuMsg(SickScanApiHandle apiHandle, SickScanImuMsg* msg);

// Wait for and return the next LFErec message. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiWaitNextLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsg* msg, double timeout_sec);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiFreeLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsg* msg);

// Wait for and return the next LIDoutputstate message. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiWaitNextLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsg* msg, double timeout_sec);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiFreeLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsg* msg);

// Wait for and return the next RadarScan message. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiWaitNextRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScan* msg, double timeout_sec);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiFreeRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScan* msg);

// Wait for and return the next LdmrsObjectArray message. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiWaitNextLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArray* msg, double timeout_sec);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiFreeLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArray* msg);

// Wait for and return the next VisualizationMarker message. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiWaitNextVisualizationMarkerMsg(SickScanApiHandle apiHandle, SickScanVisualizationMarkerMsg* msg, double timeout_sec);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiFreeVisualizationMarkerMsg(SickScanApiHandle apiHandle, SickScanVisualizationMarkerMsg* msg);

// Wait for and return the next SickScanNavPoseLandmark message. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiWaitNextNavPoseLandmarkMsg(SickScanApiHandle apiHandle, SickScanNavPoseLandmarkMsg* msg, double timeout_sec);
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiFreeNavPoseLandmarkMsg(SickScanApiHandle apiHandle, SickScanNavPoseLandmarkMsg* msg);

// Send odometry data to NAV350
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiNavOdomVelocityMsg(SickScanApiHandle apiHandle, SickScanNavOdomVelocityMsg* msg); // odometry data in nav coordinates
SICK_SCAN_API_DECLSPEC_EXPORT int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiOdomVelocityMsg(SickScanApiHandle apiHandle, SickScanOdomVelocityMsg* msg); // odometry data in system coordinates

/*
*  Error codes, return values of SickScanApi-functions
*/
enum SickScanApiErrorCodes
{
    SICK_SCAN_API_SUCCESS = 0,          // function executed successfully
    SICK_SCAN_API_ERROR = 1,            // general (unspecified) error
    SICK_SCAN_API_NOT_LOADED = 2,       // sick_scan_xd library not loaded
    SICK_SCAN_API_NOT_INITIALIZED = 3,  // API not initialized
    SICK_SCAN_API_NOT_IMPLEMENTED = 4,  // function not implemented in sick_scan_xd library
    SICK_SCAN_API_TIMEOUT = 5           // timeout during wait for response
};

#ifdef  __cplusplus
} // extern "C"
#endif

#endif // __SICK_SCAN_API_H_INCLUDED
