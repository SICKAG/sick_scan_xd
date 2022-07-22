# Generic API for sick_scan_xd

## Overview

A generic API for sick_scan_xd has the following goals:
* Easy integration of sick_scan_xd into customer systems with and without ROS
* Integrate SICK lidars with one API, independent of lidar types or underlying operating system
* Provide the same sick_scan_xd functionality on systems without ROS
* In particular: make the sick_scan_xd functionality available on non-ROS-systems without need to customize sources or configuration files.

The generic sick_scan_xd API provides an interface to all lidars supported by sick_scan_xd. This API can be used in C, C++, Python, or any other language with support of C-bindings.

The generic sick_scan_xd API ships with the API-header, the library (binary or sources) and usage examples for C, C++ and Python. The following component diagram shows the relationship between API, library, lidar and a customized application:

![apiComponentsDiagram1.png](apiComponentsDiagram1.png)

## Build and test shared library

The shared library, which implements the C-API, is built native on Linux or Windows (i.e. without ROS). Follow the instructions on [Build on Linux generic without ROS](../../README.md/#build-on-linux-generic-without-ros) for Linux. To avoid potential relocation issues, building without LDMRS and Multiscan support is currently recommended (but will be supported in the next release). Run the following commands to build the shared library `libsick_scan_shared_lib.so` on Linux:

```
git clone <repository>/sick_scan_xd.git
mkdir -p ./build
pushd ./build
cmake -DROS_VERSION=0 -DLDMRS=0 -DSCANSEGMENT_XD=0 -G "Unix Makefiles" ../sick_scan_xd
make -j4
ls -al libsick_scan_shared_lib.so sick_scan_xd_api_test sick_generic_caller
popd
```

After successfull build, the shared library `libsick_scan_shared_lib.so` and a tiny test executable `sick_scan_xd_api_test` are created. Run `sick_scan_xd_api_test <launchfile> hostname:=<ip-address>` to test the API against a lidar, e.g.:

```
cd ./build
export LD_LIBRARY_PATH=.:./build:$LD_LIBRARY_PATH
./sick_scan_xd_api_test ../sick_scan_xd/launch/sick_lms_1xx.launch hostname:=192.168.0.111
```

The executable binary `sick_scan_xd_api_test` will just load library `libsick_scan_shared_lib.so`, start the lidar and print a message when receiving lidar messages, e.g. `sick_scan_xd_api_test: pointcloud callback`. Replace `sick_lms_1xx.launch` in the example by the launchfile corresponding to your type of lidar.

To load the library, the build folder has to be included in `LD_LIBRARY_PATH`. Set environment variable `LD_LIBRARY_PATH` to your build folder, e.g.
```
export LD_LIBRARY_PATH=.:./build:$LD_LIBRARY_PATH
```

### Python example

Python example [sick_scan_xd_api_test.py](../../test/python/sick_scan_xd_api/sick_scan_xd_api_test.py) is handy to test the library in python. Like its C++ counterpart, it just loads library `libsick_scan_shared_lib.so`, starts a lidar and receives the lidar pointcloud via API. On ROS-1, the pointcloud is converted to ROS and published on topic "/sick_scan_xd_api_test/api_cloud".

Run `python3 sick_scan_xd_api_test.py <launchfile> hostname:=<ip-address>` to test the API against a lidar, e.g.:

```
cd ./build
export LD_LIBRARY_PATH=.:./build:$LD_LIBRARY_PATH
source /opt/ros/noetic/setup.bash
python3 ../sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py ../sick_scan_xd/launch/sick_lms_1xx.launch hostname:=192.168.0.111
```

### Simulation and unittest

sick_scan_xd provides a tiny server for offline tests which simulates a basic lidar. It just accepts TCP connections, responds to sopas requests with predefined responses and sends lidar data from file. See [Simulation](../../README.md/#simulation) for further details. Note that the simulation does not emulate or replace a lidar, it just supports basic unittests.

Open a new terminal and run the following steps to test the api against a TiM7xx simulation using the python example mentioned above:

1. Build library `libsick_scan_shared_lib.so` incl. emulator with option `-DCMAKE_ENABLE_EMULATOR=1`:
   ```
   mkdir -p ./src/build
   pushd ./src/build
   rm -rf ./*
   cmake -DROS_VERSION=0 -DLDMRS=0 -DSCANSEGMENT_XD=0 -DCMAKE_ENABLE_EMULATOR=1 -G "Unix Makefiles" ../sick_scan_xd
   make -j4
   ls -al libsick_scan_shared_lib.so sick_scan_xd_api_test sick_generic_caller sick_scan_emulator
   popd
   ```

2. Build sick_scan_xd for ROS-1 on Linux, see [Build on Linux ROS1](../../README.md/#build-on-linux-ros1)

3. Start the TiM7xx simulator:
   ```
   cp -f ./src/sick_scan_xd/test/emulator/scandata/sopas_et_field_test_1_2_both_010.pcapng.json /tmp/lmd_scandata.pcapng.json
   ./src/build/sick_scan_emulator ./src/sick_scan_xd/test/emulator/launch/emulator_01_default.launch &
   sleep 1
   ```

4. Run sick_scan_xd_api_test.py against the TiM7xx simulator on localhost:
   ```
   python3 ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_test.py ./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False
   ```

5. Start rviz and visualize the pointcloud on topic "/sick_scan_xd_api_test/api_cloud".

## C-API

The header file [sick_scan_api.h](../../include/sick_scan_xd_api/sick_scan_api.h) defines the C-interface. It defines all datatypes, messages and functions of the generic sick_scan_xd API. To allow equal operations on all systems, the definition of datatypes and messages is as close as possible to their equivalents currently used on ROS.

```
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
  char ns[1024];                               // Namespace to place this object in... used in conjunction with id to create a unique name for the object
  int32_t id;                                  // object ID useful in conjunction with the namespace for manipulating and deleting the object later
  int32_t type;                                // Type of object
  int32_t action;                              // 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
  SickScanVector3Msg pose_position;            // Pose of the object (positional part)
  SickScanQuaternionMsg pose_orientation;      // Pose of the object (rotational part)
  SickScanVector3Msg scale;                    // Scale of the object 1,1,1 means default (usually 1 meter square)
  SickScanColorRGBA color;                     // Color [0.0-1.0]
  double lifetime;                             // How long the object should last before being automatically deleted.  0 means forever (duration in seconds)
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
  SickScanHeader header;                          // message timestamp
  SickScanVisualizationMarkerBuffer objects;      // Array of SickScanVisualizationMarkers
} SickScanVisualizationMarkerMsg;

/*
*  Callback declarations
*/

typedef void* SickScanApiHandle;
typedef void(* SickScanPointCloudMsgCallback)(SickScanApiHandle apiHandle, const SickScanPointCloudMsg* msg);
typedef void(* SickScanImuMsgCallback)(SickScanApiHandle apiHandle, const SickScanImuMsg* msg);
typedef void(* SickScanLFErecMsgCallback)(SickScanApiHandle apiHandle, const SickScanLFErecMsg* msg);
typedef void(* SickScanLIDoutputstateMsgCallback)(SickScanApiHandle apiHandle, const SickScanLIDoutputstateMsg* msg);
typedef void(* SickScanRadarScanCallback)(SickScanApiHandle apiHandle, const SickScanRadarScan* msg);
typedef void(* SickScanLdmrsObjectArrayCallback)(SickScanApiHandle apiHandle, const SickScanLdmrsObjectArray* msg);
typedef void(* SickScanVisualizationMarkerCallback)(SickScanApiHandle apiHandle, const SickScanVisualizationMarkerMsg* msg);

/*
*  Functions to initialize and close the API and a lidar
*/

// Load sick_scan_xd api library (dll or so file)
int32_t SickScanApiLoadLibrary(const char* library_filepath);

// Unload sick_scan_xd api library
int32_t SickScanApiUnloadLibrary();

/*
*  Create an instance of sick_scan_xd api.
*  Optional commandline arguments argc, argv identical to sick_generic_caller.
*  Call SickScanApiInitByLaunchfile or SickScanApiInitByCli to process a lidar.
*/
SickScanApiHandle SickScanApiCreate(int argc, char** argv);

// Release and free all resources of a handle; the handle is invalid after SickScanApiRelease
int32_t SickScanApiRelease(SickScanApiHandle apiHandle);

// Initializes a lidar by launchfile and starts message receiving and processing
int32_t SickScanApiInitByLaunchfile(SickScanApiHandle apiHandle, const char* launchfile_args);

// Initializes a lidar by commandline arguments and starts message receiving and processing
int32_t SickScanApiInitByCli(SickScanApiHandle apiHandle, int argc, char** argv);

// Stops message receiving and processing and closes a lidar
int32_t SickScanApiClose(SickScanApiHandle apiHandle);

/*
*  Registration / deregistration of message callbacks
*/

// Register / deregister a callback for cartesian PointCloud messages, pointcloud in cartesian coordinates with fields x, y, z, intensity
int32_t SickScanApiRegisterCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback);
int32_t SickScanApiDeregisterCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback);

// Register / deregister a callback for polar PointCloud messages, pointcloud in polar coordinates with fields range, azimuth, elevation, intensity
int32_t SickScanApiRegisterPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback);
int32_t SickScanApiDeregisterPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback);

// Register / deregister a callback for Imu messages
int32_t SickScanApiRegisterImuMsg(SickScanApiHandle apiHandle, SickScanImuMsgCallback callback);
int32_t SickScanApiDeregisterImuMsg(SickScanApiHandle apiHandle, SickScanImuMsgCallback callback);

// Register / deregister a callback for SickScanLFErecMsg messages
int32_t SickScanApiRegisterLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsgCallback callback);
int32_t SickScanApiDeregisterLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsgCallback callback);

// Register / deregister a callback for SickScanLIDoutputstateMsg messages
int32_t SickScanApiRegisterLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsgCallback callback);
int32_t SickScanApiDeregisterLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsgCallback callback);

// Register / deregister a callback for SickScanRadarScan messages
int32_t SickScanApiRegisterRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScanCallback callback);
int32_t SickScanApiDeregisterRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScanCallback callback);

// Register / deregister a callback for SickScanLdmrsObjectArray messages
int32_t SickScanApiRegisterLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArrayCallback callback);
int32_t SickScanApiDeregisterLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArrayCallback callback);

// Register / deregister a callback for VisualizationMarker messages
int32_t SickScanApiRegisterVisualizationMarkerMsg(SickScanApiHandle apiHandle, SickScanVisualizationMarkerCallback callback);
int32_t SickScanApiDeregisterVisualizationMarkerMsg(SickScanApiHandle apiHandle, SickScanVisualizationMarkerCallback callback);

/*
*  Polling functions
*/

// Wait for and return the next cartesian resp. polar PointCloud messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg, double timeout_sec);
int32_t SickScanApiWaitNextPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg, double timeout_sec);
int32_t SickScanApiFreePointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg);

// Wait for and return the next Imu messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextImuMsg(SickScanApiHandle apiHandle, SickScanImuMsg* msg, double timeout_sec);
int32_t SickScanApiFreeImuMsg(SickScanApiHandle apiHandle, SickScanImuMsg* msg);

// Wait for and return the next LFErec messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsg* msg, double timeout_sec);
int32_t SickScanApiFreeLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsg* msg);

// Wait for and return the next LIDoutputstate messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsg* msg, double timeout_sec);
int32_t SickScanApiFreeLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsg* msg);

// Wait for and return the next RadarScan messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScan* msg, double timeout_sec);
int32_t SickScanApiFreeRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScan* msg);

// Wait for and return the next LdmrsObjectArray messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArray* msg, double timeout_sec);
int32_t SickScanApiFreeLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArray* msg);

// Wait for and return the next VisualizationMarker message. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextVisualizationMarkerMsg(SickScanApiHandle apiHandle, SickScanVisualizationMarkerMsg* msg, double timeout_sec);
int32_t SickScanApiFreeVisualizationMarkersg(SickScanApiHandle apiHandle, SickScanVisualizationMarkerMsg* msg);

/*
*  Error codes, return values of SickScanApi-functions
*/
enum SickScanApiErrorCodes
{
    SICK_SCAN_API_SUCCESS = 0,          // function executed successfully
    SICK_SCAN_API_ERROR = 1,            // general (unspecified) error
    SICK_SCAN_API_NOT_LOADED = 2,       // sick_scan_xd library not loaded
    SICK_SCAN_API_NOT_INITIALIZED = 3,  // API not initialized
    SICK_SCAN_API_NOT_IMPLEMENTED = 4   // function not implemented in sick_scan_xd library
};
```

Python file [sick_scan_api.py](../../test/python/sick_scan_xd_api/sick_scan_api.py) defines the same interface in python:
```
#
# Message definitions
#

class SickScanHeader(ctypes.Structure):              # sick_scan_api: struct SickScanHeader, equivalent to ros::std_msgs::Header
    _fields_ = [
        ("seq", ctypes.c_uint32),                    # sequence ID: consecutively increasing ID
        ("timestamp_sec", ctypes.c_uint32),          # seconds part of message timestamps: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
        ("timestamp_nsec", ctypes.c_uint32),         # seconds part of message timestamps: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
        ("frame_id", ctypes.c_char * 256)            # Frame this data is associated with
    ]

class SickScanUint8Array(ctypes.Structure):          # sick_scan_api: struct SickScanUint8Array
    _fields_ = [
        ("capacity", ctypes.c_uint64),               # Number of allocated elements, i.e. max. number of elements in buffer, allocated buffer size is capacity * sizeof(SickScanPointFieldMsg)
        ("size", ctypes.c_uint64),                   # Number of currently used elements in the buffer
        ("buffer", ctypes.POINTER(ctypes.c_uint8))   # Memory, data in plain order and system endianess (buffer == 0, if size == 0 && capacity == 0, otherwise allocated memory), allocation/deallocation always managed by the caller.
    ]

class SickScanNativeDataType(Enum): # This message holds the description of one point entry in the PointCloud2 message format, equivalent to type enum im ros::sensor_msgs::PointField
    SICK_SCAN_POINTFIELD_DATATYPE_INT8    = 1,
    SICK_SCAN_POINTFIELD_DATATYPE_UINT8   = 2,
    SICK_SCAN_POINTFIELD_DATATYPE_INT16   = 3,
    SICK_SCAN_POINTFIELD_DATATYPE_UINT16  = 4,
    SICK_SCAN_POINTFIELD_DATATYPE_INT32   = 5,
    SICK_SCAN_POINTFIELD_DATATYPE_UINT32  = 6,
    SICK_SCAN_POINTFIELD_DATATYPE_FLOAT32 = 7,
    SICK_SCAN_POINTFIELD_DATATYPE_FLOAT64 = 8

class SickScanPointFieldMsg(ctypes.Structure):       # sick_scan_api: struct SickScanPointFieldMsg, equivalent to ros::sensor_msgs::PointField
    # SickScanPointFieldArray is an array of SickScanPointFieldMsg, which defines the structure of the binary data of a SickScanPointCloudMsg.
    # SickScanPointFieldMsg for pointclouds in cartesian coordinates with fields (x, y, z, intensity):
    #     [ SickScanPointFieldMsg(name="x", offset=0, datatype=FLOAT32, count=1), 
    #       SickScanPointFieldMsg(name="y", offset=4, datatype=FLOAT32, count=1),
    #       SickScanPointFieldMsg(name="z", offset=8, datatype=FLOAT32, count=1),
    #       SickScanPointFieldMsg(name="intensity", offset=12, datatype=FLOAT32, count=1) ]
    # SickScanPointFieldMsg for pointclouds in polar coordinates with fields (range, azimuth, elevation, intensity):
    #     [ SickScanPointFieldMsg(name="range", offset=0, datatype=FLOAT32, count=1), 
    #       SickScanPointFieldMsg(name="azimuth", offset=4, datatype=FLOAT32, count=1),
    #       SickScanPointFieldMsg(name="elevation", offset=8, datatype=FLOAT32, count=1),
    #       SickScanPointFieldMsg(name="intensity", offset=12, datatype=FLOAT32, count=1) ]
    _fields_ = [
        ("name", ctypes.c_char * 256),               # Name of field (max. length 256 characters)
        ("offset", ctypes.c_uint32),                 # Offset from start of point struct
        ("datatype", ctypes.c_uint8),                # Datatype enumeration, see SickScanNativeDataType, equivalent to type enum im ros::sensor_msgs::PointField
        ("count", ctypes.c_uint32)                   # How many elements in the field
   ]

class SickScanPointFieldArray(ctypes.Structure):     # sick_scan_api: struct SickScanPointFieldArray, Array of SickScanPointFieldMsg, which can be serialized and imported in C, C++ or python
    _fields_ = [
        ("capacity", ctypes.c_uint64),                     # Number of allocated elements, i.e. max. number of elements in buffer, allocated buffer size is capacity * sizeof(SickScanPointFieldMsg)
        ("size", ctypes.c_uint64),                         # Number of currently used elements in the buffer
        ("buffer", ctypes.POINTER(SickScanPointFieldMsg))  # Memory, data in plain order and system endianess (buffer == 0, if size == 0 && capacity == 0, otherwise allocated memory), allocation/deallocation always managed by the caller.
    ]

class SickScanPointCloudMsg(ctypes.Structure):       # sick_scan_api: struct SickScanPointCloudMsg, equivalent to ros::std_msgs::PointCloud2
    _fields_ = [
        ("header", SickScanHeader),                  # message timestamp
        ("height", ctypes.c_uint32),                 # 2D structure of the point cloud. If the cloud is unordered, height is 1
        ("width", ctypes.c_uint32),                  # and width is the length of the point cloud.
        ("fields", SickScanPointFieldArray),         # Describes the channels and their layout in the binary data blob.
        ("is_bigendian", ctypes.c_uint8),            # Is this data bigendian?
        ("point_step", ctypes.c_uint32),             # Length of a point in bytes
        ("row_step", ctypes.c_uint32),               # Length of a row in bytes
        ("data", SickScanUint8Array),                # Actual point data, size is (row_step*height)
        ("is_dense", ctypes.c_uint8),                # True if there are no invalid points
        ("num_echos", ctypes.c_int32),               # number of echos
        ("segment_idx", ctypes.c_int32)              # segment index (or -1 if pointcloud contains data from multiple segments)
    ]

class SickScanVector3Msg(ctypes.Structure):          # sick_scan_api: struct SickScanVector3Msg, equivalent to geometry_msgs/Vector3
    _fields_ = [
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
        ("z", ctypes.c_double)
    ]

class SickScanQuaternionMsg(ctypes.Structure):       # sick_scan_api: struct SickScanQuaternionMsg, equivalent to geometry_msgs/Quaternion
    _fields_ = [
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
        ("z", ctypes.c_double),
        ("w", ctypes.c_double)
    ]

class SickScanPointArray(ctypes.Structure):          # sick_scan_api: struct SickScanPointArray, Array of SickScanVector3Msg, which can be serialized and imported in C, C++ or python
    _fields_ = [
        ("capacity", ctypes.c_uint64), # Number of allocated elements, i.e. max. number of elements in buffer, allocated buffer size is capacity * sizeof(SickScanVector3Msg)
        ("size", ctypes.c_uint64),     # Number of currently used elements in the buffer
        ("buffer", ctypes.POINTER(SickScanVector3Msg))  # Memory, data in plain order and system endianess (buffer == 0, if size == 0 && capacity == 0, otherwise allocated memory), allocation/deallocation always managed by the caller.
    ]

class SickScanImuMsg(ctypes.Structure):              # sick_scan_api: struct SickScanImuMsg, equivalent to ros sensor_msgs::Imu
    _fields_ = [
        ("header", SickScanHeader),                              # message timestamp
        ("orientation", SickScanQuaternionMsg),
        ("orientation_covariance", ctypes.c_double * 9),         # Row major about x, y, z axes
        ("angular_velocity", SickScanVector3Msg),
        ("angular_velocity_covariance", ctypes.c_double * 9),    # Row major about x, y, z axes
        ("linear_acceleration", SickScanVector3Msg),
        ("linear_acceleration_covariance", ctypes.c_double * 9)  # Row major x, y z
    ]

class SickScanLFErecFieldMsg(ctypes.Structure):      # sick_scan_api: struct SickScanLFErecFieldMsg, equivalent to LFErecFieldMsg.msg
    _fields_ = [
        ("version_number", ctypes.c_uint16),
        ("field_index", ctypes.c_uint8),
        ("sys_count", ctypes.c_uint32),
        ("dist_scale_factor", ctypes.c_float),
        ("dist_scale_offset", ctypes.c_float),
        ("angle_scale_factor", ctypes.c_uint32),
        ("angle_scale_offset", ctypes.c_int32),
        ("field_result_mrs", ctypes.c_uint8),        # field result is 0 (invalid/incorrect), 1 (free/clear) or 2 (infringed)
        ("time_state", ctypes.c_uint16),             # No time data: 0, Time data: 1
        ("year", ctypes.c_uint16),                   # f.e. 2021
        ("month", ctypes.c_uint8),                   # 1 ... 12
        ("day", ctypes.c_uint8),                     # 1 ... 31
        ("hour", ctypes.c_uint8),                    # 0 ... 23
        ("minute", ctypes.c_uint8),                  # 0 ... 59
        ("second", ctypes.c_uint8),                  # 0 ... 59
        ("microsecond", ctypes.c_uint32)             # 0 ... 999999
    ]

class SickScanLFErecMsg(ctypes.Structure):           # sick_scan_api: struct SickScanLFErecMsg, equivalent to LFErecMsg.msg
    _fields_ = [
        ("header", SickScanHeader),                  # message timestamp
        ("fields_number", ctypes.c_uint16),          # number of valid fields
        ("fields", SickScanLFErecFieldMsg * 3)       # max. 3 valid fields
    ]

class SickScanLIDoutputstateMsg(ctypes.Structure):   # sick_scan_api: struct SickScanLIDoutputstateMsg, equivalent to LIDoutputstateMsg.msg
    _fields_ = [
        ("header", SickScanHeader),                  # message timestamp
        ("version_number", ctypes.c_uint16),         # Status code version number
        ("system_counter", ctypes.c_uint32),         # Status code system counter (time in microsec since power up, max. 71 min then starting from 0 again)
        ("output_state", ctypes.c_uint8 * 8),        # array of max. 8 output states, each state with value 0 (not active), 1 (active) or 2 (not used)
        ("output_count", ctypes.c_uint32 * 8),       # array of max. 8 output counter
        ("time_state", ctypes.c_uint16),             # No time data: 0, Time data: 1 (sensortime from the last change of min. one of the outputs)
        ("year", ctypes.c_uint16),                   # f.e. 2021
        ("month", ctypes.c_uint8),                   # 1 ... 12
        ("day", ctypes.c_uint8),                     # 1 ... 31
        ("hour", ctypes.c_uint8),                    # 0 ... 23
        ("minute", ctypes.c_uint8),                  # 0 ... 59
        ("second", ctypes.c_uint8),                  # 0 ... 59
        ("microsecond", ctypes.c_uint32)             # 0 ... 999999
    ]

class SickScanRadarPreHeader(ctypes.Structure):      # sick_scan_api: struct SickScanRadarPreHeader, equivalent to RadarPreHeader.msg
    _fields_ = [
        ("uiversionno", ctypes.c_uint16),            # version number
        # sick_scan/RadarPreHeaderDeviceBlock:
        ("uiident", ctypes.c_uint32),                # Logical number of the device"
        ("udiserialno", ctypes.c_uint32),            # Serial number of the device
        ("bdeviceerror", ctypes.c_uint8),            # State of the device
        ("bcontaminationwarning", ctypes.c_uint8),   # Contamination Warning
        ("bcontaminationerror", ctypes.c_uint8),     # Contamination Error
        # sick_scan/RadarPreHeaderStatusBlock:
        ("uitelegramcount", ctypes.c_uint32),        # telegram number
        ("uicyclecount", ctypes.c_uint32),           # "scan number"
        ("udisystemcountscan", ctypes.c_uint32),     # system time since power on of scan gen. [us]
        ("udisystemcounttransmit", ctypes.c_uint32), # system time since power on of scan transmission [us]
        ("uiinputs", ctypes.c_uint16),               # state of digital inputs
        ("uioutputs", ctypes.c_uint16),              # state of digital outputs
        # sick_scan/RadarPreHeaderMeasurementParam1Block:
        ("uicycleduration", ctypes.c_uint32),        # Time in microseconds to detect this items
        ("uinoiselevel", ctypes.c_uint32),           # [1/100dB]
        # sick_scan/RadarPreHeaderEncoderBlock[]:
        ("numencoder", ctypes.c_uint16),             # number of valid encoders (0)
        ("udiencoderpos", ctypes.c_uint32 * 3),      # array of max. 3 encoder position [inc]
        ("iencoderspeed", ctypes.c_uint16 * 3)       # array of max. 3 encoder speed [inc/mm]
    ]

class SickScanRadarObject(ctypes.Structure):         # sick_scan_api: struct SickScanRadarObject, equivalent to RadarObject.msg
    _fields_ = [
        ("id", ctypes.c_int32),
        ("tracking_time_sec", ctypes.c_uint32),      # seconds part of tracking_time (since when the object is tracked): seconds (stamp_secs) since epoch
        ("tracking_time_nsec", ctypes.c_uint32),     # nanoseconds part of tracking_time (since when the object is tracked): nanoseconds since stamp_secs
        ("last_seen_sec", ctypes.c_uint32),          # seconds part of last_seen timestamp: seconds (stamp_secs) since epoch
        ("last_seen_nsec", ctypes.c_uint32),         # nanoseconds part of last_seen timestamp: nanoseconds since stamp_secs
        # geometry_msgs/TwistWithCovariance velocity
        ("velocity_linear", SickScanVector3Msg),
        ("velocity_angular", SickScanVector3Msg),
        ("velocity_covariance", ctypes.c_double * 36), # Row-major representation of the 6x6 covariance matrix (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        # geometry_msgs/Pose bounding_box_center
        ("bounding_box_center_position", SickScanVector3Msg),
        ("bounding_box_center_orientation", SickScanQuaternionMsg),
        # geometry_msgs/Vector3 bounding_box_size
        ("bounding_box_size", SickScanVector3Msg),
        # geometry_msgs/PoseWithCovariance object_box_center
        ("object_box_center_position", SickScanVector3Msg),
        ("object_box_center_orientation", SickScanQuaternionMsg),
        ("object_box_center_covariance", ctypes.c_double * 36), # Row-major representation of the 6x6 covariance matrix (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        # geometry_msgs/Vector3 object_box_size
        ("object_box_size", SickScanVector3Msg),
        # geometry_msgs/Point[] contour_points
        ("contour_points", SickScanPointArray)
    ]

class SickScanRadarObjectArray(ctypes.Structure):    # sick_scan_api: struct SickScanRadarObjectArray, Array of SickScanRadarObject
    _fields_ = [
        ("capacity", ctypes.c_uint64),                  # Number of allocated elements, i.e. max. number of elements in buffer, allocated buffer size is capacity * sizeof(SickScanRadarObject)
        ("size", ctypes.c_uint64),                      # Number of currently used elements in the buffer
        ("buffer", ctypes.POINTER(SickScanRadarObject)) # Memory, data in plain order and system endianess (buffer == 0, if size == 0 && capacity == 0, otherwise allocated memory), allocation/deallocation always managed by the caller.
    ]

class SickScanRadarScan(ctypes.Structure):           # sick_scan_api: struct SickScanRadarScan, equivalent to RadarScan.msg
    _fields_ = [
        ("header", SickScanHeader),                  # message timestamp
        ("radarpreheader", SickScanRadarPreHeader),  # RadarPreHeader.msg
        ("targets", SickScanPointCloudMsg),          # sensor_msgs/PointCloud2
        ("objects", SickScanRadarObjectArray)        # Array of RadarObject.msg
    ]

class SickScanLdmrsObject(SickScanRadarObject):      # sick_scan_api: struct SickScanLdmrsObject, equivalent to SickLdmrsObject.msg
    pass

class SickScanLdmrsObjectBuffer(ctypes.Structure):   # sick_scan_api: struct SickScanLdmrsObjectBuffer, Array of SickScanLdmrsObject
    _fields_ = [
        ("capacity", ctypes.c_uint64),                  # Number of allocated elements, i.e. max. number of elements in buffer, allocated buffer size is capacity * sizeof(SickScanLdmrsObject)
        ("size", ctypes.c_uint64),                      # Number of currently used elements in the buffer
        ("buffer", ctypes.POINTER(SickScanLdmrsObject)) # Memory, data in plain order and system endianess (buffer == 0, if size == 0 && capacity == 0, otherwise allocated memory), allocation/deallocation always managed by the caller.
    ]

class SickScanLdmrsObjectArray(ctypes.Structure):    # sick_scan_api: struct SickScanLdmrsObjectArray, equivalent to SickLdmrsObjectArray.msg
    _fields_ = [
        ("header", SickScanHeader),                  # message timestamp
        ("objects", SickScanLdmrsObjectBuffer)       # Array of SickScanLdmrsObjects
    ]

class SickScanColorRGBA(ctypes.Structure):               # equivalent to std_msgs::ColorRGBA
    _fields_ = [
        ("r", ctypes.c_float),
        ("g", ctypes.c_float),
        ("b", ctypes.c_float),
        ("a", ctypes.c_float)
    ]

class SickScanColorRGBAArray(ctypes.Structure):          # Array of SickScanColorRGBA, which can be serialized and imported in C, C++ or python
    _fields_ = [
        ("capacity", ctypes.c_uint64),                   # Number of allocated elements, i.e. max. number of elements in buffer, allocated buffer size is capacity * sizeof(SickScanColorRGBA)
        ("size", ctypes.c_uint64),                       # Number of currently used elements in the buffer
        ("buffer", ctypes.POINTER(SickScanColorRGBA))    # Memory, data in plain order and system endianess (buffer == 0, if size == 0 && capacity == 0, otherwise allocated memory), allocation/deallocation always managed by the caller.
    ]

class SickScanVisualizationMarker(ctypes.Structure):     # equivalent to visualization_msgs::Marker
    _fields_ = [
        ("header", SickScanHeader),                      # message timestamp
        ("ns", ctypes.c_char * 1024),                    # Namespace to place this object in... used in conjunction with id to create a unique name for the object
        ("id", ctypes.c_int32),                          # object ID useful in conjunction with the namespace for manipulating and deleting the object later
        ("type", ctypes.c_int32),                        # Type of object
        ("action", ctypes.c_int32),                      # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
        ("pose_position", SickScanVector3Msg),           # Pose of the object (positional part)
        ("pose_orientation", SickScanQuaternionMsg),     # Pose of the object (rotational part)
        ("scale", SickScanVector3Msg),                   # Scale of the object 1,1,1 means default (usually 1 meter square)
        ("color", SickScanColorRGBA),                    # Color [0.0-1.0]
        ("lifetime_sec", ctypes.c_uint32),               # How long the object should last before being automatically deleted.  0 means forever (seconds part)
        ("lifetime_nsec", ctypes.c_uint32),              # How long the object should last before being automatically deleted.  0 means forever (nanoseconds part)
        ("frame_locked", ctypes.c_uint8),                # boolean, If this marker should be frame-locked, i.e. retransformed into its frame every timestep
        ("points", SickScanPointArray),                  # Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
        ("colors", SickScanColorRGBAArray),              # Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...). Number of colors must either be 0 or equal to the number of points. NOTE: alpha is not yet used
        ("text", ctypes.c_char * 1024),                  # NOTE: only used for text markers
        ("mesh_resource", ctypes.c_char * 1024),         # NOTE: only used for MESH_RESOURCE markers
        ("mesh_use_embedded_materials", ctypes.c_uint8)  # boolean
    ]

class SickScanVisualizationMarkerBuffer(ctypes.Structure):      # Array of SickScanVisualizationMarkers
    _fields_ = [
        ("capacity", ctypes.c_uint64),                          # Number of allocated elements, i.e. max. number of elements in buffer, allocated buffer size is capacity * sizeof(SickScanVisualizationMarker)
        ("size", ctypes.c_uint64),                              # Number of currently used elements in the buffer
        ("buffer", ctypes.POINTER(SickScanVisualizationMarker)) # Memory, data in plain order and system endianess (buffer == 0, if size == 0 && capacity == 0, otherwise allocated memory), allocation/deallocation always managed by the caller.
    ]

class SickScanVisualizationMarkerMsg(ctypes.Structure):         # equivalent to visualization_msgs::MarkerArray
    _fields_ = [
        ("markers", SickScanVisualizationMarkerBuffer)          # Array of SickScanVisualizationMarkers
    ]

#
#  Callback declarations
#

SickScanPointCloudMsgCallback = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(SickScanPointCloudMsg))                 # sick_scan_api.h: typedef void(* SickScanPointCloudMsgCallback)(SickScanApiHandle apiHandle, const SickScanPointCloudMsg* msg);
SickScanImuMsgCallback = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(SickScanImuMsg))                               # sick_scan_api.h: typedef void(* SickScanImuMsgCallback)(SickScanApiHandle apiHandle, const SickScanImuMsg* msg);
SickScanLFErecMsgCallback = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(SickScanLFErecMsg))                         # sick_scan_api.h: typedef void(* SickScanLFErecMsgCallback)(SickScanApiHandle apiHandle, const SickScanLFErecMsg* msg);
SickScanLIDoutputstateMsgCallback = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(SickScanLIDoutputstateMsg))         # sick_scan_api.h: typedef void(* SickScanLIDoutputstateMsgCallback)(SickScanApiHandle apiHandle, const SickScanLIDoutputstateMsg* msg);
SickScanRadarScanCallback = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(SickScanRadarScan))                         # sick_scan_api.h: typedef void(* SickScanRadarScanCallback)(SickScanApiHandle apiHandle, const SickScanRadarScan* msg);
SickScanLdmrsObjectArrayCallback = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(SickScanLdmrsObjectArray))           # sick_scan_api.h: typedef void(* SickScanLdmrsObjectArrayCallback)(SickScanApiHandle apiHandle, const SickScanLdmrsObjectArray* msg);
SickScanVisualizationMarkerCallback = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(SickScanVisualizationMarkerMsg))  # sick_scan_api.h: typedef void(* SickScanVisualizationMarkerCallback)(SickScanApiHandle apiHandle, const SickScanVisualizationMarkerMsg* msg);

#
#  Functions to initialize and close the API and a lidar
#

# Load sick_scan_library and functions
def SickScanApiLoadLibrary(paths, lib_filname):
   pass # see implementation in sick_scan_api.py

# Unload sick_scan_xd api library
def SickScanApiUnloadLibrary(sick_scan_library):
   pass # see implementation in sick_scan_api.py

# Create an instance of sick_scan_xd api.
# Call SickScanApiInitByLaunchfile or SickScanApiInitByCli to process a lidar.
def SickScanApiCreate(sick_scan_library):
   pass # see implementation in sick_scan_api.py

# Release and free all resources of the api handle; the handle is invalid after SickScanApiRelease
def SickScanApiRelease(sick_scan_library, api_handle):
   pass # see implementation in sick_scan_api.py

# Initialize a lidar by launchfile and start message receiving and processing
def SickScanApiInitByLaunchfile(sick_scan_library, api_handle, launchfile_args):
   pass # see implementation in sick_scan_api.py

# Release and free all resources of the api handle; the handle is invalid after SickScanApiRelease
def SickScanApiClose(sick_scan_library, api_handle):
   pass # see implementation in sick_scan_api.py

#
#  Registration / deregistration of message callbacks
#

# Register / deregister a callback for cartesian PointCloud messages, pointcloud in cartesian coordinates with fields x, y, z, intensity
def SickScanApiRegisterCartesianPointCloudMsg(sick_scan_library, api_handle, pointcloud_callback):
   pass # see implementation in sick_scan_api.py
def SickScanApiDeregisterCartesianPointCloudMsg(sick_scan_library, api_handle, pointcloud_callback):
   pass # see implementation in sick_scan_api.py

# Register / deregister a callback for polar PointCloud messages, pointcloud in polar coordinates with fields range, azimuth, elevation, intensity
def SickScanApiRegisterPolarPointCloudMsg(sick_scan_library, api_handle, pointcloud_callback):
   pass # see implementation in sick_scan_api.py
def SickScanApiDeregisterPolarPointCloudMsg(sick_scan_library, api_handle, pointcloud_callback):
   pass # see implementation in sick_scan_api.py

# Register / deregister a callback for Imu messages
def SickScanApiRegisterImuMsg(sick_scan_library, api_handle, imu_callback):
   pass # see implementation in sick_scan_api.py
def SickScanApiDeregisterImuMsg(sick_scan_library, api_handle, imu_callback):
   pass # see implementation in sick_scan_api.py

# Register / deregister a callback for LFErec messages
def SickScanApiRegisterLFErecMsg(sick_scan_library, api_handle, lferec_callback):
   pass # see implementation in sick_scan_api.py
def SickScanApiDeregisterLFErecMsg(sick_scan_library, api_handle, lferec_callback):
   pass # see implementation in sick_scan_api.py

# Register / deregister a callback for LIDoutputstate messages
def SickScanApiRegisterLIDoutputstateMsg(sick_scan_library, api_handle, lidoutputstate_callback):
   pass # see implementation in sick_scan_api.py
def SickScanApiDeregisterLIDoutputstateMsg(sick_scan_library, api_handle, lidoutputstate_callback):
   pass # see implementation in sick_scan_api.py

# Register / deregister a callback for RadarScan messages
def SickScanApiRegisterRadarScanMsg(sick_scan_library, api_handle, radarscan_callback):
   pass # see implementation in sick_scan_api.py
def SickScanApiDeregisterRadarScanMsg(sick_scan_library, api_handle, radarscan_callback):
   pass # see implementation in sick_scan_api.py

# Register / deregister a callback for LdmrsObjectArray messages
def SickScanApiRegisterLdmrsObjectArrayMsg(sick_scan_library, api_handle, ldmrsobjectarray_callback):
   pass # see implementation in sick_scan_api.py
def SickScanApiDeregisterLdmrsObjectArrayMsg(sick_scan_library, api_handle, ldmrsobjectarray_callback):
   pass # see implementation in sick_scan_api.py

# Register / deregister a callback for VisualizationMarker messages
def SickScanApiRegisterVisualizationMarkerMsg(sick_scan_library, api_handle, ldmrsobjectarray_callback):
   pass # see implementation in sick_scan_api.py
def SickScanApiDeregisterVisualizationMarkerMsg(sick_scan_library, api_handle, ldmrsobjectarray_callback):
   pass # see implementation in sick_scan_api.py
```

## Usage example

The following code snippet shows a simple example of a customized C/C++ application using the sick_scan_xd API:

```
#include "sick_scan_api.h"

// Implement a callback to process pointcloud messages
void customizedPointCloudMsgCallback(SickScanApiHandle apiHandle, const SickScanPointCloudMsgCallback* msg)
{
  printf("Pointcloud message received\n");
}

// Create a sick_scan instance and initialize a LMS-511
SickScanApiLoadLibrary("sick_scan_api.dll");
SickScanApiHandle apiHandle = SickScanApiCreate();
SickScanApiInitByLaunchfile(apiHandle, "sick_lms_5xx.launch");

// Register for pointcloud messages
SickScanApiRegisterCartesianPointCloudMsg(apiHandle, customizedPointCloudMsgCallback);

// Run application or main loop
getchar();

// Close lidar and release sick_scan api
SickScanApiDeregisterPointCloudMsg(apiHandle, customizedPointCloudMsgCallback);
SickScanApiClose(apiHandle);
SickScanApiRelease(apiHandle);
SickScanApiUnloadLibrary();
```

Note: All functions named `SickScanApi` are implemented within the library file ("sick_scan_shared_lib.dll" on Windows resp. "libsick_scan_shared_lib.so" on Linux). A small wrapper is included in the examples, which loads and unloads the library (functions `SickScanApiLoadLibrary` and `SickScanApiUnloadLibrary`) and delegates the function calls to the binary.

A C/C++ usage example is implemented in [sick_scan_xd_api_test.cpp](../../test/src/sick_scan_xd_api/sick_scan_xd_api_test.cpp).

A python usage example is implemented in [sick_scan_xd_api_test.py](../../test/python/sick_scan_xd_api/sick_scan_xd_api_test.py).

## Useful links

[ctypes](https://docs.python.org/3/library/ctypes.html) is used for data exchange and function calls between Python and C-libraries:
* https://docs.python.org/3/library/ctypes.html
* https://docs.python.org/3/library/ctypes.html#structures-and-unions
* https://docs.python.org/3/library/ctypes.html#callback-functions
