#
# This file declares the functions and datatypes of the sick_scan_xd python-API.
# See doc/sick_scan_api/sick_scan_api.md for further information.
#
import ctypes
import numpy as np
import os
from enum import Enum

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

class SickScanImuMsg(ctypes.Structure):              # sick_scan_api: struct SickScanImuMsg, equivalent to SickImu.msg
    _fields_ = [
        ("header", SickScanHeader),                              # message timestamp
        ("orientation", SickScanQuaternionMsg),
        ("orientation_covariance", ctypes.c_double * 9),         # Row major about x, y, z axes
        ("angular_velocity", SickScanVector3Msg),
        ("angular_velocity_covariance", ctypes.c_double * 9),    # Row major about x, y, z axes
        ("linear_acceleration", SickScanVector3Msg),
        ("linear_acceleration_covariance", ctypes.c_double * 9), # Row major x, y z
        ("ticks", ctypes.c_uint32),                              # timestamp Ticks from laser scanner
        ("quaternion_accuracy", ctypes.c_float),                 # quaternion accuracy in rad
        ("angular_velocity_reliability", ctypes.c_uint8),        # angular velocity reliability 0 low 255 high, value should be over 3
        ("linear_acceleration_reliability", ctypes.c_uint8)      # linear acceleration reliability 0 low 255 high, value should be over 3
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
        ("objects", SickScanLdmrsObjectBuffer)      # Array of SickScanLdmrsObjects
    ]

#
#  Callback declarations
#

SickScanPointCloudMsgCallback = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(SickScanPointCloudMsg))         # sick_scan_api.h: typedef void(* SickScanPointCloudMsgCallback)(SickScanApiHandle apiHandle, const SickScanPointCloudMsg* msg);
SickScanImuMsgCallback = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(SickScanImuMsg))                       # sick_scan_api.h: typedef void(* SickScanImuMsgCallback)(SickScanApiHandle apiHandle, const SickScanImuMsg* msg);
SickScanLFErecMsgCallback = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(SickScanLFErecMsg))                 # sick_scan_api.h: typedef void(* SickScanLFErecMsgCallback)(SickScanApiHandle apiHandle, const SickScanLFErecMsg* msg);
SickScanLIDoutputstateMsgCallback = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(SickScanLIDoutputstateMsg)) # sick_scan_api.h: typedef void(* SickScanLIDoutputstateMsgCallback)(SickScanApiHandle apiHandle, const SickScanLIDoutputstateMsg* msg);
SickScanRadarScanCallback = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(SickScanRadarScan))                 # sick_scan_api.h: typedef void(* SickScanRadarScanCallback)(SickScanApiHandle apiHandle, const SickScanRadarScan* msg);
SickScanLdmrsObjectArrayCallback = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(SickScanLdmrsObjectArray))   # sick_scan_api.h: typedef void(* SickScanLdmrsObjectArrayCallback)(SickScanApiHandle apiHandle, const SickScanLdmrsObjectArray* msg);

#
#  Functions to initialize and close the API and a lidar
#

# Returns a python string from a zero terminated ctypes char array
def ctypesCharArrayToString(char_array):
    return ''.join([chr(i) for i in char_array]).rstrip('\x00')

# loads and returns a library, given its filename and a list of folder 
def loadLibrary(paths, lib_filname):
    for path in paths:
        filename = path + lib_filname
        if os.path.exists(filename):
            return ctypes.CDLL(filename)
    return ctypes.CDLL(lib_filname)

# Load sick_scan_library and functions
def SickScanApiLoadLibrary(paths, lib_filname):
    sick_scan_library = loadLibrary(paths, lib_filname)
    # sick_scan_api.h: SickScanApiHandle SickScanApiCreate(int argc, char** argv);
    sick_scan_library.SickScanApiCreate.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_char_p)]
    sick_scan_library.SickScanApiCreate.restype = ctypes.c_void_p
    # sick_scan_api.h: int32_t SickScanApiRelease(SickScanApiHandle apiHandle);
    sick_scan_library.SickScanApiRelease.argtypes = [ctypes.c_void_p]
    sick_scan_library.SickScanApiRelease.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiInitByLaunchfile(SickScanApiHandle apiHandle, const char* launchfile_args);
    sick_scan_library.SickScanApiInitByLaunchfile.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    sick_scan_library.SickScanApiInitByLaunchfile.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiInitByCli(SickScanApiHandle apiHandle, int argc, char** argv);
    sick_scan_library.SickScanApiInitByCli.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_char_p)]
    sick_scan_library.SickScanApiInitByCli.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiClose(SickScanApiHandle apiHandle);
    sick_scan_library.SickScanApiClose.argtypes = [ctypes.c_void_p]
    sick_scan_library.SickScanApiClose.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiRegisterPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback);
    sick_scan_library.SickScanApiRegisterCartesianPointCloudMsg.argtypes = [ctypes.c_void_p, SickScanPointCloudMsgCallback]
    sick_scan_library.SickScanApiRegisterCartesianPointCloudMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiDeregisterCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback);
    sick_scan_library.SickScanApiDeregisterCartesianPointCloudMsg.argtypes = [ctypes.c_void_p, SickScanPointCloudMsgCallback]
    sick_scan_library.SickScanApiDeregisterCartesianPointCloudMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiRegisterPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback);
    sick_scan_library.SickScanApiRegisterPolarPointCloudMsg.argtypes = [ctypes.c_void_p, SickScanPointCloudMsgCallback]
    sick_scan_library.SickScanApiRegisterPolarPointCloudMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiDeregisterPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback);
    sick_scan_library.SickScanApiDeregisterPolarPointCloudMsg.argtypes = [ctypes.c_void_p, SickScanPointCloudMsgCallback]
    sick_scan_library.SickScanApiDeregisterPolarPointCloudMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiRegisterImuMsg(SickScanApiHandle apiHandle, SickScanImuMsgCallback callback);
    sick_scan_library.SickScanApiRegisterImuMsg.argtypes = [ctypes.c_void_p, SickScanImuMsgCallback]
    sick_scan_library.SickScanApiRegisterImuMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiDeregisterImuMsg(SickScanApiHandle apiHandle, SickScanImuMsgCallback callback);
    sick_scan_library.SickScanApiDeregisterImuMsg.argtypes = [ctypes.c_void_p, SickScanImuMsgCallback]
    sick_scan_library.SickScanApiDeregisterImuMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiRegisterLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsgCallback callback);
    sick_scan_library.SickScanApiRegisterLFErecMsg.argtypes = [ctypes.c_void_p, SickScanLFErecMsgCallback]
    sick_scan_library.SickScanApiRegisterLFErecMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiDeregisterLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsgCallback callback);
    sick_scan_library.SickScanApiDeregisterLFErecMsg.argtypes = [ctypes.c_void_p, SickScanLFErecMsgCallback]
    sick_scan_library.SickScanApiDeregisterLFErecMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiRegisterLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsgCallback callback);
    sick_scan_library.SickScanApiRegisterLIDoutputstateMsg.argtypes = [ctypes.c_void_p, SickScanLIDoutputstateMsgCallback]
    sick_scan_library.SickScanApiRegisterLIDoutputstateMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiDeregisterLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsgCallback callback);
    sick_scan_library.SickScanApiDeregisterLIDoutputstateMsg.argtypes = [ctypes.c_void_p, SickScanLIDoutputstateMsgCallback]
    sick_scan_library.SickScanApiDeregisterLIDoutputstateMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiRegisterRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScanCallback callback);
    sick_scan_library.SickScanApiRegisterRadarScanMsg.argtypes = [ctypes.c_void_p, SickScanRadarScanCallback]
    sick_scan_library.SickScanApiRegisterRadarScanMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiDeregisterRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScanCallback callback);
    sick_scan_library.SickScanApiDeregisterRadarScanMsg.argtypes = [ctypes.c_void_p, SickScanRadarScanCallback]
    sick_scan_library.SickScanApiDeregisterRadarScanMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiRegisterLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArrayCallback callback);
    sick_scan_library.SickScanApiRegisterLdmrsObjectArrayMsg.argtypes = [ctypes.c_void_p, SickScanLdmrsObjectArrayCallback]
    sick_scan_library.SickScanApiRegisterLdmrsObjectArrayMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiDeregisterLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArrayCallback callback);
    sick_scan_library.SickScanApiDeregisterLdmrsObjectArrayMsg.argtypes = [ctypes.c_void_p, SickScanLdmrsObjectArrayCallback]
    sick_scan_library.SickScanApiDeregisterLdmrsObjectArrayMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiWaitNextCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg, double timeout_sec);
    sick_scan_library.SickScanApiWaitNextCartesianPointCloudMsg.argtypes = [ctypes.c_void_p, ctypes.POINTER(SickScanPointCloudMsg), ctypes.c_double]
    sick_scan_library.SickScanApiWaitNextCartesianPointCloudMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiWaitNextPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg, double timeout_sec);
    sick_scan_library.SickScanApiWaitNextPolarPointCloudMsg.argtypes = [ctypes.c_void_p, ctypes.POINTER(SickScanPointCloudMsg), ctypes.c_double]
    sick_scan_library.SickScanApiWaitNextPolarPointCloudMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiFreePolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg);
    sick_scan_library.SickScanApiFreePolarPointCloudMsg.argtypes = [ctypes.c_void_p, ctypes.POINTER(SickScanPointCloudMsg)]
    sick_scan_library.SickScanApiFreePolarPointCloudMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiWaitNextImuMsg(SickScanApiHandle apiHandle, SickScanImuMsg* msg, double timeout_sec);
    sick_scan_library.SickScanApiWaitNextImuMsg.argtypes = [ctypes.c_void_p, ctypes.POINTER(SickScanImuMsg), ctypes.c_double]
    sick_scan_library.SickScanApiWaitNextImuMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiFreeImuMsg(SickScanApiHandle apiHandle, SickScanImuMsg* msg)
    sick_scan_library.SickScanApiFreeImuMsg.argtypes = [ctypes.c_void_p, ctypes.POINTER(SickScanImuMsg)]
    sick_scan_library.SickScanApiFreeImuMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiWaitNextLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsg* msg, double timeout_sec);
    sick_scan_library.SickScanApiWaitNextLFErecMsg.argtypes = [ctypes.c_void_p, ctypes.POINTER(SickScanLFErecMsg), ctypes.c_double]
    sick_scan_library.SickScanApiWaitNextLFErecMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiFreeLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsg* msg);
    sick_scan_library.SickScanApiFreeLFErecMsg.argtypes = [ctypes.c_void_p, ctypes.POINTER(SickScanLFErecMsg)]
    sick_scan_library.SickScanApiFreeLFErecMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiWaitNextLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsg* msg, double timeout_sec);
    sick_scan_library.SickScanApiWaitNextLIDoutputstateMsg.argtypes = [ctypes.c_void_p, ctypes.POINTER(SickScanLIDoutputstateMsg), ctypes.c_double]
    sick_scan_library.SickScanApiWaitNextLIDoutputstateMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiFreeLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsg* msg);
    sick_scan_library.SickScanApiFreeLIDoutputstateMsg.argtypes = [ctypes.c_void_p, ctypes.POINTER(SickScanLIDoutputstateMsg)]
    sick_scan_library.SickScanApiFreeLIDoutputstateMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiWaitNextRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScan* msg, double timeout_sec);
    sick_scan_library.SickScanApiWaitNextRadarScanMsg.argtypes = [ctypes.c_void_p, ctypes.POINTER(SickScanRadarScan), ctypes.c_double]
    sick_scan_library.SickScanApiWaitNextRadarScanMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiFreeRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScan* msg);
    sick_scan_library.SickScanApiFreeRadarScanMsg.argtypes = [ctypes.c_void_p, ctypes.POINTER(SickScanRadarScan)]
    sick_scan_library.SickScanApiFreeRadarScanMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiWaitNextLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArray* msg, double timeout_sec);
    sick_scan_library.SickScanApiWaitNextLdmrsObjectArrayMsg.argtypes = [ctypes.c_void_p, ctypes.POINTER(SickScanLdmrsObjectArray), ctypes.c_double]
    sick_scan_library.SickScanApiWaitNextLdmrsObjectArrayMsg.restype = ctypes.c_int
    # sick_scan_api.h: int32_t SickScanApiFreeLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArray* msg);
    sick_scan_library.SickScanApiFreeLdmrsObjectArrayMsg.argtypes = [ctypes.c_void_p, ctypes.POINTER(SickScanLdmrsObjectArray)]
    sick_scan_library.SickScanApiFreeLdmrsObjectArrayMsg.restype = ctypes.c_int
    return sick_scan_library

# Create an instance of sick_scan_xd api.
# Optional commandline arguments argc, argv identical to sick_generic_caller.
# Call SickScanApiInitByLaunchfile or SickScanApiInitByCli to process a lidar.
def SickScanApiCreate(sick_scan_library):
    null_ptr = ctypes.POINTER(ctypes.c_char_p)()
    api_handle = sick_scan_library.SickScanApiCreate(0, null_ptr) # TODO: convert and call with argc, argv
    return api_handle

# Initialize a lidar by launchfile and start message receiving and processing
def SickScanApiInitByLaunchfile(sick_scan_library, api_handle, launchfile_args):
    ret = sick_scan_library.SickScanApiInitByLaunchfile(api_handle, ctypes.create_string_buffer(str.encode(launchfile_args)))
    return ret

# Register a callback for PointCloud messages
def SickScanApiRegisterCartesianPointCloudMsg(sick_scan_library, api_handle, pointcloud_callback):
    ret = sick_scan_library.SickScanApiRegisterCartesianPointCloudMsg(api_handle, pointcloud_callback)
    return ret
