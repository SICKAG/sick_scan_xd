#
# Dockertest for sick_scan_xd python api
#
# Make sure that libsick_scan_xd_api_lib.so is included in the system path
# Usage
# python3 sick_scan_xd_api_dockertest.py [options]
#

import json
import os
import signal
import sys
import time
sys.path.append("src/sick_scan_xd/python/api")
from sick_scan_api import *

# global settings
class ApiTestSettings:
    def __init__(self):
        self.verbose_level = 1    # Set verbose level 0=DEBUG, 1=INFO, 2=WARN, 3=ERROR, 4=FATAL or 5=QUIET (equivalent to ros::console::levels)
        self.imu_topic = ""       # option to overwrite imu topic
        self.export_jsonfile = "" # output json file, exports all pointcloud and imu messages
        self.exit_signal_received = False # set to true in signal handler
        
# Convert a pointcloud message into a (json) dictionary
def pointCloudMsgToDict(msg):
    fields = []
    for n in range(msg.fields.size):
        field = {}
        field["name"] = ctypesCharArrayToString(msg.fields.buffer[n].name)
        field["offset"] = msg.fields.buffer[n].offset
        field["datatype"] = msg.fields.buffer[n].datatype
        field["count"] = msg.fields.buffer[n].count
        fields.append(field)
    data_buffer = bytearray(msg.data.size)
    for n in range(msg.data.size):
        data_buffer[n] = msg.data.buffer[n]
    hex_data_str = "".join("{:02x}".format(x) for x in data_buffer)
    return {
        "frame_id": ctypesCharArrayToString(msg.header.frame_id),
        "width": msg.width,
        "height": msg.height,
        "point_step": msg.point_step,
        "row_step": msg.row_step,
        "fields": fields,
        "data": hex_data_str,
    }

# Convert an imu message into a (json) dictionary
def imuMsgToDict(msg):
    return {
        "frame_id": ctypesCharArrayToString(msg.header.frame_id),
        "orientation": [ msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w ],
        "orientation_covariance": [ msg.orientation_covariance[0], msg.orientation_covariance[2], msg.orientation_covariance[2], msg.orientation_covariance[3], msg.orientation_covariance[4], msg.orientation_covariance[5], msg.orientation_covariance[6], msg.orientation_covariance[7], msg.orientation_covariance[8] ],
        "angular_velocity": [ msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z ],
        "angular_velocity_covariance": [ msg.angular_velocity_covariance[0], msg.angular_velocity_covariance[1], msg.angular_velocity_covariance[2], msg.angular_velocity_covariance[3], msg.angular_velocity_covariance[4], msg.angular_velocity_covariance[5], msg.angular_velocity_covariance[6], msg.angular_velocity_covariance[7], msg.angular_velocity_covariance[8] ],
        "linear_acceleration": [ msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z ],
        "linear_acceleration_covariance": [ msg.linear_acceleration_covariance[0], msg.linear_acceleration_covariance[1], msg.linear_acceleration_covariance[2], msg.linear_acceleration_covariance[3], msg.linear_acceleration_covariance[4], msg.linear_acceleration_covariance[5], msg.linear_acceleration_covariance[6], msg.linear_acceleration_covariance[7], msg.linear_acceleration_covariance[8] ],
    }

# Prints a json value to string, replacing all '\n', '\r' and '\t' by space ' '
def jsonToOneLineString(json_msg):
    json_print = json.dumps(json_msg)
    json_print = json_print.replace('\n', ' ')
    json_print = json_print.replace('\r', ' ')
    json_print = json_print.replace('\t', ' ')
    return json_print;

# Append to temporary json logfile
def appendJsonMsgToLogfile(type_str, topic_str, json_msg):
    global api_test_settings, json_pointcloud_messages, json_imu_messages
    if len(api_test_settings.export_jsonfile) > 0:
        json_log_msg_topic = {}
        json_log_msg_topic[topic_str] = json_msg;
        json_log_msg = {}
        json_log_msg[type_str] = json_log_msg_topic;
        json_log_str = jsonToOneLineString(json_log_msg);
        try:
            with open(api_test_settings.export_jsonfile + ".log", "a") as file_stream:
                file_stream.write(f"{json_log_str}\n")
        except Exception as exc:
            print(f"## ERROR in sick_scan_xd_api_dockertest.py: append to (\"{api_test_settings.export_jsonfile}.log\") failed, exception {exc}")

#
# SickScanApi-callbacks
#

# Callback for cartesian pointcloud messages
def pySickScanCartesianPointCloudMsgCallback(api_handle, pointcloud_msg):
    pointcloud_msg = pointcloud_msg.contents # dereference msg pointer (pointcloud_msg = pointcloud_msg[0])
    global api_test_settings, json_pointcloud_messages
    topic = ctypesCharArrayToString(pointcloud_msg.topic)
    print(f"pySickScanCartesianPointCloudMsgCallback: api_handle={api_handle}, {pointcloud_msg.width}x{pointcloud_msg.height} pointcloud, {pointcloud_msg.num_echos} echo(s), segment {pointcloud_msg.segment_idx}, {pointcloud_msg.fields.size} fields, frame_id \"{pointcloud_msg.header.frame_id}\", topic {topic}, timestamp {pointcloud_msg.header.timestamp_sec}.{pointcloud_msg.header.timestamp_nsec:06d}")
    json_dict = pointCloudMsgToDict(pointcloud_msg)
    if topic not in json_pointcloud_messages:
        json_pointcloud_messages[topic] = []
    json_pointcloud_messages[topic].append(json_dict)
    appendJsonMsgToLogfile("RefPointcloudMsg", topic, json_dict)

# Callback for polar pointcloud messages
def pySickScanPolarPointCloudMsgCallback(api_handle, pointcloud_msg):
    pointcloud_msg = pointcloud_msg.contents # dereference msg pointer (pointcloud_msg = pointcloud_msg[0])
    global api_test_settings, json_pointcloud_messages
    topic = ctypesCharArrayToString(pointcloud_msg.topic)
    print(f"pySickScanPolarPointCloudMsgCallback: api_handle={api_handle}, {pointcloud_msg.width}x{pointcloud_msg.height} pointcloud, {pointcloud_msg.num_echos} echo(s), segment {pointcloud_msg.segment_idx}, {pointcloud_msg.fields.size} fields, frame_id \"{pointcloud_msg.header.frame_id}\", topic {topic}, timestamp {pointcloud_msg.header.timestamp_sec}.{pointcloud_msg.header.timestamp_nsec:06d}")
    json_dict = pointCloudMsgToDict(pointcloud_msg)
    if topic not in json_pointcloud_messages:
        json_pointcloud_messages[topic] = []
    json_pointcloud_messages[topic].append(json_dict)
    appendJsonMsgToLogfile("RefPointcloudMsg", topic, json_dict)

# Callback for Imu messages
def pySickScanImuMsgCallback(api_handle, imu_msg):
    imu_msg = imu_msg.contents # dereference msg pointer
    global api_test_settings, json_imu_messages
    topic = api_test_settings.imu_topic # ctypesCharArrayToString(imu_msg.topic) # TODO ...
    if len(api_test_settings.imu_topic) > 0:
        topic = api_test_settings.imu_topic # imu topic overwritten by commandline option
    print("pySickScanImuMsgCallback: api_handle={}, imu message: orientation=({:.8},{:.8},{:.8},{:.8}), angular_velocity=({:.8},{:.8},{:.8}), linear_acceleration=({:.8},{:.8},{:.8}), topic={}".format(
        api_handle, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w, 
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.y, 
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z, topic))
    json_dict = imuMsgToDict(imu_msg)
    if topic not in json_imu_messages:
        json_imu_messages[topic] = []
    json_imu_messages[topic].append(json_dict)
    appendJsonMsgToLogfile("RefImuMsg", topic, json_dict)

# Callback for RadarScan messages
def pySickScanRadarScanCallback(api_handle, radar_msg):
    radar_msg = radar_msg.contents # dereference msg pointer
    global api_test_settings, json_pointcloud_messages
    topic = "/cloud_radar_track"
    print(f"pySickScanRadarScanCallback: api_handle={api_handle}")
    json_dict = pointCloudMsgToDict(radar_msg.targets)
    if topic not in json_pointcloud_messages:
        json_pointcloud_messages[topic] = []
    json_pointcloud_messages[topic].append(json_dict)
    appendJsonMsgToLogfile("RefPointcloudMsg", topic, json_dict)

# Callback for SickScanDiagnosticMsg messages
def pySickScanDiagnosticMsgCallback(api_handle, diagnostic_msg):
    diagnostic_msg = diagnostic_msg.contents # dereference msg pointer
    print(f"pySickScanDiagnosticMsgCallback: api_handle={api_handle}, status_code={diagnostic_msg.status_code}, status_message={diagnostic_msg.status_message}")

# Callback for SickScanLogMsg messages
def pySickScanLogMsgCallback(api_handle, log_msg):
    log_msg = log_msg.contents # dereference msg pointer
    if log_msg.log_level >= 1: # log_level 0=DEBUG, 1=INFO, 2=WARN, 3=ERROR, 4=FATAL or 5=QUIET (equivalent to ros::console::levels)
        print(f"pySickScanLogMsgCallback: api_handle={api_handle}, log_level={log_msg.log_level}, log_message={log_msg.log_message}")

# signal handler to close docker test after SIGINT and SIGKILL
def pySignalHandler(sig, frame):
    print(f"pySignalHandler: caught signal {sig}, aborting ...")
    # Export all received messages to json-file
    global api_test_settings, json_pointcloud_messages, json_imu_messages
    json_root = {}
    if len(json_pointcloud_messages) > 0:
        json_root["RefPointcloudMsg"] = json_pointcloud_messages
    if len(json_imu_messages) > 0:
        json_root["RefImuMsg"] = json_imu_messages
    if len(api_test_settings.export_jsonfile) > 0:
        if len(json_root) > 0:
            try:
                with open(api_test_settings.export_jsonfile, "w") as file_stream:
                    json.dump(json_root, file_stream, indent=2)
                    print(f"sick_scan_xd_api_dockertest.py: messages exported to file \"{api_test_settings.export_jsonfile}\"")
            except Exception as exc:
                print(f"## ERROR in sick_scan_xd_api_dockertest.py: export to (\"{api_test_settings.export_jsonfile}\") failed, exception {exc}")
        else:
            print(f"## ERROR in sick_scan_xd_api_dockertest.py: no messages received, file \"{api_test_settings.export_jsonfile}\" not written")
    # Stop lidar, close connection and api handle
    global sick_scan_library, api_handle, cartesian_pointcloud_callback, polar_pointcloud_callback, imu_callback, radar_scan_callback, diagnostic_msg_callback, log_msg_callback
    print(f"sick_scan_xd_api_dockertest.py finishing...")
    SickScanApiDeregisterCartesianPointCloudMsg(sick_scan_library, api_handle, cartesian_pointcloud_callback)
    SickScanApiDeregisterPolarPointCloudMsg(sick_scan_library, api_handle, polar_pointcloud_callback)
    SickScanApiDeregisterImuMsg(sick_scan_library, api_handle, imu_callback)
    SickScanApiDeregisterRadarScanMsg(sick_scan_library, api_handle, radar_scan_callback)
    SickScanApiDeregisterDiagnosticMsg(sick_scan_library, api_handle, diagnostic_msg_callback)
    SickScanApiDeregisterLogMsg(sick_scan_library, api_handle, log_msg_callback)
    SickScanApiRelease(sick_scan_library, api_handle)
    SickScanApiUnloadLibrary(sick_scan_library)
    print(f"sick_scan_xd_api_dockertest.py finished.")
    api_test_settings.exit_signal_received = True

#
# Python usage example for sick_scan_api
#

if __name__ == "__main__":

    # Configuration and commandline arguments
    api_test_settings = ApiTestSettings()
    cli_args = []
    for n, cli_arg in enumerate(sys.argv):
        if cli_arg.startswith("_verbose:="):
            api_test_settings.verbose_level = int(cli_arg[10:])
        elif cli_arg.startswith("imu_topic:="):
            api_test_settings.imu_topic = cli_arg[11:]
        elif cli_arg.startswith("_jsonfile:="):
            api_test_settings.export_jsonfile = cli_arg[11:]
        elif n > 0:
            cli_args.append(cli_arg)
    cli_args = " ".join(cli_args)

    # Load sick_scan_library
    if os.name == 'nt': # Load windows dll
        sick_scan_library = SickScanApiLoadLibrary(["build/Debug/", "build_win64/Debug/", "src/build/Debug/", "src/build_win64/Debug/", "src/sick_scan_xd/build/Debug/", "src/sick_scan_xd/build_win64/Debug/", "./", "../"], "sick_scan_xd_shared_lib.dll")
    else: # Load linux so
        sick_scan_library = SickScanApiLoadLibrary(["build/", "build_linux/", "src/build/", "src/build_linux/", "src/sick_scan_xd/build/", "src/sick_scan_xd/build_linux/", "./", "../"], "libsick_scan_xd_shared_lib.so")
    api_handle = SickScanApiCreate(sick_scan_library)

    # Initialize lidar by launchfile, e.g. sick_tim_7xx.launch
    # SickScanApiInitByLaunchfile(sick_scan_library, api_handle, "./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False")
    print(f"sick_scan_xd_api_dockertest.py: initializing lidar, commandline arguments = \"{cli_args}\"")
    SickScanApiInitByLaunchfile(sick_scan_library, api_handle, cli_args)
    json_pointcloud_messages = {}
    json_imu_messages = {}
    signal.signal(signal.SIGINT, pySignalHandler)
    signal.signal(signal.SIGTERM, pySignalHandler)

    # Register a callback for PointCloud messages
    cartesian_pointcloud_callback = SickScanPointCloudMsgCallback(pySickScanCartesianPointCloudMsgCallback)
    SickScanApiRegisterCartesianPointCloudMsg(sick_scan_library, api_handle, cartesian_pointcloud_callback)
    polar_pointcloud_callback = SickScanPointCloudMsgCallback(pySickScanPolarPointCloudMsgCallback)
    SickScanApiRegisterPolarPointCloudMsg(sick_scan_library, api_handle, polar_pointcloud_callback)

    # Register a callback for Imu messages
    imu_callback = SickScanImuMsgCallback(pySickScanImuMsgCallback)
    SickScanApiRegisterImuMsg(sick_scan_library, api_handle, imu_callback)

     # Register a callback for RadarScan messages
    radar_scan_callback = SickScanRadarScanCallback(pySickScanRadarScanCallback)
    SickScanApiRegisterRadarScanMsg(sick_scan_library, api_handle, radar_scan_callback)

    # Register a callback for SickScanDiagnosticMsg messages
    diagnostic_msg_callback = SickScanDiagnosticMsgCallback(pySickScanDiagnosticMsgCallback)
    SickScanApiRegisterDiagnosticMsg(sick_scan_library, api_handle, diagnostic_msg_callback)

    # Register a callback for SickScanLogMsg messages
    log_msg_callback = SickScanLogMsgCallback(pySickScanLogMsgCallback)
    SickScanApiRegisterLogMsg(sick_scan_library, api_handle, log_msg_callback)

    # Set verbose level 0=DEBUG, 1=INFO, 2=WARN, 3=ERROR, 4=FATAL or 5=QUIET (equivalent to ros::console::levels)
    SickScanApiSetVerboseLevel(sick_scan_library, api_handle, api_test_settings.verbose_level)
    verbose_level = SickScanApiGetVerboseLevel(sick_scan_library, api_handle)
    if api_test_settings.verbose_level != verbose_level:
        print(f"## ERROR sick_scan_xd_api_dockertest.py: SickScanApiSetVerboseLevel(verbose_level={api_test_settings.verbose_level}) failed, running with verbose_level={verbose_level}")
    else:
        print(f"sick_scan_xd_api_dockertest.py running with verbose_level={verbose_level}")

    # Run main loop (wait for SIGINT or SIGKILL)
    try:
        if os.name == 'nt': # windows
            while not api_test_settings.exit_signal_received:
                time.sleep(1)
        else: # linux
            signal.pause()
    except Exception as exc:
        print(f"sick_scan_xd_api_dockertest.py: exception \"{exc}\"")
