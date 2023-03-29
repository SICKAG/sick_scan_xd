#
# Python usage example for sick_scan_api
#
# Make sure that libsick_scan_xd_api_lib.so is included in the system path, f.e. by
# python3 sick_scan_xd_api_test.py

import datetime
import numpy as np
import os
import sys
import threading
import time
from sick_scan_api import *

# set __ROS_VERSION to 0 (no ROS), 1 (publish ROS-1 pointclouds), or 2 (ROS-2)
__ROS_VERSION = os.getenv("ROS_VERSION")
if __ROS_VERSION is None:
    __ROS_VERSION = 0
else:
    __ROS_VERSION = int(__ROS_VERSION)

plt_enabled = False
if __ROS_VERSION == 0:
    try:
        from mpl_toolkits import mplot3d
        import matplotlib.pyplot as plt
        plt_enabled = True
    except ModuleNotFoundError:
        print("import matplotlib failed, visualization disabled")
elif __ROS_VERSION == 1:
    import rospy
    from sick_scan_api_converter import *
elif __ROS_VERSION == 2:
    import rclpy
    from rclpy.node import Node
    from sick_scan_api_converter import *

# global settings
class ApiTestSettings:
    def __init__(self):
        self.cartesian_pointcloud_timestamp_published = datetime.datetime.now()
        self.polar_pointcloud_timestamp_published = datetime.datetime.now()
        self.ros_pointcloud_publisher = None
        self.ros_polar_pointcloud_publisher = None
        self.ros_polar_pointcloud_is_multi_segment_scanner = False
        self.ros_visualizationmarker_publisher = None
        self.plot_figure = None
        self.plot_axes = None
        self.plot_points_x = []
        self.plot_points_y = []
        self.plot_points_z = []
        self.polling = False

# Convert a SickScanCartesianPointCloudMsg to 3D arrays
def pySickScanCartesianPointCloudMsgToXYZ(pointcloud_msg):
    # get pointcloud fields
    num_fields = pointcloud_msg.fields.size
    msg_fields_buffer = pointcloud_msg.fields.buffer
    field_offset_x = -1
    field_offset_y = -1
    field_offset_z = -1
    for n in range(num_fields):
        field_name = ctypesCharArrayToString(msg_fields_buffer[n].name)
        field_offset = msg_fields_buffer[n].offset
        if field_name == "x":
            field_offset_x = msg_fields_buffer[n].offset
        elif field_name == "y":
            field_offset_y = msg_fields_buffer[n].offset
        elif field_name == "z":
            field_offset_z = msg_fields_buffer[n].offset
    # Extract x,y,z
    cloud_data_buffer_len = (pointcloud_msg.row_step * pointcloud_msg.height) # length of polar cloud data in byte
    assert(pointcloud_msg.data.size == cloud_data_buffer_len and field_offset_x >= 0 and field_offset_y >= 0 and field_offset_z >= 0)
    cloud_data_buffer = bytearray(cloud_data_buffer_len)
    for n in range(cloud_data_buffer_len):
        cloud_data_buffer[n] = pointcloud_msg.data.buffer[n]
    points_x = np.zeros(pointcloud_msg.width * pointcloud_msg.height, dtype = np.float32)
    points_y = np.zeros(pointcloud_msg.width * pointcloud_msg.height, dtype = np.float32)
    points_z = np.zeros(pointcloud_msg.width * pointcloud_msg.height, dtype = np.float32)
    point_idx = 0
    for row_idx in range(pointcloud_msg.height):
        for col_idx in range(pointcloud_msg.width):
            # Get lidar point in polar coordinates (range, azimuth and elevation)
            pointcloud_offset = row_idx * pointcloud_msg.row_step + col_idx * pointcloud_msg.point_step
            points_x[point_idx] = np.frombuffer(cloud_data_buffer, dtype = np.float32, count = 1, offset = pointcloud_offset + field_offset_x)[0]
            points_y[point_idx] = np.frombuffer(cloud_data_buffer, dtype = np.float32, count = 1, offset = pointcloud_offset + field_offset_y)[0]
            points_z[point_idx] = np.frombuffer(cloud_data_buffer, dtype = np.float32, count = 1, offset = pointcloud_offset + field_offset_z)[0]
            point_idx = point_idx + 1
    return points_x, points_y, points_z

#
# Python examples for SickScanApi-callbacks
#

# Callback for cartesian pointcloud messages
def pySickScanCartesianPointCloudMsgCallback(api_handle, pointcloud_msg):
    pointcloud_msg = pointcloud_msg.contents # dereference msg pointer (pointcloud_msg = pointcloud_msg[0])
    print("pySickScanCartesianPointCloudMsgCallback: api_handle={}, {}x{} pointcloud, {} echo(s), segment {}".format(
        api_handle, pointcloud_msg.width, pointcloud_msg.height, pointcloud_msg.num_echos , pointcloud_msg.segment_idx))
    global api_test_settings
    # Note: Pointcloud conversion and visualization consumes cpu time, therefore we convert and publish the cartesian pointcloud with low frequency.
    cur_timestamp = datetime.datetime.now()
    if __ROS_VERSION == 0 and cur_timestamp >= api_test_settings.cartesian_pointcloud_timestamp_published + datetime.timedelta(seconds=0.5):
        api_test_settings.plot_points_x, api_test_settings.plot_points_y, api_test_settings.plot_points_z = pySickScanCartesianPointCloudMsgToXYZ(pointcloud_msg)
        api_test_settings.cartesian_pointcloud_timestamp_published = cur_timestamp
    elif __ROS_VERSION > 0 and api_test_settings.ros_pointcloud_publisher is not None and pointcloud_msg.width > 0 and pointcloud_msg.height > 0 and cur_timestamp >= api_test_settings.cartesian_pointcloud_timestamp_published + datetime.timedelta(seconds=0.2):
        # Copy cartesian pointcloud_msg to cartesian ros pointcloud and publish
        ros_pointcloud = SickScanApiConvertPointCloudToROS(pointcloud_msg)
        api_test_settings.ros_pointcloud_publisher.publish(ros_pointcloud)
        api_test_settings.cartesian_pointcloud_timestamp_published = cur_timestamp

# Callback for polar pointcloud messages
def pySickScanPolarPointCloudMsgCallback(api_handle, pointcloud_msg):
    pointcloud_msg = pointcloud_msg.contents # dereference msg pointer (pointcloud_msg = pointcloud_msg[0])
    print("pySickScanPolarPointCloudMsgCallback: api_handle={}, {}x{} pointcloud, {} echo(s), segment {}".format(
        api_handle, pointcloud_msg.width, pointcloud_msg.height, pointcloud_msg.num_echos , pointcloud_msg.segment_idx))
    if __ROS_VERSION == 1:
        # Convert polar pointcloud_msg to cartesian ros pointcloud and publish.
        # Note: Pointcloud conversion from polar to cartesian is too cpu-intensive to process all segments from a Multiscan136.
        # In case of multi-segment scanners, we just publish segment with index -1 (i.e. the 360-degree pointcloud) with low frequency.
        global api_test_settings
        if pointcloud_msg.segment_idx < 0:
            api_test_settings.ros_polar_pointcloud_is_multi_segment_scanner = True
        if api_test_settings.ros_polar_pointcloud_publisher is not None and pointcloud_msg.width > 0 and pointcloud_msg.height > 0:
            cur_timestamp = datetime.datetime.now()
            publish_polar_pointcloud = False
            if api_test_settings.ros_polar_pointcloud_is_multi_segment_scanner == False and cur_timestamp >= api_test_settings.polar_pointcloud_timestamp_published + datetime.timedelta(seconds=1.0):
                publish_polar_pointcloud = True
            elif pointcloud_msg.segment_idx < 0 and cur_timestamp >= api_test_settings.polar_pointcloud_timestamp_published + datetime.timedelta(seconds=1.0):
                publish_polar_pointcloud = True
            if publish_polar_pointcloud:
                ros_pointcloud = SickScanApiConvertPolarPointCloudToROS(pointcloud_msg)
                api_test_settings.ros_polar_pointcloud_publisher.publish(ros_pointcloud)
                api_test_settings.polar_pointcloud_timestamp_published = cur_timestamp

# Callback for Imu messages
def pySickScanImuMsgCallback(api_handle, imu_msg):
    imu_msg = imu_msg.contents # dereference msg pointer
    print("pySickScanImuMsgCallback: api_handle={}, imu message: orientation=({:.8},{:.8},{:.8},{:.8}), angular_velocity=({:.8},{:.8},{:.8}), linear_acceleration=({:.8},{:.8},{:.8})".format(
        api_handle, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w, 
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.y, 
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z))

# Callback for LFErec messages
def pySickScanLFErecMsgCallback(api_handle, lferec_msg):
    lferec_msg = lferec_msg.contents # dereference msg pointer
    field_info = ""
    for field in lferec_msg.fields:
        if field.field_result_mrs == 1:
            status = "free"
        elif field.field_result_mrs == 2:
            status = "infringed"
        else:
            status = "invalid"
        field_info = field_info + ", field {}: ({},{},{},{},{})".format(field.field_index, status, field.dist_scale_factor, field.dist_scale_offset, field.angle_scale_factor, field.angle_scale_offset)
    print("pySickScanLFErecMsgCallback: api_handle={}, lferec message: {} fields{}".format(api_handle, lferec_msg.fields_number, field_info))

# Callback for LIDoutputstate messages
def pySickScanLIDoutputstateMsgCallback(api_handle, lidoutputstate_msg):
    lidoutputstate_msg = lidoutputstate_msg.contents # dereference msg pointer
    state_info = "({},{},{},{},{},{},{},{})".format(lidoutputstate_msg.output_state[0], lidoutputstate_msg.output_state[1], lidoutputstate_msg.output_state[2], lidoutputstate_msg.output_state[3], 
        lidoutputstate_msg.output_state[4], lidoutputstate_msg.output_state[5], lidoutputstate_msg.output_state[6], lidoutputstate_msg.output_state[7])
    count_info = "({},{},{},{},{},{},{},{})".format(lidoutputstate_msg.output_count[0], lidoutputstate_msg.output_count[1], lidoutputstate_msg.output_count[2], lidoutputstate_msg.output_count[3], 
        lidoutputstate_msg.output_count[4], lidoutputstate_msg.output_count[5], lidoutputstate_msg.output_count[6], lidoutputstate_msg.output_count[7])
    print("pySickScanLIDoutputstateMsgCallback: api_handle={}, lidoutputstate message, outputstate={}, outputcount={}".format(api_handle, state_info, count_info))

# Callback for RadarScan messages
def pySickScanRadarScanCallback(api_handle, radarscan_msg):
    radarscan_msg = radarscan_msg.contents # dereference msg pointer
    print("pySickScanRadarScanCallback: api_handle={}, radarscan message: {} targets, {} objects".format(api_handle, radarscan_msg.targets.width, radarscan_msg.objects.size))
    if __ROS_VERSION == 1:
        # Copy radar target pointcloud_msg to ros pointcloud and publish
        global api_test_settings
        if api_test_settings.ros_pointcloud_publisher is not None and radarscan_msg.targets.width > 0 and radarscan_msg.targets.height > 0:
            ros_pointcloud = SickScanApiConvertPointCloudToROS(radarscan_msg.targets)
            api_test_settings.ros_pointcloud_publisher.publish(ros_pointcloud)
        if api_test_settings.ros_polar_pointcloud_publisher is not None and radarscan_msg.objects.size > 0:
            ros_pointcloud = SickScanApiConvertRadarObjectsToROS(radarscan_msg.header, radarscan_msg.objects)
            api_test_settings.ros_polar_pointcloud_publisher.publish(ros_pointcloud)

# Callback for LdmrsObjectArray messages
def pySickScanLdmrsObjectArrayCallback(api_handle, ldmrsobjectarray_msg):
    ldmrsobjectarray_msg = ldmrsobjectarray_msg.contents # dereference msg pointer
    print("pySickScanLdmrsObjectArrayCallback: api_handle={}, ldmrsobjectarray message: {} objects".format(api_handle, ldmrsobjectarray_msg.objects.size))

# Callback for VisualizationMarker messages
def pySickScanVisualizationMarkerCallback(api_handle, visualizationmarker_msg):
    visualizationmarker_msg = visualizationmarker_msg.contents # dereference msg pointer
    marker_info = ""
    for n in range(visualizationmarker_msg.markers.size):
        marker = visualizationmarker_msg.markers.buffer[n]
        marker_info = marker_info + ", marker {}: pos=({},{},{})".format(marker.id, marker.pose_position.x, marker.pose_position.y, marker.pose_position.z)
    print("pySickScanVisualizationMarkerCallback: api_handle={}, visualizationmarker message: {} marker{}".format(api_handle, visualizationmarker_msg.markers.size, marker_info))
    if __ROS_VERSION > 0:
        # Copy radar target pointcloud_msg to ros pointcloud and publish
        global api_test_settings
        if api_test_settings.ros_visualizationmarker_publisher is not None and visualizationmarker_msg.markers.size > 0:
            ros_markers = SickScanApiConvertMarkerArrayToROS(visualizationmarker_msg.markers)
            api_test_settings.ros_visualizationmarker_publisher.publish(ros_markers)

#
# Python examples for SickScanApiWaitNext-functions ("message polling")
#

# Receive lidar message by SickScanApiWaitNext-functions ("message polling")
def pyrunSickScanApiTestWaitNext(sick_scan_library, api_handle):
    wait_next_message_timeout = 0.1 # wait max. 0.1 seconds for the next message (otherwise SickScanApiWaitNext-function return with timeout)
    pointcloud_msg = SickScanPointCloudMsg()
    imu_msg = SickScanImuMsg()
    lferec_msg = SickScanLFErecMsg()
    lidoutputstate_msg = SickScanLIDoutputstateMsg()
    radarscan_msg = SickScanRadarScan()
    ldmrsobjectarray_msg = SickScanLdmrsObjectArray()
    visualizationmarker_msg = SickScanVisualizationMarkerMsg()
    global api_test_settings
    while api_test_settings.polling:
        # Get/poll the next cartesian PointCloud message
        ret = SickScanApiWaitNextCartesianPointCloudMsg(sick_scan_library, api_handle, ctypes.pointer(pointcloud_msg), wait_next_message_timeout)
        if ret == int(SickScanApiErrorCodes.SICK_SCAN_API_SUCCESS):
            pySickScanCartesianPointCloudMsgCallback(api_handle, ctypes.pointer(pointcloud_msg))
        elif ret != int(SickScanApiErrorCodes.SICK_SCAN_API_SUCCESS) and ret != int(SickScanApiErrorCodes.SICK_SCAN_API_TIMEOUT):
            print("## ERROR pyrunSickScanApiTestWaitNext: SickScanApiWaitNextCartesianPointCloudMsg failed, error code {} ({})".format(ret, int(ret)))
        SickScanApiFreePointCloudMsg(sick_scan_library, api_handle, ctypes.pointer(pointcloud_msg))
        # Get/poll the next polar PointCloud message
        ret = SickScanApiWaitNextPolarPointCloudMsg(sick_scan_library, api_handle, ctypes.pointer(pointcloud_msg), wait_next_message_timeout)
        if ret == int(SickScanApiErrorCodes.SICK_SCAN_API_SUCCESS):
            pySickScanPolarPointCloudMsgCallback(api_handle, ctypes.pointer(pointcloud_msg))
        elif ret != int(SickScanApiErrorCodes.SICK_SCAN_API_SUCCESS) and ret != int(SickScanApiErrorCodes.SICK_SCAN_API_TIMEOUT):
            print("## ERROR pyrunSickScanApiTestWaitNext: SickScanApiWaitNextPolarPointCloudMsg failed, error code {} ({})".format(ret, int(ret)))
        SickScanApiFreePointCloudMsg(sick_scan_library, api_handle, ctypes.pointer(pointcloud_msg))
        # Get/poll the next Imu message
        ret = SickScanApiWaitNextImuMsg(sick_scan_library, api_handle, ctypes.pointer(imu_msg), wait_next_message_timeout)
        if ret == int(SickScanApiErrorCodes.SICK_SCAN_API_SUCCESS):
            pySickScanImuMsgCallback(api_handle, ctypes.pointer(imu_msg))
        elif ret != int(SickScanApiErrorCodes.SICK_SCAN_API_SUCCESS) and ret != int(SickScanApiErrorCodes.SICK_SCAN_API_TIMEOUT):
            print("## ERROR pyrunSickScanApiTestWaitNext: SickScanApiWaitNextImuMsg failed, error code {} ({})".format(ret, int(ret)))
        SickScanApiFreeImuMsg(sick_scan_library, api_handle, ctypes.pointer(imu_msg))
        # Get/poll the next LFErec message
        ret = SickScanApiWaitNextLFErecMsg(sick_scan_library, api_handle, ctypes.pointer(lferec_msg), wait_next_message_timeout)
        if ret == int(SickScanApiErrorCodes.SICK_SCAN_API_SUCCESS):
            pySickScanLFErecMsgCallback(api_handle, ctypes.pointer(lferec_msg))
        elif ret != int(SickScanApiErrorCodes.SICK_SCAN_API_SUCCESS) and ret != int(SickScanApiErrorCodes.SICK_SCAN_API_TIMEOUT):
            print("## ERROR pyrunSickScanApiTestWaitNext: SickScanApiWaitNextLFErecMsg failed, error code {} ({})".format(ret, int(ret)))
        SickScanApiFreeLFErecMsg(sick_scan_library, api_handle, ctypes.pointer(lferec_msg))
        # Get/poll the next LIDoutputstate message
        ret = SickScanApiWaitNextLIDoutputstateMsg(sick_scan_library, api_handle, ctypes.pointer(lidoutputstate_msg), wait_next_message_timeout)
        if ret == int(SickScanApiErrorCodes.SICK_SCAN_API_SUCCESS):
            pySickScanLIDoutputstateMsgCallback(api_handle, ctypes.pointer(lidoutputstate_msg))
        elif ret != int(SickScanApiErrorCodes.SICK_SCAN_API_SUCCESS) and ret != int(SickScanApiErrorCodes.SICK_SCAN_API_TIMEOUT):
            print("## ERROR pyrunSickScanApiTestWaitNext: SickScanApiWaitNextLIDoutputstateMsg failed, error code {} ({})".format(ret, int(ret)))
        SickScanApiFreeLIDoutputstateMsg(sick_scan_library, api_handle, ctypes.pointer(lidoutputstate_msg))
        # Get/poll the next RadarScan message
        ret = SickScanApiWaitNextRadarScanMsg(sick_scan_library, api_handle, ctypes.pointer(radarscan_msg), wait_next_message_timeout)
        if ret == int(SickScanApiErrorCodes.SICK_SCAN_API_SUCCESS):
            pySickScanRadarScanCallback(api_handle, ctypes.pointer(radarscan_msg))
        elif ret != int(SickScanApiErrorCodes.SICK_SCAN_API_SUCCESS) and ret != int(SickScanApiErrorCodes.SICK_SCAN_API_TIMEOUT):
            print("## ERROR pyrunSickScanApiTestWaitNext: SickScanApiWaitNextRadarScanMsg failed, error code {} ({})".format(ret, int(ret)))
        SickScanApiFreeRadarScanMsg(sick_scan_library, api_handle, ctypes.pointer(radarscan_msg))
        # Get/poll the next LdmrsObjectArray message
        ret = SickScanApiWaitNextLdmrsObjectArrayMsg(sick_scan_library, api_handle, ctypes.pointer(ldmrsobjectarray_msg), wait_next_message_timeout)
        if ret == int(SickScanApiErrorCodes.SICK_SCAN_API_SUCCESS):
            pySickScanLdmrsObjectArrayCallback(api_handle, ctypes.pointer(ldmrsobjectarray_msg))
        elif ret != int(SickScanApiErrorCodes.SICK_SCAN_API_SUCCESS) and ret != int(SickScanApiErrorCodes.SICK_SCAN_API_TIMEOUT):
            print("## ERROR pyrunSickScanApiTestWaitNext: SickScanApiWaitNextLdmrsObjectArrayMsg failed, error code {} ({})".format(ret, int(ret)))
        SickScanApiFreeLdmrsObjectArrayMsg(sick_scan_library, api_handle, ctypes.pointer(ldmrsobjectarray_msg))
        # Get/poll the next VisualizationMarker message
        ret = SickScanApiWaitNextVisualizationMarkerMsg(sick_scan_library, api_handle, ctypes.pointer(visualizationmarker_msg), wait_next_message_timeout)
        if ret == int(SickScanApiErrorCodes.SICK_SCAN_API_SUCCESS):
            pySickScanVisualizationMarkerCallback(api_handle, ctypes.pointer(visualizationmarker_msg))
        elif ret != int(SickScanApiErrorCodes.SICK_SCAN_API_SUCCESS) and ret != int(SickScanApiErrorCodes.SICK_SCAN_API_TIMEOUT):
            print("## ERROR pyrunSickScanApiTestWaitNext: SickScanApiWaitNextVisualizationMarkerMsg failed, error code {} ({})".format(ret, int(ret)))
        SickScanApiFreeVisualizationMarkerMsg(sick_scan_library, api_handle, ctypes.pointer(visualizationmarker_msg))

#
# Python usage example for sick_scan_api
#

if __name__ == "__main__":

    # Configuration and commandline arguments
    api_test_settings = ApiTestSettings()
    cli_arg_start_idx = 1
    for n, cli_arg in enumerate(sys.argv):
        if cli_arg.startswith("_polling:="):
            cli_arg_start_idx = n + 1
            if int(cli_arg[10:]) > 0:
                api_test_settings.polling = True
    cli_args = " ".join(sys.argv[cli_arg_start_idx:])
    if __ROS_VERSION == 0 and plt_enabled:
        api_test_settings.plot_figure = plt.figure()
        api_test_settings.plot_axes = plt.axes(projection="3d")
    elif __ROS_VERSION == 1:
        rospy.init_node("sick_scan_api_test_py")
        api_test_settings.ros_pointcloud_publisher = rospy.Publisher("/sick_scan_xd_api_test/api_cloud", PointCloud2, queue_size=10)
        api_test_settings.ros_polar_pointcloud_publisher = rospy.Publisher("/sick_scan_xd_api_test/api_cloud_polar", PointCloud2, queue_size=10)
        api_test_settings.ros_visualizationmarker_publisher = rospy.Publisher("/sick_scan_xd_api_test/marker", MarkerArray, queue_size=10)
    elif __ROS_VERSION == 2:
        rclpy.init()
        ros_node = Node("sick_scan_api_test_py")
        api_test_settings.ros_pointcloud_publisher = ros_node.create_publisher(PointCloud2, "/sick_scan_xd_api_test/api_cloud", 10)
        api_test_settings.ros_visualizationmarker_publisher = ros_node.create_publisher(MarkerArray, "/sick_scan_xd_api_test/marker", 10)

    # Load sick_scan_library
    if os.name == 'nt': # Load windows dll
        sick_scan_library = SickScanApiLoadLibrary(["build/Debug/", "build_win64/Debug/", "src/build/Debug/", "src/build_win64/Debug/", "src/sick_scan_xd/build/Debug/", "src/sick_scan_xd/build_win64/Debug/", "./", "../"], "sick_scan_shared_lib.dll")
    else: # Load linux so
        sick_scan_library = SickScanApiLoadLibrary(["build/", "build_linux/", "src/build/", "src/build_linux/", "src/sick_scan_xd/build/", "src/sick_scan_xd/build_linux/", "./", "../"], "libsick_scan_shared_lib.so")
    api_handle = SickScanApiCreate(sick_scan_library)

    # Initialize lidar by launchfile, e.g. sick_tim_7xx.launch
    # SickScanApiInitByLaunchfile(sick_scan_library, api_handle, "./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False")
    print("sick_scan_xd_api_test.py: initializing lidar, commandline arguments = \"{}\"".format(cli_args))
    SickScanApiInitByLaunchfile(sick_scan_library, api_handle, cli_args)

    api_test_wait_next_thread = None
    if api_test_settings.polling: # Receive lidar message by SickScanApiWaitNext-functions running in a background thread ("message polling")
        api_test_wait_next_thread = threading.Thread(target=pyrunSickScanApiTestWaitNext, args=(sick_scan_library, api_handle))
        api_test_wait_next_thread.start()

    else: # Register callbacks to receive lidar messages (recommended)

        # Register a callback for PointCloud messages
        cartesian_pointcloud_callback = SickScanPointCloudMsgCallback(pySickScanCartesianPointCloudMsgCallback)
        SickScanApiRegisterCartesianPointCloudMsg(sick_scan_library, api_handle, cartesian_pointcloud_callback)
        polar_pointcloud_callback = SickScanPointCloudMsgCallback(pySickScanPolarPointCloudMsgCallback)
        SickScanApiRegisterPolarPointCloudMsg(sick_scan_library, api_handle, polar_pointcloud_callback)

        # Register a callback for Imu messages
        imu_callback = SickScanImuMsgCallback(pySickScanImuMsgCallback)
        SickScanApiRegisterImuMsg(sick_scan_library, api_handle, imu_callback)

        # Register a callback for LFErec messages
        lferec_callback = SickScanLFErecMsgCallback(pySickScanLFErecMsgCallback)
        SickScanApiRegisterLFErecMsg(sick_scan_library, api_handle, lferec_callback)

        # Register a callback for LIDoutputstate messages
        lidoutputstate_callback = SickScanLIDoutputstateMsgCallback(pySickScanLIDoutputstateMsgCallback)
        SickScanApiRegisterLIDoutputstateMsg(sick_scan_library, api_handle, lidoutputstate_callback)

        # Register a callback for RadarScan messages
        radarscan_callback = SickScanRadarScanCallback(pySickScanRadarScanCallback)
        SickScanApiRegisterRadarScanMsg(sick_scan_library, api_handle, radarscan_callback)

        # Register a callback for LdmrsObjectArray messages
        ldmrsobjectarray_callback = SickScanLdmrsObjectArrayCallback(pySickScanLdmrsObjectArrayCallback)
        SickScanApiRegisterLdmrsObjectArrayMsg(sick_scan_library, api_handle, ldmrsobjectarray_callback)

        # Register a callback for VisualizationMarker messages
        visualizationmarker_callback = SickScanVisualizationMarkerCallback(pySickScanVisualizationMarkerCallback)
        SickScanApiRegisterVisualizationMarkerMsg(sick_scan_library, api_handle, visualizationmarker_callback)

    # Run main loop
    if __ROS_VERSION == 0:
        while True:
            try:
                if plt_enabled and len(api_test_settings.plot_points_x) > 0 and len(api_test_settings.plot_points_y) > 0  and len(api_test_settings.plot_points_z) > 0:
                    print("sick_scan_xd_api_test.py plotting pointcloud by matplotlib")
                    plot_points_x = np.copy(api_test_settings.plot_points_x)
                    plot_points_y = np.copy(api_test_settings.plot_points_y)
                    plot_points_z = np.copy(api_test_settings.plot_points_z)
                    # Depending on the system, it can be recommended to close all figures instead of just clearing.
                    plt.close("all")
                    api_test_settings.plot_figure = plt.figure()
                    api_test_settings.plot_axes = plt.axes(projection="3d")                   
                    # api_test_settings.plot_axes.clear()
                    api_test_settings.plot_axes.scatter(plot_points_x, plot_points_y, plot_points_z, c='r', marker='.')
                    api_test_settings.plot_axes.set_xlabel("x")
                    api_test_settings.plot_axes.set_ylabel("y")
                    plt.draw()
                    plt.pause(2.0)
                else:
                    time.sleep(0.1)
            except:
                break
    elif __ROS_VERSION == 1:
        # rospy.spin()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
    else:
        user_key = 0
        while user_key != "\n":
            time.sleep(1)
            print("sick_scan_xd_api_test.py running. Press ENTER to exit")
            user_key = sys.stdin.read(1)

    # Stop lidar, close connection and api handle
    print("sick_scan_xd_api_test.py finishing...")
    if api_test_settings.polling:
        api_test_settings.polling = False
        if api_test_wait_next_thread is not None:
            api_test_wait_next_thread.join()
    else:
        SickScanApiDeregisterCartesianPointCloudMsg(sick_scan_library, api_handle, cartesian_pointcloud_callback)
        SickScanApiDeregisterPolarPointCloudMsg(sick_scan_library, api_handle, polar_pointcloud_callback)
        SickScanApiDeregisterImuMsg(sick_scan_library, api_handle, imu_callback)
        SickScanApiDeregisterLFErecMsg(sick_scan_library, api_handle, lferec_callback)
        SickScanApiDeregisterLIDoutputstateMsg(sick_scan_library, api_handle, lidoutputstate_callback)
        SickScanApiDeregisterRadarScanMsg(sick_scan_library, api_handle, radarscan_callback)
        SickScanApiDeregisterLdmrsObjectArrayMsg(sick_scan_library, api_handle, ldmrsobjectarray_callback)
        SickScanApiDeregisterVisualizationMarkerMsg(sick_scan_library, api_handle, visualizationmarker_callback)
    SickScanApiClose(sick_scan_library, api_handle)
    SickScanApiRelease(sick_scan_library, api_handle)
    SickScanApiUnloadLibrary(sick_scan_library)
    print("sick_scan_xd_api_test.py finished.")
