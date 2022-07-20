#
# Python usage example for sick_scan_api
#
# Make sure that libsick_scan_xd_api_lib.so is included in the system path, f.e. by
# python3 sick_scan_xd_api_test.py

import sys
from sick_scan_api import *

__ROS_VERSION = 1 # set __ROS_VERSION to 0 (no ROS) or 1 (publish ROS-1 pointclouds)
if __ROS_VERSION == 1:
    import rospy
    from sick_scan_api_converter import *

# global settings
class ApiTestSettings:
    def __init__(self):
        self.ros_pointcloud_publisher = None
        self.ros_polar_pointcloud_publisher = None
        self.ros_polar_pointcloud_timestamp_published = None
        self.ros_polar_pointcloud_is_multi_segment_scanner = False
        self.ros_visualizationmarker_publisher = None

# Callback for cartesian pointcloud messages
def pySickScanCartesianPointCloudMsgCallback(api_handle, pointcloud_msg):
    pointcloud_msg = pointcloud_msg.contents # dereference msg pointer (pointcloud_msg = pointcloud_msg[0])
    print("pySickScanCartesianPointCloudMsgCallback: api_handle={}, {}x{} pointcloud, {} echo(s), segment {}".format(
        api_handle, pointcloud_msg.width, pointcloud_msg.height, pointcloud_msg.num_echos , pointcloud_msg.segment_idx))
    if __ROS_VERSION == 1:
        # Copy cartesian pointcloud_msg to cartesian ros pointcloud and publish
        global api_test_settings
        if api_test_settings.ros_pointcloud_publisher is not None and pointcloud_msg.width > 0 and pointcloud_msg.height > 0:
            ros_pointcloud = SickScanApiConvertPointCloudToROS1(pointcloud_msg)
            api_test_settings.ros_pointcloud_publisher.publish(ros_pointcloud)

# Callback for polar pointcloud messages
def pySickScanPolarPointCloudMsgCallback(api_handle, pointcloud_msg):
    pointcloud_msg = pointcloud_msg.contents # dereference msg pointer (pointcloud_msg = pointcloud_msg[0])
    print("pySickScanPolarPointCloudMsgCallback: api_handle={}, {}x{} pointcloud, {} echo(s), segment {}".format(
        api_handle, pointcloud_msg.width, pointcloud_msg.height, pointcloud_msg.num_echos , pointcloud_msg.segment_idx))
    if __ROS_VERSION == 1:
        # Convert polar pointcloud_msg to cartesian ros pointcloud and publish.
        # Note: Pointcloud conversion from polar to cartesian is too cpu-intensive to process all segments from a Multiscan136.
        # In case of multi-segment scanners, we just publish segment with index -1 (i.e. the 360-degree pointcloud) from time to time.
        global api_test_settings
        if pointcloud_msg.segment_idx < 0:
            api_test_settings.ros_polar_pointcloud_is_multi_segment_scanner = True
        if api_test_settings.ros_polar_pointcloud_publisher is not None and pointcloud_msg.width > 0 and pointcloud_msg.height > 0:
            cur_timestamp = rospy.Time.now()
            publish_polar_pointcloud = False
            if api_test_settings.ros_polar_pointcloud_is_multi_segment_scanner == False:
                publish_polar_pointcloud = True
            elif pointcloud_msg.segment_idx < 0 and cur_timestamp > api_test_settings.ros_polar_pointcloud_timestamp_published + rospy.Duration(0.9):
                publish_polar_pointcloud = True
            if publish_polar_pointcloud:
                ros_pointcloud = SickScanApiConvertPolarPointCloudToROS1(pointcloud_msg)
                api_test_settings.ros_polar_pointcloud_publisher.publish(ros_pointcloud)
                api_test_settings.ros_polar_pointcloud_timestamp_published = cur_timestamp

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
            ros_pointcloud = SickScanApiConvertPointCloudToROS1(radarscan_msg.targets)
            api_test_settings.ros_pointcloud_publisher.publish(ros_pointcloud)
        if api_test_settings.ros_polar_pointcloud_publisher is not None and radarscan_msg.objects.size > 0:
            ros_pointcloud = SickScanApiConvertRadarObjectsToROS1(radarscan_msg.header, radarscan_msg.objects)
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
    if __ROS_VERSION == 1:
        # Copy radar target pointcloud_msg to ros pointcloud and publish
        global api_test_settings
        if api_test_settings.ros_visualizationmarker_publisher is not None and visualizationmarker_msg.markers.size > 0:
            ros_markers = SickScanApiConvertMarkerArrayToROS1(visualizationmarker_msg.markers)
            api_test_settings.ros_visualizationmarker_publisher.publish(ros_markers)

# Python usage example for sick_scan_api
if __name__ == "__main__":

    api_test_settings = ApiTestSettings()
    if __ROS_VERSION == 1:
        rospy.init_node("sick_scan_api_test_py")
        api_test_settings.ros_pointcloud_publisher = rospy.Publisher("/sick_scan_xd_api_test/api_cloud", PointCloud2, queue_size=10)
        api_test_settings.ros_polar_pointcloud_publisher = rospy.Publisher("/sick_scan_xd_api_test/api_cloud_polar", PointCloud2, queue_size=10)
        api_test_settings.ros_visualizationmarker_publisher = rospy.Publisher("/sick_scan_xd_api_test/marker", MarkerArray, queue_size=10)
        api_test_settings.ros_polar_pointcloud_timestamp_published = rospy.Time.now()

    # Load sick_scan_library
    sick_scan_library = SickScanApiLoadLibrary(["build/", "build_linux/", "src/build/", "src/build_linux/", "src/sick_scan_xd/build/", "src/sick_scan_xd/build_linux/", "./", "../"], "libsick_scan_shared_lib.so")
    api_handle = SickScanApiCreate(sick_scan_library)

    # Initialize lidar by launchfile, e.g. sick_tim_7xx.launch
    # SickScanApiInitByLaunchfile(sick_scan_library, api_handle, "./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False")
    cli_args = " ".join(sys.argv[1:])
    print("sick_scan_xd_api_test.py: initializing lidar, commandline arguments = \"{}\"".format(cli_args))
    SickScanApiInitByLaunchfile(sick_scan_library, api_handle, cli_args)

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
    if __ROS_VERSION == 1:
        # rospy.spin()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
    else:
        user_key = 0
        while user_key != "\n":
            print("sick_scan_xd_api_test.py running. Press ENTER to exit")
            user_key = sys.stdin.read(1)

    # Stop lidar, close connection and api handle
    print("sick_scan_xd_api_test.py finishing...")
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
