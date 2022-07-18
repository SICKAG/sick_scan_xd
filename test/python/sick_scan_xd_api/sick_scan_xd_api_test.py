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

# Callback for polar pointcloud messages
def pySickScanImuMsgCallback(api_handle, imu_msg):
    imu_msg = imu_msg.contents # dereference msg pointer
    print("pySickScanImuMsgCallback: api_handle={}, imu message: orientation=({:.8},{:.8},{:.8},{:.8}), angular_velocity=({:.8},{:.8},{:.8}), linear_acceleration=({:.8},{:.8},{:.8})".format(
        api_handle, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w, 
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.y, 
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z))

# Python usage example for sick_scan_api
if __name__ == "__main__":

    api_test_settings = ApiTestSettings()
    if __ROS_VERSION == 1:
        rospy.init_node("sick_scan_api_test_py")
        api_test_settings.ros_pointcloud_publisher = rospy.Publisher("/sick_scan_xd_api_test/api_cloud", PointCloud2, queue_size=10)
        api_test_settings.ros_polar_pointcloud_publisher = rospy.Publisher("/sick_scan_xd_api_test/api_cloud_polar", PointCloud2, queue_size=10)
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
    SickScanApiClose(sick_scan_library, api_handle)
    SickScanApiRelease(sick_scan_library, api_handle)
    SickScanApiUnloadLibrary(sick_scan_library)
    print("sick_scan_xd_api_test.py finished.")
