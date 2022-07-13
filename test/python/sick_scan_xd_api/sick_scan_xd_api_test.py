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
    ros_pointcloud_publisher = None

def pySickScanCartesianPointCloudMsgCallback(api_handle, pointcloud_msg):
    print("pySickScanCartesianPointCloudMsgCallback: api_handle={}, {}x{} pointcloud".format(api_handle, pointcloud_msg[0].width, pointcloud_msg[0].height))
    if __ROS_VERSION == 1:
        if ros_pointcloud_publisher is not None and pointcloud_msg[0].width > 0 and pointcloud_msg[0].height > 0:
            ros_pointcloud = SickScanApiConvertPointCloudToROS1(pointcloud_msg[0])
            ros_pointcloud_publisher.publish(ros_pointcloud)

# Python usage example for sick_scan_api
if __name__ == "__main__":

    if __ROS_VERSION == 1:
        rospy.init_node("sick_scan_api_test_py")
        ros_pointcloud_publisher = rospy.Publisher("/sick_scan_xd_api_test/api_cloud", PointCloud2, queue_size=10)

    # Load sick_scan_library
    sick_scan_library = SickScanApiLoadLibrary(["build/", "build_linux/", "src/build/", "src/build_linux/", "src/sick_scan_xd/build/", "src/sick_scan_xd/build_linux/", "./", "../"], "libsick_scan_shared_lib.so")
    api_handle = SickScanApiCreate(sick_scan_library)

    # Initialize lidar by launchfile, e.g. sick_tim_7xx.launch
    # SickScanApiInitByLaunchfile(sick_scan_library, api_handle, "./src/sick_scan_xd/launch/sick_tim_7xx.launch hostname:=127.0.0.1 port:=2111 sw_pll_only_publish:=False")
    cli_args = " ".join(sys.argv[1:])
    print("sick_scan_xd_api_test.py: initializing lidar, commandline arguments = \"{}\"".format(cli_args))
    SickScanApiInitByLaunchfile(sick_scan_library, api_handle, cli_args)

    # Register a callback for PointCloud messages
    pointcloud_callback = SickScanPointCloudMsgCallback(pySickScanCartesianPointCloudMsgCallback)
    SickScanApiRegisterCartesianPointCloudMsg(sick_scan_library, api_handle, pointcloud_callback)
    
    # Run main loop
    user_key = 0
    while user_key != "\n":
        print("sick_scan_xd_api_test.py running. Press ENTER to exit")
        user_key = sys.stdin.read(1)
