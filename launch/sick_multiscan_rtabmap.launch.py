import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    ld = LaunchDescription()
    sick_scan_pkg_prefix = get_package_share_directory('sick_scan_xd')
    sick_launch_file_path = os.path.join(sick_scan_pkg_prefix, 'launch/sick_multiscan_rtabmap.launch')
    sick_node_arguments=[sick_launch_file_path]
    
    # Append optional commandline arguments in name:=value syntax
    for arg in sys.argv:
        if len(arg.split(":=")) == 2:
            sick_node_arguments.append(arg)
    # Create sick_scan_xd node
    sick_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        output='screen',
        arguments=sick_node_arguments
    )

    # Common parameter
    wait_for_transform = 0.01
    point_cloud_topic = "/cloud_all_fields_fullframe" # Point cloud topic published by sick_scan_xd
    point_cloud_frame_id = "cloud"                    # Point cloud frame id published by sick_scan_xd
    imu_topic = "/multiScan/imu"                      # IMU topic published by sick_scan_xd
    deskewing = True                                  # Optional lidar deskewing

    if deskewing: # Create deskewing node
        point_cloud_topic_desk = f"{point_cloud_topic}/deskewed"
        deskewing_node = Node(
            package='rtabmap_util', executable='lidar_deskewing', name="lidar_deskewing", output="screen",
            parameters=[{
                "wait_for_transform": wait_for_transform,
                "fixed_frame_id": point_cloud_frame_id,
                "slerp": False,
             }],
            remappings=[("input_cloud", point_cloud_topic)],
            arguments=[],
            namespace="deskewing")
    else:
        point_cloud_topic_desk = point_cloud_topic
        deskewing_node = None

    # Create rtabmap_odom node
    rtabmap_odom_node = Node(
            package='rtabmap_odom', executable='icp_odometry', name="icp_odometry", output="screen",
            parameters=[{
                "frame_id": point_cloud_frame_id,
                "odom_frame_id": "odom",
                "guess_frame_id": point_cloud_frame_id,
                "wait_imu_to_init": True,
                "wait_for_transform_duration": str(wait_for_transform),
                "Icp/PointToPlane": "True",
                "Icp/Iterations": "10",
                "Icp/VoxelSize": "0.2",
                "Icp/DownsamplingStep": "1",
                "Icp/Epsilon": "0.001",
                "Icp/PointToPlaneK": "20",
                "Icp/PointToPlaneRadius": "0",
                "Icp/MaxTranslation": "2",
                "Icp/MaxCorrespondenceDistance": "1",
                "Icp/PM": "True",
                "Icp/PMOutlierRatio": "0.1",
                "Icp/CorrespondenceRatio": "0.01",
                "Icp/ReciprocalCorrespondences": "False",
                "Odom/ScanKeyFrameThr": "0.8",
                "Odom/Strategy": "0",
                "OdomF2M/ScanSubtractRadius": "0.2",
                "OdomF2M/ScanMaxSize": "15000",
            }],
            remappings=[("scan_cloud", point_cloud_topic_desk), ("imu", imu_topic)],
            arguments=[],
            namespace="rtabmap")


    # Create rtabmap_slam node, see https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_launch/launch/rtabmap.launch.py for an example configuration
    rtabmap_slam_node = Node(
            package='rtabmap_slam', executable='rtabmap', name="rtabmap", output="screen",
            parameters=[{
                "frame_id": point_cloud_frame_id,
                "subscribe_depth": False,
                "subscribe_rgb": False,
                "subscribe_rgbd": False,
                "subscribe_scan": False,
                "subscribe_scan_cloud": True,
                "approx_sync": True,
                "Rtabmap/DetectionRate": "1",
                "RGBD/NeighborLinkRefining": "False",
                "RGBD/ProximityBySpace": "True",
                "RGBD/ProximityMaxGraphDepth": "0",
                "RGBD/ProximityPathMaxNeighbors": "1",
                "RGBD/AngularUpdate": "0.05",
                "RGBD/LinearUpdate": "0.05",
                "Mem/NotLinkedNodesKept": "False",
                "Mem/STMSize": "30",
                "Reg/Strategy": "1",
                "Grid/CellSize": "0.1",
                "Grid/RangeMax": "20",
                "Grid/ClusterRadius": "1",
                "Grid/GroundIsObstacle": "True",
                "Optimizer/GravitySigma": "0.3",
                "Icp/VoxelSize": "0.3",
                "Icp/PointToPlaneK": "20",
                "Icp/PointToPlaneRadius": "0",
                "Icp/PointToPlane": "False",
                "Icp/Iterations": "10",
                "Icp/Epsilon": "0.001",
                "Icp/MaxTranslation": "3",
                "Icp/MaxCorrespondenceDistance": "1",
                "Icp/PM": "True",
                "Icp/PMOutlierRatio": "0.7",
                "Icp/CorrespondenceRatio": "0.4",
            }],
            remappings=[("scan_cloud", point_cloud_topic_desk), ("imu", imu_topic)],
            arguments=["-d"], # parameter "-d": delete_db_on_start
            namespace="rtabmap")

    # Create rtabmap_viz node
    rtabmap_viz_node = Node(
            package='rtabmap_viz', executable='rtabmap_viz', name="rtabmap_viz", output="screen",
            parameters=[{
                "frame_id": point_cloud_frame_id,
                "odom_frame_id": "odom",
                "subscribe_odom_info": False,
                "subscribe_scan_cloud": True,
                "approx_sync": False,
             }],
            remappings=[("scan_cloud", point_cloud_topic_desk)],
            arguments=[],
            namespace="rtabmap")

    ld.add_action(sick_node)
    if deskewing:
        ld.add_action(deskewing_node)
    ld.add_action(rtabmap_odom_node)
    ld.add_action(rtabmap_slam_node)
    ld.add_action(rtabmap_viz_node)
    return ld

