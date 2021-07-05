import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('sick_scan'),
        'config',
        'test_server_ldmrs.yaml'
        )
    node=Node(
        package='sick_scan',
        name = 'sick_test_server',
        node_executable='test_server',
        # executable='test_server',
        output='screen',
        parameters = [config]
    )
    ld.add_action(node)
    return ld
