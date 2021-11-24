# ros2_wait_for_cloud_message.py subscribes to topic "/cloud" on ROS2, receives 10 messages and exits.
# Replacement for missing "rostopic echo -n 10 /cloud" (ROS1), since ROS2 does not support parameter "-n 10" or something equivalent
# Taken from https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class CloudSubscriber(Node):

    def __init__(self):
        super().__init__('cloud_subscriber')
        self.cloud_msg_cnt = 0
        self.subscription = self.create_subscription(PointCloud2, 'cloud', self.listener_callback, 10)

    def listener_callback(self, msg):
        # self.get_logger().info('cloud_subscriber: "%s"' % msg)
        self.cloud_msg_cnt = self.cloud_msg_cnt + 1

def main(args=None):
    max_msg_cnt = 10
    rclpy.init(args=args)
    cloud_subscriber = CloudSubscriber()
    while cloud_subscriber.cloud_msg_cnt < max_msg_cnt:
        rclpy.spin_once(cloud_subscriber)
    print("CloudSubscriber: {} messages received".format(cloud_subscriber.cloud_msg_cnt))
    cloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
