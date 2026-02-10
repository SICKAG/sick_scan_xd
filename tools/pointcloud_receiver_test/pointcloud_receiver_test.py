#!/usr/bin/env python3
##
# @file pc2_to_csv.py
# @brief Receive a ROS 2 PointCloud2 message and dump the first received cloud to a CSV file.
#
# This script subscribes to a PointCloud2 topic, writes the contents of the
# first received message to a CSV file, and then terminates.
#
# The CSV header is derived directly from the PointCloud2 field definitions,
# making the output compatible with arbitrary PointCloud2 layouts.
#
# Typical use cases:
# - Debugging structured or unstructured LiDAR data
# - Offline analysis of point clouds
# - Exporting ROS PointCloud2 data to external tools
#
# @note
# - Only the first received PointCloud2 message is written.
# - NaN points are skipped.
# - The node shuts down automatically after writing the file.
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import csv
import sys


##
# @class PointCloudToCSV
# @brief ROS 2 node that converts a PointCloud2 message to a CSV file.
#
# The node subscribes to a PointCloud2 topic and writes the first received
# point cloud to a CSV file named `cloud_structured.csv`.
#
# Each row in the CSV corresponds to one point, and each column corresponds
# to a PointCloud2 field (e.g. x, y, z, intensity).
#
class PointCloudToCSV(Node):
    ##
    # @brief Constructor.
    #
    # Initializes the ROS 2 node and sets up the PointCloud2 subscription.
    #
    def __init__(self):
        super().__init__('pc2_to_csv')

        ## @brief Subscription to the PointCloud2 topic.
        self.sub = self.create_subscription(
            PointCloud2,
            '/cloud_structured',   # Adjust topic name if necessary
            self.callback,
            10
        )

        ## @brief Flag to ensure only the first message is processed.
        self.received = False

    ##
    # @brief Callback function for PointCloud2 messages.
    #
    # This method is invoked whenever a PointCloud2 message is received.
    # The first received message is written to a CSV file; subsequent messages
    # are ignored.
    #
    # @param msg The received PointCloud2 message.
    #
    def callback(self, msg: PointCloud2):
        if self.received:
            return

        self.get_logger().info('PointCloud2 received, writing CSV...')
        self.received = True

        ## Extract field names for CSV header
        fields = [f.name for f in msg.fields]

        ## Write point cloud data to CSV
        with open('cloud_structured.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(fields)

            for p in pc2.read_points(msg, skip_nans=True):
                writer.writerow(p)

        self.get_logger().info(
            f'Wrote {msg.width * msg.height} points to cloud_structured.csv'
        )

        ## Shut down ROS after the first cloud is written
        rclpy.shutdown()


##
# @brief Main entry point.
#
# Initializes the ROS 2 system, instantiates the PointCloudToCSV node,
# and spins until the node shuts down.
#
def main():
    rclpy.init()
    node = PointCloudToCSV()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
