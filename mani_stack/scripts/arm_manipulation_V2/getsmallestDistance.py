#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point32
from sensor_msgs_py import point_cloud2
from time import time  # Import time module
from mani_stack.srv import Manipulation
def convert_pointcloud2_to_xyz_array(msg):
  """Converts a PointCloud2 message to an array of dictionaries containing {x, y, z} coordinates."""
  points = []
  min_x = float('inf')
  for point in point_cloud2.read_points(msg, skip_nans=True):  # Skip NaN points
    points.append({'x': point[0], 'y': point[1], 'z': point[2]})
    x_value = point[2]  # Access the x-coordinate
    if x_value < min_x:
        min_x = x_value
    # print(point)
    # print("done")
    # exit(0)
  return points,min_x

class PointCloudConverter(Node):

    def __init__(self):
        super().__init__('pointcloud_converter')
        self.point_cloud_subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',  # Replace with your point cloud topic name
            self.point_cloud_callback,
            10)

    def point_cloud_callback(self, msg):
        self.start_time = time()
        xyz_array,minDistance = convert_pointcloud2_to_xyz_array(msg)
        # Do something with the xyz_array (e.g., print, store, process)
        total_time = time() - self.start_time
        self.get_logger().info(f"Converted point cloud to array of {len(xyz_array)} points. and minDistance is {minDistance} Total execution time: {total_time:.2f} seconds")

def main():
    rclpy.init()
    node = PointCloudConverter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()