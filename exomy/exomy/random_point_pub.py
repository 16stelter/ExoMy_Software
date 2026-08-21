#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class RandomPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('random_pointcloud_pub')
        self.pub = self.create_publisher(PointCloud2, '/random_pointcloud', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("RandomPointCloudPublisher node initialized.")
        points = np.random.rand(100000, 3).astype(np.float32)
        header = pc2.Header()
        header.frame_id = "map"
        self.cloud = pc2.create_cloud_xyz32(header, points)

    def timer_callback(self):
        self.cloud.header.stamp = self.get_clock().now().to_msg() 
        self.pub.publish(self.cloud)

def main(args=None):
    rclpy.init(args=args)
    node = RandomPointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
