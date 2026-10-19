#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf_transformations
import numpy as np
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry')

        self.declare_parameter('namespace', '')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'camera_imu_optical_frame')
        self.declare_parameter('publish_rate', 20.0)

        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.imu_frame = self.get_parameter('imu_frame').get_parameter_value().string_value

        self.base_to_imu = None

        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        ns = self.get_namespace()
        if ns and ns != "/":
            self.odom_frame = ns + "/" + self.odom_frame
            self.base_frame = ns + "/" + self.base_frame

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_sub = self.create_subscription(Odometry, 'odomimu', self.odom_callback, 10)

    def odom_callback(self, msg):
        if self.base_to_imu is None:
            try:
                self.base_to_imu_transform = self.tf_buffer.lookup_transform(self.base_frame, self.imu_frame, rclpy.time.Time())
            except Exception as e:
                self.get_logger().warn(f"Could not get transform from {self.base_frame} to {self.imu_frame}: {e}", once=True)
                return

        odom_to_imu_q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

        odom_to_imu_t = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]

        odom_to_imu_matrix = tf_transformations.quaternion_matrix(odom_to_imu_q)
        odom_to_imu_matrix[0:3, 3] = odom_to_imu_t

        base_to_imu_q = [
            self.base_to_imu_transform.transform.rotation.x,
            self.base_to_imu_transform.transform.rotation.y,
            self.base_to_imu_transform.transform.rotation.z,
            self.base_to_imu_transform.transform.rotation.w
        ]

        base_to_imu_t = [
            self.base_to_imu_transform.transform.translation.x,
            self.base_to_imu_transform.transform.translation.y,
            self.base_to_imu_transform.transform.translation.z
        ]

        base_to_imu_matrix = tf_transformations.quaternion_matrix(base_to_imu_q)
        base_to_imu_matrix[0:3, 3] = base_to_imu_t

        imu_to_base_matrix = tf_transformations.inverse_matrix(base_to_imu_matrix)
        odom_to_base_matrix = np.dot(odom_to_imu_matrix, imu_to_base_matrix)

        odom_to_base_t = tf_transformations.translation_from_matrix(odom_to_base_matrix)
        odom_to_base_q = tf_transformations.quaternion_from_matrix(odom_to_base_matrix)

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = odom_to_base_t[0]
        transform.transform.translation.y = odom_to_base_t[1]
        transform.transform.translation.z = odom_to_base_t[2]
        transform.transform.rotation.x = odom_to_base_q[0]
        transform.transform.rotation.y = odom_to_base_q[1]
        transform.transform.rotation.z = odom_to_base_q[2]
        transform.transform.rotation.w = odom_to_base_q[3]

        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)

    try:
        odom_node = OdometryNode()
        try:
            rclpy.spin(odom_node)
        finally:
            odom_node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
