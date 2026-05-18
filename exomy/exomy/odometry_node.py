#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf_transformations
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry')

        self.declare_parameter('namespace', '')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate', 20.0)

        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value


        ns = self.get_namespace()
        if ns:
            self.odom_frame = ns + "/" + self.odom_frame
            self.base_frame = ns + "/" + self.base_frame

        self.timestamp = self.get_clock().now().to_msg()


        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_cb, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, 'imu', self.imu_cb, 10
        )
        self.odom_pub = self.create_publisher(
            Odometry, 'odom', 10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = self.y = self.z = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.vroll = self.vpitch = self.vyaw = 0.0

        self.dt = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.dt, self.timer_cb)

    def cmd_vel_cb(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vz = msg.linear.z
        self.vroll = msg.angular.x
        self.vpitch = msg.angular.y
        self.vyaw = msg.angular.z

    def imu_cb(self, msg):
        q = msg.orientation
        self.roll, self.pitch, self.yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.timestamp = msg.header.stamp # hack for sim time
        rclpy.logging.get_logger('odometry').debug(f'IMU orientation: roll={self.roll}, pitch={self.pitch}, yaw={self.yaw}')

    def timer_cb(self):
        R = tf_transformations.euler_matrix(self.roll, self.pitch, self.yaw)[:3, :3]
        v_robot = np.array([self.vx, self.vy, self.vz])
        v_world = R @ v_robot
        self.x += v_world[0] * self.dt
        self.y += v_world[1] * self.dt
        self.z += v_world[2] * self.dt
        #self.roll += self.vroll * self.dt
        #self.pitch += self.vpitch * self.dt
        #self.yaw += self.vyaw * self.dt

        t = TransformStamped()
        t.header.stamp = self.timestamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        q = tf_transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = self.timestamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        q = tf_transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        odom.twist.twist.linear = Vector3(x=self.vx, y=self.vy, z=self.vz)
        odom.twist.twist.angular = Vector3(x=self.vroll, y=self.vpitch, z=self.vyaw)
        self.odom_pub.publish(odom)

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