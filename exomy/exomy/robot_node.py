#!/usr/bin/env python
from exomy_msgs.msg import RoverCommand, MotorCommands
import geometry_msgs.msg
import rclpy
from rclpy.node import Node
from .rover import Rover
import math


class RobotNode(Node):
    FL, FR, CL, CR, RL, RR = range(0, 6)
    def __init__(self):
        self.node_name = 'robot_node'
        super().__init__(self.node_name)

        self.joy_sub = self.create_subscription(
            RoverCommand,
            'rover_command',
            self.joy_callback,
            10)
        
        self.cmd_vel_sub = self.create_subscription(
            geometry_msgs.msg.Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        self.robot_pub = self.create_publisher(
            MotorCommands,
            'motor_commands',
            1)
        self.robot = Rover()

        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))

    def joy_callback(self, msg):
        cmds = MotorCommands()

        self.robot.setLocomotionMode(msg.locomotion_mode)
        cmds.motor_angles = self.robot.joystickToSteeringAngle(
            msg.vel, msg.steering)
        cmds.motor_speeds = self.robot.joystickToVelocity(
            msg.vel, msg.steering)

        self.robot_pub.publish(cmds)

    def cmd_vel_callback(self, msg):
        cmds = MotorCommands()
        max_linear_speed = 1.0 # m/s
        max_angular_speed = 1.0 # rad/s 
        front_wheel_x = 0.16
        rear_wheel_x = 0.14
        wheel_y = 0.2
        max_steering_angle = 60
        # make these parameters for tuning
        motor_speeds = [0]*6
        motor_angles = [0]*6

        if(msg.linear.y != 0):
            # crabwalking
            return
        elif(msg.linear.x != 0):
            # ackerman steering
            if(msg.angular.z == 0):
                motor_speeds = [msg.linear.x / max_linear_speed * 100]*6
            else:
                R = (msg.linear.x / msg.angular.z) - wheel_y
                motor_angles[self.FL] = math.atan(front_wheel_x, R)
                motor_angles[self.FR] = math.atan(front_wheel_x, R)
                motor_angles[self.CL] = math.atan(0, R)
                motor_angles[self.CR] = math.atan(0, R)
                motor_angles[self.RL] = math.atan(rear_wheel_x, R)
                motor_angles[self.RR] = math.atan(rear_wheel_x, R)
                motor_speeds[self.FL] = msg.angular.z * math.hypot(R, front_wheel_x) / max_linear_speed * 100
                motor_speeds[self.FR] = msg.angular.z * math.hypot(R, front_wheel_x) / max_linear_speed * 100
                motor_speeds[self.CL] = msg.angular.z * math.hypot(R, 0) / max_linear_speed * 100
                motor_speeds[self.CR] = msg.angular.z * math.hypot(R, 0) / max_linear_speed * 100
                motor_speeds[self.RL] = msg.angular.z * math.hypot(R, rear_wheel_x) / max_linear_speed * 100
                motor_speeds[self.RR] = msg.angular.z * math.hypot(R, rear_wheel_x) / max_linear_speed * 100
        elif(msg.angular.z != 0):
            motor_angles[self.FL] = 45
            motor_angles[self.FR] = -45
            motor_angles[self.RL] = -45
            motor_angles[self.RR] = 45
            motor_speeds[self.FL] = msg.angular.z / max_angular_speed * 100
            motor_speeds[self.FR] = -msg.angular.z / max_angular_speed * 100
            motor_speeds[self.CL] = msg.angular.z / max_angular_speed * 100
            motor_speeds[self.CR] = -msg.angular.z / max_angular_speed * 100
            motor_speeds[self.RL] = msg.angular.z / max_angular_speed * 100
            motor_speeds[self.RR] = -msg.angular.z / max_angular_speed * 100

        cmds.motor_speeds = motor_speeds
        cmds.motor_angles = motor_angles
        self.robot_pub.publish(cmds)


def main(args=None):
    rclpy.init(args=args)

    try:
        robot_node = RobotNode()
        try:
            rclpy.spin(robot_node)
        finally:
            robot_node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
