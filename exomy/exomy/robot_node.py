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
        max_linear_speed = 0.25 # m/s
        max_angular_speed = 1.0 # rad/s 
        front_wheel_x = 0.16
        rear_wheel_x = -0.14
        wheel_y = 0.1
        max_steering_angle = 60 # for ackermann steering
        # make these parameters for tuning and calibrate
        motor_speeds = [0]*6
        motor_angles = [0]*6

        msg.linear.x = max(-max_linear_speed, min(msg.linear.x, max_linear_speed))
        msg.linear.y = max(-max_linear_speed, min(msg.linear.y, max_linear_speed))
        msg.angular.z = max(-max_angular_speed, min(msg.angular.z, max_angular_speed))


        if(msg.linear.y != 0):
            # crabwalking
            angle = math.atan2(msg.linear.y, msg.linear.x)
            speed = max(-max_linear_speed, min(math.hypot(msg.linear.x, msg.linear.y), max_linear_speed))
            if angle > math.pi/2:
                angle -= math.pi
                speed = -speed
            elif angle < -math.pi/2:
                angle += math.pi
                speed = -speed
            self.get_logger().info(f"cmd_vel: linear.x={msg.linear.x}, linear.y={msg.linear.y}, angular.z={msg.angular.z} => angle={math.degrees(angle)}, speed={speed}")
            motor_speeds = [int(speed / max_linear_speed * 100)]*6
            motor_angles = [int(math.degrees(angle))]*6
        elif(msg.linear.x != 0):
            # ackerman steering
            if(msg.angular.z == 0):
                motor_speeds = [int(msg.linear.x / max_linear_speed * 100)]*6
            else:
                R = abs(msg.linear.x) / abs(msg.angular.z)
                theta_front_in = max(-max_steering_angle, min(max_steering_angle, math.degrees(math.atan2(front_wheel_x, (R - wheel_y)))))
                theta_front_out = max(-max_steering_angle, min(max_steering_angle, math.degrees(math.atan2(front_wheel_x, (R + wheel_y)))))
                theta_rear_in = max(-max_steering_angle, min(max_steering_angle, math.degrees(math.atan2(rear_wheel_x, (R - wheel_y)))))
                theta_rear_out = max(-max_steering_angle, min(max_steering_angle, math.degrees(math.atan2(rear_wheel_x, (R + wheel_y)))))
                if msg.angular.z < 0:
                    motor_angles[self.FL] = int(theta_front_in)
                    motor_angles[self.FR] = int(theta_front_out)
                    motor_angles[self.RL] = int(theta_rear_in)
                    motor_angles[self.RR] = int(theta_rear_out)
                else:
                    motor_angles[self.FL] = int(-theta_front_out)
                    motor_angles[self.FR] = int(-theta_front_in)
                    motor_angles[self.RL] = int(-theta_rear_out)
                    motor_angles[self.RR] = int(-theta_rear_in)
                motor_speeds = [int(msg.linear.x / max_linear_speed * 100)]*6
        elif(msg.angular.z != 0):
            motor_angles[self.FL] = 45
            motor_angles[self.FR] = -45
            motor_angles[self.RL] = -45
            motor_angles[self.RR] = 45
            motor_speeds[self.FL] = int(-msg.angular.z / max_angular_speed * 100)
            motor_speeds[self.FR] = int(msg.angular.z / max_angular_speed * 100)
            motor_speeds[self.CL] = int(-msg.angular.z / max_angular_speed * 100)
            motor_speeds[self.CR] = int(msg.angular.z / max_angular_speed * 100)
            motor_speeds[self.RL] = int(-msg.angular.z / max_angular_speed * 100)
            motor_speeds[self.RR] = int(msg.angular.z / max_angular_speed * 100)

        self.get_logger().info(str(motor_angles))
        
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
