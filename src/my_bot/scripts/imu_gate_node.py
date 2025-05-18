#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class ImuGateNode(Node):
    def __init__(self):
        super().__init__('imu_gate_node')

        self.moving = False
        self.vel_threshold = 0.01  # m/s
        self.angular_threshold = 0.01  # rad/s

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu/mpu6050', self.imu_callback, 10)

        self.pub_imu = self.create_publisher(Imu, '/imu/filtered', 10)

    def odom_callback(self, msg):
        lin_speed = (msg.twist.twist.linear.x ** 2 + msg.twist.twist.linear.y ** 2) ** 0.5
        ang_speed = abs(msg.twist.twist.angular.z)

        if lin_speed > self.vel_threshold or ang_speed > self.angular_threshold:
            self.moving = True
        else:
            self.moving = False

    def imu_callback(self, msg):
        if self.moving:
            self.pub_imu.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuGateNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
