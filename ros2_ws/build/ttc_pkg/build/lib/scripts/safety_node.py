#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from std_msgs.msg import Header

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import math


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """

        self.speed = 0.00
        self.ttc_threshold = 2.0
        # TODO: create ROS subscribers and publishers.
        self.publisher_drive = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.scan_sub  # prevent unused variable warning

        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)
        self.odom_sub  # prevent unused variable warning


    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC

        angle = scan_msg.angle_min
        brake = False
        for i in scan_msg.ranges:
            if i != math.inf and i != math.nan:
                r_d = self.speed * math.cos(angle)
                if r_d > 0:
                    ttc = i / r_d
                    if ttc < self.ttc_threshold:
                        brake = True
                        break
                        
            angle += scan_msg.angle_increment

        # TODO: publish command to brake
        if brake == True:
            drive_msg = AckermannDriveStamped()
            header = Header()
            header.stamp = self.get_clock().now().to_msg()

            drive_msg.header = header

            drive_msg.drive.speed = 0.0

            self.publisher_drive.publish(drive_msg)
            self.get_logger().info('Braking command published')
        pass

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()