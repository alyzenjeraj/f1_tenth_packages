import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header
import math
import signal

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)

        # 2 LaserScans
        self.a = 0 # Taken at 50 Degrees
        self.b = 0 # Taken at 0/90 Degrees

        # TODO: set PID gains
        self.kp = 0.8
        self.kd = 0.2
        self.ki = 0.0002

        # TODO: store history
        self.integral = 0
        self.prev_error = 0
        # self.error = 

        # TODO: store any necessary values you think you'll need
        # signal.signal(signal.SIGINT, self.shutdown_handler)

    def get_range(self, range_data, angle, angle_min, angle_increment):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        
        index = int((math.radians(angle) - angle_min)/angle_increment)

        #TODO: implement

        range_dist = range_data[index]
        if range_dist != math.inf and range_dist != math.nan:
            return range_dist



    def get_error(self, a_dist, b_dist, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        numerator = (a_dist * math.cos(math.radians(45))) - b_dist 
        denomenator = a_dist * math.sin(math.radians(45))
        alpha = math.atan(numerator/denomenator)

        d_t = b_dist * math.cos(alpha)
        print(f'Distance: {d_t}')
        l = 0.1
        d_t_1 = d_t + (l * math.sin(alpha))

        #TODO:implement
        return (dist - d_t_1)

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """


        # TODO: Use kp, ki & kd to implement a PID controller
        self.integral += error * 0.1 # Time between messages
        deriv = (error - self.prev_error) / 0.1

        angle = (self.kp * error) + (self.ki * self.integral) + (self.kd * deriv)
        print(f'Angle: {math.degrees(angle)}')

        
        # TODO: fill in drive message and publish
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = angle
        self.drive_pub.publish(drive_msg)

        self.prev_error = error

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        angle_increment = msg.angle_increment

        angle_min = msg.angle_min
        range_a = self.get_range(msg.ranges, -45, angle_min, angle_increment) 
        range_b = self.get_range(msg.ranges, -90, angle_min, angle_increment) 

        # error = 0.0 # TODO: replace with error calculated by get_error()
        error = self.get_error(range_a, range_b, 0.7)

        velocity = 0.0
        if abs(error) < 0.2:
            velocity = 1.5
        elif abs(error) < 0.5:
            velocity = 1.0
        else:
            velocity = 0.5

        # print(f'Velocity {velocity}, Error: {error}')
         # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID

    def shutdown_handler(self, signum, frame):
        print('SHUTTING DOWN')
        stop_msg = AckermannDriveStamped()
        stop_msg.drive.speed = 0.0
        stop_msg.drive.steering_angle = 0.0
        self.drive_pub.publish(stop_msg)

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # print('STOPPED')
    # wall_follow_node.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()