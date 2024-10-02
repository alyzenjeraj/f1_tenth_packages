import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

import random
import math

class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planner')

        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.path_pub = self.create_publisher(Path, '/rrt_path', 10)

        self.start_pose = None
        self.goal_pose = None
        self.scan_data = None

    def odom_callback(self, odom_msg):
        self.start_pose = PoseStamped()
        self.start_pose.pose = odom_msg.pose.pose
        self.get_logger().info(f'Current Position Recieved')

        if self.start_pose is not None:
            self.generate_rrt_path()

    def goal_callback(self, goal_pose):
        self.goal_pose = goal_pose
        self.get_logger().info(f'Goal Recieved {goal_pose}')

        if self.start_pose is not None:
            self.generate_rrt_path()

    def scan_callback(self, scan_msg):
        self.scan_data = scan_msg
        self.get_logger().info('Laser Scan Data Recieved')

    def generate_rrt_path(self):
        if self.start_pose is None or self.goal_pose is None or self.scan_data is None:
            self.get_logger().warn('Waiting for start, goal, or scan data')
            return

        self.get_logger().info('Generating RRT Path...')
        path = self.run_rrt(self.start_pose, self.goal_pose)

        self.publish_path(path)


    def run_rrt(self, start, goal):
        path = []

        start_point = (start.pose.position.x, start.pose.position.y)
        goal_point = (goal.pose.position.x, goal.pose.position.y)

        path.append(start)
        current_pose = start

        for _ in range(50):
            random_point = self.sample_random_point()
            nearest_node = self.find_nearest_node(random_point, current_pose)
            new_node = self.steer(nearest_node.pose.position, random_point)

            if self.is_collision_free(nearest_node.pose.position, new_node.pose.position):
                current_pose = new_node
                path.append(new_node)

            if self.reached_goal((new_node.pose.position.x, new_node.pose.position.y), goal_point):
                path.append(goal)
                break

        return path


    def sample_random_point(self):
        return(random.uniform(-5,5), random.uniform(-5, 5))

    def find_nearest_node(self, random_point, current_pose):
        # TODO: Implement this properly
        return current_pose
        
    def steer(self, nearest_node, random_point):
        direction = math.atan2(random_point[1] - nearest_node.y, random_point[0] - nearest_node.x)
        step_size = 1.0  # Step size for RRT
        new_node_x = nearest_node.x + step_size * math.cos(direction)
        new_node_y = nearest_node.y + step_size * math.sin(direction)
        
        new_pose = PoseStamped()
        new_pose.pose.position.x = new_node_x
        new_pose.pose.position.y = new_node_y
        return new_pose

    def is_collision_free(self, nearest_node, new_node):

        if self.scan_data is None:
            return False

        new_node_dist = math.hypot(new_node.x - nearest_node.x, new_node.y - nearest_node.y)

        for i, range in enumerate(self.scan_data.ranges):
            if range < self.scan_data.range_max:
                # Convert laser angle to Cartesian coordinates
                angle = self.scan_data.angle_min + i * self.scan_data.angle_increment
                obstacle_x = range * math.cos(angle)
                obstacle_y = range * math.sin(angle)
                dist_to_obstacle = math.hypot(new_node.x - obstacle_x, new_node.y - obstacle_y)

                if dist_to_obstacle < 0.5:  # Threshold for collision
                    return False

        return True

    def reached_goal(self, new_node, goal_point):
        # Check if we've reached the goal
        return math.dist(new_node, goal_point) < 0.5

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        path_msg.poses = path
        self.path_pub.publish(path_msg)
        self.get_logger().info("Path published")


def main(args=None):
    rclpy.init(args=args)
    node = RRTPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()