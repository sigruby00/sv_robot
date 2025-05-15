import os
import math
import rclpy
import numpy as np
import random
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

MAX_SCAN_ANGLE = 240  # degrees

class LidarObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_avoidance')

        self.timeout = 10 # seconds
        self.threshold = 0.35  # meters
        self.speed = 0.2  # m/s
        self.lidar_type = ""
        self.random_angular_factor = random.uniform(0.4, 1.6)

        self.start_time = self.get_clock().now()
        self.should_shutdown = False

        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 50)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan_raw', self.lidar_callback,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        )
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.latest_scan = None
        self.get_logger().info(f'Started with threshold={self.threshold}, speed={self.speed}')

    def lidar_callback(self, msg):
        self.latest_scan = msg

    def timer_callback(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.timeout:
            self.get_logger().info('Timeout reached. Stopping robot and requesting shutdown...')
            self.cmd_pub.publish(Twist())
            self.should_shutdown = True
            return

        if self.latest_scan is None:
            return

        scan = self.latest_scan
        twist = Twist()
        center_index = len(scan.ranges) // 2
        front_index = (center_index + len(scan.ranges) // 2) % len(scan.ranges)
        front_range = scan.ranges[front_index]
        if not math.isfinite(front_range) or front_range < 0.05 or front_range > 5.0:
            self.get_logger().warn(f"Invalid front range: {front_range}")
            twist.linear.x = 0.0
            twist.angular.z = self.speed * random.choice([-1, 1])
            self.cmd_pub.publish(twist)
            return

        safe_forward_distance = 0.4  # meters
        self.get_logger().info(f'Front range: {front_range:.2f}')
        if front_range > safe_forward_distance:
            twist.linear.x = self.speed
            twist.angular.z = 0.0
            self.get_logger().info("Clear ahead, moving forward")
            self.cmd_pub.publish(twist)
            return
        else:
            twist.linear.x = 0.0
            twist.angular.z = self.speed * random.choice([-1, 1])
            self.get_logger().warn("Too close to obstacle in front, turning")
            self.cmd_pub.publish(twist)
            return

        if self.lidar_type != 'G4':
            max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / scan.angle_increment)
            left_ranges = scan.ranges[:max_index]
            right_ranges = scan.ranges[::-1][:max_index]
        else:
            min_index = int(math.radians((360 - MAX_SCAN_ANGLE) / 2.0) / scan.angle_increment)
            max_index = min_index + int(math.radians(MAX_SCAN_ANGLE / 2.0) / scan.angle_increment)
            left_ranges = scan.ranges[::-1][min_index:max_index][::-1]
            right_ranges = scan.ranges[min_index:max_index][::-1]

        left_valid = [r for r in left_ranges if 0.01 < r < 10.0]
        right_valid = [r for r in right_ranges if 0.01 < r < 10.0]

        min_left = min(left_valid, default=10.0)
        min_right = min(right_valid, default=10.0)

        if min(min_left, min_right) < 0.05:
            self.get_logger().warn('Too close to obstacle — backing up and turning')
            twist.linear.x = -self.speed / 2
            twist.angular.z = random.choice([-1, 1]) * self.speed * 4.0 * random.uniform(0.8, 1.2)
            self.cmd_pub.publish(twist)
            return

        if min_left <= self.threshold and min_right > self.threshold:
            twist.linear.x = self.speed / 6
            twist.angular.z = -self.speed * 6.0 * self.random_angular_factor
        elif min_left <= self.threshold and min_right <= self.threshold:
            twist.linear.x = random.uniform(0.05, self.speed / 5)
            twist.angular.z = random.choice([-1, 1]) * self.speed * 6.0 * random.uniform(0.8, 1.5)
        elif min_left > self.threshold and min_right <= self.threshold:
            twist.linear.x = self.speed / 6
            twist.angular.z = self.speed * 6.0 * self.random_angular_factor
        else:
            twist.linear.x = self.speed
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = LidarObstacleAvoidance()

    try:
        while rclpy.ok() and not node.should_shutdown:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
