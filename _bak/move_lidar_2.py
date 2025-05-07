import os
import math
import rclpy
import numpy as np
import random
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

MAX_SCAN_ANGLE = 240  # degrees

class LidarObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_avoidance')

        self.timeout = 10 # seconds
        self.threshold = 0.3  # meters
        self.speed = 0.2  # m/s
        self.lidar_type = ""
        self.random_angular_factor = random.uniform(0.4, 1.6)

        self.start_time = self.get_clock().now()
        self.should_shutdown = False

        self.last_act = 0
        self.timestamp = 0

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

        if self.latest_scan is None or self.timestamp > time.time():
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

        if self.lidar_type != 'G4':
            max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / scan.angle_increment)
            left_ranges = scan.ranges[:max_index]
            right_ranges = scan.ranges[::-1][:max_index]
        else:
            min_index = int(math.radians((360 - MAX_SCAN_ANGLE) / 2.0) / scan.angle_increment)
            max_index = min_index + int(math.radians(MAX_SCAN_ANGLE / 2.0) / scan.angle_increment)
            left_ranges = scan.ranges[::-1][min_index:max_index][::-1]
            right_ranges = scan.ranges[min_index:max_index][::-1]

        left_array = np.array(left_ranges)
        right_array = np.array(right_ranges)
        left_nonzero = left_array.nonzero()
        right_nonzero = right_array.nonzero()
        left_nonan = np.isfinite(left_array[left_nonzero])
        right_nonan = np.isfinite(right_array[right_nonzero])
        min_dist_left_ = left_array[left_nonzero][left_nonan]
        min_dist_right_ = right_array[right_nonzero][right_nonan]

        if len(min_dist_left_) > 1 and len(min_dist_right_) > 1:
            min_left = min_dist_left_.min()
            min_right = min_dist_right_.min()

            if min_left <= self.threshold and min_right > self.threshold:
                twist.linear.x = self.speed / 6
                max_angle = math.radians(90)
                w = self.speed * 6.0
                twist.angular.z = -w
                if self.last_act != 0 and self.last_act != 1:
                    twist.angular.z = w
                self.last_act = 1
                self.cmd_pub.publish(twist)
                self.timestamp = time.time() + (max_angle / w / 2)
                return

            elif min_left <= self.threshold and min_right <= self.threshold:
                twist.linear.x = self.speed / 6
                w = self.speed * 6.0
                twist.angular.z = w
                self.last_act = 3
                self.cmd_pub.publish(twist)
                self.timestamp = time.time() + (math.radians(180) / w / 2)
                return

            elif min_left > self.threshold and min_right <= self.threshold:
                twist.linear.x = self.speed / 6
                max_angle = math.radians(90)
                w = self.speed * 6.0
                twist.angular.z = w
                if self.last_act != 0 and self.last_act != 2:
                    twist.angular.z = -w
                self.last_act = 2
                self.cmd_pub.publish(twist)
                self.timestamp = time.time() + (max_angle / w / 2)
                return

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