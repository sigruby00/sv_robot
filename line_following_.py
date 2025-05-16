# 최적화 포인트:
# 1. image_callback 내 PID, Twist publish 호출 최소화
# 2. 라이다 처리는 1초에 1~2회로 줄이기 (타이머 기반으로 옮겨도 고려)
# 3. color_picker 강제 지정값 제거
# 4. 디버그 출력 기본 OFF 유지
# 5. cv2.imshow 사용 시 조건 명확히

import os
import cv2
import math
import rclpy
import queue
import threading
import numpy as np
import sdk.pid as pid
import sdk.common as common
from rclpy.node import Node
from app.common import Heart
from cv_bridge import CvBridge
from app.common import ColorPicker
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image, LaserScan
from interfaces.srv import SetPoint, SetFloat64
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from ros_robot_controller_msgs.msg import MotorsState, SetPWMServoState, PWMServoState

MAX_SCAN_ANGLE = 240

class LineFollowingNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.name = name
        self.lock = threading.RLock()
        self.bridge = CvBridge()

        self.is_running = False
        self.stop = False
        self.image_height = None
        self.image_width = None
        self.threshold = 0.5
        self.stop_threshold = 0.4
        self.color_picker = None
        self.follower = None
        self.pid = pid.PID(1.1, 0.0, 0.0)

        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.image_sub = self.create_subscription(Image, 'ascamera/camera_publisher/rgb0/image', self.image_callback, 1)
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, qos)

        self.create_service(Trigger, '~/enter', self.enter_callback)
        self.create_service(Trigger, '~/exit', self.exit_callback)
        self.create_service(SetBool, '~/set_running', self.set_running_callback)
        self.create_service(SetFloat64, '~/set_threshold', self.set_threshold_callback)
        self.create_service(SetPoint, '~/set_target_color', self.set_target_color_callback)
        self.create_service(Trigger, '~/force_pick_color', self.force_pick_color_callback)
        Heart(self, self.name + '/heartbeat', 5, lambda _: self.exit_callback(Trigger.Request(), Trigger.Response()))
        self.get_logger().info(f'{name} started')

    def enter_callback(self, request, response):
        with self.lock:
            self.is_running = False
            self.stop = False
            self.pid.clear()
        response.success = True
        response.message = "enter"
        return response

    def exit_callback(self, request, response):
        with self.lock:
            self.is_running = False
            self.stop = False
            self.pid.clear()
            self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "exit"
        return response

    def set_running_callback(self, request, response):
        with self.lock:
            self.is_running = request.data
            if not self.is_running:
                self.mecanum_pub.publish(Twist())
        response.success = True
        return response

    def set_threshold_callback(self, request, response):
        self.threshold = request.data
        response.success = True
        return response

    def set_target_color_callback(self, request, response):
        with self.lock:
            self.color_picker = ColorPicker(request.data, 5)
            self.follower = None
        response.success = True
        return response

    def force_pick_color_callback(self, request, response):
        with self.lock:
            from geometry_msgs.msg import Point
            msg = Point()
            msg.x, msg.y = 0.0, 0.0
            self.color_picker = ColorPicker(msg, 5)
            self.follower = None
        response.success = True
        return response

    def lidar_callback(self, msg):
        try:
            angle = math.radians(45) / 2
            idx = int(angle / msg.angle_increment)
            left = np.array(msg.ranges[:idx])
            right = np.array(msg.ranges[::-1][:idx])
            min_l = np.min(left[np.isfinite(left)])
            min_r = np.min(right[np.isfinite(right)])
            self.stop = min(min_l, min_r) < self.stop_threshold
        except Exception as e:
            self.get_logger().warn(f"LIDAR error: {e}")

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            self.image_height, self.image_width = img.shape[:2]
            with self.lock:
                if self.color_picker is not None:
                    target_color, _ = self.color_picker(img, img.copy())
                    if target_color:
                        self.color_picker = None
                        self.follower = LineFollower(target_color, self)
                        return

                if self.follower and self.is_running:
                    _, angle = self.follower(img, img.copy(), self.threshold)
                    if angle and not self.stop:
                        self.pid.update(angle)
                        cmd = Twist()
                        cmd.linear.x = 0.25
                        cmd.angular.z = common.set_range(-self.pid.output, -1.0, 1.0)
                        self.mecanum_pub.publish(cmd)
                        return
            self.mecanum_pub.publish(Twist())
        except Exception as e:
            self.get_logger().error(f"Image error: {e}")
            self.mecanum_pub.publish(Twist())


def main():
    rclpy.init()
    node = LineFollowingNode('line_following')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
