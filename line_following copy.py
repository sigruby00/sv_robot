#!/usr/bin/env python3
# encoding: utf-8
# 巡线(line following)
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

MAX_SCAN_ANGLE = 240  # 激光的扫描角度,去掉总是被遮挡的部分degree(the scanning angle of lidar. The covered part is always eliminated)
class LineFollower:
    def __init__(self, color, node):
        self.node = node
        self.target_lab, self.target_rgb = color
        if os.environ['DEPTH_CAMERA_TYPE'] == 'ascamera':
            self.rois = ((0.9, 0.95, 0, 1, 0.7), (0.8, 0.85, 0, 1, 0.2), (0.7, 0.75, 0, 1, 0.1))
        else:
            self.rois = ((0.81, 0.83, 0, 1, 0.7), (0.69, 0.71, 0, 1, 0.2), (0.57, 0.59, 0, 1, 0.1))
        self.weight_sum = 1.0

    @staticmethod
    def get_widest_contour(contours, threshold=100):
        """
        获取最大宽度的轮廓(get the contour of the widest bounding box)
        :param contours:
        :param threshold:
        :return:
        """
        widest = None
        max_width = 0
        for c in contours:
            if cv2.contourArea(c) < threshold:
                continue
            rect = cv2.minAreaRect(c)
            (w, h) = rect[1]
            width = max(w, h)
            if width > max_width:
                max_width = width
                widest = c
        return (widest, max_width) if widest is not None else None

    def __call__(self, image, result_image, threshold):
        centroid_sum = 0
        h, w = image.shape[:2]
        if os.environ['DEPTH_CAMERA_TYPE'] == 'ascamera':
            w = w + 200
        # Use HSV-based green color mask extraction for green tape following.
        for roi in self.rois:
            blob = image[int(roi[0]*h):int(roi[1]*h), int(roi[2]*w):int(roi[3]*w)]  # 截取roi(intercept roi)
            # Convert to HSV and extract green mask
            hsv = cv2.cvtColor(blob, cv2.COLOR_RGB2HSV)
            # lower_green = np.array([40, 50, 50])
            # upper_green = np.array([85, 255, 255])
            # mask = cv2.inRange(hsv, lower_green, upper_green)
            lower_red1 = np.array([0, 70, 50])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 70, 50])
            upper_red2 = np.array([180, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            # mask_hsv = cv2.bitwise_or(mask1, mask2)
            mask = cv2.bitwise_or(mask1, mask2)

            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(corrode)
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilate)
            # cv2.imshow('section:{}:{}'.format(roi[0], roi[1]), cv2.cvtColor(dilated, cv2.COLOR_GRAY2BGR))
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找轮廓(find the contour)
            max_contour_area = self.get_widest_contour(contours, 100)  # 获取最大宽度对应轮廓(get the contour corresponding to the widest bounding box)
            if max_contour_area is not None:
                rect = cv2.minAreaRect(max_contour_area[0])  # 最小外接矩形(minimum circumscribed rectangle)
                box = np.intp(cv2.boxPoints(rect))  # 四个角(four corners)
                for j in range(4):
                    box[j, 1] = box[j, 1] + int(roi[0]*h)
                cv2.drawContours(result_image, [box], -1, (0, 255, 255), 2)  # 画出四个点组成的矩形(draw the rectangle composed of four points)

                # 获取矩形对角点(acquire the diagonal points of the rectangle)
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                # 线的中心点(center point of the line)
                line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2

                cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)   # 画出中心点(draw the center point)
                centroid_sum += line_center_x * roi[-1]
        if centroid_sum == 0:
            return result_image, None
        center_pos = centroid_sum / self.weight_sum  # 按比重计算中心点(calculate the center point according to the ratio)
        deflection_angle = -math.atan((center_pos - (w / 2.0)) / (h / 2.0))   # 计算线角度(calculate the line angle)
        return result_image, deflection_angle

class LineFollowingNode(Node):
    def __init__(self, name):
        # rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.name = name
        self.set_callback = False
        self.is_running = False
        self.color_picker = None
        self.follower = None
        self.scan_angle = math.radians(45)
        self.pid = pid.PID(0.005, 0.001, 0.0)
        self.empty = 0
        self.count = 0
        self.stop = False
        self.threshold = 0.5
        self.stop_threshold = 0.4
        self.lock = threading.RLock()
        self.image_sub = None
        self.lidar_sub = None
        self.image_height = None
        self.image_width = None
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(2)
        #self.camera_type = os.environ['DEPTH_CAMERA_TYPE']
        self.lidar_type = os.environ.get('LIDAR_TYPE')
        self.machine_type = os.environ.get('MACHINE_TYPE')
        self.pwm_pub = self.create_publisher(SetPWMServoState,'ros_robot_controller/pwm_servo/set_state',10)
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)  # 底盘控制(chassis control)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)  # 图像处理结果发布(publish the image processing result)
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)  # 进入玩法(enter the game)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)  # 退出玩法(exit the game)
        self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)  # 开启玩法(start the game)
        self.create_service(SetPoint, '~/set_target_color', self.set_target_color_srv_callback)  # 设置颜色(set the color)
        self.create_service(Trigger, '~/force_pick_color', self.force_pick_color_srv_callback)
        self.create_service(Trigger, '~/get_target_color', self.get_target_color_srv_callback)   # 获取颜色(get the color)
        self.create_service(SetFloat64, '~/set_threshold', self.set_threshold_srv_callback)  # 设置阈值(set the threshold)
        Heart(self, self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))  # 心跳包(heartbeat package)
        self.debug = self.get_parameter('debug').value
        if self.debug:
            threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def pwm_controller(self,position_data):
        pwm_list = []
        msg = SetPWMServoState()
        msg.duration = 0.2
        for i in range(len(position_data)):
            pos = PWMServoState()
            pos.id = [i+1]
            pos.position = [int(position_data[i])]
            pwm_list.append(pos)
        msg.state = pwm_list
        self.pwm_pub.publish(msg)

    def get_node_state(self, request, response):
        response.success = True
        return response

    def main(self):
        while True:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue

            result = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imshow("result", result)
            if self.debug and not self.set_callback:
                self.set_callback = True
                # 设置鼠标点击事件的回调函数(set a callback function for mouse click event)
                cv2.setMouseCallback("result", self.mouse_callback)
            k = cv2.waitKey(1)
            if k != -1:
                break
        self.mecanum_pub.publish(Twist())
        rclpy.shutdown()

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info("x:{} y{}".format(x, y))
            msg = SetPoint.Request()
            if self.image_height is not None and self.image_width is not None:
                msg.data.x = x / self.image_width
                msg.data.y = y / self.image_height
                self.set_target_color_srv_callback(msg, SetPoint.Response())

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "line following enter")
        if os.environ['DEPTH_CAMERA_TYPE'] != 'ascamera':
            self.pwm_controller([1850,1500])
        with self.lock:
            self.stop = False
            self.is_running = False
            self.color_picker = None
            self.pid = pid.PID(1.1, 0.0, 0.0)
            self.follower = None
            self.threshold = 0.5
            #self.camera_type = os.environ['DEPTH_CAMERA_TYPE']
            self.empty = 0
            if self.image_sub is None:
                 self.image_sub = self.create_subscription(Image, 'ascamera/camera_publisher/rgb0/image', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)
            if self.lidar_sub is None:
                qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
                self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, qos)  # 订阅雷达数据(subscribe to Lidar data)
            #set_servo_position(self.joints_pub, 1, ((10, 300), (5, 500), (4, 210), (3, 40), (2, 665), (1, 500)))
            self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "line following exit")
        try:
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.image_sub = None
            if self.lidar_sub is not None:
                self.destroy_subscription(self.lidar_sub)
                self.lidar_sub = None
        except Exception as e:
            self.get_logger().error(str(e))
        with self.lock:
            self.is_running = False
            self.color_picker = None
            self.pid = pid.PID(0.00, 0.001, 0.0)
            self.follower = None
            self.threshold = 0.5
            self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "exit"
        return response

    def set_target_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_target_color")
        with self.lock:
            x, y = request.data.x, request.data.y
            self.follower = None
            if x == -1 and y == -1:
                self.color_picker = None
            else:
                self.color_picker = ColorPicker(request.data, 5)
                self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "set_target_color"
        return response

    def force_pick_color_srv_callback(self, request, response):
        self.get_logger().info("[force_pick_color] Forcing ColorPicker to black (0,0)")
        msg = SetPoint.Request()
        msg.data.x = 0.0
        msg.data.y = 0.0
        with self.lock:
            self.follower = None
            self.color_picker = ColorPicker(msg.data, 5)
        response.success = True
        response.message = "ColorPicker set to (0.0, 0.0)"
        return response

    def get_target_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "get_target_color")
        response.success = False
        response.message = "get_target_color"
        with self.lock:
            if self.follower is not None:
                response.success = True
                rgb = self.follower.target_rgb
                response.message = "{},{},{}".format(int(rgb[0]), int(rgb[1]), int(rgb[2]))
        return response

    def set_running_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_running")
        with self.lock:
            self.is_running = request.data
            self.empty = 0
            if not self.is_running:
                self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "set_running"
        return response

    def set_threshold_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set threshold")
        with self.lock:
            self.threshold = request.data
            response.success = True
            response.message = "set_threshold"
            return response

    def lidar_callback(self, lidar_data):
        try:
            # 数据大小 = 扫描角度/每扫描一次增加的角度(data size= scanning angle/ the increased angle per scan)
            if self.lidar_type != 'G4':
                min_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
                max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
                left_ranges = lidar_data.ranges[:max_index]  # 左半边数据(left data)
                right_ranges = lidar_data.ranges[::-1][:max_index]  # 右半边数据(right data)
            elif self.lidar_type == 'G4':
                min_index = int(math.radians((360 - MAX_SCAN_ANGLE) / 2.0) / lidar_data.angle_increment)
                max_index = int(math.radians(180) / lidar_data.angle_increment)
                left_ranges = lidar_data.ranges[min_index:max_index][::-1]  # 左半边数据 (the left data)
                right_ranges = lidar_data.ranges[::-1][min_index:max_index][::-1]  # 右半边数据 (the right data)

            # 根据设定取数据(Get data according to settings)
            angle = self.scan_angle / 2
            angle_index = int(angle / lidar_data.angle_increment + 0.50)
            left_range, right_range = np.array(left_ranges[:angle_index]), np.array(right_ranges[:angle_index])

            left_nonzero = left_range.nonzero()
            right_nonzero = right_range.nonzero()
            left_nonan = np.isfinite(left_range[left_nonzero])
            right_nonan = np.isfinite(right_range[right_nonzero])
            # 取左右最近的距离(Take the nearest distance left and right)
            min_dist_left_ = left_range[left_nonzero][left_nonan]
            min_dist_right_ = right_range[right_nonzero][right_nonan]
            with self.lock:
                if len(min_dist_left_) > 1 and len(min_dist_right_) > 1:
                    min_dist_left = min_dist_left_.min()
                    min_dist_right = min_dist_right_.min()
                    if min_dist_left < self.stop_threshold or min_dist_right < self.stop_threshold:
                        # if not self.stop:
                            # self.get_logger().warn(f"[LIDAR] STOP! min_left={min_dist_left:.2f}, min_right={min_dist_right:.2f}")
                        self.stop = True
                    else:
                        self.count += 1
                        if self.count > 5:
                            # if self.stop:
                                # self.get_logger().info("[LIDAR] Resume, no obstacle.")
                            self.count = 0
                            self.stop = False
                # self.get_logger().info(f"[LIDAR] self.stop={self.stop}")
        except Exception as e:
            self.get_logger().error(f"[LIDAR] Exception: {str(e)}")

    def image_callback(self, ros_image):
        try:
            # self.get_logger().info(f"[IMAGE] image_callback called, self.stop={self.stop}")
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
            rgb_image = np.array(cv_image, dtype=np.uint8)
            self.image_height, self.image_width = rgb_image.shape[:2]
            result_image = np.copy(rgb_image)
            with self.lock:
                if self.color_picker is not None:
                    try:
                        target_color, result_image = self.color_picker(rgb_image, result_image)
                        target_color = ([0, 128, 128], (0, 0, 0))
                        if target_color is not None:
                            self.color_picker = None
                            self.follower = LineFollower(target_color, self)
                            self.get_logger().info("target color: {}".format(target_color))
                    except Exception as e:
                        self.get_logger().error(f"[IMAGE] ColorPicker Exception: {str(e)}")
                else:
                    twist = Twist()
                    twist.linear.x = 0.25
                    if self.follower is not None:
                        try:
                            result_image, deflection_angle = self.follower(rgb_image, result_image, self.threshold)
                            if deflection_angle is not None and self.is_running and not self.stop:
                                # Publish cmd_vel at most every 500ms (2Hz)
                                now = self.get_clock().now()
                                if not hasattr(self, 'last_pub_time'):
                                    self.last_pub_time = now
                                # if (now - self.last_pub_time).nanoseconds < 5e8:
                                # if (now - self.last_pub_time).nanoseconds < 2e8: # 5Hz
                                if (now - self.last_pub_time).nanoseconds < 1e8: # 5Hz
                                    return
                                self.last_pub_time = now
                                self.pid.update(deflection_angle)
                                if self.machine_type == 'MentorPi_Acker':
                                    steering_angle = common.set_range(-self.pid.output, -math.radians(322/2000*180), math.radians(322/2000*180))
                                    if steering_angle != 0:
                                        R = 0.145/math.tan(steering_angle)
                                        twist.angular.z = twist.linear.x/R
                                else:
                                    twist.angular.z = common.set_range(-self.pid.output, -1.0, 1.0)
                                # self.get_logger().info(f"[IMAGE] Following line, stop={self.stop}, twist=({twist.linear.x},{twist.angular.z})")
                                self.mecanum_pub.publish(twist)
                                # self.get_logger().info("[IMAGE] Published twist for line following.")
                            elif self.stop:
                                # self.get_logger().warn("[IMAGE] STOP requested by LIDAR! Publishing Twist() (stop)")
                                self.mecanum_pub.publish(Twist())
                                # self.get_logger().info("[IMAGE] Published Twist() for STOP.")
                            else:
                                # self.get_logger().info(f"[IMAGE] PID clear, stop={self.stop}")
                                self.pid.clear()
                        except Exception as e:
                            # self.get_logger().error(f"[IMAGE] Follower Exception: {str(e)}")
                            self.mecanum_pub.publish(Twist())
                            # self.get_logger().warn("[IMAGE] Exception occurred, forced STOP.")
        except Exception as e:
            self.get_logger().error(f"[IMAGE] Outer Exception: {str(e)}")
            try:
                self.mecanum_pub.publish(Twist())
                self.get_logger().warn("[IMAGE] Outer Exception, forced STOP.")
            except Exception as ee:
                self.get_logger().error(f"[IMAGE] Publish Exception: {str(ee)}")

def main():
    node = LineFollowingNode('line_following')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

