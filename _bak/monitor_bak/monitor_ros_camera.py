import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import socket
import json
import math
from sensor_msgs.msg import BatteryState
from std_msgs.msg import UInt16
from sensor_msgs.msg import Imu  # 추가
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import base64
from nav_msgs.msg import Odometry

class PoseSender(Node):
    def __init__(self):
        super().__init__('pose_sender')
        self.imu_data = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.host_ip = '127.0.0.1'
        self.port = 9000

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(UInt16, '/ros_robot_controller/battery', self.battery_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)  # IMU 구독 추가
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.create_timer(1.0, self.lookup_transform)  # 1Hz
        self.pos = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

    def battery_callback(self, msg):
        self.battery = msg.data / 100.0

    def imu_callback(self, msg):
        # 최신 IMU 데이터 저장 (선형 가속도와 각속도만)
        self.imu_data = {
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            }
        }

    def odom_callback(self, msg):
        if self.imu_data is None:
            self.imu_data = {}
        self.imu_data['linear_speed'] = msg.twist.twist.linear.x
        self.imu_data['angular_speed'] = msg.twist.twist.angular.z

    def lookup_transform(self):
        try:
            current_time = self.get_clock().now() - rclpy.duration.Duration(seconds=0.1)
            transform = self.tf_buffer.lookup_transform('map', 'base_link', current_time)

            if not self.imu_data or \
               'linear_speed' not in self.imu_data or \
               'angular_speed' not in self.imu_data:
                self.get_logger().warn("Waiting for odometry speed data...")
                return

            t = transform.transform.translation
            q = transform.transform.rotation

            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z)
            )

            self.pos['x'] = t.x
            self.pos['y'] = t.y
            self.pos['yaw'] = yaw

            data = {
                'pos': {
                    'x': self.pos['x'],
                    'y': self.pos['y'],
                    'yaw': self.pos['yaw']
                },
                'imu': self.imu_data,
                'battery': self.battery
            }

            print("\n== Transform: map → base_link ==")
            print(f"Pos X: {self.pos['x']:.3f}")
            print(f"Pos Y: {self.pos['y']:.3f}")
            print(f"Pos Yaw: {self.pos['yaw']:.3f}")

            if self.imu_data:
                print(f"IMU Acc: {self.imu_data['linear_acceleration']}")
                print(f"IMU Gyro: {self.imu_data['angular_velocity']}")
                print(f"IMU Speed - Linear: {self.imu_data['linear_speed']:.3f} m/s")
                print(f"IMU Speed - Angular: {self.imu_data['angular_speed']:.3f} rad/s")

            print(f"Battery Level: {self.battery:.2f}%" if self.battery is not None else "Battery Level: N/A")

            self.sock.sendto(json.dumps(data).encode(), (self.host_ip, self.port))

        except Exception as e:
            self.get_logger().error(f'Could not transform: {e}')

class CameraSender(Node):
    def __init__(self):
        super().__init__('camera_sender')
        self.bridge = CvBridge()
        self.camera_frame = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.host_ip = '127.0.0.1'
        self.port = 9001  # separate port for camera

        self.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image', self.camera_callback, 10)
        self.create_timer(0.1, self.send_image)

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.resize(cv_image, (320, 240))  # 예: 320x240으로 축소

            _, jpeg = cv2.imencode('.jpg', cv_image)
            b64_image = base64.b64encode(jpeg).decode('utf-8')
            self.camera_frame = b64_image
        except Exception as e:
            self.get_logger().error(f"Camera callback error: {e}")

    def send_image(self):
        if self.camera_frame:
            data = {
                'image': self.camera_frame
            }
            try:
                self.sock.sendto(json.dumps(data).encode(), (self.host_ip, self.port))
                self.get_logger().info(f"Sent camera frame (base64 size: {len(self.camera_frame)})")
            except Exception as e:
                self.get_logger().error(f"Failed to send camera frame: {e}")

def main():
    rclpy.init()
    pose_node = PoseSender()
    camera_node = CameraSender()
    try:
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(pose_node)
        executor.add_node(camera_node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    pose_node.destroy_node()
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()