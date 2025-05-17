import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import socket
import json
import math
import time
from std_msgs.msg import UInt16
from sensor_msgs.msg import Imu, Image
import cv2
from cv_bridge import CvBridge
import subprocess
import os, sys
import threading
import select
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

sys.path.append(os.path.dirname(__file__))
from robot_config.robot_config_id import ROBOT_ID

robot_id = ROBOT_ID

# Improved run_command to prevent zombie processes
def run_command(cmd, wait=True):
    if wait:
        subprocess.run(cmd, shell=True, executable='/bin/bash')
    else:
        p = subprocess.Popen(
            cmd, shell=True, preexec_fn=os.setpgrp,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            executable='/bin/bash'
        )
        threading.Thread(target=p.wait, daemon=True).start()
        return p

class CommandReceiver(Node):
    def __init__(self):
        super().__init__('command_receiver')
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.bind(('0.0.0.0', 9002))
        threading.Thread(target=self.receive_command_loop, daemon=True).start()

    def receive_command_loop(self):
        while rclpy.ok():
            ready, _, _ = select.select([self.cmd_sock], [], [], 0.1)
            if not ready:
                continue
            try:
                data, _ = self.cmd_sock.recvfrom(1024)
                command = json.loads(data.decode())
                print(command)
                # navigation commands
                nav = command.get('navigation')
                if nav is not None:
                    action_value = str(nav['action'])
                    command = f"ros2 service call /line_following/set_running std_srvs/srv/SetBool '{{data: {action_value}}}'"

                    run_command(command, wait=False)


                # manual actions
                action = command.get('action')
                if action:
                    twist = Twist()
                    if action == 'move_forward': twist.linear.x = 0.5
                    elif action == 'move_backward': twist.linear.x = -0.5
                    elif action == 'turn_left': twist.angular.z = 1.0
                    elif action == 'turn_right': twist.angular.z = -1.0
                    elif action == 'stop':
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)
            except Exception:
                pass

class PoseSender(Node):
    def __init__(self):
        super().__init__('pose_sender')
        self.imu_data = {}
        self.battery = 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.host_ip, self.port = '127.0.0.1', 9000

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

        self.create_subscription(UInt16, '/ros_robot_controller/battery', self.battery_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_timer(1.0, self.lookup_transform)
        self.pos = {'x':0.0, 'y':0.0, 'yaw':0.0}

    def battery_callback(self, msg):
        self.battery = msg.data / 105.0

    def imu_callback(self, msg):
        self.imu_data.update({
            'linear_acceleration':{
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            },
            'angular_velocity':{
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            }
        })

    def odom_callback(self, msg):
        self.imu_data['linear_speed'] = msg.twist.twist.linear.x
        self.imu_data['angular_speed'] = msg.twist.twist.angular.z

    def lookup_transform(self):
        try:
            trans = self.tf_buffer.lookup_transform('map','base_link',rclpy.time.Time())
            t=trans.transform.translation; q=trans.transform.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.pos.update({'x':t.x, 'y':t.y, 'yaw':yaw})
            data = {'pos': self.pos, 'imu': self.imu_data, 'battery': self.battery}
            self.sock.sendto(json.dumps(data).encode(), (self.host_ip, self.port))
        except Exception:
            pass

class CameraSender(Node):
    def __init__(self):
        super().__init__('camera_sender')
        self.bridge = CvBridge()
        self.camera_frame = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.host_ip, self.port = '10.243.76.27', 9001

        self.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image', self.camera_callback, 10)
        # self.create_timer(0.2, self.send_image)  # reduced to 5Hz

    def camera_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            img = cv2.resize(img, (320, 240))
            _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 100])
            self.send_image(jpeg.tobytes())  # frame 새로 들어올 때마다 바로 전송
        except Exception:
            pass

    def send_image(self, jpeg_bytes):
        rid = int(robot_id).to_bytes(2, 'big')
        payload = rid + jpeg_bytes
        if len(payload) < 30720:
            payload += b'\x00' * (30720 - len(payload))
        try:
            self.sock.sendto(payload, (self.host_ip, self.port))
        except Exception:
            pass

    # def camera_callback(self, msg):
    #     try:
    #         img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    #         img = cv2.resize(img, (320, 240))
    #         _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 50])
    #         self.camera_frame = jpeg.tobytes()
    #     except Exception:
    #         pass

    # def send_image(self):
    #     if not self.camera_frame:
    #         return
    #     rid = int(robot_id).to_bytes(2, 'big')
    #     payload = rid + self.camera_frame
    #     if len(payload) < 30720:
    #         payload += b'\x00' * (30720 - len(payload))
    #     try:
    #         self.sock.sendto(payload, (self.host_ip, self.port))
    #     except Exception:
    #         pass

from line_following import LineFollowingNode

def main():
    rclpy.init()
    nodes = [PoseSender(), CameraSender(), CommandReceiver(), LineFollowingNode('line_following')]
    executor = rclpy.executors.MultiThreadedExecutor()
    for n in nodes:
        executor.add_node(n)

    time.sleep(3)
    run_command("ros2 service call /line_following/enter std_srvs/srv/Trigger '{}'", wait=False)
    time.sleep(3)
    run_command("ros2 service call /line_following/force_pick_color std_srvs/srv/Trigger '{}'", wait=False)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # send final stop
        twist = Twist()
        for n in nodes:
            if hasattr(n, 'cmd_vel_pub'):
                n.cmd_vel_pub.publish(twist)
            n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
