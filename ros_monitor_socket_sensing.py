import os
import sys
import socket  # Importing socket module again
from datetime import datetime
import random
import time

pose_x, pose_y, pose_yaw, speed_linear, speed_angular = 0.0, 0.0, 0.0, 0.0, 0.0

# 현재 스크립트 기준 상대 경로에 설정 파일이 있다고 가정
sys.path.append(os.path.dirname(__file__))
from robot_config.robot_config_id import ROBOT_ID

import json
import socketio  # pip install "python-socketio[client]"
import subprocess

# Socket.IO 클라이언트 생성
robot_id = ROBOT_ID
sio = socketio.Client()

@sio.event
def connect():
    print('Connected to server.')

@sio.event
def disconnect():
    print('Disconnected from server.')

@sio.event
def command(data):
    if data.get('robot_id') == str(robot_id):
        print(f"[{robot_id}] Received command: {data}")
        action = data.get('action')
        if action:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                docker_ip = '172.17.0.1'
                docker_port = 9002
                msg = json.dumps({'action': action}).encode()
                sock.sendto(msg, (docker_ip, docker_port))
                print(f"Forwarded command to Docker: {action}")
            except Exception as e:
                print(f"Failed to forward command to Docker: {e}")

def sensing_loop():
    gateways = [1, 2, 3, 4]
    while True:
        connected_gateway = random.choice(gateways)  # Dummy: randomly pick one as connected
        time_now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        sensing_data = {
            "timestamp": time_now,
            "data": {
                "ca_id": robot_id,
                "location": {
                    "x": pose_x,
                    "y": pose_y,
                    "t": pose_yaw
                },
                "imu": {
                    "linear_speed": speed_linear,
                    "angular_speed": speed_angular,
                },
                "connections": [
                    {
                        "gateway_id": gw_id,
                        "mac_address": f"00:00:00:00:00:{gw_id:02}",
                        "connected": str(gw_id == connected_gateway).lower(),
                        "rssi": f"{-random.randint(40, 90)}",
                        "throughput": round(random.uniform(0.0, 100.0), 2) if gw_id == connected_gateway else 0.0,
                        "delay": round(random.uniform(0.0, 100.0), 2) if gw_id == connected_gateway else 0.0,
                        "jitter": round(random.uniform(0.0, 100.0), 2) if gw_id == connected_gateway else 0.0
                    }
                    for gw_id in gateways
                ]
            }
        }

        sio.emit('robot_ss_data', sensing_data)
        print(f"Sent sensing data from robot {robot_id}")
        time.sleep(3)

# SERVER_URL = 'https://0cd8b22324dc.ngrok.app'
# SERVER_URL_RTP = 'https://ccf9aec2f845.ngrok.app:5002'
SERVER_URL = 'http://10.243.76.27:6789'

def main():
    host_ip = '0.0.0.0'  # 모든 인터페이스에서 수신
    port = 9000

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host_ip, port))
    print(f"Listening for position data on port {port}...")

    # 추가 카메라용 소켓
    cam_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cam_sock.bind((host_ip, 9001))
    print("Listening for camera data on port 9001...")

    # socket.io 서버 연결
    try:
        # sio.connect(SERVER_URL)
        sio.connect(SERVER_URL, auth={'robot_id': str(robot_id)}) # handshake with robot_id
    except Exception as e:
        print(f"Failed to connect to server: {e}")
        return

    import threading

    # web-socket based streaming
    def camera_loop():
        while True:
            data, addr = cam_sock.recvfrom(100000)
            try:
                decoded = json.loads(data.decode())
                image = decoded.get('image')
                if image:
                    sio.emit('robot_image', {
                        'robot_id': robot_id,
                        'image': image
                    })
                    print(f"Send image to server from robot {robot_id}")
            except Exception as e:
                print(f"Error decoding camera data: {e}")

    # 카메라 루프를 별도의 스레드에서 실행
    threading.Thread(target=camera_loop, daemon=True).start()
    threading.Thread(target=sensing_loop, daemon=True).start()


    # 수신 및 전송 루프
    while True:
        data, addr = sock.recvfrom(1024)
        try:
            decoded = json.loads(data.decode())
            pos = decoded.get('pos', {})
            x = pos.get('x')
            y = pos.get('y')
            yaw = pos.get('yaw')
            battery = decoded.get('battery')

            imu = decoded.get('imu', {})
            linear_speed = imu.get('linear_speed')
            angular_speed = imu.get('angular_speed')
            linear_acc = imu.get('linear_acceleration', {})
            angular_vel = imu.get('angular_velocity', {})

            print(f"robot_id: {robot_id}")
            print(f"Pos X: {x:.3f}")
            print(f"Pos Y: {y:.3f}")
            print(f"Pos Yaw: {yaw:.3f}")
            print(f"Battery Level: {battery:.2f}%" if battery is not None else "Battery Level: N/A")
            if linear_speed is not None:
                print(f"IMU Speed - Linear: {linear_speed:.3f} m/s")
            if angular_speed is not None:
                print(f"IMU Speed - Angular: {angular_speed:.3f} rad/s")
            if linear_acc:
                print(f"Linear Acc: x={linear_acc.get('x', 0.0):.3f}, y={linear_acc.get('y', 0.0):.3f}, z={linear_acc.get('z', 0.0):.3f}")
            if angular_vel:
                print(f"Angular Vel: x={angular_vel.get('x', 0.0):.3f}, y={angular_vel.get('y', 0.0):.3f}, z={angular_vel.get('z', 0.0):.3f}")

            global pose_x, pose_y, pose_yaw, speed_linear, speed_angular
            pose_x = x if x is not None else 0.0
            pose_y = y if y is not None else 0.0
            pose_yaw = yaw if yaw is not None else 0.0
            speed_linear = linear_speed if linear_speed is not None else 0.0
            speed_angular = angular_speed if angular_speed is not None else 0.0


            # 서버에 전송
            sio.emit('robot_rt_data', {
                'robot_id': robot_id,
                'pos': {
                    'x': x + 0.1,
                    'y': y,
                    'yaw': yaw + 0.1
                },
                'battery': battery,
                'imu': {
                    'linear_acceleration': linear_acc,
                    'angular_velocity': angular_vel,
                    'linear_speed': linear_speed,
                    'angular_speed': angular_speed
                }
            })

        except Exception as e:
            print(f"Error decoding or sending data: {e}")

if __name__ == '__main__':
    main()