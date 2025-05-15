import os
import sys

# 현재 스크립트 기준 상대 경로에 설정 파일이 있다고 가정
sys.path.append(os.path.dirname(__file__))
from robot_config import ROBOT_ID

import socket
import json
import socketio  # pip install "python-socketio[client]"

# Socket.IO 클라이언트 생성
robot_id = ROBOT_ID
sio = socketio.Client()

# 연결 시 출력
@sio.event
def connect():
    print('Connected to server.')

@sio.event
def disconnect():
    print('Disconnected from server.')

# 서버 주소 (예: 로컬 서버는 'http://localhost:6789')
SERVER_URL = 'https://ccf9aec2f845.ngrok.app'

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
        sio.connect(SERVER_URL)
    except Exception as e:
        print(f"Failed to connect to server: {e}")
        return

    import threading

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

    threading.Thread(target=camera_loop, daemon=True).start()

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

            # 서버에 전송
            for i in range(1):
                sio.emit('robot_position', {
                    'robot_id': robot_id + i,
                    'pos': {
                        'x': x + 0.1 * i,
                        'y': y,
                        'yaw': yaw + 0.1 * i
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