import os
import sys
import socket  # Importing socket module again
from datetime import datetime
import random
import time
import statistics
import threading

pose_x, pose_y, pose_yaw, speed_linear, speed_angular = 0.0, 0.0, 0.0, 0.0, 0.0

# 현재 스크립트 기준 상대 경로에 설정 파일이 있다고 가정
sys.path.append(os.path.dirname(__file__))
from robot_config.robot_config_id import ROBOT_ID
from robot_config.ap_config import AP_INFO

import json
import socketio  # pip install "python-socketio[client]"
import subprocess

# Socket.IO 클라이언트 생성
robot_id = ROBOT_ID
INTERFACE = "wlan0"
sio = socketio.Client(
    reconnection=True,
    reconnection_attempts=10,
    reconnection_delay=0.5,
    reconnection_delay_max=1
)

delay_samples = []
scan_lock = threading.Lock()
last_handover_time = 0

handover_mode = None
handover_thread = None
handover_stop_event = threading.Event()

#
# RSSI moving average 저장용
rssi_history = {}  # bssid: [rssi1, rssi2, ...]
MOVING_AVG_N = 2  # moving average window 축소(더 빠른 반영)


# Reconnect helper for socket.io
def reconnect_socket():
    for i in range(5):
        try:
            if sio.connected:
                # sio.disconnect()
                print("✅ Already connected to server.")
                return
            time.sleep(0.5)
            sio.connect(SERVER_URL, auth={'robot_id': str(robot_id)})
            print("✅ Reconnected to server after handover.")
            return True
        except Exception as e:
            print(f"Reconnect attempt {i+1} failed: {e}")
            time.sleep(2)
    print("❌ Failed to reconnect after handover.")
    return False

# Reconnect socket in background thread
def reconnect_socket_background():
    def attempt():
        reconnect_socket()
    threading.Thread(target=attempt, daemon=True).start()

@sio.event
def connect():
    print('Connected to server.')

@sio.event
def disconnect():
    print('Disconnected from server.')


@sio.event
def navigation(data):
    print(f"Received navigation data: {data}")
    if data.get('robot_id') == str(robot_id):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        docker_ip = '172.17.0.1'
        docker_port = 9002
        msg = json.dumps({'navigation': data}).encode()
        sock.sendto(msg, (docker_ip, docker_port))
        print('action', data.get('action'))


@sio.event
def command(data):
    print(data)
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

        handover = data.get('handover')
        if handover:
            handover_id = int(handover, 0)
            target_bssid = AP_INFO[handover_id]['bssid'].lower()
            print(f"[{robot_id}] Received handover request to BSSID: {target_bssid}")

            current_bssid = get_current_bssid()
            if current_bssid == target_bssid:
                print(f"[{robot_id}] Already connected to BSSID {current_bssid}. Skipping handover.")
                return

            # Try to acquire lock for handover
            acquired = scan_lock.acquire(timeout=5)
            if not acquired:
                print("Timeout: Unable to acquire scan lock for handover")
                return

            try:
                print(f"Performing handover to AP {handover_id}")
                handover_ap(target_bssid)
            finally:
                scan_lock.release()

# handover method
@sio.event
def handover_method(data):
    global handover_mode, handover_thread, handover_stop_event
    print(f"Received handover method: {data}")
    mode = data.get('mode')
    status = data.get('status')
    if status == 'on':
        # 기존 스레드 종료
        if handover_thread and handover_thread.is_alive():
            handover_stop_event.set()
            handover_thread.join()
        handover_stop_event = threading.Event()
        if mode == 'RSSI':
            handover_mode = 'RSSI'
            handover_thread = threading.Thread(target=handover_rssi_loop, args=(handover_stop_event,), daemon=True)
            handover_thread.start()
        elif mode == 'Random':
            handover_mode = 'Random'
            handover_thread = threading.Thread(target=handover_random_loop, args=(handover_stop_event,), daemon=True)
            handover_thread.start()
        elif mode == 'NeuroRAT':
            # NeuroRAT은 별도 처리, 기존 RSSI/Random off만
            handover_mode = 'NeuroRAT'
            # 기존 스레드 이미 종료됨
    elif status == 'off':
        if handover_mode == mode:
            if handover_thread and handover_thread.is_alive():
                handover_stop_event.set()
                handover_thread.join()
            handover_mode = None
            handover_thread = None

# RSSI 기반 handover 루프
THRESHOLD_RSSI = -70  # Random용 threshold, 필요시 조정

def handover_rssi_loop(stop_event):
    while not stop_event.is_set():
        rssi_map = get_rssi_map_from_scan_results()
        # HSLSV만 필터링
        candidates = [(bssid, rssi) for bssid, rssi in rssi_map.items() if any(ap['bssid'].lower() == bssid for ap in AP_INFO.values())]
        if candidates:
            # moving average 값으로 handover 판단
            best_bssid, best_rssi = max(candidates, key=lambda x: x[1])
            cur_bssid = get_current_bssid()
            if best_bssid != cur_bssid:
                print(f"[RSSI] Roaming to best BSSID: {best_bssid} (RSSI: {best_rssi})")
                handover_ap(best_bssid)
        stop_event.wait(3)

def handover_random_loop(stop_event):
    while not stop_event.is_set():
        rssi_map = get_rssi_map_from_scan_results()
        # threshold 이상, HSLSV만 필터링
        candidates = [bssid for bssid, rssi in rssi_map.items() if rssi > THRESHOLD_RSSI and any(ap['bssid'].lower() == bssid for ap in AP_INFO.values())]
        if candidates:
            import random
            target_bssid = random.choice(candidates)
            cur_bssid = get_current_bssid()
            if target_bssid != cur_bssid:
                print(f"[Random] Roaming to random BSSID: {target_bssid}")
                handover_ap(target_bssid)
        stop_event.wait(3)

# Measure RTT and Jitter
@sio.event
def ping_request(data):
    sio.emit('pong_reply')


def sensing_loop():
    gateway_list = list(AP_INFO.keys())
    while True:
        try:
            cur_bssid = get_current_bssid()
            cur_ap_id = get_ap_id_from_bssid(cur_bssid)
            rssi_map = get_rssi_map_from_scan_results()
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
                            "mac_address": AP_INFO[gw_id]['bssid'],
                            "connected": str(gw_id == cur_ap_id).lower(),
                            "rssi": rssi_map.get(AP_INFO[gw_id]['bssid'].lower(), -100),
                        }
                        for gw_id in gateway_list
                    ]
                }
            }

            if sio.connected:
                sio.emit('robot_ss_data', sensing_data)
            else:
                print("⚠️ Skipped emit: Socket.IO not connected")
            time.sleep(2.0)
        except Exception as e:
            print(f"Error in sensing loop: {e}")
            time.sleep(1)

def get_current_bssid():
    try:
        output = subprocess.check_output(["sudo", "wpa_cli", "status"], text=True)
        for line in output.splitlines():
            if line.startswith("bssid="):
                return line.split("=", 1)[1].strip().lower()
    except subprocess.CalledProcessError as e:
        print(f"Failed to get BSSID via wpa_cli: {e}")
    return None

def get_ap_id_from_bssid(bssid):
    try:
        for ap in AP_INFO.values():
            if ap['bssid'].lower() == bssid.lower():
                return ap['ap_id']
    except:
        print(f"Error getting AP ID for BSSID {bssid}")
        return None
    return None

def get_rssi_map_from_scan_results():
    global rssi_history
    try:
        output = subprocess.check_output(["sudo", "wpa_cli", "scan_results"], text=True)
    except subprocess.CalledProcessError as e:
        print(f"Failed to get scan results: {e}")
        return {}

    rssi_map = {}
    lines = output.splitlines()
    for line in lines[1:]:
        parts = line.split()
        if len(parts) >= 5:
            bssid, freq, signal, flags, ssid = parts[0], parts[1], parts[2], parts[3], " ".join(parts[4:])
            if ssid == "HSLSV":
                try:
                    rssi_val = float(signal)
                    # moving average 적용
                    if bssid not in rssi_history:
                        rssi_history[bssid] = []
                    rssi_history[bssid].append(rssi_val)
                    if len(rssi_history[bssid]) > MOVING_AVG_N:
                        rssi_history[bssid].pop(0)
                    if len(rssi_history[bssid]) < MOVING_AVG_N:
                        avg_rssi = rssi_history[bssid][-1]
                    else:
                        avg_rssi = sum(rssi_history[bssid]) / MOVING_AVG_N
                    rssi_map[bssid.lower()] = avg_rssi
                except ValueError:
                    continue
    return rssi_map

def lock_bssid(bssid):
    try:
        subprocess.run(["sudo", "wpa_cli", "set_network", "0", "bssid", bssid], check=True)
        subprocess.run(["sudo", "wpa_cli", "set_network", "0", "bgscan", ""], check=True)
        print(f"[LOCK] BSSID locked to {bssid} and bgscan disabled")
    except subprocess.CalledProcessError as e:
        print(f"[LOCK] Failed to lock BSSID or disable bgscan: {e}")

def handover_ap(target_bssid):
    global last_handover_time
    try:
        subprocess.run(["sudo", "wpa_cli", "roam", target_bssid], check=True)
        lock_bssid(target_bssid)
        print(f"Successfully handed over to BSSID: {target_bssid}")
        last_handover_time = time.time()

        # BSSID 전환 확인
        for _ in range(15):
            bssid = get_current_bssid()
            if bssid and bssid == target_bssid:
                print(f"Confirmed BSSID after roam: {bssid}")
                break
            print("Waiting for BSSID confirmation...")
            time.sleep(0.1)
        else:
            print(f"Warning: BSSID {target_bssid} not confirmed after roam.")

        # 네트워크 연결 확인 (고정 IP 환경)
        for _ in range(5):
            try:
                subprocess.check_output(["ping", "-c", "1", "-W", "1", "10.243.76.1"], stderr=subprocess.DEVNULL)
                print("Network connectivity confirmed after roam.")
                break
            except subprocess.CalledProcessError:
                print("Waiting for network availability...")
                time.sleep(0.1)

        # socket.io 강제 reconnect
        reconnect_socket_background()

        # handover 후 scan 즉시 실행 (RSSI 최신화)
        try:
            subprocess.run(["sudo", "wpa_cli", "scan", "freq", "5180"], check=True)
            print("[Handover] Immediate scan after handover.")
        except Exception as e:
            print(f"[Handover] Immediate scan error: {e}")



    except subprocess.CalledProcessError as e:
        print(f"Error during handover: {e}")
    except Exception as e:
        print(f"Unexpected error in handover_ap: {e}")

def scan_loop():
    global last_handover_time
    while True:
        if time.time() - last_handover_time < 3:
            time.sleep(0.5)
            continue
        if scan_lock.acquire(blocking=False):
            try:
                # print("[SCAN] Starting scan")
                subprocess.run(["sudo", "wpa_cli", "scan", "freq", "5180"], check=True)
                time.sleep(1.0)  # scan 후 대기시간 단축
            except subprocess.CalledProcessError as e:
                print(f"Scan error: {e}")
            finally:
                scan_lock.release()
                # print("[SCAN] Scan complete")
        time.sleep(0.5)

# SERVER_URL = 'https://0cd8b22324dc.ngrok.app'
# SERVER_URL_RTP = 'https://ccf9aec2f845.ngrok.app:5002'
SERVER_URL = 'http://10.243.76.27:6789'

def socketio_reconnect_watchdog():
    while True:
        if not sio.connected:
            print("[Watchdog] Socket.IO not connected. Trying to reconnect...")
            reconnect_socket()
        time.sleep(3)

def main():
    host_ip = '0.0.0.0'  # 모든 인터페이스에서 수신
    port = 9000

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host_ip, port))
    print(f"Listening for position data on port {port}...")

    # socket.io 서버 연결
    try:
        # sio.connect(SERVER_URL)
        sio.connect(SERVER_URL, auth={'robot_id': str(robot_id)}) # handshake with robot_id
    except Exception as e:
        print(f"Failed to connect to server: {e}")
        return

    import threading

    threading.Thread(target=sensing_loop, daemon=True).start()
    threading.Thread(target=scan_loop, daemon=True).start()
    threading.Thread(target=socketio_reconnect_watchdog, daemon=True).start()

    # 수신 및 전송 루프
    while True:
        try:
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

                global pose_x, pose_y, pose_yaw, speed_linear, speed_angular
                pose_x = x if x is not None else 0.0
                pose_y = y if y is not None else 0.0
                pose_yaw = yaw if yaw is not None else 0.0
                speed_linear = linear_speed if linear_speed is not None else 0.0
                speed_angular = angular_speed if angular_speed is not None else 0.0

                # 서버에 전송
                if sio.connected:
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
                else:
                    print("⚠️ Skipped emit: Socket.IO not connected")

            except Exception as e:
                print(f"Error decoding or sending data: {e}")
        except Exception as e:
            print(f"Error in main UDP loop: {e}")
            time.sleep(1)

    print("ros_monitor_socket_sensing.py 종료됨")

if __name__ == '__main__':
    main()