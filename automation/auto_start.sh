#!/bin/bash
cd /home/pi/docker/tmp/sv_robot

# 1. 최신 코드 pull (필요시 주석 해제)
echo "[INFO] Pulling latest code from GitHub..."
git pull origin main
sleep 5

# 맵 파일 ROS2 워크스페이스로 동기화
echo "[INFO] Syncing maps to ROS2 workspace..."
docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -c "mkdir -p /home/ubuntu/ros2_ws/src/slam/maps && cp -r /home/ubuntu/shared/sv_robot/map_info/* /home/ubuntu/ros2_ws/src/slam/maps/"

docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -c "~/.stop_ros.sh"

# 2. navigation 및 AMCL 등 메인 ROS2 스택 실행 (init_robot.py는 navigation만 실행)
echo "[INFO] Starting navigation stack (init_robot.py)..."
docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -c "source /home/ubuntu/ros2_ws/.zshrc; python3 /home/ubuntu/shared/sv_robot/init_robot.py > /tmp/init_robot.log 2>&1 &"


# # 3-2. AMCL get_state 서비스 대기
# echo "[INFO] Waiting for /get_state service to be available..."
# while true; do
#     docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -c "source /home/ubuntu/ros2_ws/.zshrc; ros2 service list | grep /get_state" > /dev/null 2>&1
#     if [ $? -eq 0 ]; then
#         echo "[INFO] /get_state service is available."
#         break
#     fi
#     sleep 2
# done

# # 5. 카메라 토픽 대기
# CAMERA_TOPIC="ascamera/camera_publisher/rgb0/image"
# echo "[INFO] Waiting for $CAMERA_TOPIC topic to be available..."
# while true; do
#     docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -c "source ~/.zshrc; ros2 topic list | grep $CAMERA_TOPIC" > /dev/null 2>&1
#     if [ $? -eq 0 ]; then
#         echo "[INFO] $CAMERA_TOPIC topic is available."
#         break
#     fi
#     sleep 2
# done

# # 6. 라이다 토픽 대기
# LIDAR_TOPIC="/scan_raw"
# echo "[INFO] Waiting for $LIDAR_TOPIC topic to be available..."
# while true; do
#     docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -c "source ~/.zshrc; ros2 topic list | grep $LIDAR_TOPIC" > /dev/null 2>&1
#     if [ $? -eq 0 ]; then
#         echo "[INFO] $LIDAR_TOPIC topic is available."
#         break
#     fi
#     sleep 2
# done

# # 7. mecanum 제어 토픽 대기
# CMDVEL_TOPIC="/controller/cmd_vel"
# echo "[INFO] Waiting for $CMDVEL_TOPIC topic to be available..."
# while true; do
#     docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -c "source ~/.zshrc; ros2 topic list | grep $CMDVEL_TOPIC" > /dev/null 2>&1
#     if [ $? -eq 0 ]; then
#         echo "[INFO] $CMDVEL_TOPIC topic is available."
#         break
#     fi
#     sleep 2
# done

# 8. line_following 서비스 준비 확인
# echo "[INFO] Waiting for /line_following/init_finish service..."
# while true; do
#     docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -c "source ~/.zshrc; ros2 service list | grep /line_following/init_finish" > /dev/null 2>&1
#     if [ $? -eq 0 ]; then
#         echo "[INFO] /line_following/init_finish service is available."
#         break
#     fi
#     sleep 2
# done

# 9. ros_monitor_docker.py 실행
echo "[INFO] Starting ros_monitor_docker.py..."
docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -c "source ~/.zshrc; python3 /home/ubuntu/shared/sv_robot/ros_monitor_docker.py > /tmp/ros_monitor.log 2>&1 &"
sleep 2

# 10. socket monitor 실행 (로그 버퍼링 해제, 환경/시간 정보 추가)
echo "[$(date)] ENV: $PATH" >> /tmp/ros_monitor_socket.log
echo "[$(date)] PWD: $(pwd)" >> /tmp/ros_monitor_socket.log
exec /bin/zsh -c "source /home/pi/.zshrc; /usr/bin/python3 -u /home/pi/docker/tmp/sv_robot/ros_monitor_socket_sensing.py >> /tmp/ros_monitor_socket.log 2>&1"