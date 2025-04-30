#!/bin/bash

cd /home/pi/docker/tmp/sv_robot

# 2. Git 최신 코드 받아오기
echo "[INFO] Pulling latest code from GitHub..."
git pull origin main

# 3. 실행할 메인 프로그램 시작
echo "[INFO] Starting main robot process..."
# python3 init_robot.py  # 또는 실행할 스크립트

# 4. 로그 기록 등 추가
echo "[INFO] Robot startup completed."