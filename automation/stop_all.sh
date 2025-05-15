#!/bin/bash
# stop_ros_monitor.sh

echo "[INFO] Stopping ros_monitor related Python processes..."

# ros_monitor 관련 python 프로세스만 골라서 종료
PIDS=$(ps aux | grep '[r]os_monitor' | grep 'python' | awk '{print $2}')

if [ -z "$PIDS" ]; then
    echo "[INFO] No ros_monitor processes found."
else
    echo "[INFO] Killing PIDs: $PIDS"
    kill -9 $PIDS
    echo "[INFO] All ros_monitor processes terminated."
fi
