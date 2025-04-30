import subprocess
import time
import os
import sys
import signal
import random

# 현재 스크립트 기준 상대 경로에 설정 파일이 있다고 가정
sys.path.append(os.path.dirname(__file__))
from robot_config import ROBOT_ID


# from robot_config import ROBOT_ID

def update_ros_domain_in_file(filepath: str, new_id: str):
    with open(filepath, 'r') as file:
        lines = file.readlines()

    with open(filepath, 'w') as file:
        for line in lines:
            if line.startswith("export ROS_DOMAIN_ID="):
                file.write(f"export ROS_DOMAIN_ID={new_id}\n")
            else:
                file.write(line)

def run_command(cmd, wait=True):
    print(f"[RUNNING] {cmd}")
    if wait:
        result = subprocess.run(cmd, shell=True)
        print(f"[DONE] Exit code: {result.returncode}")
        return result.returncode
    else:
        return subprocess.Popen(cmd, shell=True, preexec_fn=os.setpgrp)  # allow detaching from parent

def main():
    # 0. set ROS ID
    # os.environ["ROS_DOMAIN_ID"] = str(ROBOT_ID)
    update_ros_domain_in_file("/home/ubuntu/ros2_ws/.typerc", str(ROBOT_ID))

    # 1. stop existing ROS nodes
    run_command("~/.stop_ros.sh")
    time.sleep(1)

    # 2. launch navigation (run in background using subprocess)
    # launch_cmd = "source /home/ubuntu/ros2_ws/.zshrc && ros2 launch navigation navigation.launch.py map:=room_01"
    # launch_cmd = "source /home/ubuntu/ros2_ws/.zshrc && ros2 launch navigation navigation.launch.py map:=home_02"
    launch_cmd = "source /home/ubuntu/ros2_ws/.zshrc && ros2 launch navigation navigation.launch.py map:=lab_test use_teb:=true params_file:=nav2_controller_teb.yaml"
    # launch_cmd = "source /home/ubuntu/ros2_ws/.zshrc && ros2 launch nav2_bringup bringup_launch.py map:=~/ros2_ws/src/slam/maps/home_02 params_file:=nav2_controller_teb.yaml"
    nav_process = run_command(f"zsh -c '{launch_cmd}'", wait=False)

    # 3. wait for navigation to initialize
    print("Waiting 7 seconds for navigation stack to be ready...")
    time.sleep(10)

    # 4. call reinitialize global localization
    run_command("ros2 service call /reinitialize_global_localization std_srvs/srv/Empty")

    time.sleep(1)
    # run_command("python3 /home/ubuntu/shared/monitor/move_lidar.py")

    # (Optional) You can kill the launch process later if needed like this:
    # os.killpg(os.getpgid(nav_process.pid), signal.SIGTERM)

if __name__ == "__main__":
    main()