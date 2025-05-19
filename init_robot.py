import subprocess
import time
import os
import sys
import signal
import random

USE_PURE_PURSUIT = True  # Set to False to use TEB instead

# 현재 스크립트 기준 상대 경로에 설정 파일이 있다고 가정
sys.path.append(os.path.dirname(__file__))
from robot_config.robot_config_id import ROBOT_ID
from robot_config.robot_config_loc import INITIAL_POSES


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

# def run_command(cmd, wait=True):
#     print(f"[RUNNING] {cmd}")
#     if wait:
#         result = subprocess.run(cmd, shell=True)
#         print(f"[DONE] Exit code: {result.returncode}")
#         return result.returncode
#     else:
#         return subprocess.Popen(cmd, shell=True, preexec_fn=os.setpgrp)  # allow detaching from parent

def run_command(cmd, wait=True):
    print(f"[RUNNING] {cmd}")
    if wait:
        result = subprocess.run(cmd, shell=True, executable='/bin/bash')
        print(f"[DONE] Exit code: {result.returncode}")
        return result.returncode
    else:
        return subprocess.Popen(cmd, shell=True, preexec_fn=os.setpgrp, executable='/bin/bash')


def wait_for_service(service_name, timeout=60):
    """
    service_name: 예) '/amcl/get_state'
    timeout: 최대 대기 시간(초)
    """
    import time, subprocess
    start = time.time()
    while time.time() - start < timeout:
        try:
            result = subprocess.run(f"ros2 service list | grep '{service_name}'", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, executable='/bin/bash')
            if result.returncode == 0:
                print(f"Service {service_name} is available.")
                return True
        except Exception as e:
            print(f"Error while waiting for service: {e}")
        print(f"Waiting for {service_name} ...")
        time.sleep(1)
    print(f"Timeout: {service_name} not available after {timeout} seconds.")
    return False


def main():
    # 0. set ROS ID
    # os.environ["ROS_DOMAIN_ID"] = str(ROBOT_ID)
    update_ros_domain_in_file("/home/ubuntu/ros2_ws/.typerc", str(ROBOT_ID))

    pose = INITIAL_POSES.get(ROBOT_ID)
    if pose:


        # Set EKF frequency
        run_command("ros2 param set /ekf_filter_node frequency 30.0")

        # Start map server
        run_command("ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/ubuntu/shared/sv_robot/map_info/lab_test.yaml", wait=False)
        time.sleep(10)

        # Configure and activate map_server lifecycle
        run_command("ros2 lifecycle set /map_server configure")
        time.sleep(10)
        run_command("ros2 lifecycle set /map_server activate")
        time.sleep(5)

        import math
        def quaternion_to_yaw(z, w):
            return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

        yaw = quaternion_to_yaw(pose['ori_z'], pose['ori_w'])

        static_tf_cmd = (
            f"ros2 run tf2_ros static_transform_publisher "
            f"{pose['x']} {pose['y']} 0 0 0 {yaw} map odom"
        )
        run_command(static_tf_cmd, wait=False)
    else:
        print(f"Warning: No initial pose found for robot ID {ROBOT_ID}")


    # 1. stop existing ROS nodes
    # run_command("~/.stop_ros.sh")
    # run_command("/home/ubuntu/.stop_ros.sh", wait=True)
    # time.sleep(5)
    #

    # 2. launch navigation (run in background using subprocess)
    # launch_cmd = "source /home/ubuntu/ros2_ws/.zshrc && ros2 launch navigation navigation.launch.py map:=room_01"
    # launch_cmd = "source /home/ubuntu/ros2_ws/.zshrc && ros2 launch navigation navigation.launch.py map:=home_02"
    # params_file = "nav2_controller_pure_pursuit.yaml" if USE_PURE_PURSUIT else "nav2_controller_teb.yaml"
    # params_file = "~/ros2_ws/src/navigation/config/nav2_params.yaml"
    # params_file = "~/ros2_ws/src/navigation/config/nav2_controller_teb.yaml"
    # map_file = "/home/ubuntu/shared/sv_robot/map_info/lab_test"
    # launch_cmd = f"source /home/ubuntu/ros2_ws/.zshrc && ros2 launch navigation navigation.launch.py map:=lab_test tab:=true params_file:={params_file}"
    # launch_cmd = f"source /home/ubuntu/ros2_ws/.zshrc && ros2 launch navigation navigation.launch.py map:=lab_test"
    # launch_cmd = f"source /home/ubuntu/ros2_ws/.zshrc && ros2 launch navigation navigation.launch.py map:={map_file} tab:=true params_file:={params_file}"
    # launch_cmd = f"source /home/ubuntu/ros2_ws/.zshrc && ros2 launch navigation navigation.launch.py map:={map_file} tab:=true params_file:={params_file}"
    # launch_cmd = f"source /home/ubuntu/ros2_ws/.zshrc && ros2 launch navigation navigation.launch.py map:={map_file}"
    # launch_cmd = "source /home/ubuntu/ros2_ws/.zshrc && ros2 launch nav2_bringup bringup_launch.py map:=~/ros2_ws/src/slam/maps/home_02 params_file:=nav2_controller_teb.yaml"

    # launch_cmd = f"ros2 run nav2_map_server map_server --ros-args -r map:=lab_test"
    # launch_cmd = f"ros2 run nav2_map_server map_server --ros-args -r map:=lab_test -p use_sim_time:=true --params-file /home/ubuntu/ros2_ws/src/navigation/config/nav2_params.yaml"
    # nav_process = run_command(f"zsh -c '{launch_cmd}'", wait=False)


    # navigation stack이 완전히 뜰 때까지 대기 (/amcl/get_state 서비스 등장 대기)
    # wait_for_service('/amcl/get_state', timeout=90)

    # 4. call reinitialize global localization
    # run_command("ros2 service call /reinitialize_global_localization std_srvs/srv/Empty", wait=False)
    # time.sleep(5)

    # import rclpy
    # from rclpy.node import Node
    # from geometry_msgs.msg import PoseWithCovarianceStamped
    # from lifecycle_msgs.srv import GetState

    # class InitialposePublisher(Node):
    #     def __init__(self, x, y, ori_z, ori_w):
    #         super().__init__('initialpose_publisher')
    #         self.x = x
    #         self.y = y
    #         self.ori_z = ori_z
    #         self.ori_w = ori_w

    #         self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    #         self.cli = self.create_client(GetState, '/amcl/get_state')
    #         self.timer = self.create_timer(1.0, self.check_amcl_state)

    #     def check_amcl_state(self):
    #         if not self.cli.wait_for_service(timeout_sec=1.0):
    #             self.get_logger().info('Waiting for /amcl/get_state service...')
    #             return

    #         self.get_logger().info('Requesting AMCL state...')
    #         request = GetState.Request()
    #         future = self.cli.call_async(request)
    #         future.add_done_callback(self.handle_amcl_state_response)

    #     def handle_amcl_state_response(self, future):
    #         try:
    #             response = future.result()
    #             state = response.current_state.label
    #             self.get_logger().info(f'AMCL current state: {state}')
    #             if state == 'active':
    #                 self.publish_initial_pose()
    #                 self.destroy_timer(self.timer)
    #                 rclpy.shutdown()
    #         except Exception as e:
    #             self.get_logger().error(f'Failed to get AMCL state: {e}')

    #     def publish_initial_pose(self):
    #         msg = PoseWithCovarianceStamped()
    #         msg.header.stamp = self.get_clock().now().to_msg()
    #         msg.header.frame_id = 'map'
    #         msg.pose.pose.position.x = self.x
    #         msg.pose.pose.position.y = self.y
    #         msg.pose.pose.position.z = 0.0
    #         msg.pose.pose.orientation.x = 0.0
    #         msg.pose.pose.orientation.y = 0.0
    #         msg.pose.pose.orientation.z = self.ori_z
    #         msg.pose.pose.orientation.w = self.ori_w
    #         msg.pose.covariance = [0.0] * 36
    #         self.pose_pub.publish(msg)
    #         self.get_logger().info('Initial pose published')

    # pose = INITIAL_POSES.get(ROBOT_ID)
    # if pose:
    #     rclpy.init()
    #     node = InitialposePublisher(pose['x'], pose['y'], pose['ori_z'], pose['ori_w'])
    #     rclpy.spin(node)
    # else:
    #     print(f"Warning: No initial pose found for robot ID {ROBOT_ID}")

    # time.sleep(1)
    # run_command("python3 /home/ubuntu/shared/monitor/move_lidar.py")


if __name__ == "__main__":
    main()