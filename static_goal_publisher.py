import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration

class StaticGoalPublisher(Node):
    def __init__(self):
        super().__init__('static_goal_publisher')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 두 개의 왕복 위치
        self.goals = [
            (-3.0, -1.5),
            (-2.954, -0.45)
        ]
        self.current_index = 0

        self._client.wait_for_server()
        self.send_goal()

    def send_goal(self):
        x, y = self.goals[self.current_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending goal to ({x:.2f}, {y:.2f})")
        self._client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected')
            return

        self.get_logger().info('Goal accepted')
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal reached ✅')

        # 다음 목표로 인덱스 전환
        self.current_index = (self.current_index + 1) % len(self.goals)

        # 2초 후 다음 goal 전송
        self.create_timer(2.0, self.send_goal, callback_group=None)

def main(args=None):
    rclpy.init(args=args)
    node = StaticGoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()