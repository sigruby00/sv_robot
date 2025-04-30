import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
import json
import time
import os

class SaveMapOnce(Node):
    def __init__(self):
        super().__init__('save_map_once')
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.callback,
            qos
        )
        # self.subscription = self.create_subscription(OccupancyGrid, '/map', self.callback, 10)
        self.get_logger().info('Subscription to /map is active.')

    def callback(self, msg):
        self.get_logger().info('Callback triggered for map data.')

        self.get_logger().info('Saving full map to map_raw.json')

        # OccupancyGrid → dict 변환
        data = {
            "resolution": msg.info.resolution,
            "width": msg.info.width,
            "height": msg.info.height,
            "origin": {
                "x": msg.info.origin.position.x,
                "y": msg.info.origin.position.y
            },
            "data": list(msg.data)
        }

        print(data, flush=True)

        # 이 스크립트 파일과 같은 디렉토리에 저장
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # output_path = os.path.join(script_dir, "map_raw_home.json")
        output_path = os.path.join(script_dir, "map_raw_lab_test.json")

        with open(output_path, "w") as f:
            json.dump(data, f)

        self.get_logger().info(f'✅ Saved to {output_path}')
        rclpy.shutdown()

def main():
    rclpy.init()
    time.sleep(2)
    node = SaveMapOnce()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()