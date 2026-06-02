import rclpy
from rclpy.node import Node
from rmf_task_msgs.msg import ApiResponse
import json
import sys

class TaskMonitor(Node):
    def __init__(self):
        super().__init__('task_monitor')
        from rclpy.qos import QoSProfile, DurabilityPolicy
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription = self.create_subscription(ApiResponse, '/task_api_responses', self.listener_callback, qos)

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.json_msg)
            print(json.dumps(data, indent=2))
        except Exception as e:
            print(f"Error parsing JSON: {e}")
            print(msg.json_msg)
        sys.exit(0)

def main():
    rclpy.init()
    node = TaskMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
