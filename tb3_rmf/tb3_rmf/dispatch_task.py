#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rmf_task_msgs.msg import ApiRequest
import json
import uuid
import sys

class TaskDispatcher(Node):
    def __init__(self):
        super().__init__('task_dispatcher')
        from rclpy.qos import QoSProfile, DurabilityPolicy
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(ApiRequest, '/task_api_requests', qos)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.sent = False

    def timer_callback(self):
        if self.sent:
            return

        # Get target from command line or default to tb3_dock
        target_waypoint = sys.argv[1] if len(sys.argv) > 1 else "tb3_dock"
        robot_name = sys.argv[2] if len(sys.argv) > 2 else "tb1"

        task_id = f"task_{uuid.uuid4().hex[:8]}"
        
        import time
        # If a robot is specified, use robot_task_request to force it
        if len(sys.argv) > 2:
            self.get_logger().info(f"Forcing task for robot: {robot_name}")
            task_request = {
                "type": "robot_task_request",
                "robot": robot_name,
                "fleet": "turtlebot3",
                "request": {
                    "unix_millis_request_time": int(time.time() * 1000),
                    "requester": "antigravity",
                    "category": "compose",
                    "description": {
                        "phases": [
                            {
                                "activity": {
                                    "category": "go_to_place",
                                    "description": target_waypoint
                                }
                            }
                        ]
                    }
                }
            }
        else:
            # RMF Task Dispatch Request JSON (Full Humble Schema)
            task_request = {
                "type": "dispatch_task_request",
                "request": {
                    "unix_millis_request_time": int(time.time() * 1000),
                    "requester": "antigravity",
                    "fleet_name": "turtlebot3",
                    "category": "compose",
                    "description": {
                        "phases": [
                            {
                                "activity": {
                                    "category": "go_to_place",
                                    "description": target_waypoint
                                }
                            }
                        ]
                    }
                }
            }

        msg = ApiRequest()
        msg.request_id = task_id
        msg.json_msg = json.dumps(task_request)
        
        self.publisher.publish(msg)
        self.get_logger().info(f"Published task request {task_id}: {robot_name} -> {target_waypoint}")
        self.sent = True
        
        # Give it a moment to send then exit
        self.create_timer(2.0, lambda: sys.exit(0))

def main():
    rclpy.init()
    node = TaskDispatcher()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
