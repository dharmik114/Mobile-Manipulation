#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class MockPickService(Node):
    def __init__(self):
        super().__init__("mock_pick_service")
        self.service = self.create_service(Trigger, "/pick_object", self.pick_callback)
        self.get_logger().info("Mock /pick_object service ready.")

    def pick_callback(self, request, response):
        del request
        self.get_logger().info("Pretending to pick object.")
        time.sleep(1.0)
        response.success = True
        response.message = "Mock pick completed"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockPickService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
