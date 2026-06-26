#!/usr/bin/env python3
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardVoiceNode(Node):
    def __init__(self):
        super().__init__("keyboard_voice_node")
        self.publisher = self.create_publisher(String, "/voice_command", 10)
        self.thread = threading.Thread(target=self._input_loop, daemon=True)
        self.thread.start()
        self.get_logger().info(
            "Type commands like 'bring me the bottle' or 'fetch cup'."
        )

    def _input_loop(self):
        while rclpy.ok():
            try:
                text = input("voice> ").strip()
            except EOFError:
                return

            if text:
                self.publisher.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardVoiceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
