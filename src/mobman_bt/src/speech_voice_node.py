#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SpeechVoiceNode(Node):
    def __init__(self):
        super().__init__("speech_voice_node")
        self.publisher = self.create_publisher(String, "/voice_command", 10)
        self.declare_parameter("energy_threshold", 300)
        self.declare_parameter("pause_threshold", 0.8)

        try:
            import speech_recognition as sr
        except ImportError as exc:
            raise RuntimeError(
                "speech_recognition is not installed. Install a microphone/STT stack "
                "or use keyboard_voice_node.py."
            ) from exc

        self.sr = sr
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = int(
            self.get_parameter("energy_threshold").value
        )
        self.recognizer.pause_threshold = float(
            self.get_parameter("pause_threshold").value
        )
        self.microphone = sr.Microphone()
        self.timer = self.create_timer(0.1, self.listen_once)
        self.listening = False
        self.get_logger().info("Speech voice node ready. Say a fetch command.")

    def listen_once(self):
        if self.listening:
            return

        self.listening = True
        try:
            with self.microphone as source:
                audio = self.recognizer.listen(source, timeout=2.0, phrase_time_limit=5.0)
            text = self.recognizer.recognize_google(audio).strip()
        except self.sr.WaitTimeoutError:
            self.listening = False
            return
        except self.sr.UnknownValueError:
            self.get_logger().warn("Could not understand speech.")
            self.listening = False
            return
        except Exception as exc:
            self.get_logger().error(f"Speech recognition failed: {exc}")
            self.listening = False
            return

        if text:
            self.get_logger().info(f"Heard: {text}")
            self.publisher.publish(String(data=text))
        self.listening = False


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SpeechVoiceNode()
    except RuntimeError as exc:
        print(exc)
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
