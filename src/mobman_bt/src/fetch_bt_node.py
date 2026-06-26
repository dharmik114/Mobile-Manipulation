#!/usr/bin/env python3
import math
import re
from enum import Enum

import rclpy
import tf2_geometry_msgs  # Registers PointStamped transforms.
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Status(Enum):
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3


class Sequence:
    def __init__(self, children):
        self.children = children
        self.index = 0

    def reset(self):
        self.index = 0
        for child in self.children:
            child.reset()

    def tick(self):
        while self.index < len(self.children):
            status = self.children[self.index].tick()
            if status == Status.SUCCESS:
                self.index += 1
                continue
            return status
        return Status.SUCCESS


class SearchForObject:
    def __init__(self, node):
        self.node = node
        self.started_at = None

    def reset(self):
        self.started_at = None

    def tick(self):
        now = self.node.get_clock().now()
        if self.started_at is None:
            self.started_at = now
            self.node.get_logger().info(
                f"Searching for '{self.node.target_label or 'any object'}'."
            )

        if self.node.has_fresh_matching_detection():
            self.node.stop_base()
            self.node.get_logger().info("Object detected.")
            return Status.SUCCESS

        elapsed = (now - self.started_at).nanoseconds * 1e-9
        if elapsed > self.node.search_timeout_sec:
            self.node.stop_base()
            self.node.get_logger().error("Search timed out.")
            return Status.FAILURE

        twist = Twist()
        twist.angular.z = self.node.search_angular_z
        self.node.cmd_vel_pub.publish(twist)
        return Status.RUNNING


class NavigateNearObject:
    def __init__(self, node):
        self.node = node
        self.sent = False
        self.result_future = None
        self.goal_handle = None
        self.rejected = False

    def reset(self):
        self.sent = False
        self.result_future = None
        self.goal_handle = None
        self.rejected = False

    def tick(self):
        if not self.sent:
            goal = self.node.make_object_nav_goal()
            if goal is None:
                return Status.FAILURE
            self.node.get_logger().info("Navigating near detected object.")
            if not self.node.nav_client.wait_for_server(timeout_sec=0.1):
                self.node.get_logger().warn("Waiting for Nav2 action server.")
                return Status.RUNNING
            send_future = self.node.nav_client.send_goal_async(goal)
            send_future.add_done_callback(self._goal_response)
            self.sent = True
            return Status.RUNNING

        if self.rejected:
            return Status.FAILURE

        if self.result_future is None:
            return Status.RUNNING

        if not self.result_future.done():
            return Status.RUNNING

        result = self.result_future.result()
        if result is not None and result.status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info("Navigation to object succeeded.")
            return Status.SUCCESS

        self.node.get_logger().error("Navigation to object failed.")
        return Status.FAILURE

    def _goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.node.get_logger().error("Nav2 rejected object goal.")
            self.rejected = True
            return
        self.result_future = self.goal_handle.get_result_async()


class PickObject:
    def __init__(self, node):
        self.node = node
        self.future = None

    def reset(self):
        self.future = None

    def tick(self):
        if self.future is None:
            self.node.get_logger().info("Calling /pick_object.")
            if not self.node.pick_client.wait_for_service(timeout_sec=0.1):
                self.node.get_logger().warn("Waiting for /pick_object service.")
                return Status.RUNNING
            self.future = self.node.pick_client.call_async(Trigger.Request())
            return Status.RUNNING

        if not self.future.done():
            return Status.RUNNING

        response = self.future.result()
        if response is not None and response.success:
            self.node.get_logger().info(f"Pick succeeded: {response.message}")
            return Status.SUCCESS

        message = response.message if response is not None else "no response"
        self.node.get_logger().error(f"Pick failed: {message}")
        return Status.FAILURE


class ReturnHome:
    def __init__(self, node):
        self.node = node
        self.sent = False
        self.result_future = None
        self.goal_handle = None
        self.rejected = False

    def reset(self):
        self.sent = False
        self.result_future = None
        self.goal_handle = None
        self.rejected = False

    def tick(self):
        if not self.sent:
            self.node.get_logger().info("Returning to base.")
            if not self.node.nav_client.wait_for_server(timeout_sec=0.1):
                self.node.get_logger().warn("Waiting for Nav2 action server.")
                return Status.RUNNING
            send_future = self.node.nav_client.send_goal_async(self.node.make_home_goal())
            send_future.add_done_callback(self._goal_response)
            self.sent = True
            return Status.RUNNING

        if self.rejected:
            return Status.FAILURE

        if self.result_future is None:
            return Status.RUNNING

        if not self.result_future.done():
            return Status.RUNNING

        result = self.result_future.result()
        if result is not None and result.status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info("Returned to base.")
            return Status.SUCCESS

        self.node.get_logger().error("Return-to-base navigation failed.")
        return Status.FAILURE

    def _goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.node.get_logger().error("Nav2 rejected home goal.")
            self.rejected = True
            return
        self.result_future = self.goal_handle.get_result_async()


class FetchBTNode(Node):
    def __init__(self):
        super().__init__("fetch_bt_node")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("detection_topic", "/detected_object_3d")
        self.declare_parameter("label_topic", "/detected_object_label")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("navigate_action", "/navigate_to_pose")
        self.declare_parameter("pick_service", "/pick_object")
        self.declare_parameter("detection_timeout_sec", 1.5)
        self.declare_parameter("search_timeout_sec", 45.0)
        self.declare_parameter("search_angular_z", 0.35)
        self.declare_parameter("object_standoff_m", 0.55)
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("home_yaw", 0.0)

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.detection_timeout_sec = float(
            self.get_parameter("detection_timeout_sec").value
        )
        self.search_timeout_sec = float(self.get_parameter("search_timeout_sec").value)
        self.search_angular_z = float(self.get_parameter("search_angular_z").value)
        self.object_standoff_m = float(self.get_parameter("object_standoff_m").value)
        self.home_x = float(self.get_parameter("home_x").value)
        self.home_y = float(self.get_parameter("home_y").value)
        self.home_yaw = float(self.get_parameter("home_yaw").value)

        self.target_label = None
        self.latest_detection = None
        self.latest_detection_time = None
        self.latest_label = None
        self.active = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        self.create_subscription(String, "/voice_command", self.voice_callback, 10)
        self.create_subscription(
            PointStamped,
            self.get_parameter("detection_topic").value,
            self.detection_callback,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter("label_topic").value,
            self.label_callback,
            10,
        )

        self.nav_client = ActionClient(
            self, NavigateToPose, self.get_parameter("navigate_action").value
        )
        self.pick_client = self.create_client(
            Trigger, self.get_parameter("pick_service").value
        )

        self.tree = Sequence([
            SearchForObject(self),
            NavigateNearObject(self),
            PickObject(self),
            ReturnHome(self),
        ])
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info("Fetch BT ready. Waiting for /voice_command.")

    def voice_callback(self, msg):
        target = self.parse_target_label(msg.data)
        self.target_label = target
        self.active = True
        self.tree.reset()
        self.get_logger().info(
            f"Voice command accepted: '{msg.data}', target='{target or 'any'}'."
        )

    def detection_callback(self, msg):
        self.latest_detection = msg
        self.latest_detection_time = self.get_clock().now()

    def label_callback(self, msg):
        self.latest_label = msg.data.strip().lower()

    def tick(self):
        if not self.active:
            return

        status = self.tree.tick()
        if status == Status.SUCCESS:
            self.stop_base()
            self.active = False
            self.get_logger().info("Fetch task completed.")
        elif status == Status.FAILURE:
            self.stop_base()
            self.active = False
            self.get_logger().error("Fetch task failed.")

    def parse_target_label(self, text):
        cleaned = text.lower().strip()
        cleaned = re.sub(r"[^a-z0-9 ]+", " ", cleaned)
        for phrase in ["bring me", "bring", "fetch", "get", "pick up", "find"]:
            cleaned = cleaned.replace(phrase, " ")
        cleaned = " ".join(cleaned.split())
        return cleaned or None

    def has_fresh_matching_detection(self):
        if self.latest_detection is None or self.latest_detection_time is None:
            return False

        age = (self.get_clock().now() - self.latest_detection_time).nanoseconds * 1e-9
        if age > self.detection_timeout_sec:
            return False

        if not self.target_label:
            return True

        return self.latest_label is not None and self.target_label in self.latest_label

    def make_object_nav_goal(self):
        try:
            object_map = self.tf_buffer.transform(
                self.latest_detection,
                self.map_frame,
                timeout=Duration(seconds=0.5),
            )
            base_tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.5),
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().error(f"Cannot transform object for Nav2 goal: {exc}")
            return None

        base_x = base_tf.transform.translation.x
        base_y = base_tf.transform.translation.y
        obj_x = object_map.point.x
        obj_y = object_map.point.y
        dx = obj_x - base_x
        dy = obj_y - base_y
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            self.get_logger().error("Object is too close to compute approach pose.")
            return None

        goal_x = obj_x - self.object_standoff_m * dx / dist
        goal_y = obj_y - self.object_standoff_m * dy / dist
        yaw = math.atan2(obj_y - goal_y, obj_x - goal_x)
        return self.make_nav_goal(goal_x, goal_y, yaw)

    def make_home_goal(self):
        return self.make_nav_goal(self.home_x, self.home_y, self.home_yaw)

    def make_nav_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)
        return goal

    def stop_base(self):
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = FetchBTNode()
    try:
        rclpy.spin(node)
    finally:
        node.stop_base()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
