#!/usr/bin/env python3
import numpy as np
import rclpy
import tf2_geometry_msgs  # Registers PointStamped transforms with tf2.
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String  # Imported for Behavior Tree communication
from ultralytics import YOLO


class YOLO3DDetector(Node):
    def __init__(self):
        super().__init__("yolo_3d_detector")

        # Declare configurable parameters
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("rgb_topic", "/intel_camera/image_raw")
        self.declare_parameter("depth_topic", "/intel_camera/depth_image")
        self.declare_parameter("camera_info_topic", "/intel_camera/camera_info")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("confidence_threshold", 0.01)  # Set low for simulation object testing
        self.declare_parameter("depth_window_px", 5)

        # Load tools and libraries
        self.model = YOLO(self.get_parameter("model_path").value)
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize internal storage variables
        self.depth_image = None
        self.camera_info = None
        self.target_frame = self.get_parameter("target_frame").value
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.depth_window_px = int(self.get_parameter("depth_window_px").value)

        # Setup subscribers to listen to the camera sensors
        self.info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self.info_callback,
            10,
        )
        self.depth_sub = self.create_subscription(
            Image,
            self.get_parameter("depth_topic").value,
            self.depth_callback,
            10,
        )
        self.rgb_sub = self.create_subscription(
            Image,
            self.get_parameter("rgb_topic").value,
            self.rgb_callback,
            10,
        )

        # Setup publishers to talk to Rviz and the Behavior Tree
        self.camera_point_pub = self.create_publisher(PointStamped, "/detected_object_camera", 10)
        self.goal_pub = self.create_publisher(PointStamped, "/detected_object_3d", 10)
        
        # New publisher so your Behavior Tree stops spinning and handles navigation goals
        self.label_pub = self.create_publisher(String, "/detected_object_label", 10)

        self.get_logger().info(
            f"✅ 3D detector initialized. RGB={self.get_parameter('rgb_topic').value}, "
            f"depth={self.get_parameter('depth_topic').value}, "
            f"target_frame={self.target_frame}"
        )

    def info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            # Handle both 16UC1 (mm) and 32FC1 (meters)
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) * 0.001
            self.depth_image = np.asanyarray(depth)
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def rgb_callback(self, rgb_msg):
        if self.depth_image is None or self.camera_info is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        results = self.model(cv_image, verbose=True)  # Keeps verbose visible for terminal analysis

        best = self._best_detection(results)
        if best is None:
            return

        label, confidence, u, v = best
        z_depth = self._median_depth(u, v)
        if z_depth is None:
            return

        # Calculate coordinates using your unrotated rpy="0 0 0" camera alignment matrix
        point_camera = self._pixel_to_point(u, v, z_depth, rgb_msg.header)
        self.camera_point_pub.publish(point_camera)

        # Transform coordinates relative to the robot's base footprint map reference link
        try:
            point_target = self.tf_buffer.transform(
                point_camera,
                self.target_frame,
                timeout=Duration(seconds=0.2),
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f"Detected {label}, but TF transformation matrix calculation to {self.target_frame} failed: {exc}"
            )
            return

        # 1. Publish 3D navigation coordinates
        self.goal_pub.publish(point_target)
        
        # 2. Publish matching text string label to trigger Behavior Tree state changes
        label_msg = String()
        label_msg.data = label
        self.label_pub.publish(label_msg)

        self.get_logger().info(
            f"🎯 Tracking Match: [{label}] (Conf: {confidence:.2f}) -> "
            f"camera=[{point_camera.point.x:.2f}, {point_camera.point.y:.2f}, {point_camera.point.z:.2f}] | "
            f"{self.target_frame}=[{point_target.point.x:.2f}, {point_target.point.y:.2f}, {point_target.point.z:.2f}]"
        )

    def _best_detection(self, results):
        best = None
        best_conf = self.confidence_threshold

        for result in results:
            for box in result.boxes:
                confidence = float(box.conf[0])
                if confidence < best_conf:
                    continue

                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]
                xyxy = box.xyxy[0].to("cpu").numpy()
                u = int((xyxy[0] + xyxy[2]) / 2)
                v = int((xyxy[1] + xyxy[3]) / 2)
                best = (label, confidence, u, v)
                best_conf = confidence

        return best

    def _median_depth(self, u, v):
        height, width = self.depth_image.shape[:2]
        if not (0 <= u < width and 0 <= v < height):
            return None

        half = max(1, self.depth_window_px // 2)
        u_min = max(0, u - half)
        u_max = min(width, u + half + 1)
        v_min = max(0, v - half)
        v_max = min(height, v + half + 1)

        roi = self.depth_image[v_min:v_max, u_min:u_max]
        # Filter out NaN, Inf, and noise (static)
        valid = roi[np.isfinite(roi) & (roi > 0.1) & (roi < 10.0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _pixel_to_point(self, u, v, depth_m, header):
        """
        Calculates 3D coordinates based on an unrotated rpy="0 0 0" camera link frame.
        Maps the camera viewing pipeline parameters safely to standard robot coordinate links.
        """
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        # OpenCV pinhole camera matrix variables calculation
        x_optical = float((u - cx) * depth_m / fx)
        y_optical = float((v - cy) * depth_m / fy)
        z_optical = float(depth_m)

        point_msg = PointStamped()
        point_msg.header = header
        
        # Standard ROS Optical Frame coordinates (REP-103)
        # X: Right, Y: Down, Z: Forward
        point_msg.point.x = x_optical
        point_msg.point.y = y_optical
        point_msg.point.z = z_optical
        
        return point_msg


def main(args=None):
    rclpy.init(args=args)
    node = YOLO3DDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()