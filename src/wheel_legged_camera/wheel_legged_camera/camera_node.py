#!/usr/bin/env python3
"""OpenCV 相机节点：发布 sensor_msgs/Image（需 cv_bridge）。

无 cv_bridge 时仍可用 mjpeg_server 给 PC 直播。
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

try:
    import cv2
except ImportError:
    cv2 = None

try:
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
except ImportError:
    CvBridge = None
    Image = None


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("wheel_legged_camera")
        self.declare_parameter("device_index", 4)  # D435i RGB = /dev/video4
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)

        if cv2 is None or Image is None or CvBridge is None:
            self.get_logger().error("需要 opencv-python 与 cv_bridge；或改用 mjpeg_server")
            raise RuntimeError("camera deps missing")

        idx = int(self.get_parameter("device_index").value)
        w = int(self.get_parameter("width").value)
        h = int(self.get_parameter("height").value)
        fps = float(self.get_parameter("fps").value)

        self.cap = cv2.VideoCapture(idx)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        period = 1.0 / max(fps, 1.0)
        self.create_timer(period, self._tick)
        self.get_logger().info(f"camera index={idx} {w}x{h}@{fps}")

    def _tick(self) -> None:
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("camera frame grab failed", throttle_duration_sec=2.0)
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color_optical_frame"
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        if hasattr(node, "cap") and node.cap is not None:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
