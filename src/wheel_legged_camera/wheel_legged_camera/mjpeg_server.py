#!/usr/bin/env python3
"""简易 MJPEG HTTP 服务，供 pc_host 直播拉流。

默认: http://0.0.0.0:8081/stream
可在无完整 ROS 图像链路时单独跑：
  ros2 run wheel_legged_camera mjpeg_server
"""

from __future__ import annotations

import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node

try:
    import cv2
except ImportError:
    cv2 = None


class _FrameBuffer:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._jpeg = b""

    def set_bgr(self, frame) -> None:
        ok, enc = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ok:
            return
        with self._lock:
            self._jpeg = enc.tobytes()

    def get(self) -> bytes:
        with self._lock:
            return self._jpeg


FRAME_BUF = _FrameBuffer()


def make_handler(path: str):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):  # noqa: N802
            if self.path != path:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    jpg = FRAME_BUF.get()
                    if not jpg:
                        continue
                    self.wfile.write(b"--frame\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpg)))
                    self.end_headers()
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
            except BrokenPipeError:
                return

        def log_message(self, fmt, *args):  # noqa: A003
            return

    return Handler


class MjpegServerNode(Node):
    def __init__(self) -> None:
        super().__init__("mjpeg_server")
        if cv2 is None:
            raise RuntimeError("opencv-python required")

        self.declare_parameter("device_index", 4)  # D435i RGB = /dev/video4
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8081)
        self.declare_parameter("path", "/stream")
        self.declare_parameter("fps", 20)

        idx = int(self.get_parameter("device_index").value)
        host = self.get_parameter("host").get_parameter_value().string_value
        port = int(self.get_parameter("port").value)
        path = self.get_parameter("path").get_parameter_value().string_value
        fps = float(self.get_parameter("fps").value)

        self.cap = cv2.VideoCapture(idx)
        self.create_timer(1.0 / max(fps, 1.0), self._grab)

        handler = make_handler(path)
        self.httpd = ThreadingHTTPServer((host, port), handler)
        self._thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self._thread.start()
        self.get_logger().info(f"MJPEG http://{host}:{port}{path}")

    def _grab(self) -> None:
        ok, frame = self.cap.read()
        if ok:
            FRAME_BUF.set_bgr(frame)


def main() -> None:
    rclpy.init()
    node = MjpegServerNode()
    try:
        rclpy.spin(node)
    finally:
        node.httpd.shutdown()
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
