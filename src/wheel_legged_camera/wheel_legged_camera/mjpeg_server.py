#!/usr/bin/env python3
"""简易 MJPEG HTTP 服务，供 pc_host 直播拉流。

默认: http://0.0.0.0:8081/stream
D435i 彩色一般为 /dev/video4（YUYV），默认 640x480@30 + JPEG 质量可调。
"""

from __future__ import annotations

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node

try:
    import cv2
except ImportError:
    cv2 = None


class _FrameBuffer:
    """最新一帧 JPEG；用 version 通知发送端只推新帧。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._jpeg = b""
        self._version = 0

    def set_bgr(self, frame, jpeg_quality: int) -> None:
        ok, enc = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)],
        )
        if not ok:
            return
        with self._cond:
            self._jpeg = enc.tobytes()
            self._version += 1
            self._cond.notify_all()

    def wait_jpeg(self, last_version: int, timeout_s: float = 1.0) -> tuple[bytes, int]:
        with self._cond:
            if self._version == last_version or not self._jpeg:
                self._cond.wait(timeout=timeout_s)
            return self._jpeg, self._version


FRAME_BUF = _FrameBuffer()


def make_handler(path: str):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def do_GET(self):  # noqa: N802
            if self.path.split("?", 1)[0] != path:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last_ver = -1
            try:
                while True:
                    jpg, ver = FRAME_BUF.wait_jpeg(last_ver, timeout_s=1.0)
                    if not jpg or ver == last_ver:
                        continue
                    last_ver = ver
                    self.wfile.write(b"--frame\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpg)))
                    self.end_headers()
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
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
        self.declare_parameter("fps", 30)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("jpeg_quality", 60)

        idx = int(self.get_parameter("device_index").value)
        host = self.get_parameter("host").get_parameter_value().string_value
        port = int(self.get_parameter("port").value)
        path = self.get_parameter("path").get_parameter_value().string_value
        self._fps = max(float(self.get_parameter("fps").value), 1.0)
        self._width = int(self.get_parameter("width").value)
        self._height = int(self.get_parameter("height").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self.cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(idx)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self.cap.set(cv2.CAP_PROP_FPS, self._fps)
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(self.cap.get(cv2.CAP_PROP_FPS))

        self._stop = threading.Event()
        self._grab_thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._grab_thread.start()

        handler = make_handler(path)
        self.httpd = ThreadingHTTPServer((host, port), handler)
        self.httpd.daemon_threads = True
        self._http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self._http_thread.start()

        self.get_logger().info(
            f"MJPEG http://{host}:{port}{path} "
            f"device={idx} req={self._width}x{self._height}@{self._fps} "
            f"actual={actual_w}x{actual_h}@{actual_fps:.1f} jpeg_q={self._jpeg_quality}"
        )

    def _grab_loop(self) -> None:
        period = 1.0 / self._fps
        while not self._stop.is_set():
            t0 = time.monotonic()
            ok, frame = self.cap.read()
            if ok and frame is not None:
                # 若驱动仍给更高分辨率，缩小以降低编码与带宽
                h, w = frame.shape[:2]
                if w != self._width or h != self._height:
                    frame = cv2.resize(frame, (self._width, self._height), interpolation=cv2.INTER_AREA)
                FRAME_BUF.set_bgr(frame, self._jpeg_quality)
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, period - elapsed))

    def destroy_node(self) -> bool:
        self._stop.set()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MjpegServerNode()
    try:
        rclpy.spin(node)
    finally:
        node._stop.set()
        node.httpd.shutdown()
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
