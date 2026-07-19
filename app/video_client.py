"""图像接收：HTTP multipart MJPEG（后台线程），失败时显示占位画面。

不用 OpenCV VideoCapture 拉网络流（FFmpeg 对 multipart MJPEG 很慢且易 30s timeout）。
"""

from __future__ import annotations

import threading
import time
import urllib.error
import urllib.request
from typing import Optional, Tuple

import cv2
import numpy as np


class VideoClient:
    def __init__(self, host: str, port: int, path: str = "/stream"):
        self.url = f"http://{host}:{port}{path}"
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._last_ok = 0.0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._error = "等待视频流…"

    def open(self) -> bool:
        self.close()
        self._stop.clear()
        self._error = "等待视频流…"
        self._thread = threading.Thread(target=self._reader_loop, name="mjpeg-reader", daemon=True)
        self._thread.start()
        return True

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        with self._lock:
            if self._frame is not None:
                return True, self._frame.copy()
            text = self._error
        return False, self._placeholder(text)

    def last_ok_age(self) -> float:
        return time.time() - self._last_ok

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.5)
        self._thread = None

    def _reader_loop(self) -> None:
        boundary = b"--frame"
        while not self._stop.is_set():
            try:
                req = urllib.request.Request(
                    self.url,
                    headers={"Connection": "keep-alive", "Cache-Control": "no-cache"},
                )
                with urllib.request.urlopen(req, timeout=5.0) as resp:
                    self._error = f"已连接: {self.url}"
                    buf = b""
                    while not self._stop.is_set():
                        chunk = resp.read(4096)
                        if not chunk:
                            break
                        buf += chunk
                        while True:
                            # 找 JPEG 段：Content-Length 或 SOI/EOI
                            start = buf.find(b"\xff\xd8")
                            if start < 0:
                                if len(buf) > 1_000_000:
                                    buf = buf[-64:]
                                break
                            end = buf.find(b"\xff\xd9", start + 2)
                            if end < 0:
                                if start > 0:
                                    buf = buf[start:]
                                break
                            jpg = buf[start : end + 2]
                            buf = buf[end + 2 :]
                            # 丢掉 boundary 残留
                            bpos = buf.find(boundary)
                            if 0 <= bpos < 64:
                                buf = buf[bpos + len(boundary) :]
                            img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                            if img is None:
                                continue
                            with self._lock:
                                self._frame = img
                                self._last_ok = time.time()
                                self._error = self.url
            except Exception as exc:  # noqa: BLE001 — 拉流重连
                self._error = f"重连中: {exc}"
                with self._lock:
                    # 保留最后一帧，避免闪占位
                    pass
                self._stop.wait(0.8)

    @staticmethod
    def _placeholder(text: str) -> np.ndarray:
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[:] = (40, 40, 40)
        # 长错误信息折行
        y = 200
        for i in range(0, len(text), 40):
            cv2.putText(
                img,
                text[i : i + 40],
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (220, 220, 220),
                1,
                cv2.LINE_AA,
            )
            y += 28
        return img
