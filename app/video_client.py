"""图像接收：默认尝试 MJPEG HTTP；失败时显示占位画面。"""

from __future__ import annotations

import time
from typing import Optional, Tuple

import cv2
import numpy as np


class VideoClient:
    def __init__(self, host: str, port: int, path: str = "/stream"):
        self.url = f"http://{host}:{port}{path}"
        self._cap: Optional[cv2.VideoCapture] = None
        self._last_ok = 0.0

    def open(self) -> bool:
        self.close()
        self._cap = cv2.VideoCapture(self.url)
        ok = bool(self._cap is not None and self._cap.isOpened())
        if ok:
            self._last_ok = time.time()
        return ok

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        if self._cap is None or not self._cap.isOpened():
            return False, self._placeholder("等待视频流…")
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return False, self._placeholder(f"断流: {self.url}")
        self._last_ok = time.time()
        return True, frame

    def last_ok_age(self) -> float:
        return time.time() - self._last_ok

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    @staticmethod
    def _placeholder(text: str) -> np.ndarray:
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[:] = (40, 40, 40)
        cv2.putText(img, text, (40, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 220, 220), 2)
        return img
