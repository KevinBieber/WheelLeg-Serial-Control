"""状态接收：TCP 按行 JSON（NUC 侧可改成同协议）。"""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any, Dict, Optional


class StatusClient:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest: Dict[str, Any] = {"online": False, "msg": "未连接"}
        self._last_rx = 0.0

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def get(self) -> Dict[str, Any]:
        with self._lock:
            data = dict(self._latest)
        data["age_s"] = time.time() - self._last_rx if self._last_rx else None
        return data

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                sock = socket.create_connection((self.host, self.port), timeout=2.0)
                self._sock = sock
                f = sock.makefile("r", encoding="utf-8", newline="\n")
                with self._lock:
                    self._latest = {"online": True, "msg": "已连接"}
                for line in f:
                    if self._stop.is_set():
                        break
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        payload = json.loads(line)
                    except json.JSONDecodeError:
                        payload = {"raw": line}
                    payload["online"] = True
                    self._last_rx = time.time()
                    with self._lock:
                        self._latest = payload
            except OSError as e:
                with self._lock:
                    self._latest = {"online": False, "msg": f"状态链路异常: {e}"}
                time.sleep(1.0)
            finally:
                if self._sock is not None:
                    try:
                        self._sock.close()
                    except OSError:
                        pass
                    self._sock = None
