"""指令发送：TCP 每行一条 JSON，由 NUC teleop 转发到 H7。"""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any, Dict, Optional


class CommandClient:
    def __init__(self, host: str, port: int, rate_hz: float = 20.0):
        self.host = host
        self.port = port
        self.period = 1.0 / max(rate_hz, 1.0)
        self._cmd: Dict[str, Any] = {
            "vel_x": 0.0,
            "vel_y": 0.0,
            "vel_w": 0.0,
            "leg_length": 0.15,
            "control_mode": 0,
            "estop": False,
        }
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._connected = False
        self.enabled = True

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def set_cmd(self, **kwargs: Any) -> None:
        with self._lock:
            self._cmd.update(kwargs)

    def zero(self) -> None:
        """速度清零；不改 control_mode（失能请显式设 control_mode=1）。"""
        self.set_cmd(vel_x=0.0, vel_y=0.0, vel_w=0.0, estop=False)

    @property
    def connected(self) -> bool:
        return self._connected

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                with socket.create_connection((self.host, self.port), timeout=2.0) as sock:
                    self._connected = True
                    sock_file = sock.makefile("w", encoding="utf-8", newline="\n")
                    while not self._stop.is_set():
                        if self.enabled:
                            with self._lock:
                                payload = dict(self._cmd)
                            sock_file.write(json.dumps(payload, ensure_ascii=False) + "\n")
                            sock_file.flush()
                        time.sleep(self.period)
            except OSError:
                self._connected = False
                time.sleep(1.0)
            finally:
                self._connected = False
