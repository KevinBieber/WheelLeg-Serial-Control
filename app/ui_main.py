"""主窗口：视频 + 状态 + WASD 遥控。"""

from __future__ import annotations

import time
from typing import Any, Dict

import cv2
import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap, QKeySequence
from PyQt5.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QShortcut,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from .command_client import CommandClient
from .status_client import StatusClient
from .video_client import VideoClient


class MainWindow(QMainWindow):
    def __init__(self, cfg: Dict[str, Any]):
        super().__init__()
        conn = cfg["connection"]
        ui = cfg.get("ui", {})
        ctrl = cfg.get("control", {})
        video_cfg = cfg.get("video", {})

        self.setWindowTitle(ui.get("window_title", "Wheel-Legged Host"))
        self.resize(1100, 700)

        self._vx_max = float(ctrl.get("v_x_max", 1.8))
        self._w_max = float(ctrl.get("w_max", 2.5))
        self._leg = float(ctrl.get("leg_length_default", 0.15))
        self._keys = set()
        self._control_mode = 0

        self.video = VideoClient(conn["host"], int(conn["video_port"]), video_cfg.get("path", "/stream"))
        self.status = StatusClient(conn["host"], int(conn["status_port"]))
        self.command = CommandClient(
            conn["host"],
            int(conn["command_port"]),
            rate_hz=float(ctrl.get("send_rate_hz", 20)),
        )
        self.command.set_cmd(leg_length=self._leg, control_mode=0)

        self.video_label = QLabel("视频")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setStyleSheet("background:#222;color:#ddd;")

        self.status_view = QTextEdit()
        self.status_view.setReadOnly(True)
        self.status_view.setMinimumWidth(320)

        root = QWidget()
        layout = QHBoxLayout(root)
        layout.addWidget(self.video_label, stretch=3)
        right = QVBoxLayout()
        hint = QLabel(
            "WASD 移动转向 | Q/E 腿高 | J 跳跃 K 回跑 | 空格急停 | R 重连视频\n"
            "车上遥控器 SB：上=仅遥控 / 中=双控 / 下=仅PC"
        )
        right.addWidget(hint)
        right.addWidget(self.status_view, stretch=1)
        layout.addLayout(right, stretch=1)
        self.setCentralWidget(root)

        QShortcut(QKeySequence("Space"), self, activated=self._estop)
        QShortcut(QKeySequence("R"), self, activated=self._reconnect_video)
        QShortcut(QKeySequence("J"), self, activated=self._jump)
        QShortcut(QKeySequence("K"), self, activated=self._run_mode)

        self.video.open()
        self.status.start()
        self.command.start()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_tick)
        self.timer.start(int(1000 / max(int(ui.get("status_refresh_hz", 20)), 1)))

        self._hb_timeout = float(conn.get("heartbeat_timeout_s", 1.5))

    def _reconnect_video(self) -> None:
        self.video.open()

    def _estop(self) -> None:
        self._keys.clear()
        self.command.zero()
        self._control_mode = 0

    def _jump(self) -> None:
        self._control_mode = 2
        self.command.set_cmd(control_mode=2, estop=False)

    def _run_mode(self) -> None:
        self._control_mode = 0
        self.command.set_cmd(control_mode=0, estop=False)

    def keyPressEvent(self, event):  # noqa: N802
        if event.isAutoRepeat():
            return
        key = event.key()
        self._keys.add(key)
        if key == Qt.Key_Space:
            self._estop()
        elif key == Qt.Key_Q:
            self._leg = max(0.10, self._leg - 0.01)
        elif key == Qt.Key_E:
            self._leg = min(0.20, self._leg + 0.01)
        self._update_cmd_from_keys()

    def keyReleaseEvent(self, event):  # noqa: N802
        if event.isAutoRepeat():
            return
        self._keys.discard(event.key())
        self._update_cmd_from_keys()

    def _update_cmd_from_keys(self) -> None:
        vx = 0.0
        w = 0.0
        if Qt.Key_W in self._keys:
            vx += self._vx_max
        if Qt.Key_S in self._keys:
            vx -= self._vx_max
        if Qt.Key_A in self._keys:
            w += self._w_max
        if Qt.Key_D in self._keys:
            w -= self._w_max
        self.command.set_cmd(
            vel_x=vx,
            vel_w=w,
            vel_y=0.0,
            leg_length=self._leg,
            control_mode=self._control_mode,
            estop=False,
        )

    def _on_tick(self) -> None:
        ok, frame = self.video.read()
        if frame is not None:
            self._show_frame(frame)

        st = self.status.get()
        age = st.get("age_s")
        link_ok = bool(st.get("online")) and age is not None and age < self._hb_timeout
        self.command.enabled = link_ok and not self.command._cmd.get("estop", False)

        lines = [
            f"时间: {time.strftime('%H:%M:%S')}",
            f"视频: {'OK' if ok else '等待/断流'}  age={self.video.last_ok_age():.1f}s",
            f"状态链路: {'OK' if link_ok else '异常'}  cmd_tx={'ON' if self.command.connected else 'OFF'}",
            f"指令使能: {self.command.enabled}",
            "",
            "状态数据:",
            str(st),
            "",
            "当前指令:",
            str(self.command._cmd),
        ]
        self.status_view.setPlainText("\n".join(lines))

    def _show_frame(self, frame: np.ndarray) -> None:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        self.video_label.setPixmap(
            QPixmap.fromImage(qimg).scaled(
                self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
        )

    def closeEvent(self, event):  # noqa: N802
        self.timer.stop()
        self.command.zero()
        self.command.stop()
        self.status.stop()
        self.video.close()
        super().closeEvent(event)
