#!/usr/bin/env python3
"""NUC ↔ H7 USB 桥接节点。

订阅 /wheel_legged/cmd（JSON），按固件 usb_receive_task 组帧下发。
控制权由 H7 根据遥控器 SB 仲裁；本节点负责把 PC 指令送到 H7。
"""

from __future__ import annotations

import json
import struct
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
except ImportError:
    serial = None


def crc16_modbus(data: bytes) -> int:
    """与 H7 usb_receive_task 查表算法一致（CRC-16/MODBUS）。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def escape_payload(raw: bytes) -> bytes:
    out = bytearray()
    for b in raw:
        if b == 0xFD:
            out.extend([0xFE, 0x7D])
        elif b == 0xF8:
            out.extend([0xFE, 0x78])
        elif b == 0xFE:
            out.extend([0xFE, 0x7E])
        else:
            out.append(b)
    return bytes(out)


def build_cmd_frame(
    vel_x: float,
    vel_y: float,
    vel_w: float,
    leg_length: float,
    control_mode: int,
) -> bytes:
    """对齐 H7 解包：

    反转义后：
      [0] 占位
      [1] msg_type=0x01
      [2] app_id=0x02
      [3:5] length 大端
      [5..] payload: 4×float 大端 + control_mode(u8)
      [-3:-1] CRC 大端（对 [1 .. -3)）
      [-1] 0xF8
    """
    payload = struct.pack(
        ">ffffB",
        float(vel_x),
        float(vel_y),
        float(vel_w),
        float(leg_length),
        int(control_mode) & 0xFF,
    )
    # 从 msg_type 起参与 CRC（不含占位字节与 CRC/帧尾）
    mid = bytes([0x01, 0x02, (len(payload) >> 8) & 0xFF, len(payload) & 0xFF]) + payload
    crc = crc16_modbus(mid)
    raw = bytes([0x00]) + mid + bytes([(crc >> 8) & 0xFF, crc & 0xFF, 0xF8])
    return escape_payload(raw)


class H7BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("h7_bridge")
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("dry_run", True)

        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud = int(self.get_parameter("baudrate").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self._ser: Optional["serial.Serial"] = None
        if not self.dry_run and serial is not None:
            try:
                self._ser = serial.Serial(port, baudrate=baud, timeout=0.05)
                self.get_logger().info(f"opened serial {port} @ {baud}")
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"serial open failed: {e}; fallback dry_run")
                self.dry_run = True
        else:
            self.get_logger().warn("dry_run=true 或无 pyserial，仅打印帧不写串口")

        self.status_pub = self.create_publisher(String, "/wheel_legged/status", 10)
        self.create_subscription(String, "/wheel_legged/cmd", self._on_cmd, 10)
        self.create_timer(0.05, self._on_timer)

    def _on_cmd(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("cmd JSON 解析失败")
            return
        if data.get("estop"):
            data = {
                **data,
                "vel_x": 0.0,
                "vel_y": 0.0,
                "vel_w": 0.0,
                "control_mode": 0,
            }
        frame = build_cmd_frame(
            float(data.get("vel_x", 0.0)),
            float(data.get("vel_y", 0.0)),
            float(data.get("vel_w", 0.0)),
            float(data.get("leg_length", 0.15)),
            int(data.get("control_mode", 0)),
        )
        if self.dry_run or self._ser is None:
            self.get_logger().info(f"TX({len(frame)}B): {frame.hex()}")
        else:
            self._ser.write(frame)

    def _on_timer(self) -> None:
        payload = {
            "source": "h7_bridge",
            "dry_run": self.dry_run,
            "note": "SB on RC selects RC/Both/PC; H7 arbitrates",
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = H7BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        if node._ser is not None:
            node._ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
