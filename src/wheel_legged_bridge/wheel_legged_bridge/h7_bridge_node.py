#!/usr/bin/env python3
"""NUC ↔ H7 USB 桥接节点。

订阅 /wheel_legged/cmd（JSON），按固件 usb_receive_task 组帧下发。
协议说明见仓库 doc/NUC_H7_USB通信协议.md。
"""

from __future__ import annotations

import json
import struct
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
except ImportError:
    serial = None

# 与 H7 usb_receive_task.c 中 crc16_table 逐字一致（含固件原表）
_CRC16_TABLE = (
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1600, 0xD6C1, 0xD781, 0x1740, 0xD501, 0x15C0, 0x1480, 0xD441,
    0xD201, 0x22C0, 0x2380, 0xD341, 0x2100, 0xD1C1, 0xD081, 0x2040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2E00, 0xEEC1, 0xEF81, 0x2F40, 0xED01, 0x2DC0, 0x2C80, 0xEC41,
    0xE601, 0x26C0, 0x2780, 0xE741, 0x2500, 0xE5C1, 0xE481, 0x2440,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x3800, 0xF8C1, 0xF981, 0x3940, 0xFB01, 0x3BC0, 0x3A80, 0xFA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0x7600, 0xB6C1, 0xB781, 0x7740, 0xB501, 0x75C0, 0x7480, 0xB441,
    0xB201, 0x72C0, 0x7380, 0xB341, 0x7100, 0xB1C1, 0xB081, 0x7040,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4600, 0x86C1, 0x8781, 0x4740, 0x8501, 0x45C0, 0x4480, 0x8441,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
    0xC001, 0x00C0, 0x0180, 0xC141, 0x0300, 0xC3C1, 0xC281, 0x0240,
    0x0600, 0xC6C1, 0xC781, 0x0740, 0xC501, 0x05C0, 0x0480, 0xC441,
    0x0C00, 0xCCC1, 0xCD81, 0x0D40, 0xCF01, 0x0FC0, 0x0E80, 0xCE41,
    0xCA01, 0x0AC0, 0x0B80, 0xCB41, 0x0900, 0xC9C1, 0xC881, 0x0840,
)

SOF = 0xFD
EOF = 0xF8
MSG_TYPE_CMD = 0x01
APP_ID = 0x02


def crc16_h7(data: bytes) -> int:
    """与 H7 checkCrc16 / crc16_table 一致。"""
    crc = 0xFFFF
    for b in data:
        index = (crc ^ b) & 0xFF
        crc = (crc >> 8) ^ _CRC16_TABLE[index]
    return crc & 0xFFFF


def escape_body(raw: bytes) -> bytes:
    """仅转义帧头与帧尾之间的字节。SOF/EOF 本身不得转义。"""
    out = bytearray()
    for b in raw:
        if b == SOF:
            out.extend([0xFE, 0x7D])
        elif b == EOF:
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
    payload = struct.pack(
        ">ffffB",
        float(vel_x),
        float(vel_y),
        float(vel_w),
        float(leg_length),
        int(control_mode) & 0xFF,
    )
    length = len(payload)
    header_and_payload = bytes(
        [
            MSG_TYPE_CMD,
            APP_ID,
            (length >> 8) & 0xFF,
            length & 0xFF,
        ]
    ) + payload
    crc = crc16_h7(header_and_payload)
    body = header_and_payload + bytes([(crc >> 8) & 0xFF, crc & 0xFF])
    return bytes([SOF]) + escape_body(body) + bytes([EOF])


def _list_acm_ports() -> list[str]:
    return sorted(str(p) for p in Path("/dev").glob("ttyACM*") if p.exists())


def _parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    s = str(value).strip().lower()
    return s in ("1", "true", "yes", "on")


class H7BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("h7_bridge")
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        # 用字符串，避免 launch 传入 "false" 与 bool 声明冲突
        self.declare_parameter("dry_run", "false")

        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud = int(self.get_parameter("baudrate").value)
        self.dry_run = _parse_bool(self.get_parameter("dry_run").value)
        self._port_name = port
        self._baud = baud
        self._ser: Optional["serial.Serial"] = None
        self._tx_ok = 0
        self._tx_fail = 0
        self._rx_cmd = 0
        self._last_hex = ""

        if serial is None:
            self.get_logger().error("未安装 pyserial（python3-serial），无法写 USB")
            self.dry_run = True

        if self.dry_run:
            self.get_logger().warn(
                "dry_run=true → 只打印帧，不写串口。"
                "实机请: DRY_RUN=false ./run.sh start"
            )
        else:
            self._open_serial()

        self.status_pub = self.create_publisher(String, "/wheel_legged/status", 10)
        self.create_subscription(String, "/wheel_legged/cmd", self._on_cmd, 50)
        self.create_timer(0.5, self._on_timer)
        self.create_timer(2.0, self._log_tx_stats)

    def _open_serial(self) -> bool:
        if serial is None:
            return False
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
            self._ser = None

        candidates = []
        if self._port_name:
            candidates.append(self._port_name)
        for p in _list_acm_ports():
            if p not in candidates:
                candidates.append(p)

        last_err = None
        for port in candidates:
            try:
                ser = serial.Serial(
                    port=port,
                    baudrate=self._baud,
                    timeout=0.05,
                    write_timeout=0.2,
                )
                # 部分 CDC 设备需要拉高 DTR 才真正收数
                try:
                    ser.dtr = True
                    ser.rts = True
                except Exception:  # noqa: BLE001
                    pass
                time.sleep(0.05)
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                self._ser = ser
                self._port_name = port
                self.get_logger().info(
                    f"USB LIVE: opened {port} @ {self._baud}  "
                    f"acm_list={_list_acm_ports()}"
                )
                return True
            except Exception as e:  # noqa: BLE001
                last_err = e
                self.get_logger().warn(f"open {port} failed: {e}")

        self.get_logger().error(
            f"无法打开任何串口（尝试 {candidates}），last={last_err}；"
            "请检查 H7 USB、权限: ls -l /dev/ttyACM* ; groups"
        )
        return False

    def _write_frame(self, frame: bytes) -> bool:
        if self.dry_run:
            self._last_hex = frame.hex()
            return False
        if self._ser is None or not self._ser.is_open:
            if not self._open_serial():
                self._tx_fail += 1
                return False
        try:
            n = self._ser.write(frame)
            self._ser.flush()
            self._last_hex = frame.hex()
            if n != len(frame):
                self.get_logger().warn(f"short write {n}/{len(frame)}")
                self._tx_fail += 1
                return False
            self._tx_ok += 1
            return True
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"serial write failed: {e}")
            self._tx_fail += 1
            try:
                if self._ser is not None:
                    self._ser.close()
            except Exception:  # noqa: BLE001
                pass
            self._ser = None
            return False

    def _on_cmd(self, msg: String) -> None:
        self._rx_cmd += 1
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
            }
        frame = build_cmd_frame(
            float(data.get("vel_x", 0.0)),
            float(data.get("vel_y", 0.0)),
            float(data.get("vel_w", 0.0)),
            float(data.get("leg_length", 0.15)),
            int(data.get("control_mode", 0)),
        )
        if self.dry_run:
            # 限频：大约每秒一条，避免刷屏
            if self._rx_cmd % 20 == 1:
                self.get_logger().info(
                    f"[dry_run] TX({len(frame)}B) mode={data.get('control_mode')} "
                    f"hex={frame.hex()}"
                )
            return
        self._write_frame(frame)

    def _log_tx_stats(self) -> None:
        port = self._port_name if self._ser else "CLOSED"
        self.get_logger().info(
            f"h7_bridge: dry_run={self.dry_run} port={port} "
            f"cmd_rx={self._rx_cmd} tx_ok={self._tx_ok} tx_fail={self._tx_fail} "
            f"last={self._last_hex[:40]}{'...' if len(self._last_hex) > 40 else ''}"
        )

    def _on_timer(self) -> None:
        payload = {
            "source": "h7_bridge",
            "dry_run": self.dry_run,
            "serial_port": self._port_name,
            "serial_open": bool(self._ser is not None and self._ser.is_open),
            "cmd_rx": self._rx_cmd,
            "tx_ok": self._tx_ok,
            "tx_fail": self._tx_fail,
            "acm": _list_acm_ports(),
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
