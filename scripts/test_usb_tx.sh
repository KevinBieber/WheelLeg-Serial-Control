#!/usr/bin/env bash
# 不依赖 ROS：直接往 H7 USB 发一帧，用于确认串口通路
# 用法:
#   ./scripts/test_usb_tx.sh
#   SERIAL_PORT=/dev/ttyACM1 ./scripts/test_usb_tx.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${SERIAL_PORT:-/dev/ttyACM0}"

echo "[test] acm devices:"
ls -l /dev/ttyACM* 2>/dev/null || echo "  (none)"

python3 - <<PY
import sys
sys.path.insert(0, "${ROOT}/src/wheel_legged_bridge")
from wheel_legged_bridge.h7_bridge_node import build_cmd_frame
import serial, time

port = "${PORT}"
frame = build_cmd_frame(0.0, 0.0, 0.0, 0.15, 3)  # mode=3 起身
print(f"[test] port={port}")
print(f"[test] frame({len(frame)}B)={frame.hex()}")
ser = serial.Serial(port, 115200, timeout=0.05, write_timeout=0.5)
ser.dtr = True
ser.rts = True
time.sleep(0.05)
n = ser.write(frame)
ser.flush()
print(f"[test] wrote {n} bytes OK")
ser.close()
PY
