#!/usr/bin/env bash
# 一键关闭：ROS 节点 + frpc
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PID_FILE="${ROOT}/.run/bringup.pid"
FRPC_PID_FILE="${ROOT}/.run/frpc.pid"
FRPC_CONFIG="${FRPC_CONFIG:-${ROOT}/frp/frpc.toml}"

stop_pid() {
  local name="$1"
  local pid="$2"
  if [[ -z "${pid}" ]]; then
    return 0
  fi
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "[stop] ${name} 进程 ${pid} 已不存在"
    return 0
  fi
  echo "[stop] 停止 ${name} (pid=${pid}) ..."
  kill -INT "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
  for _ in $(seq 1 20); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      echo "[stop] ${name} 已退出"
      return 0
    fi
    sleep 0.25
  done
  echo "[stop] ${name} 强制 SIGKILL"
  kill -KILL "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
}

# 先停 ROS
if [[ -f "${PID_FILE}" ]]; then
  stop_pid "ROS" "$(cat "${PID_FILE}" 2>/dev/null || true)"
  rm -f "${PID_FILE}"
else
  echo "[stop] 未找到 ROS pid 文件，按节点名清理"
fi

pkill -f "wheel_legged_bringup/bringup.launch.py" 2>/dev/null || true
pkill -f "h7_bridge" 2>/dev/null || true
pkill -f "mjpeg_server" 2>/dev/null || true
pkill -f "net_gateway" 2>/dev/null || true

# 再停 frpc
if [[ -f "${FRPC_PID_FILE}" ]]; then
  stop_pid "frpc" "$(cat "${FRPC_PID_FILE}" 2>/dev/null || true)"
  rm -f "${FRPC_PID_FILE}"
fi
pkill -f "frpc -c ${FRPC_CONFIG}" 2>/dev/null || true
# 兜底：默认 frpc 路径
pkill -f "/home/jianghaoran/Workspace/frp/frpc" 2>/dev/null || true

echo "[stop] 完成"
