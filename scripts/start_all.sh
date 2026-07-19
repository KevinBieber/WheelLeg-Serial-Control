#!/usr/bin/env bash
# 一键启动：ROS 全部节点 + frpc（后台，日志写入 logs/）
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PID_FILE="${ROOT}/.run/bringup.pid"
FRPC_PID_FILE="${ROOT}/.run/frpc.pid"
LOG_DIR="${ROOT}/logs"
mkdir -p "${ROOT}/.run" "${LOG_DIR}"

# frpc 可执行文件：默认你的路径；可用 FRPC_BIN 覆盖
# 支持「文件」或「目录」（目录下找名为 frpc 的二进制）
FRPC_BIN="${FRPC_BIN:-/home/jianghaoran/Workspace/frp/frpc}"
if [[ -d "${FRPC_BIN}" ]]; then
  FRPC_BIN="${FRPC_BIN}/frpc"
fi
# 配置：默认用本仓库 frp/frpc.toml；可用 FRPC_CONFIG 覆盖
FRPC_CONFIG="${FRPC_CONFIG:-${ROOT}/frp/frpc.toml}"
# SKIP_FRP=1 时只起 ROS、不起 frpc
SKIP_FRP="${SKIP_FRP:-0}"

stop_pid() {
  local pid="$1"
  [[ -z "${pid}" ]] && return 0
  kill -0 "${pid}" 2>/dev/null || return 0
  kill -INT "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 "${pid}" 2>/dev/null || return 0
    sleep 0.25
  done
  kill -KILL "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
}

# 已在跑则拒绝重复启动
if [[ -f "${PID_FILE}" ]]; then
  old_pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
  if [[ -n "${old_pid}" ]] && kill -0 "${old_pid}" 2>/dev/null; then
    echo "[start] ROS 已在运行 (pid=${old_pid})，请先 ./scripts/stop_all.sh"
    exit 1
  fi
  rm -f "${PID_FILE}"
fi

# ROS 环境
if [[ -z "${ROS_DISTRO:-}" ]]; then
  for d in humble iron jazzy foxy; do
    if [[ -f "/opt/ros/${d}/setup.bash" ]]; then
      # shellcheck disable=SC1090
      set +u
      source "/opt/ros/${d}/setup.bash"
      set -u
      break
    fi
  done
fi
if [[ ! -f "${ROOT}/install/setup.bash" ]]; then
  echo "[start] 未编译，请先: ./run.sh build"
  exit 1
fi
# setup.bash 含未绑定变量，临时关闭 nounset
# shellcheck disable=SC1091
set +u
source "${ROOT}/install/setup.bash"
set -u

SERIAL_PORT="${SERIAL_PORT:-/dev/ttyACM0}"
CAMERA_INDEX="${CAMERA_INDEX:-4}"
DRY_RUN="${DRY_RUN:-true}"

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/bringup_${STAMP}.log"
FRPC_LOG="${LOG_DIR}/frpc_${STAMP}.log"
ln -sfn "${LOG_FILE}" "${LOG_DIR}/bringup_latest.log"
ln -sfn "${FRPC_LOG}" "${LOG_DIR}/frpc_latest.log"

# ---------- 先起 frpc ----------
if [[ "${SKIP_FRP}" != "1" ]]; then
  if [[ ! -x "${FRPC_BIN}" ]]; then
    echo "[start] 找不到可执行 frpc: ${FRPC_BIN}"
    echo "        请确认路径，或设置 FRPC_BIN=/path/to/frpc"
    exit 1
  fi
  if [[ ! -f "${FRPC_CONFIG}" ]]; then
    echo "[start] 找不到 frpc 配置: ${FRPC_CONFIG}"
    exit 1
  fi
  # 若旧 frpc 还在，先停
  if [[ -f "${FRPC_PID_FILE}" ]]; then
    stop_pid "$(cat "${FRPC_PID_FILE}" 2>/dev/null || true)"
    rm -f "${FRPC_PID_FILE}"
  fi
  # 兜底清同配置残留
  pkill -f "frpc -c ${FRPC_CONFIG}" 2>/dev/null || true

  echo "[start] frpc=${FRPC_BIN}"
  echo "[start] frpc config=${FRPC_CONFIG}"
  echo "[start] frpc log -> ${FRPC_LOG}"
  nohup "${FRPC_BIN}" -c "${FRPC_CONFIG}" >"${FRPC_LOG}" 2>&1 &
  echo $! >"${FRPC_PID_FILE}"
  sleep 0.5
  if ! kill -0 "$(cat "${FRPC_PID_FILE}")" 2>/dev/null; then
    echo "[start] frpc 启动失败，见日志: ${FRPC_LOG}"
    exit 1
  fi
  echo "[start] frpc 已启动 pid=$(cat "${FRPC_PID_FILE}")"
else
  echo "[start] SKIP_FRP=1，跳过 frpc"
fi

# ---------- 再起 ROS ----------
echo "[start] serial=${SERIAL_PORT} camera=${CAMERA_INDEX} dry_run=${DRY_RUN}"
echo "[start] ros log -> ${LOG_FILE}"

nohup ros2 launch wheel_legged_bringup bringup.launch.py \
  serial_port:="${SERIAL_PORT}" \
  camera_index:="${CAMERA_INDEX}" \
  dry_run:="${DRY_RUN}" \
  >"${LOG_FILE}" 2>&1 &

echo $! >"${PID_FILE}"
echo "[start] ROS 已启动 pid=$(cat "${PID_FILE}")"
echo "[start] 查看 ROS 日志:  tail -f ${LOG_DIR}/bringup_latest.log"
echo "[start] 查看 frpc 日志: tail -f ${LOG_DIR}/frpc_latest.log"
echo "[start] 停止: ./scripts/stop_all.sh"
