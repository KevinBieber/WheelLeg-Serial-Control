#!/usr/bin/env bash
# 便捷入口：./run.sh build|start|stop|restart|status|log
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CMD="${1:-}"

usage() {
  cat <<EOF
用法: ./run.sh <命令> [参数...]

  build     一键编译全部节点
  start     一键启动（ROS + frpc，后台）
  stop      一键关闭（ROS + frpc）
  restart   先 stop 再 start
  status    查看 ROS / frpc 是否在运行
  log       跟踪 ROS 最新日志
  frpc-log  跟踪 frpc 最新日志

环境变量（start 时可选）:
  DRY_RUN=false              实机写 H7 串口（默认 false；调试可 true）
  SERIAL_PORT=/dev/ttyACM0
  CAMERA_INDEX=4                 # D435i 彩色 /dev/video4；可覆盖
  FRPC_BIN=/home/jianghaoran/Workspace/frp/frpc   # 可执行文件或所在目录
  FRPC_CONFIG=${ROOT}/frp/frpc.toml
  SKIP_FRP=1                 只起 ROS、不起 frpc

示例:
  ./run.sh build
  DRY_RUN=false ./run.sh start
  ./run.sh log
  ./run.sh frpc-log
  ./run.sh stop
EOF
}

case "${CMD}" in
  build)
    shift || true
    exec bash "${ROOT}/scripts/build_all.sh" "$@"
    ;;
  start)
    shift || true
    exec bash "${ROOT}/scripts/start_all.sh" "$@"
    ;;
  stop)
    exec bash "${ROOT}/scripts/stop_all.sh"
    ;;
  restart)
    bash "${ROOT}/scripts/stop_all.sh" || true
    sleep 0.5
    exec bash "${ROOT}/scripts/start_all.sh"
    ;;
  status)
    ok=0
    PID_FILE="${ROOT}/.run/bringup.pid"
    FRPC_PID_FILE="${ROOT}/.run/frpc.pid"
    if [[ -f "${PID_FILE}" ]] && kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
      echo "[status] ROS  运行中 pid=$(cat "${PID_FILE}")"
    else
      echo "[status] ROS  未运行"
      ok=1
    fi
    if [[ -f "${FRPC_PID_FILE}" ]] && kill -0 "$(cat "${FRPC_PID_FILE}")" 2>/dev/null; then
      echo "[status] frpc 运行中 pid=$(cat "${FRPC_PID_FILE}")"
    else
      echo "[status] frpc 未运行"
      ok=1
    fi
    exit "${ok}"
    ;;
  log)
    LOG="${ROOT}/logs/bringup_latest.log"
    if [[ ! -e "${LOG}" ]]; then
      echo "[log] 尚无 ROS 日志，请先 ./run.sh start"
      exit 1
    fi
    exec tail -f "${LOG}"
    ;;
  frpc-log)
    LOG="${ROOT}/logs/frpc_latest.log"
    if [[ ! -e "${LOG}" ]]; then
      echo "[log] 尚无 frpc 日志，请先 ./run.sh start"
      exit 1
    fi
    exec tail -f "${LOG}"
    ;;
  *)
    usage
    exit 1
    ;;
esac
