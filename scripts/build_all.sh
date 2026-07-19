#!/usr/bin/env bash
# 一键编译 nuc_ros 全部包
set -euo pipefail

# 本脚本在 scripts/ 下，工作空间根目录是上一级
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# 尝试加载常见 ROS 2 发行版
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

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "[build] 未检测到 ROS 2，请先: source /opt/ros/<distro>/setup.bash"
  exit 1
fi

if [[ ! -d "${ROOT}/src" ]]; then
  echo "[build] 未找到 ${ROOT}/src，确认在正确的 nuc_ros 工作空间"
  exit 1
fi

echo "[build] ROS_DISTRO=${ROS_DISTRO}"
echo "[build] workspace=${ROOT}"

# 可选依赖提示（不强制失败）
python3 -c "import serial" 2>/dev/null || echo "[build] 提示: 未安装 pyserial，可 apt install python3-serial"
python3 -c "import cv2" 2>/dev/null || echo "[build] 提示: 未安装 opencv，可 apt install python3-opencv"

colcon build --symlink-install "$@"

# setup.bash 里有未绑定变量，临时关闭 nounset
# shellcheck disable=SC1091
set +u
source "${ROOT}/install/setup.bash"
set -u

echo "[build] 完成。下次新开终端请执行:"
echo "  source ${ROOT}/install/setup.bash"
