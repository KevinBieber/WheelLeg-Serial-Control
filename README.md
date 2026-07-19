# nuc_ros

在 **Intel NUC（Ubuntu）** 上运行的 ROS 2 工作空间。  
职责：采集相机、与 H7 USB 通信、发布整车状态、接收远程指令，并配合 FRP 给 Windows PC（`pc_host`）访问。

> 在 Windows 开发机上只编辑本目录；整包拷到 NUC 的 Ubuntu 后再编译运行（NUC 上需装 ROS 2）。

## 平台分工

| 端 | 系统 | 仓库目录 |
|----|------|----------|
| 车载计算 | **Ubuntu**（推荐 22.04） | `nuc_ros`（本目录） |
| 远端操控 | **Windows** | `../pc_host` |

```
相机 → NUC Ubuntu (本仓库) → FRP → Windows PC (pc_host)
H7 ⇄ USB ⇄ NUC
ELRS → H7（急停不经过本仓库）
```

| 包名 | 作用 |
|------|------|
| `wheel_legged_bridge` | USB↔H7：按 `doc/NUC_H7_USB通信协议.md` 组帧下发 |
| `wheel_legged_camera` | 相机 + MJPEG 直播（给 PC） |
| `wheel_legged_teleop` | TCP 状态/指令网关（对接 pc_host） |
| `wheel_legged_bringup` | launch / 参数汇总 |

## 一键脚本（Ubuntu NUC）

拷到 NUC 后先给执行权限：

```bash
cd ~/ws/nuc_ros   # 按你的实际路径
chmod +x run.sh scripts/*.sh
```

| 命令 | 作用 |
|------|------|
| `./run.sh build` | 一键编译全部节点 |
| `./run.sh start` | 一键后台启动（**ROS + frpc**） |
| `./run.sh stop` | 一键关闭（ROS + frpc） |
| `./run.sh restart` | 重启 |
| `./run.sh status` | ROS / frpc 是否在跑 |
| `./run.sh log` | 跟踪 ROS 日志 |
| `./run.sh frpc-log` | 跟踪 frpc 日志 |

`start` 默认会启动 frpc：

- 可执行文件：`/home/jianghaoran/Workspace/frp/frpc`（若是目录则用其中的 `frpc`）
- 配置文件：本仓库 `frp/frpc.toml`（请先改好 `serverAddr`）

```bash
DRY_RUN=false ./run.sh start
# 换 frpc 路径 / 配置：
FRPC_BIN=/home/jianghaoran/Workspace/frp/frpc FRPC_CONFIG=$PWD/frp/frpc.toml ./run.sh start
# 只要 ROS、不要 frpc：
SKIP_FRP=1 ./run.sh start
```

## 环境要求（NUC · Ubuntu）

- Ubuntu 22.04 + **ROS 2 Humble**（若已装其他发行版，保持一致即可）
- `python3-serial`、`python3-opencv`（或 pip：`pyserial`、`opencv-python`）
- 可选：RealSense SDK / `realsense2_camera`

### RealSense D435i

当前 `mjpeg_server` 用 OpenCV 打开 V4L2 设备。本机 D435i 映射一般为：

| 设备 | 用途 |
|------|------|
| `/dev/video0` | 深度 Z16 |
| `/dev/video2` | 红外 GREY |
| **`/dev/video4`** | **彩色 YUYV（默认推流）** |

默认 `CAMERA_INDEX=4`。若插拔顺序变了，用 `v4l2-ctl -d /dev/videoN --list-formats-ext` 找 `YUYV` 再覆盖：

```bash
CAMERA_INDEX=4 ./run.sh start
```

## 拷到 NUC 后编译

在 **Ubuntu NUC** 终端：

```bash
# 例如拷到 ~/ws/nuc_ros
cd ~/ws/nuc_ros
sudo apt update
# 按需：rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

串口权限（H7 USB 虚拟串口常见为 `/dev/ttyACM0`）：

```bash
sudo usermod -aG dialout $USER
# 重新登录后生效
```

## 运行（Ubuntu）

```bash
source ~/ws/nuc_ros/install/setup.bash
ros2 launch wheel_legged_bringup bringup.launch.py
```

实机写串口时，把 bridge 里 `dry_run` 改为 `false`（见 launch / 参数）。

默认端口（本机监听，再经 frpc 映射到**服务器**公网口）：

| NUC 本机（可改） | 服务器对外（固定） | 数据方向 | 用途 |
|------------------|--------------------|----------|------|
| 8081 | **6004** | NUC → PC | MJPEG 视频 |
| 8082 | **6005** | NUC → PC | 状态 TCP |
| 8083 | **6006** | PC → NUC | 指令 TCP |

## FRP（NUC 跑 frpc）

配置见 `frp/frpc.toml`：`remotePort` 已固定为 **6004 / 6005 / 6006**。  
PC 的 `pc_host/config/default.yaml` 应连服务器这三口，不必知道 NUC 本机端口。

```bash
./frpc -c frp/frpc.toml
```

Windows 的 `pc_host` 只填服务器 `host` + `6004/6005/6006`。

## 控制权（遥控器 SB，在 H7 仲裁）

| SB | 含义 |
|----|------|
| 上 (2) | 仅遥控器 |
| 中 (1) | 遥控 + PC（本仓库把 PC 指令经 USB 送给 H7） |
| 下 (0) | 仅 PC |

NUC 只负责转发；是否采纳由下位机决定。

对齐固件 `wheel_legged_biped_dm/User/APP/usb_receive_task.*`：

- 转义前缀 `0xFE`，结束 `0xF8`
- `app_id = 0x02`，CRC16
- 指令字段：`vel_x, vel_y, vel_w, leg_length, control_mode`

固件侧解包已有，底盘闭环接入仍待接线。

## 目录

```
nuc_ros/
├── README.md
├── frp/                 # Ubuntu 上用的 frpc 示例
└── src/
    ├── wheel_legged_bringup/
    ├── wheel_legged_bridge/
    ├── wheel_legged_camera/
    └── wheel_legged_teleop/
```
