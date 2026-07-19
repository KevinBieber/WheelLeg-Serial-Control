# pc_host

运行在 **Windows PC** 上的上位机：经 FRP 连接 NUC（Ubuntu），实时看图像（类直播）、监视整车/NUC 状态，并下发遥控指令。

> 本目录只在 Windows 上开发/运行，不依赖 ROS。

## 键盘键位

窗口需处于**前台焦点**时按键才生效。

| 按键 | 作用 | 下发字段 |
|------|------|----------|
| **W** | 前进 | `vel_x = +v_x_max`（默认 1.8） |
| **S** | 后退 | `vel_x = −v_x_max` |
| **A** | 左转 | `vel_w = +w_max`（默认 2.5） |
| **D** | 右转 | `vel_w = −w_max` |
| **↑** | 升高目标腿长 | `leg_length`，步进 +0.01 m，上限 0.20 |
| **↓** | 降低目标腿长 | `leg_length`，步进 −0.01 m，下限 0.10 |
| **E** | 起身 | `control_mode = 3`（同遥控 SA 起身） |
| **空格** | 失能 | `control_mode = 1`，速度清零（同遥控 SA 失能） |
| **J** | 请求跳跃 | `control_mode = 2` |
| **K** | 请求回跑动 | `control_mode = 0` |
| **R** | 重连视频流 | 仅本机 UI |

松键后对应速度轴回到 0；腿长保持最近一次 ↑/↓ 设定。  
`control_mode`：`0` 跑动 / `1` 失能 / `2` 跳跃 / `3` 起身。

### 与车上遥控器 SB 的关系（必须）

控制权在 **H7 下位机**按遥控器 **SB** 仲裁，PC 无法自行抢权：

| 遥控器 SB | 谁能控车 |
|-----------|----------|
| **拨上** | 仅实体遥控器 |
| **拨中** | 遥控器 + PC（速度叠加） |
| **拨下** | 仅 PC（本键盘生效） |

即使「仅 PC」档，车上遥控器仍须在线（否则 H7 判 FAULT）；**SA** 仍可失能/起身。

## 功能规划

| 模块 | 说明 |
|------|------|
| 视频 | 拉取 NUC MJPEG 流 |
| 状态 | 链路/指令监视 |
| 遥控 | 上表键位 → FRP → NUC → USB → H7 |
| 安全 | 心跳超时停发；车上 ELRS/SA 优先 |

## 与系统关系

```
Windows PC (本程序 pc_host)
        ⇄ FRP
Ubuntu NUC (nuc_ros)
        ⇄ USB
H7 下位机（SB 仲裁控制源）
```

配置见 `config/default.yaml`（远程时 `host`=服务器，端口 **6004/6005/6006**）。

## FRP 端口（走服务器）

数据方向：

```
视频:  NUC ──► 服务器:6004 ──► PC
状态:  NUC ──► 服务器:6005 ──► PC
指令:  PC  ──► 服务器:6006 ──► NUC
```

NUC 本机仍是 8081/8082/8083；**对外固定服务器 6004（视频）、6005（状态）、6006（指令）**。

## 一键启动（推荐）

双击或在 CMD 中运行：

```bat
cd /d D:\make_money\wheel_legged_biped\pc_host
start.bat
```

会：`conda activate pc_host` → `python -m app.main`。  
PC **不跑 frpc**（隧道在 NUC）；上位机按 `config/default.yaml` 直连服务器 `6004/6005/6006`。

## 环境（Windows）

```bat
cd pc_host
conda activate pc_host
pip install -r requirements.txt
python -m app.main
```

或使用 venv：

```bat
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
python -m app.main
```

## 目录

```
pc_host/
├── README.md
├── start.bat
├── requirements.txt
├── config/default.yaml
└── app/
    ├── main.py
    ├── ui_main.py
    ├── video_client.py
    ├── status_client.py
    └── command_client.py
```

## 联调顺序

1. 车上：SA 起身，SB 拨到需要的档（测 PC 时拨下或拨中）  
2. NUC：`nuc_ros` + 串口 `dry_run:=false`  
3. PC：改 `default.yaml` 的 host → `python -m app.main`
