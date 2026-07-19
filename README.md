# wheel_legged_biped_dm

轮腿双足机器人**下位机固件**：达妙 CtrlBoard-H7（STM32H723VGTx）+ FreeRTOS。  
实现姿态估计、LQR 平衡、VMC 腿部力映射、起身/跑动/跳跃状态机，以及 ELRS 遥控与 USB 调试。

CubeMX 工程文件：`CtrlBoard-H7_IMU.ioc`  
Keil 工程：`MDK-ARM/`

---

## 功能概览

| 模块 | 说明 |
|------|------|
| 姿态 | BMI088（SPI2）+ Mahony，约 1 kHz |
| 平衡 | 腿长调度 LQR：6 状态 × 2 输入，`u = K(L)·x` |
| 腿部 | 五连杆 VMC：虚拟力 → 关节力矩（雅可比） |
| 辅助环 | 腿长 / 劈叉 / 转向 / 横滚 / 摆角 PID（增量式） |
| 执行 | 6×达妙电机，FDCAN1 @ 1 Mbps，MIT 仅前馈力矩 |
| 遥控 | ELRS → USART1 CRSF @ 420000 |
| 调试 | USB HS CDC：VOFA 遥测 + 上行指令解包 |

LQR 增益与 `phi_target(L)` 由上级目录 `../myLQR` 离线计算后固化进 `User/Algorithm/LQR/LQR.c`。

---

## 目录结构

```
wheel_legged_biped_dm/
├── CtrlBoard-H7_IMU.ioc     # CubeMX 配置（引脚/外设真源）
├── Core/                    # 启动、时钟、FreeRTOS 入口
├── Drivers/                 # HAL / CMSIS
├── Middlewares/             # FreeRTOS 等
├── USB_DEVICE/              # USB HS CDC
├── MDK-ARM/                 # Keil 工程与编译产物
└── User/                    # ★ 业务代码（主要改这里）
    ├── APP/                 # 任务：底盘 / INS / 遥控 / VOFA / USB
    ├── Algorithm/           # LQR / VMC / PID / Mahony / EKF(库存) / Kalman
    ├── Devices/             # DM 电机、BMI088
    ├── Bsp/                 # CAN、DWT、PWM 等板级
    ├── Controller/          # 通用控制器库（底盘当前未用）
    └── Lib/                 # 工具函数
```

---

## FreeRTOS 任务

| 任务 | 优先级倾向 | 周期 | 职责 |
|------|------------|------|------|
| `INS_TASK` | Realtime | ~1 ms | BMI088 + Mahony |
| `OBSERVE_TASK` | High | 3 ms | 速度 KF 框架（融合暂未启用） |
| `REMOTE_CONTROL_` | AboveNormal | 10 ms | CRSF 解析、摇杆/开关 |
| `CHASSIS_TASK` | AboveNormal | 约 ≥6 ms | 状态机 + LQR/VMC/PID + MIT 下发 |
| `VOFA_TASK` | Low | 2 ms | JustFloat 遥测 |
| `USB_RECEIVE_TAS` | Low | 1 ms | USB 指令解包 |
| `defaultTask` | Normal | 1 ms | USB 设备初始化后空转 |

底盘周期：每圈对 6 个电机各 `osDelay(1)`，实际约 160 Hz 量级；`dt` 由 FreeRTOS tick 差分。

---

## 遥控器键位（ELRS / CRSF）

协议：USART1 CRSF；摇杆归一化到 `[-1, 1]`（死区 0.005，一阶滤波）。  
三档开关原始值：`191→0`，`997→1`，`1792→2`（其它为无效档 3，SA 会保持上一有效档）。

### 通道映射

| 遥控器 | CRSF 通道 | 固件变量 | 作用 |
|--------|-----------|----------|------|
| 右杆左右 | ch0 | `right_x` | 当前底盘主环**未使用** |
| 左杆前后 | ch1 | `left_y` | 前进/后退线速度 |
| 右杆前后 | ch2 | `right_y` | 目标腿长（仅 RUN） |
| 左杆左右 | ch3 | `left_x` | 转向角速度；另写入 `leg_phi_*_target`（RUN 中 Phi 跟踪已注释，暂不生效） |
| 开关 SA | ch4 | `swich_SA` | 失能 / 起身 |
| 开关 SB | ch5 | `swich_SB` | **控制权：仅遥控 / 双源 / 仅 PC** |
| 开关 SC | ch6 | `swich_SC` | 已解析，**未使用** |
| 开关 SD | ch7 | `swich_SD` | 跑动 / 跳跃（SB 允许遥控时） |

### 摇杆（COMMON 正常态）

| 操作 | 计算 | 范围/说明 |
|------|------|-----------|
| 左杆前后 | `v_x = left_y × 1.8` | 最大约 ±1.8 m/s |
| 左杆左右 | `w = −left_x × 2.5` | 最大约 ±2.5 rad/s（左推为正转向约定见代码取负） |
| 右杆前后 | `leg_length = 0.15 + 0.06×right_y` | 再经横滚修正后限幅 **[0.10, 0.20] m** |

### 开关

| 开关 | 档位 | 行为 |
|------|------|------|
| **SA** | 0 | → `REST` 失能，力矩清零（任意 SB 档均有效） |
| **SA** | 2 | → `STANDUP` 起身（任意 SB 档均有效） |
| **SA** | 1 | 无专门逻辑 |
| **SB** | **2（拨上）** | **仅遥控器**控制速度/腿长/跳跃 |
| **SB** | **1（拨中）** | **遥控器 + PC** 均可控（速度叠加，腿长优先新鲜 PC 指令） |
| **SB** | **0（拨下）** | **仅 PC** 远程控制速度/腿长/跳跃（摇杆运动无效） |
| **SD** | 0 / 2 | 跑 / 跳（仅当 SB 允许遥控时生效） |
| **SC** | — | 预留，未接控制 |

> 遥控需保持在线（否则整车 FAULT）。即使「仅 PC」档，也要用遥控器拨 SB，并可用 SA 急停/起身。

### 安全

- 遥控帧超时 **>1000 ms** → `FAULT`，力矩清零  
- 上电默认 `INIT`（力矩 0），需 **SA=2 起身** 后才能正常跑  
- PC USB 指令超时约 **500 ms** 视为无效（不再叠加 PC 速度）

**推荐操作顺序**：SA→2 起身 → 用 **SB** 选控制源 → 遥控或 PC 操控 → 收工 SA→0 失能。

---

## 底盘状态机

**主状态** `chassis_status_t`：

- `INIT` — 上电默认，力矩清零（需遥控起身）
- `REST` — 失能，力矩 0
- `STANDUP` — 起身：`START → MID → END`，完成后进 `COMMON/RUN`
- `COMMON` — 正常跑跳：`RUN` / `JUMP`
- `FAULT` — 遥控离线等，力矩 0

**跳跃** `PREPARE → TAKEOFF → FLIGHT → LANDING`；跳中轮毂力矩强制 0。  
`JUMP_STATE_RECOVER` 已定义未使用。

---

## 控制数据流（COMMON / RUN）

```
电机反馈 → VMC 正运动学(φ, L)
         → 组装 LQR 状态 x[6]
遥控/目标 → 劈叉 Tp / Roll / 腿长 Leg / 转向 Turn
         → LQRCalculate(L) → u0 叠加轮矩，u1 叠加 vmc_force[0]
         → VMCVirtual2RealCalc → τ_关节
         → MIT 力矩下发 CAN
```

**LQR 状态（左右腿各一份，位移共享）**

| 索引 | 含义 |
|------|------|
| x0 | `phi − phiTargetCalc(L)` |
| x1 | 摆角速度 |
| x2 | 位移误差积分（±2 限幅） |
| x3 | `wheel.vel × 0.05 − v_x_target` |
| x4 | `INS.Pitch` |
| x5 | `INS.Gyro[1]` |

**VMC**：半杆长 `l1 = 0.07` m；`τ = Jᵀ Fvirt`，`Fvirt = [f_φ, f_L]`。

---

## 电机 CAN ID

| 发送 ID | 回传 ID | 对象 | 接口 |
|---------|---------|------|------|
| 0x01 | 0x11 | 左轮 | `mit_ctrl2`，软件限幅 ±2.5 N·m |
| 0x02 | 0x12 | 右轮 | 同上 |
| 0x03 | 0x13 | 左关节 0 | `mit_ctrl`，限幅 ±10 N·m |
| 0x04 | 0x14 | 左关节 1 | 同上 |
| 0x05 | 0x15 | 右关节 0 | 同上 |
| 0x06 | 0x16 | 右关节 1 | 同上 |

MIT：`pos=vel=kp=kd=0`，仅前馈 `torq`。右侧收发做符号翻转。当前 6 电机均在 **FDCAN1**（FDCAN2/3 已初始化，预留）。

---

## 关键外设（对照 ioc）

| 功能 | 外设 | 要点 |
|------|------|------|
| BMI088 | SPI2 | SCK=PB13；ACC_CS=PC0，GYRO_CS=PC3 |
| ELRS | USART1 | PA9/PA10，420000，DMA+IDLE |
| USB | OTG_HS CDC | PA11/PA12 |
| 电机 | FDCAN1 | PD1/PD0，1 Mbps |

---

## 编译与烧录

1. 用 Keil 打开 `MDK-ARM` 下工程（目标板 CtrlBoard-H7_IMU）
2. 若改引脚/外设：先改 `CtrlBoard-H7_IMU.ioc` 再生成代码，注意保留 `USER CODE` 区
3. 烧录：JLink / ST-Link（SWD：PA13/PA14）

依赖：CMSIS-DSP（LQR/VMC 矩阵运算）、HAL FDCAN / USB / SPI / UART。

---

## 实现边界（改代码前请知悉）

**已闭环**

- Mahony 姿态、CRSF 遥控、LQR+VMC+PID 跑/起身/跳、MIT 力矩、VOFA、USB 指令解包框架

**未闭环 / 预留**

- `OBSERVE_TASK` 速度融合（更新代码被注释）
- `QuaternionEKF` 未接入 INS
- `usb_receive_task` 的 `robot_cmd` **尚未接入** `chassis_task`（远程/NUC 指令需在此接线）
- `PhiVel*` PID 已初始化，主环未用

增益更新：在 `../myLQR` 重跑拟合后，将 `output.m` 打印的系数贴回 `User/Algorithm/LQR/LQR.c`（勿改符号约定）。

---

## 相关文档

- 整机说明：`../doc/技术文档.docx`
- 离线 LQR：`../myLQR/README.md`
- 仓库总览：`../README.md`
