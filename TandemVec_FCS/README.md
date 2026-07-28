# TandemVec_FCS

## 共轴双桨推力矢量飞行控制系统 (Rev 5.0)

### 项目概述

本项目是基于 STM32H743 的共轴双桨 VTVL (Vertical Take-Off and Vertical Landing) 推力矢量飞控固件。飞行器通过两个共轴对转电机提供主推力，利用发动机喷口处的两轴推力矢量控制 (TVC) 舵机实现姿态控制，Yaw 轴通过双电机差速实现偏航。系统支持手动增稳、高度保持、定点悬停和上位机制导四种飞行模式。

- **目标平台**: WeAct MiniSTM32H743VITx (STM32H743VIT6, Cortex-M7 @ 480MHz)
- **框架**: Arduino on STM32 (ststm32 platform)
- **构建系统**: PlatformIO
- **作者**: LShang
- **上传协议**: CMSIS-DAP

---

### 硬件平台

#### MCU
| 项目 | 规格 |
|------|------|
| 型号 | STM32H743VIT6 |
| 内核 | ARM Cortex-M7 @ 480MHz |
| FPU | VFPv5-D16 (单/双精度硬件浮点) |
| RAM | 512KB |
| Flash | 2MB |
| 固件占用 | RAM ~27%, Flash ~9% |

#### 传感器配置

| 传感器 | 型号 | 接口 | 频率 | 用途 |
|--------|------|------|------|------|
| 板载 IMU (六轴) | ICM42688 | SPI (PA4 CS, 10MHz Mode3) | 2kHz | 角速度 + 加速度 |
| 板载 IMU/GNSS (扩展) | DETA100 | UART4 (921600 baud) | — | 备选惯导源（上电检测后可切换） |
| 磁力计 | IST8310 | I2C1 (PB7/PB6, 400kHz) | — | 航向初始化参考 |
| 气压计 | DPS310 | I2C2 (PB11/PB10, 1MHz Fm+) | 约129Hz任务读取 / 128Hz压力输出 | 气压高度 |
| 光流/测距 | LQS48 | UART7 (921600 baud) | 500Hz | 水平速度 + 激光测距 |
| TVC 角度反馈 | 模拟角度传感器 | ADC1 (PB0/PB1, 16bit) | — | TVC 摆角闭环 |
| 燃料液位 | 浮球开关 | GPIO (PC9) | — | 液位检测 |

#### 执行机构

| 通道 | 引脚 | 定时器 | 功能 | PWM 规格 |
|------|------|--------|------|----------|
| SERVO1 | PA0 | TIM2_CH1 | TVC Roll 舵机 | 333Hz, 16bit |
| SERVO2 | PA1 | TIM2_CH2 | TVC Pitch 舵机 | 333Hz, 16bit |
| MOTOR1 | PA2 | TIM2_CH3 | 主电机1 (CW) | 333Hz, 16bit |
| MOTOR2 | PA3 | TIM2_CH4 | 主电机2 (CCW) | 333Hz, 16bit |
| SERVO7 | PC8 | TIM3_CH3 | 燃料阀门舵机 | 333Hz, 16bit |
| IGNITION | PE6 | GPIO | 点火 MOS 管 | 高电平触发 |

#### 串口分配

| 串口 | 引脚 | 波特率 | 用途 |
|------|------|--------|------|
| Serial1 (USART1) | PA10/PA9 | 420000 | ELRS 接收机 (CRSF 遥控协议) |
| Serial2 (USART2) | PD6/PD5 | 921600 | 发动机控制器 / 数据转发 |
| Serial3 (USART3) | PD9/PD8 | 1500000 | 黑匣子数据记录 |
| Serial4 (UART4) | PD0/PD1 | 921600 | DETA100/UBX 共用（上电检测锁定） |
| Serial5 (UART5) | PD2/PC12 | 921600 | 上位机轨迹规划指令 |
| Serial6 (USART6) | PC7/PC6 | 921600 | 地面站通信 (MAVLink / AnoCom 互斥) |
| Serial7 (UART7) | PE7/PE8 | 921600 | LQS48 光流传感器 |
| Serial8 (UART8) | PE0/PE1 | 921600 | USB Type-C 调试输出 |

> ⚠️ Serial4 同时承载 DETA100 协议和 UBX GNSS 协议，上电先检测 DETA100，检测到则切换并注册 `handleDeta100`，否则保持内置 UBX 路径。Serial6 同时支持 MAVLink 和 AnoCom，按互斥使用。

#### 指示灯

| LED | 引脚 | 含义 |
|-----|------|------|
| Yellow (黄) | PE4 | 传感器异常/警告 |
| Green (绿) | PE5 | 系统运行/任务心跳 |

---

### 构建与烧录

#### 环境要求

- PlatformIO CLI (`pio`)
- ARM GCC 工具链 (PlatformIO 自动管理)
- CMSIS-DAP 调试器 (或 JLink, 需在 `platformio.ini` 中切换)

#### 构建

```powershell
# 编译主固件 + 全量测试环境
pio run
```

#### 烧录

```powershell
# CMSIS-DAP 上传
pio run -t upload

# 或指定 JLink
# 修改 platformio.ini: upload_protocol = jlink
```

#### 串口监视

```powershell
pio device monitor
# 波特率: 921600 (Serial8 USB 调试输出)
```

> 💡 本机 PlatformIO 安装路径为 `C:\Users\12631\.platformio\penv\Scripts\pio.exe`，如 PATH 中找不到 `pio` 请使用完整路径。

---

### 软件架构

```
                          ┌─────────────────┐
                          │     main.cpp     │  setup() / loop()
                          │  系统入口&调度    │
                          └────────┬────────┘
                                   │
         ┌─────────────────────────┼─────────────────────────┐
         │                         │                         │
  ┌──────▼──────┐          ┌───────▼───────┐         ┌───────▼──────┐
  │ state_data  │          │ task_scheduler │         │  math_utils  │
  │  全局状态中枢 │          │  2kHz 协作调度  │         │  数学工具函数  │
  └──────┬──────┘          └───────────────┘         └──────────────┘
         │
    ┌────┴──────────────────────────────┐
    │         模块分层 (数据流)           │
    │                                   │
    │  ┌─────────────────────────────┐  │
    │  │  传感器采集层 (2kHz~50Hz)    │  │
    │  │  sensor_imu.cpp              │  │  ICM42688 + IST8310
    │  │  sensor_peripheral.cpp       │  │  DPS310 + LQS48 + TVC反馈
    │  └──────────────┬──────────────┘  │
    │                 │                  │
    │  ┌──────────────▼──────────────┐  │
    │  │  状态估计层 (200Hz)          │  │
    │  │  navigation_task.cpp         │  │  EKF15 + 垂直KF + 水平KF(后备)
    │  └──────────────┬──────────────┘  │
    │                 │                  │
    │  ┌──────────────▼──────────────┐  │
    │  │  飞行控制层 (200Hz)          │  │
    │  │  flight_control.cpp          │  │  PID + TVC + 混控
    │  └──────────────┬──────────────┘  │
    │                 │                  │
    │  ┌──────────────▼──────────────┐  │
    │  │  通信遥测层 (5Hz~200Hz)      │  │
    │  │  communication.cpp           │  │  CRSF + MAVLink/AnoCom
    │  └─────────────────────────────┘  │
    └───────────────────────────────────┘
```

#### 模块职责

| 文件 | 职责 |
|------|------|
| `src/main.cpp` | 系统初始化、`setup()`、`loop()`、主循环调度、DETA100 探测/接收任务（条件注册） |
| `src/state_data.h/cpp` | 全局变量声明/定义、枚举、结构体、跨模块共享状态 |
| `src/math_utils.h` | 纯数学工具函数，`inline` header-only |
| `src/task_scheduler.h/cpp` | 2kHz 定时器驱动的协作式任务调度器 |
| `src/sensor_imu.h/cpp` | ICM42688 IMU、磁力计初始化与数据采集 |
| `src/sensor_peripheral.h/cpp` | DPS310 气压计、LQS48 光流、角度传感器 |
| `src/navigation_task.h/cpp` | EKF 组合导航、垂直/水平 KF、GNSS 处理 |
| `src/flight_control.h/cpp` | 控制律、PID、TVC、混控输出 |
| `src/communication.h/cpp` | CRSF 遥控、MAVLink 遥测、AnoCom 地面站 |

#### 任务调度

基于 TIM8 硬件定时器的 2kHz 协作式调度系统：

```
TIM8 中断 (2kHz)
  └─ timerCallback()
       └─ 遍历任务列表，置位到期任务的 flag

loop()
  └─ taskExecutor()
       └─ 遍历任务列表，执行 flag 为 true 的任务函数
```

> 关键设计：中断只置标志位，实际任务在主循环上下文执行，可安全使用串口/I2C/SPI 等阻塞外设。

##### 任务注册表

| 任务函数 | 频率 | 间隔 | 层级 |
|----------|------|------|------|
| `handleICM42688` | 2kHz | 0.5ms | 传感器采集 |
| `handleLQS48Flow` | 500Hz | 2ms | 传感器采集 |
| `handleDPS310` | 约129Hz | 7.735ms | 传感器采集 |
| `handleDeta100` | 200Hz | 5ms | DETA100 数据源，仅检测到模块时注册 |
| `handleNavigationSystem` | 200Hz | 5ms | 状态估计 |
| `handleAnoCom` | 200Hz | 5ms | 通信遥测 |
| `handleVerticalEstimation` | 200Hz | 5ms | 状态估计 |
| `handleGuidanceCommands` | 500Hz | 2ms | 控制执行 |
| `runGNCExecutive` | 200Hz | 5ms | 控制执行 |
| `sendElrsBatteryData` | 25Hz | 40ms | 通信遥测 |
| `sendPositionVelocityData` | 50Hz | 20ms | 通信遥测 |
| `handleStatusLedTask` | 100Hz | 10ms | 系统指示 |

> 同频任务按注册顺序 FIFO 串行执行。当前 200Hz 帧内顺序：EKF导航 → AnoCom遥测 → 垂直KF → GNC控制；水平KF与 MAVLink 为可选任务，默认未注册。

##### DPS310 实测配置

DPS310 采用 I2C2 1MHz Fm+、压力 128Hz、温度 1Hz、压力 2x 过采样、温度 4x 过采样。DPS310 数据手册支持 I2C HS-mode 3.4MHz，但当前 STM32duino `Wire` 仅实现 100kHz / 400kHz / 1MHz timing，正式固件不使用 3.4MHz。

2026-06-23 上板 profile 结论：固定单样本读取在 7.69ms~7.70ms 已接近 FIFO 溢出边界；改为压力优先补读后，7.736ms、7.737ms、7.739ms 在 300s 长测中均出现 `overflow=1/flush=1`，7.7405ms 与 7.742ms 在 180s 内也出现 overflow/flush。7.735ms 连续两次 300s 长测均满足 `overflow=0`、`flush=0`、`unfinished=0`、`staleMax=0ms`，DPS310 读取 `avg≈102us/max≈201us`，CPU 平均约 `36.0%`。因此默认采用 7.735ms、小数 tick 调度、`BFS_DPS310_FIFO_MAX_READS=2U` 和 `BFS_DPS310_STOP_AFTER_PRESSURE_SAMPLE`：常规只读一个压力样本，遇到温度样本占位时才补读一次以追回最新压力。

复现实测时使用诊断构建，不要把 profile 宏留在正式固件中。典型命令：

```powershell
node tools/dps_profile_runner.js --label 7p735_stop --interval-ms 7.735 --fifo-reads 2 --stop-after-pressure --duration-ms 300000 --warmup-ms 8000 --port COM10 --baud 921600
```

该工具会临时生成 `.pio/dps_profile_*.ini`、上传 `dps_profile` 环境、抓取串口输出并解析 `[TASK]` / `[LOOP]` / `[DPS310]` 窗口统计。合格判据以长测无 `overflow`、无 `flush`、无压力样本陈旧为主；若更改压力速率、温度速率、OSR、I2C 时钟或调度周期，必须重新跑 180s 以上筛选和 300s 复核。

---

### 导航与状态估计

#### 坐标系约定

| 坐标系 | 轴定义 | 用途 |
|--------|--------|------|
| 机体 FRD | X=前, Y=右, Z=下 | IMU 输出、光流速度、TVC 指令 |
| 导航 NED | N=北, E=东, D=地 | EKF 状态、GNSS 量测、位置控制 |
| 传感器 RUB | X=右, Y=上, Z=后 | ICM42688 原始安装方向 |

> ICM42688 物理安装为 RUB 系，`readIMUData()` 中通过坐标变换转为 FRD 系。Madgwick AHRS 使用 FLU (前左上) 系。

#### 导航系统组成

##### 1. 15 状态 EKF (主导航滤波器, 200Hz)

| 状态 | 维度 | 说明 |
|------|------|------|
| 姿态 | 4 (四元数) | FRD → NED 旋转 |
| NED 速度 | 3 | 北/东/地速度 (m/s) |
| LLA 位置 | 3 | 纬度(rad)/经度(rad)/大地高(m) |
| 加速度计零偏 | 3 | m/s², Markov 模型 |
| 陀螺零偏 | 3 | rad/s, Markov 模型 |

**时间更新 (预测):**
- IMU 2kHz → 每导航帧 (5ms) 积攒约 10 个 delta 样本
- 双子样拆分：按时间中点切分 Δθ₁₂、Δv₁₂
- 调用 `nav_ekf.TimeUpdateTwoSample()` 执行：
  - 圆锥补偿 (coning compensation)
  - 划摇补偿 (sculling compensation)
  - 四元数姿态推进 (中点积分)
  - 速度/位置更新 (重力 + 地球自转 + 科氏力补偿)
  - 15×15 协方差矩阵预测
  - GNSS 延迟回放历史快照记录

**量测更新 (修正):**

| 量测类型 | 触发条件 | 频率 | 说明 |
|----------|----------|------|------|
| GNSS 全量融合 | fix≥3D, sv≥9, hacc<6m, vacc<12m, sacc<1m/s, pdop<3, iTOW 不重复 | 5~10Hz | 6 维位置+速度量测，含 NIS 门控 |
| GNSS 降级融合 | fix≥3D, 位置精度不达标但速度精度合格 | 5~10Hz | 仅融合速度，位置噪声设 1e4m 关闭 |
| 首次 GNSS 重锚定 | 假原点启动后首个合格 GNSS | 一次性 | 重锚绝对位置/速度，并反算真实 WGS84 起飞点以保持已有相对位移 |
| 气压相对高度 | EKF 已初始化且起飞点原点有效 | 25Hz | `origin_alt_m + baro_altitude` 转为绝对高度后观测 D 位置 |
| 重力方向辅助 | 静止确认 | 5Hz | 约束 Roll/Pitch |
| ZUPT 零速修正 | 静止确认 | 5Hz | 约束 NED 速度为 0 |
| AHRS 姿态辅助 | 无 GNSS + 运动中 + 加速度可信 | 200Hz | Madgwick roll/pitch 约束姿态漂移 |
| 双矢量航向融合 | 全量 GNSS 帧 + 光流有效 | 5~10Hz | GNSS 地速 + 光流机速 → 几何解算航向 |
| 后台 AHRS 航向校正 | GNSS 有效时 | 200Hz | 慢速牵引 Madgwick 航向至 EKF 航向 |

**GNSS 延迟回放:**
- EKF 内部维护 64 格环形状态缓冲
- GNSS 观测到达时回溯至对应历史时刻执行更新，然后重传播至当前时刻
- 预设 GNSS 延迟 15ms (适配 921600 baud UBX)
- 纯惯导模式可设 `BFS_NAVIGATION_EMBEDDED_DISABLE_GNSS_DELAY_REPLAY=1` 关闭

##### 2. 垂直 2 态卡尔曼滤波器（当前停用）

| 状态 | 说明 |
|------|------|
| 高度 | m (向上为正) |
| 垂直速度 | m/s (向上为正) |

- 预测: IMU 天向加速度 (去重力)
- 更新: 激光测距高度 (优先, 噪声 0.05m) / 气压高度 (噪声 0.35m)
- 当前任务未注册；控制用 `estimated_height/velocity` 由主 EKF 的起飞点相对高度和 NED 速度统一更新

##### 3. 水平卡尔曼滤波器 (双轴各 2 态, 200Hz)

| 轴 | 状态 |
|----|------|
| 北向 | 位置 + 速度 |
| 东向 | 位置 + 速度 |

- 预测: NED 水平加速度
- 更新: 光流纯平移速度 (旋转到 NED 系)
- 输出: `fused_north_pos/vel`, `fused_east_pos/vel`
- 无 GNSS 时注入 `relative_*` 和 `INS_GNSS_Packet.velocity_*`

##### 4. 静止检测

| 判据 | 阈值 |
|------|------|
| 加速度模长 | 9.5 ~ 10.1 m/s² |
| 角速度模长 | < 0.01 rad/s (~0.57°/s) |
| 相邻帧加速度变化 | < 0.5 m/s² |
| 相邻帧角速度变化 | < 0.05 rad/s |
| 连续满足帧数 | ≥ 50 帧 (~250ms @200Hz) |

##### 5. GNSS 质量门限

| 参数 | 阈值 | 说明 |
|------|------|------|
| fixType | ≥ 3D Fix | 至少 3D 定位 |
| numSV | ≥ 9 | 可见卫星数 |
| hAcc | < 6m | 水平定位精度 |
| vAcc | < 12m | 垂直定位精度 |
| sAcc | < 1m/s | 速度精度 |
| pDOP | < 3 (或 ≤0=未提供) | 位置精度因子 |
| iTOW | 不重复 | 去重检查 |
| PVT/EOE iTOW | 一致 | 混帧检查 |

#### 导航输出桥接

| 全局变量 | 来源 | 消费者 |
|----------|------|--------|
| `AHRS_Packet` (唯一写入: navigation) | EKF 四元数；初始化前使用后台 Madgwick | GNC 姿态环、光流补偿、遥测 |
| `INS_GNSS_Packet.velocity_*` | `nav_ekf.ned_vel_mps()` | 位置环、遥测 |
| `Geodetic_Pos_Packet` | `nav_ekf.lla_rad_m()` + GNSS 原始精度 | 地面站、原点计算 |
| `relative_north/east/down`、`INS_GNSS_Packet.location_*` | `bfs::lla2ned(nav_ekf.lla, takeoff_origin)` | 位置环、遥测 |
| `estimated_height` | `-relative_down`，起飞点向上为正 | 高度环、光流、遥测 |
| `estimated_velocity` | `-INS_GNSS_Packet.velocity_down`，向上为正 | 高度环、遥测 |

---

### 飞行控制

#### 控制模式

| 模式 | 通道7 值 | 油门 | 姿态 | 高度 | 水平位置 |
|------|----------|------|------|------|----------|
| MANUAL | < 1300 | 手动 | 自稳 | 手动 | 无 |
| AUTO_ALTITUDE | 1300~1650 | 手动 | 自稳 | 自动 | 无 |
| AUTO_POSITION | 1650~1750 | 手动 | 自稳 | 自动 | 位置闭环 |
| GUIDED | > 1750 | 手动 | 自稳 | 自动 | 上位机指令 |

#### 遥控器通道映射

| 通道 | 功能 | 范围 |
|------|------|------|
| CH1 | Roll | 988 (左) ~ 2012 (右), 中位 1500 |
| CH2 | Pitch | 988 (俯) ~ 2012 (仰), 中位 1500 |
| CH3 | Throttle | 988 (低) ~ 2012 (高) |
| CH4 | Yaw | 988 (逆时针) ~ 2012 (顺时针), 中位 1500 |
| CH5 | Arm/Disarm | > 1500 解锁 (含7点中值去抖) |
| CH6 | Ignition | > 1500 点火使能 (含5点中值去抖) |
| CH7 | Flight Mode | 三档: <1300 Manual / 1300-1650 AltHold / >1650 PosHold |
| CH8 | TVC Manual | < 1200 手动TVC (摇杆直驱舵机, 旁路控制器) |
| CH9 | Attitude Mode | < 1500 角度模式 / ≥ 1500 角速率模式 |

#### PID 控制器结构

```
姿态控制 (Roll/Pitch 串级):
  RC输入 → 角度外环 → 角速率内环 → TVC摆角 → 舵机
            (Kp=4.25)   (Kp=0.25, Ki=0.0004)

Yaw控制 (单环):
  RC输入 → 角速率环 → 电机差速 PWM
            (Kp=3.8, Ki=0.15)

高度控制 (串级):
  目标高度 → 高度外环 → 速度内环 → 油门%
              (Kp=1.0)    (Kp=5.0, Ki=0.00625)

水平位置控制 (Loiter, 串级):
  目标位置 → 位置外环 → 速度内环 → 目标倾角 → 姿态环
              (Kp=0.25)    (Kp=1.75, Ki=0.00125, Kd=0.05)
```

#### 安全限制

| 参数 | 值 | 说明 |
|------|-----|------|
| MAX_TARGET_RATE | 80°/s | 姿态外环最大输出 |
| MAX_CORRECTION | 10° | TVC 最大摆角 |
| MAX_ANGLE_COMMAND | 30° | 最大倾角指令 |
| MAX_TILT (位置环) | 15° | 位置环最大倾斜 |
| MAX_SPEED (位置环) | 1.5 m/s | 最大水平速度 |
| MAX_ACCEL (速度环) | 2.6 m/s² | 最大水平加速度 |
| MAX_POSITION_ERROR | 5m | 位置误差限幅 |
| LOITER_DEADZONE | 25 (raw) | Loiter 摇杆死区 |
| GUIDANCE_TIMEOUT | 500ms | GUIDED 模式超时 → MANUAL |

#### 解锁序列

```
解锁上升沿触发 (一次性):
  1. 建立起飞点原点，target_altitude = 0
  2. targetNorth/East = 0 (当前位置设为原点)
  3. 重置全部 PID 积分/微分缓存
  4. 初始化控制流滤波器
  5. 重置水平 KF (kf_north/east)
  6. 从当前有效导航源设置 WGS84 起飞点原点
  7. 气压计相对高度、滤波状态和位置输出同步归零
  8. 点火控制 = (解锁 ∧ 点火开关) ? HIGH : LOW
  9. 燃料阀门 = TVC手动模式 ? 全开 : 关闭
```

---

### 通信协议

#### CRSF (ELRS 遥控接收)

- 串口: Serial1, 420000 baud
- 协议: CRSF RC Channels Packed (26字节帧, 16通道)
- 链路管理: `linkUpCallback` / `linkDownCallback`
- 失控保护: CH1/2/4 → 1500 (中位), 其他 → 988

#### MAVLink (地面站遥测)

- 串口: Serial6, 921600 baud (与 AnoCom 互斥)
- 发送频率: 200Hz
- 消息类型: HEARTBEAT, ATTITUDE, ATTITUDE_QUATERNION, RAW_IMU, GLOBAL_POSITION_INT, GPS_RAW_INT, BATTERY_STATUS, SYS_STATUS
- 兼容: Mission Planner, QGroundControl

#### AnoCom (匿名地面站协议)

- 串口: Serial6, 921600 baud (与 MAVLink 互斥, 默认注释)
- 4组轮发: IMU+姿态 / 高度+控制 / 速度+PWM / 位置+GPS

#### 上位机制导协议

- 串口: Serial5, 921600 baud
- 帧格式: `0xAA` + 12字节 Payload (3×float32: N/E/U加速度) + `0x55`
- 安全检查: |accel_xy| < 10m/s², |accel_z| < 10m/s²
- 超时: 500ms 无指令 → MANUAL 模式

#### 发动机数据协议

- 串口: Serial2, 921600 baud
- 帧格式: `0xA5` + Length(1B) + 12B Payload (P1/P2/ValveControl × float32 LE) + Checksum

---

### 配置与编译选项

#### platformio.ini 关键编译宏

```ini
build_flags =
    -mcpu=cortex-m7
    -mfpu=fpv5-d16
    -mfloat-abi=hard
    -O3 -DNDEBUG
    -DSERIAL_RX_BUFFER_SIZE=4096
    -DSERIAL_TX_BUFFER_SIZE=1024
```

#### 导航算法可配置宏 (在 `lib/navigation-main/src/ekf_15_state.h` 中)

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `BFS_NAVIGATION_EMBEDDED_DISABLE_GNSS_DELAY_REPLAY` | 0 | GNSS 延迟回放开关 |
| `BFS_NAVIGATION_GNSS_DELAY_S` | 0.015f | GNSS 固定回溯延迟 (秒) |
| `BFS_NAVIGATION_EMBEDDED_FAST_PROPAGATION` | 0 | 结构化 float 协方差传播快路径 |
| `BFS_NAVIGATION_EMBEDDED_FAST_FIRST_ORDER_PHI` | 0 | 一阶状态转移矩阵 (0=完整Φ) |
| `BFS_NAVIGATION_EMBEDDED_FAST_FIRST_ORDER_COVARIANCE` | 0 | 一阶协方差预测 (0=全精度) |
| `BFS_NAVIGATION_EMBEDDED_FAST_STATIC_AID_UPDATE` | 0 | 快速静止辅助 (0=完整数学) |
| `BFS_NAVIGATION_EMBEDDED_STRUCTURED_SIMPSON_QC` | 1 | 结构化的 Simpson Qc 展开 |

> 💡 本项目固件在 `platformio.ini` 中默认启用 `FAST_STABILIZE` 和 `FAST_PROPAGATION`；`FAST_STATIC_AID_UPDATE` 仅在宿主机 A/B 或诊断构建中显式开启。H743 上板实测 `handleNavigationSystem` 平均耗时约从 5.53ms 降至 0.58ms，可恢复 200Hz 导航周期裕度；更激进的一阶 Phi/协方差组合仅用于诊断对比，未作为默认飞控配置。

##### Navigation 快路径留痕

2026-06-23 的 Navigation 优化没有替换 EKF，也没有关闭双子样、圆锥/划摇、WGS84 正常重力、地球自转/导航系转动补偿等名义状态机械编排。耗时下降主要来自三处数值路径变化：

- `FAST_PROPAGATION` 利用 15 状态噪声输入矩阵的固定稀疏结构，直接展开过程噪声块，避免每帧执行通用 `Gs * Rw * Gs^T` 和大量 `double` 中间矩阵。
- `FAST_STATIC_AID_UPDATE` 利用 ZUPT 只观测速度块、Gravity 只观测姿态块的事实，直接用 `P[:,v]` / `P[:,att]` 块公式计算 `S`、`K` 和协方差更新，避免构造完整 `H` 与通用 Joseph 形式 15x15 中间矩阵。
- `FAST_STABILIZE` 和运行时 `propagation_stabilize_divider(8)` / `static_aid_stabilize_divider(8)` 将完整 LDLT/特征值稳定化降到约 25Hz；每步仍保留对称化、有限值、对角下限和快速 PSD 判据。

已做的主机 A/B 复核：使用同一份 `test/ekf_host_regression.cpp` 分别编译全精度路径和正式 FAST 路径，两者均通过 60 个场景。关键指标没有显示精度退化：Monte Carlo 最大位置误差 `0.099477m -> 0.095582m`，旋转比力捷联传播位置误差两者均为 `0.008957m`，双子样 GNSS 延迟回放后位置误差约 `0.004295m`。可复核命令如下：

```powershell
$unitSources = Get-ChildItem lib\units\src -Filter *.cpp | ForEach-Object { $_.FullName }
g++ -std=c++17 -O2 -Wl,--stack,16777216 -Iinclude -Isrc -Ilib/navigation-main/src -Ilib/eigen/src -Ilib/units/src test/ekf_host_regression.cpp @unitSources -o $env:TEMP\ekf_host_regression_full.exe
& $env:TEMP\ekf_host_regression_full.exe
g++ -std=c++17 -O2 -Wl,--stack,16777216 -Iinclude -Isrc -Ilib/navigation-main/src -Ilib/eigen/src -Ilib/units/src -DBFS_NAVIGATION_EMBEDDED_FAST_STATIC_AID_UPDATE=1 -DBFS_NAVIGATION_EMBEDDED_FAST_STABILIZE=1 -DBFS_NAVIGATION_EMBEDDED_FAST_PROPAGATION=1 test/ekf_host_regression.cpp @unitSources -o $env:TEMP\ekf_host_regression_fast.exe
& $env:TEMP\ekf_host_regression_fast.exe
```

风险边界：FAST 路径主要改变协方差、Kalman 增益、NIS 门控和 PSD 检查时机，不直接降低名义惯导机械编排精度。若后续增加高动态机动、改变 IMU 噪声模型、放宽量测门控或打开一阶 `Phi` / 一阶协方差宏，必须重新做主机 A/B 回归和上板 profile；保守回退方法是在 `platformio.ini` 中关闭 `FAST_PROPAGATION` 和 `FAST_STABILIZE`，若诊断构建曾显式打开 `FAST_STATIC_AID_UPDATE` 也应一并关闭。

#### GNSS 质量门限 (在 `state_data.cpp` 中定义)

GNSS 融合以 `PopEpoch()` 成功作为唯一消费凭证，不再依赖一次性 ready 标志。每个
`UbxEpoch` 在 NAV-EOE 完整解析时记录 MCU `micros()`，导航层用最新已接收 epoch 的
iTOW/本地时刻作为参考，把队列中的旧 epoch 映射回同一 MCU 时间轴，再将实际观测年龄
传给 EKF 历史回放。该映射能计入 UART backlog、epoch 队列积压和调度等待；在没有
PPS/TIMEPULSE 的当前硬件路径下，`BFS_NAVIGATION_GNSS_DELAY_S` 仍表示接收机内部输出
与串口传输的基础延迟估计，不能视为 PPS 级绝对时间同步。

```cpp
const float GNSS_MAX_HORZ_ACC_M = 6.0f;
const float GNSS_MAX_VERT_ACC_M = 12.0f;
const float GNSS_MAX_SPD_ACC_MPS = 1.0f;
const float GNSS_MAX_PDOP = 3.0f;
const int8_t GNSS_MIN_SV = 9;
const float GNSS_DOWNGRADE_MAX_SPD_ACC_MPS = 0.5f;
```

---

### 数据流核心路径

```
┌─────────────────────────────────────────────────────────────────────┐
│                         loop() @ ~1kHz+                             │
│                                                                     │
│  crsf.loop()  ────► raw_rc_values[16]                              │
│  ubx.Pump()   ────► UBX epoch FIFO 队列 (NAV-PVT + NAV-EOE)       │
│                       ↓                                             │
│  taskExecutor() ──► 按注册顺序 FIFO 执行到期任务                     │
└─────────────────────────────────────────────────────────────────────┘

200Hz 导航帧内部数据流:

  IMU Δθ/Δv 缓冲区 (2kHz 写入, noInterrupts 保护)
       │
       ▼ (noInterrupts 原子复制 + 清零)
  双子样拆分: Δθ₁₂, Δv₁₂
       │
       ▼ nav_update_dt_s (= 实际累计 IMU 时间)
  ╔══════════════════════════════════════════╗
  ║            EKF 时间更新                  ║
  ║  TimeUpdateTwoSample(Δθ₁,Δθ₂,Δv₁,Δv₂)   ║
  ║  · 圆锥/划摇补偿                         ║
  ║  · 四元数推进 + 速度/位置积分             ║
  ║  · 15×15 协方差预测                       ║
  ║  · GNSS 延迟回放快照                      ║
  ╚══════════════════════════════════════════╝
       │
       ├─► 静止? ─► 重力辅助 (5Hz) + ZUPT (5Hz)
       ├─► 无GNSS运动? ─► AHRS姿态辅助
       │
       ▼ PopEpoch()
  ╔══════════════════════════════════════════╗
  ║            GNSS 融合                     ║
  ║  ┌─ 质量门限 (fix/sv/hacc/vacc/sacc/pdop)│
  ║  ├─ iTOW 去重 + PVT/EOE 混帧检查         │
  ║  ├─ 全量: 首次→ResetPositionVelocity    │
  ║  │        后续→MeasurementUpdateDetailed │
  ║  └─ 降级: 仅速度融合 (位置噪声=1e4m)     │
  ╚══════════════════════════════════════════╝
       │
       ▼
  ╔══════════════════════════════════════════╗
  ║            输出桥接                      ║
  ║  AHRS_Packet ← EKF 四元数/Madgwick回退  ║
  ║  INS_GNSS_Packet.velocity ← EKF NED速度 ║
  ║  Geodetic_Pos_Packet ← EKF LLA + GNSS精度║
  ║  relative_n/e/d ← lla2ned(EKF LLA, origin)║
  ╚══════════════════════════════════════════╝
       │
       ▼
  ╔══════════════════════════════════════════╗
  ║            GNC 控制执行                  ║
  ║  process_control_inputs()               ║
  ║  determine_control_mode()               ║
  ║  compute_thrust_control()               ║
  ║  handlePositionControl() (AUTO_POSITION)║
  ║  generate_attitude_target()             ║
  ║  → TVC 舵机 + 主电机 PWM 输出            ║
  ╚══════════════════════════════════════════╝
```

---

### 依赖库

| 库 | 位置 | 说明 |
|----|------|------|
| `navigation-main` | `lib/navigation-main/` | 15 状态 EKF + 坐标变换 + 地球模型 (Bolder Flight Systems, 本地重构版) |
| `ublox-main` | `lib/ublox-main/` | u-blox UBX 协议解析 (本地重构版, 含 epoch 队列) |
| `eigen` | `lib/eigen/` | Eigen3 线性代数库 (header-only) |
| `units` | `lib/units/` | BFS 单位转换库 |
| `ICM42688` | `lib/ICM42688/` | ICM42688 六轴 IMU 驱动 |
| `DPS310-Pressure-Sensor` | `lib/DPS310-Pressure-Sensor/` | DPS310 气压计驱动 |
| `IST8310` | `lib/IST8310/` | IST8310 磁力计驱动 |
| `LQS48_Flow` | `lib/LQS48_Flow/` | LQS48 光流传感器驱动 |
| `MTF02P` | `lib/MTF02P/` | MTF02P 光流传感器驱动 (备选) |
| `MadgwickAHRS` | `lib/MadgwickAHRS-master/` | Madgwick AHRS 姿态解算 |
| `CrsfSerial` | `lib/CrsfSerial/` | CRSF 串行协议驱动 |
| `MAVLink` | `lib/MAVLink/` | MAVLink 地面站协议 |
| `AnoComProtocol` | `lib/AnoComProtocol/` | 匿名地面站协议 |
| `crc8` | `lib/crc8/` | CRC8 校验 |
| `arduino-CAN` | `lib/arduino-CAN/` | CAN 总线驱动 |

#### 需在 `main.cpp` 中单独引入的头文件

| 文件 | 说明 |
|------|------|
| `DETA100_module.h` | DETA100 IMU/GNSS 模块驱动，含解析实现和内部静态状态，必须只在 `main.cpp` 中包含 |

> 其余工具头当前都可以按需在对应 `.cpp` 中包含：`QuaternionMath.h`、`TVC_Control_3rdOrder_Poly.h`、`TVC_Control_Geometric.h`、`GeoDisplacement.h`、`MAVLink.h`。

---

### 安全关键规则

以下内容**未经用户明确要求不得修改**：

- 硬件引脚映射、串口分配、波特率、PWM 范围、舵机中心和限幅
- 任务频率、定时器配置、调度周期和控制环周期
- PID 参数、控制增益、混控比例、TVC 摆角限制
- 传感器坐标方向、FRD/NED 约定、安装方向补偿
- 解锁逻辑、故障保护阈值、超时处理、模式降级逻辑、输出限幅

#### 实机安全流程

1. **编译检查**: `pio run`（主环境）和必要时的 `pio test -e test`
2. **传感器与串口检查**: 确认 IMU、GNSS、光流数据正常
3. **断开动力电源**: 检查执行机构方向和限位
4. **系留低功率测试**: 约束条件下验证控制响应
5. **自由飞行测试**: 仅在以上步骤验证通过后进行

---

### 工作树管理

| 文件/目录 | 状态 |
|-----------|------|
| `.pio/` | 生成产物，不提交 |
| `compile_commands.json` | 生成产物，不提交 |
| `lib/` | 本地库 (vendor tree)，勿随意格式化 |
| `AGENTS.md` | 代理工作规范，修改需同步 |

---

### 版本历史

| 版本 | 日期 | 变更 |
|------|------|------|
| Rev 5.0 | — | 模块化架构重构 |
| — | 2026-06 | 导航系统升级: navigation-main + UBX epoch队列 + GNSS质量门限 + 降级融合 + 延迟回放 + 静止辅助诊断 |
