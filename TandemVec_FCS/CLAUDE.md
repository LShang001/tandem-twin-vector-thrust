# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## ⚠️ 最高优先级：必须阅读 AGENTS.md

在本仓库中执行任何任务前，**必须首先阅读 [AGENTS.md](AGENTS.md)**，该文件包含执行总原则、诊断流程、安全关键修改规则、仓库卫生和实机安全注意事项。**语言规则（最高优先级）**：所有思考过程与内部分析必须使用中文；代码标识符、命令、路径、API 名、库名保留英文。

## 构建命令

在仓库根目录使用 PowerShell：

```powershell
# 编译固件
pio run

# 运行宿主机回归测试
pio test -e test

# 上传固件（仅用户明确要求时）
pio run -t upload

# 串口监视器
pio device monitor
```

若 `pio` 不在 PATH，检查 `C:\Users\12631\.platformio\penv\Scripts\pio.exe`，不要声称构建通过而未实际执行。

## 项目概述

基于 PlatformIO 的 STM32H743（480 MHz，Cortex-M7 硬件浮点）共轴双桨 VTVL 推力矢量飞控固件。这是安全关键的嵌入式控制软件，不是普通应用代码。

- **PlatformIO 环境**：`VTVL_ElectricDualRotor_FCS`
- **板卡**：`weact_mini_h743vitx`，上传协议：`cmsis-dap`
- **框架**：Arduino on STM32

## 模块化架构

状态共享通过 `state_data.h` 暴露的全局变量完成，模块间不直接互相包含实现文件。

| 文件 | 职责 |
|---|---|
| `src/main.cpp` | 系统初始化、`setup()`、`loop()`、主循环调度、DETA100 探测/接收任务 |
| `src/state_data.h/cpp` | 全局变量声明/定义、枚举、结构体、跨模块共享状态 |
| `src/math_utils.h` | 纯数学工具函数，`inline` header-only |
| `src/task_scheduler.h/cpp` | 2 kHz 定时器驱动的任务调度器 |
| `src/sensor_imu.h/cpp` | ICM42688 IMU、磁力计初始化与数据采集 |
| `src/sensor_peripheral.h/cpp` | DPS310 气压计、LQS48 光流、角度传感器 |
| `src/navigation_task.h/cpp` | EKF 组合导航、垂直/水平 KF、GNSS 处理 |
| `src/flight_control.h/cpp` | 控制律、PID、TVC、混控输出 |
| `src/communication.h/cpp` | CRSF 遥控、MAVLink 遥测、AnoCom 地面站 |
| `src/deta100_types.h` | DETA100 模块类型定义（可安全多文件包含） |

## 关键代码约束

- `DETA100_module.h` 含解析实现和内部静态状态，**只能在 `main.cpp` 中包含**，避免多编译单元重复定义。
- `QuaternionMath.h`、`TVC_Control_*`、`GeoDisplacement.h`、`MAVLink.h` 目前可在对应模块中按需包含；如后续改成含全局实现状态的头，再收紧包含边界。
- `MAVLink` 和 `AnoCom` 共用 `Serial6`，按互斥使用处理。
- `lib/MAVLink` 是 vendor tree，不是子模块，不要编辑。
- `VECTOR3_TYPE_GUARD` 防止 `Vector3` 与 `QuaternionMath.h` 重复定义。

## 硬件文档与引脚映射

电路权威参考：`docs/电路拓扑参考.md`（2026-06-30 EDA 实测修正）。修改引脚相关代码前必须对照。

**关键硬件事实：**
- DPS310 使用 **I2C2** (PB11/PB10, 1MHz Fm+)，非 SPI4。PE15 为 I2C 地址选择 (HIGH→0x77)
- ICM42688 原理图位号 **R12**（非电阻），GYRO_INT=PC4（固件轮询未用）
- 全部 8 路 UART、4 路 SPI、2 路 I2C、8 路 PWM 引脚已于 EDA 实测验证一致

**未使用硬件（详见 `电路拓扑参考.md` §12）：**

| 资源 | 引脚 | 接口 | 优先级 |
|------|------|------|--------|
| CAN (MCP2515+TJA1050) | PB12-15/PD10 | SPI2 | ⭐⭐⭐ |
| Flash W25N01GV (128MB) | PA15/PC10-11/PB2 | SPI3 | ⭐⭐⭐ |
| WS2812 RGB | PD15 | GPIO | ⭐⭐ |
| 备用 PWM S5/S6/S8 | PB0/PB1/PC9 | TIM | ⭐⭐ |
| 空闲 SPI4 | PE11-14 | SPI | ⭐ |
| 未连接 GPIO (10个) | PE2-3/PA8/PD11-14/PD4/PB8-9 | — | ⭐ |

> 立创EDA Pro Bridge 可用 (localhost:49620)，可实时查询原理图连接。
